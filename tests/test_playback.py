import threading
import unittest
from unittest.mock import MagicMock, patch

from kinova_movement_app.connection import _load_kortex_api
from kinova_movement_app.playback import (
    _transition_gradients,
    _transition_timeout_seconds,
    build_angular_transition,
    play_movement,
)


class RobotPlaybackTests(unittest.TestCase):
    def test_acceleration_profile_creates_synchronized_waypoints(self) -> None:
        base_messages = _load_kortex_api()[5]

        waypoints, duration = build_angular_transition(
            base_messages,
            [0.0] * 7,
            [90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [60.0] * 7,
            {
                "increase_dps2": 30.0,
                "decrease_dps2": 30.0,
                "joints": [1],
            },
        )

        self.assertGreater(duration, 1.5)
        self.assertGreater(len(waypoints.waypoints), 2)
        first_angle = waypoints.waypoints[0].angular_waypoint.angles[0]
        last_waypoint = waypoints.waypoints[-1].angular_waypoint
        self.assertGreater(first_angle, 0.0)
        self.assertLess(first_angle, 90.0)
        self.assertAlmostEqual(last_waypoint.angles[0], 90.0)
        self.assertTrue(
            all(
                point.angular_waypoint.angles[1] == 0.0
                for point in waypoints.waypoints
            )
        )
        self.assertTrue(
            all(
                abs(point.angular_waypoint.duration - duration / len(waypoints.waypoints))
                < 1e-6
                for point in waypoints.waypoints
            )
        )

    def test_once_playback_waits_for_each_measured_target(self) -> None:
        self._assert_position_tracking("once", 1, expected_targets=[10.0])

    def test_counted_loop_executes_the_loop_back_transition(self) -> None:
        self._assert_position_tracking(
            "play_count",
            2,
            expected_targets=[10.0, 0.0, 10.0],
        )

    def _assert_position_tracking(
        self, loop_mode: str, play_count: int, expected_targets: list[float]
    ) -> None:
        api = _load_kortex_api()
        base = MagicMock()
        measured_angles = [0.0] * 7

        def respond_to_speed_command(command: object) -> None:
            requested_speed = command.joint_speeds[0].value
            if requested_speed > 0:
                measured_angles[0] = 10.0
            elif requested_speed < 0:
                measured_angles[0] = 0.0

        base.SendJointSpeedsCommand.side_effect = respond_to_speed_command
        connection = MagicMock()
        connection.create_base_client.return_value = base
        connection.read_joint_angles.side_effect = lambda *_: list(measured_angles)
        movement = {
            "positions": [[0.0] * 7, [10.0] + [0.0] * 6],
            "transition_speeds_dps": [[20.0] * 7, [20.0] * 7],
            "transition_gradients": [None, None],
        }

        with patch(
            "kinova_movement_app.playback._load_kortex_api", return_value=api
        ):
            result = play_movement(
                connection,
                movement,
                loop_mode,
                play_count,
                1.0,
                threading.Event(),
            )

        self.assertTrue(result)
        self.assertEqual(base.Stop.call_count, len(expected_targets) + 1)
        self.assertGreater(
            base.SendJointSpeedsCommand.call_count, len(expected_targets)
        )
        self.assertEqual(measured_angles[0], expected_targets[-1])

    def test_braking_speed_decreases_as_joint_approaches_target(self) -> None:
        from kinova_movement_app.playback import move_to_joint_position

        base_messages = _load_kortex_api()[5]
        base = MagicMock()
        connection = MagicMock()
        connection.read_joint_angles.side_effect = [
            [0.0] * 7,
            [0.0] * 7,
            [0.0] * 7,
            [0.0] * 7,
            [7.0] + [0.0] * 6,
            [7.0] + [0.0] * 6,
            [7.0] + [0.0] * 6,
            [8.0] + [0.0] * 6,
            [9.0] + [0.0] * 6,
            [9.5] + [0.0] * 6,
            [9.5] + [0.0] * 6,
            [9.5] + [0.0] * 6,
            [9.5] + [0.0] * 6,
        ]
        sent_speeds: list[float] = []

        def record_command(command: object) -> None:
            sent_speeds.append(command.joint_speeds[0].value)

        base.SendJointSpeedsCommand.side_effect = record_command
        result = move_to_joint_position(
            connection,
            base,
            base_messages,
            [0.0] * 7,
            [10.0] + [0.0] * 6,
            [40.0] * 7,
            None,
            threading.Event(),
        )

        self.assertTrue(result)
        self.assertGreater(max(sent_speeds[:-2]), sent_speeds[-2])
        self.assertGreater(sent_speeds[-2], 0.0)
        self.assertEqual(sent_speeds[-1], 0.0)
        base.Stop.assert_called_once_with()

    def test_low_ramp_values_allow_enough_time_to_reach_the_target(self) -> None:
        start = [0.0] * 7
        destination = [10.0] + [0.0] * 6
        speeds = [20.0] * 7
        acceleration, deceleration = _transition_gradients(
            {
                "increase_dps2": 0.01,
                "decrease_dps2": 0.01,
                "joints": [1],
            }
        )

        timeout = _transition_timeout_seconds(
            start, destination, speeds, acceleration, deceleration
        )

        self.assertGreater(timeout, 60.0)


if __name__ == "__main__":
    unittest.main()

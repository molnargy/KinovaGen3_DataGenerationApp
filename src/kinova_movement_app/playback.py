"""Execute validated movement-file trajectories on a connected Kinova Gen3."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from typing import Any

from PySide6.QtCore import QThread, Signal
from kinova_movement_app.connection import RobotConnection, _load_kortex_api


_MIN_TRANSITION_DURATION_S = 0.1
_MAX_PROFILE_WAYPOINTS = 100
_CONTROL_PERIOD_S = 0.05
_POSITION_TOLERANCE_DEG = 0.5
_VELOCITY_TOLERANCE_DPS = 1.0
_SETTLED_SAMPLES_REQUIRED = 3
_DEFAULT_ACCELERATION_DPS2 = 60.0


def _minimum_profile_duration(
    distance: float, maximum_speed: float, acceleration: float, deceleration: float
) -> float:
    if distance == 0:
        return 0.0
    inverse_acceleration_sum = 1 / acceleration + 1 / deceleration
    distance_at_max_speed = (
        0.5 * maximum_speed**2 * inverse_acceleration_sum
    )
    if distance >= distance_at_max_speed:
        return (
            distance / maximum_speed
            + 0.5 * maximum_speed * inverse_acceleration_sum
        )
    peak_speed = math.sqrt(2 * distance / inverse_acceleration_sum)
    return peak_speed * inverse_acceleration_sum


def _transition_timeout_seconds(
    start: Sequence[float],
    destination: Sequence[float],
    speeds: Sequence[float],
    accelerations: Sequence[float],
    decelerations: Sequence[float],
) -> float:
    estimated_duration = max(
        (
            _minimum_profile_duration(
                abs(float(target) - float(initial)),
                speed,
                acceleration,
                deceleration,
            )
            for initial, target, speed, acceleration, deceleration in zip(
                start,
                destination,
                speeds,
                accelerations,
                decelerations,
                strict=True,
            )
        ),
        default=0.0,
    )
    return max(30.0, estimated_duration * 6.0 + 30.0)


def _profile_peak_speed(
    distance: float,
    duration: float,
    acceleration: float,
    deceleration: float,
    maximum_speed: float,
) -> float:
    if distance == 0:
        return 0.0
    inverse_acceleration_sum = 1 / acceleration + 1 / deceleration
    discriminant = max(
        0.0, duration**2 - 2 * inverse_acceleration_sum * distance
    )
    peak_speed = 2 * distance / (duration + math.sqrt(discriminant))
    return min(peak_speed, maximum_speed)


def _profile_position(
    distance: float,
    time_s: float,
    duration: float,
    acceleration: float,
    deceleration: float,
    peak_speed: float,
) -> float:
    acceleration_time = peak_speed / acceleration
    deceleration_start = duration - peak_speed / deceleration
    if time_s <= acceleration_time:
        return min(distance, 0.5 * acceleration * time_s**2)
    acceleration_distance = 0.5 * acceleration * acceleration_time**2
    if time_s >= deceleration_start:
        remaining_time = duration - time_s
        return max(
            0.0,
            distance - 0.5 * deceleration * remaining_time**2,
        )
    return min(
        distance,
        acceleration_distance + peak_speed * (time_s - acceleration_time),
    )


def build_transition_profile(
    start: Sequence[float],
    destination: Sequence[float],
    maximum_speeds: Sequence[float],
    gradient: Mapping[str, object],
) -> tuple[float, Callable[[float], list[float]]]:
    """Return a synchronized duration and normalized-time acceleration profile."""
    if not (
        len(start) == len(destination) == len(maximum_speeds) == 7
    ):
        raise ValueError("A robot transition must contain exactly seven joints.")
    distances = [
        abs(float(target) - float(initial))
        for initial, target in zip(start, destination, strict=True)
    ]
    speeds = [float(speed) for speed in maximum_speeds]
    if any(not math.isfinite(speed) or speed <= 0 for speed in speeds):
        raise ValueError("Each transition speed must be finite and greater than 0.")
    try:
        acceleration = float(gradient["increase_dps2"])
        deceleration = float(gradient["decrease_dps2"])
        selected_joints = gradient["joints"]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("The transition acceleration settings are invalid.") from error
    if (
        not math.isfinite(acceleration)
        or acceleration <= 0
        or not math.isfinite(deceleration)
        or deceleration <= 0
        or not isinstance(selected_joints, list)
        or not selected_joints
        or any(
            isinstance(joint, bool)
            or not isinstance(joint, int)
            or not 1 <= joint <= 7
            for joint in selected_joints
        )
        or len(set(selected_joints)) != len(selected_joints)
    ):
        raise ValueError("The transition acceleration settings are invalid.")

    selected = {joint - 1 for joint in selected_joints}
    duration = max(
        [
            _minimum_profile_duration(
                distances[index], speeds[index], acceleration, deceleration
            )
            for index in selected
        ]
        + [
            distance / speeds[index]
            for index, distance in enumerate(distances)
            if index not in selected
        ]
        + [_MIN_TRANSITION_DURATION_S]
    )
    peak_speeds = [
        _profile_peak_speed(
            distances[index],
            duration,
            acceleration,
            deceleration,
            speeds[index],
        )
        if index in selected
        else 0.0
        for index in range(7)
    ]

    def angles_at(progress: float) -> list[float]:
        elapsed = duration * min(1.0, max(0.0, progress))
        angles: list[float] = []
        for index, (initial, target) in enumerate(
            zip(start, destination, strict=True)
        ):
            if index in selected:
                travelled = _profile_position(
                    distances[index],
                    elapsed,
                    duration,
                    acceleration,
                    deceleration,
                    peak_speeds[index],
                )
                direction = 1.0 if target >= initial else -1.0
                angles.append(float(initial) + direction * travelled)
            else:
                angles.append(
                    float(initial)
                    + (float(target) - float(initial)) * elapsed / duration
                )
        return angles

    return duration, angles_at


def build_angular_transition(
    base_messages: Any,
    start: Sequence[float],
    destination: Sequence[float],
    maximum_speeds: Sequence[float],
    gradient: Mapping[str, object] | None,
) -> tuple[Any, float]:
    """Build a Kortex waypoint list and estimate its duration in seconds."""
    if not (
        len(start) == len(destination) == len(maximum_speeds) == 7
    ):
        raise ValueError("A robot transition must contain exactly seven joints.")

    distances = [
        abs(float(target) - float(initial))
        for initial, target in zip(start, destination, strict=True)
    ]
    speeds = [float(speed) for speed in maximum_speeds]
    if any(not math.isfinite(speed) or speed <= 0 for speed in speeds):
        raise ValueError("Each transition speed must be finite and greater than 0.")

    if gradient is None:
        waypoint_list = base_messages.WaypointList()
        waypoint_list.duration = 0.0
        waypoint_list.use_optimal_blending = False
        waypoint = waypoint_list.waypoints.add()
        waypoint.name = "movement_target"
        angular_waypoint = waypoint.angular_waypoint
        angular_waypoint.angles.extend(destination)
        angular_waypoint.maximum_velocities.extend(speeds)
        return waypoint_list, max(
            (
                distance / speed
                for distance, speed in zip(distances, speeds, strict=True)
            ),
            default=0.0,
        )

    duration, angles_at = build_transition_profile(
        start, destination, speeds, gradient
    )
    waypoint_count = min(
        _MAX_PROFILE_WAYPOINTS,
        max(2, math.ceil(duration / _MIN_TRANSITION_DURATION_S)),
    )
    step_duration = duration / waypoint_count

    waypoint_list = base_messages.WaypointList()
    waypoint_list.duration = 0.0
    waypoint_list.use_optimal_blending = False
    for step in range(1, waypoint_count + 1):
        progress = step / waypoint_count
        waypoint = waypoint_list.waypoints.add()
        waypoint.name = f"movement_profile_{step}"
        angular_waypoint = waypoint.angular_waypoint
        angular_waypoint.angles.extend(angles_at(progress))
        angular_waypoint.maximum_velocities.extend(speeds)
        angular_waypoint.duration = step_duration
    return waypoint_list, duration


def _transition_gradients(
    gradient: Mapping[str, object] | None,
) -> tuple[list[float], list[float]]:
    accelerations = [_DEFAULT_ACCELERATION_DPS2] * 7
    decelerations = [_DEFAULT_ACCELERATION_DPS2] * 7
    if gradient is None:
        return accelerations, decelerations
    try:
        acceleration = float(gradient["increase_dps2"])
        deceleration = float(gradient["decrease_dps2"])
        selected_joints = gradient["joints"]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("The transition acceleration settings are invalid.") from error
    if (
        not math.isfinite(acceleration)
        or acceleration <= 0
        or not math.isfinite(deceleration)
        or deceleration <= 0
        or not isinstance(selected_joints, list)
        or not selected_joints
        or any(
            isinstance(joint, bool)
            or not isinstance(joint, int)
            or not 1 <= joint <= 7
            for joint in selected_joints
        )
        or len(set(selected_joints)) != len(selected_joints)
    ):
        raise ValueError("The transition acceleration settings are invalid.")
    for joint in selected_joints:
        index = int(joint) - 1
        accelerations[index] = acceleration
        decelerations[index] = deceleration
    return accelerations, decelerations


def _limited_joint_speed(
    current_speed: float,
    desired_speed: float,
    acceleration: float,
    deceleration: float,
    elapsed_s: float,
) -> float:
    if current_speed * desired_speed < 0:
        desired_speed = 0.0
    rate = (
        acceleration
        if abs(desired_speed) > abs(current_speed)
        else deceleration
    )
    maximum_change = rate * elapsed_s
    change = max(
        -maximum_change,
        min(maximum_change, desired_speed - current_speed),
    )
    return current_speed + change


def _send_joint_speeds(base: Any, base_messages: Any, speeds: Sequence[float]) -> None:
    command = base_messages.JointSpeeds()
    for joint_index, speed in enumerate(speeds):
        joint_speed = command.joint_speeds.add()
        joint_speed.joint_identifier = joint_index
        joint_speed.value = float(speed)
        joint_speed.duration = 0
    base.SendJointSpeedsCommand(command)


def move_to_joint_position(
    connection: RobotConnection,
    base: Any,
    base_messages: Any,
    start: Sequence[float],
    destination: Sequence[float],
    maximum_speeds: Sequence[float],
    gradient: Mapping[str, object] | None,
    stop_event: threading.Event,
) -> bool:
    """Track a joint target with braking speeds and wait for measured settling."""
    if not (
        len(start) == len(destination) == len(maximum_speeds) == 7
    ):
        raise ValueError("A robot transition must contain exactly seven joints.")
    if any(
        not math.isfinite(float(angle))
        for angle in (*start, *destination)
    ):
        raise ValueError("Robot positions must contain finite joint angles.")
    speeds = [float(speed) for speed in maximum_speeds]
    if any(not math.isfinite(speed) or not 0 < speed <= 80 for speed in speeds):
        raise ValueError("Robot speeds must be finite and within the Gen3 limits.")
    joint_speed_limits = (80.0, 80.0, 80.0, 80.0, 70.0, 70.0, 70.0)
    if any(
        speed > joint_speed_limits[index]
        for index, speed in enumerate(speeds)
    ):
        raise ValueError("Robot speeds exceed the Gen3 joint limits.")

    accelerations, decelerations = _transition_gradients(gradient)
    timeout_at = time.monotonic() + _transition_timeout_seconds(
        start,
        destination,
        speeds,
        accelerations,
        decelerations,
    )
    commanded_speeds = [0.0] * 7
    previous_angles: list[float] | None = None
    previous_sample_at: float | None = None
    settled_samples = 0
    last_command_at = time.monotonic()

    while True:
        if stop_event.is_set():
            _send_joint_speeds(base, base_messages, [0.0] * 7)
            base.Stop()
            return False
        if time.monotonic() >= timeout_at:
            _send_joint_speeds(base, base_messages, [0.0] * 7)
            base.Stop()
            raise TimeoutError(
                "The robot did not reach the requested joint position before timeout."
            )

        now = time.monotonic()
        measured_angles = connection.read_joint_angles(base)
        errors = [
            float(target) - measured
            for target, measured in zip(destination, measured_angles, strict=True)
        ]
        elapsed_s = max(
            _CONTROL_PERIOD_S,
            now - last_command_at,
        )
        measured_velocities = [0.0] * 7
        if previous_angles is not None and previous_sample_at is not None:
            sample_interval = now - previous_sample_at
            if sample_interval > 0:
                measured_velocities = [
                    (angle - previous) / sample_interval
                    for angle, previous in zip(
                        measured_angles, previous_angles, strict=True
                    )
                ]

        settled = all(
            abs(error) <= _POSITION_TOLERANCE_DEG
            and abs(velocity) <= _VELOCITY_TOLERANCE_DPS
            for error, velocity in zip(errors, measured_velocities, strict=True)
        )
        settled_samples = settled_samples + 1 if settled else 0
        if settled_samples >= _SETTLED_SAMPLES_REQUIRED:
            _send_joint_speeds(base, base_messages, [0.0] * 7)
            base.Stop()
            return True

        next_speeds: list[float] = []
        for index, (error, maximum_speed) in enumerate(
            zip(errors, speeds, strict=True)
        ):
            distance_to_target = max(0.0, abs(error) - _POSITION_TOLERANCE_DEG)
            braking_speed = math.sqrt(
                2.0 * decelerations[index] * distance_to_target
            )
            desired_speed = math.copysign(
                min(maximum_speed, braking_speed), error
            )
            next_speeds.append(
                _limited_joint_speed(
                    commanded_speeds[index],
                    desired_speed,
                    accelerations[index],
                    decelerations[index],
                    elapsed_s,
                )
            )

        _send_joint_speeds(base, base_messages, next_speeds)
        commanded_speeds = next_speeds
        previous_angles = measured_angles
        previous_sample_at = now
        last_command_at = time.monotonic()
        stop_event.wait(_CONTROL_PERIOD_S)


def play_movement(
    connection: RobotConnection,
    movement: Mapping[str, object],
    loop_mode: str,
    play_count: int,
    time_limit_minutes: float,
    stop_event: threading.Event,
    progress_callback: Callable[[int, int], None] | None = None,
) -> bool:
    """Run a movement file, returning whether it finished without being stopped."""
    positions = movement.get("positions")
    speeds = movement.get("transition_speeds_dps")
    gradients = movement.get("transition_gradients")
    if (
        not isinstance(positions, list)
        or len(positions) < 2
        or not isinstance(speeds, list)
        or len(speeds) != len(positions)
    ):
        raise ValueError("Robot playback requires at least two valid positions.")
    if gradients is None:
        gradients = [None] * len(positions)
    elif not isinstance(gradients, list) or len(gradients) != len(positions):
        raise ValueError("Robot playback requires one transition gradient per row.")
    if any(
        gradient is not None and not isinstance(gradient, Mapping)
        for gradient in gradients
    ):
        raise ValueError("Robot playback transition gradients must be objects or null.")
    if loop_mode not in ("once", "until_stopped", "play_count", "time_limit"):
        raise ValueError(f"Unsupported robot playback mode: {loop_mode}")
    if loop_mode == "play_count" and play_count < 1:
        raise ValueError("The total play count must be at least one.")
    if loop_mode == "time_limit" and (
        not math.isfinite(time_limit_minutes) or time_limit_minutes <= 0
    ):
        raise ValueError("The time limit must be finite and greater than zero.")

    normalized_positions = [
        [float(angle) for angle in position]
        for position in positions
        if isinstance(position, list) and len(position) == 7
    ]
    normalized_speeds = [
        [float(speed) for speed in row]
        for row in speeds
        if isinstance(row, list) and len(row) == 7
    ]
    if len(normalized_positions) != len(positions) or len(normalized_speeds) != len(
        positions
    ):
        raise ValueError("Robot playback data must contain seven values per row.")
    if any(
        not math.isfinite(angle)
        for position in normalized_positions
        for angle in position
    ):
        raise ValueError("Robot playback positions must be finite joint angles.")
    speed_limits = (80.0, 80.0, 80.0, 80.0, 70.0, 70.0, 70.0)
    if any(
        not math.isfinite(speed) or not 0 < speed <= speed_limits[joint]
        for row in normalized_speeds
        for joint, speed in enumerate(row)
    ):
        raise ValueError("Robot playback speeds exceed the Gen3 joint limits.")

    base = connection.create_base_client()
    current_angles = connection.read_joint_angles()
    (
        _,
        _,
        _,
        _,
        _,
        base_messages,
    ) = _load_kortex_api()
    servoing_mode = base_messages.ServoingModeInformation()
    servoing_mode.servoing_mode = base_messages.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(servoing_mode)

    deadline = (
        time.monotonic() + time_limit_minutes * 60
        if loop_mode == "time_limit"
        else None
    )

    def execute_transition(
        start: Sequence[float],
        destination: Sequence[float],
        transition_speeds: Sequence[float],
        transition_gradient: Mapping[str, object] | None,
    ) -> bool:
        return move_to_joint_position(
            connection,
            base,
            base_messages,
            start,
            destination,
            transition_speeds,
            transition_gradient,
            stop_event,
        )

    def report_progress(row: int, completed_cycles: int) -> None:
        if progress_callback is not None:
            progress_callback(row, completed_cycles)

    if not execute_transition(
        current_angles,
        normalized_positions[0],
        normalized_speeds[0],
        gradients[0] if isinstance(gradients[0], Mapping) else None,
    ):
        return False
    report_progress(0, 0)
    if deadline is not None and time.monotonic() >= deadline:
        return True

    completed_cycles = 0
    while not stop_event.is_set():
        for row in range(len(normalized_positions) - 1):
            if not execute_transition(
                normalized_positions[row],
                normalized_positions[row + 1],
                normalized_speeds[row],
                gradients[row] if isinstance(gradients[row], Mapping) else None,
            ):
                return False
            report_progress(row + 1, completed_cycles)
            if deadline is not None and time.monotonic() >= deadline:
                return True
        completed_cycles += 1

        if loop_mode == "once" or (
            loop_mode == "play_count" and completed_cycles >= play_count
        ):
            return True
        if deadline is not None and time.monotonic() >= deadline:
            return True
        if not execute_transition(
            normalized_positions[-1],
            normalized_positions[0],
            normalized_speeds[-1],
            gradients[-1] if isinstance(gradients[-1], Mapping) else None,
        ):
            return False
        report_progress(0, completed_cycles)
    return False


class RobotPlaybackWorker(QThread):
    """Run robot playback outside the Qt interface thread."""

    progress = Signal(int, int)
    completed = Signal(bool)
    failed = Signal(str)

    def __init__(
        self,
        connection: RobotConnection,
        movement: Mapping[str, object],
        loop_mode: str,
        play_count: int,
        time_limit_minutes: float,
        parent: Any = None,
    ) -> None:
        super().__init__(parent)
        self._connection = connection
        self._movement = movement
        self._loop_mode = loop_mode
        self._play_count = play_count
        self._time_limit_minutes = time_limit_minutes
        self._stop_event = threading.Event()

    def run(self) -> None:
        try:
            finished = play_movement(
                self._connection,
                self._movement,
                self._loop_mode,
                self._play_count,
                self._time_limit_minutes,
                self._stop_event,
                self.progress.emit,
            )
        except Exception as error:
            self.failed.emit(str(error))
        else:
            self.completed.emit(finished)

    def request_stop(self) -> None:
        self._stop_event.set()

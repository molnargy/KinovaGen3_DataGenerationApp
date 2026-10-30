import json
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from kinova_movement_app.layout import (
    JOINT_NAMES,
    JOINT_ANGLE_LIMITS_DEG,
    JOINT_SPEED_LIMITS_DPS,
    GRADIENT_COLUMN,
    POSITION_COLUMN_START,
    SPEED_COLUMN_START,
    MainWindow,
)
from kinova_movement_app.preview_anim import (
    Arm3DWidget,
    MovementAnimationDialog,
    forward_kinematics,
    gripper_forward_direction,
    transition_duration_ms,
)


class LayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        self.window = MainWindow()

    def tearDown(self) -> None:
        self.window.close()

    def test_navigation_switches_between_pages(self) -> None:
        self.window.nav_buttons[1].click()

        self.assertEqual(self.window.pages.currentIndex(), 1)
        self.assertEqual(self.window.nav_buttons[1].property("active"), True)

    def test_waypoint_can_be_added_and_saved_as_json(self) -> None:
        self.window.movement_name.setText("Test movement")
        for index, spin_box in enumerate(self.window.joint_inputs, start=1):
            spin_box.setValue(float(index))
        self.window._add_waypoint()

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "movement.json"
            with (
                patch(
                    "kinova_movement_app.layout.QFileDialog.getSaveFileName",
                    return_value=(str(path), ""),
                ),
                patch("kinova_movement_app.layout.QMessageBox.information"),
            ):
                self.window._save_movement()

            saved = json.loads(path.read_text(encoding="utf-8"))

        self.assertEqual(saved["name"], "Test movement")
        self.assertEqual(saved["robot"], "Kinova Gen3")
        self.assertEqual(saved["joint_names"], list(JOINT_NAMES))
        self.assertEqual(saved["positions"], [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]])
        self.assertEqual(saved["transition_speeds_dps"], [[30.0] * 7])
        self.assertEqual(
            self.window._loaded_movement["positions"],
            saved["positions"],
        )
        self.assertIn("movement.json", self.window.loaded_file_label.text())

    def test_added_waypoint_and_transition_speed_can_be_edited_and_saved(self) -> None:
        self.window.movement_name.setText("Editable movement")
        self.window._add_waypoint()
        for index, spin_box in enumerate(self.window.joint_inputs, start=1):
            spin_box.setValue(float(index * 10))
        self.window._add_waypoint()
        self.window.record_table.item(0, 1).setText("42.5")
        self.window.record_table.item(0, 8).setText("12.5")

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "movement.json"
            with (
                patch(
                    "kinova_movement_app.layout.QFileDialog.getSaveFileName",
                    return_value=(str(path), ""),
                ),
                patch("kinova_movement_app.layout.QMessageBox.information"),
            ):
                self.window._save_movement()

            saved = json.loads(path.read_text(encoding="utf-8"))

        self.assertEqual(saved["positions"][0][0], 42.5)
        self.assertEqual(saved["transition_speeds_dps"][0][0], 12.5)
        self.assertEqual(
            saved["transition_speeds_dps"][0],
            [12.5, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0],
        )

    def test_transition_gradient_controls_are_saved_per_waypoint(self) -> None:
        self.window.movement_name.setText("Ramped movement")
        self.window._add_waypoint()
        self.window.joint_inputs[0].setValue(45)
        self.window._add_waypoint()
        editor = self.window.record_table.cellWidget(0, GRADIENT_COLUMN)
        editor.enabled.setChecked(True)
        editor.increase.setValue(18.5)
        editor.decrease.setValue(22.5)
        editor.scope.setCurrentIndex(editor.scope.findData("selected"))
        for action in editor.joint_actions:
            action.setChecked(int(action.data()) in (1, 4))

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "movement.json"
            with (
                patch(
                    "kinova_movement_app.layout.QFileDialog.getSaveFileName",
                    return_value=(str(path), ""),
                ),
                patch("kinova_movement_app.layout.QMessageBox.information"),
            ):
                self.window._save_movement()

            saved = json.loads(path.read_text(encoding="utf-8"))
            with patch(
                "kinova_movement_app.layout.QFileDialog.getOpenFileName",
                return_value=(str(path), ""),
            ):
                self.window._open_movement()

        self.assertEqual(
            saved["transition_gradients"],
            [
                {
                    "increase_dps2": 18.5,
                    "decrease_dps2": 22.5,
                    "joints": [1, 4],
                },
                None,
            ],
        )
        self.assertEqual(
            self.window._loaded_movement["transition_gradients"],
            saved["transition_gradients"],
        )

    def test_legacy_movements_default_transition_gradients_to_disabled(self) -> None:
        gradients = MainWindow._validate_transition_gradients(None, 2)

        self.assertEqual(gradients, [None, None])

    def test_transition_gradient_validation_rejects_empty_joint_selection(self) -> None:
        with self.assertRaisesRegex(ValueError, "one or more"):
            MainWindow._validate_transition_gradients(
                [
                    {
                        "increase_dps2": 30,
                        "decrease_dps2": 30,
                        "joints": [],
                    }
                ],
                1,
            )

    def test_angle_editors_apply_ultra_joint_limits(self) -> None:
        self.window._add_waypoint()
        self.assertEqual(
            JOINT_ANGLE_LIMITS_DEG,
            (None, 126.0, None, 147.0, None, 117.0, None),
        )
        for joint_index, limit in enumerate(JOINT_ANGLE_LIMITS_DEG):
            editor = self.window.record_table.itemDelegateForColumn(
                POSITION_COLUMN_START + joint_index
            ).createEditor(
                self.window.record_table,
                object(),
                self.window.record_table.model().index(
                    0, POSITION_COLUMN_START + joint_index
                ),
            )
            if limit is None:
                self.assertGreater(editor.maximum(), 1_000_000)
                self.assertLess(editor.minimum(), -1_000_000)
            else:
                self.assertEqual(editor.minimum(), -limit)
                self.assertEqual(editor.maximum(), limit)

        for joint_index, limit in enumerate(JOINT_ANGLE_LIMITS_DEG):
            self.assertEqual(self.window.joint_inputs[joint_index].minimum(),
                             -limit if limit is not None else -1_000_000_000)
            self.assertEqual(self.window.joint_inputs[joint_index].maximum(),
                             limit if limit is not None else 1_000_000_000)

    def test_angle_validation_enforces_limited_joints_and_allows_unlimited(self) -> None:
        self.window._add_waypoint()
        for joint_index in (0, 2, 4, 6):
            self.window.record_table.item(
                0, POSITION_COLUMN_START + joint_index
            ).setText("1000000")
        for joint_index, limit in ((1, 126), (3, 147), (5, 117)):
            self.window.record_table.item(
                0, POSITION_COLUMN_START + joint_index
            ).setText(str(limit))

        positions, _ = self.window._read_waypoint_data(self.window.record_table)
        self.assertEqual(positions[0][0], 1_000_000)

        for joint_index, limit in ((1, 126), (3, 147), (5, 117)):
            self.window.record_table.item(
                0, POSITION_COLUMN_START + joint_index
            ).setText(str(limit + 0.01))
            with self.assertRaisesRegex(ValueError, f"Joint {joint_index + 1}"):
                self.window._read_waypoint_data(self.window.record_table)
            self.window.record_table.item(
                0, POSITION_COLUMN_START + joint_index
            ).setText(str(limit))

    def test_movement_file_angle_validation_rejects_limited_joint_overflow(self) -> None:
        movement = {
            "name": "Out of range",
            "joint_names": list(JOINT_NAMES),
            "positions": [[0, 127, 0, 0, 0, 0, 0]],
        }

        with self.assertRaisesRegex(ValueError, "Gen3 Ultra limits"):
            MainWindow._validate_movement(movement)

    def test_selected_recorded_position_can_be_deleted(self) -> None:
        for value in (10.0, 20.0, 30.0):
            self.window.joint_inputs[0].setValue(value)
            self.window._add_waypoint()
        self.window.record_table.item(0, 8).setText("11.00")
        self.window.record_table.item(1, 8).setText("22.00")
        self.window.record_table.item(2, 8).setText("33.00")
        self.window.record_table.selectRow(1)

        self.window.delete_waypoint_button.click()

        self.assertEqual(self.window.record_table.rowCount(), 2)
        self.assertEqual(self.window.record_table.item(0, 0).text(), "1")
        self.assertEqual(self.window.record_table.item(1, 0).text(), "2")
        self.assertEqual(self.window.record_table.item(0, 1).text(), "10.00")
        self.assertEqual(self.window.record_table.item(1, 1).text(), "30.00")
        self.assertEqual(self.window.record_table.item(0, 8).text(), "11.00")
        self.assertEqual(self.window.record_table.item(1, 8).text(), "33.00")
        self.assertTrue(self.window.delete_waypoint_button.isEnabled())

    def test_deleting_first_or_last_position_keeps_rows_consistent(self) -> None:
        for value in (10.0, 20.0, 30.0):
            self.window.joint_inputs[0].setValue(value)
            self.window._add_waypoint()
        self.window.record_table.item(1, 8).setText("22.00")
        self.window.record_table.selectRow(0)
        self.window._delete_selected_waypoint()

        self.assertEqual(self.window.record_table.rowCount(), 2)
        self.assertEqual(self.window.record_table.item(0, 1).text(), "20.00")
        self.assertEqual(self.window.record_table.item(0, 8).text(), "22.00")

        self.window.record_table.selectRow(1)
        self.window._delete_selected_waypoint()

        self.assertEqual(self.window.record_table.rowCount(), 1)
        self.assertEqual(self.window.record_table.item(0, 0).text(), "1")
        self.assertEqual(self.window.record_table.item(0, 8).text(), "22.00")

    def test_deleting_only_position_disables_delete_button(self) -> None:
        self.window._add_waypoint()

        self.window.delete_waypoint_button.click()

        self.assertEqual(self.window.record_table.rowCount(), 0)
        self.assertFalse(self.window.delete_waypoint_button.isEnabled())

    def test_movement_file_can_be_loaded_for_preview(self) -> None:
        movement = {
            "name": "Preview movement",
            "robot": "Kinova Gen3",
            "units": "degrees",
            "joint_names": list(JOINT_NAMES),
            "positions": [[0, 1, 2, 3, 4, 5, 6], [1, 2, 3, 4, 5, 6, 7]],
            "transition_speeds_dps": [
                [10, 20, 30, 40, 50, 60, 70],
                [11, 21, 31, 41, 51, 61, 69],
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "movement.json"
            path.write_text(json.dumps(movement), encoding="utf-8")
            with patch(
                "kinova_movement_app.layout.QFileDialog.getOpenFileName",
                return_value=(str(path), ""),
            ):
                self.window._open_movement()

        self.assertEqual(self.window.playback_table.rowCount(), 2)
        self.assertEqual(self.window.playback_table.item(0, 7).text(), "6.00")
        self.assertEqual(self.window.playback_table.item(0, 8).text(), "10.00")
        self.assertEqual(self.window.playback_table.item(1, 8).text(), "11.00")
        self.assertIn("Preview movement", self.window.loaded_file_label.text())
        self.assertTrue(self.window.play_button.isEnabled())

    def test_record_page_can_preview_a_movement_with_repeat_count(self) -> None:
        self.window._add_waypoint()
        self.window.record_loop_mode.setCurrentIndex(
            self.window.record_loop_mode.findData("play_count")
        )
        self.window.record_play_count.setValue(2)
        self.window._toggle_preview(
            self.window.record_table,
            self.window.record_loop_mode,
            self.window.record_play_count,
            self.window.record_time_limit,
            self.window.record_play_button,
        )

        self.assertTrue(self.window._preview_timer.isActive())
        self.assertEqual(self.window.record_play_button.text(), "Stop preview")
        self.window._advance_preview()
        self.assertTrue(self.window._preview_timer.isActive())
        self.window._advance_preview()
        self.assertFalse(self.window._preview_timer.isActive())
        self.assertEqual(self.window.record_play_button.text(), "Play preview")

    def test_recording_preview_opens_and_animates_the_arm_visualization(self) -> None:
        self.window._add_waypoint()
        for spin_box in self.window.joint_inputs:
            spin_box.setValue(20)
        self.window._add_waypoint()

        self.window._toggle_preview(
            self.window.record_table,
            self.window.record_loop_mode,
            self.window.record_play_count,
            self.window.record_time_limit,
            self.window.record_play_button,
        )

        animation = self.window._preview_animation
        self.assertIsInstance(animation, MovementAnimationDialog)
        self.assertTrue(animation.isVisible())
        self.assertEqual(animation._target_angles, [20.0] * 7)
        self.assertTrue(self.window._preview_timer.isActive())

        self.window._stop_preview()

    def test_3d_arm_has_seven_movable_joints_and_interpolates(self) -> None:
        animation = MovementAnimationDialog(self.window)
        self.assertIsInstance(animation.arm, Arm3DWidget)
        self.assertEqual(len(animation.arm.joint_transforms), 7)
        animation.set_pose([0.0] * 7, 1)
        animation.animate_to([20.0] * 7, [30.0] * 7, 100, 2)
        animation._segment_started_at = time.monotonic() - 0.2
        animation._update_animation()

        self.assertEqual(animation.arm.angles, [20.0] * 7)
        self.assertFalse(animation._timer.isActive())
        animation.close()

    def test_3d_arm_preview_uses_the_transition_acceleration_profile(self) -> None:
        animation = MovementAnimationDialog(self.window)
        animation.set_pose([0.0] * 7, 1)
        animation.animate_to(
            [90.0] + [0.0] * 6,
            [60.0] * 7,
            100,
            2,
            gradient={
                "increase_dps2": 30.0,
                "decrease_dps2": 30.0,
                "joints": [1],
            },
        )
        animation._segment_started_at = time.monotonic() - (
            animation._segment_duration_ms / 4000
        )

        animation._update_animation()

        self.assertAlmostEqual(animation.arm.angles[0], 11.25, places=1)
        self.assertNotAlmostEqual(animation.arm.angles[0], 22.5, places=1)
        animation.close()

    def test_transition_duration_respects_per_joint_speed(self) -> None:
        duration = transition_duration_ms(
            [0.0] * 7,
            [90.0, 30.0, 10.0, 0.0, 0.0, 0.0, 0.0],
            [90.0, 30.0, 10.0, 30.0, 30.0, 30.0, 30.0],
        )

        self.assertEqual(duration, 1000)

    def test_preview_duration_includes_low_acceleration_ramp(self) -> None:
        duration = transition_duration_ms(
            [0.0] * 7,
            [10.0] + [0.0] * 6,
            [20.0] * 7,
            {
                "increase_dps2": 0.01,
                "decrease_dps2": 0.01,
                "joints": [1],
            },
        )

        self.assertGreater(duration, 60_000)

    def test_speed_caps_match_large_and_small_actuators(self) -> None:
        self.window._add_waypoint()
        self.assertEqual(JOINT_SPEED_LIMITS_DPS, (80, 80, 80, 80, 70, 70, 70))
        for joint_index, limit in enumerate(JOINT_SPEED_LIMITS_DPS):
            editor = self.window.record_table.itemDelegateForColumn(
                SPEED_COLUMN_START + joint_index
            ).createEditor(
                self.window.record_table,
                object(),
                self.window.record_table.model().index(0, SPEED_COLUMN_START + joint_index),
            )
            self.assertEqual(editor.maximum(), limit)

    def test_speed_validation_rejects_values_above_each_joint_limit(self) -> None:
        self.window._add_waypoint()
        for joint_index, limit in enumerate(JOINT_SPEED_LIMITS_DPS):
            self.window.record_table.item(
                0, SPEED_COLUMN_START + joint_index
            ).setText(str(limit + 1))

            with self.assertRaisesRegex(ValueError, f"Joint {joint_index + 1}"):
                self.window._read_waypoint_data(self.window.record_table)

            self.window.record_table.item(
                0, SPEED_COLUMN_START + joint_index
            ).setText(str(limit))

    def test_final_position_speed_controls_loop_back_transition(self) -> None:
        self.window._add_waypoint()
        self.window.joint_inputs[0].setValue(45)
        self.window._add_waypoint()
        loop_speeds = [45.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
        for joint_index, speed in enumerate(loop_speeds):
            self.window.record_table.item(
                1, SPEED_COLUMN_START + joint_index
            ).setText(str(speed))
        gradient_editor = self.window.record_table.cellWidget(1, GRADIENT_COLUMN)
        gradient_editor.enabled.setChecked(True)
        gradient_editor.scope.setCurrentIndex(
            gradient_editor.scope.findData("selected")
        )
        for action in gradient_editor.joint_actions:
            action.setChecked(int(action.data()) == 1)
        self.window.record_loop_mode.setCurrentIndex(
            self.window.record_loop_mode.findData("until_stopped")
        )
        self.window._toggle_preview(
            self.window.record_table,
            self.window.record_loop_mode,
            self.window.record_play_count,
            self.window.record_time_limit,
            self.window.record_play_button,
        )
        self.window._advance_preview()

        with (
            patch("kinova_movement_app.layout.transition_duration_ms", return_value=123),
            patch.object(self.window._preview_animation, "animate_to") as animate_to,
        ):
            self.window._schedule_preview_step()

        self.assertEqual(
            animate_to.call_args.args[0],
            self.window._preview_positions[0],
        )
        self.assertEqual(animate_to.call_args.args[1], loop_speeds)
        self.assertEqual(
            animate_to.call_args.kwargs["gradient"],
            {
                "increase_dps2": 30.0,
                "decrease_dps2": 30.0,
                "joints": [1],
            },
        )
        self.window._stop_preview()

    def test_gen3_forward_kinematics_matches_zero_angle_reference_pose(self) -> None:
        joint_centers, tool_position = forward_kinematics([0.0] * 7)

        self.assertEqual(len(joint_centers), 7)
        self.assertAlmostEqual(joint_centers[0].z(), 0.15643, places=5)
        self.assertAlmostEqual(tool_position.x(), 0.0, places=5)
        self.assertAlmostEqual(tool_position.y(), -0.0249, places=3)
        self.assertAlmostEqual(tool_position.z(), 1.1874, places=3)

    def test_zero_pose_visual_gripper_points_upward(self) -> None:
        direction = gripper_forward_direction([0.0] * 7)

        self.assertAlmostEqual(direction.x(), 0.0, places=5)
        self.assertAlmostEqual(direction.y(), 1.0, places=5)
        self.assertAlmostEqual(direction.z(), 0.0, places=5)

    def test_first_joint_rotation_orbits_the_arm_around_the_vertical_axis(self) -> None:
        _, zero_pose = forward_kinematics([0.0] * 7)
        _, rotated_pose = forward_kinematics([90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.assertNotAlmostEqual(zero_pose.x(), rotated_pose.x(), places=2)
        self.assertNotAlmostEqual(zero_pose.y(), rotated_pose.y(), places=2)
        self.assertAlmostEqual(zero_pose.z(), rotated_pose.z(), places=5)

    def test_preview_loop_options_include_count_and_time_limit(self) -> None:
        expected = ["once", "until_stopped", "play_count", "time_limit"]

        self.assertEqual(
            [
                self.window.play_loop_mode.itemData(index)
                for index in range(self.window.play_loop_mode.count())
            ],
            expected,
        )
        self.assertEqual(
            [
                self.window.record_loop_mode.itemData(index)
                for index in range(self.window.record_loop_mode.count())
            ],
            expected,
        )

    def test_time_limited_preview_stops_at_its_deadline(self) -> None:
        self.window._add_waypoint()
        self.window._toggle_preview(
            self.window.record_table,
            self.window.record_loop_mode,
            self.window.record_play_count,
            self.window.record_time_limit,
            self.window.record_play_button,
        )
        self.window._preview_mode = "time_limit"
        self.window._preview_deadline = time.monotonic() - 1

        self.window._advance_preview()

        self.assertFalse(self.window._preview_timer.isActive())
        self.assertEqual(self.window.record_play_button.text(), "Play preview")

    def test_legacy_movement_without_transition_speeds_loads_defaults(self) -> None:
        movement = {
            "name": "Legacy movement",
            "joint_names": list(JOINT_NAMES),
            "positions": [[0] * 7, [1] * 7],
        }

        positions, speeds = MainWindow._validate_movement(movement)

        self.assertEqual(len(positions), 2)
        self.assertEqual(speeds, [[30.0] * 7, [30.0] * 7])


if __name__ == "__main__":
    unittest.main()

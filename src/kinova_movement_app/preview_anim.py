"""Animated 3D kinematic preview for seven-joint movements."""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping

from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.QtCore import QTimer
from PySide6.QtGui import QCloseEvent, QColor, QQuaternion, QVector3D
from PySide6.QtWidgets import QDialog, QLabel, QVBoxLayout, QWidget

from kinova_movement_app.playback import build_transition_profile


JOINT_COUNT = 7
ANIMATION_FRAME_MS = 16
MIN_TRANSITION_MS = 100
JOINT_ORIGINS = (
    (0.0, 0.0, 0.15643),
    (0.0, 0.005375, -0.12838),
    (0.0, -0.21038, -0.006375),
    (0.0, 0.006375, -0.21038),
    (0.0, -0.20843, -0.006375),
    (0.0, 0.00017505, -0.10593),
    (0.0, -0.10593, -0.00017505),
)
JOINT_ORIGIN_RPY = (
    (math.pi, 0.0, 0.0),
    (math.pi / 2.0, 0.0, 0.0),
    (-math.pi / 2.0, 0.0, 0.0),
    (math.pi / 2.0, 0.0, 0.0),
    (-math.pi / 2.0, 0.0, 0.0),
    (math.pi / 2.0, 0.0, 0.0),
    (-math.pi / 2.0, 0.0, 0.0),
)
JOINT_AXIS = QVector3D(0.0, 0.0, 1.0)
END_EFFECTOR_ORIGIN = (0.0, 0.0, -0.061525)
END_EFFECTOR_RPY = (math.pi, 0.0, 0.0)
MODEL_TO_SCENE_ROTATION = QQuaternion.fromAxisAndAngle(
    QVector3D(1.0, 0.0, 0.0), -90.0
)
GRIPPER_VISUAL_ROTATION = QQuaternion()


def _origin_rotation(rpy: tuple[float, float, float]) -> QQuaternion:
    """Create the URDF fixed-axis roll/pitch/yaw rotation."""
    roll, pitch, yaw = rpy
    return (
        QQuaternion.fromAxisAndAngle(QVector3D(0.0, 0.0, 1.0), math.degrees(yaw))
        * QQuaternion.fromAxisAndAngle(QVector3D(0.0, 1.0, 0.0), math.degrees(pitch))
        * QQuaternion.fromAxisAndAngle(QVector3D(1.0, 0.0, 0.0), math.degrees(roll))
    )


def forward_kinematics(
    angles: list[float],
) -> tuple[list[QVector3D], QVector3D]:
    """Return joint centers and tool position in the Gen3 URDF base frame."""
    if len(angles) != JOINT_COUNT or any(not math.isfinite(angle) for angle in angles):
        raise ValueError("The arm pose requires seven finite joint angles.")

    position = QVector3D()
    rotation = QQuaternion()
    joint_centers: list[QVector3D] = []
    for origin, rpy, angle in zip(
        JOINT_ORIGINS, JOINT_ORIGIN_RPY, angles, strict=True
    ):
        position += rotation.rotatedVector(QVector3D(*origin))
        rotation = rotation * _origin_rotation(rpy)
        joint_centers.append(QVector3D(position))
        rotation = rotation * QQuaternion.fromAxisAndAngle(JOINT_AXIS, angle)

    tool_position = position + rotation.rotatedVector(
        QVector3D(*END_EFFECTOR_ORIGIN)
    )
    return joint_centers, tool_position


def gripper_forward_direction(angles: list[float]) -> QVector3D:
    """Return the drawn gripper's forward direction in the 3D scene."""
    if len(angles) != JOINT_COUNT or any(not math.isfinite(angle) for angle in angles):
        raise ValueError("The arm pose requires seven finite joint angles.")
    rotation = QQuaternion()
    for rpy, angle in zip(JOINT_ORIGIN_RPY, angles, strict=True):
        rotation = (
            rotation
            * _origin_rotation(rpy)
            * QQuaternion.fromAxisAndAngle(JOINT_AXIS, angle)
        )
    rotation *= _origin_rotation(END_EFFECTOR_RPY)
    local_forward = GRIPPER_VISUAL_ROTATION.rotatedVector(
        QVector3D(0.0, 0.0, 1.0)
    )
    return MODEL_TO_SCENE_ROTATION.rotatedVector(
        rotation.rotatedVector(local_forward)
    )


def transition_duration_ms(
    start: list[float],
    end: list[float],
    speeds_dps: list[float],
    gradient: Mapping[str, object] | None = None,
) -> int:
    """Return the preview duration for the configured speed and ramp."""
    if not (len(start) == len(end) == len(speeds_dps) == JOINT_COUNT):
        raise ValueError("A transition requires seven angles and seven speeds.")
    if any(not math.isfinite(speed) or speed <= 0 for speed in speeds_dps):
        raise ValueError("Joint speeds must be positive finite values.")
    if gradient is not None:
        duration_s, _ = build_transition_profile(
            start,
            end,
            speeds_dps,
            gradient,
        )
        return max(MIN_TRANSITION_MS, math.ceil(duration_s * 1000))
    seconds = max(
        abs(target - current) / speed
        for current, target, speed in zip(start, end, speeds_dps, strict=True)
    )
    return max(MIN_TRANSITION_MS, math.ceil(seconds * 1000))


def _add_transform(
    entity: Qt3DCore.QEntity,
    translation: QVector3D | None = None,
    rotation: QQuaternion | None = None,
) -> Qt3DCore.QTransform:
    transform = Qt3DCore.QTransform(entity)
    if translation is not None:
        transform.setTranslation(translation)
    if rotation is not None:
        transform.setRotation(rotation)
    entity.addComponent(transform)
    return transform


def _make_material(
    parent: Qt3DCore.QEntity, diffuse: QColor, specular: QColor
) -> Qt3DExtras.QPhongMaterial:
    material = Qt3DExtras.QPhongMaterial(parent)
    material.setDiffuse(diffuse)
    material.setSpecular(specular)
    material.setShininess(55.0)
    return material


class Arm3DWidget(QWidget):
    """Embed an orbitable Qt 3D scene containing a seven-axis arm model."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._angles = [0.0] * JOINT_COUNT
        self._joint_transforms: list[Qt3DCore.QTransform] = []
        self._gripper_visual_transform: Qt3DCore.QTransform | None = None
        self.setMinimumSize(480, 340)
        self._scene = Qt3DCore.QEntity()
        self._build_scene()

        self._view = Qt3DExtras.Qt3DWindow()
        self._view.defaultFrameGraph().setClearColor(QColor("#101722"))
        self._view.setRootEntity(self._scene)
        self._configure_camera()
        container = QWidget.createWindowContainer(self._view, self)
        container.setMinimumSize(480, 340)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(container)

    @property
    def angles(self) -> list[float]:
        return list(self._angles)

    @property
    def joint_transforms(self) -> tuple[Qt3DCore.QTransform, ...]:
        return tuple(self._joint_transforms)

    @property
    def gripper_visual_transform(self) -> Qt3DCore.QTransform | None:
        return self._gripper_visual_transform

    def set_angles(self, angles: list[float]) -> None:
        forward_kinematics(angles)
        self._angles = list(angles)
        for transform, angle in zip(self._joint_transforms, angles, strict=True):
            transform.setRotation(QQuaternion.fromAxisAndAngle(JOINT_AXIS, angle))

    def _build_scene(self) -> None:
        floor = Qt3DCore.QEntity(self._scene)
        floor_mesh = Qt3DExtras.QPlaneMesh(floor)
        floor_mesh.setWidth(5.0)
        floor_mesh.setHeight(5.0)
        floor.addComponent(floor_mesh)
        floor_material = _make_material(
            floor, QColor("#172333"), QColor("#35455a")
        )
        floor.addComponent(floor_material)
        floor_transform = _add_transform(floor, QVector3D(0.0, -0.035, 0.0))
        floor_transform.setRotation(
            QQuaternion.fromAxisAndAngle(QVector3D(1.0, 0.0, 0.0), -90.0)
        )

        base = Qt3DCore.QEntity(self._scene)
        base_mesh = Qt3DExtras.QCylinderMesh(base)
        base_mesh.setRadius(0.095)
        base_mesh.setLength(0.14)
        base.addComponent(base_mesh)
        base.addComponent(
            _make_material(base, QColor("#53657c"), QColor("#c1d0df"))
        )
        _add_transform(base, QVector3D(0.0, 0.07, 0.0))

        model_frame = Qt3DCore.QEntity(self._scene)
        _add_transform(model_frame, rotation=MODEL_TO_SCENE_ROTATION)
        link_materials = [
            _make_material(
                self._scene,
                QColor("#168696") if index % 2 == 0 else QColor("#286078"),
                QColor("#b6e4e6"),
            )
            for index in range(JOINT_COUNT)
        ]
        joint_material = _make_material(
            self._scene, QColor("#f0b35c"), QColor("#fff0cf")
        )
        end_material = _make_material(
            self._scene, QColor("#52677e"), QColor("#dce7f3")
        )

        current_frame = model_frame
        for index, (origin, rpy) in enumerate(
            zip(JOINT_ORIGINS, JOINT_ORIGIN_RPY, strict=True)
        ):
            joint_frame = Qt3DCore.QEntity(current_frame)
            _add_transform(
                joint_frame,
                QVector3D(*origin),
                _origin_rotation(rpy),
            )
            joint_sphere = Qt3DCore.QEntity(joint_frame)
            joint_mesh = Qt3DExtras.QSphereMesh(joint_sphere)
            joint_mesh.setRadius(0.055 if index in (0, 2, 4, 6) else 0.047)
            joint_sphere.addComponent(joint_mesh)
            joint_sphere.addComponent(joint_material)

            moving_frame = Qt3DCore.QEntity(joint_frame)
            joint_transform = _add_transform(moving_frame)
            self._joint_transforms.append(joint_transform)

            if index < JOINT_COUNT - 1:
                next_origin = JOINT_ORIGINS[index + 1]
                delta = QVector3D(*next_origin)
                self._add_link(moving_frame, delta, link_materials[index])
                current_frame = moving_frame
            else:
                tool_delta = QVector3D(*END_EFFECTOR_ORIGIN)
                self._add_link(moving_frame, tool_delta, link_materials[index])
                tool_frame = Qt3DCore.QEntity(moving_frame)
                _add_transform(
                    tool_frame,
                    tool_delta,
                    _origin_rotation(END_EFFECTOR_RPY),
                )
                gripper_visual = Qt3DCore.QEntity(tool_frame)
                self._gripper_visual_transform = _add_transform(
                    gripper_visual, rotation=GRIPPER_VISUAL_ROTATION
                )
                tool = Qt3DCore.QEntity(gripper_visual)
                tool_mesh = Qt3DExtras.QSphereMesh(tool)
                tool_mesh.setRadius(0.045)
                tool.addComponent(tool_mesh)
                tool.addComponent(end_material)

        for offset in (-0.075, 0.075):
            finger = Qt3DCore.QEntity(gripper_visual)
            finger_mesh = Qt3DExtras.QCuboidMesh(finger)
            finger_mesh.setXExtent(0.035)
            finger_mesh.setYExtent(0.035)
            finger_mesh.setZExtent(0.14)
            finger.addComponent(finger_mesh)
            finger.addComponent(end_material)
            _add_transform(finger, QVector3D(offset, 0.0, 0.10))

    @staticmethod
    def _add_link(
        parent: Qt3DCore.QEntity,
        endpoint: QVector3D,
        material: Qt3DExtras.QPhongMaterial,
    ) -> None:
        length = endpoint.length()
        if length == 0:
            return
        link = Qt3DCore.QEntity(parent)
        mesh = Qt3DExtras.QCylinderMesh(link)
        mesh.setRadius(0.038)
        mesh.setLength(length)
        link.addComponent(mesh)
        link.addComponent(material)
        transform = _add_transform(link, endpoint / 2.0)
        transform.setRotation(
            QQuaternion.rotationTo(
                QVector3D(0.0, 1.0, 0.0), endpoint.normalized()
            )
        )

    def _configure_camera(self) -> None:
        camera = self._view.camera()
        camera.lens().setPerspectiveProjection(42.0, 1.4, 0.05, 100.0)
        camera.setPosition(QVector3D(2.6, 1.9, 3.2))
        camera.setViewCenter(QVector3D(0.0, 0.85, 0.0))
        camera.setUpVector(QVector3D(0.0, 1.0, 0.0))

        controller = Qt3DExtras.QOrbitCameraController(self._scene)
        controller.setCamera(camera)
        controller.setLinearSpeed(4.0)
        controller.setLookSpeed(140.0)
        controller.setUpVector(QVector3D(0.0, 1.0, 0.0))


class MovementAnimationDialog(QDialog):
    """Display an animated 3D kinematic approximation of a Gen3 arm."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("3D movement preview")
        self.resize(760, 600)
        self.setMinimumSize(600, 480)
        self._start_angles = [0.0] * JOINT_COUNT
        self._target_angles = [0.0] * JOINT_COUNT
        self._segment_duration_ms = 0
        self._segment_started_at = 0.0
        self._segment_profile: Callable[[float], list[float]] | None = None

        layout = QVBoxLayout(self)
        self.description = QLabel(
            "Interactive 3D kinematic approximation with an upward-pointing "
            "visual gripper at the all-zero pose. Drag to orbit and use the mouse "
            "wheel to zoom. Gripper orientation is adjusted for display; "
            "this is not a calibrated Gen3 model or a command to the robot."
        )
        self.description.setWordWrap(True)
        self.description.setStyleSheet("color: #9cabc0;")
        layout.addWidget(self.description)
        self.arm = Arm3DWidget(self)
        layout.addWidget(self.arm, 1)
        self.pose_label = QLabel("Waypoint 1")
        self.pose_label.setStyleSheet("font-weight: 600;")
        layout.addWidget(self.pose_label)

        self._timer = QTimer(self)
        self._timer.setInterval(ANIMATION_FRAME_MS)
        self._timer.timeout.connect(self._update_animation)

    def set_pose(self, angles: list[float], waypoint_number: int) -> None:
        self._timer.stop()
        self._start_angles = list(angles)
        self._target_angles = list(angles)
        self._segment_profile = None
        self.arm.set_angles(angles)
        self.pose_label.setText(f"Waypoint {waypoint_number}")

    def animate_to(
        self,
        angles: list[float],
        speeds_dps: list[float],
        duration_ms: int,
        waypoint_number: int,
        gradient: Mapping[str, object] | None = None,
    ) -> None:
        self._start_angles = self.arm.angles
        self._target_angles = list(angles)
        self._segment_profile = None
        self._segment_duration_ms = max(MIN_TRANSITION_MS, duration_ms)
        if gradient is not None:
            duration_s, self._segment_profile = build_transition_profile(
                self._start_angles,
                self._target_angles,
                speeds_dps,
                gradient,
            )
            self._segment_duration_ms = max(
                MIN_TRANSITION_MS, math.ceil(duration_s * 1000)
            )
        self._segment_started_at = time.monotonic()
        self.pose_label.setText(f"Moving to waypoint {waypoint_number}")
        if speeds_dps:
            self.description.setText(
                "Interactive 3D kinematic approximation using the configured "
                "per-joint speeds (degrees/second). Drag to orbit and use the "
                "mouse wheel to zoom. The gripper is visually aligned to point "
                "upward at the all-zero pose; this is not a calibrated "
                "Gen3 model or a command to the robot."
            )
        self._timer.start()

    def stop_animation(self) -> None:
        self._timer.stop()

    def _update_animation(self) -> None:
        elapsed_ms = (time.monotonic() - self._segment_started_at) * 1000
        progress = min(1.0, elapsed_ms / self._segment_duration_ms)
        angles = [
            start + (target - start) * progress
            for start, target in zip(
                self._start_angles, self._target_angles, strict=True
            )
        ] if self._segment_profile is None else self._segment_profile(progress)
        self.arm.set_angles(angles)
        if progress >= 1.0:
            self._timer.stop()

    def closeEvent(self, event: QCloseEvent) -> None:
        self.stop_animation()
        super().closeEvent(event)

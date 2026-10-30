"""Qt interface for the Kinova movement application."""

from __future__ import annotations

import json
import math
import time
from collections.abc import Callable, Mapping
from functools import partial
from pathlib import Path

from PySide6.QtCore import QModelIndex, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QAction, QCloseEvent
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QHeaderView,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QMenu,
    QPushButton,
    QSpinBox,
    QStackedWidget,
    QStyledItemDelegate,
    QTableWidget,
    QTableWidgetItem,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from kinova_movement_app.preview_anim import (
    MovementAnimationDialog,
    transition_duration_ms,
)
from kinova_movement_app.connection import RobotConnection
from kinova_movement_app.playback import RobotPlaybackWorker


JOINT_NAMES = tuple(f"Joint {number}" for number in range(1, 8))
MOVEMENT_FILTER = "Kinova movement files (*.json)"
DEFAULT_JOINT_SPEED_DPS = 30.0
JOINT_SPEED_LIMITS_DPS = (80.0, 80.0, 80.0, 80.0, 70.0, 70.0, 70.0)
JOINT_ANGLE_LIMITS_DEG: tuple[float | None, ...] = (
    None,
    126.0,
    None,
    147.0,
    None,
    117.0,
    None,
)
UNLIMITED_JOINT_EDITOR_RANGE_DEG = 1_000_000_000.0
POSITION_COLUMN_START = 1
SPEED_COLUMN_START = POSITION_COLUMN_START + len(JOINT_NAMES)
GRADIENT_COLUMN = SPEED_COLUMN_START + len(JOINT_NAMES)


class TransitionGradientEditor(QWidget):
    """Edit optional acceleration and deceleration limits for one transition."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 0, 2, 0)
        layout.setSpacing(4)

        self.enabled = QCheckBox("Ramp")
        self.enabled.setToolTip(
            "Use the acceleration profile for this waypoint's transition."
        )
        layout.addWidget(self.enabled)

        self.increase = self._gradient_input("Increase speed gradient (°/s²)")
        layout.addWidget(self.increase)
        self.decrease = self._gradient_input("Decrease speed gradient (°/s²)")
        layout.addWidget(self.decrease)

        self.scope = QComboBox()
        self.scope.addItem("All joints", "all")
        self.scope.addItem("Selected joints", "selected")
        self.scope.setToolTip("Apply the gradient to all joints or selected joints.")
        layout.addWidget(self.scope)

        self.joints_button = QToolButton()
        self.joints_button.setText("Joints")
        self.joints_button.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        self.joints_menu = QMenu(self.joints_button)
        self.joint_actions: list[QAction] = []
        for joint_index, joint_name in enumerate(JOINT_NAMES, start=1):
            action = QAction(joint_name, self.joints_menu)
            action.setCheckable(True)
            action.setChecked(True)
            action.setData(joint_index)
            self.joints_menu.addAction(action)
            self.joint_actions.append(action)
        self.joints_button.setMenu(self.joints_menu)
        self.joints_button.setToolTip("Choose which joints receive the gradient.")
        layout.addWidget(self.joints_button)

        self.enabled.toggled.connect(self._update_enabled_state)
        self.scope.currentIndexChanged.connect(self._update_enabled_state)
        self._update_enabled_state()

    @staticmethod
    def _gradient_input(tooltip: str) -> QDoubleSpinBox:
        field = QDoubleSpinBox()
        field.setRange(0.01, 10000.0)
        field.setDecimals(2)
        field.setSingleStep(5.0)
        field.setValue(30.0)
        field.setSuffix("°/s²")
        field.setToolTip(tooltip)
        return field

    def _update_enabled_state(self, *_: object) -> None:
        enabled = self.enabled.isChecked()
        self.increase.setEnabled(enabled)
        self.decrease.setEnabled(enabled)
        self.scope.setEnabled(enabled)
        self.joints_button.setEnabled(
            enabled and self.scope.currentData() == "selected"
        )

    def gradient_data(self) -> dict[str, object] | None:
        if not self.enabled.isChecked():
            return None
        if self.scope.currentData() == "all":
            joints = list(range(1, len(JOINT_NAMES) + 1))
        else:
            joints = [
                int(action.data())
                for action in self.joint_actions
                if action.isChecked()
            ]
            if not joints:
                raise ValueError("Select at least one joint for the transition ramp.")
        return {
            "increase_dps2": self.increase.value(),
            "decrease_dps2": self.decrease.value(),
            "joints": joints,
        }

    def set_gradient_data(self, gradient: Mapping[str, object] | None) -> None:
        if gradient is None:
            self.enabled.setChecked(False)
            return
        self.enabled.setChecked(True)
        self.increase.setValue(float(gradient["increase_dps2"]))
        self.decrease.setValue(float(gradient["decrease_dps2"]))
        selected = {int(joint) for joint in gradient["joints"]}
        for action in self.joint_actions:
            action.setChecked(int(action.data()) in selected)
        self.scope.setCurrentIndex(
            self.scope.findData(
                "all"
                if selected == set(range(1, len(JOINT_NAMES) + 1))
                else "selected"
            )
        )


class JointAngleDelegate(QStyledItemDelegate):
    """Provide numeric editors that enforce finite Gen3 joint limits."""

    def createEditor(
        self, parent: QWidget, option: object, index: QModelIndex
    ) -> QDoubleSpinBox:
        del option
        joint_index = index.column() - POSITION_COLUMN_START
        limit = JOINT_ANGLE_LIMITS_DEG[joint_index]
        editor = QDoubleSpinBox(parent)
        if limit is None:
            editor.setRange(
                -UNLIMITED_JOINT_EDITOR_RANGE_DEG,
                UNLIMITED_JOINT_EDITOR_RANGE_DEG,
            )
        else:
            editor.setRange(-limit, limit)
        editor.setDecimals(2)
        editor.setSingleStep(1.0)
        return editor


class JointSpeedDelegate(QStyledItemDelegate):
    """Provide bounded numeric editors for per-joint transition speeds."""

    def createEditor(
        self, parent: QWidget, option: object, index: QModelIndex
    ) -> QDoubleSpinBox:
        del option
        joint_index = index.column() - SPEED_COLUMN_START
        editor = QDoubleSpinBox(parent)
        editor.setRange(0.01, JOINT_SPEED_LIMITS_DPS[joint_index])
        editor.setDecimals(2)
        editor.setSingleStep(1.0)
        return editor


class ConnectionWorker(QThread):
    """Run blocking Kortex connection and disconnection calls off the UI thread."""

    connection_opened = Signal(object)
    connection_closed = Signal()
    failed = Signal(str, str)

    def __init__(
        self,
        operation: str,
        *,
        ip: str = "",
        username: str = "",
        password: str = "",
        connection: RobotConnection | None = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.operation = operation
        self.ip = ip
        self.username = username
        self.password = password
        self.connection = connection

    def run(self) -> None:
        if self.operation == "connect":
            try:
                connection = RobotConnection.open(
                    self.ip, self.username, self.password
                )
            except Exception as error:
                self.failed.emit(self.operation, str(error))
            else:
                self.connection_opened.emit(connection)
            return

        if self.operation == "disconnect" and self.connection is not None:
            try:
                self.connection.close()
            except Exception as error:
                self.failed.emit(self.operation, str(error))
            else:
                self.connection_closed.emit()
            return

        self.failed.emit(
            self.operation,
            f"Unsupported connection operation: {self.operation}",
        )


class MainWindow(QMainWindow):
    """Main window with home, movement creation, and playback pages."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Kinova Gen3 controller")
        self.resize(1120, 760)
        self.setMinimumSize(900, 620)
        self._preview_timer = QTimer(self)
        self._preview_timer.setSingleShot(True)
        self._preview_timer.timeout.connect(self._advance_preview)
        self._preview_table: QTableWidget | None = None
        self._preview_positions: list[list[float]] = []
        self._preview_speeds: list[list[float]] = []
        self._preview_gradients: list[dict[str, object] | None] = []
        self._preview_row = 0
        self._preview_completed_plays = 0
        self._preview_mode = "once"
        self._preview_play_count = 1
        self._preview_deadline: float | None = None
        self._preview_button: QPushButton | None = None
        self._preview_animation: MovementAnimationDialog | None = None
        self._robot_connection: RobotConnection | None = None
        self._connection_worker: ConnectionWorker | None = None
        self._hardware_worker: RobotPlaybackWorker | None = None
        self._connection_state = "disconnected"
        self._robot_ip = ""
        self._close_when_disconnected = False
        self._close_when_playback_stops = False
        self._loaded_movement: dict[str, object] | None = None
        self.setStyleSheet(
            """
            QWidget {
                color: #167d8d;
                font-family: "Segoe UI";
                font-size: 14px;
            }
            QMainWindow, #content, #page {
                background: #101722;
            }
            #sidebar {
                background: #151f2d;
                border-right: 1px solid #263449;
            }
            #brand {
                color: #f3f7fc;
                font-size: 19px;
                font-weight: 700;
            }
            #muted {
                color: #9cabc0;
            }
            #pageTitle {
                color: #f3f7fc;
                font-size: 28px;
                font-weight: 700;
            }
            #panel, #actionCard {
                background: #192536;
                border: 1px solid #2a3a50;
                border-radius: 12px;
            }
            #actionCard {
                min-height: 150px;
            }
            QPushButton {
                background: #25354a;
                border: 1px solid #354a63;
                border-radius: 8px;
                color: #e7edf5;
                padding: 10px 14px;
                text-align: left;
            }
            QPushButton:hover {
                background: #30455f;
            }
            QPushButton:disabled {
                color: #77869a;
                background: #202b39;
                border-color: #2b394a;
            }
            QPushButton[active="true"], QPushButton[primary="true"] {
                background: #167d8d;
                border-color: #2199a7;
                color: white;
                font-weight: 600;
            }
            QPushButton[primary="true"]:hover {
                background: #1b92a1;
            }
            QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox {
                background: #101722;
                border: 1px solid #35465c;
                border-radius: 6px;
                padding: 8px;
                selection-background-color: #167d8d;
            }
            QTableWidget {
                background: #131d2a;
                alternate-background-color: #192536;
                border: 1px solid #2a3a50;
                border-radius: 8px;
                gridline-color: #2a3a50;
            }
            QHeaderView::section {
                background: #202d3f;
                border: none;
                padding: 8px;
                color: #b9c7d8;
                font-weight: 600;
            }
            """
        )

        root = QWidget()
        root_layout = QHBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        root_layout.addWidget(self._build_sidebar())

        self.pages = QStackedWidget()
        self.pages.setObjectName("content")
        self.pages.addWidget(self._build_home_page())
        self.pages.addWidget(self._build_record_page())
        self.pages.addWidget(self._build_playback_page())
        root_layout.addWidget(self.pages, 1)
        self.setCentralWidget(root)
        self._show_page(0)
        self._set_connection_ui_state("disconnected", "Not connected")

    def _build_sidebar(self) -> QWidget:
        sidebar = QWidget()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(220)
        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(18, 24, 18, 20)
        layout.setSpacing(12)

        brand = QLabel("KINOVA\nController")
        brand.setObjectName("brand")
        layout.addWidget(brand)
        subtitle = QLabel("GEN3 WORKSPACE")
        subtitle.setObjectName("muted")
        layout.addWidget(subtitle)
        layout.addSpacing(24)

        self.nav_buttons: list[QPushButton] = []
        for label, page_index in (
            ("Home", 0),
            ("Create movement", 1),
            ("Play movement", 2),
        ):
            button = QPushButton(label)
            button.setCursor(Qt.CursorShape.PointingHandCursor)
            button.clicked.connect(partial(self._show_page, page_index))
            self.nav_buttons.append(button)
            layout.addWidget(button)

        layout.addStretch()
        status_title = QLabel("ROBOT STATUS")
        status_title.setObjectName("muted")
        layout.addWidget(status_title)
        status_row = QHBoxLayout()
        status_row.setContentsMargins(0, 0, 0, 0)
        self.connection_indicator = QLabel()
        self.connection_indicator.setFixedSize(12, 12)
        status_row.addWidget(self.connection_indicator)
        self.connection_status_label = QLabel("Not connected")
        self.connection_status_label.setObjectName("muted")
        status_row.addWidget(self.connection_status_label)
        status_row.addStretch()
        layout.addLayout(status_row)
        return sidebar

    def _build_home_page(self) -> QWidget:
        page = self._new_page()
        layout = page.layout()
        layout.addWidget(self._heading("Welcome to Movement Studio"))
        layout.addWidget(
            self._muted_label(
                "Create and organize joint-space movement files for a Kinova Gen3."
            )
        )
        layout.addSpacing(10)

        status = self._panel()
        status_layout = QVBoxLayout(status)
        status_layout.addWidget(QLabel("Robot connection"))
        status_layout.addWidget(
            self._muted_label(
                "Connect to a Kinova Gen3 over TCP. Your credentials are used for "
                "this session and are not saved."
            )
        )
        connection_form = QFormLayout()
        self.robot_ip_input = QLineEdit("192.168.1.10")
        self.robot_ip_input.setPlaceholderText("e.g. 192.168.1.10")
        connection_form.addRow("Robot IP address", self.robot_ip_input)
        self.robot_username_input = QLineEdit("admin")
        self.robot_username_input.setPlaceholderText("Username")
        connection_form.addRow("Username", self.robot_username_input)
        self.robot_password_input = QLineEdit()
        self.robot_password_input.setEchoMode(QLineEdit.EchoMode.Password)
        self.robot_password_input.setPlaceholderText("Password")
        connection_form.addRow("Password", self.robot_password_input)
        status_layout.addLayout(connection_form)
        connection_actions = QHBoxLayout()
        self.connect_button = QPushButton("Connect")
        self.connect_button.setProperty("primary", True)
        self.connect_button.clicked.connect(self._connect_to_robot)
        connection_actions.addWidget(self.connect_button)
        self.close_connection_button = QPushButton("Close connection")
        self.close_connection_button.setEnabled(False)
        self.close_connection_button.clicked.connect(self._close_robot_connection)
        connection_actions.addWidget(self.close_connection_button)
        connection_actions.addStretch()
        status_layout.addLayout(connection_actions)
        self.home_connection_status = self._muted_label("Not connected")
        status_layout.addWidget(self.home_connection_status)
        layout.addWidget(status)
        layout.addSpacing(8)

        cards = QHBoxLayout()
        cards.setSpacing(16)
        cards.addWidget(
            self._action_card(
                "Create a movement",
                "Enter joint angles to build and save a sequence of waypoints.",
                "Open movement editor",
                lambda: self._show_page(1),
            )
        )
        cards.addWidget(
            self._action_card(
                "Play a movement",
                "Open a saved movement file and review its waypoints.",
                "Browse movement files",
                lambda: self._show_page(2),
            )
        )
        layout.addLayout(cards)
        layout.addStretch()
        return page

    def _build_record_page(self) -> QWidget:
        page = self._new_page()
        layout = page.layout()
        layout.addWidget(self._heading("Create a movement"))
        layout.addWidget(
            self._muted_label(
                "Add joint-angle waypoints, then save them as a Kinova movement file."
            )
        )

        details = self._panel()
        details_layout = QVBoxLayout(details)
        details_layout.addWidget(QLabel("Movement name"))
        self.movement_name = QLineEdit()
        self.movement_name.setPlaceholderText("e.g. Pick and place")
        details_layout.addWidget(self.movement_name)
        layout.addWidget(details)

        editor = self._panel()
        editor_layout = QVBoxLayout(editor)
        editor_layout.addWidget(
            self._muted_label(
                "Edit waypoint angles and transition speeds. The final row sets "
                "the speed for looping back to the first position."
            )
        )
        inputs = QHBoxLayout()
        self.joint_inputs: list[QDoubleSpinBox] = []
        for joint_name in JOINT_NAMES:
            column = QVBoxLayout()
            column.addWidget(self._muted_label(joint_name))
            spin_box = QDoubleSpinBox()
            joint_index = len(self.joint_inputs)
            angle_limit = JOINT_ANGLE_LIMITS_DEG[joint_index]
            if angle_limit is None:
                spin_box.setRange(
                    -UNLIMITED_JOINT_EDITOR_RANGE_DEG,
                    UNLIMITED_JOINT_EDITOR_RANGE_DEG,
                )
            else:
                spin_box.setRange(-angle_limit, angle_limit)
            spin_box.setDecimals(2)
            spin_box.setSingleStep(1.0)
            spin_box.setSuffix("°")
            self.joint_inputs.append(spin_box)
            column.addWidget(spin_box)
            inputs.addLayout(column)
        editor_layout.addLayout(inputs)

        editor_actions = QHBoxLayout()
        add_button = QPushButton("Add waypoint")
        add_button.setProperty("primary", True)
        add_button.clicked.connect(self._add_waypoint)
        editor_actions.addWidget(add_button)
        capture_button = QPushButton("Capture from robot")
        capture_button.setEnabled(False)
        capture_button.setToolTip("Live capture is not connected yet.")
        editor_actions.addWidget(capture_button)
        editor_actions.addStretch()
        editor_layout.addLayout(editor_actions)
        editor_layout.addWidget(
            self._muted_label(
                "Live capture is unavailable. Transition ramps are saved with the "
                "movement and applied during robot playback."
            )
        )
        layout.addWidget(editor)

        self.record_table = self._make_waypoint_table(editable=True)
        for joint_index in range(len(JOINT_NAMES)):
            self.record_table.setItemDelegateForColumn(
                POSITION_COLUMN_START + joint_index,
                JointAngleDelegate(self.record_table),
            )
        for joint_index in range(1, len(JOINT_NAMES)):
            self.record_table.setItemDelegateForColumn(
                SPEED_COLUMN_START + joint_index,
                JointSpeedDelegate(self.record_table),
            )
        self.record_table.setItemDelegateForColumn(
            SPEED_COLUMN_START, JointSpeedDelegate(self.record_table)
        )
        self.record_table.setToolTip(
            "Each row's speed controls the transition to the next position. "
            "The final row controls the loop-back transition. Optional ramp controls "
            "in each row shape acceleration and deceleration for that transition."
        )
        layout.addWidget(self.record_table, 1)
        waypoint_actions = QHBoxLayout()
        self.delete_waypoint_button = QPushButton("Delete selected position")
        self.delete_waypoint_button.setEnabled(False)
        self.delete_waypoint_button.clicked.connect(self._delete_selected_waypoint)
        self.record_table.currentCellChanged.connect(self._on_record_selection_changed)
        waypoint_actions.addWidget(self.delete_waypoint_button)
        waypoint_actions.addStretch()
        layout.addLayout(waypoint_actions)
        (
            self.record_loop_mode,
            self.record_play_count,
            self.record_time_limit,
            self.record_play_button,
        ) = self._add_preview_controls(layout, self.record_table)
        save_button = QPushButton("Save movement file")
        save_button.setProperty("primary", True)
        save_button.clicked.connect(self._save_movement)
        layout.addWidget(save_button, alignment=Qt.AlignmentFlag.AlignRight)
        return page

    def _build_playback_page(self) -> QWidget:
        page = self._new_page()
        layout = page.layout()
        layout.addWidget(self._heading("Play a movement"))
        layout.addWidget(
            self._muted_label(
                "Load a saved movement file to review its joint-angle waypoints."
            )
        )

        file_row = QHBoxLayout()
        self.loaded_file_label = self._muted_label("No movement file loaded")
        file_row.addWidget(self.loaded_file_label, 1)
        browse_button = QPushButton("Open movement file")
        browse_button.clicked.connect(self._open_movement)
        file_row.addWidget(browse_button)
        layout.addLayout(file_row)

        self.playback_table = self._make_waypoint_table()
        self.playback_table.setToolTip(
            "Each row's speed controls the transition to the next position. "
            "The final row's speed controls a loop from the last position to the first."
        )
        layout.addWidget(self.playback_table, 1)
        (
            self.play_loop_mode,
            self.play_play_count,
            self.play_time_limit,
            self.play_button,
        ) = self._add_preview_controls(layout, self.playback_table)
        (
            self.robot_loop_mode,
            self.robot_play_count,
            self.robot_time_limit,
            self.robot_play_button,
        ) = self._add_preview_controls(
            layout,
            self.playback_table,
            title="Robot playback:",
            button_text="Start robot playback",
            action=self._start_robot_playback,
        )
        self.robot_stop_button = QPushButton("Stop robot")
        self.robot_stop_button.setEnabled(False)
        self.robot_stop_button.clicked.connect(self._stop_robot_playback)
        layout.addWidget(self.robot_stop_button, alignment=Qt.AlignmentFlag.AlignRight)
        return page

    def _new_page(self) -> QWidget:
        page = QWidget()
        page.setObjectName("page")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(34, 30, 34, 30)
        layout.setSpacing(16)
        return page

    @staticmethod
    def _heading(text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("pageTitle")
        return label

    @staticmethod
    def _muted_label(text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("muted")
        label.setWordWrap(True)
        return label

    @staticmethod
    def _panel() -> QFrame:
        panel = QFrame()
        panel.setObjectName("panel")
        return panel

    def _action_card(
        self, title: str, description: str, button_text: str, action: Callable[[], None]
    ) -> QFrame:
        card = self._panel()
        card.setObjectName("actionCard")
        layout = QVBoxLayout(card)
        title_label = QLabel(title)
        title_label.setStyleSheet("font-size: 18px; font-weight: 600;")
        layout.addWidget(title_label)
        layout.addWidget(self._muted_label(description))
        button = QPushButton(button_text)
        button.setProperty("primary", True)
        button.clicked.connect(action)
        layout.addWidget(button)
        return card

    @staticmethod
    def _make_waypoint_table(editable: bool = False) -> QTableWidget:
        headers = (
            ["Waypoint"]
            + [f"J{number} (°)" for number in range(1, len(JOINT_NAMES) + 1)]
            + [f"J{number} speed (°/s)" for number in range(1, len(JOINT_NAMES) + 1)]
            + ["Transition ramp"]
        )
        table = QTableWidget(0, len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.setAlternatingRowColors(True)
        table.setEditTriggers(
            QTableWidget.EditTrigger.DoubleClicked
            | QTableWidget.EditTrigger.EditKeyPressed
            if editable
            else QTableWidget.EditTrigger.NoEditTriggers
        )
        table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        table.verticalHeader().setVisible(False)
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.ResizeToContents
        )
        return table

    def _add_preview_controls(
        self,
        layout: QVBoxLayout,
        table: QTableWidget,
        *,
        title: str = "Preview:",
        button_text: str = "Play preview",
        action: Callable[[], None] | None = None,
    ) -> tuple[QComboBox, QSpinBox, QDoubleSpinBox, QPushButton]:
        controls = QHBoxLayout()
        controls.addWidget(QLabel(title))

        loop_mode = QComboBox()
        loop_mode.addItem("Once", "once")
        loop_mode.addItem("Repeat until stopped", "until_stopped")
        loop_mode.addItem("Total play count", "play_count")
        loop_mode.addItem("Time limit", "time_limit")
        controls.addWidget(loop_mode)

        count_label = QLabel("Total plays:")
        play_count = QSpinBox()
        play_count.setRange(1, 100000)
        play_count.setValue(2)
        controls.addWidget(count_label)
        controls.addWidget(play_count)

        time_label = QLabel("Limit (minutes):")
        time_limit = QDoubleSpinBox()
        time_limit.setRange(0.1, 1440.0)
        time_limit.setDecimals(1)
        time_limit.setValue(1.0)
        controls.addWidget(time_label)
        controls.addWidget(time_limit)

        def update_loop_fields() -> None:
            count_visible = loop_mode.currentData() == "play_count"
            time_visible = loop_mode.currentData() == "time_limit"
            count_label.setVisible(count_visible)
            play_count.setVisible(count_visible)
            time_label.setVisible(time_visible)
            time_limit.setVisible(time_visible)

        loop_mode.currentIndexChanged.connect(update_loop_fields)
        update_loop_fields()

        button = QPushButton(button_text)
        button.setProperty("primary", True)
        if action is None:
            button.clicked.connect(
                partial(
                    self._toggle_preview,
                    table,
                    loop_mode,
                    play_count,
                    time_limit,
                    button,
                )
            )
        else:
            button.clicked.connect(action)
        controls.addStretch()
        controls.addWidget(button)
        layout.addLayout(controls)
        return loop_mode, play_count, time_limit, button

    def _show_page(self, page_index: int) -> None:
        self.pages.setCurrentIndex(page_index)
        for index, button in enumerate(self.nav_buttons):
            button.setProperty("active", index == page_index)
            button.style().unpolish(button)
            button.style().polish(button)

    def _set_connection_ui_state(self, state: str, message: str) -> None:
        self._connection_state = state
        is_disconnected = state == "disconnected"
        is_connected = state == "connected"
        self.robot_ip_input.setEnabled(is_disconnected)
        self.robot_username_input.setEnabled(is_disconnected)
        self.robot_password_input.setEnabled(is_disconnected)
        self.connect_button.setEnabled(is_disconnected)
        self.close_connection_button.setEnabled(is_connected)
        self.home_connection_status.setText(message)

        sidebar_messages = {
            "disconnected": "Not connected",
            "connecting": "Connecting",
            "connected": "Connected",
            "disconnecting": "Disconnecting",
        }
        colors = {
            "disconnected": "#ef5b65",
            "connecting": "#f2b84b",
            "connected": "#35c98f",
            "disconnecting": "#f2b84b",
        }
        self.connection_status_label.setText(sidebar_messages[state])
        self.connection_indicator.setStyleSheet(
            f"background-color: {colors[state]}; border-radius: 6px;"
        )

    def _start_connection_worker(self, worker: ConnectionWorker) -> None:
        self._connection_worker = worker
        worker.connection_opened.connect(self._on_connection_opened)
        worker.connection_closed.connect(self._on_connection_closed)
        worker.failed.connect(self._on_connection_failed)
        worker.finished.connect(self._on_connection_worker_finished)
        worker.start()

    def _connect_to_robot(self) -> None:
        if self._connection_state != "disconnected":
            return
        ip = self.robot_ip_input.text().strip()
        username = self.robot_username_input.text().strip()
        password = self.robot_password_input.text()
        if not ip or not username or not password:
            QMessageBox.warning(
                self,
                "Connection details required",
                "Enter the robot IP address, username, and password.",
            )
            return

        self._robot_ip = ip
        self._set_connection_ui_state("connecting", f"Connecting to {ip}...")
        worker = ConnectionWorker(
            "connect",
            ip=ip,
            username=username,
            password=password,
            parent=self,
        )
        self.robot_password_input.clear()
        self._start_connection_worker(worker)

    def _close_robot_connection(self) -> None:
        if (
            self._connection_state != "connected"
            or self._robot_connection is None
        ):
            return
        self._set_connection_ui_state("disconnecting", "Closing robot connection...")
        worker = ConnectionWorker(
            "disconnect", connection=self._robot_connection, parent=self
        )
        self._start_connection_worker(worker)

    def _on_connection_opened(self, connection: object) -> None:
        if not isinstance(connection, RobotConnection):
            raise TypeError("The connection worker returned an invalid connection.")
        self._robot_connection = connection
        self._set_connection_ui_state(
            "connected", f"Connected to {self._robot_ip or self.robot_ip_input.text()}."
        )

    def _on_connection_closed(self) -> None:
        self._robot_connection = None
        self._set_connection_ui_state("disconnected", "Connection closed.")
        self._close_after_disconnect_if_requested()

    def _on_connection_failed(self, operation: str, error: str) -> None:
        if operation == "disconnect":
            self._robot_connection = None
            self._close_when_disconnected = False
            self._set_connection_ui_state(
                "disconnected", "Disconnected with a cleanup error."
            )
            QMessageBox.warning(
                self, "Connection closed with an error", error
            )
            return

        self._set_connection_ui_state(
            "disconnected", "Could not connect. Check the robot and login details."
        )
        QMessageBox.critical(self, "Robot connection failed", error)

    def _on_connection_worker_finished(self) -> None:
        worker = self._connection_worker
        self._connection_worker = None
        if worker is not None:
            worker.deleteLater()
        self._close_after_disconnect_if_requested()

    def _close_after_disconnect_if_requested(self) -> None:
        if (
            self._close_when_disconnected
            and self._connection_worker is None
            and self._robot_connection is None
            and self._connection_state == "disconnected"
        ):
            self._close_when_disconnected = False
            self.close()

    def closeEvent(self, event: QCloseEvent) -> None:
        if self._connection_worker is not None:
            event.ignore()
            QMessageBox.information(
                self, "Connection in progress", "Wait for the connection operation to finish."
            )
            return
        if self._hardware_worker is not None:
            self._close_when_playback_stops = True
            self._hardware_worker.request_stop()
            event.ignore()
            return
        if self._robot_connection is not None:
            self._close_when_disconnected = True
            self._close_robot_connection()
            event.ignore()
            return
        event.accept()

    def _start_robot_playback(self) -> None:
        if self._hardware_worker is not None:
            return
        if self._robot_connection is None or self._connection_state != "connected":
            QMessageBox.warning(
                self, "Robot not connected", "Connect to the robot before playback."
            )
            return
        if self._loaded_movement is None:
            QMessageBox.warning(
                self,
                "No movement loaded",
                "Open a saved movement file before starting robot playback.",
            )
            return
        if len(self._loaded_movement["positions"]) < 2:
            QMessageBox.warning(
                self,
                "Not enough positions",
                "Robot playback requires at least two positions.",
            )
            return

        answer = QMessageBox.warning(
            self,
            "Confirm robot movement",
            "The connected robot will move through this trajectory. Clear the "
            "arm's workspace and keep the robot's emergency stop accessible. "
            "Continue only if the movement is safe for your setup.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return

        self._stop_preview()
        worker = RobotPlaybackWorker(
            self._robot_connection,
            self._loaded_movement,
            str(self.robot_loop_mode.currentData()),
            self.robot_play_count.value(),
            self.robot_time_limit.value(),
            parent=self,
        )
        self._hardware_worker = worker
        worker.progress.connect(self._on_robot_playback_progress)
        worker.completed.connect(self._on_robot_playback_completed)
        worker.failed.connect(self._on_robot_playback_failed)
        worker.finished.connect(self._on_robot_playback_worker_finished)
        self.robot_play_button.setEnabled(False)
        self.robot_stop_button.setEnabled(True)
        self.close_connection_button.setEnabled(False)
        self.home_connection_status.setText("Robot playback in progress.")
        worker.start()

    def _stop_robot_playback(self) -> None:
        if self._hardware_worker is None:
            return
        self.robot_stop_button.setEnabled(False)
        self.home_connection_status.setText("Stopping robot playback...")
        self._hardware_worker.request_stop()

    def _on_robot_playback_progress(self, row: int, completed_cycles: int) -> None:
        if row < self.playback_table.rowCount():
            self.playback_table.selectRow(row)
        self.home_connection_status.setText(
            f"Robot playback: waypoint {row + 1}, cycle {completed_cycles + 1}."
        )

    def _on_robot_playback_completed(self, finished: bool) -> None:
        self.robot_stop_button.setEnabled(False)
        self.robot_play_button.setEnabled(True)
        if self._robot_connection is not None:
            self.close_connection_button.setEnabled(True)
        self.home_connection_status.setText(
            "Robot playback finished."
            if finished
            else "Robot playback stopped."
        )

    def _on_robot_playback_failed(self, error: str) -> None:
        self._close_when_playback_stops = False
        self.robot_stop_button.setEnabled(False)
        self.robot_play_button.setEnabled(True)
        if self._robot_connection is not None:
            self.close_connection_button.setEnabled(True)
        self.home_connection_status.setText("Robot playback failed.")
        QMessageBox.critical(self, "Robot playback failed", error)

    def _on_robot_playback_worker_finished(self) -> None:
        worker = self._hardware_worker
        self._hardware_worker = None
        if worker is not None:
            worker.deleteLater()
        if self._close_when_playback_stops:
            self._close_when_playback_stops = False
            self.close()

    def _add_waypoint(self) -> None:
        row = self.record_table.rowCount()
        self.record_table.insertRow(row)
        number_item = QTableWidgetItem(str(row + 1))
        number_item.setFlags(number_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
        number_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.record_table.setItem(row, 0, number_item)
        for joint_index, spin_box in enumerate(self.joint_inputs):
            self._set_table_item(
                self.record_table,
                row,
                POSITION_COLUMN_START + joint_index,
                f"{spin_box.value():.2f}",
            )
        for joint_index in range(len(JOINT_NAMES)):
            self._set_table_item(
                self.record_table,
                row,
                SPEED_COLUMN_START + joint_index,
                f"{DEFAULT_JOINT_SPEED_DPS:.2f}",
            )
        self.record_table.setCellWidget(
            row, GRADIENT_COLUMN, TransitionGradientEditor(self.record_table)
        )
        self.record_table.selectRow(row)
        self.delete_waypoint_button.setEnabled(True)

    def _delete_selected_waypoint(self) -> None:
        row = self.record_table.currentRow()
        if row < 0:
            return
        if self._preview_table is self.record_table:
            self._stop_preview()

        self.record_table.removeRow(row)
        for remaining_row in range(self.record_table.rowCount()):
            number_item = self.record_table.item(remaining_row, 0)
            if number_item is not None:
                number_item.setText(str(remaining_row + 1))

        if self.record_table.rowCount():
            last_row = self.record_table.rowCount() - 1
            selected_row = min(row, last_row)
            self.record_table.selectRow(selected_row)
            self.delete_waypoint_button.setEnabled(True)
        else:
            self.delete_waypoint_button.setEnabled(False)

    def _on_record_selection_changed(self, current_row: int) -> None:
        self.delete_waypoint_button.setEnabled(current_row >= 0)

    @staticmethod
    def _set_table_item(
        table: QTableWidget, row: int, column: int, text: str
    ) -> None:
        item = QTableWidgetItem(text)
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        table.setItem(row, column, item)

    @staticmethod
    def _read_waypoint_data(
        table: QTableWidget,
    ) -> tuple[list[list[float]], list[list[float]]]:
        if table.rowCount() == 0:
            raise ValueError("Add at least one waypoint before playing or saving.")

        positions: list[list[float]] = []
        speeds: list[list[float]] = []
        for row in range(table.rowCount()):
            position: list[float] = []
            for joint_index in range(len(JOINT_NAMES)):
                item = table.item(row, POSITION_COLUMN_START + joint_index)
                try:
                    angle = float(item.text()) if item is not None else math.nan
                except ValueError:
                    angle = math.nan
                if not MainWindow._is_valid_joint_angle(angle, joint_index):
                    limit = JOINT_ANGLE_LIMITS_DEG[joint_index]
                    limit_message = (
                        "any finite angle"
                        if limit is None
                        else f"between {-limit:g}° and {limit:g}°"
                    )
                    raise ValueError(
                        f"Waypoint {row + 1}, Joint {joint_index + 1} "
                        f"must be {limit_message}."
                    )
                position.append(angle)
            positions.append(position)

            transition_speeds: list[float] = []
            for joint_index, speed_limit in enumerate(JOINT_SPEED_LIMITS_DPS):
                item = table.item(row, SPEED_COLUMN_START + joint_index)
                try:
                    speed = float(item.text()) if item is not None else math.nan
                except ValueError:
                    speed = math.nan
                if not math.isfinite(speed) or not 0 < speed <= speed_limit:
                    raise ValueError(
                        f"Position {row + 1}, Joint {joint_index + 1} speed "
                        f"must be greater than 0 and no more than {speed_limit:g}°/s."
                    )
                transition_speeds.append(speed)
            speeds.append(transition_speeds)
        return positions, speeds

    @staticmethod
    def _read_transition_gradients(
        table: QTableWidget,
    ) -> list[dict[str, object] | None]:
        gradients: list[dict[str, object] | None] = []
        for row in range(table.rowCount()):
            editor = table.cellWidget(row, GRADIENT_COLUMN)
            if not isinstance(editor, TransitionGradientEditor):
                gradients.append(None)
                continue
            gradients.append(editor.gradient_data())
        return gradients

    def _save_movement(self) -> None:
        name = self.movement_name.text().strip()
        if not name:
            QMessageBox.warning(self, "Movement name required", "Enter a movement name.")
            return
        try:
            waypoints, transition_speeds = self._read_waypoint_data(self.record_table)
            transition_gradients = self._read_transition_gradients(self.record_table)
        except ValueError as error:
            QMessageBox.warning(self, "Invalid movement", str(error))
            return

        path_text, _ = QFileDialog.getSaveFileName(
            self, "Save movement file", f"{name}.json", MOVEMENT_FILTER
        )
        if not path_text:
            return

        movement = {
            "name": name,
            "robot": "Kinova Gen3",
            "units": "degrees",
            "joint_names": list(JOINT_NAMES),
            "positions": waypoints,
            "transition_speeds_dps": transition_speeds,
            "transition_gradients": transition_gradients,
        }
        movement_path = Path(path_text)
        if movement_path.suffix.lower() != ".json":
            movement_path = movement_path.with_suffix(".json")
        try:
            movement_path.write_text(
                json.dumps(movement, indent=2) + "\n", encoding="utf-8"
            )
        except OSError as error:
            QMessageBox.critical(
                self,
                "Could not save movement",
                f"Could not write the file:\n{error}",
            )
            return
        self._populate_waypoint_table(
            self.playback_table,
            waypoints,
            transition_speeds,
            transition_gradients,
        )
        self.loaded_file_label.setText(f"{name} — {movement_path}")
        self._loaded_movement = movement
        QMessageBox.information(
            self, "Movement saved", f"Saved movement to:\n{movement_path}"
        )

    def _open_movement(self) -> None:
        path_text, _ = QFileDialog.getOpenFileName(
            self, "Open movement file", str(Path.home()), MOVEMENT_FILTER
        )
        if not path_text:
            return
        try:
            movement = json.loads(Path(path_text).read_text(encoding="utf-8"))
            positions, transition_speeds = self._validate_movement(movement)
            transition_gradients = self._validate_transition_gradients(
                movement.get("transition_gradients"), len(positions)
            )
        except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
            QMessageBox.critical(
                self, "Could not open movement", f"Could not load the file:\n{error}"
            )
            return

        self._stop_preview()
        self._populate_waypoint_table(
            self.playback_table,
            positions,
            transition_speeds,
            transition_gradients,
        )

        movement_name = movement["name"]
        self.loaded_file_label.setText(f"{movement_name} — {path_text}")
        self._loaded_movement = {
            "name": movement_name,
            "joint_names": list(JOINT_NAMES),
            "positions": positions,
            "transition_speeds_dps": transition_speeds,
            "transition_gradients": transition_gradients,
        }

    def _populate_waypoint_table(
        self,
        table: QTableWidget,
        positions: list[list[float]],
        transition_speeds: list[list[float]],
        transition_gradients: list[dict[str, object] | None] | None = None,
    ) -> None:
        if transition_gradients is None:
            transition_gradients = [None] * len(positions)
        table.setRowCount(0)
        for row, position in enumerate(positions):
            table.insertRow(row)
            number_item = QTableWidgetItem(str(row + 1))
            number_item.setFlags(number_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            number_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            table.setItem(row, 0, number_item)
            for joint_index, angle in enumerate(position):
                self._set_table_item(
                    table,
                    row,
                    POSITION_COLUMN_START + joint_index,
                    f"{angle:.2f}",
                )
                speed = transition_speeds[row][joint_index]
                speed_item = QTableWidgetItem(f"{speed:.2f}")
                speed_item.setFlags(speed_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                speed_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                table.setItem(row, SPEED_COLUMN_START + joint_index, speed_item)
            gradient = transition_gradients[row]
            gradient_text = (
                "Off"
                if gradient is None
                else (
                    f"+{gradient['increase_dps2']} / "
                    f"-{gradient['decrease_dps2']} °/s² "
                    f"(Joints {', '.join(map(str, gradient['joints']))})"
                )
            )
            self._set_table_item(table, row, GRADIENT_COLUMN, gradient_text)

    @staticmethod
    def _validate_movement(
        movement: object,
    ) -> tuple[list[list[float]], list[list[float]]]:
        if not isinstance(movement, dict):
            raise ValueError("The movement file must contain a JSON object.")
        if movement.get("joint_names") != list(JOINT_NAMES):
            raise ValueError("The file must contain the seven Kinova Gen3 joint names.")
        name = movement.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError("The movement file is missing a valid name.")
        positions = movement.get("positions")
        if not isinstance(positions, list) or not positions:
            raise ValueError("The movement file must contain at least one position.")

        validated: list[list[float]] = []
        for index, position in enumerate(positions, start=1):
            if (
                not isinstance(position, list)
                or len(position) != len(JOINT_NAMES)
                or any(
                    not MainWindow._is_valid_joint_angle(angle, joint_index)
                    for joint_index, angle in enumerate(position)
                )
            ):
                raise ValueError(
                    f"Waypoint {index} must contain seven angles within the "
                    "Gen3 Ultra limits (Joints 1, 3, 5, 7 unrestricted; "
                    "Joint 2 ±126°, Joint 4 ±147°, Joint 6 ±117°)."
                )
            validated.append([float(angle) for angle in position])
        raw_speeds = movement.get("transition_speeds_dps")
        if raw_speeds is None:
            validated_speeds = [
                [DEFAULT_JOINT_SPEED_DPS] * len(JOINT_NAMES)
                for _ in range(len(validated))
            ]
        else:
            if not isinstance(raw_speeds, list):
                raise ValueError(
                    "The movement file transition_speeds_dps value must be a list."
                )
            if len(raw_speeds) == len(validated) - 1:
                raw_speeds = list(raw_speeds) + [
                    [DEFAULT_JOINT_SPEED_DPS] * len(JOINT_NAMES)
                ]
            if len(raw_speeds) != len(validated):
                raise ValueError(
                    "The movement file must contain one speed row per position."
                )
            validated_speeds = []
            for index, speed_row in enumerate(raw_speeds, start=1):
                if (
                    not isinstance(speed_row, list)
                    or len(speed_row) != len(JOINT_NAMES)
                    or any(
                        not MainWindow._is_valid_joint_speed(speed, joint_index)
                        for joint_index, speed in enumerate(speed_row)
                    )
                ):
                    raise ValueError(
                        f"Position {index} must contain seven speeds within "
                        "their joint limits (80°/s for Joints 1–4, 70°/s for Joints 5–7)."
                    )
                validated_speeds.append([float(speed) for speed in speed_row])
        return validated, validated_speeds

    @staticmethod
    def _validate_transition_gradients(
        gradients: object, position_count: int
    ) -> list[dict[str, object] | None]:
        if gradients is None:
            return [None] * position_count
        if not isinstance(gradients, list):
            raise ValueError("The movement transition_gradients value must be a list.")
        if len(gradients) == position_count - 1:
            gradients = list(gradients) + [None]
        if len(gradients) != position_count:
            raise ValueError(
                "The movement file must contain one gradient setting per position."
            )

        validated: list[dict[str, object] | None] = []
        for row, gradient in enumerate(gradients, start=1):
            if gradient is None:
                validated.append(None)
                continue
            if not isinstance(gradient, dict):
                raise ValueError(
                    f"Position {row} transition gradient must be an object or null."
                )
            increase = gradient.get("increase_dps2")
            decrease = gradient.get("decrease_dps2")
            joints = gradient.get("joints")
            if (
                isinstance(increase, bool)
                or not isinstance(increase, (int, float))
                or not math.isfinite(increase)
                or increase <= 0
                or isinstance(decrease, bool)
                or not isinstance(decrease, (int, float))
                or not math.isfinite(decrease)
                or decrease <= 0
                or not isinstance(joints, list)
                or not joints
                or any(
                    isinstance(joint, bool)
                    or not isinstance(joint, int)
                    or not 1 <= joint <= len(JOINT_NAMES)
                    for joint in joints
                )
                or len(set(joints)) != len(joints)
            ):
                raise ValueError(
                    f"Position {row} transition gradient must have positive finite "
                    "increase/decrease values and one or more distinct joint numbers "
                    "from 1 through 7."
                )
            validated.append(
                {
                    "increase_dps2": float(increase),
                    "decrease_dps2": float(decrease),
                    "joints": sorted(joints),
                }
            )
        return validated

    def _toggle_preview(
        self,
        table: QTableWidget,
        loop_mode: QComboBox,
        play_count: QSpinBox,
        time_limit: QDoubleSpinBox,
        button: QPushButton,
    ) -> None:
        if self._preview_timer.isActive() and self._preview_button is button:
            self._stop_preview()
            return
        self._stop_preview()
        try:
            positions, speeds = self._read_waypoint_data(table)
        except ValueError as error:
            QMessageBox.warning(self, "Cannot play preview", str(error))
            return

        try:
            gradients = self._read_preview_gradients(table)
        except ValueError as error:
            QMessageBox.warning(self, "Cannot play preview", str(error))
            return

        self._preview_table = table
        self._preview_positions = positions
        self._preview_speeds = speeds
        self._preview_gradients = gradients
        self._preview_row = 0
        self._preview_completed_plays = 0
        self._preview_mode = str(loop_mode.currentData())
        self._preview_play_count = play_count.value()
        self._preview_deadline = (
            time.monotonic() + time_limit.value() * 60
            if self._preview_mode == "time_limit"
            else None
        )
        self._preview_button = button
        self._preview_animation = MovementAnimationDialog(self)
        self._preview_animation.set_pose(positions[0], 1)
        self._preview_animation.show()
        button.setText("Stop preview")
        self._select_preview_row(0)
        self._schedule_preview_step()

    def _select_preview_row(self, row: int) -> None:
        if self._preview_table is None:
            return
        self._preview_table.selectRow(row)
        item = self._preview_table.item(row, 0)
        if item is not None:
            self._preview_table.scrollToItem(item)

    def _read_preview_gradients(
        self, table: QTableWidget
    ) -> list[dict[str, object] | None]:
        if table is self.playback_table:
            if self._loaded_movement is None:
                return [None] * table.rowCount()
            gradients = self._loaded_movement["transition_gradients"]
            return list(gradients)
        return self._read_transition_gradients(table)

    def _schedule_preview_step(self) -> None:
        if self._preview_row < len(self._preview_positions) - 1:
            destination = self._preview_row + 1
            destination_position = self._preview_positions[destination]
            speeds = self._preview_speeds[self._preview_row]
            gradient = self._preview_gradients[self._preview_row]
            interval_ms = transition_duration_ms(
                self._preview_positions[self._preview_row],
                destination_position,
                speeds,
                gradient,
            )
            if self._preview_animation is not None:
                self._preview_animation.animate_to(
                    destination_position,
                    speeds,
                    interval_ms,
                    destination + 1,
                    gradient=gradient,
                )
        elif self._preview_will_repeat():
            destination_position = self._preview_positions[0]
            speeds = self._preview_speeds[self._preview_row]
            gradient = self._preview_gradients[self._preview_row]
            interval_ms = transition_duration_ms(
                self._preview_positions[self._preview_row],
                destination_position,
                speeds,
                gradient,
            )
            if self._preview_animation is not None:
                self._preview_animation.animate_to(
                    destination_position,
                    speeds,
                    interval_ms,
                    1,
                    gradient=gradient,
                )
        else:
            interval_ms = 250
            if self._preview_animation is not None:
                self._preview_animation.set_pose(
                    self._preview_positions[self._preview_row],
                    self._preview_row + 1,
                )

        if self._preview_deadline is not None:
            remaining_ms = math.ceil(
                (self._preview_deadline - time.monotonic()) * 1000
            )
            if remaining_ms <= 0:
                self._stop_preview()
                return
            interval_ms = min(interval_ms, remaining_ms)
        self._preview_timer.start(interval_ms)

    def _preview_will_repeat(self) -> bool:
        return self._preview_mode in ("until_stopped", "time_limit") or (
            self._preview_mode == "play_count"
            and self._preview_completed_plays + 1 < self._preview_play_count
        )

    def _advance_preview(self) -> None:
        if (
            self._preview_deadline is not None
            and time.monotonic() >= self._preview_deadline
        ):
            self._stop_preview()
            return

        if self._preview_row == len(self._preview_positions) - 1:
            self._preview_completed_plays += 1
            if self._preview_mode == "once" or (
                self._preview_mode == "play_count"
                and self._preview_completed_plays >= self._preview_play_count
            ):
                self._stop_preview()
                return
            self._preview_row = 0
        else:
            self._preview_row += 1

        self._select_preview_row(self._preview_row)
        self._schedule_preview_step()

    def _stop_preview(self) -> None:
        self._preview_timer.stop()
        if self._preview_button is not None:
            self._preview_button.setText("Play preview")
        if self._preview_table is not None:
            self._preview_table.clearSelection()
        if self._preview_animation is not None:
            self._preview_animation.stop_animation()
            self._preview_animation.close()
            self._preview_animation.deleteLater()
        self._preview_animation = None
        self._preview_table = None
        self._preview_button = None

    @staticmethod
    def _is_valid_joint_angle(angle: object, joint_index: int) -> bool:
        if (
            isinstance(angle, bool)
            or not isinstance(angle, (int, float))
            or not math.isfinite(angle)
        ):
            return False
        limit = JOINT_ANGLE_LIMITS_DEG[joint_index]
        return limit is None or -limit <= angle <= limit

    @staticmethod
    def _is_valid_joint_speed(value: object, joint_index: int) -> bool:
        return (
            not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value)
            and 0 < value <= JOINT_SPEED_LIMITS_DPS[joint_index]
        )

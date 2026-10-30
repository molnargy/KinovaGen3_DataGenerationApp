import os
import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from kinova_movement_app.connection import RobotConnection, TCP_PORT, _load_kortex_api
from kinova_movement_app.layout import MainWindow


class RobotConnectionTests(unittest.TestCase):
    def test_installed_kortex_sdk_imports(self) -> None:
        (
            transport_class,
            router_class,
            session_class,
            session_messages,
            base_client_class,
            base_messages,
        ) = _load_kortex_api()

        self.assertEqual(transport_class.__name__, "TCPTransport")
        self.assertEqual(router_class.__name__, "RouterClient")
        self.assertEqual(session_class.__name__, "SessionManager")
        self.assertEqual(base_client_class.__name__, "BaseClient")
        self.assertTrue(callable(base_messages.WaypointList))
        self.assertTrue(callable(session_messages.CreateSessionInfo))
        session_info = session_messages.CreateSessionInfo()
        session_info.username = "test-user"
        self.assertEqual(session_info.username, "test-user")

    def test_open_authenticates_and_close_releases_the_session(self) -> None:
        transport_class = MagicMock()
        router_class = MagicMock()
        session_manager_class = MagicMock()
        base_client_class = MagicMock()
        base_messages = MagicMock()
        session_message = MagicMock()
        session_pb2 = MagicMock()
        session_pb2.CreateSessionInfo.return_value = session_message
        transport = transport_class.return_value
        router = router_class.return_value
        session_manager = session_manager_class.return_value

        with patch(
            "kinova_movement_app.connection._load_kortex_api",
            return_value=(
                transport_class,
                router_class,
                session_manager_class,
                session_pb2,
                base_client_class,
                base_messages,
            ),
        ):
            connection = RobotConnection.open("10.0.0.8", "robot-user", "secret")

        transport.connect.assert_called_once_with("10.0.0.8", TCP_PORT)
        router_class.assert_called_once_with(transport)
        session_manager_class.assert_called_once_with(router)
        session_info = session_manager.CreateSession.call_args.args[0]
        self.assertEqual(session_info.username, "robot-user")
        self.assertEqual(session_info.password, "secret")
        self.assertEqual(session_info.session_inactivity_timeout, 60000)
        self.assertEqual(session_info.connection_inactivity_timeout, 2000)

        connection.close()

        session_manager.CloseSession.assert_called_once_with()
        transport.disconnect.assert_called_once_with()

    def test_failed_authentication_disconnects_the_transport(self) -> None:
        transport_class = MagicMock()
        router_class = MagicMock()
        session_manager_class = MagicMock()
        transport = transport_class.return_value
        session_manager_class.return_value.CreateSession.side_effect = RuntimeError(
            "authentication rejected"
        )

        with (
            patch(
                "kinova_movement_app.connection._load_kortex_api",
                return_value=(
                    transport_class,
                    router_class,
                    session_manager_class,
                    MagicMock(),
                    MagicMock(),
                    MagicMock(),
                ),
            ),
            self.assertRaisesRegex(RuntimeError, "authentication rejected"),
        ):
            RobotConnection.open("10.0.0.8", "robot-user", "wrong")

        transport.disconnect.assert_called_once_with()

    def test_close_disconnects_transport_if_session_close_fails(self) -> None:
        transport = MagicMock()
        router = MagicMock()
        session_manager = MagicMock()
        session_manager.CloseSession.side_effect = RuntimeError("session close failed")
        connection = RobotConnection(transport, router, session_manager)

        with self.assertRaisesRegex(RuntimeError, "session close failed"):
            connection.close()

        transport.disconnect.assert_called_once_with()

    def test_read_joint_angles_uses_the_tcp_session(self) -> None:
        transport = MagicMock()
        router = MagicMock()
        session_manager = MagicMock()
        base_client_class = MagicMock()
        base_client_class.return_value.GetMeasuredJointAngles.return_value = (
            SimpleNamespace(
                joint_angles=[
                    SimpleNamespace(
                        joint_identifier=joint + 1,
                        value=float(joint),
                    )
                    for joint in reversed(range(7))
                ]
            )
        )
        api = (
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(),
            base_client_class,
            MagicMock(),
        )
        connection = RobotConnection(transport, router, session_manager)

        with patch("kinova_movement_app.connection._load_kortex_api", return_value=api):
            angles = connection.read_joint_angles()

        self.assertEqual(angles, [float(joint) for joint in range(7)])
        base_client_class.assert_called_once_with(router)


class ConnectionUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        self.window = MainWindow()

    def tearDown(self) -> None:
        self.window.close()

    def _wait_for_state(self, state: str) -> None:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            QTest.qWait(10)
            if (
                self.window._connection_state == state
                and self.window._connection_worker is None
            ):
                return
        self.fail(f"Connection state did not become {state!r}")

    def test_home_controls_connect_close_and_update_sidebar_indicator(self) -> None:
        connection = object.__new__(RobotConnection)
        self.window.robot_ip_input.setText("10.0.0.8")
        self.window.robot_username_input.setText("robot-user")
        self.window.robot_password_input.setText("secret")

        with (
            patch.object(RobotConnection, "open", return_value=connection) as open_robot,
            patch.object(RobotConnection, "close") as close_robot,
        ):
            self.window.connect_button.click()
            self.assertEqual(self.window.robot_password_input.text(), "")
            self._wait_for_state("connected")

            open_robot.assert_called_once_with("10.0.0.8", "robot-user", "secret")
            self.assertEqual(self.window.connection_status_label.text(), "Connected")
            self.assertIn("#35c98f", self.window.connection_indicator.styleSheet())
            self.assertTrue(self.window.close_connection_button.isEnabled())

            self.window.close_connection_button.click()
            self._wait_for_state("disconnected")

            close_robot.assert_called_once_with()
            self.assertEqual(
                self.window.connection_status_label.text(), "Not connected"
            )
            self.assertIn("#ef5b65", self.window.connection_indicator.styleSheet())


if __name__ == "__main__":
    unittest.main()

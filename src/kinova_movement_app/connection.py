"""Manage an authenticated TCP session with a Kinova Gen3 robot."""

from __future__ import annotations

import collections
import collections.abc
import threading
from typing import Any


TCP_PORT = 10000
_KORTEX_IMPORT_LOCK = threading.Lock()


def _load_kortex_api() -> tuple[Any, Any, Any, Any, Any, Any]:
    # Kortex pins protobuf 3.5.1, which still imports these removed aliases.
    legacy_names = (
        "Callable",
        "Iterable",
        "Mapping",
        "MutableMapping",
        "MutableSequence",
        "Sequence",
    )
    missing_names = [
        name for name in legacy_names if not hasattr(collections, name)
    ]
    with _KORTEX_IMPORT_LOCK:
        for name in missing_names:
            setattr(collections, name, getattr(collections.abc, name))
        try:
            from kortex_api.RouterClient import RouterClient
            from kortex_api.SessionManager import SessionManager
            from kortex_api.TCPTransport import TCPTransport
            from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
            from kortex_api.autogen.messages import Base_pb2, Session_pb2
        finally:
            for name in missing_names:
                delattr(collections, name)
    return (
        TCPTransport,
        RouterClient,
        SessionManager,
        Session_pb2,
        BaseClient,
        Base_pb2,
    )


class RobotConnection:
    """Keep the transport and authenticated session open until explicitly closed."""

    def __init__(self, transport: Any, router: Any, session_manager: Any) -> None:
        self._transport = transport
        self._router = router
        self._session_manager = session_manager
        self._closed = False

    @classmethod
    def open(cls, ip: str, username: str, password: str) -> RobotConnection:
        """Connect to the robot and create an authenticated Kortex session."""
        (
            TCPTransport,
            RouterClient,
            SessionManager,
            Session_pb2,
            _,
            _,
        ) = _load_kortex_api()
        transport = TCPTransport()
        router = RouterClient(transport)
        session_manager = SessionManager(router)
        transport_connected = False

        try:
            transport.connect(ip, TCP_PORT)
            transport_connected = True

            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = username
            session_info.password = password
            session_info.session_inactivity_timeout = 60000
            session_info.connection_inactivity_timeout = 2000
            session_manager.CreateSession(session_info)
        except Exception as error:
            if transport_connected:
                try:
                    transport.disconnect()
                except Exception as cleanup_error:
                    raise RuntimeError(
                        f"{error}; connection cleanup also failed: {cleanup_error}"
                    ) from error
            raise

        return cls(transport, router, session_manager)

    def create_base_client(self) -> Any:
        """Create a Kortex base-service client for this live session."""
        if self._closed:
            raise RuntimeError("The robot connection is closed.")
        base_client_class = _load_kortex_api()[4]
        return base_client_class(self._router)

    def read_joint_angles(self, base_client: Any | None = None) -> list[float]:
        """Read the current seven actuator angles in degrees."""
        if self._closed:
            raise RuntimeError("The robot connection is closed.")
        client = base_client or self.create_base_client()
        feedback = client.GetMeasuredJointAngles()
        angles_by_joint = {
            int(joint.joint_identifier): float(joint.value)
            for joint in feedback.joint_angles
        }
        expected_joints = set(range(1, 8))
        if set(angles_by_joint) != expected_joints:
            raise RuntimeError(
                "Expected measured angles for Joints 1 through 7."
            )
        return [angles_by_joint[joint] for joint in range(1, 8)]

    def close(self) -> None:
        """Close the authenticated session and disconnect its TCP transport."""
        if self._closed:
            return
        self._closed = True

        errors: list[Exception] = []
        try:
            self._session_manager.CloseSession()
        except Exception as error:
            errors.append(error)
        try:
            self._transport.disconnect()
        except Exception as error:
            errors.append(error)

        if len(errors) == 1:
            raise errors[0]
        if errors:
            raise RuntimeError(
                "Could not close the Kortex session and TCP transport: "
                + "; ".join(str(error) for error in errors)
            ) from errors[0]

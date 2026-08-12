import queue
import threading
from types import SimpleNamespace

import pytest

from decentralized_swarm_integration.px4_agent import Px4Agent
from decentralized_swarm_integration.px4_protocol import SafeSetpoint


class FakeMav:
    def __init__(self):
        self.setpoint_args = None
        self.command_args = None

    def set_position_target_local_ned_send(self, *args):
        self.setpoint_args = args

    def command_long_send(self, *args):
        self.command_args = args


class FakeConnection:
    target_system = 3
    target_component = 1

    def __init__(self, acknowledgement):
        self.mav = FakeMav()
        self.acknowledgement = acknowledgement

    def recv_match(self, **_kwargs):
        result, self.acknowledgement = self.acknowledgement, None
        return result


def agent_with_result(result=0):
    constants = SimpleNamespace(
        POSITION_TARGET_TYPEMASK_VX_IGNORE=1,
        POSITION_TARGET_TYPEMASK_VY_IGNORE=2,
        POSITION_TARGET_TYPEMASK_VZ_IGNORE=4,
        POSITION_TARGET_TYPEMASK_AX_IGNORE=8,
        POSITION_TARGET_TYPEMASK_AY_IGNORE=16,
        POSITION_TARGET_TYPEMASK_AZ_IGNORE=32,
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE=64,
        MAV_FRAME_LOCAL_NED=1,
        MAV_RESULT_ACCEPTED=0,
        MAV_RESULT_IN_PROGRESS=5,
    )
    agent = Px4Agent.__new__(Px4Agent)
    agent.mavutil = SimpleNamespace(mavlink=constants)
    acknowledgement = SimpleNamespace(command=400, result=result)
    agent._connection = FakeConnection(acknowledgement)
    return agent


def test_setpoint_wire_order_is_local_ned_position_and_yaw():
    agent = agent_with_result()
    agent._send_setpoint(SafeSetpoint(4.0, 3.0, -8.0, 0.75))
    args = agent._connection.mav.setpoint_args
    assert args[1:4] == (3, 1, 1)
    assert args[5:8] == (4.0, 3.0, -8.0)
    assert args[14] == 0.75
    assert args[4] == 127


def test_command_waits_for_positive_px4_acknowledgement():
    agent = agent_with_result(result=0)
    agent._send_command(400, 1.0)
    assert agent._connection.mav.command_args[:5] == (3, 1, 400, 0, 1.0)


def test_rejected_px4_command_raises():
    agent = agent_with_result(result=2)
    with pytest.raises(RuntimeError, match="rejected command"):
        agent._send_command(400, 1.0)


def test_arm_offboard_confirms_mode_before_sending_arm_command():
    events = []

    class SequencedMav:
        def set_mode_send(self, *_args):
            events.append("request_offboard")

        def command_long_send(self, *_args):
            events.append("request_arm")

    class SequencedConnection:
        target_system = 3
        target_component = 1
        mav = SequencedMav()

        def recv_match(self, *, type, **_kwargs):
            if type == "HEARTBEAT":
                events.append("confirm_offboard")
                return SimpleNamespace(custom_mode=6 << 16)
            if type == "COMMAND_ACK":
                events.append("confirm_arm")
                return SimpleNamespace(command=400, result=0)
            raise AssertionError(f"unexpected MAVLink message request: {type}")

    constants = SimpleNamespace(
        MAV_CMD_COMPONENT_ARM_DISARM=400,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1,
        MAV_RESULT_ACCEPTED=0,
        MAV_RESULT_IN_PROGRESS=5,
    )
    agent = Px4Agent.__new__(Px4Agent)
    agent.mavutil = SimpleNamespace(mavlink=constants)
    agent._connection = SequencedConnection()
    agent._command_queue = queue.Queue()
    completed = threading.Event()
    result = {}
    agent._command_queue.put(("arm_offboard", completed, result))

    agent._process_command()

    assert completed.is_set()
    assert result == {"success": True, "message": "PX4 arm_offboard confirmed"}
    assert events == [
        "request_offboard",
        "confirm_offboard",
        "request_arm",
        "confirm_arm",
    ]

import pytest

from dji_swarm_integration.dji_protocol import DjiCommand, decode_command, encode_command
from dji_swarm_integration.mission_geometry import MissionPoint, role_setpoint
from dji_swarm_integration.role_protocol import AgentState, assign_roles


def state(uav_id, *, camera=False, lidar=False, detection=0.0, link=0.8):
    return AgentState(
        uav_id=uav_id,
        sequence=1,
        stamp_ns=1,
        received_ns=1,
        battery=0.9,
        link_quality=link,
        detection_confidence=detection,
        camera_capable=camera,
        lidar_capable=lidar,
        mobile=True,
    )


def test_decentralized_role_to_map_mission_to_dji_absolute_enu():
    """Exercise the complete pure control path across subsystem boundaries."""
    states = [
        state("dji0", lidar=True, link=0.9),
        state("dji1", lidar=True, link=0.7),
        state("dji2", camera=True, detection=0.95, link=0.8),
    ]
    roles = assign_roles(states)
    assert roles["dji2"] == "observer"

    anchor = MissionPoint(20.0, 30.0, 1.0, 0.0)
    target = MissionPoint(28.0, 30.0, 1.0, 0.0)
    map_goal = role_setpoint(
        role=roles["dji2"],
        slot=2,
        anchor=anchor,
        semantic_target=target,
        altitude_m=7.0,
        scout_lead_m=8.0,
        formation_spacing_m=3.0,
        observer_standoff_m=5.0,
    )
    assert map_goal is not None
    assert (map_goal.x, map_goal.y, map_goal.z) == pytest.approx((23.0, 33.0, 8.0))

    # The tested DJI/PSDK-compatible interface consumes absolute ENU values.
    wire_goal = DjiCommand(
        uav_id="dji2",
        sequence=2,
        stamp_ns=20,
        kind="position",
        x=map_goal.x,
        y=map_goal.y,
        z=map_goal.z,
        yaw=map_goal.yaw,
        reason="observer_role",
    )
    decoded = decode_command(encode_command(wire_goal))
    assert (decoded.x, decoded.y, decoded.z) == pytest.approx((23.0, 33.0, 8.0))
    assert decoded.yaw == pytest.approx(map_goal.yaw)


def test_observer_without_semantic_consensus_cannot_generate_control_goal():
    goal = role_setpoint(
        role="observer",
        slot=0,
        anchor=MissionPoint(0.0, 0.0, 0.0, 0.0),
        semantic_target=None,
        altitude_m=7.0,
        scout_lead_m=8.0,
        formation_spacing_m=3.0,
        observer_standoff_m=5.0,
    )
    assert goal is None

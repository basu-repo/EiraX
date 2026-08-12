import math

import pytest

from decentralized_swarm_integration.mission_geometry import MissionPoint, role_setpoint
from decentralized_swarm_integration.px4_protocol import Point3, validate_setpoint
from decentralized_swarm_integration.role_protocol import AgentState, assign_roles


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


def test_decentralized_role_to_map_mission_to_px4_local_ned():
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

    # dji2 was spawned at (20, 20, 0) in the shared map. The adapter must
    # transmit a LOCAL_NED target relative to that origin, not map coordinates.
    wire_goal = validate_setpoint(
        x_enu=map_goal.x,
        y_enu=map_goal.y,
        z_enu=map_goal.z,
        yaw_enu=map_goal.yaw,
        home_enu=Point3(20.0, 20.0, 0.0),
        current_enu=Point3(22.0, 30.0, 7.0),
        max_radius_m=120.0,
        min_altitude_m=2.0,
        max_altitude_m=30.0,
        max_step_m=15.0,
    )
    assert (wire_goal.north, wire_goal.east, wire_goal.down) == pytest.approx(
        (13.0, 3.0, -8.0)
    )
    assert wire_goal.yaw_ned == pytest.approx(
        math.pi / 2.0 - math.atan2(-3.0, 5.0)
    )


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

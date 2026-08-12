import math

from decentralized_swarm_integration.mission_geometry import MissionPoint, role_setpoint


ANCHOR = MissionPoint(10.0, 20.0, 0.0, 0.0)


def setpoint(role, slot=1, target=None):
    return role_setpoint(
        role=role,
        slot=slot,
        anchor=ANCHOR,
        semantic_target=target,
        altitude_m=10.0,
        scout_lead_m=20.0,
        formation_spacing_m=5.0,
        observer_standoff_m=8.0,
    )


def test_scout_leads_mission_anchor():
    assert setpoint("scout") == MissionPoint(30.0, 20.0, 10.0, 0.0)


def test_slots_create_lateral_separation():
    first = setpoint("mapper", slot=0)
    third = setpoint("mapper", slot=2)
    assert math.dist((first.x, first.y), (third.x, third.y)) == 10.0


def test_observer_requires_semantic_target_and_faces_it():
    assert setpoint("observer") is None
    result = setpoint("observer", target=MissionPoint(50.0, 20.0, 0.0, 0.0))
    assert result.x == 42.0
    assert result.yaw == 0.0


def test_reserve_does_not_command_motion():
    assert setpoint("reserve") is None


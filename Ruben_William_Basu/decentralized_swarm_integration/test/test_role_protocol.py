from decentralized_swarm_integration.role_protocol import (
    AgentState,
    assign_roles,
    decode_agent_state,
    encode_agent_state,
)


def state(uav_id, *, battery=0.8, link=0.8, detection=0.0, camera=True, lidar=True):
    return AgentState(
        uav_id=uav_id,
        sequence=1,
        stamp_ns=10,
        received_ns=20,
        battery=battery,
        link_quality=link,
        detection_confidence=detection,
        camera_capable=camera,
        lidar_capable=lidar,
        mobile=True,
    )


def test_state_protocol_round_trip_sets_local_receive_time():
    original = state("dji0")
    decoded = decode_agent_state(encode_agent_state(original), 99)
    assert decoded.received_ns == 99
    assert decoded.uav_id == original.uav_id


def test_assignments_are_order_independent():
    agents = [
        state("dji0", detection=0.95, lidar=False),
        state("dji1", detection=0.1, lidar=True),
        state("dji2", detection=0.2, link=0.99),
    ]
    assert assign_roles(agents) == assign_roles(list(reversed(agents)))


def test_observer_prefers_camera_with_strong_detection():
    agents = [
        state("dji0", detection=0.99, lidar=False),
        state("dji1", detection=0.0, camera=False, lidar=True),
        state("dji2", detection=0.1),
    ]
    assert assign_roles(agents)["dji0"] == "observer"


def test_more_agents_than_roles_become_reserve():
    agents = [state(f"uav{i}") for i in range(5)]
    result = assign_roles(agents)
    assert sorted(result.values()).count("reserve") == 1


def test_duplicate_agent_state_does_not_create_extra_agent():
    old = state("dji0")
    new = AgentState(**{**old.__dict__, "sequence": 2, "battery": 0.2})
    result = assign_roles([old, new, state("dji1")])
    assert set(result) == {"dji0", "dji1"}


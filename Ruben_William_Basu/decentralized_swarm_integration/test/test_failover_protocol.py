from pathlib import Path

from vehicle_stack.uav.failover_protocol import (
    elect_successor, initial_state, read_state, reconnect_as_follower, write_state,
)


def test_permanent_failures_transfer_scout_then_leave_nav2_fallback(tmp_path: Path):
    state = initial_state()
    assert state.active_scout == "uav0"
    state = elect_successor(state, "uav0", permanent=True)
    assert state.active_scout == "uav1" and state.role_of("uav0") == "returning"
    state = elect_successor(state, "uav1", permanent=True)
    assert state.active_scout == "uav2"
    state = elect_successor(state, "uav2", permanent=True)
    assert state.active_scout is None
    path = tmp_path / "roles.json"
    write_state(path, state)
    assert read_state(path) == state


def test_reconnected_uav_does_not_reclaim_scout():
    state = elect_successor(initial_state(), "uav0", permanent=False)
    assert state.active_scout == "uav1"
    state = reconnect_as_follower(state, "uav0")
    assert state.active_scout == "uav1"
    assert state.role_of("uav0") == "follower"

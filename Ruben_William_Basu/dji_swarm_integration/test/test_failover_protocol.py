from pathlib import Path

from dji_swarm_integration.failover_protocol import (
    elect_successor, initial_state, read_state, reconnect_as_follower, write_state,
)


def test_permanent_failures_transfer_scout_then_leave_nav2_fallback(tmp_path: Path):
    state = initial_state()
    assert state.active_scout == "dji0"
    state = elect_successor(state, "dji0", permanent=True)
    assert state.active_scout == "dji1" and state.role_of("dji0") == "returning"
    state = elect_successor(state, "dji1", permanent=True)
    assert state.active_scout == "dji2"
    state = elect_successor(state, "dji2", permanent=True)
    assert state.active_scout is None
    path = tmp_path / "roles.json"
    write_state(path, state)
    assert read_state(path) == state


def test_reconnected_uav_does_not_reclaim_scout():
    state = elect_successor(initial_state(), "dji0", permanent=False)
    assert state.active_scout == "dji1"
    state = reconnect_as_follower(state, "dji0")
    assert state.active_scout == "dji1"
    assert state.role_of("dji0") == "follower"

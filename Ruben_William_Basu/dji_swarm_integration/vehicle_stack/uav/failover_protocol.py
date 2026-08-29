"""Atomic, deterministic mission-role state shared by the isolated UAV processes."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path


UAV_IDS = ("uav0", "uav1", "uav2")
VALID_STATES = {"scout", "follower", "disconnected", "recovering", "returning", "landed"}


@dataclass(frozen=True)
class SwarmRoleState:
    term: int
    active_scout: str | None
    states: dict[str, str]
    reason: str

    def role_of(self, uav_id: str) -> str:
        return self.states.get(uav_id, "follower")


def initial_state() -> SwarmRoleState:
    return SwarmRoleState(1, "uav0", {"uav0": "scout", "uav1": "follower", "uav2": "follower"}, "mission_start")


def elect_successor(state: SwarmRoleState, failed_uav: str, permanent: bool) -> SwarmRoleState:
    states = dict(state.states)
    states[failed_uav] = "returning" if permanent else "disconnected"
    candidates = [uav for uav in UAV_IDS if uav != failed_uav and states.get(uav) in {"scout", "follower", "recovering"}]
    successor = candidates[0] if candidates else None
    for uav in UAV_IDS:
        if uav == successor:
            states[uav] = "scout"
        elif states.get(uav) == "scout":
            states[uav] = "follower"
    return SwarmRoleState(state.term + 1, successor, states, f"{failed_uav}_{'permanent' if permanent else 'link_lost'}")


def reconnect_as_follower(state: SwarmRoleState, uav_id: str) -> SwarmRoleState:
    states = dict(state.states)
    states[uav_id] = "follower"
    return SwarmRoleState(state.term, state.active_scout, states, f"{uav_id}_rejoined_as_follower")


def write_state(path: Path, state: SwarmRoleState) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps({"term": state.term, "active_scout": state.active_scout, "states": state.states, "reason": state.reason}, sort_keys=True), encoding="utf-8")
    temporary.replace(path)


def read_state(path: Path) -> SwarmRoleState:
    payload = json.loads(path.read_text(encoding="utf-8"))
    states = {str(key): str(value) for key, value in payload["states"].items()}
    if any(value not in VALID_STATES for value in states.values()):
        raise ValueError("invalid UAV role state")
    scout = payload.get("active_scout")
    if scout is not None and states.get(scout) != "scout":
        raise ValueError("active scout and role map disagree")
    return SwarmRoleState(int(payload["term"]), scout, states, str(payload.get("reason", "")))

"""Deterministic three-DJI role state used by independent flight processes."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path


DJI_IDS = ("dji0", "dji1", "dji2")


@dataclass(frozen=True)
class SwarmState:
    term: int
    active_scout: str | None
    states: dict[str, str]
    reason: str


def initial_state() -> SwarmState:
    return SwarmState(
        1, "dji0", {"dji0": "scout", "dji1": "follower", "dji2": "follower"},
        "mission_start",
    )


def fail(state: SwarmState, aircraft: str, permanent: bool) -> SwarmState:
    roles = dict(state.states)
    roles[aircraft] = "returning" if permanent else "disconnected"
    candidates = [
        name for name in DJI_IDS
        if name != aircraft and roles.get(name) in {"scout", "follower", "recovering"}
    ]
    successor = candidates[0] if candidates else None
    for name in DJI_IDS:
        if name == successor:
            roles[name] = "scout"
        elif roles.get(name) == "scout":
            roles[name] = "follower"
    reason = f"{aircraft}_{'permanent' if permanent else 'link_lost'}"
    return SwarmState(state.term + 1, successor, roles, reason)


def reconnect(state: SwarmState, aircraft: str) -> SwarmState:
    roles = dict(state.states)
    roles[aircraft] = "follower"
    return SwarmState(state.term, state.active_scout, roles, f"{aircraft}_rejoined")


def write(path: Path, state: SwarmState) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(asdict(state), sort_keys=True), encoding="utf-8")
    temporary.replace(path)


def read(path: Path) -> SwarmState:
    data = json.loads(path.read_text(encoding="utf-8"))
    return SwarmState(
        int(data["term"]), data.get("active_scout"),
        {str(k): str(v) for k, v in data["states"].items()}, str(data["reason"]),
    )

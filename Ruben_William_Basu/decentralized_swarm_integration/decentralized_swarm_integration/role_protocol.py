"""Pure leaderless role-state protocol and deterministic assignment."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import itertools
import json
import math


ROLES = ("scout", "mapper", "observer", "relay")


@dataclass(frozen=True)
class AgentState:
    uav_id: str
    sequence: int
    stamp_ns: int
    received_ns: int
    battery: float
    link_quality: float
    detection_confidence: float
    camera_capable: bool
    lidar_capable: bool
    mobile: bool


def encode_agent_state(state: AgentState) -> str:
    payload = asdict(state)
    payload.pop("received_ns")
    payload["protocol"] = "eirax.agent_state.v1"
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def decode_agent_state(data: str, received_ns: int) -> AgentState:
    payload = json.loads(data)
    if payload.get("protocol") != "eirax.agent_state.v1":
        raise ValueError("unsupported agent-state protocol")
    state = AgentState(
        uav_id=str(payload["uav_id"]).strip(),
        sequence=int(payload["sequence"]),
        stamp_ns=int(payload["stamp_ns"]),
        received_ns=int(received_ns),
        battery=float(payload["battery"]),
        link_quality=float(payload["link_quality"]),
        detection_confidence=float(payload["detection_confidence"]),
        camera_capable=bool(payload["camera_capable"]),
        lidar_capable=bool(payload["lidar_capable"]),
        mobile=bool(payload["mobile"]),
    )
    if not state.uav_id or state.sequence < 0:
        raise ValueError("invalid agent identity or sequence")
    for name, value in (
        ("battery", state.battery),
        ("link_quality", state.link_quality),
        ("detection_confidence", state.detection_confidence),
    ):
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} must be finite and within [0, 1]")
    return state


def role_score(state: AgentState, role: str) -> float:
    mobility = 1.0 if state.mobile else 0.0
    camera = 1.0 if state.camera_capable else 0.0
    lidar = 1.0 if state.lidar_capable else 0.0
    if role == "scout":
        return 0.35 * state.battery + 0.25 * lidar + 0.20 * mobility + 0.20 * state.link_quality
    if role == "mapper":
        return 0.35 * lidar + 0.25 * state.battery + 0.20 * mobility + 0.20 * state.link_quality
    if role == "observer":
        return 0.35 * camera + 0.35 * state.detection_confidence + 0.15 * state.battery + 0.15 * state.link_quality
    if role == "relay":
        return 0.45 * state.link_quality + 0.35 * state.battery + 0.20 * mobility
    raise ValueError(f"unsupported role: {role}")


def assign_roles(states: list[AgentState], roles=ROLES) -> dict[str, str]:
    """Find the maximum-score one-agent-per-role assignment.

    Identical inputs always produce identical assignments, so every peer can
    reach the same result without a leader or election service.
    """
    current = {}
    for state in states:
        previous = current.get(state.uav_id)
        if previous is None or (state.sequence, state.stamp_ns) > (
            previous.sequence,
            previous.stamp_ns,
        ):
            current[state.uav_id] = state
    agents = [current[key] for key in sorted(current)]
    if not agents:
        return {}
    active_roles = tuple(roles[: min(len(roles), len(agents))])
    best_key = None
    best_assignment = None
    for chosen in itertools.permutations(agents, len(active_roles)):
        score = sum(role_score(agent, role) for agent, role in zip(chosen, active_roles))
        assignment_tuple = tuple(agent.uav_id for agent in chosen)
        # Lexicographically smaller identities win exact score ties.
        key = (round(score, 12), tuple(-ord(char) for char in "|".join(assignment_tuple)))
        if best_key is None or key > best_key:
            best_key = key
            best_assignment = {
                agent.uav_id: role for agent, role in zip(chosen, active_roles)
            }
    assert best_assignment is not None
    for agent in agents:
        best_assignment.setdefault(agent.uav_id, "reserve")
    return best_assignment


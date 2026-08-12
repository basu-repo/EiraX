"""Pure decentralized role-to-formation geometry."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MissionPoint:
    x: float
    y: float
    z: float
    yaw: float


def role_setpoint(
    *,
    role: str,
    slot: int,
    anchor: MissionPoint,
    semantic_target: MissionPoint | None,
    altitude_m: float,
    scout_lead_m: float,
    formation_spacing_m: float,
    observer_standoff_m: float,
) -> MissionPoint | None:
    """Compute a deterministic setpoint; `anchor.yaw` defines mission forward."""
    if role == "reserve":
        return None
    forward = (math.cos(anchor.yaw), math.sin(anchor.yaw))
    left = (-forward[1], forward[0])
    lateral_slot = (slot - 1) * formation_spacing_m
    if role == "scout":
        x = anchor.x + scout_lead_m * forward[0] + lateral_slot * left[0]
        y = anchor.y + scout_lead_m * forward[1] + lateral_slot * left[1]
        yaw = anchor.yaw
    elif role == "mapper":
        x = anchor.x + lateral_slot * left[0]
        y = anchor.y + lateral_slot * left[1]
        yaw = anchor.yaw
    elif role == "relay":
        # Remain near mission control while preserving a distinct altitude/slot.
        x = anchor.x - formation_spacing_m * forward[0] + lateral_slot * left[0]
        y = anchor.y - formation_spacing_m * forward[1] + lateral_slot * left[1]
        yaw = anchor.yaw
    elif role == "observer":
        if semantic_target is None:
            return None
        x = semantic_target.x - observer_standoff_m * forward[0] + lateral_slot * left[0]
        y = semantic_target.y - observer_standoff_m * forward[1] + lateral_slot * left[1]
        yaw = math.atan2(semantic_target.y - y, semantic_target.x - x)
    else:
        raise ValueError(f"unsupported role: {role}")
    return MissionPoint(x, y, anchor.z + altitude_m, yaw)


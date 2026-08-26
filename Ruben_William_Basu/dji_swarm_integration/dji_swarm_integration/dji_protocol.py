"""Validated command contract shared by DJI simulation and hardware adapters."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math


COMMAND_PROTOCOL = "eirax.dji_command.v1"
COMMAND_KINDS = {"hold", "takeoff", "position", "return", "land"}


@dataclass(frozen=True)
class DjiCommand:
    uav_id: str
    sequence: int
    stamp_ns: int
    kind: str
    x: float
    y: float
    z: float
    yaw: float
    pan_deg: float = 0.0
    tilt_deg: float = -45.0
    reason: str = ""


def encode_command(command: DjiCommand) -> str:
    validate_command(command)
    payload = asdict(command)
    payload["protocol"] = COMMAND_PROTOCOL
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def decode_command(data: str) -> DjiCommand:
    payload = json.loads(data)
    if payload.get("protocol") != COMMAND_PROTOCOL:
        raise ValueError("unsupported DJI command protocol")
    command = DjiCommand(
        uav_id=str(payload["uav_id"]).strip(),
        sequence=int(payload["sequence"]),
        stamp_ns=int(payload["stamp_ns"]),
        kind=str(payload["kind"]).strip().lower(),
        x=float(payload["x"]),
        y=float(payload["y"]),
        z=float(payload["z"]),
        yaw=float(payload["yaw"]),
        pan_deg=float(payload.get("pan_deg", 0.0)),
        tilt_deg=float(payload.get("tilt_deg", -45.0)),
        reason=str(payload.get("reason", "")),
    )
    validate_command(command)
    return command


def validate_command(command: DjiCommand) -> None:
    if not command.uav_id or command.sequence < 0 or command.stamp_ns < 0:
        raise ValueError("invalid DJI command identity, sequence, or timestamp")
    if command.kind not in COMMAND_KINDS:
        raise ValueError(f"unsupported DJI command kind: {command.kind}")
    values = (
        command.x, command.y, command.z, command.yaw,
        command.pan_deg, command.tilt_deg,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("DJI command values must be finite")
    if not -math.pi <= command.yaw <= math.pi:
        raise ValueError("yaw must be in radians within [-pi, pi]")
    if not -180.0 <= command.pan_deg <= 180.0:
        raise ValueError("pan must be within [-180, 180] degrees")
    if not -90.0 <= command.tilt_deg <= 30.0:
        raise ValueError("tilt must be within [-90, 30] degrees")


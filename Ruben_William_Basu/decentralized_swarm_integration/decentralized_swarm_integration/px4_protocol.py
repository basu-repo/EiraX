"""Pure coordinate conversion and safety checks for PX4 offboard control."""

from __future__ import annotations

from dataclasses import dataclass
import math


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class SafeSetpoint:
    north: float
    east: float
    down: float
    yaw_ned: float


def ned_to_enu(north: float, east: float, down: float) -> Point3:
    return Point3(east, north, -down)


def enu_to_ned(x: float, y: float, z: float) -> Point3:
    return Point3(y, x, -z)


def yaw_ned_to_enu(yaw_ned: float) -> float:
    return wrap_pi(math.pi / 2.0 - yaw_ned)


def yaw_enu_to_ned(yaw_enu: float) -> float:
    return wrap_pi(math.pi / 2.0 - yaw_enu)


def validate_setpoint(
    *,
    x_enu: float,
    y_enu: float,
    z_enu: float,
    yaw_enu: float,
    home_enu: Point3,
    current_enu: Point3 | None,
    max_radius_m: float,
    min_altitude_m: float,
    max_altitude_m: float,
    max_step_m: float,
) -> SafeSetpoint:
    values = (x_enu, y_enu, z_enu, yaw_enu)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("setpoint contains a non-finite value")
    relative_altitude = z_enu - home_enu.z
    if not min_altitude_m <= relative_altitude <= max_altitude_m:
        raise ValueError("setpoint altitude is outside the safety envelope")
    radius = math.hypot(x_enu - home_enu.x, y_enu - home_enu.y)
    if radius > max_radius_m:
        raise ValueError("setpoint exceeds maximum home radius")
    if current_enu is not None:
        step = math.dist(
            (x_enu, y_enu, z_enu),
            (current_enu.x, current_enu.y, current_enu.z),
        )
        if step > max_step_m:
            raise ValueError("setpoint step exceeds maximum allowed distance")
    # PX4 LOCAL_NED is relative to the vehicle's initialized home, whereas
    # mission setpoints are expressed in the shared ROS map frame.
    ned = enu_to_ned(
        x_enu - home_enu.x,
        y_enu - home_enu.y,
        z_enu - home_enu.z,
    )
    return SafeSetpoint(ned.x, ned.y, ned.z, yaw_enu_to_ned(yaw_enu))

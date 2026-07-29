#!/usr/bin/env python3
"""Fly a PX4 x500 circuit and land at its launch position."""

from __future__ import annotations

import argparse
import csv
import math
import threading
import time
from pathlib import Path
import sys

try:
    from pymavlink import mavutil
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "px4_runtime/python"))
    from pymavlink import mavutil


def wait_for_position(connection, timeout: float = 300.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = connection.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if message and message.lat and message.lon:
            return message.lat / 1e7, message.lon / 1e7, message.alt / 1000.0
    raise TimeoutError("PX4 did not provide a valid global position")


def command_and_wait(connection, command: int, *parameters: float, timeout: float = 10.0) -> None:
    values = list(parameters) + [0.0] * (7 - len(parameters))
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        command,
        0,
        *values[:7],
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        acknowledgement = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if acknowledgement and acknowledgement.command == command:
            if acknowledgement.result in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                return
            raise RuntimeError(f"PX4 rejected command {command}: result {acknowledgement.result}")
    raise TimeoutError(f"No acknowledgement for PX4 command {command}")


POSITION_ONLY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


def send_local_setpoint(connection, north: float, east: float, down: float) -> None:
    connection.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        POSITION_ONLY_MASK,
        north,
        east,
        down,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def fly(port: int, output: Path, radius: float, altitude: float, timeout: float) -> int:
    output.parent.mkdir(parents=True, exist_ok=True)
    connection = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}", source_system=250)
    if connection.wait_heartbeat(timeout=15) is None:
        raise TimeoutError(f"No PX4 heartbeat received on UDP port {port}")
    print("[OK] Connected to PX4 x500")

    heartbeat_stop = threading.Event()

    def send_heartbeats() -> None:
        while not heartbeat_stop.is_set():
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            heartbeat_stop.wait(0.8)

    heartbeat_thread = threading.Thread(target=send_heartbeats, daemon=True)
    heartbeat_thread.start()
    setpoint_stop: threading.Event | None = None
    setpoint_thread: threading.Thread | None = None

    try:
        home_latitude, home_longitude, home_altitude = wait_for_position(connection)
        print(
            f"[HOME] latitude={home_latitude:.7f}, longitude={home_longitude:.7f}, "
            f"altitude={home_altitude:.2f} m"
        )

        # North-East-Down circuit: climb, fly a square, return above launch, land.
        targets = [
            (0.0, 0.0, -altitude),
            (radius, -radius, -altitude),
            (radius, radius, -altitude),
            (-radius, radius, -altitude),
            (-radius, -radius, -altitude),
            (0.0, 0.0, -altitude),
        ]

        # PX4 requires a stream of setpoints before it permits offboard mode.
        active_target = list(targets[0])
        setpoint_stop = threading.Event()

        def stream_setpoints() -> None:
            while setpoint_stop is not None and not setpoint_stop.is_set():
                send_local_setpoint(connection, *active_target)
                setpoint_stop.wait(0.1)

        setpoint_thread = threading.Thread(target=stream_setpoints, daemon=True)
        setpoint_thread.start()
        time.sleep(2.0)
        command_and_wait(
            connection,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1.0,
        )
        print("[ARMED] Motors enabled")
        custom_mode = 6 << 16  # PX4 main mode 6: OFFBOARD
        connection.mav.set_mode_send(
            connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode,
        )
        mode_deadline = time.monotonic() + 10.0
        while time.monotonic() < mode_deadline:
            heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if heartbeat and ((int(heartbeat.custom_mode) >> 16) & 0xFF) == 6:
                break
        else:
            raise TimeoutError("PX4 did not enter offboard mode")
        print("[MISSION] takeoff -> circuit -> launch point -> land")

        start = time.monotonic()
        highest_relative_altitude = 0.0
        final_distance = float("inf")
        last_report = -10.0
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ["elapsed_sec", "latitude_deg", "longitude_deg", "relative_altitude_m", "distance_from_home_m"]
            )
            target_index = 0
            latest_local = None
            while time.monotonic() - start < timeout and target_index < len(targets):
                elapsed = time.monotonic() - start
                active_target[:] = targets[target_index]
                message = connection.recv_match(
                    type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"],
                    blocking=True,
                    timeout=0.05,
                )
                if message is None:
                    continue
                if message.get_type() == "LOCAL_POSITION_NED":
                    latest_local = message
                    target = targets[target_index]
                    target_error = math.sqrt(
                        (message.x - target[0]) ** 2
                        + (message.y - target[1]) ** 2
                        + (message.z - target[2]) ** 2
                    )
                    if target_error <= 1.0:
                        print(f"[REACHED] circuit point {target_index + 1}/{len(targets)}")
                        target_index += 1
                if message.get_type() == "GLOBAL_POSITION_INT":
                    latitude = message.lat / 1e7
                    longitude = message.lon / 1e7
                    relative_altitude = message.relative_alt / 1000.0
                    north = math.radians(latitude - home_latitude) * 6_378_137.0
                    east = (
                        math.radians(longitude - home_longitude)
                        * 6_378_137.0
                        * math.cos(math.radians(home_latitude))
                    )
                    final_distance = math.hypot(north, east)
                    highest_relative_altitude = max(highest_relative_altitude, relative_altitude)
                    writer.writerow(
                        [f"{elapsed:.3f}", f"{latitude:.7f}", f"{longitude:.7f}", f"{relative_altitude:.3f}", f"{final_distance:.3f}"]
                    )
                    stream.flush()
                    if elapsed - last_report >= 5.0:
                        print(
                            f"[FLIGHT] t={elapsed:.0f} s, altitude={relative_altitude:.1f} m, "
                            f"home distance={final_distance:.1f} m"
                        )
                        last_report = elapsed
            if target_index < len(targets):
                print("[SAFETY] Circuit timeout; commanding land")
            else:
                print("[RETURNED] Vehicle is above the launch point; commanding land")
            command_and_wait(connection, mavutil.mavlink.MAV_CMD_NAV_LAND)
            landing_deadline = time.monotonic() + 60.0
            while time.monotonic() < landing_deadline:
                heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                if heartbeat and not (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    print(
                        f"[LANDED] maximum altitude={highest_relative_altitude:.2f} m, "
                        f"final home error={final_distance:.2f} m"
                    )
                    return 0 if target_index == len(targets) and final_distance <= 2.5 else 2
            raise TimeoutError("PX4 did not confirm landing and disarming")
    finally:
        if setpoint_stop is not None:
            setpoint_stop.set()
        if setpoint_thread is not None:
            setpoint_thread.join(timeout=2)
        heartbeat_stop.set()
        heartbeat_thread.join(timeout=2)
        connection.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=14540)
    parser.add_argument("--radius", type=float, default=10.0, help="Circuit half-width in metres")
    parser.add_argument("--altitude", type=float, default=5.0, help="Flight altitude in metres")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--output", type=Path, default=Path("datasets/uav_x500_flight.csv"))
    args = parser.parse_args()
    try:
        return fly(args.port, args.output, args.radius, args.altitude, args.timeout)
    except (RuntimeError, TimeoutError) as error:
        print(f"[FAILED] {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

"""Minimal PX4 mission: spawn, take off, fly to the goal, land and disarm."""

import csv
import math
from pathlib import Path
import sys
import threading
import time


HERE = Path(__file__).resolve().parent
PX4_RUNTIME = HERE.parents[1] / "UGV_UAV/px4_runtime"
sys.path.insert(0, str(PX4_RUNTIME / "python"))

from pymavlink import mavutil  # noqa: E402


POSITION_ONLY_MASK = 0
for _ignored in ("VX", "VY", "VZ", "AX", "AY", "AZ", "YAW", "YAW_RATE"):
    POSITION_ONLY_MASK |= getattr(
        mavutil.mavlink, f"POSITION_TARGET_TYPEMASK_{_ignored}_IGNORE")


# Send a MAVLink command and wait for PX4 to acknowledge it.
def command_and_wait(connection, command, *parameters, timeout=10.0):
    values = (list(parameters) + [0.0] * 7)[:7]
    connection.mav.command_long_send(
        connection.target_system, connection.target_component, command, 0, *values)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ack = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if not ack or ack.command != command:
            continue
        if ack.result in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                          mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
            return
        raise RuntimeError(f"PX4 rejected command {command}: result {ack.result}")
    raise TimeoutError(f"No acknowledgement for PX4 command {command}")


# Send one position setpoint in the local NED frame.
def send_local_setpoint(connection, north, east, down):
    connection.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, POSITION_ONLY_MASK,
        north, east, down,
        *(0.0,) * 8)


# Block until PX4 reports a usable global position.
def wait_for_position(connection, timeout=300.0):
    deadline = time.monotonic() + timeout
    next_report = 0.0
    while time.monotonic() < deadline:
        message = connection.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if message and message.lat and message.lon:
            return
        if time.monotonic() >= next_report:
            print("[WAITING] Click Play; waiting for PX4 position and heading.")
            next_report = time.monotonic() + 10.0
    raise TimeoutError(f"PX4 gave no valid position within {timeout:.0f} seconds")


# Set PX4's horizontal speed limits, confirming each parameter.
def set_speed(connection, speed_mps):
    for name in ("MPC_XY_VEL_MAX", "MPC_XY_CRUISE"):
        connection.mav.param_set_send(
            connection.target_system, connection.target_component,
            name.encode("ascii"), speed_mps, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        deadline = time.monotonic() + 5.0
        confirmed = False
        while not confirmed and time.monotonic() < deadline:
            value = connection.recv_match(
                type="PARAM_VALUE", blocking=True, timeout=1)
            confirmed = bool(value and value.param_id.rstrip("\x00") == name)
        if not confirmed:
            raise TimeoutError(f"PX4 did not confirm {name}")


# Fly the route as far as the named endpoint, then land and disarm.
def fly_mission(*, port, output_directory, route, endpoint, altitude_m,
                speed_mps, timeout_seconds, duration_seconds=None,
                source_system=250):
    names = [name for name, _, _, _ in route]
    if endpoint not in names:
        raise ValueError(f"endpoint {endpoint!r} is not in the route: {names}")
    chosen = route[:names.index(endpoint) + 1]
    final_north, final_east, final_ground_down = chosen[-1][1:]

    outward = [(name, north, east, -altitude_m, 3.0) for name, north, east, _ in chosen]
    legs = [("takeoff", 0.0, 0.0, -altitude_m, 1.5)]
    if duration_seconds:
        # Patrol out along the route and back to spawn, repeated until the
        # mission is long enough. The flight still lands early once the
        # requested duration is reached, so the lap count only has to be enough.
        homeward = [(f"{name} back", north, east, down, radius)
                    for name, north, east, down, radius in reversed(outward[:-1])]
        homeward.append(("spawn", 0.0, 0.0, -altitude_m, 3.0))
        lap = outward + homeward
        span = sum(math.dist(a[1:3], b[1:3]) for a, b in zip(lap, lap[1:]))
        laps = max(1, math.ceil(duration_seconds * speed_mps / max(span, 1.0)))
        legs += lap * laps
        print(f"[PATROL] {laps} lap(s) of {span:.0f} m for a {duration_seconds:.0f}s mission")
    else:
        # Only an untimed mission flies to the endpoint and lands there. A timed
        # one lands wherever the clock runs out, so it does not overshoot.
        legs += outward
        legs.append(("approach", final_north, final_east, final_ground_down - 2.0, 1.0))

    output_directory.mkdir(parents=True, exist_ok=True)
    # Each drone needs its own ground station identity. Three connections
    # claiming one system id makes PX4 route acknowledgements ambiguously.
    connection = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}",
                                            source_system=source_system)
    if connection.wait_heartbeat(timeout=30) is None:
        raise TimeoutError(f"No PX4 heartbeat on UDP port {port}")
    print("[OK] Connected to PX4")

    stop = threading.Event()
    target = [0.0, 0.0, -altitude_m]

    # PX4 leaves offboard mode unless a heartbeat and a setpoint keep arriving.
    def keep_alive():
        while not stop.is_set():
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            send_local_setpoint(connection, *target)
            stop.wait(0.1)

    keeper = threading.Thread(target=keep_alive, daemon=True)
    keeper.start()

    try:
        wait_for_position(connection)
        set_speed(connection, speed_mps)
        print(f"[OK] Position ready, speed limit {speed_mps:.1f} m/s")

        arm_deadline = time.monotonic() + 45.0
        while True:
            try:
                command_and_wait(
                    connection, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    1.0, timeout=5.0)
                break
            except RuntimeError:
                if time.monotonic() >= arm_deadline:
                    raise RuntimeError("PX4 preflight checks never became ready")
                print("[WAITING] PX4 preflight checks are not ready yet.")
                time.sleep(2.0)
        print("[ARMED] Motors enabled")

        connection.mav.set_mode_send(
            connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6 << 16)
        mode_deadline = time.monotonic() + 10.0
        while time.monotonic() < mode_deadline:
            beat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if beat and ((int(beat.custom_mode) >> 16) & 0xFF) == 6:
                break
        else:
            raise TimeoutError("PX4 did not enter offboard mode")
        print("[OFFBOARD] Accepting position setpoints")

        print(f"[MISSION] {' -> '.join(name for name, *_ in legs)} "
              f"({math.hypot(final_north, final_east):.0f} m to {endpoint})")

        start = time.monotonic()
        leg = 0
        error = float("inf")
        with (output_directory / "mission_trajectory.csv").open(
                "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ["elapsed_sec", "phase", "north_m", "east_m", "down_m", "endpoint_error_m"])
            while leg < len(legs) and time.monotonic() - start < timeout_seconds:
                # A timed mission stops patrolling and goes to the landing leg.
                if duration_seconds and time.monotonic() - start >= duration_seconds:
                    print(f"[PATROL] {duration_seconds:.0f}s flown, landing here")
                    leg = len(legs)
                    break
                phase, north, east, down, radius = legs[leg]
                target[:] = [north, east, down]
                position = connection.recv_match(
                    type="LOCAL_POSITION_NED", blocking=True, timeout=0.2)
                if position is None:
                    continue
                error = math.hypot(position.x - final_north, position.y - final_east)
                writer.writerow([
                    f"{time.monotonic() - start:.3f}", phase,
                    f"{position.x:.3f}", f"{position.y:.3f}", f"{position.z:.3f}",
                    f"{error:.3f}"])
                remaining = math.dist(
                    (position.x, position.y, position.z), (north, east, down))
                if remaining <= radius:
                    print(f"[REACHED] {phase} within {remaining:.2f} m")
                    leg += 1

        reached = leg >= len(legs)
        print(f"[LANDING] At {endpoint}." if reached
              else "[TIMEOUT] Landing at the current position.")
        command_and_wait(connection, mavutil.mavlink.MAV_CMD_NAV_LAND)

        landed = False
        land_deadline = time.monotonic() + 120.0
        while not landed and time.monotonic() < land_deadline:
            beat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            landed = bool(beat) and not (
                beat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        print(f"[LANDED] Disarmed, {endpoint} error {error:.2f} m" if landed
              else "[FAILED] PX4 did not confirm landing and disarming.")
        return 0 if reached and landed else 2
    finally:
        stop.set()
        keeper.join(timeout=2)
        connection.close()

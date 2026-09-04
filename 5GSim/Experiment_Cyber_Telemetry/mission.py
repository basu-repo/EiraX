"""Minimal PX4 mission: take off, fly the route or hover, land and disarm."""

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


# Fly the route, land and disarm. An untimed mission lands at the goal. A timed
# one patrols out and back along the route until the duration is reached, or
# holds the takeoff point when the route is empty, then returns to spawn to land.
def fly_mission(*, port, output_directory, route, altitude_m, speed_mps,
                timeout_seconds, duration_seconds=None, source_system=250):
    outward = [(name, north, east, -altitude_m, 3.0) for name, north, east, _ in route]
    legs = [("takeoff", 0.0, 0.0, -altitude_m, 1.5)]
    if duration_seconds:
        homeward = [(f"{name} back", north, east, down, radius)
                    for name, north, east, down, radius in reversed(outward[:-1])]
        if outward:
            homeward.append(("spawn", 0.0, 0.0, -altitude_m, 3.0))
        legs += outward + homeward
    else:
        final_north, final_east, final_ground_down = route[-1][1:]
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

        print(f"[MISSION] {' -> '.join(name for name, *_ in legs)}"
              + (f" for {duration_seconds:.0f}s" if duration_seconds else ""))

        start = time.monotonic()
        leg = 0
        done = False
        with (output_directory / "mission_trajectory.csv").open(
                "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["elapsed_sec", "phase", "north_m", "east_m", "down_m"])
            while time.monotonic() - start < timeout_seconds:
                if duration_seconds and time.monotonic() - start >= duration_seconds:
                    # Landing far along the route is not reliable, so a timed
                    # mission flies back to spawn and lands where a route flight does.
                    print(f"[PATROL] {duration_seconds:.0f}s flown, returning to spawn to land")
                    legs, leg, duration_seconds = [("return", 0.0, 0.0, -altitude_m, 3.0)], 0, None
                if leg >= len(legs) and not duration_seconds:
                    done = True
                    break
                # A hover has only the takeoff leg and holds it until the clock runs out.
                holding = leg >= len(legs)
                phase, north, east, down, radius = legs[min(leg, len(legs) - 1)]
                target[:] = [north, east, down]
                position = connection.recv_match(
                    type="LOCAL_POSITION_NED", blocking=True, timeout=0.2)
                if position is None:
                    continue
                writer.writerow([
                    f"{time.monotonic() - start:.3f}", phase,
                    f"{position.x:.3f}", f"{position.y:.3f}", f"{position.z:.3f}"])
                remaining = math.dist(
                    (position.x, position.y, position.z), (north, east, down))
                if remaining <= radius and not holding:
                    print(f"[REACHED] {phase} within {remaining:.2f} m")
                    leg += 1
                    if leg >= len(legs) and duration_seconds and len(legs) > 1:
                        leg = 1  # next patrol lap, skipping the takeoff leg

        print("[LANDING]" if done else "[TIMEOUT] Landing at the current position.")
        command_and_wait(connection, mavutil.mavlink.MAV_CMD_NAV_LAND)

        landed = False
        land_deadline = time.monotonic() + 120.0
        while not landed and time.monotonic() < land_deadline:
            beat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            landed = bool(beat) and not (
                beat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        print("[LANDED] Disarmed" if landed
              else "[FAILED] PX4 did not confirm landing and disarming.")
        return 0 if done and landed else 2
    finally:
        stop.set()
        keeper.join(timeout=2)
        connection.close()

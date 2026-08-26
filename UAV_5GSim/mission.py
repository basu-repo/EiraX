"""Minimal PX4 mission: spawn, take off, fly to the goal, land and disarm."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import sys
import threading
import time


HERE = Path(__file__).resolve().parent
PX4_RUNTIME = HERE.parent / "UGV_UAV/px4_runtime"
sys.path.insert(0, str(PX4_RUNTIME / "python"))

from pymavlink import mavutil  # noqa: E402


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


def command_and_wait(
    connection,
    command: int,
    *parameters: float,
    timeout: float = 10.0,
) -> None:
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
        acknowledgement = connection.recv_match(
            type="COMMAND_ACK", blocking=True, timeout=1
        )
        if acknowledgement and acknowledgement.command == command:
            if acknowledgement.result in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                return
            raise RuntimeError(
                f"PX4 rejected command {command}: result {acknowledgement.result}"
            )
    raise TimeoutError(f"No acknowledgement for PX4 command {command}")


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


def wait_for_position(connection, timeout: float = 300.0) -> None:
    deadline = time.monotonic() + timeout
    next_report = time.monotonic()
    while time.monotonic() < deadline:
        message = connection.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=1
        )
        if message and message.lat and message.lon:
            return
        if time.monotonic() >= next_report:
            print("[WAITING] Click Play; waiting for PX4 position and heading.")
            next_report = time.monotonic() + 10.0
    raise TimeoutError("PX4 did not provide a valid position within 300 seconds")


def set_speed(connection, speed_mps: float) -> None:
    for parameter_name in ("MPC_XY_VEL_MAX", "MPC_XY_CRUISE"):
        connection.mav.param_set_send(
            connection.target_system,
            connection.target_component,
            parameter_name.encode("ascii"),
            speed_mps,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            value = connection.recv_match(
                type="PARAM_VALUE", blocking=True, timeout=1
            )
            if value and value.param_id.rstrip("\x00") == parameter_name:
                break
        else:
            raise TimeoutError(f"PX4 did not confirm {parameter_name}")


def fly_to_goal(
    *,
    port: int,
    output_directory: Path,
    goal_north_m: float,
    goal_east_m: float,
    goal_ground_down_m: float,
    altitude_m: float,
    speed_mps: float,
    timeout_seconds: float,
) -> int:
    output_directory.mkdir(parents=True, exist_ok=True)
    connection = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{port}", source_system=250
    )
    if connection.wait_heartbeat(timeout=30) is None:
        raise TimeoutError(f"No PX4 heartbeat received on UDP port {port}")
    print("[OK] Connected to PX4")

    heartbeat_stop = threading.Event()
    setpoint_stop = threading.Event()
    setpoint_thread: threading.Thread | None = None
    active_target = [0.0, 0.0, -altitude_m]
    armed = False
    landing_commanded = False

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

    try:
        wait_for_position(connection)
        print("[OK] PX4 position and heading are ready")
        set_speed(connection, speed_mps)
        print(f"[SPEED] Horizontal speed limit: {speed_mps:.1f} m/s")

        def stream_setpoints() -> None:
            while not setpoint_stop.is_set():
                send_local_setpoint(connection, *active_target)
                setpoint_stop.wait(0.1)

        setpoint_thread = threading.Thread(target=stream_setpoints, daemon=True)
        setpoint_thread.start()
        time.sleep(2.0)

        arm_deadline = time.monotonic() + 45.0
        while True:
            try:
                command_and_wait(
                    connection,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    1.0,
                    timeout=5.0,
                )
                armed = True
                break
            except RuntimeError:
                if time.monotonic() >= arm_deadline:
                    raise RuntimeError(
                        "PX4 preflight checks did not become ready within 45 seconds"
                    )
                print("[WAITING] PX4 preflight checks are not ready yet.")
                time.sleep(2.0)
        print("[ARMED] Motors enabled")

        connection.mav.set_mode_send(
            connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6 << 16,
        )
        mode_deadline = time.monotonic() + 10.0
        while time.monotonic() < mode_deadline:
            heartbeat = connection.recv_match(
                type="HEARTBEAT", blocking=True, timeout=1
            )
            if heartbeat and ((int(heartbeat.custom_mode) >> 16) & 0xFF) == 6:
                break
        else:
            raise TimeoutError("PX4 did not enter offboard mode")

        targets = [
            ("takeoff", 0.0, 0.0, -altitude_m, 1.5),
            ("goal", goal_north_m, goal_east_m, -altitude_m, 3.0),
            (
                "landing_approach",
                goal_north_m,
                goal_east_m,
                goal_ground_down_m - 2.0,
                1.0,
            ),
        ]
        print(
            "[MISSION] spawn -> takeoff -> goal -> land "
            f"(distance {math.hypot(goal_north_m, goal_east_m):.1f} m)"
        )
        trajectory_path = output_directory / "mission_trajectory.csv"
        start = time.monotonic()
        target_index = 0
        last_report = -10.0
        final_horizontal_error = float("inf")

        with trajectory_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "elapsed_sec",
                    "phase",
                    "north_m",
                    "east_m",
                    "down_m",
                    "target_distance_m",
                    "goal_horizontal_error_m",
                ]
            )
            while (
                time.monotonic() - start < timeout_seconds
                and target_index < len(targets)
            ):
                phase, north, east, down, radius = targets[target_index]
                active_target[:] = [north, east, down]
                message = connection.recv_match(
                    type="LOCAL_POSITION_NED", blocking=True, timeout=0.2
                )
                if message is None:
                    continue
                elapsed = time.monotonic() - start
                target_distance = math.sqrt(
                    (message.x - north) ** 2
                    + (message.y - east) ** 2
                    + (message.z - down) ** 2
                )
                final_horizontal_error = math.hypot(
                    message.x - goal_north_m,
                    message.y - goal_east_m,
                )
                writer.writerow(
                    [
                        f"{elapsed:.3f}",
                        phase,
                        f"{message.x:.3f}",
                        f"{message.y:.3f}",
                        f"{message.z:.3f}",
                        f"{target_distance:.3f}",
                        f"{final_horizontal_error:.3f}",
                    ]
                )
                stream.flush()
                if elapsed - last_report >= 5.0:
                    print(
                        f"[FLIGHT] {phase}: distance={target_distance:.1f} m, "
                        f"goal error={final_horizontal_error:.1f} m"
                    )
                    last_report = elapsed
                if target_distance <= radius:
                    print(f"[REACHED] {phase} within {target_distance:.2f} m")
                    target_index += 1

        mission_completed = target_index == len(targets)
        if mission_completed:
            print("[GOAL REACHED] Descending and landing at the goal.")
        else:
            print("[SAFETY] Mission timeout; landing at the current position.")

        command_and_wait(connection, mavutil.mavlink.MAV_CMD_NAV_LAND)
        landing_commanded = True
        landing_deadline = time.monotonic() + 120.0
        landed = False
        while time.monotonic() < landing_deadline:
            message = connection.recv_match(
                type=["HEARTBEAT", "LOCAL_POSITION_NED"],
                blocking=True,
                timeout=1,
            )
            if message is None:
                continue
            if message.get_type() == "LOCAL_POSITION_NED":
                final_horizontal_error = math.hypot(
                    message.x - goal_north_m,
                    message.y - goal_east_m,
                )
            elif not (
                message.base_mode
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ):
                landed = True
                armed = False
                break

        summary = {
            "mission_completed": mission_completed,
            "landed_and_disarmed": landed,
            "goal_north_m": goal_north_m,
            "goal_east_m": goal_east_m,
            "goal_ground_down_m": goal_ground_down_m,
            "landing_approach_height_m": 2.0,
            "flight_altitude_m": altitude_m,
            "speed_limit_mps": speed_mps,
            "final_goal_horizontal_error_m": final_horizontal_error,
            "ground_truth_used_for_control": False,
        }
        (output_directory / "mission_summary.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )
        if landed:
            print(
                "[LANDED] UAV disarmed on the ground; "
                f"horizontal goal error={final_horizontal_error:.2f} m"
            )
        else:
            print("[FAILED] PX4 did not confirm landing and disarming.")
        return 0 if mission_completed and landed else 2
    finally:
        if armed and not landing_commanded:
            try:
                command_and_wait(
                    connection,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    timeout=3.0,
                )
            except (RuntimeError, TimeoutError):
                pass
        setpoint_stop.set()
        if setpoint_thread is not None:
            setpoint_thread.join(timeout=2)
        heartbeat_stop.set()
        heartbeat_thread.join(timeout=2)
        connection.close()

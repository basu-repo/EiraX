"""Fly from the Baylands spawn to a saved waypoint, return, and evaluate pose."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import statistics
import sys
import threading
import time

try:
    from pymavlink import mavutil
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "px4_runtime/python"))
    from pymavlink import mavutil

from uav.gazebo_pose import GazeboPoseReader
from uav.x500_mission import command_and_wait, send_local_setpoint, wait_for_position


def fly_waypoint(
    port: int,
    output_directory: Path,
    north_m: float,
    east_m: float,
    altitude_m: float = 30.0,
    hover_seconds: float = 5.0,
    timeout_seconds: float = 240.0,
    model_name: str = "x500_0",
    speed_mps: float | None = None,
    local_mapping_seconds: float | None = None,
    mapping_radius_m: float = 5.0,
) -> int:
    output_directory.mkdir(parents=True, exist_ok=True)
    connection = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}", source_system=250)
    if connection.wait_heartbeat(timeout=20) is None:
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
    pose_reader = GazeboPoseReader("baylands_editable", model_name=model_name)
    setpoint_stop = threading.Event()
    setpoint_thread: threading.Thread | None = None

    try:
        latitude, longitude, altitude = wait_for_position(connection)
        ground_origin = pose_reader.wait()
        print(f"[SPAWN] latitude={latitude:.7f}, longitude={longitude:.7f}, altitude={altitude:.2f} m")
        print(
            f"[WAYPOINT] north={north_m:.2f} m, east={east_m:.2f} m, "
            f"flight altitude={altitude_m:.1f} m"
        )
        if speed_mps is not None:
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
                    value = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
                    if value and value.param_id.rstrip("\x00") == parameter_name:
                        break
                else:
                    raise TimeoutError(f"PX4 did not confirm parameter {parameter_name}")
            print(f"[SPEED] Horizontal mapping speed limited to {speed_mps:.1f} m/s")

        mapping_mode = local_mapping_seconds is not None
        reach_radius = 0.75 if mapping_mode else 1.5
        if mapping_mode:
            mapping_speed = speed_mps or 1.0
            loop_seconds = 8.0 * mapping_radius_m / mapping_speed
            loop_count = max(1, math.ceil(local_mapping_seconds / loop_seconds))
            corners = [
                (mapping_radius_m, -mapping_radius_m),
                (mapping_radius_m, mapping_radius_m),
                (-mapping_radius_m, mapping_radius_m),
                (-mapping_radius_m, -mapping_radius_m),
            ]
            targets = [(0.0, 0.0, -altitude_m, 0.0, "takeoff")]
            for loop_index in range(loop_count):
                targets.extend(
                    (
                        north,
                        east,
                        -altitude_m,
                        0.0,
                        f"mapping_loop_{loop_index + 1}_point_{point_index + 1}",
                    )
                    for point_index, (north, east) in enumerate(corners)
                )
            targets.append((0.0, 0.0, -altitude_m, 0.0, "spawn_return"))
            timeout_seconds = max(timeout_seconds, local_mapping_seconds + 120.0)
            print(
                f"[MAPPING PATTERN] {loop_count} local loops, radius={mapping_radius_m:.1f} m, "
                f"planned mapping time≈{local_mapping_seconds:.0f} s"
            )
        else:
            targets = [
                (0.0, 0.0, -altitude_m, 0.0, "takeoff"),
                (north_m, east_m, -altitude_m, hover_seconds, "waypoint_1"),
                (0.0, 0.0, -altitude_m, 0.0, "spawn_return"),
            ]
        active_target = [targets[0][0], targets[0][1], targets[0][2]]

        def stream_setpoints() -> None:
            while not setpoint_stop.is_set():
                send_local_setpoint(connection, *active_target)
                setpoint_stop.wait(0.1)

        setpoint_thread = threading.Thread(target=stream_setpoints, daemon=True)
        setpoint_thread.start()
        time.sleep(2.0)
        arm_deadline = time.monotonic() + 30.0
        while True:
            try:
                command_and_wait(
                    connection,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    1.0,
                    timeout=5.0,
                )
                break
            except RuntimeError:
                if time.monotonic() >= arm_deadline:
                    raise RuntimeError("PX4 preflight checks did not become ready within 30 seconds")
                print("[WAITING] PX4 preflight heading estimate is not ready yet.")
                time.sleep(2.0)
        print("[ARMED] Motors enabled")
        offboard_mode = 6 << 16
        connection.mav.set_mode_send(
            connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            offboard_mode,
        )
        mode_deadline = time.monotonic() + 10.0
        while time.monotonic() < mode_deadline:
            heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if heartbeat and ((int(heartbeat.custom_mode) >> 16) & 0xFF) == 6:
                break
        else:
            raise TimeoutError("PX4 did not enter offboard mode")

        if mapping_mode:
            print("[MISSION] spawn -> local mapping loops -> spawn -> land")
        else:
            print("[MISSION] spawn -> waypoint_1 -> hover -> spawn -> land")
        csv_path = output_directory / "pose_comparison.csv"
        errors: list[float] = []
        target_index = 0
        reached_at: float | None = None
        start = time.monotonic()
        last_report = -10.0
        latest_global = None
        final_spawn_error = float("inf")
        waypoint_error: float | None = None

        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "elapsed_sec", "simulation_time_sec", "phase", "px4_north_m", "px4_east_m", "px4_down_m",
                    "gazebo_north_m", "gazebo_east_m", "gazebo_down_m",
                    "position_error_m", "latitude_deg", "longitude_deg", "relative_altitude_m",
                ]
            )
            while time.monotonic() - start < timeout_seconds and target_index < len(targets):
                target = targets[target_index]
                active_target[:] = target[:3]
                message = connection.recv_match(
                    type=["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"],
                    blocking=True,
                    timeout=0.1,
                )
                if message is None:
                    continue
                elapsed = time.monotonic() - start
                if message.get_type() == "GLOBAL_POSITION_INT":
                    latest_global = message
                    continue

                ground = pose_reader.at(message.time_boot_ms / 1000.0)
                if ground is None:
                    continue
                # Gazebo is East-North-Up; PX4 is North-East-Down.
                gazebo_north = ground[1] - ground_origin[1]
                gazebo_east = ground[0] - ground_origin[0]
                gazebo_down = -(ground[2] - ground_origin[2])
                error = math.sqrt(
                    (message.x - gazebo_north) ** 2
                    + (message.y - gazebo_east) ** 2
                    + (message.z - gazebo_down) ** 2
                )
                errors.append(error)
                latitude_value = latest_global.lat / 1e7 if latest_global else float("nan")
                longitude_value = latest_global.lon / 1e7 if latest_global else float("nan")
                relative_altitude = latest_global.relative_alt / 1000.0 if latest_global else float("nan")
                writer.writerow(
                    [
                        f"{elapsed:.3f}", f"{message.time_boot_ms / 1000.0:.3f}", target[4],
                        f"{message.x:.3f}", f"{message.y:.3f}",
                        f"{message.z:.3f}", f"{gazebo_north:.3f}", f"{gazebo_east:.3f}",
                        f"{gazebo_down:.3f}", f"{error:.4f}", f"{latitude_value:.7f}",
                        f"{longitude_value:.7f}", f"{relative_altitude:.3f}",
                    ]
                )
                stream.flush()

                target_error = math.sqrt(
                    (message.x - target[0]) ** 2
                    + (message.y - target[1]) ** 2
                    + (message.z - target[2]) ** 2
                )
                if elapsed - last_report >= 5.0:
                    print(
                        f"[FLIGHT] {target[4]}: distance={target_error:.1f} m, "
                        f"localization error={error:.2f} m"
                    )
                    last_report = elapsed
                if target_error <= reach_radius:
                    if reached_at is None:
                        reached_at = time.monotonic()
                        print(f"[REACHED] {target[4]} within {target_error:.2f} m")
                        if target[4] == "waypoint_1":
                            waypoint_error = target_error
                    if time.monotonic() - reached_at >= target[3]:
                        if target[4] == "spawn_return":
                            final_spawn_error = target_error
                        target_index += 1
                        reached_at = None

        if target_index < len(targets):
            print("[SAFETY] Mission timeout; commanding land")
        else:
            print("[RETURNED] Vehicle is above spawn; commanding land")
        command_and_wait(connection, mavutil.mavlink.MAV_CMD_NAV_LAND)
        landing_deadline = time.monotonic() + 90.0
        landed = False
        while time.monotonic() < landing_deadline:
            heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if heartbeat and not (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                landed = True
                break

        summary = {
            "mission_completed": target_index == len(targets),
            "landed_and_disarmed": landed,
            "waypoint_north_m": None if mapping_mode else north_m,
            "waypoint_east_m": None if mapping_mode else east_m,
            "waypoint_reach_error_m": waypoint_error,
            "local_mapping_seconds_planned": local_mapping_seconds,
            "mapping_radius_m": mapping_radius_m if mapping_mode else None,
            "return_spawn_error_m": final_spawn_error,
            "localization_samples": len(errors),
            "localization_mean_error_m": statistics.fmean(errors) if errors else None,
            "localization_rmse_m": math.sqrt(statistics.fmean(value * value for value in errors)) if errors else None,
            "localization_max_error_m": max(errors) if errors else None,
            "ground_truth_used_for_control": False,
        }
        (output_directory / "localization_summary.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )
        if landed and mapping_mode:
            print(f"[LANDED] local mapping complete, return error={final_spawn_error:.2f} m")
        elif landed:
            print(
                f"[LANDED] waypoint error={waypoint_error:.2f} m, "
                f"return error={final_spawn_error:.2f} m"
            )
        return 0 if summary["mission_completed"] and landed else 2
    finally:
        setpoint_stop.set()
        if setpoint_thread is not None:
            setpoint_thread.join(timeout=2)
        pose_reader.close()
        heartbeat_stop.set()
        heartbeat_thread.join(timeout=2)
        connection.close()

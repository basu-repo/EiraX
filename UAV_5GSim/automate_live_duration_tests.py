#!/usr/bin/env python3
"""Run the document-required 5, 10 and 20 minute UAV/5G validations.

Each test starts a fresh headless Gazebo and PX4 instance, records MAVROS topics
to a ROS 2 bag, flies through the two document waypoints to the goal, remains
active for the requested mission duration, lands at the goal, exports the
measured ROS pose, runs a one-UAV Simu5G baseline, assembles the 44-column CSV,
and validates it. Runs are kept outside the canonical 46-run experiment set so
that an additional physical validation cannot silently change published data.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import gzip
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

import yaml


HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent
WORLD = HERE / "worlds/baylands_uav_experiment.world"
MODELS = PROJECT / "simulation/models"
PX4 = PROJECT / "UGV_UAV/px4_runtime"
SIMU5G = PROJECT / "UGV_UAV_5G_CoSimulation/sim/simu5g/Simu5G"
OMNETPP = Path("/home/basudeo/omnetpp-6.0.1")
INET = Path("/home/basudeo/inet")
SCHEMA = HERE / "schema_reference.json"
ASSEMBLER = HERE / "sim/bridge/assemble.py"
VALIDATOR = HERE / "validate_dataset.py"
OUTPUT_ROOT = HERE / "experiments/live_duration_validation"
NETWORK_TEMPLATE = (
    HERE / "experiments/logs/B_DURATION_20_001/network/input"
)

WORLD_NAME = "baylands_editable"
# Duration testing exercises flight and communication, not mapping.  The plain
# PX4 x500 avoids running the expensive GPU LiDAR for 20 unattended minutes.
UAV_ENTITY = "x500_duration_0"
SPAWN = (211.12966918945318, -256.3474426269531, 2.935702896118163)
WAYPOINTS = (
    ("waypoint_1", 133.30400085449219, -279.45498657226562),
    ("waypoint_2", 82.877403259277344, -241.2203369140625),
    ("goal", -29.546520233154297, -286.43212890625),
)

sys.path.insert(0, str(PX4 / "python"))
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


class ManagedProcess:
    def __init__(self, name: str, command: list[str], log: Path, **kwargs):
        self.name = name
        self.log_handle = log.open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            command,
            stdout=self.log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            **kwargs,
        )

    def stop(self) -> None:
        process = self.process
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        self.log_handle.close()


def disk_free_gib() -> float:
    return shutil.disk_usage(HERE).free / 1024**3


def require_environment() -> None:
    required = [
        WORLD,
        MODELS,
        PX4 / "bin/px4",
        PX4 / "rootfs",
        SIMU5G / "bin/simu5g",
        OMNETPP / "setenv",
        INET / "setenv",
        SCHEMA,
        ASSEMBLER,
        VALIDATOR,
        NETWORK_TEMPLATE / "EiraXExperiment.ned",
        NETWORK_TEMPLATE / "demo.xml",
        NETWORK_TEMPLATE / "flow_contract.json",
        NETWORK_TEMPLATE / "omnetpp.ini",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError("Required environment is missing:\n" + "\n".join(missing))
    services = subprocess.run(
        ["gz", "service", "-l"], capture_output=True, text=True, timeout=10
    )
    if "/server_control" in services.stdout.splitlines():
        raise RuntimeError("Another Gazebo server is already running")
    if disk_free_gib() < 15:
        raise RuntimeError("Less than 15 GiB free; duration tests were not started")


def send_setpoint(connection, north: float, east: float, down: float) -> None:
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


def command_and_wait(connection, command: int, *parameters: float, timeout=10.0) -> None:
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
                f"PX4 rejected command {command}: {acknowledgement.result}"
            )
    raise TimeoutError(f"No acknowledgement for PX4 command {command}")


def set_speed(connection, speed_mps: float) -> None:
    parameters = {
        "MPC_XY_VEL_MAX": speed_mps,
        "MPC_XY_CRUISE": speed_mps,
        # PX4 SITL defaults to a deliberately short simulated battery discharge.
        # A full-day discharge preserves battery publication while preventing an
        # artificial failsafe during the document's 20-minute endurance case.
        "SIM_BAT_DRAIN": 86400.0,
    }
    for parameter_name, parameter_value in parameters.items():
        connection.mav.param_set_send(
            connection.target_system,
            connection.target_component,
            parameter_name.encode("ascii"),
            parameter_value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            reply = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if reply and reply.param_id.rstrip("\x00") == parameter_name:
                break
        else:
            raise TimeoutError(f"PX4 did not confirm {parameter_name}")


def fly_for_duration(
    run_dir: Path, duration_sec: int, speed_mps: float, altitude_m: float
) -> dict:
    connection = mavutil.mavlink_connection(
        "udpin:0.0.0.0:14540", source_system=250
    )
    if connection.wait_heartbeat(timeout=45) is None:
        raise TimeoutError("No PX4 heartbeat on UDP 14540")

    heartbeat_stop = threading.Event()
    setpoint_stop = threading.Event()
    active_target = [0.0, 0.0, -altitude_m]
    setpoint_thread = None
    armed = False
    landing_commanded = False

    def heartbeat() -> None:
        while not heartbeat_stop.is_set():
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            heartbeat_stop.wait(0.8)

    heartbeat_thread = threading.Thread(target=heartbeat, daemon=True)
    heartbeat_thread.start()

    try:
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            position = connection.recv_match(
                type="GLOBAL_POSITION_INT", blocking=True, timeout=1
            )
            if position and position.lat and position.lon:
                break
        else:
            raise TimeoutError("PX4 position and heading did not become ready")
        set_speed(connection, speed_mps)

        def stream_setpoints() -> None:
            while not setpoint_stop.is_set():
                send_setpoint(connection, *active_target)
                setpoint_stop.wait(0.1)

        setpoint_thread = threading.Thread(target=stream_setpoints, daemon=True)
        setpoint_thread.start()
        time.sleep(2)

        arm_deadline = time.monotonic() + 60
        while True:
            try:
                command_and_wait(
                    connection,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    1.0,
                    timeout=5,
                )
                armed = True
                break
            except RuntimeError:
                if time.monotonic() >= arm_deadline:
                    raise RuntimeError("PX4 preflight checks never became ready")
                time.sleep(2)

        connection.mav.set_mode_send(
            connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6 << 16,
        )
        mode_deadline = time.monotonic() + 15
        while time.monotonic() < mode_deadline:
            status = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if status and ((int(status.custom_mode) >> 16) & 0xFF) == 6:
                break
        else:
            raise TimeoutError("PX4 did not enter offboard mode")

        relative_targets = [("takeoff", 0.0, 0.0, -altitude_m, 2.0)]
        for name, x, y in WAYPOINTS:
            relative_targets.append(
                (name, y - SPAWN[1], x - SPAWN[0], -altitude_m, 4.0)
            )

        trajectory = run_dir / "mavlink_pose_trace.csv"
        reached = []
        started = time.monotonic()
        last_report = -30.0
        current = None
        with trajectory.open("w", newline="", encoding="utf-8") as stream:
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

            for phase, north, east, down, radius in relative_targets:
                active_target[:] = [north, east, down]
                phase_deadline = time.monotonic() + 240
                while time.monotonic() < phase_deadline:
                    current = connection.recv_match(
                        type="LOCAL_POSITION_NED", blocking=True, timeout=0.25
                    )
                    if current is None:
                        continue
                    elapsed = time.monotonic() - started
                    distance = math.sqrt(
                        (current.x - north) ** 2
                        + (current.y - east) ** 2
                        + (current.z - down) ** 2
                    )
                    goal_error = math.hypot(
                        current.x - relative_targets[-1][1],
                        current.y - relative_targets[-1][2],
                    )
                    writer.writerow(
                        [elapsed, phase, current.x, current.y, current.z, distance, goal_error]
                    )
                    if distance <= radius:
                        reached.append(phase)
                        print(f"[REACHED] {phase} at {elapsed:.1f}s ({distance:.2f}m)", flush=True)
                        break
                else:
                    raise TimeoutError(f"Timed out while flying to {phase}")

            active_target[:] = list(relative_targets[-1][1:4])
            while time.monotonic() - started < duration_sec:
                current = connection.recv_match(
                    type="LOCAL_POSITION_NED", blocking=True, timeout=0.25
                )
                if current is None:
                    continue
                elapsed = time.monotonic() - started
                goal_error = math.hypot(
                    current.x - relative_targets[-1][1],
                    current.y - relative_targets[-1][2],
                )
                writer.writerow(
                    [elapsed, "goal_hold", current.x, current.y, current.z, goal_error, goal_error]
                )
                if elapsed - last_report >= 30:
                    print(
                        f"[ACTIVE] {elapsed:.0f}/{duration_sec}s; goal error={goal_error:.2f}m",
                        flush=True,
                    )
                    last_report = elapsed
                stream.flush()

        active_duration = time.monotonic() - started
        landing_deadline = time.monotonic() + 30
        while True:
            try:
                command_and_wait(connection, mavutil.mavlink.MAV_CMD_NAV_LAND)
                landing_commanded = True
                break
            except RuntimeError:
                if time.monotonic() >= landing_deadline:
                    raise
                # Reassert a safe goal hold before retrying a transient mode denial.
                active_target[:] = list(relative_targets[-1][1:4])
                time.sleep(2)
        landing_deadline = time.monotonic() + 150
        landed = False
        final_error = float("inf")
        while time.monotonic() < landing_deadline:
            message = connection.recv_match(
                type=["HEARTBEAT", "LOCAL_POSITION_NED"], blocking=True, timeout=1
            )
            if message is None:
                continue
            if message.get_type() == "LOCAL_POSITION_NED":
                final_error = math.hypot(
                    message.x - relative_targets[-1][1],
                    message.y - relative_targets[-1][2],
                )
            elif not (
                message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ):
                landed = True
                armed = False
                break
        if not landed:
            raise TimeoutError("PX4 did not confirm landing and disarming")
        summary = {
            "requested_duration_sec": duration_sec,
            "active_duration_sec": active_duration,
            "speed_mps": speed_mps,
            "altitude_m": altitude_m,
            "route": [item[0] for item in relative_targets] + ["land_at_goal"],
            "reached": reached,
            "landed_and_disarmed": landed,
            "final_goal_horizontal_error_m": final_error,
            "trajectory_rows": sum(1 for _ in trajectory.open()) - 1,
        }
        (run_dir / "flight_summary.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )
        return summary
    finally:
        if armed and not landing_commanded:
            try:
                command_and_wait(
                    connection, mavutil.mavlink.MAV_CMD_NAV_LAND, timeout=3
                )
            except Exception:
                pass
        setpoint_stop.set()
        if setpoint_thread is not None:
            setpoint_thread.join(timeout=2)
        heartbeat_stop.set()
        heartbeat_thread.join(timeout=2)
        connection.close()


ROS_EXTRACTOR = r'''from pathlib import Path
import csv,sys,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag,out=Path(sys.argv[1]),Path(sys.argv[2])
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
types={x.name:x.type for x in r.get_all_topics_and_types()}
count={k:0 for k in types};size={k:0 for k in types};first={};last={};pose=[]
pose_type=get_message(types['/mavros/local_position/pose'])
while r.has_next():
 topic,data,t=r.read_next();count[topic]+=1;size[topic]+=len(data);first.setdefault(topic,t);last[topic]=t
 if topic=='/mavros/local_position/pose':
  p=deserialize_message(data,pose_type).pose.position;pose.append((t,p.x,p.y,p.z))
with (out/'topic_measurements.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['topic','message_count','duration_sec','frequency_hz','total_serialized_bytes','average_serialized_message_bytes','bandwidth_kbps'])
 for topic in sorted(types):
  duration=max(1e-9,(last[topic]-first[topic])/1e9);n=count[topic]
  w.writerow([topic,n,duration,n/duration,size[topic],size[topic]/max(1,n),size[topic]*8/duration/1000])
assert pose
t0,x0,y0,z0=pose[0]
with (out/'ros_pose_trace.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['elapsed_sec','east_m','north_m','up_m'])
 for t,x,y,z in pose:w.writerow([(t-t0)/1e9,x-x0,y-y0,z-z0])
'''


def extract_ros_evidence(run_dir: Path, bag_dir: Path) -> dict:
    info = subprocess.run(
        [
            "bash",
            "-lc",
            f"source /opt/ros/jazzy/setup.bash && ros2 bag info {bag_dir}",
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )
    (run_dir / "rosbag_info.txt").write_text(info.stdout + info.stderr)
    if info.returncode:
        raise RuntimeError("ros2 bag info failed")
    extractor_path = run_dir / "extract_rosbag.py"
    extractor_path.write_text(ROS_EXTRACTOR, encoding="utf-8")
    extract = subprocess.run(
        [
            "bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash && exec /usr/bin/python3 "
            + f"{extractor_path!s} {bag_dir!s} {run_dir!s}",
        ],
        capture_output=True,
        text=True,
        timeout=300,
    )
    if extract.returncode:
        raise RuntimeError("ROS bag extraction failed:\n" + extract.stdout + extract.stderr)
    measurements = list(csv.DictReader((run_dir / "topic_measurements.csv").open()))
    by_topic = {row["topic"]: row for row in measurements}
    required = {
        "/mavros/local_position/pose",
        "/mavros/state",
        "/mavros/global_position/raw/fix",
    }
    if not required.issubset(by_topic):
        raise RuntimeError("Required MAVROS topics are missing from the ROS bag")
    return {
        topic: {
            "message_count": int(by_topic[topic]["message_count"]),
            "duration_sec": float(by_topic[topic]["duration_sec"]),
            "frequency_hz": float(by_topic[topic]["frequency_hz"]),
        }
        for topic in sorted(required)
    }


def resume_after_flight(run_dir: Path) -> dict:
    """Resume ROS/Simu5G processing when a completed flight was preserved."""
    flight_path = run_dir / "flight_summary.json"
    bag_dir = run_dir / "rosbag"
    if not flight_path.is_file() or not (bag_dir / "metadata.yaml").is_file():
        raise FileNotFoundError("A completed flight summary and ROS bag are required")
    flight = json.loads(flight_path.read_text())
    duration_sec = int(flight["requested_duration_sec"])
    speed_mps = float(flight["speed_mps"])
    run_id = run_dir.name
    print(f"[RESUME] Extracting preserved ROS bag for {run_id}", flush=True)
    ros = extract_ros_evidence(run_dir, bag_dir)
    print(f"[SIMU5G] Processing preserved live trace for {run_id}", flush=True)
    network = run_simu5g(run_dir, run_id, duration_sec, speed_mps)
    result = {
        "run_id": run_id,
        "requested_duration_sec": duration_sec,
        "status": "passed",
        "completed_utc": datetime.now(timezone.utc).isoformat(),
        "run_directory": str(run_dir.relative_to(HERE)),
        "flight": flight,
        "ros_topics": ros,
        "network": network,
        "free_gib_after": disk_free_gib(),
        "resumed_from_preserved_flight": True,
    }
    (run_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    compress_raw_network(run_dir)
    result["free_gib_after_compression"] = disk_free_gib()
    (run_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    write_master_summary(completed_results())
    print(f"[PASSED] {run_id}", flush=True)
    return result


def prepare_simu5g(run_dir: Path, run_id: str, duration_sec: int, speed_mps: float) -> tuple[Path, Path, Path]:
    network = run_dir / "network"
    input_dir = network / "input"
    raw_dir = network / "raw"
    data_dir = network / "data"
    for directory in (input_dir, raw_dir, data_dir):
        directory.mkdir(parents=True, exist_ok=True)
    for name in ("EiraXExperiment.ned", "demo.xml", "flow_contract.json"):
        shutil.copy2(NETWORK_TEMPLATE / name, input_dir / name)

    trace_rows = list(csv.DictReader((run_dir / "ros_pose_trace.csv").open()))
    selected = [row for row in trace_rows if float(row["elapsed_sec"]) <= duration_sec]
    if not selected:
        raise RuntimeError("The exported ROS pose trace is empty")
    movement = []
    previous_time = -1.0
    for row in selected:
        stamp = float(row["elapsed_sec"])
        if stamp <= previous_time:
            continue
        previous_time = stamp
        movement.extend(
            [
                f"{stamp:.6f}",
                f"{500 + float(row['east_m']):.6f}",
                f"{500 + float(row['north_m']):.6f}",
                f"{max(0.05, float(row['up_m'])):.6f}",
            ]
        )
    if previous_time < duration_sec:
        movement[4 * (len(movement) // 4 - 1)] = f"{duration_sec:.6f}"
    (input_dir / "uav.movements").write_text(" ".join(movement) + "\n")

    ini = (NETWORK_TEMPLATE / "omnetpp.ini").read_text()
    ini = re.sub(r"sim-time-limit\s*=\s*[0-9.]+s", f"sim-time-limit = {duration_sec}s", ini)
    ini = re.sub(r"finishTime\s*=\s*[0-9.]+s", f"finishTime = {duration_sec - 0.1:.1f}s", ini)
    (input_dir / "omnetpp.ini").write_text(ini)

    manifest = {
        "organization_id": "eirax",
        "fleet_id": "fleet_1",
        "drone_id": "uav_1",
        "mission_id": run_id,
        "domain": "public_safety",
        "mission_type": "surveillance",
        "scenario_id": run_id,
        "scenario_family": "benign_live_duration_validation",
        "simulation": {
            "drone_count": 1,
            "mission_speed_mps": speed_mps,
            "duration_sec": duration_sec,
            "aggregation_window_sec": 1.0,
            "cells": 1,
            "background_traffic": "low",
            "network_slice_profile": "single_mission_slice",
            "route": ["spawn", "waypoint_1", "waypoint_2", "goal", "hold", "land"],
            "physical_source_run": run_id,
            "trace_derivation": "fresh live headless Gazebo/PX4/MAVROS measurement",
        },
        "attack": {
            "enabled": False,
            "attack_type": "none",
            "injection_point": "none",
            "start_sec": None,
            "end_sec": None,
            "intensity": "off",
        },
        "label_windows": [
            {
                "start_sec": 0,
                "end_sec": duration_sec,
                "attack_type": "none",
                "attack_stage": "none",
                "severity": "none",
                "incident_label": "normal",
                "recommended_response": "continue_monitoring",
            }
        ],
        "allowed_destinations": ["10.0.0.2"],
        "expected_effects": ["stable benign mission telemetry"],
    }
    manifest_path = run_dir / "scenario_manifest.yaml"
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False))
    shutil.copy2(manifest_path, input_dir / "scenario_manifest.yaml")
    return input_dir, raw_dir, data_dir


def run_simu5g(run_dir: Path, run_id: str, duration_sec: int, speed_mps: float) -> dict:
    input_dir, raw_dir, data_dir = prepare_simu5g(
        run_dir, run_id, duration_sec, speed_mps
    )
    ned_paths = ":".join(
        [
            str(SIMU5G / "simulations"),
            str(SIMU5G / "emulation"),
            str(SIMU5G / "src"),
            str(SIMU5G.parent / "inet4.5/src"),
            str(input_dir),
        ]
    )
    script = (
        "set -e; "
        f"source {OMNETPP}/setenv -q; "
        f"source {INET}/setenv -q; "
        f"cd {SIMU5G}; source ./setenv -f; "
        f"cd {input_dir}; "
        f"simu5g -n {ned_paths} -u Cmdenv -r 0 --result-dir={raw_dir}; "
        f"opp_scavetool x {raw_dir}/baseline.sca {raw_dir}/baseline.vec "
        f"-o {raw_dir}/network_metrics.csv"
    )
    with (run_dir / "simu5g.log").open("w", encoding="utf-8") as log:
        result = subprocess.run(
            ["bash", "-lc", script], stdout=log, stderr=subprocess.STDOUT
        )
    if result.returncode:
        raise RuntimeError(f"Simu5G failed; see {run_dir / 'simu5g.log'}")
    log_text = (run_dir / "simu5g.log").read_text(errors="replace")
    if "End." not in log_text or f"T={duration_sec}" not in log_text:
        raise RuntimeError("Simu5G did not prove the requested simulated duration")

    output = data_dir / f"{run_id}_cosim.csv"
    bridge_log = run_dir / "network/bridge_log.json"
    assemble = subprocess.run(
        [
            sys.executable,
            str(ASSEMBLER),
            "--window",
            "1.0",
            "--ros-log",
            str(run_dir),
            "--net-csv",
            str(raw_dir / "network_metrics.csv"),
            "--manifest",
            str(run_dir / "scenario_manifest.yaml"),
            "--flow-contract",
            str(input_dir / "flow_contract.json"),
            "--bridge-log",
            str(bridge_log),
            "--schema",
            str(SCHEMA),
            "--out",
            str(output),
        ],
        capture_output=True,
        text=True,
        timeout=600,
    )
    if assemble.returncode:
        raise RuntimeError("Bridge failed:\n" + assemble.stdout + assemble.stderr)
    validation = subprocess.run(
        [sys.executable, str(VALIDATOR), str(output), str(SCHEMA)],
        capture_output=True,
        text=True,
        timeout=300,
    )
    (run_dir / "validation.txt").write_text(validation.stdout + validation.stderr)
    if validation.returncode:
        raise RuntimeError("Dataset validation failed:\n" + validation.stdout)
    with output.open(newline="") as stream:
        reader = csv.reader(stream)
        columns = len(next(reader))
        rows = sum(1 for _ in reader)
    return {
        "simulated_duration_sec": duration_sec,
        "simu5g_end_confirmed": True,
        "dataset": str(output.relative_to(HERE)),
        "dataset_rows": rows,
        "dataset_columns": columns,
        "dataset_validation": validation.stdout.strip(),
    }


def compress_raw_network(run_dir: Path) -> None:
    raw = run_dir / "network/raw"
    for path in (raw / "baseline.vec", raw / "network_metrics.csv"):
        if not path.is_file():
            continue
        archive = path.with_suffix(path.suffix + ".gz")
        with path.open("rb") as source, gzip.open(archive, "wb", compresslevel=6) as target:
            shutil.copyfileobj(source, target, length=1024 * 1024)
        path.unlink()


def run_one(duration_sec: int, speed_mps: float, altitude_m: float) -> dict:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_id = f"LIVE_DURATION_{duration_sec // 60:02d}MIN_{stamp}"
    run_dir = OUTPUT_ROOT / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    state = {
        "run_id": run_id,
        "requested_duration_sec": duration_sec,
        "status": "started",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "run_directory": str(run_dir.relative_to(HERE)),
    }
    (run_dir / "status.json").write_text(json.dumps(state, indent=2) + "\n")

    # Use the document world but replace only the mapping UAV with PX4's plain
    # x500.  This generated run-local world leaves both saved source worlds intact.
    runtime_world = run_dir / "baylands_uav_duration.world"
    world_text = WORLD.read_text(encoding="utf-8")
    world_text = world_text.replace("model://x500_mapping", "model://x500")
    world_text = world_text.replace("<name>x500_mapping_0</name>", f"<name>{UAV_ENTITY}</name>")
    runtime_world.write_text(world_text, encoding="utf-8")

    env = os.environ.copy()
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(
        [str(MODELS), str(PX4 / "models"), env.get("GZ_SIM_RESOURCE_PATH", "")]
    ).rstrip(":")
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(
        [str(PX4 / "plugins"), env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]
    ).rstrip(":")
    processes = []
    try:
        gazebo = ManagedProcess(
            "gazebo",
            ["gz", "sim", "-s", "-r", str(runtime_world)],
            run_dir / "gazebo.log",
            cwd=HERE,
            env=env,
            stdin=subprocess.DEVNULL,
        )
        processes.append(gazebo)
        time.sleep(8)
        if gazebo.process.poll() is not None:
            raise RuntimeError("Gazebo exited during startup")

        px4_env = env.copy()
        px4_env.update(
            {
                "PX4_SYS_AUTOSTART": "4001",
                "PX4_GZ_MODEL_NAME": UAV_ENTITY,
                "PX4_GZ_STANDALONE": "1",
                "PX4_GZ_NO_FOLLOW": "1",
                "PX4_GZ_WORLD": WORLD_NAME,
                "HEADLESS": "1",
            }
        )
        px4 = ManagedProcess(
            "px4",
            [str(PX4 / "bin/px4"), "-d"],
            run_dir / "px4.log",
            cwd=PX4 / "rootfs",
            env=px4_env,
            stdin=subprocess.PIPE,
        )
        processes.append(px4)
        time.sleep(10)
        if px4.process.poll() is not None:
            raise RuntimeError("PX4 exited during startup")

        ros_prefix = "source /opt/ros/jazzy/setup.bash && "
        mavros = ManagedProcess(
            "mavros",
            [
                "bash",
                "-lc",
                ros_prefix
                + "exec ros2 run mavros mavros_node --ros-args "
                + "-p fcu_url:=udp://:14550@127.0.0.1:18570",
            ],
            run_dir / "mavros.log",
            cwd=HERE,
            env=env,
            stdin=subprocess.DEVNULL,
        )
        processes.append(mavros)
        time.sleep(6)
        if mavros.process.poll() is not None:
            raise RuntimeError("MAVROS exited during startup")

        bag_dir = run_dir / "rosbag"
        rosbag = ManagedProcess(
            "rosbag",
            [
                "bash",
                "-lc",
                ros_prefix
                + f"exec ros2 bag record -o {bag_dir} "
                + "/mavros/local_position/pose /mavros/state "
                + "/mavros/global_position/raw/fix",
            ],
            run_dir / "rosbag.log",
            cwd=HERE,
            env=env,
            stdin=subprocess.DEVNULL,
        )
        processes.append(rosbag)
        time.sleep(3)

        print(f"[LIVE TEST] {duration_sec // 60} minutes: starting flight", flush=True)
        flight = fly_for_duration(run_dir, duration_sec, speed_mps, altitude_m)
        rosbag.stop()
        processes.remove(rosbag)
        ros = extract_ros_evidence(run_dir, bag_dir)

        for process in reversed(processes):
            process.stop()
        processes.clear()
        time.sleep(4)

        print(f"[SIMU5G] {duration_sec // 60} minutes: processing measured trace", flush=True)
        network = run_simu5g(run_dir, run_id, duration_sec, speed_mps)
        result = {
            **state,
            "status": "passed",
            "completed_utc": datetime.now(timezone.utc).isoformat(),
            "flight": flight,
            "ros_topics": ros,
            "network": network,
            "free_gib_after": disk_free_gib(),
        }
        (run_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        compress_raw_network(run_dir)
        result["free_gib_after_compression"] = disk_free_gib()
        (run_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(f"[PASSED] {run_id}", flush=True)
        return result
    except Exception as error:
        state.update(
            {
                "status": "failed",
                "completed_utc": datetime.now(timezone.utc).isoformat(),
                "error": f"{type(error).__name__}: {error}",
                "free_gib_after": disk_free_gib(),
            }
        )
        (run_dir / "result.json").write_text(json.dumps(state, indent=2) + "\n")
        raise
    finally:
        for process in reversed(processes):
            try:
                process.stop()
            except Exception:
                pass


def completed_results() -> list[dict]:
    """Return the newest passing result for each required duration."""
    newest = {}
    for path in sorted(OUTPUT_ROOT.glob("LIVE_DURATION_*/result.json")):
        try:
            result = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        duration = int(result.get("requested_duration_sec", 0))
        if result.get("status") == "passed" and duration in (300, 600, 1200):
            newest[duration] = result
    return [newest[key] for key in sorted(newest)]


def write_master_summary(results: list[dict] | None = None) -> Path:
    results = completed_results() if results is None else results
    summary = {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "document_duration_requirements_sec": [300, 600, 1200],
        "all_passed": len(results) == 3
        and all(item.get("status") == "passed" for item in results),
        "tests": results,
        "canonical_dataset_unchanged": True,
        "free_gib": disk_free_gib(),
    }
    path = OUTPUT_ROOT / "live_duration_test_summary.json"
    path.write_text(json.dumps(summary, indent=2) + "\n")
    with (OUTPUT_ROOT / "live_duration_test_summary.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "run_id",
                "requested_duration_sec",
                "active_duration_sec",
                "landed_and_disarmed",
                "final_goal_error_m",
                "pose_messages",
                "ros_recording_duration_sec",
                "simu5g_end_confirmed",
                "dataset_rows",
                "dataset_columns",
                "status",
            ]
        )
        for item in results:
            pose = item["ros_topics"]["/mavros/local_position/pose"]
            writer.writerow(
                [
                    item["run_id"],
                    item["requested_duration_sec"],
                    item["flight"]["active_duration_sec"],
                    item["flight"]["landed_and_disarmed"],
                    item["flight"]["final_goal_horizontal_error_m"],
                    pose["message_count"],
                    pose["duration_sec"],
                    item["network"]["simu5g_end_confirmed"],
                    item["network"]["dataset_rows"],
                    item["network"]["dataset_columns"],
                    item["status"],
                ]
            )
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--durations",
        nargs="+",
        type=int,
        default=[300, 600, 1200],
        help="Mission durations in seconds (default: 300 600 1200).",
    )
    parser.add_argument("--speed", type=float, default=3.0)
    parser.add_argument("--altitude", type=float, default=30.0)
    parser.add_argument(
        "--resume-run",
        type=Path,
        help="Resume ROS/Simu5G processing for a preserved completed flight.",
    )
    args = parser.parse_args()
    if args.resume_run is not None:
        require_environment()
        resume_after_flight(args.resume_run.resolve())
        return 0
    if not args.durations or not set(args.durations).issubset({300, 600, 1200}):
        parser.error("durations must be selected from 300, 600 and 1200 seconds")
    require_environment()
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    results = []
    for duration in sorted(args.durations):
        print(
            f"[START] {duration // 60}-minute test; free space={disk_free_gib():.1f} GiB",
            flush=True,
        )
        results.append(run_one(duration, args.speed, args.altitude))
    summary = write_master_summary(completed_results())
    completed = {item["requested_duration_sec"] for item in completed_results()}
    if completed == {300, 600, 1200}:
        print(f"[ALL PASSED] {summary}", flush=True)
    else:
        print(f"[PARTIAL PASS] completed durations: {sorted(completed)}; {summary}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

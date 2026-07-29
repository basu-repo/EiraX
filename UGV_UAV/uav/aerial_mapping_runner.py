"""Run the Baylands x500 three-dimensional laser mapping baseline."""

from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

COOPERATIVE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = COOPERATIVE_ROOT.parent
sys.path.insert(0, str(PROJECT_ROOT))

from UGV_Standalone.baseline.core.process_manager import ProcessManager
from UGV_Standalone.baseline.monitoring.topic_health import (
    wait_for_message,
    wait_for_topics,
)
from uav.analyze_waypoint import create_plot
from uav.baylands_waypoint_runner import world_pose
from uav import mapping_commands
from uav.nav2_config import build as build_uav_nav2_config
from uav.obstacle_aware_route import fly_route
from uav.x500_waypoint_mission import fly_waypoint


PX4_RUNTIME = COOPERATIVE_ROOT / "px4_runtime"
SOURCE_WORLD = PROJECT_ROOT / "simulation/worlds/baylands_editable.world"


def create_aerial_world(output: Path) -> None:
    """Copy Baylands and replace Husky with a level UAV landing pad."""
    tree = ET.parse(SOURCE_WORLD)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError(f"No world element in {SOURCE_WORLD}")
    pad_pose: tuple[float, float, float] | None = None
    for include in list(world.findall("include")):
        if include.findtext("name") == "husky":
            values = (include.findtext("pose") or "0 0 0").split()
            pad_pose = (float(values[0]), float(values[1]), float(values[2]))
            world.remove(include)
    if pad_pose is None:
        raise ValueError("Husky spawn pose is unavailable for the UAV landing pad")
    x, y, z = pad_pose
    world.append(
        ET.fromstring(
            f"""
            <model name="uav_landing_pad">
              <static>true</static>
              <pose>{x} {y} {z - 0.025} 0 0 0</pose>
              <link name="pad">
                <collision name="collision">
                  <geometry><box><size>1.5 1.5 0.05</size></box></geometry>
                </collision>
                <visual name="visual">
                  <geometry><box><size>1.5 1.5 0.05</size></box></geometry>
                  <material>
                    <ambient>0.12 0.12 0.12 1</ambient>
                    <diffuse>0.18 0.18 0.18 1</diffuse>
                  </material>
                </visual>
              </link>
            </model>
            """
        )
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="unicode", xml_declaration=True)


def stop_process(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--altitude", type=float, default=30.0)
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--hover-seconds", type=float, default=5.0)
    parser.add_argument(
        "--small-map", action="store_true",
        help="Fly a short 15 m north / 15 m east mapping leg instead of waypoint 1",
    )
    parser.add_argument(
        "--hover-map-seconds", type=float,
        help="Fly slow local loops around spawn for approximately this mapping duration",
    )
    parser.add_argument(
        "--viewer", choices=("rtab", "rviz", "none"), default="rviz",
        help="3D SLAM and planned-path viewer for a visible run (default: rviz)",
    )
    args = parser.parse_args()

    existing_px4 = subprocess.run(
        ["pgrep", "-f", str(PX4_RUNTIME / "bin/px4")],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if existing_px4.returncode == 0:
        print("[FAILED] Another PX4 UAV simulation is already open. Close its Gazebo window first.")
        return 1

    run_directory = PROJECT_ROOT / "datasets" / datetime.now().strftime("run_%Y%m%d_%H%M%S") / "uav_mapping"
    logs = run_directory / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    aerial_world = run_directory / "world" / "baylands_uav.world"
    create_aerial_world(aerial_world)

    spawn = world_pose("husky")
    waypoint = world_pose("waypoint_1")
    route_mode = not args.small_map and args.hover_map_seconds is None
    route_targets = []
    for name in ("waypoint_1", "waypoint_2", "waypoint_3", "goal"):
        target = world_pose(name)
        route_targets.append(
            (name, target[1] - spawn[1], target[0] - spawn[0])
        )
    north = waypoint[1] - spawn[1]
    east = waypoint[0] - spawn[0]
    if args.small_map:
        north, east = 15.0, 15.0
        args.altitude = min(args.altitude, 10.0)
        args.speed = min(args.speed, 2.0)
    if args.hover_map_seconds is not None:
        north, east = 0.0, 0.0
        args.altitude = min(args.altitude, 10.0)
        args.speed = min(args.speed, 1.0)

    environment = os.environ.copy()
    environment["GZ_SIM_RESOURCE_PATH"] = ":".join(
        [
            str(PROJECT_ROOT / "simulation/models"),
            str(PX4_RUNTIME / "models"),
            environment.get("GZ_SIM_RESOURCE_PATH", ""),
        ]
    ).rstrip(":")
    environment["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(
        [str(PX4_RUNTIME / "plugins"), environment.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]
    ).rstrip(":")
    environment["PYTHONPATH"] = ":".join(
        [
            str(PX4_RUNTIME / "python"),
            str(COOPERATIVE_ROOT),
            str(PROJECT_ROOT),
            environment.get("PYTHONPATH", ""),
        ]
    ).rstrip(":")
    environment["ROS_LOG_DIR"] = str(logs / "ros")
    environment["MPLCONFIGDIR"] = str(run_directory / ".matplotlib")
    # Keep discovery isolated from any Gazebo process left open by an earlier
    # desktop or headless session. PX4 and all bridge processes inherit this.
    partition = f"eirax_{run_directory.parent.name}"
    environment["GZ_PARTITION"] = partition
    os.environ["GZ_PARTITION"] = partition

    gazebo_log = (logs / "gazebo.log").open("w", encoding="utf-8")
    px4_log = (logs / "px4.log").open("w", encoding="utf-8")
    gazebo: subprocess.Popen | None = None
    px4: subprocess.Popen | None = None
    ros_processes = ProcessManager(logs, environment)
    try:
        gazebo_command = ["gz", "sim"]
        if args.headless:
            gazebo_command.extend(["-r", "-s"])
        gazebo_command.append(str(aerial_world))
        gazebo = subprocess.Popen(
            gazebo_command,
            cwd=COOPERATIVE_ROOT,
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=gazebo_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        if args.headless:
            print("[WAITING] Aerial-only Baylands is starting.")
        else:
            print("[PAUSED] Wait for Baylands and the x500 to load, then click Play in Gazebo.")
        time.sleep(6)
        if gazebo.poll() is not None:
            print(f"[FAILED] Gazebo stopped. Check {logs / 'gazebo.log'}")
            return 1

        px4_environment = environment.copy()
        px4_environment.update(
            {
                "PX4_SYS_AUTOSTART": "4001",
                "PX4_SIM_MODEL": "gz_x500_mapping",
                "PX4_GZ_STANDALONE": "1",
                "PX4_GZ_NO_FOLLOW": "1",
                "PX4_GZ_WORLD": "baylands_editable",
                # Place the landing gear just above the level pad without a drop.
                "PX4_GZ_MODEL_POSE": f"{spawn[0]},{spawn[1]},{spawn[2] + 0.02}",
            }
        )
        px4 = subprocess.Popen(
            [str(PX4_RUNTIME / "bin/px4"), "-d"],
            cwd=PX4_RUNTIME / "rootfs",
            env=px4_environment,
            stdin=subprocess.PIPE,
            stdout=px4_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        print("[WAITING] PX4 x500 with 32-channel 3D LiDAR is starting.")
        time.sleep(8)
        if px4.poll() is not None:
            print(f"[FAILED] PX4 stopped. Check {logs / 'px4.log'}")
            return 1

        if args.headless:
            # A headless run has no Play button, so explicitly ensure that the
            # standalone world advances after PX4 inserts the vehicle.
            subprocess.run(
                [
                    "gz", "service", "-s", "/world/baylands_editable/control",
                    "--reqtype", "gz.msgs.WorldControl",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "3000", "--req", "pause: false",
                ],
                cwd=COOPERATIVE_ROOT,
                env=environment,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )

        ros_processes.start("bridge", mapping_commands.bridge())
        ros_processes.start("sensor_transform", mapping_commands.sensor_transform())
        if not args.headless and args.viewer == "rtab":
            ros_processes.start(
                "slam_viewer",
                mapping_commands.rtabmap_viewer(
                    COOPERATIVE_ROOT / "uav/config/aerial_rtabmap_gui.ini"
                ),
            )
            print("[3D VIEW] Native RTAB-Map 3D SLAM viewer opened.")
        elif not args.headless and args.viewer == "rviz":
            ros_processes.start(
                "slam_viewer",
                mapping_commands.rviz_viewer(
                    COOPERATIVE_ROOT / "uav/config/uav_mapping.rviz"
                ),
            )
            print("[3D VIEW] RViz opened for the live RTAB-Map cloud.")
        if not wait_for_message(mapping_commands.POINTS, 300.0):
            print("[FAILED] No aerial three-dimensional point cloud received.")
            return 2
        print("[OK] 32 x 512 point cloud is publishing at 10 Hz")

        ros_processes.start("laser_odometry", mapping_commands.laser_odometry())
        ready, missing = wait_for_topics(["/uav/lidar_odom"], 60.0)
        if not ready or not wait_for_message("/uav/lidar_odom", 60.0):
            print(f"[FAILED] Aerial laser odometry is unavailable: {missing}")
            return 2
        print("[OK] Independent three-dimensional laser odometry is publishing")

        database = run_directory / "aerial_rtabmap.db"
        ros_processes.start("mapping", mapping_commands.mapping(database))
        ready, missing = wait_for_topics(["/uav_rtabmap/mapGraph"], 60.0)
        if (
            not ready
            or not wait_for_message("/uav_rtabmap/mapGraph", 60.0)
            or not wait_for_message("/uav_rtabmap/cloud_map", 60.0)
        ):
            print(f"[FAILED] Aerial mapping topics are unavailable: {missing}")
            return 2
        print("[OK] Aerial RTAB-Map graph and accumulated 3D cloud are publishing")

        if route_mode:
            nav2_params = build_uav_nav2_config(
                run_directory / "config/uav_nav2_params.yaml",
                args.altitude,
            )
            ros_processes.start(
                "uav_nav2_planner", mapping_commands.nav2_planner(nav2_params)
            )
            ros_processes.start(
                "uav_nav2_lifecycle", mapping_commands.nav2_lifecycle()
            )
            print("[OK] Aerial Nav2 obstacle planner is starting")

        ros_processes.start("recorder", mapping_commands.recorder(run_directory / "rosbag"))
        print(f"[RECORDING] {run_directory}")
        if route_mode:
            result = fly_route(
                14540,
                run_directory,
                route_targets,
                args.altitude,
                speed_mps=args.speed,
                model_name="x500_mapping_0",
            )
        else:
            result = fly_waypoint(
                14540,
                run_directory,
                north,
                east,
                args.altitude,
                args.hover_seconds,
                model_name="x500_mapping_0",
                speed_mps=args.speed,
                local_mapping_seconds=args.hover_map_seconds,
            )
        time.sleep(3)
        ros_processes.stop("recorder")
        plot = create_plot(run_directory)
        print(f"[PLOT] {plot}")
        print(f"[SAVED] {run_directory}")
        if not args.headless:
            print("[GAZEBO OPEN] Mapping run complete. Press Ctrl+C when finished viewing it.")
            while gazebo.poll() is None:
                time.sleep(1)
        return result
    except KeyboardInterrupt:
        return 130
    finally:
        ros_processes.stop_all()
        stop_process(px4)
        stop_process(gazebo)
        gazebo_log.close()
        px4_log.close()

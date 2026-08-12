#!/usr/bin/env python3
"""Launch the proven standalone Husky navigation and mapping pipeline."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import time

import yaml

from baseline.config.nav2_config import build as build_nav2_config
from baseline.core.process_manager import ProcessManager
from baseline.data_logging.run_dataset import RunDataset
from baseline.launchers import commands
from baseline.mission.world_poses import rolling_costmap_plan
from baseline.monitoring.topic_health import (
    gazebo_running,
    wait_for_finite_odometry,
    wait_for_message,
    wait_for_topics,
)


UGV_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = UGV_ROOT.parent
CONFIG_FILE = UGV_ROOT / "baseline/config/baseline.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the standalone EiraX Husky UGV simulation."
    )
    parser.add_argument(
        "--no-motion",
        action="store_true",
        help="Start sensing, localization, SLAM and Nav2 without sending a mission.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run Gazebo without its GUI and start simulation time immediately.",
    )
    parser.add_argument(
        "--view-3d-slam",
        action="store_true",
        help="Open the RTAB-Map interface for the live OS1-64 three-dimensional map.",
    )
    parser.add_argument(
        "--return-to-spawn",
        action="store_true",
        help="Navigate back to the saved Husky spawn after reaching the goal.",
    )
    args = parser.parse_args()

    if gazebo_running():
        print("[FAILED] Another Gazebo server is already running. Close it first.")
        return 1

    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    world_file = (PROJECT_ROOT / config["world_file"]).resolve()
    mission_targets = ["waypoint_1", "waypoint_2", "waypoint_3", "goal"]
    if args.return_to_spawn:
        mission_targets.append("spawn")

    global_size_m, global_resolution_m, longest_leg_m = rolling_costmap_plan(
        world_file, mission_targets
    )
    dataset = RunDataset(PROJECT_ROOT, world_file)

    env = os.environ.copy()
    resource_paths = [str(PROJECT_ROOT / "simulation/models")]
    if env.get("GZ_SIM_RESOURCE_PATH"):
        resource_paths.append(env["GZ_SIM_RESOURCE_PATH"])
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(resource_paths)
    env["PYTHONPATH"] = ":".join(
        path
        for path in (str(UGV_ROOT), env.get("PYTHONPATH", ""))
        if path
    )
    env["ROS_LOG_DIR"] = str(dataset.logs / "ros")

    manager = ProcessManager(dataset.logs, env)
    stopping = False
    saved_reported = False

    def request_stop(_signum=None, _frame=None) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        dataset.event("runner", "run_started", mode="ugv_standalone")
        gazebo = manager.start(
            "gazebo", commands.gazebo(world_file, headless=args.headless)
        )
        time.sleep(3)
        manager.start("bridge", commands.bridge())
        manager.start("monotonic_clock", commands.monotonic_clock())
        manager.start("lidar3d_transform", commands.lidar3d_transform())
        manager.start("imu_transform", commands.imu_transform())
        manager.start("gps_transform", commands.gps_transform())
        manager.start(
            "local_localization",
            commands.local_localization(
                UGV_ROOT / "baseline/config/local_ekf_params.yaml"
            ),
        )
        manager.start(
            "localization",
            commands.localization(UGV_ROOT / "baseline/config/ekf_params.yaml"),
        )
        manager.start(
            "navsat_transform",
            commands.navsat_transform(
                UGV_ROOT / "baseline/config/navsat_params.yaml"
            ),
        )
        manager.start("lidar3d_odometry", commands.lidar3d_odometry())
        manager.start(
            "lidar3d_slam",
            commands.lidar3d_slam(dataset.root / "rtabmap_lidar.db"),
        )
        manager.start("slam3d", commands.slam3d(dataset.root / "rtabmap3d.db"))

        if args.view_3d_slam and not args.headless:
            time.sleep(1)
            manager.start(
                "slam3d_viewer",
                commands.slam3d_viewer(
                    UGV_ROOT / "baseline/config/ugv_rtabmap_gui.ini"
                ),
            )
            print("[3D VIEW] RTAB-Map UI opened. Configure it before clicking Play.")

        if args.headless:
            print("[HEADLESS] Gazebo server started and simulation is running.")
        else:
            print("[PAUSED] Wait for the models to load, then click Play in Gazebo.")
            print(
                f"[WAITING] Sensor startup timeout: "
                f"{config['startup_timeout_sec']} seconds"
            )

        healthy, missing = wait_for_topics(
            list(config["required_topics"]),
            float(config["startup_timeout_sec"]),
        )
        if not healthy:
            dataset.event("health", "startup_failed", missing_topics=missing)
            print(f"[FAILED] Missing topics: {', '.join(missing)}")
            return 2

        if not wait_for_message(
            "/husky/lidar3d/points", float(config["startup_timeout_sec"])
        ):
            dataset.event(
                "health", "sensor_data_timeout", topic="/husky/lidar3d/points"
            )
            print("[FAILED] No OS1-64 point cloud received. Click Play and try again.")
            return 2

        if not wait_for_finite_odometry(
            "/odometry/gps", float(config["startup_timeout_sec"])
        ):
            dataset.event("health", "invalid_gps_odometry")
            print("[FAILED] GNSS odometry is missing or contains invalid values.")
            return 2

        print("[OK] Gazebo, bridge, 3D LiDAR, GNSS, odometry, TF and IMU")
        map_ready, map_missing = wait_for_topics(
            list(config["slam_required_topics"]),
            float(config["startup_timeout_sec"]),
        )
        if not map_ready:
            dataset.event(
                "health", "slam3d_startup_failed", missing_topics=map_missing
            )
            print(f"[FAILED] 3D SLAM missing topics: {', '.join(map_missing)}")
            return 2

        if not wait_for_message("/map", float(config["startup_timeout_sec"])):
            dataset.event("health", "slam3d_data_timeout", topic="/map")
            print("[FAILED] RTAB-Map did not publish its projected map.")
            return 2

        print("[OK] RTAB-Map 3D SLAM and projected Nav2 map are publishing")
        dataset.event(
            "health", "baseline_ready", topics=list(config["required_topics"])
        )
        manager.start(
            "localization_evaluator",
            commands.localization_evaluator(
                dataset.root / "localization/trajectory_and_errors.csv"
            ),
        )
        print("[LOCALIZATION] Ground truth and estimator errors are being recorded.")
        manager.start(
            "rosbag",
            commands.recorder(dataset.rosbag, list(config["record_topics"])),
        )
        print(f"[RECORDING] {dataset.root}")

        nav2_params = build_nav2_config(
            dataset.root / "config/nav2_params.yaml",
            global_size_m=global_size_m,
            global_resolution_m=global_resolution_m,
        )
        dataset.event(
            "navigation",
            "planning_area_configured",
            size_m=global_size_m,
            resolution_m=global_resolution_m,
            longest_leg_m=longest_leg_m,
        )
        print(
            f"[PLANNING AREA] {global_size_m:.0f} x {global_size_m:.0f} m "
            f"at {global_resolution_m:.2f} m resolution "
            f"(longest mission leg: {longest_leg_m:.1f} m)"
        )
        manager.start("nav2", commands.nav2(nav2_params))
        print(
            "[WAITING] Nav2 is starting with the Husky footprint "
            "and conservative speed limits."
        )

        if args.no_motion:
            print("[NO MOTION] Nav2 validation only. Press Ctrl+C to stop.")
            while not stopping:
                failures = manager.failures(ignored={"slam3d_viewer"})
                if failures:
                    dataset.event("health", "component_failed", failures=failures)
                    print(f"[FAILED] {failures}")
                    return 3
                time.sleep(float(config["health_period_sec"]))
            return 0

        mission = manager.start(
            "mission",
            commands.waypoint_mission(
                world_file,
                dataset.events_path,
                mission_targets,
            ),
        )
        print(f"[MISSION] spawn -> {' -> '.join(mission_targets)} -> stop")
        print("Press Ctrl+C at any time for an emergency stop.")

        while not stopping:
            mission_code = mission.process.poll()
            if mission_code is not None:
                dataset.event(
                    "runner", "mission_finished", return_code=mission_code
                )
                print(f"[MISSION FINISHED] return code {mission_code}")
                if not args.headless:
                    manager.stop("rosbag")
                    manager.stop("localization_evaluator")
                    dataset.event("runner", "recording_stopped")
                    print(f"[SAVED] {dataset.root}")
                    saved_reported = True
                    if mission_code == 0:
                        print(
                            "[GAZEBO OPEN] Recording has stopped. "
                            "Close Gazebo when you are finished."
                        )
                    else:
                        print(
                            "[GAZEBO OPEN] Mission stopped with an error; "
                            "Gazebo is being kept open for inspection."
                        )
                    while not stopping and gazebo.process.poll() is None:
                        time.sleep(float(config["health_period_sec"]))
                return mission_code

            failures = manager.failures(ignored={"slam3d_viewer"})
            if failures:
                dataset.event("health", "component_failed", failures=failures)
                print(f"[FAILED] {failures}")
                return 3
            time.sleep(float(config["health_period_sec"]))
        return 0
    finally:
        try:
            dataset.event("runner", "shutdown_started")
        except OSError as error:
            print(f"[DATASET WARNING] Could not write shutdown event: {error}")
        manager.stop_all()
        try:
            dataset.event("runner", "run_finished")
        except OSError as error:
            print(f"[DATASET WARNING] Could not write final event: {error}")
        if not saved_reported:
            print(f"[SAVED] {dataset.root}")


if __name__ == "__main__":
    raise SystemExit(main())

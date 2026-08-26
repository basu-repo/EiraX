#!/usr/bin/env python3
"""Launch the proven standalone Husky navigation and mapping pipeline."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
PACKAGE_SOURCE = PROJECT_ROOT / "src/lrs_halmstad"
if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))

from robots.husky.navigation.config.nav2_config import build as build_nav2_config
from robots.husky.runtime.process_manager import ProcessManager
from robots.husky.data_logging.run_dataset import RunDataset
from robots.husky.hardware.launchers import commands
from robots.husky.missions.world_poses import rolling_costmap_plan
from robots.husky.monitoring.topic_health import (
    gazebo_running,
    wait_for_advancing_clock,
    wait_for_finite_odometry,
    wait_for_lifecycle_active,
    wait_for_message,
    wait_for_topics,
)
from simulation.run_one_dji import (
    DJI_PARAMS,
    GAZEBO_WORLD_NAME,
    build_one_dji_world,
    build_dji_swarm_world,
    named_pose as dji_world_pose,
    ros_command,
)
from lrs_halmstad.coordination.failover_protocol import (
    fail as fail_dji,
    initial_state as initial_dji_state,
    reconnect as reconnect_dji,
    write as write_dji_state,
)

YOLO_WEIGHTS = (
    PROJECT_ROOT
    / "src/lrs_halmstad/models/obb/mymodels/baylands-leader-v9-tuned-full.pt"
)


UGV_ROOT = PROJECT_ROOT / "robots/husky"
CONFIG_FILE = UGV_ROOT / "navigation/config/baseline.yaml"


def activate_navigation(env: dict[str, str], timeout_sec: float = 120.0) -> bool:
    """Bring up Nav2 only after all lifecycle services have appeared."""
    required = (
        "/controller_server",
        "/planner_server",
        "/behavior_server",
        "/bt_navigator",
        "/collision_monitor",
    )
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        try:
            services = subprocess.run(
                ["ros2", "service", "list"], env=env, capture_output=True,
                text=True, timeout=5, check=False,
            ).stdout
            lifecycle_services = {
                "/lifecycle_manager_navigation/manage_nodes",
                *(f"{node}/get_state" for node in required),
                *(f"{node}/change_state" for node in required),
            }
            if all(service in services for service in lifecycle_services):
                break
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.5)
    else:
        print("[FAILED] Nav2 lifecycle manager service did not appear.")
        return False

    for attempt in range(1, 4):
        try:
            result = subprocess.run(
                [
                    "ros2", "service", "call",
                    "/lifecycle_manager_navigation/manage_nodes",
                    "nav2_msgs/srv/ManageLifecycleNodes", "{command: 0}",
                ],
                env=env, capture_output=True, text=True,
                timeout=min(60.0, max(5.0, deadline - time.monotonic())),
                check=False,
            )
            if result.returncode == 0 and "success=True" in result.stdout:
                remaining = max(1.0, deadline - time.monotonic())
                per_node = max(1.0, remaining / len(required))
                if all(wait_for_lifecycle_active(node, per_node) for node in required):
                    print("[OK] Nav2 controller, planner, behavior, navigator and collision monitor are active.")
                    return True
        except subprocess.TimeoutExpired:
            pass
        print(f"[WAITING] Nav2 activation attempt {attempt} did not complete; retrying.")
        time.sleep(2.0)
    print("[FAILED] Nav2 did not reach a fully active lifecycle state.")
    return False


def component_params(
    output: Path,
    section: str,
    node_name: str,
    overrides: dict,
) -> Path:
    """Write one self-contained ROS parameter file with deterministic overrides."""
    source = yaml.safe_load(DJI_PARAMS.read_text(encoding="utf-8"))
    # ROS normally applies the /** parameters together with the node-specific
    # section. Since this helper writes a self-contained per-UAV file, preserve
    # that merge explicitly instead of silently dropping shared camera/gimbal
    # geometry required by the simulator and tracker.
    parameters = dict(source.get("/**", {}).get("ros__parameters", {}))
    parameters.update(source.get(section, {}).get("ros__parameters", {}))
    parameters.update(overrides)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump({node_name: {"ros__parameters": parameters}}, sort_keys=False),
        encoding="utf-8",
    )
    return output


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
    parser.add_argument(
        "--dji-follow",
        action="store_true",
        help="Run one DJI M100 with the UGV_UAV scout-and-escort sequence.",
    )
    parser.add_argument(
        "--single-dji-follower",
        action="store_true",
        help=(
            "Run one DJI M100 as a pure Husky follower. The Husky starts each "
            "Nav2 leg without waiting for an aerial survey."
        ),
    )
    parser.add_argument("--dji-swarm", action="store_true", help="Run three DJI M100s.")
    failure = parser.add_mutually_exclusive_group()
    failure.add_argument("--permanent-failure", action="store_true")
    failure.add_argument("--connection-failure-reconnect", action="store_true")
    args = parser.parse_args()
    selected_dji_modes = sum(
        bool(value)
        for value in (
            args.dji_follow,
            args.single_dji_follower,
            args.dji_swarm,
            args.permanent_failure,
            args.connection_failure_reconnect,
        )
    )
    if selected_dji_modes > 1:
        parser.error(
            "select only one DJI scenario: --single-dji-follower, --dji-follow, "
            "--dji-swarm, --permanent-failure, or --connection-failure-reconnect"
        )
    dji_swarm = args.dji_swarm or args.permanent_failure or args.connection_failure_reconnect
    single_dji_follower = args.single_dji_follower
    dji_enabled = args.dji_follow or single_dji_follower or dji_swarm

    if gazebo_running():
        print("[FAILED] Another Gazebo server is already running. Close it first.")
        return 1

    if not CONFIG_FILE.is_file():
        print(f"[FAILED] Husky configuration is missing: {CONFIG_FILE}")
        return 2

    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    world_file = (PROJECT_ROOT / config["world_file"]).resolve()
    if not world_file.is_file():
        print(f"[FAILED] Baylands world is missing: {world_file}")
        return 2
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
    sdf_paths = [str(PROJECT_ROOT / "simulation/models")]
    if env.get("SDF_PATH"):
        sdf_paths.append(env["SDF_PATH"])
    env["SDF_PATH"] = ":".join(sdf_paths)
    env["PYTHONPATH"] = ":".join(
        path
        for path in (str(PROJECT_ROOT), env.get("PYTHONPATH", ""))
        if path
    )
    env["ROS_LOG_DIR"] = str(dataset.logs / "ros")

    dji_starts: list[tuple[str, float, float, float]] = []
    if dji_enabled:
        husky_x, husky_y, husky_z = dji_world_pose("husky")
        dji_starts = [("dji0", husky_x + 3.0, husky_y, husky_z + 0.45)]
        if dji_swarm:
            dji_starts.extend([
                ("dji1", husky_x + 3.0, husky_y + 4.0, husky_z + 0.45),
                ("dji2", husky_x + 3.0, husky_y - 4.0, husky_z + 0.45),
            ])
        cooperative_world = dataset.world / "dji_husky_follow_baylands.world"
        try:
            if dji_swarm:
                build_dji_swarm_world(cooperative_world, dji_starts, env)
            else:
                _, x, y, z = dji_starts[0]
                build_one_dji_world(cooperative_world, x, y, z, env)
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            print(f"[FAILED] Could not build the DJI-Husky world: {error}")
            return 2
        world_file = cooperative_world

    manager = ProcessManager(dataset.logs, env)
    stopping = False
    saved_reported = False

    def request_stop(_signum=None, _frame=None) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        try:
            validation = subprocess.run(
                ["gz", "sdf", "-k", str(world_file)],
                env=env,
                capture_output=True,
                text=True,
                timeout=60,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            print(f"[FAILED] Could not validate the Baylands world: {error}")
            return 2
        if validation.returncode != 0:
            details = (validation.stderr or validation.stdout).strip()
            print("[FAILED] Baylands or one of its copied models is invalid.")
            if details:
                print(details)
            return 2
        print("[PREFLIGHT] Baylands and all referenced model files are valid.")

        mode = (
            "dji_swarm" if dji_swarm else
            "single_dji_follower" if single_dji_follower else
            "dji_husky_scout" if args.dji_follow else
            "ugv_standalone"
        )
        dataset.event("runner", "run_started", mode=mode)
        # Keep the same known-good Gazebo interface used by the standalone
        # Husky and UGV_UAV runs. DJI behavior must not replace the GUI.
        gazebo_command = commands.gazebo(world_file, headless=args.headless)
        gazebo = manager.start("gazebo", gazebo_command)
        time.sleep(3)
        gazebo_code = gazebo.process.poll()
        if gazebo_code is not None:
            dataset.event(
                "health", "gazebo_startup_failed", return_code=gazebo_code
            )
            print(
                f"[FAILED] Gazebo exited during startup with code {gazebo_code}."
            )
            print(f"[LOG] {dataset.logs / 'gazebo.log'}")
            return 2
        manager.start("bridge", commands.bridge())
        manager.start("monotonic_clock", commands.monotonic_clock())
        if dji_enabled:
            if not YOLO_WEIGHTS.is_file():
                print(f"[FAILED] Other-team YOLO weights are missing: {YOLO_WEIGHTS}")
                return 2
            bridge_arguments = [
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                f"/world/{GAZEBO_WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
                "/model/husky/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            ]
            for name, *_ in dji_starts:
                bridge_arguments.extend([
                    f"/{name}/camera0/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    f"/{name}/camera0/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                ])
            bridge_arguments.extend(["--ros-args"])
            bridge_arguments.extend([
                "-r", "/model/husky/pose:=/dji_swarm/husky_pose",
            ])
            for name, *_ in dji_starts:
                bridge_arguments.extend(
                    ["-r", f"/{name}/camera0/image:=/{name}/camera0/image_raw"]
                )
            manager.start(
                "dji_bridges",
                ros_command(bridge_arguments),
            )
            for name, x, y, z in dji_starts:
                simulator_params = component_params(
                    dataset.root / f"config/{name}_simulator.yaml",
                    "uav_simulator",
                    "uav_simulator",
                    {
                        "use_sim_time": False,
                        "world": GAZEBO_WORLD_NAME,
                        "uav_name": name,
                        "start_x": x,
                        "start_y": y,
                        "start_z": z,
                        "tilt_enable": not single_dji_follower,
                        "pan_enable": False,
                    },
                )
                simulator_process = manager.start(
                    f"{name}_simulator",
                    ros_command([
                    "ros2", "run", "lrs_halmstad", "simulator", "--ros-args",
                    # The DJI adapter drives its 20 Hz control loop from wall
                    # time, as it does on the physical onboard computer. Gazebo
                    # simulation time remains in use by the UGV/Nav2 stack.
                    "--params-file", str(simulator_params),
                    ]),
                )
                # A missing required parameter previously left the follower
                # waiting for a pose for five minutes. Surface simulator
                # startup failures immediately with the exact log location.
                time.sleep(0.75)
                simulator_code = simulator_process.process.poll()
                if simulator_code is not None:
                    print(
                        f"[FAILED] {name} simulator exited with code "
                        f"{simulator_code}: {dataset.logs / f'{name}_simulator.log'}"
                    )
                    return 2
                detection_topic = f"/coord/{name}/leader_detection"
                detection_status_topic = f"/coord/{name}/leader_detection_status"
                detector_params = component_params(
                    dataset.root / f"config/{name}_yolo.yaml",
                    "leader_detector",
                    "leader_detector",
                    {
                        "use_sim_time": False,
                        "uav_name": name,
                        "camera_topic": f"/{name}/camera0/image_raw",
                        "yolo_weights": str(YOLO_WEIGHTS),
                        "out_topic": detection_topic,
                        "status_topic": detection_status_topic,
                        # Preserve the scout's faster perception cadence while
                        # preventing three concurrent YOLO workers from
                        # starving Gazebo rendering. Followers remain fully
                        # independent observers at a lower advisory rate.
                        "predict_hz": (
                            2.0 if not dji_swarm or name == "dji0" else 1.0
                        ),
                        "async_inference": True,
                        "latest_frame_only": True,
                        "benchmark_csv_path": str(
                            dataset.root / f"perception/{name}_yolo_benchmark.csv"
                        ),
                        # Selective annotated evidence copied from the proven
                        # decentralized PX4 integration. This stores bounded
                        # JPEG samples, not the complete camera stream.
                        "evidence_root": str(dataset.root / "yolo_frames"),
                        "evidence_positive_interval_s": 2.0,
                        "evidence_negative_interval_s": 20.0,
                        "evidence_jpeg_quality": 85,
                        "evidence_max_positive": 500,
                        "evidence_max_negative": 100,
                    },
                )
                manager.start(
                    f"{name}_yolo",
                    ros_command([
                        "ros2", "run", "lrs_halmstad", "leader_detector", "--ros-args",
                        "--params-file", str(detector_params),
                    ]),
                )
                if not single_dji_follower:
                    tracker_params = component_params(
                        dataset.root / f"config/{name}_camera_tracker.yaml",
                        "camera_tracker",
                        "camera_tracker",
                        {
                            "use_sim_time": False,
                            "uav_name": name,
                            "leader_input_type": "pose",
                            "leader_pose_topic": "/dji_swarm/husky_pose",
                            "leader_detection_topic": detection_topic,
                            "leader_detection_status_topic": detection_status_topic,
                            "tilt_enable": True,
                            "pan_enable": False,
                            "image_center_correction_enable": True,
                            "weak_status_center_correction_enable": True,
                        },
                    )
                    manager.start(
                        f"{name}_camera_tracker",
                        ros_command([
                            "ros2", "run", "lrs_halmstad", "camera_tracker", "--ros-args",
                            "--params-file", str(tracker_params),
                        ]),
                    )
            print(f"[DJI] {len(dji_starts)} embedded M100 aircraft and camera bridges are starting.")
            if single_dji_follower:
                print(
                    "[FIXED CAMERA] Gimbal actuation OFF; camera fixed at -45 deg. "
                    "DJI0 will use matching altitude/standoff geometry."
                )
            else:
                print("[GIMBAL] YOLO-assisted camera pitch tracking enabled; UAV body control is unchanged.")
        manager.start("lidar3d_transform", commands.lidar3d_transform())
        manager.start("safety_scan", commands.safety_scan())
        manager.start("imu_transform", commands.imu_transform())
        manager.start("gps_transform", commands.gps_transform())
        manager.start(
            "local_localization",
            commands.local_localization(
                UGV_ROOT / "navigation/config/local_ekf_params.yaml"
            ),
        )
        manager.start(
            "localization",
            commands.localization(UGV_ROOT / "navigation/config/ekf_params.yaml"),
        )
        manager.start(
            "navsat_transform",
            commands.navsat_transform(
                UGV_ROOT / "navigation/config/navsat_params.yaml"
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
                    UGV_ROOT / "navigation/config/ugv_rtabmap_gui.ini"
                ),
            )
            print("[3D VIEW] RTAB-Map UI opened. Configure it before clicking Play.")

        if args.headless:
            print("[HEADLESS] Gazebo server started and simulation is running.")
        else:
            print("[PAUSED] Wait for the models to load, then click Play in Gazebo.")
            print(
                f"[WAITING] Gazebo clock/startup timeout: "
                f"{config['startup_timeout_sec']} seconds"
            )

        if not wait_for_advancing_clock(float(config["startup_timeout_sec"])):
            dataset.event("health", "simulation_clock_not_advancing")
            print(
                "[FAILED] Gazebo simulation time did not advance. "
                "Click Play after the models finish loading."
            )
            return 2
        print("[OK] Gazebo simulation time is advancing.")

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

        if not wait_for_message(
            "/husky/safety_scan", float(config["startup_timeout_sec"])
        ):
            dataset.event(
                "health", "sensor_data_timeout", topic="/husky/safety_scan"
            )
            print("[FAILED] No height-filtered OS1-64 safety scan received.")
            return 2

        if not wait_for_finite_odometry(
            "/odometry/gps", float(config["startup_timeout_sec"])
        ):
            dataset.event("health", "invalid_gps_odometry")
            print("[FAILED] GNSS odometry is missing or contains invalid values.")
            return 2

        print("[OK] Gazebo, bridge, 3D LiDAR, safety scan, GNSS, odometry, TF and IMU")
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
        record_topics = list(config["record_topics"])
        if dji_enabled:
            # Full-resolution point clouds and RTAB-Map graph payloads made a
            # routine three-DJI run exceed 13 GB. They are processed live but
            # omitted from the cooperative evidence bag, which focuses on
            # localization, control, gimbal and perception results.
            bulky_topics = {
                "/husky/lidar3d/points",
                "/rtabmap3d/cloud_map",
                "/rtabmap3d/mapData",
                "/rtabmap3d/mapGraph",
                "/rtabmap_lidar/mapData",
                "/rtabmap_lidar/mapGraph",
            }
            record_topics = [
                topic for topic in record_topics if topic not in bulky_topics
            ]
            # Record compact evidence of DJI motion, gimbal tracking and YOLO
            # decisions. Continuous raw video is intentionally excluded to
            # keep Git/local datasets manageable; selected frames are handled
            # by the perception frame saver.
            for name, *_ in dji_starts:
                record_topics.extend(
                    [
                        f"/{name}/pose",
                        f"/{name}/pose_cmd",
                        f"/{name}/psdk_ros2/flight_control_setpoint_ENUposition_yaw",
                        f"/{name}/camera0/camera_info",
                        f"/{name}/update_tilt",
                        f"/{name}/follow/actual/tilt_deg",
                        f"/coord/{name}/leader_detection",
                        f"/coord/{name}/leader_detection_status",
                    ]
                )
        manager.start(
            "rosbag",
            commands.recorder(dataset.rosbag, record_topics),
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
        if not activate_navigation(env):
            dataset.event("navigation", "nav2_activation_failed")
            return 3

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

        if dji_enabled:
            coordination = dataset.root / "cooperative"
            coordination.mkdir(parents=True, exist_ok=True)
            stop_file = dataset.root / "dji_stop"
            role_state_file = coordination / "roles.json"
            swarm_state = initial_dji_state()
            if dji_swarm:
                write_dji_state(role_state_file, swarm_state)
            followers = []
            # A scout ignores its lateral slot. If DJI0 later reconnects as a
            # follower, +7 m keeps it clear of the Husky while DJI1 leads.
            offsets = {"dji0": 7.0, "dji1": 7.0, "dji2": -7.0}
            for name, start_x, start_y, start_z in dji_starts:
                ready_file = dataset.root / f"{name}_ready"
                follower_arguments = [
                    "ros2", "run", "lrs_halmstad", "vastervik_dji_follow",
                    "--name", name,
                    "--output", str(dataset.root / f"{name}_trajectory.csv"),
                    "--stop-file", str(stop_file), "--ready-file", str(ready_file),
                    "--coordination-dir", str(coordination),
                    "--altitude", str(15 + 2 * int(name[-1])), "--lead-distance", "20",
                    "--lateral-offset", str(offsets[name]),
                    "--survey-settle-sec", "4", "--speed", "2", "--ground-z", "1.5",
                    "--start-x", str(start_x), "--start-y", str(start_y), "--start-z", str(start_z),
                    "--ugv-start-x", str(dji_world_pose("husky")[0]),
                    "--ugv-start-y", str(dji_world_pose("husky")[1]),
                    "--ugv-start-z", str(dji_world_pose("husky")[2]),
                ]
                if dji_swarm:
                    follower_arguments.extend(["--role-state", str(role_state_file)])
                if single_dji_follower:
                    follower_arguments.extend(
                        ["--follow-only", "--follow-distance", "15.0"]
                    )
                for target_name in mission_targets:
                    pose_name = "husky" if target_name == "spawn" else target_name
                    target_x, target_y, _ = dji_world_pose(pose_name)
                    follower_arguments.extend(["--target", f"{target_name},{target_x},{target_y}"])
                followers.append(manager.start(f"{name}_follower", ros_command(follower_arguments)))
            if single_dji_follower:
                print(
                    "[DJI FOLLOW ONLY] DJI0 follows 15 m behind and 15 m above "
                    "the Husky for the fixed -45 deg camera; "
                    "aerial scouting and survey gating are OFF."
                )
                print(
                    "[UGV AUTONOMY] Husky Nav2, LiDAR costmaps and collision "
                    "monitor own every route leg and obstacle decision."
                )
            else:
                print(
                    f"[DJI FOLLOW] {len(followers)} DJI controller(s): native ENU movement, "
                    "20 m progressive scouting and decentralized role state."
                )
            for name, *_ in dji_starts:
                print(f"[CAMERA] /{name}/camera0/image_raw")
                print(
                    f"[YOLO FRAMES] {dataset.root / 'yolo_frames' / name}"
                )
            if dji_swarm:
                print("[SWARM STATUS] Current leader/scout: DJI0 | Followers: DJI1, DJI2")

            if single_dji_follower:
                follower_ready = dataset.root / "dji0_ready"
                deadline = time.monotonic() + 300.0
                while (
                    not stopping
                    and not follower_ready.exists()
                    and time.monotonic() < deadline
                ):
                    if followers[0].process.poll() is not None:
                        print("[FAILED] DJI0 stopped before reaching follow altitude.")
                        return 3
                    time.sleep(0.5)
                if stopping:
                    stop_file.touch()
                    return 0
                if not follower_ready.exists():
                    print("[FAILED] DJI0 follower takeoff timed out.")
                    stop_file.touch()
                    return 3
                print("[DJI FOLLOWER READY] DJI0 airborne; starting Husky Nav2 mission.")

            mission_code = 0
            reconnect_deadline: float | None = None

            def complete_reconnect() -> None:
                nonlocal swarm_state, reconnect_deadline
                if reconnect_deadline is None or time.monotonic() < reconnect_deadline:
                    return
                swarm_state = reconnect_dji(swarm_state, "dji0")
                write_dji_state(role_state_file, swarm_state)
                print("[RECONNECTED] DJI0 communication restored; rejoins as FOLLOWER.")
                print("[SWARM STATUS] Current leader/scout: DJI1 | Followers: DJI0, DJI2")
                reconnect_deadline = None

            for index, target_name in enumerate(mission_targets, start=1):
                complete_reconnect()
                if single_dji_follower:
                    print(
                        f"[FOLLOW-ONLY LEG {index}] Husky Nav2 driving to "
                        f"{target_name}; DJI0 follows without scouting."
                    )
                else:
                    survey_ready = coordination / f"survey_{index:02d}_ready"
                    if dji_swarm and swarm_state.active_scout is None:
                        survey_ready.touch()
                        print("[NAV2 FALLBACK] No DJI link remains; Husky owns navigation.")
                    print(
                        f"[DJI SURVEY] {(swarm_state.active_scout.upper() if dji_swarm and swarm_state.active_scout else 'DJI0')} "
                        f"scouting toward {target_name}; "
                        "the UGV remains stopped."
                    )
                    deadline = time.monotonic() + 300.0
                    while (
                        not stopping
                        and not survey_ready.exists()
                        and time.monotonic() < deadline
                    ):
                        complete_reconnect()
                        if any(item.process.poll() is not None for item in followers):
                            print("[FAILED] A DJI controller stopped during survey.")
                            return 3
                        time.sleep(0.5)
                    if stopping:
                        stop_file.touch()
                        return 0
                    if not survey_ready.exists():
                        print(f"[FAILED] DJI survey of {target_name} timed out.")
                        stop_file.touch()
                        return 3

                    print(
                        f"[DJI SURVEY READY] Forward lead to {target_name} is ready; "
                        "starting the UGV."
                    )
                leg = manager.start(
                    f"mission_leg_{index:02d}",
                    commands.waypoint_mission(
                        world_file, dataset.events_path, [target_name]
                    ),
                )
                print(f"[UGV LEG {index}] Nav2 driving to {target_name}.")
                while not stopping and leg.process.poll() is None:
                    complete_reconnect()
                    if any(item.process.poll() is not None for item in followers):
                        print("[FAILED] A DJI controller stopped while escorting the UGV.")
                        return 3
                    time.sleep(float(config["health_period_sec"]))
                if stopping:
                    stop_file.touch()
                    return 0
                mission_code = int(leg.process.returncode or 0)
                if mission_code != 0:
                    print(
                        f"[FAILED] UGV leg to {target_name} returned "
                        f"code {mission_code}."
                    )
                    break
                (coordination / f"leg_{index:02d}_complete").touch()
                dataset.event(
                    "cooperative_mission", "leg_completed",
                    leg=index, target=target_name,
                )
                if single_dji_follower:
                    print(
                        f"[LEG COMPLETE] UGV and DJI0 reached {target_name}; "
                        "continuing in follow-only mode."
                    )
                else:
                    print(
                        f"[LEG COMPLETE] UGV reached {target_name}; "
                        "DJI will scout the next leg."
                    )
                if dji_swarm and args.permanent_failure and index <= 3:
                    failed = f"dji{index - 1}"
                    swarm_state = fail_dji(swarm_state, failed, permanent=True)
                    write_dji_state(role_state_file, swarm_state)
                    leader = swarm_state.active_scout.upper() if swarm_state.active_scout else "UGV NAV2"
                    print(f"[DISCONNECTED] {failed.upper()} permanent communication failure.")
                    print(f"[RETURN TO BASE] {failed.upper()} returning to its launch point at z=1.5 m.")
                    print(f"[LEADER CHANGE] Current leader/scout -> {leader}.")
                elif dji_swarm and args.connection_failure_reconnect and index == 1:
                    swarm_state = fail_dji(swarm_state, "dji0", permanent=False)
                    write_dji_state(role_state_file, swarm_state)
                    reconnect_deadline = time.monotonic() + 30.0
                    print("[DISCONNECTED] DJI0 temporary communication failure; holding position.")
                    print("[LEADER CHANGE] Current leader/scout -> DJI1; DJI2 remains follower.")
                    print("[RECOVERY PENDING] DJI0 reconnect in 30 s; permanent threshold is 60 s.")

            stop_file.touch()
            print("[DJI LANDING] Landing 4 m clear of the Husky.")
            landing_deadline = time.monotonic() + 120.0
            while (
                not stopping
                and any(item.process.poll() is None for item in followers)
                and time.monotonic() < landing_deadline
            ):
                time.sleep(0.5)
            for item in followers:
                if item.process.poll() is None:
                    manager.stop(item.name)
            dji_code = 0 if all((item.process.returncode or 0) == 0 for item in followers) else 2
            if dji_code != 0 and mission_code == 0:
                mission_code = 3
            dataset.event("runner", "dji_mission_finished", return_code=dji_code)
            dataset.event("runner", "mission_finished", return_code=mission_code)
            print(f"[MISSION FINISHED] return code {mission_code}")
            if not args.headless:
                manager.stop("rosbag")
                manager.stop("localization_evaluator")
                dataset.event("runner", "recording_stopped")
                print(f"[SAVED] {dataset.root}")
                saved_reported = True
                print("[GAZEBO OPEN] Inspection mode; press Ctrl+C to close.")
                while not stopping and gazebo.process.poll() is None:
                    time.sleep(float(config["health_period_sec"]))
            return mission_code

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

#!/usr/bin/env python3
"""Launch the cooperative EiraX UGV and UAV simulation."""
from __future__ import annotations

import os
import argparse
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import yaml
from uav.failover_protocol import (
    elect_successor, initial_state, reconnect_as_follower, write_state,
)

COOPERATIVE_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = COOPERATIVE_ROOT.parent
sys.path.insert(0, str(PROJECT_ROOT))

from ground_stack.baseline.data_logging.run_dataset import RunDataset
from ground_stack.baseline.launchers import commands
from ground_stack.baseline.monitoring.topic_health import (
    gazebo_running,
    wait_for_finite_odometry,
    wait_for_message,
    wait_for_topics,
)
from ground_stack.baseline.mission.world_poses import (
    named_pose,
    rolling_costmap_plan,
    world_target_in_enu_odom,
)
from uav import mapping_commands as uav_mapping_commands
from uav.cooperative_world import build as build_cooperative_world
from cooperative_nav2_config import build as build_nav2_config
from uav.nav2_config import build as build_uav_nav2_config
from process_manager import ProcessManager


CONFIG_FILE = PROJECT_ROOT / "ground_stack/baseline/config/baseline.yaml"
PX4_RUNTIME = PROJECT_ROOT / "px4_runtime"
LINK_CONFIG_FILE = COOPERATIVE_ROOT / "communication/link_config.yaml"
SWARM_SITE_CONFIG = COOPERATIVE_ROOT / "config/swarm_launch_site.yaml"


def model_height(world_file: Path, name: str) -> float:
    """Read a model's saved world Z without changing the 2D mission API."""
    root = ET.parse(world_file).getroot()
    for include in root.findall(".//include"):
        if include.findtext("name") == name:
            values = [
                float(value)
                for value in include.findtext("pose", "0 0 0 0 0 0").split()
            ]
            return values[2]
    raise ValueError(f"Entity {name!r} is missing from {world_file}")


def lifecycle_state(node: str, env: dict[str, str]) -> str | None:
    """Read one Nav2 lifecycle state using the command-line interface."""
    try:
        result = subprocess.run(
            ["ros2", "lifecycle", "get", node],
            capture_output=True,
            text=True,
            timeout=5,
            env=env,
        )
    except subprocess.TimeoutExpired:
        return None
    if result.returncode != 0:
        return None
    return result.stdout.strip()


def wait_for_nav2_stack(env: dict[str, str], timeout: float) -> bool:
    """Require the UGV controller, planner, and navigator to all be active."""
    nodes = ("/controller_server", "/planner_server", "/bt_navigator")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        states = [lifecycle_state(node, env) for node in nodes]
        if all(state is not None and "active [3]" in state for state in states):
            return True
        time.sleep(1)
    return False


def wait_for_lifecycle_node(
    node: str,
    env: dict[str, str],
    timeout: float,
    *,
    require_active: bool,
) -> bool:
    """Wait for a lifecycle service, optionally requiring its active state."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = lifecycle_state(node, env)
        if state is not None and (
            not require_active or "active [3]" in state
        ):
            return True
        time.sleep(1)
    return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-motion", action="store_true", help="Start Nav2 but do not send a goal")
    parser.add_argument("--headless", action="store_true", help="Run Gazebo server without GUI and start simulation immediately")
    parser.add_argument(
        "--no-recording",
        action="store_true",
        help="Integration-test mode: do not start rosbag or localization CSV recording",
    )
    parser.add_argument(
        "--view-3d-slam",
        action="store_true",
        help="Build and display a live RTAB-Map 3D point-cloud map from the OS1-64 LiDAR",
    )
    parser.add_argument(
        "--return-to-spawn",
        action="store_true",
        help="After reaching the goal, navigate back to the saved Husky spawn pose",
    )
    parser.add_argument(
        "--ugv-only",
        dest="uav_follow",
        action="store_false",
        help="Diagnostic mode: start only the UGV side of this cooperative runner",
    )
    parser.set_defaults(uav_follow=True)
    parser.add_argument(
        "--uav-test-waypoint-1",
        action="store_true",
        help="Cooperative test only: stop after the first waypoint",
    )
    parser.add_argument(
        "--uav-count",
        type=int,
        choices=range(1, 4),
        default=1,
        metavar="{1,2,3}",
        help="Spawn up to three isolated PX4 x500_mapping instances (the accepted follower controls instance 0)",
    )
    failure = parser.add_mutually_exclusive_group()
    failure.add_argument("--permanent-failure", action="store_true")
    failure.add_argument("--connection-failure-reconnect", action="store_true")
    args = parser.parse_args()
    if args.uav_test_waypoint_1 and not args.uav_follow:
        parser.error("--uav-test-waypoint-1 requires --uav-follow")
    if args.uav_test_waypoint_1 and args.return_to_spawn:
        parser.error("--uav-test-waypoint-1 cannot be combined with --return-to-spawn")
    if gazebo_running():
        print("[FAILED] Another Gazebo server is already running. Close it first.")
        return 1

    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    link_config = yaml.safe_load(LINK_CONFIG_FILE.read_text(encoding="utf-8"))
    swarm_site_config = yaml.safe_load(SWARM_SITE_CONFIG.read_text(encoding="utf-8"))
    required_topics = list(config["required_topics"])
    slam_required_topics = list(config["slam_required_topics"])
    if args.uav_follow:
        # These belong only to the standalone independent LiDAR-evaluation
        # graph, which cooperative mode intentionally omits.
        required_topics.remove("/lidar3d/odom")
        slam_required_topics.remove("/rtabmap_lidar/mapData")
    world_file = (PROJECT_ROOT / config["world_file"]).resolve()
    uav_pad: tuple[float, float, float] | None = None
    if args.uav_follow:
        cooperative_world = Path("/tmp") / f"eirax_cooperative_{os.getpid()}.world"
        uav_pad = build_cooperative_world(
            world_file, cooperative_world, SWARM_SITE_CONFIG
        )
        world_file = cooperative_world
    mission_targets = ["waypoint_1", "waypoint_2", "waypoint_3", "goal"]
    if args.uav_test_waypoint_1:
        mission_targets = ["waypoint_1"]
    if args.return_to_spawn:
        mission_targets.append("spawn")
    global_size_m, global_resolution_m, longest_leg_m = rolling_costmap_plan(
        world_file, mission_targets
    )
    dataset = RunDataset(PROJECT_ROOT, world_file)
    env = os.environ.copy()
    model_path = str(PROJECT_ROOT / "simulation/models")
    resource_paths = [model_path]
    if args.uav_follow:
        resource_paths.append(str(PX4_RUNTIME / "models"))
    if env.get("GZ_SIM_RESOURCE_PATH"):
        resource_paths.append(env["GZ_SIM_RESOURCE_PATH"])
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(resource_paths)
    if args.uav_follow:
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(
            [str(PX4_RUNTIME / "plugins"), env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]
        ).rstrip(":")
        env["PYTHONPATH"] = ":".join(
            [
                str(PX4_RUNTIME / "python"),
                str(COOPERATIVE_ROOT),
                str(PROJECT_ROOT),
                str(PROJECT_ROOT / "ground_stack"),
                env.get("PYTHONPATH", ""),
            ]
        ).rstrip(":")
        assert uav_pad is not None
        env.update(
            {
                "PX4_SYS_AUTOSTART": "4001",
                "PX4_SIM_MODEL": "gz_x500_mapping",
                "PX4_GZ_STANDALONE": "1",
                "PX4_GZ_NO_FOLLOW": "1",
                "PX4_GZ_WORLD": "baylands_editable",
                "PX4_GZ_MODEL_POSE": f"{uav_pad[0]},{uav_pad[1]},{uav_pad[2] + 0.02}",
            }
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
        dataset.event("runner", "run_started")
        gazebo = manager.start("gazebo", commands.gazebo(world_file, headless=args.headless))
        time.sleep(3)
        manager.start("bridge", commands.bridge())
        manager.start("monotonic_clock", commands.monotonic_clock())
        if args.uav_follow:
            husky_spawn = named_pose(world_file, "husky")
            assert uav_pad is not None
            uav_up_offset = (
                uav_pad[2] + 0.02 - model_height(world_file, "husky")
            )
            manager.start(
                "communication_channel",
                uav_mapping_commands.communication_channel(
                    dataset.root / "communication",
                    LINK_CONFIG_FILE,
                    uav_pad[0] - husky_spawn.x,
                    uav_pad[1] - husky_spawn.y,
                ),
            )
            print(
                "[COMMUNICATION] Distance-limited UGV-UAV channel active "
                f"(normal lead {link_config['desired_lead_m']:.0f} m, "
                f"warning {link_config['warning_range_m']:.0f} m, "
                f"maximum {link_config['max_range_m']:.0f} m); "
                "all transferred messages are being measured in "
                f"{dataset.root / 'communication'}."
            )
            dataset.event(
                "communication",
                "distance_limited_link_started",
                output_directory=str(dataset.root / "communication"),
                max_range_m=float(link_config["max_range_m"]),
                warning_range_m=float(link_config["warning_range_m"]),
                reconnect_range_m=float(link_config["reconnect_range_m"]),
                desired_lead_m=float(link_config["desired_lead_m"]),
                intentional_delay_ms=0.0,
                packet_loss_rate=0.0,
                noise_enabled=False,
            )
            # Instance 0 is the accepted, validated follower. Additional PX4
            # instances are deliberately brought up as independent vehicles;
            # no second controller is attached to port 14540.
            # Three user-editable marked bays on the generated staging deck.
            swarm_spawn_offsets = tuple(
                (float(bay["offset_x"]), float(bay["offset_y"]))
                for bay in swarm_site_config["bays"]
            )
            for instance in range(args.uav_count):
                offset_x, offset_y = swarm_spawn_offsets[instance]
                instance_env = env.copy()
                instance_env["PX4_GZ_MODEL_POSE"] = (
                    f"{uav_pad[0] + offset_x},{uav_pad[1] + offset_y},"
                    f"{uav_pad[2] + 0.02},0,0,3.14159265"
                )
                manager.start(
                    f"uav{instance}_px4",
                    [str(PX4_RUNTIME / "bin/px4"), "-d", "-i", str(instance)],
                    cwd=PX4_RUNTIME / "rootfs",
                    stdin=subprocess.PIPE,
                    env=instance_env,
                )
                if instance + 1 < args.uav_count:
                    # Stagger model/plugin initialization to protect the
                    # Gazebo GUI thread on the validated rendering setup.
                    time.sleep(2)
            manager.start("uav_bridge", uav_mapping_commands.bridge())
            manager.start("uav_sensor_transform", uav_mapping_commands.sensor_transform())
            manager.start(
                "cooperative_frame_transform",
                uav_mapping_commands.cooperative_frame_transform(
                    uav_pad[0] - husky_spawn.x,
                    uav_pad[1] - husky_spawn.y,
                    uav_up_offset,
                ),
            )
            manager.start(
                "aerial_obstacle_filter",
                uav_mapping_commands.aerial_obstacle_filter(
                    uav_pad[0] - husky_spawn.x,
                    uav_pad[1] - husky_spawn.y,
                    uav_up_offset,
                    [
                        (
                            world_target_in_enu_odom(world_file, target).x,
                            world_target_in_enu_odom(world_file, target).y,
                        )
                        for target in mission_targets
                        if target != "spawn"
                    ],
                ),
            )
            manager.start(
                "uav_lidar_odometry",
                uav_mapping_commands.laser_odometry(cooperative=True),
            )
            manager.start(
                "uav_rtabmap",
                uav_mapping_commands.mapping(
                    dataset.root / "uav_aerial_rtabmap.db",
                    cooperative=True,
                ),
            )
        manager.start("lidar3d_transform", commands.lidar3d_transform())
        manager.start("imu_transform", commands.imu_transform())
        manager.start("gps_transform", commands.gps_transform())
        manager.start(
            "local_localization",
            commands.local_localization(
                PROJECT_ROOT / "ground_stack/baseline/config/local_ekf_params.yaml"
            ),
        )
        manager.start(
            "localization",
            commands.localization(
                PROJECT_ROOT / "ground_stack/baseline/config/ekf_params.yaml"
            ),
        )
        manager.start(
            "navsat_transform",
            commands.navsat_transform(
                PROJECT_ROOT / "ground_stack/baseline/config/navsat_params.yaml"
            ),
        )
        # Cooperative mode already runs the primary /rtabmap3d UGV map used
        # by Nav2. The independent /rtabmap_lidar evaluation graph duplicates
        # the same raw cloud at high CPU cost and is not part of navigation.
        # Keep it unchanged for every standalone UGV run.
        if not args.uav_follow:
            manager.start("lidar3d_odometry", commands.lidar3d_odometry())
            manager.start(
                "lidar3d_slam",
                commands.lidar3d_slam(dataset.root / "rtabmap_lidar.db"),
            )
        # Start 3D SLAM while Gazebo is still paused. This lets the operator
        # arrange RViz before starting simulation time.
        manager.start(
            "slam3d",
            commands.slam3d(dataset.root / "rtabmap3d.db"),
        )
        if args.view_3d_slam and not args.headless:
            time.sleep(1)
            manager.start(
                "slam3d_viewer",
                commands.slam3d_viewer(
                    PROJECT_ROOT
                    / "ground_stack/baseline/config/ugv_rtabmap_gui.ini"
                ),
            )
            print("[3D VIEW] RTAB-Map UI opened. Configure it before clicking Play.")

        if args.headless:
            print("[HEADLESS] Gazebo server started and simulation is running.")
        else:
            print("[PAUSED] Wait for the models to load, then click Play in Gazebo.")
            print(f"[WAITING] Sensor startup timeout: {config['startup_timeout_sec']} seconds")
        healthy, missing = wait_for_topics(
            required_topics, float(config["startup_timeout_sec"])
        )
        if not healthy:
            dataset.event("health", "startup_failed", missing_topics=missing)
            print(f"[FAILED] Missing topics: {', '.join(missing)}")
            return 2

        if not wait_for_message("/husky/lidar3d/points", float(config["startup_timeout_sec"])):
            dataset.event("health", "sensor_data_timeout", topic="/husky/lidar3d/points")
            print("[FAILED] No OS1-64 point cloud received. Click Play and try again.")
            return 2

        if not wait_for_finite_odometry(
            "/odometry/gps", float(config["startup_timeout_sec"])
        ):
            dataset.event("health", "invalid_gps_odometry")
            print("[FAILED] GNSS odometry is missing or contains invalid values.")
            return 2

        print("[OK] Gazebo, bridge, 3D LiDAR, GNSS, odometry, TF and IMU")
        if args.uav_follow:
            if not wait_for_message(
                "/communication/uav/rx/ugv_gps",
                float(config["startup_timeout_sec"]),
            ):
                print("[FAILED] UGV-to-UAV communication channel has no GNSS data.")
                return 2
            if not wait_for_message(uav_mapping_commands.POINTS, float(config["startup_timeout_sec"])):
                print("[FAILED] Cooperative UAV 3D LiDAR is not publishing.")
                return 2
            if not wait_for_message("/uav/lidar_odom", float(config["startup_timeout_sec"])):
                print("[FAILED] Cooperative UAV LiDAR odometry is not publishing.")
                return 2
            print("[OK] Cooperative UAV, 3D LiDAR, aerial odometry and RTAB-Map")
        map_ready, map_missing = wait_for_topics(
            slam_required_topics, float(config["startup_timeout_sec"])
        )
        if not map_ready:
            dataset.event("health", "slam3d_startup_failed", missing_topics=map_missing)
            print(f"[FAILED] 3D SLAM missing topics: {', '.join(map_missing)}")
            return 2

        if not wait_for_message("/map", float(config["startup_timeout_sec"])):
            dataset.event("health", "slam3d_data_timeout", topic="/map")
            print("[FAILED] RTAB-Map did not publish its projected map.")
            return 2

        print("[OK] RTAB-Map 3D SLAM and projected Nav2 map are publishing")
        dataset.event("health", "baseline_ready", topics=required_topics)
        record_topics = list(config["record_topics"])
        if args.uav_follow:
            record_topics = [
                topic
                for topic in record_topics
                if topic != "/lidar3d/odom"
                and not topic.startswith("/lidar3d/odom_")
                and topic != "/lidar_slam/map"
                and not topic.startswith("/rtabmap_lidar/")
            ]
            record_topics.extend(
                [
                    uav_mapping_commands.POINTS,
                    "/communication/link/status",
                    "/communication/link/distance_m",
                    "/communication/link/quality",
                    "/communication/link/state",
                    "/communication/ugv/rx/uav_odom",
                    "/cooperative/aerial_obstacles",
                    "/cooperative/ugv_global_path",
                    "/global_costmap/costmap",
                    "/uav_nav/global_costmap/costmap",
                    "/uav/lidar_odom",
                    "/uav_rtabmap/cloud_map",
                    "/uav_rtabmap/mapGraph",
                    "/coord/support/uav0/leader_detection",
                    "/coord/support/uav0/leader_detection_status",
                    "/coord/support/uav0/leader_estimate",
                    "/coord/support/uav1/leader_detection",
                    "/coord/support/uav1/leader_detection_status",
                    "/coord/support/uav1/leader_estimate",
                    "/coord/support/uav2/leader_detection",
                    "/coord/support/uav2/leader_detection_status",
                    "/coord/support/uav2/leader_estimate",
                    "/coord/swarm/semantic_observations",
                ]
            )
        if not args.uav_follow and not args.no_recording:
            manager.start(
                "localization_evaluator",
                commands.localization_evaluator(
                    dataset.root / "localization/trajectory_and_errors.csv"
                ),
            )
            print("[LOCALIZATION] Ground truth and estimator errors are being recorded.")
            manager.start("rosbag", commands.recorder(dataset.rosbag, record_topics))
            print(f"[RECORDING] {dataset.root}")
        nav2_params = build_nav2_config(
            dataset.root / "config/nav2_params.yaml",
            global_size_m=global_size_m,
            global_resolution_m=global_resolution_m,
            use_aerial_guidance=args.uav_follow,
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
        print("[WAITING] Nav2 is starting with the Husky footprint and conservative speed limits.")
        uav_stop_file = dataset.root / "uav_stop"
        uav_ready_file = dataset.root / "uav_ready"
        uav_ready_files = [uav_ready_file]
        coordination_dir: Path | None = None
        uav_follower = None
        uav_followers = []
        if args.uav_follow:
            if not wait_for_nav2_stack(env, 30.0):
                dataset.event("health", "nav2_initial_activation_failed")
                print(
                    "[RECOVERY] UGV Nav2 was only partially active; "
                    "restarting the complete Nav2 process group once."
                )
                manager.stop("nav2")
                time.sleep(3)
                manager.start("nav2_retry", commands.nav2(nav2_params))
            if not wait_for_nav2_stack(env, 90.0):
                states = {
                    node: lifecycle_state(node, env)
                    for node in (
                        "/controller_server",
                        "/planner_server",
                        "/bt_navigator",
                    )
                }
                dataset.event(
                    "health", "nav2_activation_timeout", lifecycle_states=states
                )
                print(f"[FAILED] UGV Nav2 is not fully active: {states}")
                return 3
            print("[OK] UGV Nav2 controller, planner and navigator are active.")

            if not args.no_motion:
                # The follower publishes uav_px4_odom -> uav_base_link. Start
                # it before activating the planner costmap so lifecycle
                # activation cannot deadlock while waiting for that transform.
                coordination_dir = dataset.root / "cooperative"
                coordination_dir.mkdir(parents=True, exist_ok=True)
                swarm_state_file = coordination_dir / "swarm_role_state.json"
                swarm_state = initial_state()
                write_state(swarm_state_file, swarm_state)
                husky_spawn = named_pose(world_file, "husky")
                assert uav_pad is not None
                east_offset = uav_pad[0] - husky_spawn.x
                north_offset = uav_pad[1] - husky_spawn.y
                follower_command = [
                    "python3", "-u", "-m", "uav.follow_husky",
                    "--output", str(dataset.root / "uav_follow_trajectory.csv"),
                    "--stop-file", str(uav_stop_file),
                    "--ready-file", str(uav_ready_file),
                    "--coordination-dir", str(coordination_dir),
                    "--altitude", "15",
                    "--port", "14540",
                    "--uav-id", "uav",
                    "--failover-id", "uav0",
                    "--swarm-state-file", str(swarm_state_file),
                    "--odom-topic", "/uav/px4_odom",
                    "--source-system", "251",
                    "--lead-distance", str(link_config["desired_lead_m"]),
                    "--max-link-range", str(link_config["max_range_m"]),
                    "--link-warning-range", str(link_config["warning_range_m"]),
                    "--survey-settle-sec", str(link_config["survey_settle_sec"]),
                    "--landing-delay-sec", "0",
                ]
                for target_name in mission_targets:
                    target = world_target_in_enu_odom(world_file, target_name)
                    follower_command.extend(
                        [
                            "--survey-target",
                            (
                                f"{target_name},"
                                f"{target.y - north_offset},"
                                f"{target.x - east_offset}"
                            ),
                        ]
                    )
                uav_follower = manager.start("uav_follower", follower_command)
                uav_followers.append(uav_follower)
                expected_uav_returns: set[str] = set()
                reconnect_deadline: float | None = None
                # The two peer aircraft use their own MAVLink endpoints and
                # vertically/laterally separated formation slots. UAV 0 alone
                # owns the mapping planner; peers use bounded direct steps so
                # they cannot contend for its Nav2 action server or LiDAR.
                peer_slots = ((1, 14541, 19.0, 0.0, 7.0), (2, 14542, 23.0, 0.0, -7.0))
                for peer_index, port, altitude, offset_north, offset_east in peer_slots[: max(0, args.uav_count - 1)]:
                    peer_ready = dataset.root / f"uav{peer_index}_ready"
                    uav_ready_files.append(peer_ready)
                    peer_command = [
                        "python3", "-u", "-m", "uav.follow_husky",
                        "--output", str(dataset.root / f"uav{peer_index}_follow_trajectory.csv"),
                        "--stop-file", str(uav_stop_file),
                        "--ready-file", str(peer_ready),
                        "--altitude", str(altitude),
                        "--port", str(port),
                        "--uav-id", f"uav{peer_index}",
                        "--failover-id", f"uav{peer_index}",
                        "--swarm-state-file", str(swarm_state_file),
                        "--coordination-dir", str(coordination_dir),
                        "--preserve-coordination",
                        "--odom-topic", f"/uav{peer_index}/px4_odom",
                        "--source-system", str(251 + peer_index),
                        "--formation-north", str(offset_north),
                        "--formation-east", str(offset_east),
                        "--direct-follow",
                        "--landing-delay-sec", str(peer_index * 12),
                        "--lead-distance", str(link_config["desired_lead_m"]),
                        "--max-link-range", str(link_config["max_range_m"]),
                        "--link-warning-range", str(link_config["warning_range_m"]),
                        "--survey-settle-sec", str(link_config["survey_settle_sec"]),
                    ]
                    for target_name in mission_targets:
                        target = world_target_in_enu_odom(world_file, target_name)
                        peer_command.extend(["--survey-target", f"{target_name},{target.y - north_offset},{target.x - east_offset}"])
                    peer = manager.start(f"uav{peer_index}_follower", peer_command)
                    uav_followers.append(peer)
                    time.sleep(2)
                print(
                    "[WAITING] UAV is starting and publishing its flight-control pose."
                )
                if not wait_for_message("/uav/px4_odom", 90.0):
                    if uav_follower.process.poll() is not None:
                        print(
                            "[FAILED] UAV follower stopped before publishing its pose."
                        )
                    else:
                        print("[FAILED] UAV flight-control pose did not start.")
                    return 3
                print("[OK] UAV pose and transform are publishing.")

                uav_nav2_params = build_uav_nav2_config(
                    dataset.root / "config/uav_nav2_params.yaml"
                )
                manager.start(
                    "uav_nav2_planner",
                    uav_mapping_commands.nav2_planner(uav_nav2_params),
                )
                if not wait_for_lifecycle_node(
                    "/uav_nav/planner_server",
                    env,
                    30.0,
                    require_active=False,
                ):
                    print("[FAILED] UAV Nav2 planner lifecycle service did not start.")
                    return 3
                manager.start(
                    "uav_nav2_lifecycle",
                    uav_mapping_commands.nav2_lifecycle(),
                )
                print("[WAITING] UAV Nav2 obstacle planner is starting in /uav_nav.")
                if not wait_for_lifecycle_node(
                    "/uav_nav/planner_server",
                    env,
                    45.0,
                    require_active=True,
                ):
                    state = lifecycle_state("/uav_nav/planner_server", env)
                    dataset.event(
                        "health",
                        "uav_nav2_initial_activation_failed",
                        lifecycle_state=state,
                    )
                    print(
                        f"[RECOVERY] UAV Nav2 planner is {state or 'unavailable'}; "
                        "restarting its planner and lifecycle manager once."
                    )
                    manager.stop("uav_nav2_lifecycle")
                    manager.stop("uav_nav2_planner")
                    time.sleep(2)
                    manager.start(
                        "uav_nav2_planner_retry",
                        uav_mapping_commands.nav2_planner(uav_nav2_params),
                    )
                    if not wait_for_lifecycle_node(
                        "/uav_nav/planner_server",
                        env,
                        30.0,
                        require_active=False,
                    ):
                        print(
                            "[FAILED] Restarted UAV Nav2 planner lifecycle "
                            "service did not start."
                        )
                        return 3
                    manager.start(
                        "uav_nav2_lifecycle_retry",
                        uav_mapping_commands.nav2_lifecycle(),
                    )
                    if not wait_for_lifecycle_node(
                        "/uav_nav/planner_server",
                        env,
                        60.0,
                        require_active=True,
                    ):
                        state = lifecycle_state("/uav_nav/planner_server", env)
                        dataset.event(
                            "health",
                            "uav_nav2_activation_timeout",
                            lifecycle_state=state,
                        )
                        print(
                            "[FAILED] UAV Nav2 obstacle planner did not become "
                            f"active: {state or 'unavailable'}."
                        )
                        return 3
                print("[OK] UAV Nav2 obstacle planner is active.")
            else:
                print(
                    "[NO MOTION] UAV follower and aerial obstacle planner are "
                    "intentionally not started."
                )
            if args.view_3d_slam and not args.headless:
                manager.start(
                    "uav_slam_viewer",
                    uav_mapping_commands.rtabmap_viewer(
                        COOPERATIVE_ROOT / "uav/config/aerial_rtabmap_gui.ini"
                    ),
                )
                print("[UAV 3D VIEW] Sharp height-colored RTAB-Map aerial view opened.")
            if not args.no_recording:
                manager.start(
                    "localization_evaluator",
                    commands.localization_evaluator(
                        dataset.root / "localization/trajectory_and_errors.csv"
                    ),
                )
                print(
                    "[LOCALIZATION] Ground truth and estimator errors "
                    "are being recorded."
                )
                manager.start(
                    "rosbag", commands.recorder(dataset.rosbag, record_topics)
                )
                print(f"[RECORDING] {dataset.root}")
            else:
                print("[TEST MODE] Sensor bag and localization CSV are disabled.")
        if args.no_motion:
            print("[NO MOTION] Nav2 validation only. Press Ctrl+C to stop.")
            while not stopping:
                if failures := manager.failures(ignored={"slam3d_viewer", "uav_slam_viewer"}):
                    dataset.event("health", "component_failed", failures=failures)
                    print(f"[FAILED] {failures}")
                    return 3
                time.sleep(float(config["health_period_sec"]))
            return 0
        if args.uav_follow:
            assert coordination_dir is not None
            assert uav_follower is not None
            # Gazebo may intentionally run slower than wall time with three
            # PX4 instances and RGB-D sensors. Do not abort healthy climbing
            # aircraft at the old two-minute wall-clock boundary.
            ready_deadline = time.monotonic() + 360.0
            while not all(path.exists() for path in uav_ready_files) and time.monotonic() < ready_deadline:
                if any(item.process.poll() is not None and item.name not in expected_uav_returns for item in uav_followers):
                    print("[FAILED] A UAV follower stopped before reaching its formation position.")
                    return 3
                time.sleep(0.5)
            if not all(path.exists() for path in uav_ready_files):
                missing = [path.name for path in uav_ready_files if not path.exists()]
                print(f"[FAILED] UAV formation takeoff timed out: {missing}")
                return 3
            print(f"[UAV READY] All {len(uav_followers)} UAVs are airborne in formation; waiting for the first aerial survey.")

            mission_code = 0
            for leg_index, target_name in enumerate(mission_targets, start=1):
                survey_ready = coordination_dir / f"survey_{leg_index:02d}_ready"
                if swarm_state.active_scout is None:
                    survey_ready.touch()
                    print(f"[NAV2 FALLBACK] No UAV link remains; UGV owns navigation to {target_name}.")
                print(
                    f"[SURVEY] {swarm_state.active_scout or 'UGV Nav2'} is preparing the corridor to {target_name}; "
                    "the UGV remains stopped."
                )
                survey_deadline = time.monotonic() + 900.0
                while (
                    not stopping
                    and not survey_ready.exists()
                    and time.monotonic() < survey_deadline
                ):
                    if reconnect_deadline is not None and time.monotonic() >= reconnect_deadline:
                        swarm_state = reconnect_as_follower(swarm_state, "uav0")
                        write_state(swarm_state_file, swarm_state)
                        dataset.event("failover", "uav_reconnected", uav="uav0", active_scout=swarm_state.active_scout, term=swarm_state.term)
                        print("[RECONNECTED] UAV0 rejoined as follower; UAV1 remains scout.")
                        reconnect_deadline = None
                    if any(item.process.poll() is not None and item.name not in expected_uav_returns for item in uav_followers):
                        print(
                            f"[FAILED] UAV stopped while surveying {target_name}."
                        )
                        return 3
                    time.sleep(0.5)
                if stopping:
                    return 0
                if not survey_ready.exists():
                    print(f"[FAILED] UAV survey timed out for {target_name}.")
                    return 3

                print(
                    f"[PROGRESSIVE MAP READY] UAV has established the forward "
                    f"lead toward {target_name}; the UGV may move while aerial "
                    "mapping continues."
                )

                mission = manager.start(
                    f"mission_leg_{leg_index:02d}",
                    commands.waypoint_mission(
                        world_file,
                        dataset.events_path,
                        [target_name],
                    ),
                )
                print(
                    f"[UGV LEG {leg_index}] Existing UGV Nav2 is dynamically "
                    f"planning and driving to {target_name}."
                )
                while not stopping and mission.process.poll() is None:
                    if any(item.process.poll() is not None and item.name not in expected_uav_returns for item in uav_followers):
                        print(
                            f"[FAILED] UAV stopped while escorting the UGV "
                            f"to {target_name}."
                        )
                        return 3
                    time.sleep(float(config["health_period_sec"]))
                if stopping:
                    return 0
                mission_code = mission.process.returncode
                if mission_code != 0:
                    print(
                        f"[FAILED] UGV leg to {target_name} returned "
                        f"code {mission_code}."
                    )
                    break
                (coordination_dir / f"leg_{leg_index:02d}_complete").touch()
                dataset.event(
                    "cooperative_mission",
                    "leg_completed",
                    leg=leg_index,
                    target=target_name,
                )
                print(
                    f"[LEG COMPLETE] UGV reached {target_name}; "
                    "the UAV will survey the next leg."
                )

                if args.permanent_failure and leg_index <= 3:
                    failed = f"uav{leg_index - 1}"
                    process_name = "uav_follower" if failed == "uav0" else f"{failed}_follower"
                    expected_uav_returns.add(process_name)
                    swarm_state = elect_successor(swarm_state, failed, permanent=True)
                    write_state(swarm_state_file, swarm_state)
                    dataset.event("failover", "permanent_failure", failed_uav=failed, active_scout=swarm_state.active_scout, term=swarm_state.term)
                    print(f"[PERMANENT FAILURE] {failed} returning to bay; new scout={swarm_state.active_scout or 'none (UGV Nav2)'}.")
                elif args.connection_failure_reconnect and leg_index == 1:
                    swarm_state = elect_successor(swarm_state, "uav0", permanent=False)
                    write_state(swarm_state_file, swarm_state)
                    reconnect_deadline = time.monotonic() + 30.0
                    dataset.event("failover", "temporary_disconnect", failed_uav="uav0", active_scout=swarm_state.active_scout, reconnect_after_sec=30.0, permanent_threshold_sec=60.0, term=swarm_state.term)
                    print("[LINK LOST] UAV0 holding in place; UAV1 is scout. Reconnect is scheduled after 30 s (before the 60 s permanent threshold).")

            uav_stop_file.touch()
            if mission_code == 0:
                print("[UAV RETURN] Goal reached; returning all UAVs to their launch bays.")
            else:
                print(
                    "[UAV RETURN] UGV mission stopped early; returning all UAVs "
                    "to their launch bays."
                )
            landing_deadline = time.monotonic() + 900.0
            while (
                not stopping
                and any(item.process.poll() is None for item in uav_followers)
                and time.monotonic() < landing_deadline
            ):
                time.sleep(0.5)
            for item in uav_followers:
                if item.process.poll() is None:
                    manager.stop(item.name)
            codes = [item.process.returncode for item in uav_followers]
            uav_code = 0 if all(code == 0 for code in codes) else (130 if stopping else 2)
            if uav_code != 0 and mission_code == 0:
                mission_code = 3
            dataset.event("runner", "uav_mission_finished", return_code=uav_code)
            dataset.event("runner", "mission_finished", return_code=mission_code)
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
            if (mission_code := mission.process.poll()) is not None:
                if args.uav_follow and uav_follower is not None:
                    uav_stop_file.touch()
                    print("[UAV LANDING] Husky mission ended; waiting for UAV landing.")
                    landing_deadline = time.monotonic() + 150.0
                    while (
                        not stopping
                        and uav_follower.process.poll() is None
                        and time.monotonic() < landing_deadline
                    ):
                        time.sleep(0.5)
                    if uav_follower.process.poll() is None:
                        manager.stop("uav_follower")
                        uav_code = 130 if stopping else 2
                    else:
                        uav_code = uav_follower.process.returncode
                    dataset.event("runner", "uav_mission_finished", return_code=uav_code)
                    if uav_code != 0 and mission_code == 0:
                        mission_code = 3
                dataset.event("runner", "mission_finished", return_code=mission_code)
                print(f"[MISSION FINISHED] return code {mission_code}")
                if not args.headless:
                    manager.stop("rosbag")
                    manager.stop("localization_evaluator")
                    dataset.event("runner", "recording_stopped")
                    print(f"[SAVED] {dataset.root}")
                    saved_reported = True
                    if mission_code == 0:
                        print("[GAZEBO OPEN] Recording has stopped. Close Gazebo when you are finished.")
                    else:
                        print("[GAZEBO OPEN] Mission stopped with an error; Gazebo is being kept open for inspection.")
                    while not stopping and gazebo.process.poll() is None:
                        time.sleep(float(config["health_period_sec"]))
                return mission_code
            if failures := manager.failures(ignored={"slam3d_viewer", "uav_slam_viewer"}):
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
        # Process cleanup must never be skipped because an active dataset was
        # externally removed or the disk became unavailable.
        manager.stop_all()
        try:
            dataset.event("runner", "run_finished")
        except OSError as error:
            print(f"[DATASET WARNING] Could not write final event: {error}")
        if not saved_reported:
            print(f"[SAVED] {dataset.root}")


if __name__ == "__main__":
    raise SystemExit(main())

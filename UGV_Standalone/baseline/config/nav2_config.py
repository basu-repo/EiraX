"""Generate the standalone UGV Nav2 configuration."""

from __future__ import annotations

from pathlib import Path
import yaml


DEFAULT = Path("/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml")
DEFAULT_NAV_TO_POSE_TREE = Path(
    "/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/"
    "navigate_to_pose_w_replanning_and_recovery.xml"
)
FOOTPRINT = "[[0.55, 0.38], [0.55, -0.38], [-0.55, -0.38], [-0.55, 0.38]]"


def build(
    output: Path,
    *,
    global_size_m: int = 60,
    global_resolution_m: float = 0.05,
) -> Path:
    data = yaml.safe_load(DEFAULT.read_text(encoding="utf-8"))
    if global_size_m <= 120:
        controller_frequency, vx_samples, vtheta_samples, replan_hz, bond_timeout = (
            10.0, 30, 24, 1.0, 10.0
        )
    elif global_size_m <= 300:
        controller_frequency, vx_samples, vtheta_samples, replan_hz, bond_timeout = (
            8.0, 24, 20, 0.5, 15.0
        )
    else:
        controller_frequency, vx_samples, vtheta_samples, replan_hz, bond_timeout = (
            6.0, 20, 16, 0.25, 20.0
        )

    def update(value):
        if isinstance(value, dict):
            for key, item in list(value.items()):
                if key == "use_sim_time":
                    value[key] = True
                elif key in ("robot_base_frame", "base_frame_id"):
                    value[key] = "base_link"
                elif key == "robot_radius":
                    value.pop(key)
                    value["footprint"] = FOOTPRINT
                else:
                    update(item)
        elif isinstance(value, list):
            for item in value:
                update(item)

    update(data)
    data["controller_server"]["ros__parameters"]["controller_frequency"] = controller_frequency
    data["controller_server"]["ros__parameters"]["FollowPath"] = {
        "plugin": "dwb_core::DWBLocalPlanner",
        "debug_trajectory_details": False,
        "min_vel_x": 0.0,
        "max_vel_x": 0.80,
        "max_vel_theta": 0.60,
        "min_speed_xy": 0.0,
        "max_speed_xy": 0.80,
        "min_speed_theta": 0.0,
        "acc_lim_x": 1.00,
        "acc_lim_theta": 0.80,
        "decel_lim_x": -1.00,
        "decel_lim_theta": -0.80,
        "vx_samples": vx_samples,
        "vy_samples": 1,
        "vtheta_samples": vtheta_samples,
        "sim_time": 2.0,
        "linear_granularity": 0.05,
        "angular_granularity": 0.025,
        "transform_tolerance": 0.20,
        "xy_goal_tolerance": 1.0,
        "trans_stopped_velocity": 0.15,
        "short_circuit_trajectory_evaluation": True,
        "stateful": True,
        "critics": [
            "RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign",
            "PathAlign", "PathDist", "GoalDist"
        ],
        "BaseObstacle.scale": 0.02,
        "PathAlign.scale": 24.0,
        "PathAlign.forward_point_distance": 0.35,
        "GoalAlign.scale": 18.0,
        "GoalAlign.forward_point_distance": 0.35,
        "PathDist.scale": 24.0,
        "GoalDist.scale": 18.0,
        "RotateToGoal.scale": 24.0,
        "RotateToGoal.slowing_factor": 5.0,
        "RotateToGoal.lookahead_time": -1.0,
    }
    goal_checker = data["controller_server"]["ros__parameters"]["general_goal_checker"]
    goal_checker["plugin"] = "nav2_controller::PositionGoalChecker"
    goal_checker["stateful"] = False
    goal_checker["xy_goal_tolerance"] = 1.0
    goal_checker.pop("yaw_goal_tolerance", None)
    progress_checker = data["controller_server"]["ros__parameters"]["progress_checker"]
    progress_checker["movement_time_allowance"] = 10.0
    velocity_smoother = data["velocity_smoother"]["ros__parameters"]
    velocity_smoother["max_velocity"] = [0.80, 0.0, 0.60]
    velocity_smoother["min_velocity"] = [-0.15, 0.0, -0.60]
    velocity_smoother["max_accel"] = [1.00, 0.0, 0.80]
    velocity_smoother["max_decel"] = [-1.00, 0.0, -0.80]
    data["controller_server"]["ros__parameters"]["failure_tolerance"] = 0.5
    data["planner_server"]["ros__parameters"]["expected_planner_frequency"] = replan_hz
    lifecycle = data.setdefault("lifecycle_manager_navigation", {}).setdefault("ros__parameters", {})
    lifecycle["service_timeout"] = 20.0
    lifecycle["bond_timeout"] = bond_timeout

    # A large rolling map and two live 3D SLAM estimators need more CPU per
    # planning cycle. Generate a mission-sized behavior tree instead of
    # hard-coding the default 1 Hz global replanning rate for every world.
    behavior_tree = output.parent / "navigate_to_pose_dynamic.xml"
    behavior_tree.parent.mkdir(parents=True, exist_ok=True)
    behavior_tree.write_text(
        DEFAULT_NAV_TO_POSE_TREE.read_text(encoding="utf-8").replace(
            '<RateController hz="1.0">',
            f'<RateController hz="{replan_hz:.2f}">',
            1,
        ),
        encoding="utf-8",
    )
    data["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] = str(
        behavior_tree
    )

    # Mission markers are fixed relative to the Husky's starting pose. Keep
    # navigation in odom so SLAM scan-matching corrections cannot move those
    # physical targets while the mission is running. SLAM still publishes and
    # records the map independently.
    data["bt_navigator"]["ros__parameters"]["global_frame"] = "odom"
    data["behavior_server"]["ros__parameters"]["global_frame"] = "odom"

    local = data["local_costmap"]["local_costmap"]["ros__parameters"]
    local["width"] = 12
    local["height"] = 12
    local_voxel = local["voxel_layer"]
    ugv_obstacle_topic = "/rtabmap3d/local_grid_obstacle"
    local_voxel["observation_sources"] = "lidar3d"
    local_voxel.pop("scan", None)
    local_voxel["lidar3d"] = {
        "topic": ugv_obstacle_topic,
        "data_type": "PointCloud2",
        # Baylands' uneven mesh still produces segmented terrain returns up to
        # about 0.68 m in base_link. Keep walls and substantial obstacles.
        "min_obstacle_height": 0.70,
        "max_obstacle_height": 2.5,
        "clearing": True,
        "marking": True,
        "raytrace_min_range": 0.8,
        "raytrace_max_range": 12.0,
        "obstacle_min_range": 0.8,
        "obstacle_max_range": 10.0,
    }

    # The first SLAM map only covers currently observed cells. A rolling global
    # costmap lets Nav2 plan toward a mission waypoint beyond that initial
    # rectangle while LiDAR continuously marks and clears obstacles.
    global_costmap = data["global_costmap"]["global_costmap"]["ros__parameters"]
    global_costmap["global_frame"] = "odom"
    global_costmap["rolling_window"] = True
    global_costmap["width"] = global_size_m
    global_costmap["height"] = global_size_m
    global_costmap["resolution"] = global_resolution_m
    global_costmap["track_unknown_space"] = False
    global_costmap["plugins"] = ["obstacle_layer", "inflation_layer"]
    global_obstacles = global_costmap["obstacle_layer"]
    global_obstacles["observation_sources"] = "lidar3d"
    global_obstacles.pop("scan", None)
    global_obstacles["lidar3d"] = {
        "topic": ugv_obstacle_topic,
        "data_type": "PointCloud2",
        "min_obstacle_height": 0.70,
        "max_obstacle_height": 2.5,
        "clearing": True,
        "marking": True,
        "raytrace_min_range": 0.8,
        "raytrace_max_range": 35.0,
        "obstacle_min_range": 0.8,
        "obstacle_max_range": 30.0,
    }
    collision = data["collision_monitor"]["ros__parameters"]
    # RTAB-Map publishes the terrain-segmented local cloud at about 1 Hz and
    # can add sub-second processing delay as the map grows.
    collision["source_timeout"] = 3.0
    collision["observation_sources"] = ["lidar3d"]
    collision.pop("scan", None)
    collision["lidar3d"] = {
        "type": "pointcloud",
        "topic": ugv_obstacle_topic,
        "min_height": 0.70,
        "max_height": 2.5,
        "enabled": True,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return output

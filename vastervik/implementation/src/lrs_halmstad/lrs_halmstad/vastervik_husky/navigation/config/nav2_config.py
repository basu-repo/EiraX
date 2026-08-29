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
# Keep the proven UGV_UAV obstacle-clearance envelope explicit instead of
# relying on whichever values happen to ship in Nav2's system defaults.
# Proven cooperative UGV clearance envelope.  A 0.70 m inflation radius let
# the planner skim Baylands building corners and thin shelter columns.  The
# wider, slower-decaying field keeps the rectangular Husky footprint clear.
INFLATION_RADIUS_M = 1.20
INFLATION_COST_SCALING = 1.50
COLLISION_LOOKAHEAD_S = 2.00
OUTER_SLOW_RADIUS_M = 4.0
OUTER_SLOW_RATIO = 0.50
INNER_LIMIT_RADIUS_M = 2.0
INNER_LINEAR_LIMIT_MPS = 0.50
# Normal DWB planning is forward-only. Reverse is reserved for Nav2's explicit
# BackUp recovery, which is distance-bounded by the behavior tree (0.30 m at
# 0.15 m/s) and therefore cannot become the normal route-following behavior.
RECOVERY_REVERSE_SPEED_MPS = 0.15


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
        "max_vel_x": 2.00,
        "max_vel_theta": 1.00,
        "min_speed_xy": 0.0,
        "max_speed_xy": 2.00,
        "min_speed_theta": 0.0,
        "acc_lim_x": 1.20,
        "acc_lim_theta": 0.60,
        "decel_lim_x": -1.20,
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
        "BaseObstacle.scale": 0.15,
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
    velocity_smoother["max_velocity"] = [2.00, 0.0, 1.00]
    # Keep just enough negative range for the bounded BackUp recovery action.
    # DWB itself cannot request reverse because FollowPath.min_vel_x is zero.
    velocity_smoother["min_velocity"] = [
        -RECOVERY_REVERSE_SPEED_MPS, 0.0, -1.00
    ]
    velocity_smoother["max_accel"] = [1.20, 0.0, 0.60]
    velocity_smoother["max_decel"] = [-1.20, 0.0, -0.80]
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
    local["inflation_layer"]["inflation_radius"] = INFLATION_RADIUS_M
    local["inflation_layer"]["cost_scaling_factor"] = INFLATION_COST_SCALING
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
    global_costmap["inflation_layer"]["inflation_radius"] = INFLATION_RADIUS_M
    global_costmap["inflation_layer"]["cost_scaling_factor"] = INFLATION_COST_SCALING
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
    # Keep costmap marking on RTAB-Map's terrain-segmented cloud, but collision
    # stopping must consume the fresh sensor stream directly.  Under the full
    # three-DJI/YOLO load the segmented cloud can lag enough to permit contact.
    collision["source_timeout"] = 3.0
    collision["polygons"] = [
        "OuterSlowZone", "InnerLimitZone", "FootprintApproach"
    ]
    collision["OuterSlowZone"] = {
        "type": "circle",
        "radius": OUTER_SLOW_RADIUS_M,
        "action_type": "slowdown",
        "slowdown_ratio": OUTER_SLOW_RATIO,
        "min_points": 6,
        "visualize": False,
        "enabled": True,
    }
    collision["InnerLimitZone"] = {
        "type": "circle",
        "radius": INNER_LIMIT_RADIUS_M,
        "action_type": "limit",
        "linear_limit": INNER_LINEAR_LIMIT_MPS,
        "angular_limit": 0.60,
        "min_points": 6,
        "visualize": False,
        "enabled": True,
    }
    collision["FootprintApproach"]["time_before_collision"] = COLLISION_LOOKAHEAD_S
    # Use the compact, height-filtered safety scan for immediate collision
    # actions. RTAB-Map and the costmaps retain their segmented 3D cloud.
    collision["observation_sources"] = ["safety_scan"]
    collision.pop("lidar3d", None)
    collision.pop("scan", None)
    collision["safety_scan"] = {
        "type": "scan",
        "topic": "/husky/safety_scan",
        "enabled": True,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    # Preserve literal UTF-8 if this deployable tree is placed beneath a
    # non-ASCII directory. Escaped YAML paths can be altered by ROS parameter
    # parsing and make bt_navigator look for a non-existent directory.
    output.write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output

"""Build the independent Nav2 configuration used by the following UAV."""

from __future__ import annotations

from pathlib import Path
import yaml


DEFAULT = Path("/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml")
FOOTPRINT = "[[0.90, 0.90], [0.90, -0.90], [-0.90, -0.90], [-0.90, 0.90]]"


def build(output: Path, flight_altitude_m: float = 15.0) -> Path:
    data = yaml.safe_load(DEFAULT.read_text(encoding="utf-8"))

    def update(value):
        if isinstance(value, dict):
            for key, item in list(value.items()):
                if key == "use_sim_time":
                    value[key] = False
                elif key in ("robot_base_frame", "base_frame_id"):
                    value[key] = "uav_base_link"
                elif key == "global_frame":
                    value[key] = "uav_px4_odom"
                elif key == "odom_topic":
                    value[key] = "/uav/px4_odom"
                elif key == "robot_radius":
                    value.pop(key)
                    value["footprint"] = FOOTPRINT
                else:
                    update(item)
        elif isinstance(value, list):
            for item in value:
                update(item)

    update(data)

    # The UAV controller consumes paths from planner_server directly.  The
    # remaining Nav2 servers stay namespaced and cannot command the Husky.
    planner = data["planner_server"]["ros__parameters"]
    planner["expected_planner_frequency"] = 2.0
    planner["GridBased"]["tolerance"] = 3.0
    planner["GridBased"]["use_astar"] = True
    planner["GridBased"]["allow_unknown"] = True

    for name in ("local_costmap", "global_costmap"):
        costmap = data[name][name]["ros__parameters"]
        costmap["global_frame"] = "uav_px4_odom"
        costmap["robot_base_frame"] = "uav_base_link"
        costmap["rolling_window"] = True
        costmap["track_unknown_space"] = True
        costmap["resolution"] = 0.20
        costmap["footprint"] = FOOTPRINT
        costmap["plugins"] = ["obstacle_layer", "inflation_layer"]
        if name == "local_costmap":
            costmap["width"] = 30
            costmap["height"] = 30
        else:
            costmap["width"] = 280
            costmap["height"] = 280
        obstacle = costmap.setdefault("obstacle_layer", {})
        obstacle["plugin"] = "nav2_costmap_2d::ObstacleLayer"
        obstacle["observation_sources"] = "aerial_lidar"
        obstacle.pop("scan", None)
        obstacle["aerial_lidar"] = {
            # Use the live sensor for immediate collision avoidance. RTAB-Map
            # separately builds the accumulated three-dimensional map.
            "topic": "/x500/lidar3d/points",
            "data_type": "PointCloud2",
            # The point cloud is expressed in the aerial odometry frame. Only
            # geometry intersecting the fixed-height flight corridor is a
            # horizontal collision threat; lower objects can be overflown.
            # ObstacleLayer applies these limits in the incoming moving sensor
            # frame. Branches intersecting the UAV are near sensor Z=0, while
            # terrain at 15 m flight altitude is near Z=-15 and is ignored.
            "min_obstacle_height": -6.0,
            "max_obstacle_height": 6.0,
            "clearing": True,
            "marking": True,
            "raytrace_min_range": 0.8,
            "raytrace_max_range": 80.0,
            "obstacle_min_range": 0.8,
            "obstacle_max_range": 70.0,
        }
        inflation = costmap["inflation_layer"]
        # Tree canopies are sparse point returns. A wider buffer prevents the
        # rotor disc from clipping branches between individual LiDAR points.
        inflation["inflation_radius"] = 3.0
        inflation["cost_scaling_factor"] = 2.0

    lifecycle = data.setdefault("lifecycle_manager_navigation", {}).setdefault("ros__parameters", {})
    lifecycle["service_timeout"] = 20.0
    lifecycle["bond_timeout"] = 10.0

    output.parent.mkdir(parents=True, exist_ok=True)
    # ROS parameter files match fully-qualified node names.  Wrapping the
    # standard Nav2 structure makes these parameters apply to /uav_nav/* while
    # the Husky continues using the root namespace.
    output.write_text(yaml.safe_dump({"uav_nav": data}, sort_keys=False), encoding="utf-8")
    return output

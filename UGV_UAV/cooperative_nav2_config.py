"""Extend the proven UGV Nav2 configuration with aerial obstacle guidance."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from UGV_Standalone.baseline.config.nav2_config import build as build_ugv_config


COOPERATIVE_CONFIG = (
    Path(__file__).resolve().parent / "config/cooperative_navigation.yaml"
)


def _remove_fixed_backup_recovery(data: dict) -> None:
    """Remove distance-commanded backup from the combined-mode behavior tree.

    Reverse motion remains available to DWB as a locally sampled trajectory.
    The standalone UGV behavior tree is generated separately and is untouched.
    """
    navigator = data["bt_navigator"]["ros__parameters"]
    tree_path = Path(navigator["default_nav_to_pose_bt_xml"])
    tree = ET.parse(tree_path)
    root = tree.getroot()
    removed = 0
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "BackUp":
                parent.remove(child)
                removed += 1
    if removed:
        ET.indent(tree, space="  ")
        tree.write(tree_path, encoding="unicode", xml_declaration=False)


def build(
    output: Path,
    *,
    global_size_m: int,
    global_resolution_m: float,
    use_aerial_guidance: bool = True,
) -> Path:
    """Generate UGV Nav2 parameters without changing UGV_Standalone.

    The UGV retains its existing controller, footprint, local costmap and
    onboard three-dimensional LiDAR collision protection. When enabled, the
    UAV contributes only a second, long-range observation source to the global
    UGV costmap.
    """
    build_ugv_config(
        output,
        global_size_m=global_size_m,
        global_resolution_m=global_resolution_m,
    )
    if not use_aerial_guidance:
        return output

    data = yaml.safe_load(output.read_text(encoding="utf-8"))
    global_costmap = data["global_costmap"]["global_costmap"]["ros__parameters"]
    obstacle_layer = global_costmap["obstacle_layer"]
    obstacle_layer["observation_sources"] = "lidar3d aerial_lidar"
    obstacle_layer["aerial_lidar"] = {
        "topic": "/cooperative/aerial_obstacles",
        "data_type": "PointCloud2",
        "min_obstacle_height": 0.70,
        "max_obstacle_height": 30.0,
        "clearing": False,
        "marking": True,
        "obstacle_min_range": 1.0,
        "obstacle_max_range": 120.0,
    }
    cooperative = yaml.safe_load(
        COOPERATIVE_CONFIG.read_text(encoding="utf-8")
    )
    reverse = cooperative["reverse_motion"]
    # Permit DWB to sample reverse trajectories from the live local costmap.
    # This is a velocity safety envelope, not a fixed reverse command.
    controller = data["controller_server"]["ros__parameters"]["FollowPath"]
    maximum_reverse_speed = (
        float(reverse["maximum_speed_mps"]) if reverse["enabled"] else 0.0
    )
    controller["min_vel_x"] = -maximum_reverse_speed
    controller["vx_samples"] = max(
        int(reverse["velocity_samples"]), int(controller["vx_samples"])
    )
    velocity_smoother = data["velocity_smoother"]["ros__parameters"]
    velocity_smoother["min_velocity"][0] = -maximum_reverse_speed

    _remove_fixed_backup_recovery(data)
    data["planner_server"]["ros__parameters"]["GridBased"]["tolerance"] = 1.0
    output.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return output

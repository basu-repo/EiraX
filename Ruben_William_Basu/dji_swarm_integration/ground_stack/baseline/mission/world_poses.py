"""Read dynamic standalone UGV mission poses from the saved world."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def named_pose(world_file: Path, name: str) -> Pose2D:
    root = ET.parse(world_file).getroot()
    for include in root.findall(".//include"):
        if include.findtext("name") == name:
            values = [float(value) for value in include.findtext("pose", "0 0 0 0 0 0").split()]
            return Pose2D(values[0], values[1], values[5])
    raise ValueError(f"Entity {name!r} is missing from {world_file}")


def world_target_in_enu_odom(world_file: Path, target_name: str) -> Pose2D:
    """Express a saved target in the GNSS ENU odom frame at the start datum."""
    home = named_pose(world_file, "husky")
    target = home if target_name == "spawn" else named_pose(world_file, target_name)
    return Pose2D(
        target.x - home.x,
        target.y - home.y,
        target.yaw,
    )


def rolling_costmap_plan(
    world_file: Path,
    target_names: list[str],
    *,
    minimum_size_m: float = 60.0,
    margin_m: float = 15.0,
) -> tuple[int, float, float]:
    """Return costmap size, resolution and longest consecutive target leg."""
    poses = [named_pose(world_file, "husky")]
    poses.extend(
        poses[0] if name == "spawn" else named_pose(world_file, name)
        for name in target_names
    )
    legs = [
        math.hypot(end.x - start.x, end.y - start.y)
        for start, end in zip(poses, poses[1:])
    ]
    largest_axis_delta = max(
        max(abs(end.x - start.x), abs(end.y - start.y))
        for start, end in zip(poses, poses[1:])
    )
    required_size = 2.0 * (largest_axis_delta + margin_m)
    size_m = int(max(minimum_size_m, math.ceil(required_size / 10.0) * 10.0))
    resolution_m = 0.05 if size_m <= 100.0 else 0.10 if size_m <= 300.0 else 0.20
    return size_m, resolution_m, max(legs)

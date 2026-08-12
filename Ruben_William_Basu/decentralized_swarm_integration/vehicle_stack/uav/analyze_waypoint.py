"""Create a compact pose/localization plot for an aerial waypoint run."""

from __future__ import annotations

import csv
import math
from pathlib import Path


def create_plot(run_directory: Path) -> Path:
    import matplotlib.pyplot as plt

    route_file = run_directory / "uav_route_trajectory.csv"
    legacy_file = run_directory / "pose_comparison.csv"
    source = route_file if route_file.exists() else legacy_file
    rows = list(csv.DictReader(source.open(encoding="utf-8")))
    if not rows:
        raise ValueError(f"{source.name} contains no samples")

    elapsed = [float(row["elapsed_sec"]) for row in rows]
    px4_north = [float(row["px4_north_m"]) for row in rows]
    px4_east = [float(row["px4_east_m"]) for row in rows]
    gazebo_north = [float(row["gazebo_north_m"]) for row in rows]
    gazebo_east = [float(row["gazebo_east_m"]) for row in rows]
    error_field = (
        "localization_error_m" if "localization_error_m" in rows[0] else "position_error_m"
    )
    errors = [float(row[error_field]) for row in rows]
    px4_altitude = [-float(row["px4_down_m"]) for row in rows]
    gazebo_altitude = [-float(row["gazebo_down_m"]) for row in rows]
    valid_ground = [
        index
        for index, values in enumerate(zip(gazebo_east, gazebo_north, gazebo_altitude, errors))
        if all(math.isfinite(value) for value in values)
    ]

    figure, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    axes[0].plot(px4_east, px4_north, label="PX4 estimated pose", linewidth=1.5)
    axes[0].plot(
        [gazebo_east[index] for index in valid_ground],
        [gazebo_north[index] for index in valid_ground],
        "--",
        label="Gazebo ground truth",
        linewidth=1.2,
    )
    axes[0].scatter([0], [0], marker="s", color="green", label="Spawn")
    axes[0].scatter([px4_east[-1]], [px4_north[-1]], marker="x", color="red", label="Finish")
    axes[0].set_title("Horizontal trajectory")
    axes[0].set_xlabel("East (m)")
    axes[0].set_ylabel("North (m)")
    axes[0].axis("equal")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)

    axes[1].plot(elapsed, px4_altitude, label="PX4 estimate")
    axes[1].plot(
        [elapsed[index] for index in valid_ground],
        [gazebo_altitude[index] for index in valid_ground],
        "--",
        label="Gazebo ground truth",
    )
    axes[1].set_title("Altitude above spawn")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Altitude (m)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=8)

    axes[2].plot(
        [elapsed[index] for index in valid_ground],
        [errors[index] for index in valid_ground],
        color="tab:red",
    )
    axes[2].set_title("Three-dimensional position error")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Error (m)")
    axes[2].grid(True, alpha=0.3)

    figure.tight_layout()
    output = run_directory / "pose_and_localization.png"
    figure.savefig(output, dpi=160)
    plt.close(figure)
    return output

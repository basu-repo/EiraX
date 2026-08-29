"""Analyze a recorded standalone UGV localization run."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import xml.etree.ElementTree as ET


ESTIMATORS = ("wheel", "ekf", "local_ekf", "lidar_odom", "lidar_slam", "rtab")


def husky_spawn_yaw(world: Path) -> float:
    root = ET.parse(world).getroot()
    for include in root.findall(".//include"):
        if include.findtext("name") == "husky":
            return float(include.findtext("pose", "0 0 0 0 0 0").split()[5])
    raise ValueError(f"No husky pose in {world}")


def percentile(values: list[float], fraction: float) -> float:
    return values[min(len(values) - 1, round((len(values) - 1) * fraction))]


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze an EiraX localization dataset")
    parser.add_argument("dataset", type=Path)
    parser.add_argument(
        "--repair-legacy-ground-truth",
        action="store_true",
        help="Undo the heading rotation used by recorder versions before 2026-07-18",
    )
    args = parser.parse_args()

    source = args.dataset / "localization/trajectory_and_errors.csv"
    rows = list(csv.DictReader(source.open(encoding="utf-8")))
    if not rows:
        raise SystemExit(f"No samples in {source}")
    source_samples = len(rows)

    # The runner intentionally keeps Gazebo open after mission completion. The
    # evaluator can therefore contain a long stationary tail even though rosbag
    # recording has stopped. Exclude that idle tail from mission accuracy.
    last_motion = None
    previous = None
    for index, row in enumerate(rows):
        point = float(row["ground_truth_x"]), float(row["ground_truth_y"])
        if previous is not None and math.hypot(
            point[0] - previous[0], point[1] - previous[1]
        ) > 0.002:
            last_motion = index
        previous = point
    if last_motion is not None and len(rows) - last_motion > 300:
        rows = rows[: min(len(rows), last_motion + 31)]

    if args.repair_legacy_ground_truth:
        heading = husky_spawn_yaw(args.dataset / "world/baylands_editable.world")
        c, s = math.cos(heading), math.sin(heading)
        for row in rows:
            x, y = float(row["ground_truth_x"]), float(row["ground_truth_y"])
            row["ground_truth_x"] = c * x - s * y
            row["ground_truth_y"] = s * x + c * y
            for name in ("wheel", "local_ekf"):
                if row.get(f"{name}_x"):
                    x, y = float(row[f"{name}_x"]), float(row[f"{name}_y"])
                    row[f"{name}_x"] = c * x - s * y
                    row[f"{name}_y"] = s * x + c * y
            for name in ("ekf", "rtab"):
                if row.get(f"{name}_yaw"):
                    estimate_yaw = float(row[f"{name}_yaw"])
                    row[f"{name}_yaw"] = math.atan2(
                        math.sin(estimate_yaw - heading),
                        math.cos(estimate_yaw - heading),
                    )

    for row in rows:
        gx, gy, gyaw = (float(row[k]) for k in
                        ("ground_truth_x", "ground_truth_y", "ground_truth_yaw"))
        for name in ESTIMATORS:
            if not row.get(f"{name}_x"):
                continue
            x, y, eyaw = (float(row[f"{name}_{axis}"]) for axis in ("x", "y", "yaw"))
            row[f"{name}_position_error"] = math.hypot(x - gx, y - gy)
            row[f"{name}_yaw_error"] = math.atan2(
                math.sin(eyaw - gyaw), math.cos(eyaw - gyaw)
            )

    corrected = args.dataset / "localization/trajectory_and_errors_corrected.csv"
    with corrected.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    points = [(float(r["ground_truth_x"]), float(r["ground_truth_y"])) for r in rows]
    summary: dict[str, object] = {
        "source_samples": source_samples,
        "samples": len(rows),
        "ground_truth_path_length_m": sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])
        ),
        "estimators": {},
    }
    for name in ESTIMATORS:
        position = sorted(float(r[f"{name}_position_error"]) for r in rows
                          if r.get(f"{name}_position_error"))
        yaw = [abs(float(r[f"{name}_yaw_error"])) for r in rows
               if r.get(f"{name}_yaw_error")]
        if position:
            summary["estimators"][name] = {
                "samples": len(position),
                "position_rmse_m": math.sqrt(statistics.fmean(v * v for v in position)),
                "position_mean_m": statistics.fmean(position),
                "position_median_m": statistics.median(position),
                "position_p95_m": percentile(position, 0.95),
                "position_max_m": max(position),
                "yaw_mae_deg": math.degrees(statistics.fmean(yaw)),
            }

    summary_path = args.dataset / "localization/summary_corrected.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print(f"Wrote {corrected}\nWrote {summary_path}\nMatplotlib unavailable; plots skipped.")
        return

    figure, axes = plt.subplots(1, 2, figsize=(14, 6))
    axes[0].plot([p[0] for p in points], [p[1] for p in points], label="ground truth", linewidth=2)
    for name in ESTIMATORS:
        xy = [(float(r[f"{name}_x"]), float(r[f"{name}_y"])) for r in rows
              if r.get(f"{name}_x")]
        if xy:
            axes[0].plot([p[0] for p in xy], [p[1] for p in xy], label=name, alpha=0.8)
    axes[0].set(title="Trajectory comparison", xlabel="x (m)", ylabel="y (m)")
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].legend()

    time0 = float(rows[0]["stamp_sec"])
    for name in ESTIMATORS:
        samples = [(float(r["stamp_sec"]) - time0, float(r[f"{name}_position_error"]))
                   for r in rows if r.get(f"{name}_position_error")]
        if samples:
            axes[1].plot([p[0] for p in samples], [p[1] for p in samples], label=name)
    axes[1].set(title="Position error", xlabel="elapsed simulation time (s)", ylabel="error (m)")
    axes[1].grid(True)
    axes[1].legend()
    figure.tight_layout()
    plot = args.dataset / "localization/trajectory_and_errors_corrected.png"
    figure.savefig(plot, dpi=160)
    print(f"Wrote {corrected}\nWrote {summary_path}\nWrote {plot}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Export a Simu5G-compatible mobility trace from a recorded ROS 2 bag.

This is the ``sim/bridge/export_pose_trace.py`` step of the setup document:

    python3 sim/bridge/export_pose_trace.py logs/team0_benign trace/team0_pose.txt

It reads ``/mavros/local_position/pose`` from the bag and writes the measured
x/y/z trace relative to the first recorded sample, which is the form consumed
by ``experiments/trace/`` and by the Simu5G mobility input. Pass ``--movements``
to additionally emit the OMNeT++ ``uav.movements`` waypoint string used by the
scenario ``omnetpp.ini``.

Reading a bag needs the ROS 2 runtime, so run it under a sourced environment:

    source /opt/ros/jazzy/setup.bash

A previously extracted ``ros_pose_trace.csv`` can be converted without ROS by
passing it in place of the bag directory.
"""

import argparse
import math
import csv
from pathlib import Path

TRACE_HEADER = "time_sec x_east_m y_north_m z_up_m"

# Simu5G places the cell at the centre of a 1000 m playground, so the measured
# trace is offset to that origin and held above ground for the movement string.
PLAYGROUND_CENTRE_M = 500.0
MINIMUM_ALTITUDE_M = 0.05


def arguments():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", help="ROS 2 bag directory, or an extracted ros_pose_trace.csv")
    parser.add_argument("out", help="Simu5G mobility trace to write")
    parser.add_argument("--topic", default="/mavros/local_position/pose")
    parser.add_argument("--max-step", type=float, default=1.0,
                        help="Largest allowed distance between consecutive poses")
    parser.add_argument("--duration", type=float,
                        help="Keep only samples up to this elapsed time in seconds")
    parser.add_argument("--movements",
                        help="Also write the OMNeT++ uav.movements waypoint string here")
    return parser.parse_args()


def read_extracted_csv(path):
    """Read poses from a ros_pose_trace.csv produced by an earlier extraction."""
    with path.open(encoding="utf-8") as stream:
        return [
            (float(row["elapsed_sec"]), float(row["east_m"]),
             float(row["north_m"]), float(row["up_m"]))
            for row in csv.DictReader(stream)
        ]


def read_bag(path, topic):
    """Read poses directly from a ROS 2 bag. Requires the ROS 2 runtime."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:  # pragma: no cover - depends on the ROS runtime
        raise SystemExit(
            f"{error}. Source the ROS 2 environment first, for example "
            "'source /opt/ros/jazzy/setup.bash', or pass an extracted "
            "ros_pose_trace.csv instead of the bag directory."
        ) from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in types:
        raise SystemExit(f"{path}: the bag does not contain {topic}")
    pose_type = get_message(types[topic])

    samples = []
    while reader.has_next():
        name, data, stamp = reader.read_next()
        if name != topic:
            continue
        position = deserialize_message(data, pose_type).pose.position
        samples.append((stamp, position.x, position.y, position.z))
    if not samples:
        raise SystemExit(f"{path}: no {topic} messages were recorded")

    first_stamp, east, north, up = samples[0]
    return [((stamp - first_stamp) / 1e9, x - east, y - north, z - up)
            for stamp, x, y, z in samples]


def strictly_increasing(samples):
    """Simu5G rejects repeated or reordered timestamps in a mobility trace."""
    kept = []
    previous = -1.0
    for stamp, east, north, up in samples:
        if stamp <= previous:
            continue
        previous = stamp
        kept.append((stamp, east, north, up))
    return kept


def write_trace(path, samples):
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [TRACE_HEADER]
    lines += [f"{stamp} {east} {north} {up}" for stamp, east, north, up in samples]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_movements(path, samples, duration):
    path.parent.mkdir(parents=True, exist_ok=True)
    movement = []
    for stamp, east, north, up in samples:
        movement += [
            f"{stamp:.6f}",
            f"{PLAYGROUND_CENTRE_M + east:.6f}",
            f"{PLAYGROUND_CENTRE_M + north:.6f}",
            f"{max(MINIMUM_ALTITUDE_M, up):.6f}",
        ]
    # Hold the final waypoint to the requested simulation time limit.
    if duration is not None and samples and samples[-1][0] < duration:
        movement[4 * (len(movement) // 4 - 1)] = f"{duration:.6f}"
    path.write_text(" ".join(movement) + "\n", encoding="utf-8")


def main():
    args = arguments()
    source = Path(args.bag)
    if not source.exists():
        raise SystemExit(f"{source}: no such bag directory or pose CSV")

    samples = read_extracted_csv(source) if source.is_file() else read_bag(source, args.topic)
    if args.duration is not None:
        samples = [row for row in samples if row[0] <= args.duration]
    samples = strictly_increasing(samples)
    if not samples:
        raise SystemExit(f"{source}: the exported pose trace is empty")

    # A healthy flight moves a few centimetres between samples. A large step
    # means the position estimate broke and the aircraft chased it, which would
    # put the UE far off course when Simu5G replays the trace.
    jumps = [(a[0], math.dist(a[1:4], b[1:4])) for a, b in zip(samples, samples[1:])]
    worst = max((step for _, step in jumps), default=0.0)
    if worst > args.max_step:
        raise SystemExit(
            f"{source}: position estimate broke, largest step {worst:.2f} m "
            f"exceeds {args.max_step:.2f} m over {sum(1 for _, s in jumps if s > args.max_step)} "
            "samples; re-fly this run")

    output = Path(args.out)
    write_trace(output, samples)
    print(f"EXPORTED: {output} ({len(samples)} poses, "
          f"{samples[-1][0] - samples[0][0]:.3f} s, largest step {worst:.3f} m)")

    if args.movements:
        movements = Path(args.movements)
        write_movements(movements, samples, args.duration)
        print(f"EXPORTED: {movements} (OMNeT++ waypoint string)")


if __name__ == "__main__":
    main()

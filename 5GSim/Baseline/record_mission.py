#!/usr/bin/env python3
"""Record a benign mission: ROS 2 bag, topic measurements and a pose trace.

This is section 4 of the setup document. It starts Gazebo, PX4 and MAVROS,
records the three required MAVROS topics while the UAV flies the route, then
measures topic rates and sizes and exports the pose trace Simu5G needs.

Gazebo opens paused. The mission waits for you to press play, and only then
does the bag start recording, so the bag holds the flight and nothing else.

    ./record_mission.py                        # fly to the goal
    ./record_mission.py --endpoint waypoint_1  # a shorter route
"""

import argparse
import shutil
import subprocess
import sys
import time

from mission import fly_mission
from run_simulation import (HERE, WORLD_NAME, gazebo_already_running, read_route,
                            start_gazebo_and_px4, stop)

TOPICS = ["/mavros/local_position/pose",
          "/mavros/state",
          "/mavros/global_position/raw/fix"]
FCU_URL = "udp://:14550@127.0.0.1:18570"
ROS_SETUP = "source /opt/ros/jazzy/setup.bash && "


def arguments():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--headless", action="store_true",
                        help="No window; physics starts immediately and no play click is needed")
    parser.add_argument("--duration", type=float,
                        help="Patrol the route until this many seconds have passed")
    parser.add_argument("--endpoint", default="goal")
    parser.add_argument("--run-id", default="B1_BENIGN_001")
    parser.add_argument("--altitude", type=float, default=30.0)
    parser.add_argument("--speed", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=600.0)
    return parser.parse_args()


# Block until the Gazebo world is unpaused. WorldStatistics omits the field
# when it is false, so an absent "paused: true" means physics is running.
def wait_for_play(timeout=900.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        stats = subprocess.run(
            ["gz", "topic", "-e", "-t", f"/world/{WORLD_NAME}/stats", "-n", "1"],
            capture_output=True, text=True, timeout=20)
        if stats.returncode == 0 and "paused: true" not in stats.stdout:
            return True
        time.sleep(1)
    return False


# Run a command inside a sourced ROS 2 environment.
def ros_run(command, log, background=False):
    handle = log.open("w", encoding="utf-8")
    process = subprocess.Popen(["bash", "-lc", ROS_SETUP + command],
                               cwd=HERE, stdout=handle, stderr=subprocess.STDOUT,
                               stdin=subprocess.DEVNULL, start_new_session=True)
    if background:
        return process
    process.wait()
    return process


# Wait until MAVROS is publishing every topic the document requires.
def wait_for_topics(timeout=90.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        listing = subprocess.run(["bash", "-lc", ROS_SETUP + "ros2 topic list"],
                                 capture_output=True, text=True)
        published = set(listing.stdout.split())
        if all(topic in published for topic in TOPICS):
            return True
        time.sleep(2)
    return False


# Measure publication rate and bandwidth for each recorded topic.
def measure_topics(run_dir):
    rows = ["topic,frequency_hz,bandwidth"]
    for topic in TOPICS:
        hz = subprocess.run(["bash", "-lc", ROS_SETUP + f"timeout 8 ros2 topic hz {topic}"],capture_output=True, text=True)
        bw = subprocess.run(["bash", "-lc", ROS_SETUP + f"timeout 8 ros2 topic bw {topic}"],capture_output=True, text=True)
        rate = next((line.split()[-1] for line in hz.stdout.splitlines()
                     if "average rate" in line), "n/a")
        band = next((line.strip().split()[0] for line in bw.stdout.splitlines()
                     if "/s" in line), "n/a")
        rows.append(f"{topic},{rate},{band}")
        print(f"  {topic:36s} {rate:>10s} Hz   {band}", flush=True)
    (run_dir / "topic_measurements.csv").write_text("\n".join(rows) + "\n")


def main():
    args = arguments()
    if gazebo_already_running():
        print("[FAILED] A Gazebo server is already running. Close it first.")
        return 1

    run_dir = HERE / "logs" / args.run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    bag_dir = run_dir / "rosbag"

    gazebo = px4 = mavros = recorder = None
    try:
        gazebo, px4, _ = start_gazebo_and_px4(args.headless)
        route = read_route()
        print(f"[ROUTE] {', '.join(name for name, *_ in route)}", flush=True)

        if not args.headless:
            print("[WAITING] press play in the Gazebo window", flush=True)
            if not wait_for_play():
                print("[FAILED] The world was never unpaused.")
                return 1
            print("[PLAYING] physics is running", flush=True)

        mavros = ros_run(f"exec ros2 run mavros mavros_node --ros-args "
                         f"-p fcu_url:={FCU_URL}", run_dir / "mavros.log", background=True)
        if not wait_for_topics():
            print("[FAILED] MAVROS did not publish the required topics.")
            return 1
        print(f"[MAVROS] publishing {len(TOPICS)} topics", flush=True)

        # Measured before recording starts, so the bag holds only the flight.
        print("[TOPIC RATES]", flush=True)
        measure_topics(run_dir)

        # ros2 bag refuses to write into a directory that already exists, and
        # would otherwise leave the previous flight in place.
        shutil.rmtree(bag_dir, ignore_errors=True)
        recorder = ros_run(f"exec ros2 bag record -o {bag_dir} {' '.join(TOPICS)}",
                           run_dir / "rosbag.log", background=True)
        time.sleep(3)
        if not (bag_dir / "metadata.yaml").is_file() and not any(bag_dir.glob("*.mcap")):
            print(f"[FAILED] the recorder did not start; see {(run_dir / 'rosbag.log').relative_to(HERE)}")
            return 1
        print(f"[RECORDING] {bag_dir.relative_to(HERE)}", flush=True)

        result = fly_mission(port=14540, output_directory=run_dir, route=route,
                             endpoint=args.endpoint, altitude_m=args.altitude,
                             speed_mps=args.speed, duration_seconds=args.duration,
                             timeout_seconds=max(args.timeout, (args.duration or 0) + 240))

        stop(recorder)
        recorder = None
        print("[RECORDED] bag closed", flush=True)
        if result != 0:
            print(f"[MISSION FAILED] return code {result}")
            return result

        trace = HERE / "trace" / f"{args.run_id}_pose.txt"
        # rosbag2_py only exists in the ROS 2 Python, not the conda one.
        export = ros_run(
            f"/usr/bin/python3 {HERE}/sim/bridge/export_pose_trace.py "
            f"{bag_dir} {trace}", run_dir / "export.log")
        if export.returncode:
            print(f"[FAILED] pose export; see {(run_dir / 'export.log').relative_to(HERE)}")
            return 1
        print(f"[POSE TRACE] {trace.relative_to(HERE)} "
              f"({sum(1 for _ in trace.open()) - 1} samples)", flush=True)
        return 0
    except (FileNotFoundError, RuntimeError) as error:
        print(f"[FAILED] {error}")
        return 1
    finally:
        for process in (recorder, mavros, px4, gazebo):
            stop(process)
        print("[SHUTDOWN] Gazebo, PX4, MAVROS and the recorder stopped.", flush=True)


if __name__ == "__main__":
    sys.exit(main())

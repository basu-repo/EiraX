#!/usr/bin/env python3
"""Fly and record one run: ROS 2 bag, topic measurements and a pose trace.

The manifest sets the flight: speed, duration, drone count and whether the
drone flies the route, patrols it for the duration, or hovers. Gazebo opens
paused. The bag starts once physics is running and the topic rates have been
measured, so it holds the flight and nothing else.

A run with several drones records one real flight per drone, one after the
other. Simu5G then flies all of them together in the same cell.

    ./record_mission.py --run-id B_BASE_001
    ./record_mission.py --run-id B_BASE_001 --headless
"""

import argparse
import shutil
import subprocess
import sys
import time

import yaml

from mission import fly_mission
from run_simulation import (HERE, WORLD_NAME, gazebo_already_running, read_route,
                            start_gazebo_and_px4, stop)

TOPICS = ["/mavros/local_position/pose",
          "/mavros/state",
          "/mavros/global_position/raw/fix"]
FCU_URL = "udp://:14550@127.0.0.1:18570"
ROS_SETUP = "source /opt/ros/jazzy/setup.bash && "
ALTITUDE_M = 30.0


def arguments():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--headless", action="store_true",
                        help="No window; physics starts immediately and no play click is needed")
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
        hz = subprocess.run(["bash", "-lc", ROS_SETUP + f"timeout 8 ros2 topic hz {topic}"],
                            capture_output=True, text=True)
        bw = subprocess.run(["bash", "-lc", ROS_SETUP + f"timeout 8 ros2 topic bw {topic}"],
                            capture_output=True, text=True)
        rate = next((line.split()[-1] for line in hz.stdout.splitlines()
                     if "average rate" in line), "n/a")
        band = next((line.strip().split()[0] for line in bw.stdout.splitlines()
                     if "/s" in line), "n/a")
        rows.append(f"{topic},{rate},{band}")
        print(f"  {topic:36s} {rate:>10s} Hz   {band}", flush=True)
    (run_dir / "topic_measurements.csv").write_text("\n".join(rows) + "\n")


# One flight: start the simulator, record the bag, fly, export the trace.
# Drone 0 owns the plain file names; each further drone gets a _d<N> suffix.
def record(run_id, drone, route, speed, patrol, duration, headless):
    suffix = f"_d{drone}" if drone else ""
    run_dir = HERE / "logs" / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    bag_dir = run_dir / f"rosbag{suffix}"

    gazebo = px4 = mavros = recorder = None
    try:
        gazebo, px4, _ = start_gazebo_and_px4(headless)
        if not headless:
            print("[WAITING] press play in the Gazebo window", flush=True)
            if not wait_for_play():
                print("[FAILED] The world was never unpaused.")
                return 1
            print("[PLAYING] physics is running", flush=True)

        mavros = ros_run(f"exec ros2 run mavros mavros_node --ros-args "
                         f"-p fcu_url:={FCU_URL}", run_dir / f"mavros{suffix}.log", background=True)
        if not wait_for_topics():
            print("[FAILED] MAVROS did not publish the required topics.")
            return 1
        print(f"[MAVROS] publishing {len(TOPICS)} topics", flush=True)

        # Measured before recording starts, so the bag holds only the flight.
        if drone == 0:
            print("[TOPIC RATES]", flush=True)
            measure_topics(run_dir)

        # ros2 bag refuses to write into a directory that already exists, and
        # would otherwise leave the previous flight in place.
        shutil.rmtree(bag_dir, ignore_errors=True)
        recorder = ros_run(f"exec ros2 bag record -o {bag_dir} {' '.join(TOPICS)}",
                           run_dir / f"rosbag{suffix}.log", background=True)
        time.sleep(3)
        if not (bag_dir / "metadata.yaml").is_file() and not any(bag_dir.glob("*.mcap")):
            print(f"[FAILED] the recorder did not start; see {run_dir / f'rosbag{suffix}.log'}")
            return 1
        print(f"[RECORDING] {bag_dir.relative_to(HERE)}", flush=True)

        result = fly_mission(port=14540, output_directory=run_dir / (f"uav{drone}" if drone else "."),
                             route=route, altitude_m=ALTITUDE_M, speed_mps=speed,
                             duration_seconds=patrol, timeout_seconds=duration + 300)

        stop(recorder)
        recorder = None
        print("[RECORDED] bag closed", flush=True)
        if result != 0:
            print(f"[MISSION FAILED] return code {result}")
            return result

        trace = HERE / "trace" / f"{run_id}_pose{suffix}.txt"
        # rosbag2_py only exists in the ROS 2 Python, not the conda one. The
        # trace ends at the manifest duration: the flight is over by then, and
        # a landed drone's estimate can wander while the bag is still open.
        export = ros_run(f"/usr/bin/python3 {HERE}/sim/bridge/export_pose_trace.py {bag_dir} {trace} "
                         f"--duration {duration:g}", run_dir / f"export{suffix}.log")
        if export.returncode:
            print(f"[FAILED] pose export; see {run_dir / f'export{suffix}.log'}")
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


def main():
    args = arguments()
    if gazebo_already_running():
        print("[FAILED] A Gazebo server is already running. Close it first.")
        return 1

    manifest = yaml.safe_load((HERE / "manifests" / f"{args.run_id}.yaml").read_text())
    setup = manifest["simulation"]
    count = int(setup.get("drone_count", 1))
    speed = float(setup["mission_speed_mps"])
    duration = float(setup["duration_sec"])
    profile = setup.get("mission_profile", "route")
    # The route flight lands at the goal, which takes about 140 s at 3 m/s.
    # A patrol or hover flies for exactly the manifest duration.
    patrol = None if profile == "route" else duration
    route = [] if profile == "hover" else read_route()
    print(f"[FLIGHT] {profile}, {speed:g} m/s, {duration:g}s, {count} drone(s); "
          f"route {', '.join(name for name, *_ in route) or 'none'}", flush=True)

    for drone in range(count):
        if count > 1:
            print(f"\n[DRONE {drone}] flight {drone + 1} of {count}", flush=True)
        result = record(args.run_id, drone, route, speed, patrol, duration, args.headless)
        if result:
            return result
        time.sleep(3)
    return 0


if __name__ == "__main__":
    sys.exit(main())

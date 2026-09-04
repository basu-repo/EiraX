#!/usr/bin/env python3
"""Start Gazebo and PX4 SITL and fly the benign mission. Nothing is recorded.

This is the simulation only: no MAVROS, no ROS 2 bag, no Simu5G, no dataset.
Use it to check that the simulator itself works.

    ./run_simulation.py                    # Gazebo opens paused
    ./run_simulation.py --headless         # no window, physics running
    ./run_simulation.py --no-mission       # open the world, do not fly
    ./run_simulation.py --check 10         # start, wait 10 s, exit
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import xml.etree.ElementTree as ET

from mission import fly_mission

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
PX4_RUNTIME = PROJECT / "UGV_UAV/px4_runtime"
SHARED_MODELS = PROJECT / "simulation/models"

WORLD_NAME = "baylands_editable"
WORLD_FILE = HERE / "worlds/baylands_uav.world"
UAV_MODEL = "gz_x500_mapping"
SPAWN = {"x": 211.12966918945318, "y": -256.3474426269531, "z": 2.935702896118163}


# Read the mission markers straight from the world, so moving one in Gazebo and
# saving is enough to change the route. Returns NED offsets from spawn.
def read_route():
    world = ET.parse(WORLD_FILE).getroot().find("world")
    markers = {}
    for entity in world:
        name = entity.get("name") or entity.findtext("name")
        pose = (entity.findtext("pose") or "").split()
        if name in ("goal",) or (name or "").startswith("waypoint"):
            x, y, z = (float(value) for value in pose[:3])
            markers[name] = (y - SPAWN["y"], x - SPAWN["x"], SPAWN["z"] - z)
    order = sorted(n for n in markers if n.startswith("waypoint")) + ["goal"]
    return [(name, *markers[name]) for name in order if name in markers]


def arguments():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--headless", action="store_true",
                        help="No window; physics starts immediately")
    parser.add_argument("--no-mission", action="store_true",
                        help="Open the world and leave the UAV at spawn")
    parser.add_argument("--check", type=float, metavar="SECONDS",
                        help="Start, wait this long, then shut down")
    parser.add_argument("--altitude", type=float, default=30.0)
    parser.add_argument("--speed", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=420.0,
                        help="Give up on the mission after this many seconds")
    return parser.parse_args()


def gazebo_already_running():
    """A second Gazebo server would fight the first for the same world."""
    try:
        probe = subprocess.run(["gz", "service", "-l"], capture_output=True,
                               text=True, timeout=10)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return probe.returncode == 0 and "/server_control" in probe.stdout


def simulation_environment():
    env = os.environ.copy()
    env["QT_QPA_PLATFORM"] = env.get("QT_QPA_PLATFORM", "xcb")
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(filter(None, [
        str(SHARED_MODELS), str(PX4_RUNTIME / "models"),
        env.get("GZ_SIM_RESOURCE_PATH", "")]))
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(filter(None, [
        str(PX4_RUNTIME / "plugins"), env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]))
    return env


def stop(process):
    """Signal the whole process group; Gazebo and PX4 both spawn children."""
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=15)
        # A MAVROS node can outlive the ros2 launcher that started it.
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            # A PX4 daemon that survives leaves its instance claimed, and the
            # next run silently attaches to the stale one instead of starting.
            os.killpg(process.pid, signal.SIGKILL)


# Start Gazebo and PX4 SITL. Returns both processes; the caller stops them.
def start_gazebo_and_px4(headless=False):
    world = WORLD_FILE.resolve()
    if not world.is_file():
        raise FileNotFoundError(f"World is missing: {world}")
    px4_binary = PX4_RUNTIME / "bin/px4"
    if not px4_binary.is_file() or not (PX4_RUNTIME / "rootfs").is_dir():
        raise FileNotFoundError(f"PX4 runtime is missing: {PX4_RUNTIME}")

    env = simulation_environment()
    command = ["gz", "sim"] + (["-s", "-r"] if headless else []) + [str(world)]
    gazebo = subprocess.Popen(command, cwd=HERE, env=env,
                              stdin=subprocess.DEVNULL, start_new_session=True)
    print(f"[WORLD] {world.name}", flush=True)
    time.sleep(6)
    if gazebo.poll() is not None:
        raise RuntimeError("Gazebo exited during startup")

    px4_env = env.copy()
    px4_env.update({
        "PX4_SYS_AUTOSTART": "4001",
        "PX4_SIM_MODEL": UAV_MODEL,
        "PX4_GZ_STANDALONE": "1",
        "PX4_GZ_NO_FOLLOW": "1",
        "PX4_GZ_WORLD": WORLD_NAME,
        "PX4_GZ_MODEL_POSE": f"{SPAWN['x']},{SPAWN['y']},{SPAWN['z']}",
        "HEADLESS": "1" if headless else "0",
    })
    px4 = subprocess.Popen([str(px4_binary), "-d"], cwd=PX4_RUNTIME / "rootfs",
                           env=px4_env, stdin=subprocess.PIPE, start_new_session=True)
    print(f"[UAV] spawn x={SPAWN['x']:.1f} y={SPAWN['y']:.1f} z={SPAWN['z']:.1f}", flush=True)
    time.sleep(8)
    if px4.poll() is not None:
        raise RuntimeError("PX4 exited during startup")
    return gazebo, px4, env


def main():
    args = arguments()

    if gazebo_already_running():
        print("[FAILED] A Gazebo server is already running. Close it first.")
        return 1

    gazebo = px4 = None
    try:
        gazebo, px4, _ = start_gazebo_and_px4(args.headless)
        route = read_route()
        print(f"[ROUTE] {', '.join(name for name, *_ in route)}")
        print("[READY] Gazebo and PX4 are running. Nothing is being recorded.")

        if args.check is not None:
            time.sleep(args.check)
            print("[CHECK PASSED] The simulator starts and stays up.")
            return 0

        if args.no_mission:
            print("[NO MISSION] The UAV stays at spawn. Press Ctrl+C to finish.")
        else:
            result = fly_mission(
                port=14540,
                output_directory=HERE / "runs",
                route=route,
                altitude_m=args.altitude,
                speed_mps=args.speed,
                timeout_seconds=args.timeout,
            )
            if result != 0:
                print(f"[MISSION FAILED] return code {result}")
                return result
            print("[MISSION COMPLETE] Landed and disarmed.")
            print("[OPEN] Press Ctrl+C when you have finished viewing.")

        while gazebo.poll() is None and px4.poll() is None:
            time.sleep(1)
        return 0
    except KeyboardInterrupt:
        print("\n[STOPPING]")
        return 0
    finally:
        stop(px4)
        stop(gazebo)
        print("[SHUTDOWN] Gazebo and PX4 stopped.")


if __name__ == "__main__":
    sys.exit(main())

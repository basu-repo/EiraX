"""Launch the Baylands x500 spawn-to-waypoint localization test."""

from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from uav.x500_waypoint_mission import fly_waypoint
from uav.analyze_waypoint import create_plot


COOPERATIVE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = COOPERATIVE_ROOT.parent
PX4_RUNTIME = COOPERATIVE_ROOT / "px4_runtime"
WORLD = PROJECT_ROOT / "simulation/worlds/baylands_editable.world"


def world_pose(name: str) -> tuple[float, float, float]:
    for include in ET.parse(WORLD).getroot().findall(".//include"):
        if include.findtext("name") == name:
            values = [float(value) for value in include.findtext("pose", "0 0 0 0 0 0").split()]
            return values[0], values[1], values[2]
    raise ValueError(f"{name!r} is missing from {WORLD}")


def stop_group(process: subprocess.Popen) -> None:
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--altitude", type=float, default=30.0)
    parser.add_argument("--hover-seconds", type=float, default=5.0)
    args = parser.parse_args()

    spawn = world_pose("husky")
    waypoint = world_pose("waypoint_1")
    north = waypoint[1] - spawn[1]
    east = waypoint[0] - spawn[0]
    run_directory = PROJECT_ROOT / "datasets" / datetime.now().strftime("run_%Y%m%d_%H%M%S") / "uav"
    run_directory.mkdir(parents=True, exist_ok=True)
    gazebo_log = (run_directory / "gazebo.log").open("w", encoding="utf-8")
    px4_log = (run_directory / "px4.log").open("w", encoding="utf-8")

    environment = os.environ.copy()
    environment["GZ_SIM_RESOURCE_PATH"] = ":".join(
        [
            str(PROJECT_ROOT / "simulation/models"),
            str(PX4_RUNTIME / "models"),
            environment.get("GZ_SIM_RESOURCE_PATH", ""),
        ]
    ).rstrip(":")
    environment["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(
        [str(PX4_RUNTIME / "plugins"), environment.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]
    ).rstrip(":")
    gazebo_command = ["gz", "sim"]
    if args.headless:
        gazebo_command.extend(["-r", "-s"])
    gazebo_command.append(str(WORLD))
    gazebo = subprocess.Popen(
        gazebo_command,
        cwd=COOPERATIVE_ROOT,
        env=environment,
        stdin=subprocess.DEVNULL,
        stdout=gazebo_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    px4: subprocess.Popen | None = None
    try:
        if args.headless:
            print("[WAITING] Baylands is starting.")
        else:
            print("[PAUSED] Wait for Baylands and the x500 to load, then click Play in Gazebo.")
        time.sleep(6)
        if gazebo.poll() is not None:
            print(f"[FAILED] Gazebo startup failed. Check {run_directory / 'gazebo.log'}")
            return 1
        # The aerial-only test reuses the stable Husky spawn surface. This removes
        # the runtime entity only; it does not edit the saved world or Husky model.
        subprocess.run(
            [
                "gz", "service", "-s", "/world/baylands_editable/remove",
                "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
                "--timeout", "3000", "--req", 'name: "husky" type: MODEL',
            ],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        px4_environment = environment.copy()
        px4_environment.update(
            {
                "PX4_SYS_AUTOSTART": "4001",
                "PX4_SIM_MODEL": "gz_x500",
                "PX4_GZ_STANDALONE": "1",
                "PX4_GZ_NO_FOLLOW": "1",
                "PX4_GZ_WORLD": "baylands_editable",
                "PX4_GZ_MODEL_POSE": f"{spawn[0]},{spawn[1]},{spawn[2] + 0.35}",
                "HEADLESS": "1" if args.headless else "0",
            }
        )
        px4 = subprocess.Popen(
            [str(PX4_RUNTIME / "bin/px4"), "-d"],
            cwd=PX4_RUNTIME / "rootfs",
            env=px4_environment,
            # Keep PX4's interactive shell input open. /dev/null makes the shell
            # repeatedly redraw its prompt and can create a huge meaningless log.
            stdin=subprocess.PIPE,
            stdout=px4_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        print("[WAITING] PX4 x500 is starting at the saved spawn.")
        time.sleep(8)
        if px4.poll() is not None:
            print(f"[FAILED] PX4 startup failed. Check {run_directory / 'px4.log'}")
            return 1
        result = fly_waypoint(
            14540,
            run_directory,
            north,
            east,
            args.altitude,
            args.hover_seconds,
        )
        plot = create_plot(run_directory)
        print(f"[PLOT] {plot}")
        print(f"[SAVED] {run_directory}")
        if not args.headless:
            print("[GAZEBO OPEN] Test complete. Press Ctrl+C when finished viewing it.")
            while gazebo.poll() is None:
                time.sleep(1)
        return result
    except KeyboardInterrupt:
        return 130
    finally:
        if px4 is not None:
            stop_group(px4)
        stop_group(gazebo)
        gazebo_log.close()
        px4_log.close()

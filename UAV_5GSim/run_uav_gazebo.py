#!/usr/bin/env python3
"""Open isolated Baylands with the goal and one PX4 x500 UAV."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from mission import fly_to_goal


HERE = Path(__file__).resolve().parent
PROJECT_ROOT = HERE.parent
PX4_RUNTIME = PROJECT_ROOT / "UGV_UAV/px4_runtime"
SHARED_MODELS = PROJECT_ROOT / "simulation/models"


def gazebo_running() -> bool:
    try:
        result = subprocess.run(
            ["gz", "service", "-l"],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0 and "/server_control" in result.stdout.splitlines()


def stop_group(process: subprocess.Popen[bytes] | None) -> None:
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run the Gazebo server without the GUI and start physics immediately.",
    )
    parser.add_argument(
        "--smoke-seconds",
        type=float,
        default=None,
        help="Testing only: close automatically after this many seconds.",
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=30.0,
        help="Cruise altitude above the spawn in metres (default: 30).",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=5.0,
        help="Horizontal PX4 speed limit in m/s (default: 5).",
    )
    parser.add_argument(
        "--mission-timeout",
        type=float,
        default=300.0,
        help="Maximum takeoff-and-travel time before a safety landing.",
    )
    parser.add_argument(
        "--no-mission",
        action="store_true",
        help="Open the world and spawn the UAV without arming or flying.",
    )
    args = parser.parse_args()

    if gazebo_running():
        print("[FAILED] Another Gazebo server is already running. Close it first.")
        return 1

    config = json.loads((HERE / "config.json").read_text(encoding="utf-8"))
    world = (HERE / config["world_file"]).resolve()
    if not world.is_file():
        print(f"[FAILED] Isolated world is missing: {world}")
        print("Run: ./prepare_world.py")
        return 1

    px4_binary = PX4_RUNTIME / "bin/px4"
    px4_rootfs = PX4_RUNTIME / "rootfs"
    if not px4_binary.is_file() or not px4_rootfs.is_dir():
        print(f"[FAILED] PX4 runtime is missing: {PX4_RUNTIME}")
        return 1

    run_dir = HERE / "logs" / datetime.now().strftime("run_%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=False)
    gazebo_log = (run_dir / "gazebo.log").open("w", encoding="utf-8")
    px4_log = (run_dir / "px4.log").open("w", encoding="utf-8")

    env = os.environ.copy()
    env["QT_QPA_PLATFORM"] = env.get("QT_QPA_PLATFORM", "xcb")
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(
        value
        for value in (
            str(SHARED_MODELS),
            str(PX4_RUNTIME / "models"),
            env.get("GZ_SIM_RESOURCE_PATH", ""),
        )
        if value
    )
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(
        value
        for value in (
            str(PX4_RUNTIME / "plugins"),
            env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
        )
        if value
    )

    gazebo_command = ["gz", "sim"]
    if args.headless:
        gazebo_command.extend(["-s", "-r"])
    gazebo_command.append(str(world))

    gazebo: subprocess.Popen[bytes] | None = None
    px4: subprocess.Popen[bytes] | None = None
    try:
        gazebo = subprocess.Popen(
            gazebo_command,
            cwd=HERE,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=gazebo_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        print(f"[WORLD] {world}")
        if args.headless:
            print("[WAITING] Headless Baylands is starting.")
        else:
            print("[PAUSED] Let Baylands and the UAV load, then click Play in Gazebo.")
        time.sleep(6)
        if gazebo.poll() is not None:
            print(f"[FAILED] Gazebo exited. Check {run_dir / 'gazebo.log'}")
            return 1

        spawn = config["uav_spawn"]
        px4_env = env.copy()
        px4_env.update(
            {
                "PX4_SYS_AUTOSTART": "4001",
                "PX4_SIM_MODEL": str(config["uav_model"]),
                "PX4_GZ_STANDALONE": "1",
                "PX4_GZ_NO_FOLLOW": "1",
                "PX4_GZ_WORLD": str(config["world_name"]),
                "PX4_GZ_MODEL_POSE": (
                    f"{spawn['x']},{spawn['y']},{spawn['z']}"
                ),
                "HEADLESS": "1" if args.headless else "0",
            }
        )
        px4 = subprocess.Popen(
            [str(px4_binary), "-d"],
            cwd=px4_rootfs,
            env=px4_env,
            stdin=subprocess.PIPE,
            stdout=px4_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        print(
            "[UAV] Spawning at "
            f"x={spawn['x']:.3f}, y={spawn['y']:.3f}, z={spawn['z']:.3f}"
        )
        goal = config["goal"]
        print(
            "[GOAL] Preserved at "
            f"x={goal['x']:.3f}, y={goal['y']:.3f}, z={goal['z']:.3f}"
        )
        time.sleep(8)
        if px4.poll() is not None:
            print(f"[FAILED] PX4 exited. Check {run_dir / 'px4.log'}")
            return 1

        print(f"[READY] Gazebo and PX4 are running. Logs: {run_dir}")
        if args.smoke_seconds is not None:
            if args.smoke_seconds < 0:
                parser.error("--smoke-seconds cannot be negative")
            time.sleep(args.smoke_seconds)
            print("[SMOKE TEST] Completed successfully.")
            return 0

        if not args.no_mission:
            goal_north = float(goal["y"]) - float(spawn["y"])
            goal_east = float(goal["x"]) - float(spawn["x"])
            goal_ground_down = float(spawn["z"]) - float(goal["z"])
            mission_result = fly_to_goal(
                port=14540,
                output_directory=run_dir,
                goal_north_m=goal_north,
                goal_east_m=goal_east,
                goal_ground_down_m=goal_ground_down,
                altitude_m=args.altitude,
                speed_mps=args.speed,
                timeout_seconds=args.mission_timeout,
            )
            if mission_result != 0:
                print(f"[MISSION FAILED] Return code {mission_result}")
            elif args.headless:
                return 0
            else:
                print(
                    "[GAZEBO OPEN] Mission finished. "
                    "Press Ctrl+C when you are finished viewing it."
                )
        else:
            print("[NO MISSION] UAV will remain at spawn.")

        while gazebo.poll() is None and px4.poll() is None:
            time.sleep(1)
        print("[STOPPED] Gazebo or PX4 exited.")
        return 1
    except KeyboardInterrupt:
        print("\n[STOPPING] Closing the isolated UAV simulation.")
        return 0
    except (RuntimeError, TimeoutError) as error:
        print(f"[FAILED] {error}")
        return 1
    finally:
        stop_group(px4)
        stop_group(gazebo)
        gazebo_log.close()
        px4_log.close()


if __name__ == "__main__":
    raise SystemExit(main())

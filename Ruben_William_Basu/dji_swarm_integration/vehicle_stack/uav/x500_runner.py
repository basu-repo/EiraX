"""Launch PX4/Gazebo and run the verified x500 circuit mission."""

from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import time

from uav.x500_mission import fly


COOPERATIVE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = COOPERATIVE_ROOT.parent
PX4_RUNTIME = PROJECT_ROOT / "px4_runtime"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run without the Gazebo window")
    parser.add_argument("--radius", type=float, default=8.0, help="Circuit half-width in metres")
    parser.add_argument("--altitude", type=float, default=5.0, help="Flight altitude in metres")
    parser.add_argument("--timeout", type=float, default=180.0, help="Maximum circuit time")
    args = parser.parse_args()

    if not (PX4_RUNTIME / "bin/px4").exists():
        print(f"[FAILED] PX4 runtime is missing from {PX4_RUNTIME}")
        return 1

    run_id = datetime.now().strftime("run_%Y%m%d_%H%M%S")
    output = PROJECT_ROOT / "datasets" / run_id / "uav"
    output.mkdir(parents=True, exist_ok=True)
    log_stream = (output / "px4_gazebo.log").open("w", encoding="utf-8")
    environment = os.environ.copy()
    environment["GZ_SIM_RESOURCE_PATH"] = ":".join(
        [str(PX4_RUNTIME / "models"), environment.get("GZ_SIM_RESOURCE_PATH", "")]
    ).rstrip(":")
    environment["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(
        [str(PX4_RUNTIME / "plugins"), environment.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]
    ).rstrip(":")
    environment["GZ_PARTITION"] = f"eirax_{run_id}"
    gazebo_command = ["gz", "sim"]
    if args.headless:
        gazebo_command.extend(["-r", "-s"])
    gazebo_command.append(str(PX4_RUNTIME / "worlds/default.sdf"))
    gazebo = subprocess.Popen(
        gazebo_command,
        cwd=COOPERATIVE_ROOT,
        env=environment,
        stdin=subprocess.DEVNULL,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    px4: subprocess.Popen | None = None
    try:
        if args.headless:
            print("[WAITING] PX4 x500 and Gazebo are starting.")
        else:
            print("[PAUSED] Wait for the x500 to load, then click Play in Gazebo.")
        time.sleep(5)
        if gazebo.poll() is not None:
            print(f"[FAILED] Gazebo stopped during startup. Check {output / 'px4_gazebo.log'}")
            return 1
        px4_environment = environment.copy()
        px4_environment.update(
            {
                "PX4_SYS_AUTOSTART": "4001",
                "PX4_SIM_MODEL": "gz_x500",
                "PX4_GZ_STANDALONE": "1",
                "PX4_GZ_NO_FOLLOW": "1",
                "PX4_GZ_WORLD": "default",
                "PX4_GZ_MODEL_POSE": "0,0,0.25",
            }
        )
        px4 = subprocess.Popen(
            [str(PX4_RUNTIME / "bin/px4"), "-d"],
            cwd=PX4_RUNTIME / "rootfs",
            env=px4_environment,
            stdin=subprocess.PIPE,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        time.sleep(8)
        if px4.poll() is not None:
            print(f"[FAILED] PX4 stopped during startup. Check {output / 'px4_gazebo.log'}")
            return 1
        result = fly(
            14540,
            output / "flight_path.csv",
            args.radius,
            args.altitude,
            args.timeout,
        )
        print(f"[SAVED] {output}")
        if args.headless:
            return result
        print("[GAZEBO OPEN] Flight is complete. Press Ctrl+C when you are finished viewing it.")
        while gazebo.poll() is None:
            time.sleep(1)
        return result
    except KeyboardInterrupt:
        print("\n[STOPPING] Landing/test process interrupted.")
        return 130
    finally:
        for process in (px4, gazebo):
            if process is None or process.poll() is not None:
                continue
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
        log_stream.close()

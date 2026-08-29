#!/usr/bin/env python3
"""One-command launcher for the isolated Baylands UGV + three-UAV stack."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time


ROOT = Path(__file__).resolve().parents[1]
VEHICLE_LAUNCHER = ROOT / "scripts/run_baylands_swarm.py"
ROS_SETUP = ROOT / "install_combined/setup.bash"
OMNET_SETUP = Path("/home/basudeo/omnetpp-6.0.1/setenv")
OMNET_EXECUTABLE = ROOT / "omnet/out/gcc-release/omnet"

TERMINAL_EVENT_PREFIXES = (
    "[UAV SPAWN VERIFIED]", "[UAV ID MARKER]", "[UAV ID MARKER WARNING]",
    "[UAV READY]", "[LEADER]", "[LEADER CHANGE]", "[DISCONNECTED]",
    "[RECOVERY]", "[RECOVERY PENDING]", "[RETURN TO BASE]",
    "[NAV2 FALLBACK]", "[SURVEY]", "[UGV LEG", "[LEG COMPLETE]",
    "[NAV2 RECOVERY]",
    "[REVERSE RECOVERY]",
    "[UAV LANDED VERIFIED]", "[MISSION FINISHED]", "[FAILED]",
    "[SWARM STATUS]",
)


def relay_vehicle_events(log_path: Path, process: subprocess.Popen, stop_requested) -> None:
    """Echo important child events while retaining the complete file log."""
    position = 0
    while not stop_requested() and (process.poll() is None or log_path.exists()):
        if log_path.exists():
            # Binary readline keeps byte offsets reliable while another
            # process appends. Text iteration followed by tell() can raise
            # "telling position disabled by next() call" and silently kill
            # this daemon thread.
            with log_path.open("rb") as stream:
                stream.seek(position)
                while line := stream.readline():
                    text = line.decode("utf-8", errors="replace").rstrip()
                    if text.startswith(TERMINAL_EVENT_PREFIXES):
                        print(text, flush=True)
                position = stream.tell()
        if process.poll() is not None:
            break
        time.sleep(0.25)


def print_swarm_status(active_run: Path) -> None:
    """Print the current elected scout from the authoritative role file."""
    state_file = active_run / "cooperative/swarm_role_state.json"
    try:
        state = json.loads(state_file.read_text(encoding="utf-8"))
        scout = state.get("active_scout")
        states = state.get("states", {})
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        print("[SWARM STATUS] Leader/scout state is not available yet.", flush=True)
        return
    leader = scout.upper() if scout else "UGV NAV2"
    followers = [name.upper() for name, role in states.items() if role == "follower"]
    suffix = f" | Followers: {', '.join(followers)}" if followers else ""
    print(f"[SWARM STATUS] Current leader/scout: {leader}{suffix}", flush=True)


def sourced_environment(scripts: list[Path]) -> dict[str, str]:
    missing = [str(path) for path in scripts if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing environment setup: " + ", ".join(missing))
    source_commands = "\n".join(f'source "{path}" >/dev/null' for path in scripts)
    result = subprocess.run(
        ["bash", "--noprofile", "--norc", "-c", source_commands + "\nenv -0"],
        check=True,
        stdout=subprocess.PIPE,
    )
    env = os.environ.copy()
    for entry in result.stdout.split(b"\0"):
        if b"=" in entry:
            key, value = entry.split(b"=", 1)
            env[key.decode()] = value.decode(errors="surrogateescape")
    return env


def wait_for_port(port: int, process: subprocess.Popen, timeout: float = 60.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"ROS overlay stopped with exit code {process.returncode}")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.25):
                return
        except OSError:
            time.sleep(0.25)
    raise TimeoutError(f"ROS pose bridge did not open TCP {port} within {timeout:.0f}s")


def wait_for_airborne_swarm(
    vehicle: subprocess.Popen,
    previous_runs: set[Path],
    stop_requested,
    timeout: float = 600.0,
) -> Path | None:
    """Wait until the new vehicle run reports all three formation slots ready."""
    datasets = ROOT / "datasets"
    deadline = time.monotonic() + timeout
    active_run: Path | None = None
    ready_names = ("uav_ready", "uav1_ready", "uav2_ready")
    while time.monotonic() < deadline:
        if stop_requested():
            return None
        code = vehicle.poll()
        if code is not None:
            raise RuntimeError(f"vehicles exited with code {code} before swarm takeoff")
        if active_run is None:
            candidates = [path for path in datasets.glob("run_*") if path not in previous_runs]
            if candidates:
                active_run = max(candidates, key=lambda path: path.stat().st_mtime)
        if active_run is not None and all((active_run / name).exists() for name in ready_names):
            return active_run
        time.sleep(0.5)
    missing = list(ready_names)
    if active_run is not None:
        missing = [name for name in ready_names if not (active_run / name).exists()]
    raise TimeoutError(f"swarm takeoff did not become ready; missing={missing}")


def stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Launch Baylands, Husky, three PX4 x500 UAVs, YOLO, ROS swarm peers and OMNeT++"
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-motion", action="store_true")
    parser.add_argument("--no-yolo", action="store_true")
    parser.add_argument("--no-omnet", action="store_true")
    parser.add_argument("--no-recording", action="store_true")
    failure = parser.add_mutually_exclusive_group()
    failure.add_argument("--permanent-failure", action="store_true", help="Permanently fail UAV0, UAV1 and UAV2 in sequence; each returns to its bay")
    failure.add_argument("--connection-failure-reconnect", action="store_true", help="Temporarily disconnect UAV0; UAV1 remains scout after UAV0 rejoins")
    args = parser.parse_args()

    required = [VEHICLE_LAUNCHER, ROS_SETUP]
    if not args.no_omnet:
        required.extend([OMNET_SETUP, OMNET_EXECUTABLE])
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        print("[FAILED] Required built files are missing:\n  " + "\n  ".join(missing), file=sys.stderr)
        return 2

    logs = ROOT / "runtime_logs"
    logs.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    handles = []
    processes: list[tuple[str, subprocess.Popen]] = []
    optional_process_names = {"swarm camera viewer"}
    stopping = False

    def request_stop(_signum=None, _frame=None):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        ros_env = sourced_environment([Path("/opt/ros/jazzy/setup.bash"), ROS_SETUP])
        isolated_python = str(ROOT / "third_party/python")
        ros_env["PYTHONPATH"] = isolated_python + os.pathsep + ros_env.get("PYTHONPATH", "")
        ros_log_dir = logs / f"{stamp}_ros"
        ros_log_dir.mkdir(exist_ok=True)
        ros_env["ROS_LOG_DIR"] = str(ros_log_dir)

        # Run the complete cooperative route. The vehicle runner coordinates
        # UAV survey readiness before each UGV leg and returns all aircraft to
        # their individual launch bays after the final goal.
        vehicle_args = [
            sys.executable, "-u", str(VEHICLE_LAUNCHER), "--uav-count", "3",
        ]
        for enabled, option in (
            (args.headless, "--headless"),
            (args.no_motion, "--no-motion"),
            (args.no_recording, "--no-recording"),
            (args.permanent_failure, "--permanent-failure"),
            (args.connection_failure_reconnect, "--connection-failure-reconnect"),
        ):
            if enabled:
                vehicle_args.append(option)
        vehicle_log = (logs / f"{stamp}_vehicles.log").open("w", encoding="utf-8")
        handles.append(vehicle_log)
        previous_runs = set((ROOT / "datasets").glob("run_*"))
        vehicle = subprocess.Popen(
            vehicle_args, cwd=ROOT, env=ros_env, stdout=vehicle_log,
            stderr=subprocess.STDOUT, start_new_session=True, text=True,
        )
        processes.append(("vehicles", vehicle))
        threading.Thread(
            target=relay_vehicle_events,
            args=(Path(vehicle_log.name), vehicle, lambda: stopping),
            daemon=True,
        ).start()
        print("[STARTED] Baylands + Husky + PX4 x500_mapping_0/1/2", flush=True)
        if args.no_motion:
            # Diagnostic mode never creates airborne-ready markers.
            time.sleep(5)
        else:
            print(
                "[WAITING] Click Play in Gazebo. ROS/YOLO/OMNeT++ will start "
                "after all three UAVs are safely airborne.",
                flush=True,
            )
            active_run = wait_for_airborne_swarm(
                vehicle, previous_runs, stop_requested=lambda: stopping
            )
            if active_run is None:
                return 0
            print(f"[READY] Three-UAV takeoff verified: {active_run}", flush=True)
            print_swarm_status(active_run)

        ros_command = [
            "ros2", "launch", "decentralized_swarm_integration", "full_swarm.launch.py",
            "uav_names:=uav0,uav1,uav2", "start_px4:=false",
            # This overlay is advisory and does not own flight. Wall-clock
            # timers prevent every Python peer from consuming Gazebo's 1 kHz
            # /clock stream while sensor messages retain their source stamps.
            "use_sim_time:=false",
            f"start_yolo:={'false' if args.no_yolo else 'true'}",
            f"yolo_evidence_root:={active_run / 'yolo_frames' if active_run else ''}",
            "control_enabled:=false",
        ]
        ros_log = (logs / f"{stamp}_ros_overlay.log").open("w", encoding="utf-8")
        handles.append(ros_log)
        ros = subprocess.Popen(
            ros_command, cwd=ROOT, env=ros_env, stdout=ros_log,
            stderr=subprocess.STDOUT, start_new_session=True, text=True,
        )
        processes.append(("ROS overlay", ros))
        wait_for_port(5555, ros)
        print(
            "[STARTED] ROS decentralized peers"
            + ("" if args.no_yolo else " + three independent UAV YOLO observers")
        )

        # Open one lightweight selector instead of three costly GUI windows.
        # All three independent bridges remain available in its topic menu.
        if not args.headless:
            camera_viewer = subprocess.Popen(
                [
                    "ros2", "run", "rqt_image_view", "rqt_image_view",
                ],
                cwd=ROOT,
                env=ros_env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
                text=True,
            )
            processes.append(("swarm camera viewer", camera_viewer))
            print("[CAMERA] Select a live image topic in rqt_image_view:")
            for index in range(3):
                print(f"[CAMERA]   /swarm/uav{index}/camera/image_raw")

        if not args.no_omnet:
            omnet_env = sourced_environment([OMNET_SETUP])
            omnet_log = (logs / f"{stamp}_omnet.log").open("w", encoding="utf-8")
            handles.append(omnet_log)
            omnet_command = [
                str(OMNET_EXECUTABLE), "-u", "Cmdenv",
                "-n", f".:uav_ugv:/home/basudeo/inet/src",
                "-l", "/home/basudeo/inet/src/INET", "omnetpp.ini",
            ]
            omnet = subprocess.Popen(
                omnet_command, cwd=ROOT / "omnet", env=omnet_env,
                stdout=omnet_log, stderr=subprocess.STDOUT,
                start_new_session=True, text=True,
            )
            processes.append(("OMNeT++", omnet))
            print("[STARTED] OMNeT++/INET Wi-Fi overlay for all three UAV links")

        print(f"[RUNNING] One launcher owns the complete stack. Logs: {logs}")
        print("[RUNNING] Press Ctrl+C once to stop everything cleanly.")
        while not stopping:
            for name, process in processes:
                code = process.poll()
                if code is not None:
                    # A user may close the image window at any time. It is an
                    # observer only and must never own or terminate the mission.
                    if name in optional_process_names:
                        continue
                    if code == 0 and name == "vehicles":
                        print(
                            "[COMPLETED] All waypoints, the goal, and all UAV "
                            "landings succeeded."
                        )
                        return 0
                    print(f"[FAILED] {name} exited with code {code}", file=sys.stderr)
                    return code or 1
            time.sleep(0.5)
        return 0
    except (FileNotFoundError, RuntimeError, TimeoutError, subprocess.CalledProcessError) as exc:
        print(f"[FAILED] {exc}", file=sys.stderr)
        return 1
    finally:
        print("[STOPPING] Closing the complete isolated stack...")
        for _name, process in reversed(processes):
            stop_process(process)
        for handle in handles:
            handle.close()
        print("[STOPPED] Gazebo, PX4, ROS and OMNeT++ are closed.")


if __name__ == "__main__":
    raise SystemExit(main())

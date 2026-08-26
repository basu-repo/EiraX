#!/usr/bin/env python3
"""One-command launcher for isolated Baylands + Husky + DJI M100 swarm."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import threading
import time


ROOT = Path(__file__).resolve().parents[1]
ROS_SETUPS = (
    Path("/opt/ros/jazzy/setup.bash"),
    ROOT / "install_dji_lrs/setup.bash",
    ROOT / "install_dji/setup.bash",
)

TERMINAL_EVENTS = (
    "[SWARM STATUS]", "[DISCONNECTED]", "[LEADER CHANGE]", "[RECOVERY]",
    "[RETURN TO BASE]", "[NAV2 FALLBACK]", "[LEG COMPLETE]",
    "[UAV LANDED VERIFIED]", "[MISSION FINISHED]",
)


def relay_events(path: Path, process: subprocess.Popen, stop_requested) -> None:
    position = 0
    while not stop_requested() and (process.poll() is None or path.exists()):
        if path.exists():
            with path.open("rb") as stream:
                stream.seek(position)
                while line := stream.readline():
                    value = line.decode("utf-8", errors="replace").rstrip()
                    for prefix in TERMINAL_EVENTS:
                        marker = value.find(prefix)
                        if marker >= 0:
                            print(value[marker:], flush=True)
                            break
                position = stream.tell()
        if process.poll() is not None:
            return
        time.sleep(0.25)


def sourced_environment(paths: tuple[Path, ...]) -> dict[str, str]:
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing built ROS setup: " + ", ".join(missing))
    command = "\n".join(f'source "{path}" >/dev/null' for path in paths) + "\nenv -0"
    result = subprocess.run(
        ["bash", "--noprofile", "--norc", "-c", command],
        check=True, stdout=subprocess.PIPE,
    )
    env = os.environ.copy()
    for entry in result.stdout.split(b"\0"):
        if b"=" in entry:
            key, value = entry.split(b"=", 1)
            env[key.decode()] = value.decode(errors="surrogateescape")
    return env


def stop(process: subprocess.Popen) -> None:
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


def wait_for_world(process: subprocess.Popen, env: dict[str, str], timeout_s=90.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"UGV/Gazebo exited with code {process.returncode}")
        result = subprocess.run(
            ["gz", "service", "-l"], env=env, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True,
        )
        if "/world/baylands_editable/set_pose" in result.stdout:
            return
        time.sleep(1.0)
    raise TimeoutError("Baylands Gazebo services were not ready within 90 seconds")


def wait_for_port(port: int, process: subprocess.Popen, timeout_s=60.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"network bridge exited with code {process.returncode}")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.25):
                return
        except OSError:
            time.sleep(0.25)
    raise TimeoutError(f"network bridge port {port} was not ready")


def wait_for_dji_models(
    process: subprocess.Popen, env: dict[str, str], timeout_s: float = 30.0
) -> None:
    """Verify that create-service success produced entities in this world."""
    required = {"dji0", "dji1", "dji2"}
    deadline = time.monotonic() + timeout_s
    last_output = ""
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"DJI vehicle launch exited with code {process.returncode}")
        try:
            result = subprocess.run(
                [
                    "gz", "service", "-s", "/world/baylands_editable/scene/info",
                    "--reqtype", "gz.msgs.Empty", "--reptype", "gz.msgs.Scene",
                    "--timeout", "2000", "--req", "",
                ],
                env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=4,
            )
            last_output = result.stdout
        except subprocess.TimeoutExpired:
            # Gazebo discovery can briefly stall while the three sensor models
            # are inserted. Keep polling within the overall bounded deadline.
            time.sleep(0.5)
            continue
        present = set(re.findall(r"\bdji[012]\b", last_output))
        if required <= present:
            print("[DJI SPAWN VERIFIED] Gazebo entities: dji0, dji1, dji2", flush=True)
            return
        time.sleep(0.5)
    found = ", ".join(sorted(required & set(re.findall(r"\bdji[012]\b", last_output))))
    raise TimeoutError(
        "Gazebo did not retain all DJI entities within 30 seconds"
        + (f"; found: {found}" if found else "; found none")
    )


def require_working_gpu() -> None:
    """Fail before Gazebo when the NVIDIA device is unusable.

    Ogre2 otherwise starts with a null EGL driver and aborts later while the
    three camera textures are being created, which looks like an unexplained
    automatic launcher shutdown.
    """
    if not Path("/dev/nvidiactl").exists():
        raise RuntimeError(
            "NVIDIA device nodes are missing (/dev/nvidiactl). Gazebo's Ogre2 "
            "renderer would crash. Run `nvidia-smi`; repair the NVIDIA driver "
            "before starting the DJI simulation."
        )
    probe = subprocess.run(
        ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if probe.returncode != 0 or not probe.stdout.strip():
        detail = probe.stderr.strip() or "nvidia-smi returned no GPU"
        raise RuntimeError(
            f"NVIDIA GPU preflight failed: {detail}. Gazebo was not started."
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Launch Baylands, Husky Nav2, DJI M100 x3, YOLO and OMNeT++"
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-motion", action="store_true")
    parser.add_argument("--no-yolo", action="store_true")
    parser.add_argument("--no-omnet", action="store_true")
    failure = parser.add_mutually_exclusive_group()
    failure.add_argument("--permanent-failure", action="store_true")
    failure.add_argument("--connection-failure-reconnect", action="store_true")
    args = parser.parse_args()

    scenario = (
        "permanent_failure" if args.permanent_failure else
        "reconnect" if args.connection_failure_reconnect else "normal"
    )
    processes: list[tuple[str, subprocess.Popen]] = []
    handles = []
    stopping = False

    def request_stop(_signum=None, _frame=None):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    logs = ROOT / "runtime_logs"
    logs.mkdir(exist_ok=True)
    try:
        # Headless runs still use Ogre2 for the three simulated camera sensors.
        require_working_gpu()
        env = sourced_environment(ROS_SETUPS)
        env["ROS_LOG_DIR"] = str(logs / f"{stamp}_ros")
        Path(env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
        env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(filter(None, (
            str(ROOT / "simulation/models"), str(ROOT / "dji_stack/models"),
            env.get("GZ_SIM_RESOURCE_PATH", ""),
        )))
        previous_runs = set((ROOT / "datasets").glob("run_*"))
        ugv_cmd = [
            sys.executable, "-u", str(ROOT / "ground_stack/run_UGV_simulation.py"),
            "--hold-after-mission",
        ]
        if args.headless: ugv_cmd.append("--headless")
        if args.no_motion: ugv_cmd.append("--no-motion")
        ugv_log = (logs / f"{stamp}_ugv.log").open("w", encoding="utf-8")
        handles.append(ugv_log)
        ugv = subprocess.Popen(
            ugv_cmd, cwd=ROOT, env=env, stdout=ugv_log, stderr=subprocess.STDOUT,
            start_new_session=True, text=True,
        )
        processes.append(("UGV/Gazebo", ugv))
        print("[STARTED] Baylands + known-good Husky/Nav2 stack", flush=True)
        wait_for_world(ugv, env)

        candidates = [path for path in (ROOT / "datasets").glob("run_*") if path not in previous_runs]
        active_run = max(candidates, key=lambda path: path.stat().st_mtime) if candidates else ROOT / "datasets" / f"run_{stamp}"
        active_run.mkdir(parents=True, exist_ok=True)

        vehicle_log = (logs / f"{stamp}_dji_vehicles.log").open("w", encoding="utf-8")
        handles.append(vehicle_log)
        vehicles = subprocess.Popen(
            ["ros2", "launch", "dji_swarm_integration", "dji_vehicles.launch.py", f"scenario:={scenario}"],
            cwd=ROOT, env=env, stdout=vehicle_log, stderr=subprocess.STDOUT,
            start_new_session=True, text=True,
        )
        processes.append(("DJI vehicles", vehicles))
        threading.Thread(
            target=relay_events,
            args=(Path(vehicle_log.name), vehicles, lambda: stopping),
            daemon=True,
        ).start()
        print(f"[STARTED] DJI M100 dji0/dji1/dji2 | scenario={scenario}", flush=True)
        wait_for_dji_models(vehicles, env)
        print("[WAITING] DJI mission clock is paused. Click Play to activate DJI0 as scout.", flush=True)

        network_log = (logs / f"{stamp}_network_bridge.log").open("w", encoding="utf-8")
        handles.append(network_log)
        network = subprocess.Popen(
            ["ros2", "launch", "dji_swarm_integration", "dji_network.launch.py"],
            cwd=ROOT, env=env, stdout=network_log, stderr=subprocess.STDOUT,
            start_new_session=True, text=True,
        )
        processes.append(("DJI network bridge", network))
        wait_for_port(5555, network)

        if not args.no_yolo:
            perception_env = env.copy()
            perception_env["PYTHONPATH"] = (
                str(ROOT / "third_party/python") + os.pathsep
                + perception_env.get("PYTHONPATH", "")
            )
            perception_log = (logs / f"{stamp}_perception.log").open("w", encoding="utf-8")
            handles.append(perception_log)
            perception = subprocess.Popen(
                ["ros2", "launch", "dji_swarm_integration", "dji_perception.launch.py", f"evidence_root:={active_run / 'yolo_frames'}"],
                cwd=ROOT, env=perception_env, stdout=perception_log, stderr=subprocess.STDOUT,
                start_new_session=True, text=True,
            )
            processes.append(("DJI YOLO", perception))
            print("[STARTED] Three independent DJI camera + YOLO + 3D semantic observers", flush=True)
            print("[CAMERA] /dji0/camera0/image_raw", flush=True)
            print("[CAMERA] /dji1/camera0/image_raw", flush=True)
            print("[CAMERA] /dji2/camera0/image_raw", flush=True)

        if not args.no_omnet:
            omnet_env = sourced_environment((Path("/home/basudeo/omnetpp-6.0.1/setenv"),))
            omnet_log = (logs / f"{stamp}_omnet.log").open("w", encoding="utf-8")
            handles.append(omnet_log)
            omnet = subprocess.Popen(
                [str(ROOT / "omnet/out/gcc-release/omnet"), "-u", "Cmdenv", "-n", ".:uav_ugv:/home/basudeo/inet/src", "-l", "/home/basudeo/inet/src/INET", "omnetpp.ini"],
                cwd=ROOT / "omnet", env=omnet_env, stdout=omnet_log,
                stderr=subprocess.STDOUT, start_new_session=True, text=True,
            )
            processes.append(("OMNeT++", omnet))
            print("[STARTED] OMNeT++/INET Wi-Fi overlay for DJI0/1/2 links", flush=True)

        print(f"[RUNNING] Logs: {logs}", flush=True)
        print("[RUNNING] Click Play in Gazebo, then press Ctrl+C once to stop cleanly.", flush=True)
        while not stopping:
            for name, process in processes:
                if process.poll() is not None:
                    raise RuntimeError(f"{name} exited with code {process.returncode}")
            time.sleep(0.5)
        return 0
    except (
        FileNotFoundError, RuntimeError, TimeoutError,
        subprocess.CalledProcessError, subprocess.TimeoutExpired,
    ) as exc:
        print(f"[FAILED] {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        print("[STOPPING] Closing isolated DJI, UGV, Gazebo, ROS and OMNeT++ stack...", flush=True)
        for _name, process in reversed(processes): stop(process)
        for handle in handles: handle.close()
        print("[STOPPED] Complete DJI integration stack closed.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())

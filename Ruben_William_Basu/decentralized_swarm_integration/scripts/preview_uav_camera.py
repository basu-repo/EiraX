#!/usr/bin/env python3
"""Preview the isolated x500 camera mount and its live ROS image."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "simulation/worlds/uav_camera_preview.sdf"
GZ_IMAGE = (
    "/world/camera_preview/model/x500_mapping_preview/link/base_link/"
    "sensor/uav_rgbd/image"
)
ROS_IMAGE = "/preview/uav_camera/image_raw"


def stop(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def main() -> int:
    env = os.environ.copy()
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(
        (
            str(ROOT / "simulation/models"),
            str(ROOT / "px4_runtime/models"),
            env.get("GZ_SIM_RESOURCE_PATH", ""),
        )
    ).rstrip(":")
    ros_log = ROOT / "runtime_logs/camera_preview_ros"
    ros_log.mkdir(parents=True, exist_ok=True)
    env["ROS_LOG_DIR"] = str(ros_log)

    gazebo = bridge = viewer = None
    try:
        gazebo = subprocess.Popen(
            ["gz", "sim", "-r", str(WORLD)], env=env, start_new_session=True
        )
        time.sleep(4)
        if gazebo.poll() is not None:
            raise RuntimeError("Gazebo camera-preview world failed to start")
        bridge = subprocess.Popen(
            [
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                f"{GZ_IMAGE}@sensor_msgs/msg/Image[gz.msgs.Image",
                "--ros-args", "--remap", f"{GZ_IMAGE}:={ROS_IMAGE}",
            ],
            env=env,
            start_new_session=True,
        )
        time.sleep(3)
        if bridge.poll() is not None:
            raise RuntimeError("Gazebo-to-ROS camera bridge failed to start")
        viewer = subprocess.Popen(
            ["ros2", "run", "rqt_image_view", "rqt_image_view", ROS_IMAGE],
            env=env,
            start_new_session=True,
        )
        print("[RUNNING] Blank world + x500 camera preview")
        print(f"[CAMERA] Live ROS image: {ROS_IMAGE}")
        print("[RUNNING] Press Ctrl+C once to close Gazebo, bridge and viewer.")
        while gazebo.poll() is None and viewer.poll() is None:
            time.sleep(0.5)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        print("[STOPPING] Closing camera preview...")
        for process in (viewer, bridge, gazebo):
            stop(process)
        print("[STOPPED] Camera preview closed.")


if __name__ == "__main__":
    raise SystemExit(main())

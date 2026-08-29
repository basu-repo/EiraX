"""Monitor standalone UGV ROS and Gazebo health."""

from __future__ import annotations

import subprocess
import time


def gazebo_running() -> bool:
    result = subprocess.run(
        ["gz", "service", "-l"], capture_output=True, text=True, timeout=10
    )
    return result.returncode == 0 and "/server_control" in result.stdout.splitlines()


def topic_names() -> set[str]:
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"], capture_output=True, text=True, timeout=10
        )
    except subprocess.TimeoutExpired:
        # DDS discovery can briefly exceed this per-attempt timeout while
        # Gazebo, PX4 and both LiDAR pipelines are starting. The enclosing
        # wait_for_topics() deadline remains the authoritative failure limit.
        return set()
    return set(result.stdout.splitlines()) if result.returncode == 0 else set()


def wait_for_topics(required: list[str], timeout: float) -> tuple[bool, list[str]]:
    deadline = time.monotonic() + timeout
    missing = required
    while time.monotonic() < deadline:
        names = topic_names()
        missing = [topic for topic in required if topic not in names]
        if not missing:
            return True, []
        time.sleep(1)
    return False, missing


def wait_for_message(topic: str, timeout: float) -> bool:
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", topic, "--once"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
        )
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def wait_for_finite_odometry(topic: str, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(
                [
                    "ros2", "topic", "echo", topic, "--once",
                    "--field", "pose.pose.position.x",
                ],
                capture_output=True,
                text=True,
                timeout=min(5.0, max(0.1, deadline - time.monotonic())),
            )
            if result.returncode == 0:
                value = result.stdout.strip().splitlines()[0]
                if value.lower() not in ("nan", "inf", "-inf"):
                    float(value)
                    return True
        except (subprocess.TimeoutExpired, ValueError, IndexError):
            pass
        time.sleep(0.5)
    return False


def wait_for_lifecycle_active(node: str, timeout: float) -> bool:
    """Wait for a ROS 2 lifecycle node to report the active state."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(
                ["ros2", "lifecycle", "get", node],
                capture_output=True,
                text=True,
                timeout=min(5.0, max(0.1, deadline - time.monotonic())),
            )
            if result.returncode == 0 and "active [3]" in result.stdout:
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.5)
    return False

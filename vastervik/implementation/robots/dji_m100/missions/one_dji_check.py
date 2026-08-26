#!/usr/bin/env python3
"""Fly one simulated DJI M100 through a small, observable test path."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy


class OneDjiCheck(Node):
    def __init__(self, name: str) -> None:
        super().__init__("one_dji_check")
        self.name = name
        self.pose_received = False
        self.image_received = False
        self.publisher = self.create_publisher(
            Joy, f"/{name}/psdk_ros2/flight_control_setpoint_ENUposition_yaw", 10
        )
        self.create_subscription(PoseStamped, f"/{name}/pose", self._pose, 10)
        self.create_subscription(Image, f"/{name}/camera0/image_raw", self._image, 1)

    def _pose(self, _message: PoseStamped) -> None:
        self.pose_received = True

    def _image(self, _message: Image) -> None:
        self.image_received = True

    def publish_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        message = Joy()
        message.axes = [float(x), float(y), float(z), float(yaw)]
        self.publisher.publish(message)


def wait_until(node: OneDjiCheck, predicate, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def move(
    node: OneDjiCheck,
    start: tuple[float, float, float, float],
    end: tuple[float, float, float, float],
    speed: float = 1.0,
) -> None:
    distance = math.dist(start[:3], end[:3])
    duration = max(1.0, distance / speed)
    steps = max(1, int(duration * 20.0))
    for index in range(1, steps + 1):
        ratio = index / steps
        pose = tuple(a + (b - a) * ratio for a, b in zip(start, end))
        node.publish_pose(*pose)
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", default="dji0")
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = OneDjiCheck(args.name)
    try:
        print("[WAITING] Click Play in Gazebo; waiting for the DJI pose topic.", flush=True)
        if not wait_until(node, lambda: node.pose_received, 300.0):
            print("[FAILED] No DJI pose received.", flush=True)
            return 2
        print(f"[DJI READY] {args.name} pose interface is active.", flush=True)
        if not wait_until(node, lambda: node.image_received, 30.0):
            print("[FAILED] DJI camera did not publish an image.", flush=True)
            return 2
        print(
            f"[CAMERA READY] /{args.name}/camera0/image_raw is publishing.",
            flush=True,
        )

        initial = (args.x, args.y, args.z, 0.0)
        climb = (args.x, args.y, args.z + 2.0, 0.0)
        forward = (args.x - 8.0, args.y, args.z + 2.0, 0.0)
        side = (args.x - 8.0, args.y - 4.0, args.z + 2.0, -0.5)
        print("[FLIGHT] Climb -> forward -> side -> return to hover.", flush=True)
        move(node, initial, climb, 0.7)
        move(node, climb, forward, 1.0)
        move(node, forward, side, 0.8)
        move(node, side, climb, 1.0)
        for _ in range(40):
            node.publish_pose(*climb)
            rclpy.spin_once(node, timeout_sec=0.05)
        print("[DJI CHECK COMPLETE] Model, command interface and camera passed.", flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

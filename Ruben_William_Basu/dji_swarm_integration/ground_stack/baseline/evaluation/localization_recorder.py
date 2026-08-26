"""Record standalone UGV localization estimates and errors."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


FIELDS = [
    "stamp_sec",
    "ground_truth_x", "ground_truth_y", "ground_truth_yaw",
    "wheel_x", "wheel_y", "wheel_yaw", "wheel_position_error", "wheel_yaw_error",
    "ekf_x", "ekf_y", "ekf_yaw", "ekf_position_error", "ekf_yaw_error",
    "local_ekf_x", "local_ekf_y", "local_ekf_yaw", "local_ekf_position_error", "local_ekf_yaw_error",
    "lidar_odom_x", "lidar_odom_y", "lidar_odom_yaw", "lidar_odom_position_error", "lidar_odom_yaw_error",
    "lidar_slam_x", "lidar_slam_y", "lidar_slam_yaw", "lidar_slam_position_error", "lidar_slam_yaw_error",
    "rtab_x", "rtab_y", "rtab_yaw", "rtab_position_error", "rtab_yaw_error",
]


def yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(value: float, reference: float) -> float:
    return math.atan2(math.sin(value - reference), math.cos(value - reference))


class LocalizationRecorder(Node):
    def __init__(self, output: Path) -> None:
        super().__init__("localization_evaluator")
        output.parent.mkdir(parents=True, exist_ok=True)
        self.stream = output.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.stream, fieldnames=FIELDS)
        self.writer.writeheader()
        self.samples = 0
        self.origin: tuple[float, float, float] | None = None
        self.latest: dict[str, Odometry] = {}
        self.latest_lidar_slam: PoseWithCovarianceStamped | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, "/wheel/odom", lambda m: self._odom("wheel", m), 30)
        self.create_subscription(Odometry, "/odom", lambda m: self._odom("ekf", m), 30)
        self.create_subscription(Odometry, "/odometry/local", lambda m: self._odom("local_ekf", m), 30)
        self.create_subscription(Odometry, "/lidar3d/odom", lambda m: self._odom("lidar_odom", m), 30)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/rtabmap_lidar/localization_pose",
            self._lidar_slam,
            30,
        )
        self.create_subscription(PoseStamped, "/ground_truth/pose", self._ground_truth, 30)

    def _odom(self, name: str, msg: Odometry) -> None:
        self.latest[name] = msg

    def _lidar_slam(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_lidar_slam = msg

    def _normalized_ground_truth(self, msg: PoseStamped) -> tuple[float, float, float]:
        gx, gy, gyaw = msg.pose.position.x, msg.pose.position.y, yaw(msg.pose.orientation)
        if self.origin is None:
            self.origin = gx, gy, gyaw
        ox, oy, oyaw = self.origin
        # Gazebo DiffDrive odometry and mission goals are translated from the
        # spawn pose but remain aligned with the world ENU axes. Do not rotate
        # positions by the Husky's initial heading.
        return gx - ox, gy - oy, angle_error(gyaw, oyaw)

    @staticmethod
    def _add_estimate(row: dict, name: str, x: float, y: float, estimate_yaw: float,
                      gt: tuple[float, float, float]) -> None:
        row[f"{name}_x"] = x
        row[f"{name}_y"] = y
        row[f"{name}_yaw"] = estimate_yaw
        row[f"{name}_position_error"] = math.hypot(x - gt[0], y - gt[1])
        row[f"{name}_yaw_error"] = angle_error(estimate_yaw, gt[2])

    def _ground_truth(self, msg: PoseStamped) -> None:
        gt = self._normalized_ground_truth(msg)
        row = {field: "" for field in FIELDS}
        row.update({
            "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "ground_truth_x": gt[0],
            "ground_truth_y": gt[1],
            "ground_truth_yaw": gt[2],
        })
        for name, odom in self.latest.items():
            pose = odom.pose.pose
            x, y = pose.position.x, pose.position.y
            # Wheel odometry and the GNSS-free local EKF start with x along the
            # vehicle's spawn heading. Rotate those local coordinates into the
            # world-aligned, spawn-relative axes used by ground truth. Their yaw
            # already remains relative to the spawn heading.
            if name in {"wheel", "local_ekf", "lidar_odom"} and self.origin is not None:
                heading = self.origin[2]
                c, s = math.cos(heading), math.sin(heading)
                x, y = c * x - s * y, s * x + c * y
            estimate_yaw = yaw(pose.orientation)
            if name == "ekf" and self.origin is not None:
                estimate_yaw = angle_error(estimate_yaw, self.origin[2])
            self._add_estimate(row, name, x, y, estimate_yaw, gt)
        if self.latest_lidar_slam is not None and self.origin is not None:
            pose = self.latest_lidar_slam.pose.pose
            heading = self.origin[2]
            c, s = math.cos(heading), math.sin(heading)
            x = c * pose.position.x - s * pose.position.y
            y = s * pose.position.x + c * pose.position.y
            self._add_estimate(row, "lidar_slam", x, y, yaw(pose.orientation), gt)
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            p = transform.transform.translation
            estimate_yaw = yaw(transform.transform.rotation)
            if self.origin is not None:
                estimate_yaw = angle_error(estimate_yaw, self.origin[2])
            self._add_estimate(row, "rtab", p.x, p.y, estimate_yaw, gt)
        except TransformException:
            pass
        self.writer.writerow(row)
        self.samples += 1
        if self.samples % 30 == 0:
            self.stream.flush()

    def destroy_node(self) -> bool:
        self.stream.flush()
        self.stream.close()
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = LocalizationRecorder(args.output)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

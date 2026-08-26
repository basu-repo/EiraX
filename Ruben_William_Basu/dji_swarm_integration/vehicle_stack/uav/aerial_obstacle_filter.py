"""Downsample the UAV LiDAR cloud for cooperative UGV global planning."""

from __future__ import annotations

import argparse
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class AerialObstacleFilter(Node):
    def __init__(
        self,
        east_offset_m: float,
        north_offset_m: float,
        up_offset_m: float,
        exclusions: list[tuple[float, float]],
    ) -> None:
        super().__init__("aerial_obstacle_filter")
        self.origin = east_offset_m, north_offset_m, up_offset_m
        self.pose: tuple[float, float, float, float, float, float, float] | None = None
        self.ugv_xy: tuple[float, float] | None = None
        self.exclusions = exclusions
        self.publisher = self.create_publisher(
            PointCloud2, "/communication/uav/tx/aerial_obstacles", 2
        )
        self.create_subscription(Odometry, "/uav/px4_odom", self.update_pose, 10)
        self.create_subscription(
            Odometry,
            "/communication/uav/rx/ugv_odom",
            self.update_ugv_pose,
            10,
        )
        self.create_subscription(
            PointCloud2, "/x500/lidar3d/points", self.filter_cloud, 2
        )

    def update_pose(self, message: Odometry) -> None:
        position = message.pose.pose.position
        rotation = message.pose.pose.orientation
        self.pose = (
            position.x,
            position.y,
            position.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )

    def update_ugv_pose(self, message: Odometry) -> None:
        self.ugv_xy = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        )

    def filter_cloud(self, message: PointCloud2) -> None:
        if self.pose is None:
            return
        pose_x, pose_y, pose_z, qx, qy, qz, qw = self.pose
        origin_x, origin_y, origin_z = self.origin
        # Quaternion rotation matrix. Doing this explicitly avoids carrying
        # Gazebo's padded PointCloud2 fields into tf2_sensor_msgs.
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        r00, r01, r02 = 1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)
        r10, r11, r12 = 2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)
        r20, r21, r22 = 2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)
        filtered: list[tuple[float, float, float]] = []
        for index, point in enumerate(
            point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True
            )
        ):
            # The 32 x 512 sensor is intentionally reduced before it reaches
            # the large UGV costmap. Height classification is performed after
            # TF transforms the points into the UGV odometry frame.
            if index % 8:
                continue
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            distance = math.sqrt(x * x + y * y + z * z)
            if 1.0 <= distance <= 120.0:
                odom_point = (
                    r00 * x + r01 * y + r02 * z + origin_x + pose_x,
                    r10 * x + r11 * y + r12 * z + origin_y + pose_y,
                    # The LiDAR is mounted 0.04 m below uav_base_link.
                    r20 * x + r21 * y + r22 * z + origin_z + pose_z - 0.04,
                )
                # Do not leave a permanent aerial-costmap trail of the moving
                # Husky itself. Its onboard local costmap still sees every
                # nearby external obstacle and retains final safety authority.
                if self.ugv_xy is not None and math.hypot(
                    odom_point[0] - self.ugv_xy[0],
                    odom_point[1] - self.ugv_xy[1],
                ) < 1.5:
                    continue
                # Gazebo waypoint/goal models are visual mission annotations,
                # not real terrain obstacles. Remove only their compact
                # footprints; surrounding physical geometry remains intact.
                if any(
                    math.hypot(odom_point[0] - target_x, odom_point[1] - target_y)
                    < 0.8
                    for target_x, target_y in self.exclusions
                ):
                    continue
                filtered.append(odom_point)
        header = Header(stamp=message.header.stamp, frame_id="odom")
        output = point_cloud2.create_cloud_xyz32(header, filtered)
        self.publisher.publish(output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--east-offset", type=float, required=True)
    parser.add_argument("--north-offset", type=float, required=True)
    parser.add_argument("--up-offset", type=float, required=True)
    parser.add_argument(
        "--exclude",
        action="append",
        default=[],
        metavar="X,Y",
        help="Compact odom-frame visual marker footprint to ignore",
    )
    args = parser.parse_args()
    exclusions = [
        tuple(float(value) for value in item.split(",", maxsplit=1))
        for item in args.exclude
    ]
    rclpy.init()
    node = AerialObstacleFilter(
        args.east_offset, args.north_offset, args.up_offset, exclusions
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

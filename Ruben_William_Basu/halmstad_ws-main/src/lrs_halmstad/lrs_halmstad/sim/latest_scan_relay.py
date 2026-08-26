#!/usr/bin/env python3

import copy

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


class LatestScanRelay(Node):
    def __init__(self) -> None:
        super().__init__("latest_scan_relay")

        numeric_param = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_relay")
        self.declare_parameter("publish_hz", 2.0, numeric_param)
        self.declare_parameter("max_age_s", 1.0, numeric_param)
        self.declare_parameter("restamp", True)
        self.declare_parameter("stamp_offset_s", 0.0, numeric_param)
        self.declare_parameter("startup_hold_s", 0.0, numeric_param)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        publish_hz = max(0.1, float(self.get_parameter("publish_hz").value))
        self.max_age_s = max(0.0, float(self.get_parameter("max_age_s").value))
        self.restamp = bool(self.get_parameter("restamp").value)
        self.stamp_offset_ns = int(float(self.get_parameter("stamp_offset_s").value) * 1e9)
        self.startup_hold_ns = int(max(0.0, float(self.get_parameter("startup_hold_s").value)) * 1e9)

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.latest_scan: LaserScan | None = None
        self.latest_received_ns: int | None = None
        self.ready_after_ns: int | None = None

        self.create_subscription(LaserScan, input_topic, self._on_scan, input_qos)
        self.publisher = self.create_publisher(LaserScan, output_topic, output_qos)
        self.create_timer(1.0 / publish_hz, self._on_timer)

        self.get_logger().info(
            f"Relaying latest scan from {input_topic} to {output_topic} at {publish_hz:.2f} Hz "
            f"(restamp={self.restamp}, stamp_offset_s={self.stamp_offset_ns / 1e9:.3f}, "
            f"max_age_s={self.max_age_s:.2f}, startup_hold_s={self.startup_hold_ns / 1e9:.2f})"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_received_ns = self.get_clock().now().nanoseconds
        if self.ready_after_ns is None:
            self.ready_after_ns = self.latest_received_ns + self.startup_hold_ns

    def _on_timer(self) -> None:
        if self.latest_scan is None or self.latest_received_ns is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.ready_after_ns is not None and now_ns < self.ready_after_ns:
            return

        age_s = (now_ns - self.latest_received_ns) / 1e9
        if self.max_age_s > 0.0 and age_s > self.max_age_s:
            return

        msg = copy.deepcopy(self.latest_scan)
        if self.restamp:
            stamp_ns = max(0, self.get_clock().now().nanoseconds + self.stamp_offset_ns)
            msg.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LatestScanRelay()
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

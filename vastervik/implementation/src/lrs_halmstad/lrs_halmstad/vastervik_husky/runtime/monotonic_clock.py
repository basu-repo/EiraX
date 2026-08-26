"""Publish a monotonic ROS simulation clock for the standalone UGV."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class MonotonicClock(Node):
    """Keep delayed Gazebo clock packets from rewinding ROS simulation time."""

    def __init__(self) -> None:
        super().__init__("monotonic_clock")
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
        self._last_nanoseconds = -1
        self._publisher = self.create_publisher(Clock, "/clock", output_qos)
        self.create_subscription(Clock, "/gazebo_clock", self._forward, input_qos)

    def _forward(self, message: Clock) -> None:
        nanoseconds = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        if nanoseconds <= self._last_nanoseconds:
            return
        self._last_nanoseconds = nanoseconds
        self._publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = MonotonicClock()
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

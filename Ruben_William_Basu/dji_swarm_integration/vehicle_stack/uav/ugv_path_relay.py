"""Publish the UAV-informed UGV global Nav2 plan on a cooperative topic."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node


class UgvPathRelay(Node):
    def __init__(self) -> None:
        super().__init__("cooperative_ugv_path_relay")
        self.publisher = self.create_publisher(
            Path, "/cooperative/ugv_global_path", 10
        )
        self.create_subscription(Path, "/plan", self.publisher.publish, 10)


def main() -> None:
    rclpy.init()
    node = UgvPathRelay()
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

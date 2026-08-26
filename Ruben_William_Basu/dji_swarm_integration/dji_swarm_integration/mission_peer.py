"""Publish this peer's PX4 setpoint from its role lease and shared intent."""

from __future__ import annotations

import json
import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .mission_geometry import MissionPoint, role_setpoint


def _yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )


class MissionPeer(Node):
    def __init__(self):
        super().__init__("mission_peer")
        self.uav_id = str(self.declare_parameter("uav_id", "uav").value).strip()
        self.slot = int(self.declare_parameter("slot", 0).value)
        self.output_enabled = bool(self.declare_parameter("output_enabled", False).value)
        self.world_frame = str(self.declare_parameter("world_frame", "map").value).strip()
        self.altitude_m = float(self.declare_parameter("altitude_m", 10.0).value)
        self.scout_lead_m = float(self.declare_parameter("scout_lead_m", 20.0).value)
        self.spacing_m = float(self.declare_parameter("formation_spacing_m", 5.0).value)
        self.observer_standoff_m = float(
            self.declare_parameter("observer_standoff_m", 8.0).value
        )
        self.input_timeout_s = float(self.declare_parameter("input_timeout_s", 2.0).value)
        role_topic = f"/coord/swarm/{self.uav_id}/role"
        consensus_pose_topic = f"/coord/swarm/{self.uav_id}/consensus_pose"
        anchor_topic = str(
            self.declare_parameter("anchor_topic", "/coord/swarm/mission_anchor").value
        )
        setpoint_topic = f"/coord/swarm/{self.uav_id}/setpoint"
        status_topic = f"/coord/swarm/{self.uav_id}/mission_status"
        self._setpoint_pub = self.create_publisher(PoseStamped, setpoint_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, role_topic, self._on_role, 10)
        self.create_subscription(PoseStamped, anchor_topic, self._on_anchor, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, consensus_pose_topic, self._on_target, 10
        )
        self._role = "reserve"
        self._role_expiry_ns = 0
        self._anchor = None
        self._anchor_ns = 0
        self._target = None
        self._target_ns = 0
        self.create_timer(0.1, self._on_timer)

    def _on_role(self, message):
        try:
            payload = json.loads(message.data)
            if payload.get("uav_id") != self.uav_id:
                return
            self._role = str(payload["role"])
            self._role_expiry_ns = int(payload["lease_expires_ros_ns"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self._role = "reserve"
            self._role_expiry_ns = 0

    def _on_anchor(self, message):
        if str(message.header.frame_id).strip() != self.world_frame:
            self._status("mission_anchor_wrong_frame")
            return
        p = message.pose.position
        self._anchor = MissionPoint(float(p.x), float(p.y), float(p.z), _yaw(message.pose.orientation))
        self._anchor_ns = self.get_clock().now().nanoseconds

    def _on_target(self, message):
        if str(message.header.frame_id).strip() != self.world_frame:
            self._status("semantic_target_wrong_frame")
            return
        p = message.pose.pose.position
        self._target = MissionPoint(float(p.x), float(p.y), float(p.z), 0.0)
        self._target_ns = self.get_clock().now().nanoseconds

    def _on_timer(self):
        now_ns = self.get_clock().now().nanoseconds
        timeout_ns = int(self.input_timeout_s * 1_000_000_000)
        if not self.output_enabled:
            self._status("disabled")
            return
        if now_ns >= self._role_expiry_ns:
            self._status("role_lease_expired")
            return
        if self._anchor is None or now_ns - self._anchor_ns > timeout_ns:
            self._status("mission_anchor_stale")
            return
        target = self._target if now_ns - self._target_ns <= timeout_ns else None
        try:
            result = role_setpoint(
                role=self._role,
                slot=self.slot,
                anchor=self._anchor,
                semantic_target=target,
                altitude_m=self.altitude_m,
                scout_lead_m=self.scout_lead_m,
                formation_spacing_m=self.spacing_m,
                observer_standoff_m=self.observer_standoff_m,
            )
        except ValueError as exc:
            self._status(f"invalid_role:{exc}")
            return
        if result is None:
            self._status("holding")
            return
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.world_frame
        message.pose.position.x = result.x
        message.pose.position.y = result.y
        message.pose.position.z = result.z
        message.pose.orientation.z = math.sin(result.yaw / 2.0)
        message.pose.orientation.w = math.cos(result.yaw / 2.0)
        self._setpoint_pub.publish(message)
        self._status("commanding")

    def _status(self, state):
        message = String()
        message.data = (
            f"uav_id={self.uav_id} role={self._role} state={state} "
            f"output_enabled={str(self.output_enabled).lower()}"
        )
        self._status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPeer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

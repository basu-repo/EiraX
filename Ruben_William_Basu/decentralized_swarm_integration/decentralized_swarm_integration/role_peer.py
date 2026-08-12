"""Decentralized role peer: advertise local capability and compute all leases."""

from __future__ import annotations

import json

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from .role_protocol import AgentState, assign_roles, decode_agent_state, encode_agent_state


class RolePeer(Node):
    def __init__(self):
        super().__init__("role_peer")
        self.uav_id = str(self.declare_parameter("uav_id", "uav").value).strip()
        self.camera_capable = bool(self.declare_parameter("camera_capable", False).value)
        self.lidar_capable = bool(self.declare_parameter("lidar_capable", True).value)
        self.default_battery = float(self.declare_parameter("default_battery", 1.0).value)
        self.state_timeout_s = float(self.declare_parameter("state_timeout_s", 2.5).value)
        self.lease_duration_s = float(self.declare_parameter("lease_duration_s", 3.0).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 2.0).value)
        shared_state_topic = str(
            self.declare_parameter("shared_state_topic", "/coord/swarm/agent_states").value
        )
        network_topic = str(
            self.declare_parameter("network_topic", "/coord/swarm/network_metrics").value
        )
        consensus_topic = str(
            self.declare_parameter(
                "consensus_topic", f"/coord/swarm/{self.uav_id}/consensus"
            ).value
        )
        battery_topic = str(
            self.declare_parameter("battery_topic", f"/{self.uav_id}/battery").value
        )
        odom_topic = str(
            self.declare_parameter("odom_topic", f"/{self.uav_id}/px4/odom").value
        )
        role_topic = str(
            self.declare_parameter("role_topic", f"/coord/swarm/{self.uav_id}/role").value
        )

        self._states = {}
        self._sequence = 0
        self._battery = max(0.0, min(1.0, self.default_battery))
        self._link_quality = 0.0
        self._detection_confidence = 0.0
        self._last_odom_ns = 0
        self._state_pub = self.create_publisher(String, shared_state_topic, 20)
        self._role_pub = self.create_publisher(String, role_topic, 10)
        self.create_subscription(String, shared_state_topic, self._on_state, 20)
        self.create_subscription(String, network_topic, self._on_network, 20)
        self.create_subscription(String, consensus_topic, self._on_consensus, 10)
        self.create_subscription(BatteryState, battery_topic, self._on_battery, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_timer(1.0 / max(0.2, self.publish_rate_hz), self._on_timer)

    def _on_state(self, message):
        now_ns = self.get_clock().now().nanoseconds
        try:
            state = decode_agent_state(message.data, now_ns)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"invalid agent state: {exc}", throttle_duration_sec=2.0)
            return
        previous = self._states.get(state.uav_id)
        if previous is None or (state.sequence, state.stamp_ns) > (
            previous.sequence,
            previous.stamp_ns,
        ):
            self._states[state.uav_id] = state

    def _on_network(self, message):
        try:
            payload = json.loads(message.data)
            if payload.get("uav_id") == self.uav_id and payload.get("quality") is not None:
                self._link_quality = max(0.0, min(1.0, float(payload["quality"])))
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def _on_consensus(self, message):
        try:
            payload = json.loads(message.data)
            value = float(payload.get("confidence", 0.0))
            self._detection_confidence = max(0.0, min(1.0, value))
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def _on_battery(self, message):
        if 0.0 <= float(message.percentage) <= 1.0:
            self._battery = float(message.percentage)

    def _on_odom(self, _message):
        self._last_odom_ns = self.get_clock().now().nanoseconds

    def _on_timer(self):
        now_ns = self.get_clock().now().nanoseconds
        self._sequence += 1
        mobile = self._last_odom_ns > 0 and now_ns - self._last_odom_ns <= 2_000_000_000
        local = AgentState(
            uav_id=self.uav_id,
            sequence=self._sequence,
            stamp_ns=now_ns,
            received_ns=now_ns,
            battery=self._battery,
            link_quality=self._link_quality,
            detection_confidence=self._detection_confidence,
            camera_capable=self.camera_capable,
            lidar_capable=self.lidar_capable,
            mobile=mobile,
        )
        self._states[self.uav_id] = local
        state_message = String()
        state_message.data = encode_agent_state(local)
        self._state_pub.publish(state_message)

        max_age_ns = int(self.state_timeout_s * 1_000_000_000)
        self._states = {
            identity: state
            for identity, state in self._states.items()
            if 0 <= now_ns - state.received_ns <= max_age_ns
        }
        assignments = assign_roles(list(self._states.values()))
        role = assignments.get(self.uav_id, "reserve")
        role_message = String()
        role_message.data = json.dumps(
            {
                "protocol": "eirax.role_lease.v1",
                "uav_id": self.uav_id,
                "role": role,
                "assignments": assignments,
                "computed_ros_ns": now_ns,
                "lease_expires_ros_ns": now_ns
                + int(self.lease_duration_s * 1_000_000_000),
                "participants": sorted(self._states),
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self._role_pub.publish(role_message)


def main(args=None):
    rclpy.init(args=args)
    node = RolePeer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

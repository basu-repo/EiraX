"""Safety boundary between decentralized commands and DJI PSDK-compatible ROS topics."""

from __future__ import annotations

import json
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, String

from .dji_protocol import DjiCommand, decode_command


class DjiVehicleAgent(Node):
    """Validate, rate-limit and publish one DJI aircraft's commands.

    Simulation and the physical-team interface share the same Joy payload:
    axes=[absolute ENU x, y, z, yaw]. Hardware output is disabled unless both
    ``mode=hardware`` and ``hardware_authorized=true`` are explicitly set.
    """

    def __init__(self):
        super().__init__("dji_vehicle_agent")
        self.uav_id = str(self.declare_parameter("uav_id", "dji0").value).strip()
        self.mode = str(self.declare_parameter("mode", "simulation").value).strip().lower()
        self.hardware_authorized = bool(
            self.declare_parameter("hardware_authorized", False).value
        )
        self.command_timeout_s = float(self.declare_parameter("command_timeout_s", 2.5).value)
        self.pose_timeout_s = float(self.declare_parameter("pose_timeout_s", 2.0).value)
        self.max_horizontal_step_m = float(
            self.declare_parameter("max_horizontal_step_m", 12.0).value
        )
        self.min_altitude_m = float(self.declare_parameter("min_altitude_m", 0.0).value)
        self.max_altitude_m = float(self.declare_parameter("max_altitude_m", 30.0).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 10.0).value)
        if self.mode not in {"simulation", "hardware"}:
            raise ValueError("mode must be simulation or hardware")
        if self.mode == "hardware" and not self.hardware_authorized:
            raise RuntimeError("DJI hardware mode requires hardware_authorized:=true")

        self._last_pose: PoseStamped | None = None
        self._last_pose_recv_ns = 0
        self._pending: DjiCommand | None = None
        self._last_sequence = -1
        self._last_command_recv_ns = 0
        self._state = "WAITING_FOR_POSE"

        command_topic = f"/coord/swarm/{self.uav_id}/dji_command"
        pose_topic = f"/{self.uav_id}/pose"
        psdk_topic = f"/{self.uav_id}/psdk_ros2/flight_control_setpoint_ENUposition_yaw"
        self._psdk_pub = self.create_publisher(Joy, psdk_topic, 10)
        self._pan_pub = self.create_publisher(Float64, f"/{self.uav_id}/update_pan", 10)
        self._tilt_pub = self.create_publisher(Float64, f"/{self.uav_id}/update_tilt", 10)
        self._status_pub = self.create_publisher(
            String, f"/coord/swarm/{self.uav_id}/dji_status", 10
        )
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)
        self.create_subscription(String, command_topic, self._on_command, 10)
        self.create_timer(1.0 / max(1.0, self.publish_rate_hz), self._tick)
        self.get_logger().info(
            f"DJI agent {self.uav_id}: mode={self.mode}, command={command_topic}, "
            f"pose={pose_topic}, psdk={psdk_topic}"
        )

    def _on_pose(self, message: PoseStamped) -> None:
        self._last_pose = message
        self._last_pose_recv_ns = self.get_clock().now().nanoseconds

    def _on_command(self, message: String) -> None:
        try:
            command = decode_command(message.data)
            if command.uav_id != self.uav_id:
                raise ValueError("command addressed to another aircraft")
            if command.sequence <= self._last_sequence:
                return
            self._validate_operational_envelope(command)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Rejected DJI command: {exc}", throttle_duration_sec=1.0)
            self._state = "COMMAND_REJECTED"
            return
        self._pending = command
        self._last_sequence = command.sequence
        self._last_command_recv_ns = self.get_clock().now().nanoseconds
        self._state = command.kind.upper()

    def _validate_operational_envelope(self, command: DjiCommand) -> None:
        if not self.min_altitude_m <= command.z <= self.max_altitude_m:
            raise ValueError("altitude outside configured envelope")
        if self._last_pose is None or command.kind in {"return", "land"}:
            return
        p = self._last_pose.pose.position
        horizontal_step = math.hypot(command.x - p.x, command.y - p.y)
        if horizontal_step > self.max_horizontal_step_m:
            raise ValueError(
                f"horizontal step {horizontal_step:.2f} m exceeds "
                f"{self.max_horizontal_step_m:.2f} m"
            )

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        pose_fresh = (
            self._last_pose is not None
            and now_ns - self._last_pose_recv_ns <= int(self.pose_timeout_s * 1e9)
        )
        command_fresh = (
            self._pending is not None
            and now_ns - self._last_command_recv_ns <= int(self.command_timeout_s * 1e9)
        )
        if not pose_fresh:
            self._state = "POSE_STALE"
        elif not command_fresh:
            self._state = "HOLD_STALE_COMMAND"
        else:
            command = self._pending
            joy = Joy()
            joy.header.stamp = self.get_clock().now().to_msg()
            joy.axes = [command.x, command.y, command.z, command.yaw]
            self._psdk_pub.publish(joy)
            pan = Float64(); pan.data = command.pan_deg
            tilt = Float64(); tilt.data = command.tilt_deg
            self._pan_pub.publish(pan)
            self._tilt_pub.publish(tilt)
        status = String()
        status.data = json.dumps(
            {
                "protocol": "eirax.dji_status.v1",
                "uav_id": self.uav_id,
                "mode": self.mode,
                "state": self._state,
                "pose_fresh": pose_fresh,
                "command_fresh": command_fresh,
                "last_sequence": self._last_sequence,
            },
            separators=(",", ":"), sort_keys=True,
        )
        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = DjiVehicleAgent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


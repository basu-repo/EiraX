"""ROS peer that converts local Halmstad YOLO evidence into swarm consensus."""

from __future__ import annotations

import json
import math
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .consensus import (
    SemanticObservation,
    decode_observation,
    encode_observation,
    reach_consensus,
)


class SemanticPeer(Node):
    def __init__(self) -> None:
        super().__init__("semantic_peer")
        self.uav_id = str(self.declare_parameter("uav_id", "uav").value).strip()
        if not self.uav_id:
            raise ValueError("uav_id must not be empty")

        self.detection_topic = str(self.declare_parameter("detection_topic", "").value).strip()
        self.estimate_topic = str(self.declare_parameter("estimate_topic", "").value).strip()
        self.status_topic = str(self.declare_parameter("status_topic", "").value).strip()
        self.shared_topic = str(
            self.declare_parameter("shared_topic", "/coord/swarm/semantic_observations").value
        ).strip()
        self.consensus_topic = str(
            self.declare_parameter("consensus_topic", f"/coord/swarm/{self.uav_id}/consensus").value
        ).strip()
        self.consensus_pose_topic = str(
            self.declare_parameter(
                "consensus_pose_topic", f"/coord/swarm/{self.uav_id}/consensus_pose"
            ).value
        ).strip()
        self.max_age_s = float(self.declare_parameter("max_age_s", 1.5).value)
        self.local_pair_timeout_s = float(self.declare_parameter("local_pair_timeout_s", 0.5).value)
        self.min_confidence = float(self.declare_parameter("min_confidence", 0.35).value)
        self.min_independent_sources = int(self.declare_parameter("min_independent_sources", 2).value)
        self.max_spread_m = float(self.declare_parameter("max_spread_m", 3.0).value)
        self.allow_single_source = bool(self.declare_parameter("allow_single_source", False).value)
        self.single_source_confidence = float(
            self.declare_parameter("single_source_confidence", 0.90).value
        )
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 5.0).value)

        if not self.detection_topic or not self.estimate_topic:
            raise ValueError("detection_topic and estimate_topic are required")

        self._last_detection: dict | None = None
        self._last_detection_recv_ns = 0
        self._last_estimate: PoseStamped | None = None
        self._last_estimate_recv_ns = 0
        self._last_status = ""
        self._observations: dict[tuple[str, str], SemanticObservation] = {}
        self._last_local_observation_id = ""

        self._shared_pub = self.create_publisher(String, self.shared_topic, 20)
        self._consensus_pub = self.create_publisher(String, self.consensus_topic, 10)
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.consensus_pose_topic, 10
        )
        self.create_subscription(String, self.detection_topic, self._on_detection, 10)
        self.create_subscription(PoseStamped, self.estimate_topic, self._on_estimate, 10)
        if self.status_topic:
            self.create_subscription(String, self.status_topic, self._on_status, 10)
        self.create_subscription(String, self.shared_topic, self._on_peer_observation, 20)
        self.create_timer(1.0 / max(0.1, self.publish_rate_hz), self._on_timer)

        self.get_logger().info(
            f"semantic peer={self.uav_id} detection={self.detection_topic} "
            f"estimate={self.estimate_topic} shared={self.shared_topic}"
        )

    def _on_detection(self, message: String) -> None:
        now_ns = self.get_clock().now().nanoseconds
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("detection payload is not an object")
            self._last_detection = payload
            self._last_detection_recv_ns = now_ns
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"invalid local detection: {exc}", throttle_duration_sec=2.0)

    def _on_estimate(self, message: PoseStamped) -> None:
        self._last_estimate = message
        self._last_estimate_recv_ns = self.get_clock().now().nanoseconds

    def _on_status(self, message: String) -> None:
        self._last_status = message.data.strip()

    def _on_peer_observation(self, message: String) -> None:
        now_ns = self.get_clock().now().nanoseconds
        try:
            observation = decode_observation(message.data, now_ns)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"invalid swarm observation: {exc}", throttle_duration_sec=2.0)
            return
        self._observations[(observation.source_id, observation.observation_id)] = observation

    def _make_local_observation(self, now_ns: int) -> SemanticObservation | None:
        detection = self._last_detection
        if detection is None or not bool(detection.get("valid", False)):
            return None
        stamp_ns = int(detection.get("stamp_ns", 0) or now_ns)
        observation_id = f"{self.uav_id}:{stamp_ns}"
        if observation_id == self._last_local_observation_id:
            return None

        estimate_fresh = (
            self._last_estimate is not None
            and now_ns - self._last_estimate_recv_ns
            <= int(self.local_pair_timeout_s * 1_000_000_000)
        )
        position = self._last_estimate.pose.position if estimate_fresh else None
        geometry_valid = bool(
            position is not None
            and all(math.isfinite(value) for value in (position.x, position.y, position.z))
        )
        observation = SemanticObservation(
            source_id=self.uav_id,
            observation_id=observation_id,
            stamp_ns=stamp_ns,
            received_ns=now_ns,
            class_id=None if detection.get("cls_id") is None else int(detection["cls_id"]),
            class_name=str(detection.get("cls_name", "")),
            confidence=float(detection.get("conf", 0.0)),
            track_id=None if detection.get("track_id") is None else int(detection["track_id"]),
            track_state=str(detection.get("track_state", "raw")),
            x=position.x if geometry_valid else None,
            y=position.y if geometry_valid else None,
            z=position.z if geometry_valid else None,
            frame_id=str(self._last_estimate.header.frame_id) if geometry_valid else "",
            geometry_valid=geometry_valid,
            status=self._last_status,
        )
        self._last_local_observation_id = observation_id
        return observation

    def _on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        local = self._make_local_observation(now_ns)
        if local is not None:
            message = String()
            message.data = encode_observation(local)
            self._shared_pub.publish(message)
            # Do not rely on local ROS loopback timing for local evidence.
            self._observations[(local.source_id, local.observation_id)] = local

        expiry_ns = int(self.max_age_s * 2.0 * 1_000_000_000)
        self._observations = {
            key: item
            for key, item in self._observations.items()
            if now_ns - item.received_ns <= expiry_ns
        }
        result = reach_consensus(
            self._observations.values(),
            now_ns=now_ns,
            max_age_s=self.max_age_s,
            min_confidence=self.min_confidence,
            min_independent_sources=self.min_independent_sources,
            max_spread_m=self.max_spread_m,
            allow_single_source=self.allow_single_source,
            single_source_confidence=self.single_source_confidence,
        )
        output = String()
        output.data = json.dumps(
            {
                "protocol": "eirax.semantic_consensus.v1",
                "peer_id": self.uav_id,
                "decision_id": str(uuid.uuid5(uuid.NAMESPACE_OID, repr(result))),
                **result.__dict__,
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self._consensus_pub.publish(output)
        if result.accepted:
            self._publish_pose(result, now_ns)

    def _publish_pose(self, result, now_ns: int) -> None:
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rclpy.time.Time(nanoseconds=now_ns).to_msg()
        frames = {
            item.frame_id
            for item in self._observations.values()
            if item.source_id in result.source_ids and item.frame_id
        }
        pose.header.frame_id = next(iter(frames)) if len(frames) == 1 else "map"
        pose.pose.pose.position.x = result.x
        pose.pose.pose.position.y = result.y
        pose.pose.pose.position.z = result.z
        pose.pose.pose.orientation.w = 1.0
        variance = max(0.05, result.spread_m or 0.0) ** 2
        pose.pose.covariance[0] = variance
        pose.pose.covariance[7] = variance
        pose.pose.covariance[14] = variance
        pose.pose.covariance[35] = 999.0  # Semantic position does not establish yaw.
        self._pose_pub.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticPeer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

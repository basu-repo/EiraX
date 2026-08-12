"""Serve UGV and PX4 UAV odometry snapshots to OMNeT++ over TCP."""

from __future__ import annotations

import math
import socketserver
import threading
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .network_protocol import PoseSample, build_pose_snapshot, validate_model_name


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, owner):
        self.owner = owner
        super().__init__(address, _Handler)


class _Handler(socketserver.StreamRequestHandler):
    def handle(self):
        while rclpy.ok():
            raw = self.rfile.readline()
            if not raw:
                return
            if raw.decode("ascii", errors="ignore").strip().upper() != "GET":
                self.wfile.write(b"ERR unsupported command\n")
                self.wfile.flush()
                continue
            self.wfile.write((self.server.owner.snapshot() + "\n").encode("ascii"))
            self.wfile.flush()


class MultiPoseBridge(Node):
    def __init__(self):
        super().__init__("multi_pose_bridge")
        model_names = [
            validate_model_name(value)
            for value in self.declare_parameter(
                "model_names", ["ugv", "uav"]
            ).value
        ]
        odom_topics = [
            str(value).strip()
            for value in self.declare_parameter(
                "odom_topics",
                ["/odom", "/uav/px4_odom"],
            ).value
        ]
        if len(model_names) != len(odom_topics) or not model_names:
            raise ValueError("model_names and odom_topics must be non-empty and equal length")
        if len(set(model_names)) != len(model_names):
            raise ValueError("model_names must be unique")
        if any(not topic.startswith("/") for topic in odom_topics):
            raise ValueError("every odom topic must be absolute")

        self.stale_timeout_s = float(self.declare_parameter("stale_timeout_s", 2.0).value)
        bind_host = str(self.declare_parameter("bind_host", "127.0.0.1").value)
        port = int(self.declare_parameter("port", 5555).value)
        self._samples: dict[str, PoseSample] = {}
        self._lock = threading.Lock()
        self._subscriptions = [
            self.create_subscription(
                Odometry, topic, self._callback_for(model_name), 10
            )
            for model_name, topic in zip(model_names, odom_topics)
        ]
        try:
            self._server = _Server((bind_host, port), self)
        except OSError as exc:
            raise RuntimeError(
                f"cannot bind OMNeT pose bridge to {bind_host}:{port}; "
                "stop the existing bridge or choose a different port"
            ) from exc
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()
        self.get_logger().info(
            f"OMNeT pose server {bind_host}:{port}; tracked={list(zip(model_names, odom_topics))}"
        )

    def _callback_for(self, model_name):
        def callback(message: Odometry):
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            qx, qy, qz, qw = (
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if norm <= 0.0:
                return
            qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            sample = PoseSample(
                model_name,
                float(position.x),
                float(position.y),
                float(position.z),
                yaw,
                time.monotonic(),
            )
            with self._lock:
                self._samples[model_name] = sample

        return callback

    def snapshot(self):
        with self._lock:
            samples = tuple(self._samples.values())
        return build_pose_snapshot(
            samples, now_monotonic=time.monotonic(), stale_timeout_s=self.stale_timeout_s
        )

    def destroy_node(self):
        self._server.shutdown()
        self._server.server_close()
        self._server_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiPoseBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

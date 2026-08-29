"""Receive independent OMNeT++ link metrics for every swarm UAV."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import socket
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .network_protocol import link_quality, parse_metrics_line, validate_model_name


@dataclass(frozen=True)
class Endpoint:
    uav_id: str
    host: str
    port: int


def parse_endpoint(value: str) -> Endpoint:
    parts = str(value).strip().split(":")
    if len(parts) == 2:
        uav_id, port_text = parts
        host = "127.0.0.1"
    elif len(parts) == 3:
        uav_id, host, port_text = parts
    else:
        raise ValueError("endpoint must be UAV:PORT or UAV:HOST:PORT")
    uav_id = validate_model_name(uav_id)
    host = host.strip()
    port = int(port_text)
    if not host or not 1 <= port <= 65535:
        raise ValueError(f"invalid endpoint: {value!r}")
    return Endpoint(uav_id, host, port)


class MultiMetricsBridge(Node):
    def __init__(self):
        super().__init__("multi_metrics_bridge")
        raw_endpoints = self.declare_parameter(
            "endpoints", ["uav:5556"]
        ).value
        self.endpoints = [parse_endpoint(value) for value in raw_endpoints]
        if len({item.uav_id for item in self.endpoints}) != len(self.endpoints):
            raise ValueError("endpoint UAV identifiers must be unique")
        self.reconnect_s = float(self.declare_parameter("reconnect_s", 1.0).value)
        self.read_timeout_s = float(self.declare_parameter("read_timeout_s", 3.0).value)
        shared_topic = str(
            self.declare_parameter("shared_topic", "/coord/swarm/network_metrics").value
        )
        self._shared_pub = self.create_publisher(String, shared_topic, 20)
        # Do not shadow rclpy.Node._publishers, which is an internal list.
        self._metric_publishers = {}
        for endpoint in self.endpoints:
            prefix = f"/coord/swarm/{endpoint.uav_id}/network"
            self._metric_publishers[endpoint.uav_id] = {
                name: self.create_publisher(Float64, f"{prefix}/{name}", 10)
                for name in (
                    "rssi_dbm",
                    "snir_db",
                    "packet_error_rate",
                    "packet_delivery_ratio",
                    "latency_s",
                    "jitter_s",
                    "radio_distance_m",
                    "quality",
                )
            }
        self._stop = threading.Event()
        self._threads = [
            threading.Thread(target=self._run_endpoint, args=(endpoint,), daemon=True)
            for endpoint in self.endpoints
        ]
        for thread in self._threads:
            thread.start()
        self.get_logger().info(f"OMNeT metrics endpoints={self.endpoints}")

    def _run_endpoint(self, endpoint: Endpoint):
        while not self._stop.is_set():
            try:
                with socket.create_connection(
                    (endpoint.host, endpoint.port), timeout=self.read_timeout_s
                ) as connection:
                    connection.settimeout(self.read_timeout_s)
                    with connection.makefile("r", encoding="ascii", errors="ignore") as stream:
                        for line in stream:
                            if self._stop.is_set():
                                return
                            try:
                                self._publish(endpoint.uav_id, parse_metrics_line(line))
                            except ValueError as exc:
                                self.get_logger().warn(
                                    f"invalid OMNeT metrics for {endpoint.uav_id}: {exc}",
                                    throttle_duration_sec=2.0,
                                )
            except (ConnectionError, OSError, TimeoutError) as exc:
                if not self._stop.is_set():
                    self.get_logger().debug(
                        f"OMNeT endpoint {endpoint.uav_id} unavailable: {exc}",
                        throttle_duration_sec=5.0,
                    )
            self._stop.wait(max(0.1, self.reconnect_s))

    def _publish(self, uav_id, metrics):
        quality = link_quality(metrics)
        values = {
            "rssi_dbm": metrics.rssi_dbm,
            "snir_db": metrics.snir_db,
            "packet_error_rate": metrics.packet_error_rate,
            "packet_delivery_ratio": metrics.packet_delivery_ratio,
            "latency_s": metrics.latency_s,
            "jitter_s": metrics.jitter_s,
            "radio_distance_m": metrics.radio_distance_m,
            "quality": quality,
        }
        for name, value in values.items():
            message = Float64()
            message.data = float(value)
            self._metric_publishers[uav_id][name].publish(message)
        payload = asdict(metrics)
        payload.update(
            protocol="eirax.network_metrics.v1",
            uav_id=uav_id,
            quality=quality,
            received_ros_ns=self.get_clock().now().nanoseconds,
        )
        # JSON permits NaN by default, but strict consumers do not. Use null.
        payload = {
            key: None if isinstance(value, float) and not math.isfinite(value) else value
            for key, value in payload.items()
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self._shared_pub.publish(message)

    def destroy_node(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=self.read_timeout_s + 0.5)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiMetricsBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

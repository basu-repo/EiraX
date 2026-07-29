"""Relay and measure the distance-limited UGV-UAV communication link.

The transmit and receive topics form the insertion boundary for a later
OMNeT++/Simu5G or security co-simulation. This deterministic baseline drops
vehicle-to-vehicle traffic outside its configured range but adds no artificial
delay, corruption, noise, or attack behavior.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time
from typing import Any

import rclpy
import yaml
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import NavSatFix, PointCloud2
from std_msgs.msg import Bool, Float32, String


@dataclass(frozen=True)
class Flow:
    name: str
    direction: str
    source_component: str
    destination_component: str
    source_ip: str
    destination_ip: str
    source_port: int
    destination_port: int
    message_type: type
    activity_type: str
    source_topic: str
    destination_topic: str
    queue_depth: int = 10


FLOWS = (
    Flow(
        "ugv_gnss",
        "UGV_TO_UAV",
        "ugv",
        "uav",
        "10.0.0.2",
        "10.0.0.3",
        5601,
        6601,
        NavSatFix,
        "telemetry",
        "/husky/gps",
        "/communication/uav/rx/ugv_gps",
    ),
    Flow(
        "ugv_odometry",
        "UGV_TO_UAV",
        "ugv",
        "uav",
        "10.0.0.2",
        "10.0.0.3",
        5602,
        6602,
        Odometry,
        "telemetry",
        "/odom",
        "/communication/uav/rx/ugv_odom",
    ),
    Flow(
        "ugv_global_path",
        "UGV_TO_UAV",
        "ugv",
        "uav",
        "10.0.0.2",
        "10.0.0.3",
        5603,
        6603,
        NavPath,
        "command",
        "/plan",
        "/cooperative/ugv_global_path",
    ),
    Flow(
        "uav_aerial_obstacles",
        "UAV_TO_UGV",
        "uav",
        "ugv",
        "10.0.0.3",
        "10.0.0.2",
        5701,
        6701,
        PointCloud2,
        "sensor_map",
        "/communication/uav/tx/aerial_obstacles",
        "/cooperative/aerial_obstacles",
        queue_depth=2,
    ),
    Flow(
        "uav_odometry",
        "UAV_TO_UGV",
        "uav",
        "ugv",
        "10.0.0.3",
        "10.0.0.2",
        5702,
        6702,
        Odometry,
        "telemetry",
        "/uav/px4_odom",
        "/communication/ugv/rx/uav_odom",
    ),
)


class CommunicationChannel(Node):
    def __init__(
        self,
        output_dir: Path,
        window_sec: float,
        config: dict[str, Any],
        uav_east_offset_m: float,
        uav_north_offset_m: float,
    ) -> None:
        super().__init__("ugv_uav_communication_channel")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.window_sec = window_sec
        self.config = config
        self.max_range_m = float(config["max_range_m"])
        self.warning_range_m = float(config["warning_range_m"])
        self.reconnect_range_m = float(config["reconnect_range_m"])
        self.uav_east_offset_m = uav_east_offset_m
        self.uav_north_offset_m = uav_north_offset_m
        self.ugv_xy: tuple[float, float] | None = None
        # PX4 local position starts at zero at the configured launch pad.
        self.uav_xy = (uav_east_offset_m, uav_north_offset_m)
        self.distance_m: float | None = None
        self.maximum_observed_distance_m = 0.0
        self.link_active = True
        self.link_state = "starting"
        self.started_monotonic_ns = time.monotonic_ns()
        self.started_utc = datetime.now(timezone.utc)
        self.sequence = {flow.name: 0 for flow in FLOWS}
        self.latencies_ms: dict[str, list[float]] = {
            flow.name: [] for flow in FLOWS
        }
        self.flow_totals: dict[str, dict[str, int]] = {
            flow.name: {
                "packets": 0,
                "bytes": 0,
                "delivered": 0,
                "dropped": 0,
                "delivered_bytes": 0,
            }
            for flow in FLOWS
        }
        self.flow_first_seen_sec: dict[str, float] = {}
        self.windows: dict[tuple[str, int], dict[str, Any]] = {}
        self.events_path = self.output_dir / "communication_events.csv"
        self.windows_path = self.output_dir / "flow_windows.csv"
        self.summary_path = self.output_dir / "communication_summary.json"
        self.contract_path = self.output_dir / "channel_contract.json"
        self._events = self.events_path.open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self._event_writer = csv.writer(self._events)
        self._event_writer.writerow(
            [
                "timestamp",
                "elapsed_sec",
                "sequence",
                "direction",
                "source_component",
                "destination_component",
                "source_ip",
                "destination_ip",
                "source_port",
                "destination_port",
                "protocol",
                "session_id",
                "message_type",
                "activity_type",
                "source_topic",
                "destination_topic",
                "payload_bytes",
                "tx_monotonic_ns",
                "rx_monotonic_ns",
                "latency_ms",
                "delivered",
                "link_active",
                "distance_m",
                "link_state",
                "normalized_link_quality",
            ]
        )

        self._flow_publishers: dict[str, Any] = {}
        self._flow_subscriptions = []
        for flow in FLOWS:
            publisher = self.create_publisher(
                flow.message_type, flow.destination_topic, flow.queue_depth
            )
            self._flow_publishers[flow.name] = publisher
            self._flow_subscriptions.append(
                self.create_subscription(
                    flow.message_type,
                    flow.source_topic,
                    self._callback(flow, publisher),
                    flow.queue_depth,
                )
            )

        self.link_publisher = self.create_publisher(
            Bool, "/communication/link/status", 10
        )
        self.distance_publisher = self.create_publisher(
            Float32, "/communication/link/distance_m", 10
        )
        self.quality_publisher = self.create_publisher(
            Float32, "/communication/link/quality", 10
        )
        self.state_publisher = self.create_publisher(
            String, "/communication/link/state", 10
        )
        self.status_timer = self.create_timer(
            1.0 / float(config["status_rate_hz"]), self._publish_link_status
        )
        self._write_contract()
        self.get_logger().info(
            f"Distance-limited UGV-UAV channel active: warning at "
            f"{self.warning_range_m:.1f} m, disconnect at {self.max_range_m:.1f} m"
        )

    def _update_link_state(self) -> None:
        if self.ugv_xy is None:
            self.link_state = "starting"
            return
        self.distance_m = math.hypot(
            self.uav_xy[0] - self.ugv_xy[0],
            self.uav_xy[1] - self.ugv_xy[1],
        )
        self.maximum_observed_distance_m = max(
            self.maximum_observed_distance_m, self.distance_m
        )
        if self.link_active and self.distance_m > self.max_range_m:
            self.link_active = False
        elif not self.link_active and self.distance_m <= self.reconnect_range_m:
            self.link_active = True
        if not self.link_active:
            self.link_state = "disconnected"
        elif self.distance_m >= self.warning_range_m:
            self.link_state = "warning"
        else:
            self.link_state = "connected"

    def _callback(self, flow: Flow, publisher: Any):
        def relay(message: Any) -> None:
            tx_ns = time.monotonic_ns()
            payload_bytes = len(serialize_message(message))
            if flow.name == "ugv_odometry":
                self.ugv_xy = (
                    float(message.pose.pose.position.x),
                    float(message.pose.pose.position.y),
                )
            elif flow.name == "uav_odometry":
                self.uav_xy = (
                    self.uav_east_offset_m
                    + float(message.pose.pose.position.x),
                    self.uav_north_offset_m
                    + float(message.pose.pose.position.y),
                )
            self._update_link_state()
            delivered = self.link_active
            if delivered:
                publisher.publish(message)
            rx_ns = time.monotonic_ns()
            latency_ms = (rx_ns - tx_ns) / 1_000_000.0
            elapsed_sec = (
                tx_ns - self.started_monotonic_ns
            ) / 1_000_000_000.0
            timestamp = datetime.now(timezone.utc).isoformat()
            self.sequence[flow.name] += 1
            sequence = self.sequence[flow.name]
            session_id = f"eirax-{flow.source_component}-{flow.destination_component}"
            normalized_quality = (
                max(0.0, min(1.0, 1.0 - self.distance_m / self.max_range_m))
                if self.distance_m is not None
                else 0.0
            )
            self._event_writer.writerow(
                [
                    timestamp,
                    f"{elapsed_sec:.6f}",
                    sequence,
                    flow.direction,
                    flow.source_component,
                    flow.destination_component,
                    flow.source_ip,
                    flow.destination_ip,
                    flow.source_port,
                    flow.destination_port,
                    "UDP",
                    session_id,
                    flow.name,
                    flow.activity_type,
                    flow.source_topic,
                    flow.destination_topic,
                    payload_bytes,
                    tx_ns,
                    rx_ns,
                    f"{latency_ms:.6f}",
                    int(delivered),
                    int(self.link_active),
                    (
                        f"{self.distance_m:.6f}"
                        if self.distance_m is not None
                        else ""
                    ),
                    self.link_state,
                    f"{normalized_quality:.6f}",
                ]
            )
            self.flow_totals[flow.name]["packets"] += 1
            self.flow_totals[flow.name]["bytes"] += payload_bytes
            if delivered:
                self.flow_totals[flow.name]["delivered"] += 1
                self.flow_totals[flow.name]["delivered_bytes"] += payload_bytes
                self.latencies_ms[flow.name].append(latency_ms)
            else:
                self.flow_totals[flow.name]["dropped"] += 1
            self.flow_first_seen_sec.setdefault(flow.name, elapsed_sec)
            window_index = int(elapsed_sec // self.window_sec)
            bucket = self.windows.setdefault(
                (flow.name, window_index),
                {
                    "flow": flow,
                    "packet_count": 0,
                    "byte_count": 0,
                    "delivered_count": 0,
                    "latencies_ms": [],
                },
            )
            bucket["packet_count"] += 1
            bucket["byte_count"] += payload_bytes
            if delivered:
                bucket["delivered_count"] += 1
                bucket["latencies_ms"].append(latency_ms)

        return relay

    def _publish_link_status(self) -> None:
        self._update_link_state()
        message = Bool()
        message.data = self.link_active
        self.link_publisher.publish(message)
        distance = Float32()
        distance.data = (
            float(self.distance_m) if self.distance_m is not None else math.nan
        )
        self.distance_publisher.publish(distance)
        quality = Float32()
        quality.data = (
            max(0.0, min(1.0, 1.0 - self.distance_m / self.max_range_m))
            if self.distance_m is not None
            else 0.0
        )
        self.quality_publisher.publish(quality)
        state = String()
        state.data = self.link_state
        self.state_publisher.publish(state)

    def _write_contract(self) -> None:
        contract = {
            "version": 1,
            "mode": "distance_limited",
            "protocol": "UDP",
            "addressing": "logical_simulation_identifiers",
            "intentional_delay_ms": 0.0,
            "range_model": self.config["range_model"],
            "max_range_m": self.max_range_m,
            "warning_range_m": self.warning_range_m,
            "reconnect_range_m": self.reconnect_range_m,
            "desired_lead_m": float(self.config["desired_lead_m"]),
            "carrier_frequency_mhz": float(
                self.config["carrier_frequency_mhz"]
            ),
            "corruption_rate": 0.0,
            "noise_enabled": False,
            "aggregation_window_sec": self.window_sec,
            "simu5g_insertion_boundary": {
                "transmit_side": [flow.source_topic for flow in FLOWS],
                "receive_side": [flow.destination_topic for flow in FLOWS],
            },
            "flows": [
                {
                    key: getattr(flow, key)
                    for key in (
                        "name",
                        "direction",
                        "source_component",
                        "destination_component",
                        "source_ip",
                        "destination_ip",
                        "source_port",
                        "destination_port",
                        "activity_type",
                        "source_topic",
                        "destination_topic",
                    )
                }
                for flow in FLOWS
            ],
        }
        self.contract_path.write_text(
            json.dumps(contract, indent=2) + "\n", encoding="utf-8"
        )

    def _write_windows(self) -> None:
        with self.windows_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "timestamp",
                    "window_start_sec",
                    "window_end_sec",
                    "source_component",
                    "destination_component",
                    "source_ip",
                    "destination_ip",
                    "source_port",
                    "destination_port",
                    "protocol",
                    "session_id",
                    "message_type",
                    "activity_type",
                    "packet_count",
                    "byte_count",
                    "packets_per_second",
                    "throughput_kbps",
                    "latency_ms",
                    "jitter_ms",
                    "packet_loss_rate",
                    "retransmission_rate",
                    "connection_duration_sec",
                    "command_channel_activity",
                    "video_stream_activity",
                    "telemetry_stream_activity",
                ]
            )
            for (flow_name, window_index), bucket in sorted(
                self.windows.items(), key=lambda item: (item[0][1], item[0][0])
            ):
                flow: Flow = bucket["flow"]
                latencies = bucket["latencies_ms"]
                latency_ms = statistics.fmean(latencies) if latencies else 0.0
                jitter_ms = (
                    statistics.fmean(
                        abs(current - previous)
                        for previous, current in zip(latencies, latencies[1:])
                    )
                    if len(latencies) > 1
                    else 0.0
                )
                count = bucket["packet_count"]
                delivered_count = bucket["delivered_count"]
                byte_count = bucket["byte_count"]
                start_sec = window_index * self.window_sec
                end_sec = start_sec + self.window_sec
                connection_duration_sec = max(
                    0.0,
                    end_sec - self.flow_first_seen_sec.get(flow_name, end_sec),
                )
                timestamp = (
                    self.started_utc.timestamp() + start_sec
                )
                writer.writerow(
                    [
                        datetime.fromtimestamp(timestamp, timezone.utc).isoformat(),
                        f"{start_sec:.3f}",
                        f"{end_sec:.3f}",
                        flow.source_component,
                        flow.destination_component,
                        flow.source_ip,
                        flow.destination_ip,
                        flow.source_port,
                        flow.destination_port,
                        "UDP",
                        f"eirax-{flow.source_component}-{flow.destination_component}",
                        flow_name,
                        flow.activity_type,
                        count,
                        byte_count,
                        f"{count / self.window_sec:.6f}",
                        f"{byte_count * 8.0 / 1000.0 / self.window_sec:.6f}",
                        f"{latency_ms:.6f}",
                        f"{jitter_ms:.6f}",
                        f"{(count - delivered_count) / count:.6f}",
                        "0.0",
                        f"{connection_duration_sec:.3f}",
                        count if flow.activity_type == "command" else 0,
                        0,
                        count if flow.activity_type == "telemetry" else 0,
                    ]
                )

    def _write_summary(self) -> None:
        duration_sec = (
            time.monotonic_ns() - self.started_monotonic_ns
        ) / 1_000_000_000.0
        flows: dict[str, Any] = {}
        total_packets = 0
        total_bytes = 0
        total_delivered = 0
        total_dropped = 0
        for flow in FLOWS:
            totals = self.flow_totals[flow.name]
            latencies = self.latencies_ms[flow.name]
            total_packets += totals["packets"]
            total_bytes += totals["bytes"]
            total_delivered += totals["delivered"]
            total_dropped += totals["dropped"]
            flows[flow.name] = {
                "direction": flow.direction,
                "packet_count": totals["packets"],
                "byte_count": totals["bytes"],
                "delivered_packet_count": totals["delivered"],
                "dropped_packet_count": totals["dropped"],
                "delivered_byte_count": totals["delivered_bytes"],
                "delivery_ratio": (
                    totals["delivered"] / totals["packets"]
                    if totals["packets"]
                    else None
                ),
                "packet_loss_rate": (
                    totals["dropped"] / totals["packets"]
                    if totals["packets"]
                    else None
                ),
                "mean_latency_ms": (
                    statistics.fmean(latencies) if latencies else None
                ),
                "maximum_latency_ms": max(latencies) if latencies else None,
                "mean_packets_per_second": (
                    totals["packets"] / duration_sec if duration_sec > 0 else 0.0
                ),
                "mean_throughput_kbps": (
                    totals["bytes"] * 8.0 / 1000.0 / duration_sec
                    if duration_sec > 0
                    else 0.0
                ),
            }
        summary = {
            "mode": "distance_limited",
            "started_at": self.started_utc.isoformat(),
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "duration_sec": duration_sec,
            "maximum_range_m": self.max_range_m,
            "maximum_observed_distance_m": self.maximum_observed_distance_m,
            "warning_range_m": self.warning_range_m,
            "reconnect_range_m": self.reconnect_range_m,
            "link_uptime_ratio": (
                total_delivered / total_packets if total_packets else None
            ),
            "total_packets": total_packets,
            "total_bytes": total_bytes,
            "total_delivered_packets": total_delivered,
            "total_dropped_packets": total_dropped,
            "delivery_ratio": (
                total_delivered / total_packets if total_packets else None
            ),
            "flows": flows,
            "radio_metrics": {
                "signal_quality_dbm": None,
                "handover_event": None,
                "handover_count": None,
                "source": "reserved_for_Simu5G",
            },
            "security_metrics": {
                "attack_type": None,
                "incident_label": None,
                "source": "reserved_for_scenario_manifest",
            },
        }
        self.summary_path.write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )

    def close(self) -> None:
        if self._events.closed:
            return
        self._write_windows()
        self._write_summary()
        self._events.flush()
        self._events.close()

    def destroy_node(self) -> None:
        self.close()
        super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--uav-east-offset", type=float, required=True)
    parser.add_argument("--uav-north-offset", type=float, required=True)
    parser.add_argument("--window-sec", type=float, default=1.0)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.window_sec) or args.window_sec <= 0.0:
        parser.error("--window-sec must be a positive finite value")
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    maximum = float(config["max_range_m"])
    warning = float(config["warning_range_m"])
    reconnect = float(config["reconnect_range_m"])
    if not (0.0 < warning < reconnect < maximum):
        parser.error(
            "link ranges must satisfy 0 < warning < reconnect < maximum"
        )
    rclpy.init(args=ros_args)
    node = CommunicationChannel(
        args.output_dir,
        args.window_sec,
        config,
        args.uav_east_offset,
        args.uav_north_offset,
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

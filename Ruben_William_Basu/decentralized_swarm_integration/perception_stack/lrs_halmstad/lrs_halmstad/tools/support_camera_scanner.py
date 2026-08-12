#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, String


def _coerce_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _parse_float_list(value) -> list[float]:
    raw = str(value).strip()
    if not raw:
        return []
    parsed: list[float] = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        parsed.append(float(item))
    return parsed


def _value_for_index(values: list[float], index: int, default: float) -> float:
    if not values:
        return default
    if index < len(values):
        return values[index]
    return values[-1]


class SupportCameraScanner(Node):
    """Small optional pan/tilt sweep for support-UAV cameras."""

    def __init__(self) -> None:
        super().__init__("support_camera_scanner")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        uav_names_raw = str(self.declare_parameter("uav_names", "dji1,dji2").value)
        self.uav_names = [name.strip() for name in uav_names_raw.split(",") if name.strip()]
        if not self.uav_names:
            self.uav_names = ["dji1", "dji2"]

        self.yaw_center_deg = float(self.declare_parameter("yaw_center_deg", 0.0, dyn_num).value)
        self.yaw_amplitude_deg = max(
            0.0, float(self.declare_parameter("yaw_amplitude_deg", 35.0, dyn_num).value)
        )
        self.period_s = max(0.5, float(self.declare_parameter("period_s", 8.0, dyn_num).value))
        self.pitch_deg = float(self.declare_parameter("pitch_deg", -20.0, dyn_num).value)
        self.pitch_amplitude_deg = max(
            0.0, float(self.declare_parameter("pitch_amplitude_deg", 0.0, dyn_num).value)
        )
        pitch_period_value = float(self.declare_parameter("pitch_period_s", 0.0, dyn_num).value)
        self.pitch_period_s = self.period_s if pitch_period_value <= 0.0 else max(0.5, pitch_period_value)
        self.rate_hz = max(0.5, float(self.declare_parameter("rate_hz", 10.0, dyn_num).value))
        self.publish_pitch = _coerce_bool(self.declare_parameter("publish_pitch", True).value)
        self.pan_phase_offsets_deg = _parse_float_list(
            self.declare_parameter("pan_phase_offsets_deg", "").value
        )
        self.pitch_phase_offsets_deg = _parse_float_list(
            self.declare_parameter("pitch_phase_offsets_deg", "").value
        )
        self.status_topic = str(
            self.declare_parameter("status_topic", "/coord/support/camera_scan_status").value
        ).strip() or "/coord/support/camera_scan_status"

        self._pan_pubs = {
            name: self.create_publisher(Float64, f"/{name}/update_pan", 10)
            for name in self.uav_names
        }
        self._tilt_pubs = {
            name: self.create_publisher(Float64, f"/{name}/update_tilt", 10)
            for name in self.uav_names
        }
        self._status_pub = self.create_publisher(String, self.status_topic, 10)
        self._timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)

        self.get_logger().info(
            "[support_camera_scanner] Started: "
            f"uavs={','.join(self.uav_names)}, yaw_center_deg={self.yaw_center_deg:.1f}, "
            f"yaw_amplitude_deg={self.yaw_amplitude_deg:.1f}, period_s={self.period_s:.1f}, "
            f"pitch_deg={self.pitch_deg:.1f}, pitch_amplitude_deg={self.pitch_amplitude_deg:.1f}, "
            f"pitch_period_s={self.pitch_period_s:.1f}, rate_hz={self.rate_hz:.1f}, "
            f"pan_phase_offsets_deg={self.pan_phase_offsets_deg or [0.0]}, "
            f"pitch_phase_offsets_deg={self.pitch_phase_offsets_deg or [0.0]}"
        )

    @staticmethod
    def _wave(center_deg: float, amplitude_deg: float, period_s: float, phase_deg: float, now_s: float) -> float:
        if amplitude_deg <= 0.0:
            return center_deg
        return center_deg + amplitude_deg * math.sin(
            (2.0 * math.pi * now_s) / period_s + math.radians(phase_deg)
        )

    def _on_timer(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        status_entries: list[str] = []

        for index, name in enumerate(self.uav_names):
            pan_deg = self._wave(
                self.yaw_center_deg,
                self.yaw_amplitude_deg,
                self.period_s,
                _value_for_index(self.pan_phase_offsets_deg, index, 0.0),
                now_s,
            )
            pitch_deg = self._wave(
                self.pitch_deg,
                self.pitch_amplitude_deg,
                self.pitch_period_s,
                _value_for_index(self.pitch_phase_offsets_deg, index, 0.0),
                now_s,
            )

            pan_msg = Float64()
            pan_msg.data = pan_deg
            self._pan_pubs[name].publish(pan_msg)
            if self.publish_pitch:
                tilt_msg = Float64()
                tilt_msg.data = pitch_deg
                self._tilt_pubs[name].publish(tilt_msg)
            status_entries.append(f"{name}:{pan_deg:.1f}/{pitch_deg:.1f}")

        status = String()
        status.data = (
            "task=support_camera_scan "
            f"state=OK uavs={','.join(self.uav_names)} yaw_center_deg={self.yaw_center_deg:.2f} "
            f"yaw_amplitude_deg={self.yaw_amplitude_deg:.2f} period_s={self.period_s:.2f} "
            f"pitch_deg={self.pitch_deg:.2f} pitch_amplitude_deg={self.pitch_amplitude_deg:.2f} "
            f"pitch_period_s={self.pitch_period_s:.2f} samples={';'.join(status_entries)}"
        )
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupportCameraScanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

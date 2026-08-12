"""Pure wire protocols shared by the ROS/OMNeT++ integration nodes."""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Iterable


MODEL_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_-]{0,63}$")


@dataclass(frozen=True)
class PoseSample:
    model_name: str
    x: float
    y: float
    z: float
    yaw: float
    updated_monotonic: float


@dataclass(frozen=True)
class LinkMetrics:
    sim_time_s: float
    rssi_dbm: float
    snir_db: float
    packet_error_rate: float
    radio_distance_m: float
    packet_delivery_ratio: float
    latency_s: float
    jitter_s: float


def validate_model_name(name: str) -> str:
    normalized = str(name).strip()
    if not MODEL_NAME_RE.fullmatch(normalized):
        raise ValueError(f"invalid OMNeT model name: {name!r}")
    return normalized


def build_pose_snapshot(
    samples: Iterable[PoseSample], *, now_monotonic: float, stale_timeout_s: float
) -> str:
    fresh = []
    for sample in samples:
        validate_model_name(sample.model_name)
        values = (sample.x, sample.y, sample.z, sample.yaw)
        if not all(math.isfinite(value) for value in values):
            continue
        age = now_monotonic - sample.updated_monotonic
        if age < 0:
            continue
        if stale_timeout_s > 0.0 and age > stale_timeout_s:
            continue
        fresh.append(sample)
    fresh.sort(key=lambda item: item.model_name)
    parts = [str(len(fresh))]
    for sample in fresh:
        parts.extend(
            (
                sample.model_name,
                f"{sample.x:.6f}",
                f"{sample.y:.6f}",
                f"{sample.z:.6f}",
                f"{sample.yaw:.6f}",
            )
        )
    return " ".join(parts)


def parse_metrics_line(line: str) -> LinkMetrics:
    """Parse the current eight-field or historical six-field bridge line.

    Current `OmnetMetricsServer.cc` emits:
      sim_time rssi snir per radio_distance pdr latency jitter

    The old Python bridge documented and expected:
      sim_time geometric_distance rssi snir per radio_distance

    Geometric distance is intentionally discarded because navigation must not
    treat OMNeT's ground-truth positions as an observational input.
    """
    fields = line.strip().split()
    if len(fields) not in (6, 8):
        raise ValueError(f"expected 6 or 8 metrics fields, got {len(fields)}")
    try:
        values = [float(field) for field in fields]
    except ValueError as exc:
        raise ValueError("metrics line contains a non-numeric field") from exc
    if len(values) == 8:
        metrics = LinkMetrics(*values)
    else:
        sim_time, _geometric_distance, rssi, snir, per, radio_distance = values
        metrics = LinkMetrics(
            sim_time,
            rssi,
            snir,
            per,
            radio_distance,
            math.nan,
            math.nan,
            math.nan,
        )
    if metrics.sim_time_s < 0.0:
        raise ValueError("sim_time must be non-negative")
    for name, value in (
        ("packet_error_rate", metrics.packet_error_rate),
        ("packet_delivery_ratio", metrics.packet_delivery_ratio),
    ):
        if math.isfinite(value) and not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} must be within [0, 1]")
    if math.isfinite(metrics.latency_s) and metrics.latency_s < 0.0:
        raise ValueError("latency must be non-negative")
    if math.isfinite(metrics.jitter_s) and metrics.jitter_s < 0.0:
        raise ValueError("jitter must be non-negative")
    return metrics


def link_quality(metrics: LinkMetrics) -> float:
    """Return a conservative normalized quality without using pose distance."""
    components = []
    if math.isfinite(metrics.packet_delivery_ratio):
        components.append(max(0.0, min(1.0, metrics.packet_delivery_ratio)))
    if math.isfinite(metrics.packet_error_rate):
        components.append(max(0.0, min(1.0, 1.0 - metrics.packet_error_rate)))
    if math.isfinite(metrics.snir_db):
        components.append(max(0.0, min(1.0, (metrics.snir_db + 10.0) / 30.0)))
    if not components:
        return 0.0
    return min(components)


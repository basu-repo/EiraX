import math

import pytest

from decentralized_swarm_integration.network_protocol import (
    PoseSample,
    build_pose_snapshot,
    link_quality,
    parse_metrics_line,
    validate_model_name,
)


def test_pose_snapshot_is_sorted_and_stale_samples_are_removed():
    line = build_pose_snapshot(
        [
            PoseSample("dji1", 2.0, 3.0, 4.0, 0.2, 9.5),
            PoseSample("ugv", 0.0, 1.0, 0.0, 0.0, 7.0),
            PoseSample("dji0", 1.0, 2.0, 3.0, 0.1, 9.0),
        ],
        now_monotonic=10.0,
        stale_timeout_s=2.0,
    )
    assert line == (
        "2 dji0 1.000000 2.000000 3.000000 0.100000 "
        "dji1 2.000000 3.000000 4.000000 0.200000"
    )


def test_current_eight_field_metrics_protocol():
    result = parse_metrics_line("12.0 -88.0 7.0 0.1 42.0 0.9 0.04 0.01")
    assert result.rssi_dbm == -88.0
    assert result.packet_delivery_ratio == 0.9
    assert result.latency_s == 0.04


def test_legacy_six_field_metrics_protocol_discards_ground_truth_distance():
    result = parse_metrics_line("12.0 123.0 -88.0 7.0 0.1 42.0")
    assert result.rssi_dbm == -88.0
    assert result.radio_distance_m == 42.0
    assert math.isnan(result.packet_delivery_ratio)


def test_invalid_probability_is_rejected():
    with pytest.raises(ValueError, match="packet_error_rate"):
        parse_metrics_line("1 -80 5 1.5 20 0.5 0.1 0.0")


def test_link_quality_uses_worst_available_observational_metric():
    metrics = parse_metrics_line("1 -80 5 0.2 20 0.9 0.1 0.0")
    assert link_quality(metrics) == 0.5


@pytest.mark.parametrize("name", ["bad name", "../uav", "", "x" * 65])
def test_model_name_rejects_protocol_injection(name):
    with pytest.raises(ValueError):
        validate_model_name(name)


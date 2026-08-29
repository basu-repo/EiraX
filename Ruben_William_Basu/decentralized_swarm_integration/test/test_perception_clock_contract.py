from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "perception_stack/lrs_halmstad"))

from lrs_halmstad.perception.runtime_metrics import FrameTiming


def timing(source_ns: int, received_ns: int, published_ns: int) -> FrameTiming:
    item = FrameTiming(
        source_stamp_ns=source_ns,
        frame_recv_ros_ns=received_ns,
        frame_recv_perf_ns=1_000_000_000,
        backend="test",
    )
    item.publish_ros_ns = published_ns
    item.publish_perf_ns = 1_200_000_000
    return item


def test_sim_time_camera_stamp_uses_wall_receive_epoch_for_latency():
    item = timing(38_000_000_000, 1_786_466_100_000_000_000, 1_786_466_100_150_000_000)
    assert item.source_clock_compatible is False
    assert item.camera_to_publish_latency_ms == 150.0
    assert item.metadata()["stale_detection"] is False


def test_matching_clock_domain_preserves_camera_to_publish_latency():
    item = timing(100_000_000_000, 100_050_000_000, 100_200_000_000)
    assert item.source_clock_compatible is True
    assert item.camera_to_publish_latency_ms == 200.0


def test_launch_uses_estimator_camera_parameter_names_and_pitch_convention():
    launch = (PROJECT_ROOT / "launch/full_swarm.launch.py").read_text(encoding="utf-8")
    assert "cam_x_offset_m=0.153" in launch
    assert "cam_y_offset_m=0.0" in launch
    assert "cam_z_offset_m=-0.043" in launch
    assert "cam_pitch_offset_deg=-45.0" in launch
    assert "camera_mount_pitch_deg" not in launch

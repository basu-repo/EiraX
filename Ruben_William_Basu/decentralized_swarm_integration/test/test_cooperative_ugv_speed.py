from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_cooperative_forward_speed_is_increased_without_raising_reverse_cap():
    config = yaml.safe_load(
        (ROOT / "vehicle_stack/config/cooperative_navigation.yaml").read_text()
    )
    assert config["forward_motion"]["maximum_speed_mps"] == 1.25
    assert config["reverse_motion"]["enabled"] is False
    assert config["reverse_motion"]["maximum_speed_mps"] == 0.25


def test_generated_nav2_config_applies_speed_to_controller_and_smoother():
    source = (ROOT / "vehicle_stack/cooperative_nav2_config.py").read_text()
    assert 'controller["max_vel_x"] = maximum_forward_speed' in source
    assert 'controller["max_speed_xy"] = maximum_forward_speed' in source
    assert 'velocity_smoother["max_velocity"][0] = maximum_forward_speed' in source
    assert 'controller["min_vel_x"] = minimum_forward_velocity' in source
    assert 'velocity_smoother["min_velocity"][0] = -recovery_reverse_speed' in source


def test_reverse_is_bounded_to_explicit_recovery_configuration():
    config = yaml.safe_load(
        (ROOT / "vehicle_stack/config/cooperative_navigation.yaml").read_text()
    )
    assert config["reverse_motion"]["enabled"] is False
    assert config["reverse_recovery"] == {
        "enabled": True,
        "distance_m": 1.5,
        "speed_mps": 0.15,
        "time_allowance_sec": 15,
        "lidar_freshness_sec": 1.0,
        "lidar_wait_timeout_sec": 10.0,
    }


def test_collision_monitor_uses_fresh_raw_lidar_not_lagging_slam_cloud():
    source = (ROOT / "vehicle_stack/cooperative_nav2_config.py").read_text()
    assert 'collision["lidar3d"]["topic"] = "/husky/lidar3d/points"' in source

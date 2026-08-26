"""Contract tests for the isolated accepted-baseline entry point."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/run_baylands_swarm.py"
SPEC = spec_from_file_location("run_baylands_swarm", SCRIPT)
launcher = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(launcher)


def test_entry_point_uses_only_the_local_baseline_copy():
    assert launcher.BASELINE_LAUNCHER.is_file()
    assert launcher.PROJECT_ROOT in launcher.BASELINE_LAUNCHER.parents
    assert launcher.command(["--no-motion"]) == [
        sys.executable,
        str(launcher.BASELINE_LAUNCHER),
        "--no-motion",
    ]


def test_local_baseline_uses_shared_isolated_px4_runtime():
    source = launcher.BASELINE_LAUNCHER.read_text(encoding="utf-8")
    assert 'PX4_RUNTIME = PROJECT_ROOT / "px4_runtime"' in source
    assert (launcher.PROJECT_ROOT / "px4_runtime/bin/px4").is_file()


def test_local_world_is_portable_and_uses_the_cooperative_husky():
    world_path = launcher.PROJECT_ROOT / "simulation/worlds/baylands_editable.world"
    world = world_path.read_text(encoding="utf-8")
    assert "file:///home/" not in world
    assert "model://husky_cooperative" in world
    assert (launcher.PROJECT_ROOT / "simulation/models/baylands").is_dir()
    assert (launcher.PROJECT_ROOT / "simulation/models/x500_mapping").is_dir()


def test_three_uav_runtime_has_independent_controllers_and_level_peer_pads():
    runner = launcher.BASELINE_LAUNCHER.read_text(encoding="utf-8")
    world_builder = (
        launcher.PROJECT_ROOT / "vehicle_stack/uav/cooperative_world.py"
    ).read_text(encoding="utf-8")
    site_config = (
        launcher.PROJECT_ROOT / "vehicle_stack/config/swarm_launch_site.yaml"
    ).read_text(encoding="utf-8")
    assert '"--port", "14540"' in runner
    assert "peer_slots = ((1, 14541" in runner
    assert "(2, 14542" in runner
    assert '"swarm_launch_deck"' in world_builder
    assert "swarm_launch_site.yaml" in world_builder
    assert "size_x: 8.0" in site_config
    assert "size_y: 12.0" in site_config
    assert "thickness: 0.20" in site_config
    assert site_config.count("offset_y:") == 3


def test_each_x500_instance_has_a_scoped_low_load_rgbd_yolo_camera():
    model = (
        launcher.PROJECT_ROOT / "simulation/models/x500_mapping_base/model.sdf"
    ).read_text(encoding="utf-8")
    launch = (
        launcher.PROJECT_ROOT / "launch/full_swarm.launch.py"
    ).read_text(encoding="utf-8")
    assert '<sensor name="uav_rgbd" type="rgbd_camera">' in model
    assert 'visual name="eirax_rgbd_body_bracket"' in model
    assert 'visual name="eirax_rgbd_pitch_pivot"' in model
    assert 'visual name="eirax_rgbd_pivot_arm"' in model
    assert "<width>416</width><height>256</height>" in model
    assert model.count("<pose>0.153 0 -0.043 0 0.785398 0</pose>") == 2
    assert "<update_rate>2</update_rate>" in model
    assert "model/x500_mapping_{index}" in launch
    assert 'if start_yolo:' in launch
    assert "predict_hz=0.5" in launch
    assert 'estimator["est_hz"] = 2.0' in launch
    assert "cam_x_offset_m=0.153" in launch
    assert "cam_pitch_offset_deg=-45.0" in launch
    assert "cam_z_offset_m=-0.043" in launch
    assert "evidence_positive_interval_s=2.0" in launch
    assert "evidence_negative_interval_s=20.0" in launch
    assert "evidence_max_positive=500" in launch
    assert 'yolo_evidence_root:={active_run / \'yolo_frames\'' in (
        launcher.PROJECT_ROOT / "scripts/run_everything.py"
    ).read_text(encoding="utf-8")


def test_camera_viewer_can_switch_between_all_three_bridged_streams():
    everything = (
        launcher.PROJECT_ROOT / "scripts/run_everything.py"
    ).read_text(encoding="utf-8")
    assert '"rqt_image_view", "rqt_image_view"' in everything
    assert 'f"[CAMERA]   /swarm/uav{index}/camera/image_raw"' in everything
    assert '"/swarm/uav0/camera/image_raw",' not in everything


def test_cooperative_ugv_keeps_an_explicit_obstacle_clearance_envelope():
    config = (
        launcher.PROJECT_ROOT / "vehicle_stack/cooperative_nav2_config.py"
    ).read_text(encoding="utf-8")
    assert 'inflation["inflation_radius"] = 1.20' in config
    assert 'inflation["cost_scaling_factor"] = 1.50' in config
    assert 'controller["BaseObstacle.scale"] = 0.15' in config
    assert 'collision["source_timeout"] = 3.0' in config
    assert 'collision["FootprintApproach"]["time_before_collision"] = 2.0' in config


def test_isolated_ugv_speed_limit_is_one_metre_per_second():
    config = (launcher.PROJECT_ROOT / "ground_stack/baseline/config/nav2_config.py").read_text(encoding="utf-8")
    assert '"max_vel_x": 1.00' in config
    assert '"max_speed_xy": 1.00' in config
    assert 'max_velocity"] = [1.00, 0.0, 0.60]' in config


def test_default_complete_launcher_runs_all_legs_with_safe_return():
    everything = (launcher.PROJECT_ROOT / "scripts/run_everything.py").read_text(
        encoding="utf-8"
    )
    follower = (
        launcher.PROJECT_ROOT / "vehicle_stack/uav/follow_husky.py"
    ).read_text(encoding="utf-8")
    assert '"--uav-test-waypoint-1"' not in everything
    assert "All waypoints, the goal, and all UAV" in everything
    assert '"use_sim_time:=false"' in everything
    assert "landing_north = 0.0" in follower
    assert "landing_east = 0.0" in follower
    assert "landing-delay-sec" in follower
    assert "command_and_wait(connection, mavutil.mavlink.MAV_CMD_NAV_LAND)" not in follower
    assert "landing_horizontal_error <= 0.60" in follower
    assert "[UAV LANDED VERIFIED]" in follower


def test_recording_includes_geometry_and_failover_diagnostics_for_every_uav():
    runner = (
        launcher.PROJECT_ROOT / "vehicle_stack/run_cooperative_simulation.py"
    ).read_text(encoding="utf-8")
    assert 'f"/swarm/{uav_id}/ground_truth/pose"' in runner
    assert 'f"/coord/support/{uav_id}/leader_estimate_status"' in runner
    assert 'f"/coord/support/{uav_id}/leader_estimate_fault"' in runner
    assert 'f"/coord/swarm/{uav_id}/role"' in runner
    assert 'f"/coord/swarm/{uav_id}/network/metrics"' in runner


def test_px4_models_are_serialized_until_gazebo_finishes_each_insertion():
    runner = (
        launcher.PROJECT_ROOT / "vehicle_stack/run_cooperative_simulation.py"
    ).read_text(encoding="utf-8")
    assert "def wait_for_gazebo_model_ready(" in runner
    assert 'model_name = f"x500_mapping_{instance}"' in runner
    assert '"baylands_editable", model_name, px4_instance.process' in runner
    assert "[UAV SPAWN VERIFIED]" in runner


def test_uavs_get_attached_color_markers_and_terminal_role_events():
    runner = (
        launcher.PROJECT_ROOT / "vehicle_stack/run_cooperative_simulation.py"
    ).read_text(encoding="utf-8")
    everything = (
        launcher.PROJECT_ROOT / "scripts/run_everything.py"
    ).read_text(encoding="utf-8")
    assert "def color_uav_body_marker(" in runner
    model = (
        launcher.PROJECT_ROOT / "simulation/models/x500_mapping_base/model.sdf"
    ).read_text(encoding="utf-8")
    assert 'visual name="eirax_uav_id_plate"' in model
    assert '<box><size>0.080 0.055 0.012</size></box>' in model
    assert '<pose>-0.055 0 0.064 0 0 0</pose>' in model
    assert '"eirax_uav_id_plate"' in runner
    assert '"eirax_rgbd_camera_housing"' in runner
    assert 'UAV0 blue' in runner and 'UAV1 green' in runner and 'UAV2 orange' in runner
    assert "[LEADER CHANGE]" in runner
    assert "[DISCONNECTED]" in runner
    assert "[RECOVERY PENDING]" in runner
    assert "def complete_reconnect_if_due()" in runner
    assert runner.count("complete_reconnect_if_due()") >= 4
    assert "Followers: UAV0, UAV2" in runner
    assert "TERMINAL_EVENT_PREFIXES" in everything
    assert "relay_vehicle_events" in everything
    assert 'log_path.open("rb")' in everything
    assert "print_swarm_status(active_run)" in everything
    assert "[SWARM STATUS] Current leader/scout:" in everything


def test_all_uav_spawn_origins_are_raised_half_a_meter_consistently():
    runner = (
        launcher.PROJECT_ROOT / "vehicle_stack/run_cooperative_simulation.py"
    ).read_text(encoding="utf-8")
    assert "UAV_SPAWN_Z_OFFSET_M = 0.52" in runner
    assert runner.count("uav_pad[2] + UAV_SPAWN_Z_OFFSET_M") >= 2
    assert "uav_pad[2] + 0.02" not in runner


def test_reserve_uavs_never_read_scout_only_survey_metrics():
    follower = (
        launcher.PROJECT_ROOT / "vehicle_stack/uav/follow_husky.py"
    ).read_text(encoding="utf-8")
    # Both the survey calculation and its periodic report must use the same
    # active-role guard; reserves have survey targets but no survey_error.
    assert follower.count(
        'if survey_targets and phase == "survey" and role == "scout":'
    ) == 2

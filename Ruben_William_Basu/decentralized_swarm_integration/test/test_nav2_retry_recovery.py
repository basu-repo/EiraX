from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_nav2_retry_waits_for_lifecycle_and_rebuilds_costmaps():
    source = (
        ROOT / "ground_stack/baseline/mission/waypoint_mission.py"
    ).read_text(encoding="utf-8")
    assert "def recover_navigation(" in source
    assert 'reason="mission_retry"' in source
    assert "navigator.clearAllCostmaps()" in source
    assert 'reason="post_costmap_clear"' in source
    assert '"nav2_recovery_ready"' in source


def test_inactive_action_rejection_does_not_consume_attempt():
    source = (
        ROOT / "ground_stack/baseline/mission/waypoint_mission.py"
    ).read_text(encoding="utf-8")
    rejected = source.index("if accepted is False:")
    accepted = source.index("else:", rejected)
    rejection_branch = source[rejected:accepted]
    assert "attempt += 1" not in rejection_branch
    assert '"goal_rejected_while_unavailable"' in rejection_branch


def test_main_launcher_relays_nav2_recovery_status():
    source = (ROOT / "scripts/run_everything.py").read_text(encoding="utf-8")
    assert '"[NAV2 RECOVERY]"' in source
    assert '"[REVERSE RECOVERY]"' in source


def test_reverse_recovery_requires_fresh_lidar_and_runs_only_on_first_failure():
    source = (
        ROOT / "ground_stack/baseline/mission/waypoint_mission.py"
    ).read_text(encoding="utf-8")
    assert '"/husky/lidar3d/points"' in source
    assert '"reverse_recovery_skipped"' in source
    assert "navigator.backup(" in source
    assert "allow_reverse=attempt == 1" in source
    assert '"reverse_recovery_finished"' in source

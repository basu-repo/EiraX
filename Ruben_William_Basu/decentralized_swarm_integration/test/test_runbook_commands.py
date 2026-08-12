"""Prevent the runbook from drifting away from the real launcher interfaces."""

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
def help_text(script: Path) -> str:
    result = subprocess.run(
        [sys.executable, str(script), "--help"],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout


def test_proven_launcher_runbook_options_exist():
    output = help_text(ROOT / "vehicle_stack/run_cooperative_simulation.py")
    for option in (
        "--no-motion",
        "--headless",
        "--no-recording",
        "--view-3d-slam",
        "--return-to-spawn",
        "--ugv-only",
        "--uav-test-waypoint-1",
        "--uav-count",
        "--permanent-failure",
        "--connection-failure-reconnect",
    ):
        assert option in output
    assert "--uav-follow" not in output


def test_isolated_launcher_runbook_options_exist():
    output = help_text(ROOT / "scripts/run_baylands_swarm.py")
    for option in (
        "--no-motion",
        "--headless",
        "--no-recording",
        "--view-3d-slam",
        "--return-to-spawn",
        "--ugv-only",
        "--uav-test-waypoint-1",
    ):
        assert option in output
    assert "--uavs" not in output


def test_one_command_launcher_options_exist():
    output = help_text(ROOT / "scripts/run_everything.py")
    for option in ("--headless", "--no-motion", "--no-yolo", "--no-omnet", "--permanent-failure", "--connection-failure-reconnect"):
        assert option in output

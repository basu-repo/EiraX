from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_communication_link_tracks_every_swarm_aircraft():
    source = (ROOT / "vehicle_stack/communication/channel.py").read_text()
    assert 'for uav_id in ("uav1", "uav2")' in source
    assert 'self._record_uav_position("uav0", message)' in source
    assert "self.uav_position_timeout_sec" in source
    assert "Swarm telemetry relay now anchored by" in source
    assert "min(" in source


def test_followers_still_consume_the_network_relay_not_raw_gnss():
    source = (ROOT / "vehicle_stack/uav/follow_husky.py").read_text()
    assert '"/communication/uav/rx/ugv_gps"' in source
    assert 'NavSatFix, "/husky/gps"' not in source

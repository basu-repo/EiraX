import json
import math

import pytest

from dji_swarm_integration.dji_protocol import DjiCommand, decode_command, encode_command


def command(**changes):
    values = dict(
        uav_id="dji0", sequence=1, stamp_ns=10, kind="position",
        x=1.0, y=2.0, z=7.0, yaw=0.2, pan_deg=0.0, tilt_deg=-45.0,
        reason="test",
    )
    values.update(changes)
    return DjiCommand(**values)


def test_dji_command_round_trip():
    expected = command()
    assert decode_command(encode_command(expected)) == expected


@pytest.mark.parametrize("changes", [
    {"kind": "arm"}, {"z": math.nan}, {"yaw": 4.0},
    {"tilt_deg": -91.0}, {"pan_deg": 181.0}, {"sequence": -1},
])
def test_invalid_dji_commands_are_rejected(changes):
    with pytest.raises(ValueError):
        encode_command(command(**changes))


def test_wrong_protocol_is_rejected():
    payload = json.loads(encode_command(command()))
    payload["protocol"] = "wrong"
    with pytest.raises(ValueError):
        decode_command(json.dumps(payload))


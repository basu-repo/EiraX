import math

import pytest

from decentralized_swarm_integration.px4_protocol import (
    Point3,
    enu_to_ned,
    ned_to_enu,
    validate_setpoint,
    yaw_enu_to_ned,
    yaw_ned_to_enu,
)


def test_ned_enu_position_round_trip():
    enu = ned_to_enu(4.0, -3.0, -7.0)
    assert enu == Point3(-3.0, 4.0, 7.0)
    assert enu_to_ned(enu.x, enu.y, enu.z) == Point3(4.0, -3.0, -7.0)


@pytest.mark.parametrize("yaw", [-math.pi, -1.0, 0.0, 1.0, math.pi])
def test_yaw_round_trip(yaw):
    recovered = yaw_enu_to_ned(yaw_ned_to_enu(yaw))
    assert math.isclose(math.sin(recovered), math.sin(yaw), abs_tol=1e-9)
    assert math.isclose(math.cos(recovered), math.cos(yaw), abs_tol=1e-9)


def test_safe_setpoint_is_converted_to_ned():
    result = validate_setpoint(
        x_enu=3.0,
        y_enu=4.0,
        z_enu=8.0,
        yaw_enu=0.0,
        home_enu=Point3(0.0, 0.0, 0.0),
        current_enu=Point3(2.0, 3.0, 7.0),
        max_radius_m=20.0,
        min_altitude_m=2.0,
        max_altitude_m=15.0,
        max_step_m=5.0,
    )
    assert (result.north, result.east, result.down) == (4.0, 3.0, -8.0)


def test_map_setpoint_is_made_relative_to_nonzero_px4_home():
    result = validate_setpoint(
        x_enu=103.0,
        y_enu=204.0,
        z_enu=18.0,
        yaw_enu=0.0,
        home_enu=Point3(100.0, 200.0, 10.0),
        current_enu=Point3(102.0, 203.0, 17.0),
        max_radius_m=20.0,
        min_altitude_m=2.0,
        max_altitude_m=15.0,
        max_step_m=5.0,
    )
    assert (result.north, result.east, result.down) == (4.0, 3.0, -8.0)


@pytest.mark.parametrize(
    "target,error",
    [
        ((30.0, 0.0, 8.0), "home radius"),
        ((0.0, 0.0, 1.0), "altitude"),
        ((9.0, 0.0, 8.0), "step"),
        ((math.nan, 0.0, 8.0), "non-finite"),
    ],
)
def test_unsafe_setpoint_is_rejected(target, error):
    with pytest.raises(ValueError, match=error):
        validate_setpoint(
            x_enu=target[0],
            y_enu=target[1],
            z_enu=target[2],
            yaw_enu=0.0,
            home_enu=Point3(0.0, 0.0, 0.0),
            current_enu=Point3(0.0, 0.0, 8.0),
            max_radius_m=20.0,
            min_altitude_m=2.0,
            max_altitude_m=15.0,
            max_step_m=5.0,
        )

from vehicle_stack.uav.landing_protocol import (
    FORCE_DISARM_AFTER_SEC,
    PX4_FORCE_DISARM_MAGIC,
    disarm_parameters,
)


def test_first_touchdown_disarm_is_normal():
    assert disarm_parameters(None, 100.0) == (0.0,)


def test_disarm_stays_normal_during_land_detector_grace_period():
    assert disarm_parameters(100.0, 100.0 + FORCE_DISARM_AFTER_SEC - 0.01) == (0.0,)


def test_disarm_uses_px4_force_parameter_after_bounded_grace_period():
    assert disarm_parameters(100.0, 100.0 + FORCE_DISARM_AFTER_SEC) == (
        0.0,
        PX4_FORCE_DISARM_MAGIC,
    )

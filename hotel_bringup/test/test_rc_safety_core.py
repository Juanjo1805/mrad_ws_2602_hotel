"""Deterministic safety acceptance tests, without ROS, serial ports or actuators."""

from dataclasses import replace
import math
import random

from hotel_bringup.safety_core import AEBConfig, AEBController
import pytest
from ybeb_2602_zulu.safety_core import CommandWatchdog, Limits, servo_values
from ybeb_2602_zulu.scan_support import projected_ttc, Scan


def scan(distance=4.0, obstacles=()):
    """Build a complete one-degree scan; obstacles are (degrees, metres)."""
    ranges = [distance] * 361
    for angle, value in obstacles:
        ranges[round(angle) + 180] = value
    return Scan(tuple(ranges), -math.pi, math.pi / 180, math.pi, 0.1, 12.0)


def controller(**kwargs):
    """Use confirmed synthetic calibration only in tests."""
    return AEBController(AEBConfig(calibration_confirmed=True, **kwargs), Limits())


def feed(core, now=0.0, throttle=0.1, steering=0.2, lidar=None):
    """Refresh both inputs with deterministic monotonic timestamps."""
    core.watchdog.update(throttle, steering, now)
    core.update_scan(scan() if lidar is None else lidar, now)
    return core.evaluate(now)


def test_case_01_clear_pass_through():
    """A valid bounded command passes exactly, including steering sign."""
    decision = feed(controller(), throttle=0.13, steering=-0.27)
    assert (decision.throttle, decision.steering) == (0.13, -0.27)
    assert not decision.active


def test_case_02_ttc_below_threshold_neutral():
    """Use projected distance, the 0.45 m envelope and the 2 m/s bound."""
    decision = feed(controller(), lidar=scan(obstacles=[(0, 1.0)]))
    assert decision.ttc == pytest.approx((1.0 - 0.45) / 2.0)
    assert decision.reason == 'collision' and decision.throttle == 0.0
    assert decision.steering == 0.2


def test_case_03_nonapproaching_lateral_obstacle():
    """A 90-degree lateral ray has no closing projection and cannot brake."""
    decision = feed(controller(), lidar=scan(obstacles=[(90, 0.2), (-90, 0.2)]))
    assert decision.throttle == 0.1 and not decision.active


def test_case_04_infinity():
    """An isolated no-return is tolerated; no usable scan stops the vehicle."""
    assert feed(controller(), lidar=scan(obstacles=[(0, math.inf)])).throttle == 0.1
    assert feed(controller(), lidar=scan(math.inf)).throttle == 0.0


def test_case_05_nan():
    """Nonfinite ranges cannot reach a TTC division or an actuator output."""
    assert feed(controller(), lidar=scan(obstacles=[(0, math.nan)])).throttle == 0.1
    assert feed(controller(), lidar=scan(math.nan)).reason == 'insufficient_scan_returns'


def test_case_06_zero_velocity():
    """At standstill the TTC is infinite, without division by zero."""
    decision = feed(controller(), throttle=0.0, lidar=scan(obstacles=[(0, 0.2)]))
    assert decision.throttle == 0.0 and math.isinf(decision.ttc)


def test_case_07_forward_obstacle():
    """Positive normalized throttle is blocked by a frontal obstacle."""
    assert feed(controller(), lidar=scan(obstacles=[(0, 0.8)])).active


def test_case_08_scan_loss():
    """A refreshed motion command cannot mask a stale scan."""
    core = controller()
    feed(core)
    core.watchdog.update(0.1, 0.2, 0.31)
    decision = core.evaluate(0.31)
    assert decision.throttle == 0.0 and decision.reason == 'scan_timeout'


def test_case_09_command_loss():
    """Fresh scans cannot keep a stale motion command alive."""
    core = controller()
    feed(core)
    core.update_scan(scan(), 0.31)
    decision = core.evaluate(0.31)
    assert decision.throttle == 0.0 and decision.reason == 'command_timeout'


def test_case_10_independent_hardware_watchdog():
    """Watchdog returns neutral without requiring the AEB process to run."""
    watchdog = CommandWatchdog(Limits(), 0.25)
    watchdog.update(0.3, 0.4, 10.0)
    assert watchdog.sample(10.24)[0] == (0.3, 0.4)
    assert watchdog.sample(10.25)[0] == (0.0, 0.0)
    assert servo_values(*watchdog.sample(10.3)[0], Limits()) == (91.0, 127.5)


def test_case_11_release_hysteresis_and_fresh_scans():
    """Release needs three distinct clear scans and 0.3 seconds of clear evidence."""
    core = controller()
    assert feed(core, lidar=scan(obstacles=[(0, 1.0)])).active
    # TTC=0.55, between engage and release: stay latched.
    assert feed(core, 0.1, lidar=scan(obstacles=[(0, 1.55)])).active
    assert feed(core, 0.2).active
    assert feed(core, 0.3).active
    assert feed(core, 0.4).active
    assert not feed(core, 0.51).active
    assert feed(core, 0.6, lidar=scan(obstacles=[(0, 1.0)])).active
    assert feed(core, 0.7).active
    # Timer ticks do not count as additional scans, nor clear after scan expiry.
    for now in (0.71, 0.72, 0.73, 0.74):
        assert core.evaluate(now).active


def test_case_12_steering_and_throttle_saturation():
    """Clamp both axes at both guard boundaries."""
    d = feed(controller(), throttle=9.0, steering=-9.0)
    assert (d.throttle, d.steering) == (0.4, -0.5)
    assert servo_values(9.0, -9.0, Limits()) == pytest.approx((101.8, 75.0))


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('axis', ['throttle', 'steering'])
def test_case_13_nonfinite_command_neutral(value, axis):
    """Any invalid axis invalidates the entire command, including last good input."""
    core = controller()
    feed(core)
    args = {'throttle': 0.1, 'steering': 0.2, axis: value}
    d = feed(core, 0.1, **args)
    assert d.throttle == 0.0 and d.steering == 0.0 and d.reason == 'invalid_command'
    assert servo_values(args['throttle'], args['steering'], Limits()) == (91.0, 127.5)


def test_reverse_uses_rear_projection():
    """Negative throttle checks rear rays; an isolated front hazard is receding."""
    assert not feed(controller(), throttle=-0.1, lidar=scan(obstacles=[(0, 0.2)])).active
    d = feed(controller(), throttle=-0.1, lidar=scan(obstacles=[(180, 0.8)]))
    assert d.throttle == 0.0 and d.ttc == pytest.approx(0.35)


def test_nonapproaching_rays_and_projection_angle():
    """A 60-degree ray approaches at half the longitudinal speed."""
    c = AEBConfig()
    ttc, reason = projected_ttc(scan(obstacles=[(60, 0.9)]), 2.0, c)
    assert reason == 'ok' and ttc == pytest.approx(0.45)
    assert projected_ttc(scan(obstacles=[(180, 0.1)]), 2.0, c)[0] > 0.65


@pytest.mark.parametrize('bad', [(), (0.0,) * 361, (20.0,) * 361, (-1.0,) * 361])
def test_invalid_scan_stops_immediately(bad):
    """Empty scans and returns outside sensor bounds are never clear evidence."""
    core = controller()
    feed(core)
    decision = feed(core, 0.1, lidar=replace(scan(), ranges=bad))
    assert decision.active and decision.throttle == 0


def test_blind_sector_and_incomplete_rear_scan():
    """Sufficient total returns cannot conceal a large contiguous blind sector."""
    holes = [(i, math.nan) for i in range(-15, 16)]
    assert feed(controller(), lidar=scan(obstacles=holes)).reason == 'scan_blind_sector'
    half = Scan((4.0,) * 181, -math.pi / 2, math.pi / 180, math.pi / 2, 0.1, 12.0)
    assert feed(controller(), throttle=-0.1, lidar=half).active


@pytest.mark.parametrize('field,value', [
    ('angle_increment', 0.0), ('angle_min', math.nan), ('range_max', math.inf),
    ('range_min', 20.0), ('angle_max', 0.0),
])
def test_scan_metadata_is_checked(field, value):
    """Corrupt sensor geometry fails closed."""
    assert feed(controller(), lidar=replace(scan(), **{field: value})).active


def test_hold_center_and_fault_steering():
    """Collision can hold steering; input failures always select safe steering."""
    c = controller(brake_steering_mode='center')
    assert feed(c, lidar=scan(obstacles=[(0, 0.8)])).steering == 0.0
    assert c.evaluate(0.4).steering == 0.0


def test_zero_does_not_clear_collision_latch():
    """An operator releasing throttle cannot erase a still-present collision hazard."""
    c = controller()
    obstacle = scan(obstacles=[(0, 0.8)])
    assert feed(c, lidar=obstacle).active
    for t in (0.2, 0.4, 0.6, 0.8):
        assert feed(c, t, throttle=0.0, lidar=obstacle).reason == 'collision'


def test_coast_remembers_previous_motion_direction():
    """A neutral or reversed command does not immediately assume a stopped vehicle."""
    c = controller()
    feed(c)
    d = feed(c, 0.1, throttle=-0.1, lidar=scan(obstacles=[(0, 0.8)]))
    assert d.reason == 'collision'


def test_uncalibrated_and_startup_are_neutral():
    """Unverified physical bounds cannot silently authorize real motion."""
    c = AEBController(AEBConfig(), Limits())
    assert feed(c).reason == 'calibration_required'
    assert controller().evaluate(0).throttle == 0.0


@pytest.mark.parametrize('kwargs', [
    {'ttc_threshold': math.nan}, {'release_ttc_threshold': 0.4},
    {'scan_timeout': 0.0}, {'footprint_radius_m': -1.0},
    {'brake_steering_mode': 'reverse'}, {'front_fov_deg': 181.0},
    {'min_valid_fraction': 0.0},
])
def test_reject_invalid_configuration(kwargs):
    """Configuration errors cannot disable failsafes implicitly."""
    with pytest.raises(ValueError):
        AEBConfig(**kwargs)


def test_hard_limits_and_random_finite_pwm():
    """Many oversized finite commands always remain inside the calibrated envelope."""
    with pytest.raises(ValueError):
        Limits(0.41, 0.5)
    with pytest.raises(ValueError):
        Limits(0.4, 0.51)
    rng = random.Random(73)
    for _ in range(1000):
        throttle, steering = servo_values(rng.uniform(-100, 100), rng.uniform(-100, 100), Limits())
        assert 80.2 - 1e-9 <= throttle <= 101.8 + 1e-9
        assert 75.0 <= steering <= 180.0

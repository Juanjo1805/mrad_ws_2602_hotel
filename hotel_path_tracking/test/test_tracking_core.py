import math

import numpy as np

from hotel_path_tracking.tracking_core import (
    LQRTracker,
    Pose2D,
    discrete_error_model,
    local_errors,
    nearest_forward_index,
    solve_discrete_lqr,
    wrap_to_pi,
)


def test_wrap_handles_pi_boundary():
    assert abs(wrap_to_pi(2.0 * math.pi + 0.2) - 0.2) < 1e-12
    assert abs(wrap_to_pi(math.pi + 0.1) + math.pi - 0.1) < 1e-12


def test_local_errors_are_in_reference_frame():
    ex, ey, eth = local_errors(Pose2D(1.0, 0.0, math.pi / 2.0), Pose2D(0.0, 0.0, 0.0))
    assert ex == 1.0
    assert ey == 0.0
    assert abs(eth - math.pi / 2.0) < 1e-12


def test_discrete_model_and_riccati_gain_are_finite():
    a_matrix, b_matrix = discrete_error_model(0.45, 0.2, 0.04)
    gain = solve_discrete_lqr(a_matrix, b_matrix, np.diag([1.0, 6.0, 3.0]), np.diag([0.8, 0.6]))
    assert a_matrix.shape == (3, 3)
    assert b_matrix.shape == (3, 2)
    assert gain.shape == (2, 3)
    assert np.all(np.isfinite(gain))


def test_monotonic_nearest_point_does_not_reacquire_past_waypoint():
    path = [Pose2D(float(index), 0.0, 0.0) for index in range(5)]
    assert nearest_forward_index(path, Pose2D(0.1, 0.0, 0.0), 3) == 3


def test_lqr_saturates_and_stops_at_goal():
    path = [Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0)]
    tracker = LQRTracker(max_speed=0.5, max_omega=1.0, goal_tolerance=0.2)
    command = tracker.command(path, Pose2D(0.0, 1.0, 0.0))
    assert 0.0 <= command.linear_velocity <= 0.5
    assert abs(command.angular_velocity) <= 1.0
    stop = tracker.command(path, Pose2D(0.9, 0.0, 0.0))
    assert stop.goal_reached
    assert stop.linear_velocity == 0.0
    assert stop.angular_velocity == 0.0

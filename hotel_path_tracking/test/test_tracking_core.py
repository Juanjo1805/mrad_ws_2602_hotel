import math

import numpy as np

from hotel_path_tracking.tracking_core import (
    AdaptivePurePursuitReference,
    LQRTracker,
    Pose2D,
    PurePursuitReference,
    discrete_error_model,
    headings_from_points,
    local_errors,
    nearest_forward_index,
    solve_discrete_lqr,
    wrap_to_pi,
)
from hotel_path_tracking.optimization_monitor import assess_stuck_window


def pure_pursuit_for_test():
    return PurePursuitReference(
        lookahead_l0=0.5,
        lookahead_kv=0.0,
        lookahead_min=0.5,
        lookahead_max=0.5,
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


def test_discrete_model_has_reference_frame_rotation_signs():
    matrix, _ = discrete_error_model(0.45, 0.2, 0.04)
    assert matrix[0, 1] > 0.0
    assert matrix[1, 0] < 0.0


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


def test_pure_pursuit_straight_target_has_zero_turn_rate():
    path = headings_from_points([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
    command = pure_pursuit_for_test().command(path, Pose2D(0.0, 0.0, 0.0))
    assert command.linear_velocity > 0.0
    assert abs(command.angular_velocity) < 1e-9


def test_pure_pursuit_target_on_left_turns_positive():
    path = headings_from_points([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)])
    command = pure_pursuit_for_test().command(path, Pose2D(0.0, 0.0, 0.0))
    assert command.angular_velocity > 0.0


def test_pure_pursuit_target_on_right_turns_negative():
    path = headings_from_points([(0.0, 0.0), (1.0, -1.0), (2.0, -2.0)])
    command = pure_pursuit_for_test().command(path, Pose2D(0.0, 0.0, 0.0))
    assert command.angular_velocity < 0.0


def test_pure_pursuit_stops_only_at_final_pose():
    path = headings_from_points([(0.0, 0.0), (1.0, 0.0)])
    tracker = pure_pursuit_for_test()
    command = tracker.command(path, Pose2D(0.9, 0.0, 0.0))
    assert command.goal_reached
    assert command.linear_velocity == 0.0
    assert command.angular_velocity == 0.0


def test_pure_pursuit_does_not_finish_on_first_visit_to_repeated_goal():
    """A two-lap route may pass the final XY before its last path index."""
    path = headings_from_points([
        (2.0, 0.0), (1.0, 0.0), (0.05, 0.0),
        (1.0, 0.0), (0.0, 0.0),
    ])
    tracker = pure_pursuit_for_test()

    tracker.command(path, Pose2D(1.0, 0.0, math.pi))
    first_lap_close = tracker.command(path, Pose2D(0.0, 0.0, math.pi))
    assert not first_lap_close.goal_reached

    tracker.command(path, Pose2D(1.0, 0.0, 0.0))
    final_close = tracker.command(path, Pose2D(0.0, 0.0, math.pi))
    assert final_close.goal_reached


def test_pure_pursuit_terminal_approach_turns_before_driving_to_goal():
    path = headings_from_points([(0.0, 0.0), (1.0, 0.0)])
    command = pure_pursuit_for_test().command(
        path, Pose2D(0.78, 0.20, math.pi / 2.0))
    assert command.linear_velocity == 0.0
    assert command.angular_velocity < 0.0


def test_closed_route_does_not_stop_at_its_initial_final_point():
    path = headings_from_points([
        (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0),
    ])
    pure = pure_pursuit_for_test().command(path, Pose2D(0.0, 0.0, 0.0))
    lqr = LQRTracker(lookahead_distance=0.1).command(path, Pose2D(0.0, 0.0, 0.0))
    assert not pure.goal_reached
    assert pure.linear_velocity > 0.0
    assert not lqr.goal_reached
    assert lqr.linear_velocity > 0.0


def test_lqr_turn_signs_match_differential_robot():
    left_path = headings_from_points([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)])
    right_path = headings_from_points([(0.0, 0.0), (1.0, -1.0), (2.0, -2.0)])
    left = LQRTracker(lookahead_distance=0.1).command(left_path, Pose2D(0.0, 0.0, 0.0))
    right = LQRTracker(lookahead_distance=0.1).command(right_path, Pose2D(0.0, 0.0, 0.0))
    assert left.angular_velocity > 0.0
    assert right.angular_velocity < 0.0


def test_lqr_terminal_approach_turns_before_driving_to_goal():
    path = [Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0)]
    command = LQRTracker(lookahead_distance=0.25).command(
        path, Pose2D(0.78, 0.20, math.pi / 2.0))
    assert command.linear_velocity == 0.0
    assert command.angular_velocity < 0.0


def test_adaptive_pure_pursuit_respects_coupled_wheel_limit():
    path = headings_from_points([(0.0, 0.0), (0.4, 0.5), (0.5, 1.0)])
    tracker = AdaptivePurePursuitReference(
        lookahead_min=0.3,
        lookahead_max=0.3,
        lookahead_gain=0.0,
        acceleration_limit=20.0,
        deceleration_limit=20.0,
        angular_acceleration_limit=100.0,
        max_speed=0.5,
        max_omega=2.0,
        wheel_radius=0.05,
        wheel_separation=0.44,
        wheel_max_angular_velocity=10.0,
    )
    command = tracker.command(path, Pose2D(0.0, 0.0, 0.0), dt=0.1)
    assert command.linear_velocity >= 0.0
    assert abs(command.linear_velocity) + 0.22 * abs(command.angular_velocity) <= 0.5 + 1e-9


def test_adaptive_preview_reduces_speed_before_future_curve():
    path = headings_from_points([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.0, 0.5), (1.0, 1.0)])
    common = dict(
        lookahead_min=0.3, lookahead_max=0.3, lookahead_gain=0.0,
        acceleration_limit=20.0, deceleration_limit=20.0,
        angular_acceleration_limit=100.0, max_speed=0.5,
        curvature_speed_gain=2.0,
    )
    without_preview = AdaptivePurePursuitReference(curvature_preview_distance=0.0, **common)
    with_preview = AdaptivePurePursuitReference(curvature_preview_distance=1.0, **common)
    pose = Pose2D(0.0, 0.0, 0.0)
    assert with_preview.command(path, pose, 0.1).linear_velocity < \
        without_preview.command(path, pose, 0.1).linear_velocity


def test_adaptive_pure_pursuit_applies_linear_acceleration_limit():
    path = headings_from_points([(0.0, 0.0), (2.0, 0.0)])
    tracker = AdaptivePurePursuitReference(
        lookahead_min=0.3, lookahead_max=0.3, lookahead_gain=0.0,
        acceleration_limit=0.2, deceleration_limit=1.0,
        angular_acceleration_limit=100.0, max_speed=0.5,
    )
    command = tracker.command(path, Pose2D(0.0, 0.0, 0.0), dt=0.1)
    assert command.linear_velocity <= 0.02 + 1e-12


def test_adaptive_straight_uses_effective_max_speed_and_large_lookahead():
    path = headings_from_points([(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)])
    tracker = AdaptivePurePursuitReference(
        lookahead_min=0.30, lookahead_base=0.70, lookahead_max=1.20,
        lookahead_gain=0.60, curvature_preview_distance=0.0,
        acceleration_limit=20.0, deceleration_limit=20.0,
        angular_acceleration_limit=100.0, nominal_speed=0.90,
        max_speed=1.00, max_omega=4.0,
        wheel_max_angular_velocity=10.0,
    )
    command = tracker.command(path, Pose2D(0.0, 0.0, 0.0), dt=0.1)
    # The explicit legacy interface is 10 rad/s * 0.05 m = 0.50 m/s.
    assert math.isclose(tracker.max_speed, 0.50)
    assert math.isclose(command.linear_velocity, 0.50)
    assert command.lookahead_distance >= 0.70


def test_adaptive_strong_curve_reduces_speed_and_lookahead():
    straight = headings_from_points([(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)])
    turning = headings_from_points([(0.0, 0.0), (0.50, 0.30), (0.90, 0.90)])
    common = {
        'lookahead_min': 0.30, 'lookahead_base': 0.70, 'lookahead_max': 1.20,
        'lookahead_gain': 0.60, 'lookahead_curvature_gain': 1.0,
        'curvature_speed_gain': 0.8, 'curvature_preview_distance': 1.0,
        'acceleration_limit': 20.0, 'deceleration_limit': 20.0,
        'angular_acceleration_limit': 100.0, 'max_speed': 1.0,
        'nominal_speed': 0.9, 'heading_error_speed_gain': 0.0,
    }
    straight_tracker = AdaptivePurePursuitReference(**common)
    turning_tracker = AdaptivePurePursuitReference(**common)
    straight_command = straight_tracker.command(straight, Pose2D(0.0, 0.0, 0.0), 0.1)
    turning_command = turning_tracker.command(turning, Pose2D(0.0, 0.0, 0.0), 0.1)
    assert turning_command.linear_velocity < straight_command.linear_velocity
    assert turning_command.lookahead_distance < straight_command.lookahead_distance


def test_adaptive_lateral_error_limit_is_progressive_and_configurable():
    path = headings_from_points([(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)])
    tracker = AdaptivePurePursuitReference(
        lookahead_min=0.60, lookahead_base=0.60, lookahead_max=0.60,
        lookahead_gain=0.0, acceleration_limit=20.0, deceleration_limit=20.0,
        angular_acceleration_limit=100.0, max_speed=1.0, nominal_speed=0.9,
        lateral_error_slowdown_threshold=0.10, lateral_error_speed_gain=2.0,
        heading_error_speed_gain=0.0,
    )
    tracker.command(path, Pose2D(0.0, 0.50, 0.0), 0.1)
    assert tracker.last_speed_limit_lateral_error < tracker.max_speed
    assert tracker.last_speed_limit_lateral_error > 0.0


def test_adaptive_omega_limit_reduces_linear_target_before_saturation():
    path = headings_from_points([(0.0, 0.0), (0.50, 0.25), (1.0, 0.50)])
    tracker = AdaptivePurePursuitReference(
        lookahead_min=0.50, lookahead_base=0.50, lookahead_max=0.50,
        lookahead_gain=0.0, acceleration_limit=20.0, deceleration_limit=20.0,
        angular_acceleration_limit=100.0, max_speed=1.0, nominal_speed=1.0,
        max_omega=1.0, max_curvature=4.0, wheel_max_angular_velocity=100.0,
        curvature_speed_gain=0.0, lateral_error_speed_gain=0.0,
        heading_error_speed_gain=0.0,
    )
    command = tracker.command(path, Pose2D(0.0, 0.0, 0.0), 0.1)
    assert math.isclose(tracker.last_speed_limit_omega, 0.5, abs_tol=5e-6)
    assert command.linear_velocity <= tracker.last_speed_limit_omega + 1e-9
    assert abs(command.angular_velocity) <= 1.0 + 1e-9


def test_adaptive_deceleration_rate_limit_is_asymmetric():
    assert math.isclose(
        AdaptivePurePursuitReference._rate_limit(0.0, 0.50, 0.2, 1.0, 0.1), 0.40)


def test_adaptive_requested_limits_never_bypass_wheel_interfaces():
    tracker = AdaptivePurePursuitReference(
        nominal_speed=0.90, max_speed=1.00, max_omega=4.00,
        wheel_radius=0.05, wheel_separation=0.44, wheel_max_angular_velocity=10.0)
    assert math.isclose(tracker.max_speed, 0.50)
    assert math.isclose(tracker.max_omega, 0.50 / 0.22)


def test_adaptive_twenty_rad_s_wheels_allow_one_m_s_straight_command():
    """The new wheel interface admits the requested 1 m/s straight speed."""
    path = headings_from_points([(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)])
    tracker = AdaptivePurePursuitReference(
        nominal_speed=1.00, max_speed=1.00, max_omega=4.00,
        acceleration_limit=20.0, deceleration_limit=20.0,
        wheel_radius=0.05, wheel_separation=0.44,
        wheel_max_angular_velocity=20.0)
    command = tracker.command(path, Pose2D(0.0, 0.0, 0.0), dt=0.1)
    left_wheel = (command.linear_velocity - 0.22 * command.angular_velocity) / 0.05
    right_wheel = (command.linear_velocity + 0.22 * command.angular_velocity) / 0.05

    assert math.isclose(tracker.wheel_linear_limit, 1.00)
    assert math.isclose(tracker.physical_max_omega, 1.00 / 0.22)
    assert math.isclose(tracker.max_omega, 4.00)
    assert math.isclose(command.linear_velocity, 1.00)
    assert math.isclose(command.angular_velocity, 0.0, abs_tol=1e-12)
    assert math.isclose(left_wheel, 20.0)
    assert math.isclose(right_wheel, 20.0)


def _stuck_row(time_s, x, path_s, closest_index, actual_v=0.0, actual_w=0.0, target_v=0.2):
    return {
        'time_s': time_s, 'x_m': x, 'y_m': 0.0, 'path_s_m': path_s,
        'closest_index': closest_index, 'actual_v_m_s': actual_v,
        'actual_omega_rad_s': actual_w, 'target_v_m_s': target_v, 'mux_v_m_s': target_v,
    }


def test_stuck_requires_all_no_progress_signals_and_controller_attempt():
    rows = [_stuck_row(0.0, 1.0, 5.0, 100), _stuck_row(45.0, 1.01, 5.02, 100)]
    stuck, evidence = assess_stuck_window(rows, 0.12, 0.12, 3, 0.025, 0.08, 0.04)
    assert stuck
    assert evidence['controller_attempting']


def test_stuck_allows_a_recovery_turn_or_measurable_progress():
    turning = [_stuck_row(0.0, 1.0, 5.0, 100), _stuck_row(45.0, 1.01, 5.02, 100, actual_w=0.12)]
    progressing = [_stuck_row(0.0, 1.0, 5.0, 100), _stuck_row(45.0, 1.2, 5.20, 105)]
    assert not assess_stuck_window(turning, 0.12, 0.12, 3, 0.025, 0.08, 0.04)[0]
    assert not assess_stuck_window(progressing, 0.12, 0.12, 3, 0.025, 0.08, 0.04)[0]

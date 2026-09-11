import math

import numpy as np

from hotel_path_tracking.tracking_core import (
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

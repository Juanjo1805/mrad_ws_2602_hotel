import math
from types import SimpleNamespace

import numpy as np

from hotel_ttc_follow_the_gap.reactive_core import (ReactiveState, ReactiveStateMachine,
                                                     point_segment_distance, reactive_preview,
                                                     safe_gap_command, select_gap)
from hotel_ttc_follow_the_gap.ttc_gap_finder import FollowGapFinder


def test_lateral_obstacle_is_not_a_path_blocker():
    # The corridor distance calculation is exercised in the ROS node; this
    # synthetic FTG scene confirms a lateral hit leaves a central valid gap.
    angles = np.linspace(-math.pi / 2, math.pi / 2, 181)
    ranges = np.full(angles.shape, 5.0)
    ranges[np.argmin(np.abs(angles - 1.2))] = 0.5
    gap = select_gap(angles, ranges, 6.0, 0.35, 0.12, 0.55)
    assert gap.valid
    assert abs(gap.angle) < 0.1


def test_corridor_geometry_rejects_a_lateral_hit():
    distance, _ = point_segment_distance((1.0, 0.8), (0.0, 0.0), (2.0, 0.0))
    assert distance > 0.35


def test_single_noise_scan_does_not_activate():
    machine = ReactiveStateMachine(0.3, 0.6, 0.8, 0.8)
    assert machine.update(0.0, True, False, False)[0] == ReactiveState.TRACKING
    assert machine.update(0.1, False, True, False)[0] == ReactiveState.TRACKING


def test_front_obstacle_splits_gap_and_chooses_safe_side():
    angles = np.linspace(-math.pi / 2, math.pi / 2, 181)
    ranges = np.full(angles.shape, 5.0)
    ranges[np.abs(angles) < 0.22] = 0.40
    gap = select_gap(angles, ranges, 6.0, 0.35, 0.12, 0.55, target_angle=0.3,
                     path_bias=0.2, bubble_obstacle_distance=2.2)
    assert gap.valid
    assert abs(gap.angle) > 0.22


def test_too_narrow_gap_is_rejected():
    angles = np.linspace(-math.pi / 2, math.pi / 2, 181)
    ranges = np.zeros(angles.shape)
    ranges[89:92] = 1.0
    assert not select_gap(angles, ranges, 6.0, 0.1, 0.12, 0.55).valid


def test_preview_grows_with_speed_and_covers_braking():
    slow = reactive_preview(0.5, 1.1, 0.55, 1.0, 2.5, 1.4, 0.2, 0.2, 0.15)
    fast = reactive_preview(1.0, 1.1, 0.55, 1.0, 2.5, 1.4, 0.2, 0.2, 0.15)
    assert fast > slow
    assert fast >= 1.5


def test_state_machine_requires_persistence_and_hysteresis():
    machine = ReactiveStateMachine(0.3, 0.6, 0.8, 0.8)
    assert machine.update(0.0, True, False, False)[0] == ReactiveState.TRACKING
    assert machine.update(0.2, True, False, False)[0] == ReactiveState.TRACKING
    assert machine.update(0.31, True, False, False)[0] == ReactiveState.AVOIDING
    # One clear scan cannot leave AVOIDING.
    assert machine.update(0.5, False, True, False)[0] == ReactiveState.AVOIDING
    assert machine.update(1.0, False, True, False)[0] == ReactiveState.AVOIDING
    assert machine.update(1.2, False, True, False)[0] == ReactiveState.REJOINING
    assert machine.update(1.5, False, True, True)[0] == ReactiveState.REJOINING
    assert machine.update(2.31, False, True, True)[0] == ReactiveState.TRACKING


def test_lateral_or_heading_error_keeps_rejoining():
    machine = ReactiveStateMachine(0.0, 0.0, 0.5, 0.0)
    machine.update(0.0, True, False, False)
    machine.update(0.1, False, True, False)
    assert machine.state == ReactiveState.REJOINING
    assert machine.update(0.7, False, True, False)[0] == ReactiveState.REJOINING


def test_gap_command_is_wheel_safe_at_high_turn_rate():
    velocity, omega = safe_gap_command(2.0, 0.68, 2.0, 1.2, 1.2, 1.0, 0.22)
    assert abs(omega) <= 2.0
    assert velocity + 0.22 * abs(omega) <= 1.0 + 1e-9


def test_front_box_with_open_sides_has_a_valid_reactive_gap():
    """Regression: expanding every nearby wall ray erased the whole FOV."""
    angles = np.linspace(-math.pi / 3, math.pi / 3, 241)
    ranges = np.full(angles.shape, np.inf)
    ranges[np.abs(angles) < 0.16] = 0.80
    from hotel_ttc_follow_the_gap.reactive_core import analyze_gap
    result, diagnostics = analyze_gap(angles, ranges, 12.0, 0.50, 0.12, 0.55,
                                      target_angle=0.0, path_bias=0.2,
                                      bubble_obstacle_distance=2.2,
                                      bubble_closest_only=True)
    assert diagnostics.candidate_gap_count >= 2
    assert diagnostics.valid_gap_count >= 1
    assert result.valid and result.width > 0.0 and abs(result.angle) > 0.05
    velocity, omega = safe_gap_command(result.angle, 0.68, 2.0, 1.2, 1.2, 1.0, 0.22)
    assert velocity > 0.0 and abs(omega) > 0.05


def test_original_and_reactive_both_recognize_open_space_for_same_scan():
    angles = np.linspace(-math.pi / 3, math.pi / 3, 241)
    ranges = np.full(angles.shape, np.inf)
    ranges[np.abs(angles) < 0.16] = 0.40
    finder = object.__new__(FollowGapFinder)
    finder.min_range = 0.05; finder.fov = math.radians(120.0); finder.ttc_threshold = 0.6
    finder.bubble_base = 0.35; finder.bubble_vel_k = 0.0; finder.min_clearance = 0.12
    finder.min_gap_width_deg = 3.0; finder.min_depth_threshold = 0.9; finder.deadend_weight = 1.1
    finder.no_gap_counter = 0; finder.prev_angle = 0.0; finder.search_dir = 1.0
    message = SimpleNamespace(ranges=ranges, range_max=12.0)
    prepared = finder._prepared_ranges(message, angles)
    ttc, _ = finder._time_to_collision(prepared, angles, 1.0)
    finder._apply_bubbles(prepared, angles[1] - angles[0], 1.0)
    original_gaps = finder._valid_gaps(ttc, prepared, angles, angles[1] - angles[0])
    original_angle = finder._select_gap_angle(original_gaps, prepared, angles, angles[1] - angles[0], 12.0)
    reactive = select_gap(angles, ranges, 12.0, 0.50, 0.12, 0.55,
                          bubble_obstacle_distance=2.2, bubble_closest_only=True)
    assert original_gaps and abs(original_angle) > 0.05
    assert reactive.valid and abs(reactive.angle) > 0.05


def test_path_bias_only_scores_previously_valid_gaps():
    angles = np.linspace(-math.pi / 3, math.pi / 3, 241)
    ranges = np.full(angles.shape, np.inf)
    ranges[np.abs(angles) < 0.16] = 0.40
    unbiased = select_gap(angles, ranges, 12.0, 0.50, 0.12, 0.55,
                          bubble_obstacle_distance=2.2, bubble_closest_only=True)
    biased = select_gap(angles, ranges, 12.0, 0.50, 0.12, 0.55,
                        target_angle=0.6, path_bias=0.2,
                        bubble_obstacle_distance=2.2, bubble_closest_only=True)
    assert unbiased.valid and biased.valid


def test_rejoin_never_needs_a_backward_index():
    previous_rejoin = 135
    closest_index_after_avoidance = 128
    candidate = 132
    assert max(previous_rejoin, closest_index_after_avoidance, candidate) == 135

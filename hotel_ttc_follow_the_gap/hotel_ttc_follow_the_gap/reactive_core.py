"""ROS-independent primitives used by the reactive avoidance supervisor.

The helpers deliberately keep the state machine and Follow-The-Gap selection
testable without Gazebo or a ROS graph.  They do not issue robot commands.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable

import numpy as np


class ReactiveState(str, Enum):
    TRACKING = 'TRACKING'
    AVOIDING = 'AVOIDING'
    REJOINING = 'REJOINING'


@dataclass(frozen=True)
class GapResult:
    angle: float = 0.0
    width: float = 0.0
    valid: bool = False


@dataclass(frozen=True)
class GapCandidate:
    start_index: int
    end_index: int
    start_angle: float
    end_angle: float
    angular_width: float
    representative_range: float
    physical_width: float
    accepted: bool
    rejection_reason: str


@dataclass(frozen=True)
class GapDiagnostics:
    bubble_removed_beams: int
    candidates: tuple[GapCandidate, ...]

    @property
    def candidate_gap_count(self) -> int:
        return len(self.candidates)

    @property
    def valid_gap_count(self) -> int:
        return sum(candidate.accepted for candidate in self.candidates)


class ReactiveStateMachine:
    """Hysteretic TRACKING → AVOIDING → REJOINING state machine."""

    def __init__(self, activation_s: float, clear_s: float, rejoin_s: float,
                 min_avoiding_s: float) -> None:
        self.activation_s = max(0.0, activation_s)
        self.clear_s = max(0.0, clear_s)
        self.rejoin_s = max(0.0, rejoin_s)
        self.min_avoiding_s = max(0.0, min_avoiding_s)
        self.state = ReactiveState.TRACKING
        self._blocking_since: float | None = None
        self._clear_since: float | None = None
        self._stable_since: float | None = None
        self._avoiding_since: float | None = None

    def update(self, now: float, blocking: bool, path_clear: bool,
               rejoin_aligned: bool) -> tuple[ReactiveState, bool]:
        previous = self.state
        if self.state == ReactiveState.TRACKING:
            self._blocking_since = now if blocking and self._blocking_since is None else self._blocking_since
            if not blocking:
                self._blocking_since = None
            if self._blocking_since is not None and now - self._blocking_since >= self.activation_s:
                self.state = ReactiveState.AVOIDING
                self._avoiding_since = now
                self._clear_since = None
        elif self.state == ReactiveState.AVOIDING:
            clear_candidate = path_clear and not blocking
            self._clear_since = now if clear_candidate and self._clear_since is None else self._clear_since
            if not clear_candidate:
                self._clear_since = None
            avoiding_long_enough = self._avoiding_since is not None and now - self._avoiding_since >= self.min_avoiding_s
            if (avoiding_long_enough and self._clear_since is not None
                    and now - self._clear_since >= self.clear_s):
                self.state = ReactiveState.REJOINING
                self._stable_since = None
        else:
            self._stable_since = now if rejoin_aligned and self._stable_since is None else self._stable_since
            if not rejoin_aligned:
                self._stable_since = None
            if self._stable_since is not None and now - self._stable_since >= self.rejoin_s:
                self.state = ReactiveState.TRACKING
                self._blocking_since = None
                self._clear_since = None
        return self.state, self.state != previous


def reactive_preview(speed: float, base: float, speed_gain: float, minimum: float,
                     maximum: float, deceleration: float, latency: float,
                     front_extent: float, safety_margin: float) -> float:
    """Preview covers both speed scaling and the physical braking envelope."""
    v = max(0.0, speed)
    dynamic = base + speed_gain * v
    braking = v * v / max(2.0 * deceleration, 1e-6) + latency * v + front_extent + safety_margin
    return min(maximum, max(minimum, dynamic, braking))


def point_segment_distance(point: tuple[float, float], first: tuple[float, float],
                           second: tuple[float, float]) -> tuple[float, float]:
    """Return perpendicular distance and normalized longitudinal projection."""
    px, py = point
    ax, ay = first
    bx, by = second
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay), 0.0
    projection = ((px - ax) * dx + (py - ay) * dy) / length_sq
    projection = min(1.0, max(0.0, projection))
    return math.hypot(px - (ax + projection * dx), py - (ay + projection * dy)), projection


def select_gap(angles: np.ndarray, ranges: np.ndarray, range_max: float,
               bubble_radius: float, minimum_clearance: float,
               minimum_width: float, target_angle: float = 0.0,
               path_bias: float = 0.0, progress_bias: float = 0.0,
               bubble_obstacle_distance: float | None = None,
               bubble_closest_only: bool = False) -> GapResult:
    return analyze_gap(angles, ranges, range_max, bubble_radius, minimum_clearance,
                       minimum_width, target_angle, path_bias, progress_bias,
                       bubble_obstacle_distance, bubble_closest_only)[0]


def analyze_gap(angles: np.ndarray, ranges: np.ndarray, range_max: float,
                bubble_radius: float, minimum_clearance: float,
                minimum_width: float, target_angle: float = 0.0,
                path_bias: float = 0.0, progress_bias: float = 0.0,
                bubble_obstacle_distance: float | None = None,
                bubble_closest_only: bool = False) -> tuple[GapResult, GapDiagnostics]:
    """Follow-The-Gap selection shared by the original FTG and the supervisor.

    Nearby returns are expanded into a bubble.  Candidate gaps must have a
    physical width at their shallowest depth, not only an angular width.
    """
    if angles.size == 0 or ranges.size == 0:
        return GapResult(), GapDiagnostics(0, ())
    work = np.asarray(ranges, dtype=float).copy()
    work[~np.isfinite(work)] = range_max
    work = np.clip(work, 0.0, range_max)
    source = work.copy()
    bubble_distance = bubble_radius if bubble_obstacle_distance is None else bubble_obstacle_distance
    before_bubble = work > minimum_clearance
    bubble_indices = np.flatnonzero((source > 0.03) & (source < bubble_distance))
    # A scan surface yields many neighbouring beams.  Expanding a full bubble
    # around every one of them can erase an entire FOV.  FTG's reactive mode
    # therefore expands the nearest blocking return once.
    if bubble_closest_only and bubble_indices.size:
        bubble_indices = np.array([bubble_indices[np.argmin(source[bubble_indices])]])
    for index in bubble_indices:
        angular_radius = math.atan2(bubble_radius, max(source[index], 1e-6))
        work[np.abs(angles - angles[index]) <= angular_radius] = 0.0
    safe = work > minimum_clearance
    best = GapResult()
    candidates: list[GapCandidate] = []
    start: int | None = None
    for index, is_safe in enumerate(np.r_[safe, False]):
        if is_safe and start is None:
            start = index
        elif not is_safe and start is not None:
            end = index - 1
            center = (start + end) // 2
            depth = float(np.min(work[start:end + 1]))
            angular_width = abs(float(angles[end] - angles[start])) if end > start else 0.0
            width = 2.0 * depth * math.sin(angular_width / 2.0)
            accepted = width >= minimum_width
            candidates.append(GapCandidate(
                start, end, float(angles[start]), float(angles[end]), angular_width,
                depth, width, accepted,
                '' if accepted else f'physical_width={width:.3f}<minimum_gap_width={minimum_width:.3f}',
            ))
            if accepted:
                angle = float(angles[center])
                clearance_score = min(1.0, float(np.mean(work[start:end + 1])) / max(range_max, 1e-6))
                width_score = min(1.0, width / max(2.0 * minimum_width, 1e-6))
                alignment = 0.5 * (1.0 + math.cos(angle - target_angle))
                forward = 0.5 * (1.0 + math.cos(angle))
                score = (0.55 * clearance_score + 0.25 * width_score
                         + path_bias * alignment + progress_bias * forward)
                previous_score = getattr(best, '_score', -math.inf)
                if score > previous_score:
                    best = GapResult(angle=angle, width=width, valid=True)
                    object.__setattr__(best, '_score', score)
            start = None
    return best, GapDiagnostics(int(np.count_nonzero(before_bubble & ~safe)), tuple(candidates))


def safe_gap_command(angle: float, maximum_speed: float, maximum_omega: float,
                     steering_gain: float, steering_slowdown: float,
                     wheel_linear_limit: float, half_track: float) -> tuple[float, float]:
    """Return an FTG command that respects differential-wheel capability."""
    omega = min(maximum_omega, max(-maximum_omega, steering_gain * angle))
    desired_speed = maximum_speed / (1.0 + steering_slowdown * abs(omega))
    speed = min(desired_speed, max(0.0, wheel_linear_limit - half_track * abs(omega)))
    return speed, omega

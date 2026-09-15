"""ROS-independent differential-drive path-tracking mathematics.

The LQR model uses the unicycle error system in the instantaneous reference
frame.  Keeping it independent of ROS makes the equations unit-testable and
also powers the explicitly labelled offline kinematic benchmark.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Sequence, Tuple

import numpy as np


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class TrackingCommand:
    linear_velocity: float
    angular_velocity: float
    reference: Pose2D
    reference_index: int
    cross_track_error: float
    heading_error: float
    distance_to_goal: float
    saturated_v: bool = False
    saturated_omega: bool = False
    goal_reached: bool = False
    closest_index: int = 0
    lookahead_distance: float = 0.0


def headings_from_points(points: Sequence[Tuple[float, float]], final_yaw: float = 0.0) -> List[Pose2D]:
    """Build a pose path from xy points; retained yaw is used at the end."""
    output: List[Pose2D] = []
    yaw = final_yaw
    for index, (x, y) in enumerate(points):
        if index + 1 < len(points):
            nx, ny = points[index + 1]
            if math.hypot(nx - x, ny - y) > 1e-9:
                yaw = math.atan2(ny - y, nx - x)
        output.append(Pose2D(x, y, yaw))
    return output


def nearest_forward_index(
        path: Sequence[Pose2D],
        pose: Pose2D,
        last_index: int,
        max_arc_length: float | None = None,
) -> int:
    """Nearest waypoint no farther back than ``last_index``.

    Monotonic progress avoids reacquiring a waypoint after the robot has
    passed it, which is important for paths that fold back close to themselves.
    When ``max_arc_length`` is set, the search is also bounded ahead along the
    path.  This prevents a repeated multi-lap route from matching a geometrically
    close copy belonging to a future lap.
    """
    if not path:
        raise ValueError("path is empty")
    begin = min(max(0, last_index), len(path) - 1)
    end = len(path)
    if max_arc_length is not None:
        limit = max(0.0, float(max_arc_length))
        end = begin + 1
        travelled = 0.0
        while end < len(path):
            segment = math.hypot(
                path[end].x - path[end - 1].x,
                path[end].y - path[end - 1].y,
            )
            if travelled + segment > limit:
                break
            travelled += segment
            end += 1
    return min(
        range(begin, end),
        key=lambda i: (path[i].x - pose.x) ** 2 + (path[i].y - pose.y) ** 2,
    )


def lookahead_index(path: Sequence[Pose2D], pose: Pose2D, first_index: int, distance: float) -> int:
    for index in range(first_index, len(path)):
        if math.hypot(path[index].x - pose.x, path[index].y - pose.y) >= distance:
            return index
    return len(path) - 1


def local_errors(pose: Pose2D, reference: Pose2D) -> Tuple[float, float, float]:
    """Actual-minus-reference error represented in the reference frame."""
    dx, dy = pose.x - reference.x, pose.y - reference.y
    c, s = math.cos(reference.yaw), math.sin(reference.yaw)
    e_x = c * dx + s * dy
    e_y = -s * dx + c * dy
    e_theta = wrap_to_pi(pose.yaw - reference.yaw)
    return e_x, e_y, e_theta


def target_in_robot_frame(pose: Pose2D, target: Pose2D) -> Tuple[float, float]:
    """Transform a global/path-frame target into the robot body frame."""
    dx, dy = target.x - pose.x, target.y - pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return c * dx + s * dy, -s * dx + c * dy


def reference_curvature(path: Sequence[Pose2D], index: int) -> float:
    """Estimate curvature from consecutive headings and arc length."""
    if len(path) < 2:
        return 0.0
    left = max(0, index - 1)
    right = min(len(path) - 1, index + 1)
    ds = math.hypot(path[right].x - path[left].x, path[right].y - path[left].y)
    if ds < 1e-6:
        return 0.0
    return wrap_to_pi(path[right].yaw - path[left].yaw) / ds


def discrete_error_model(v_ref: float, omega_ref: float, dt: float) -> Tuple[np.ndarray, np.ndarray]:
    """Forward-Euler discrete linearization for e=[e_x,e_y,e_theta].

    With e in the reference frame and delta-u = [v-v_ref, omega-omega_ref],
    the continuous model is:

      e_x_dot =  omega_ref e_y + delta_v
      e_y_dot = -omega_ref e_x + v_ref e_theta
      e_theta_dot = delta_omega.
    """
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    # ``e`` is actual-minus-reference in the *reference* frame.  The frame
    # rotates at omega_ref, so d(R_ref.T)/dt = -omega_ref S R_ref.T.
    a_cont = np.array([[0.0, omega_ref, 0.0], [-omega_ref, 0.0, v_ref], [0.0, 0.0, 0.0]])
    b_cont = np.array([[1.0, 0.0], [0.0, 0.0], [0.0, 1.0]])
    return np.eye(3) + dt * a_cont, dt * b_cont


def solve_discrete_lqr(a_matrix: np.ndarray, b_matrix: np.ndarray, q_matrix: np.ndarray,
                       r_matrix: np.ndarray, iterations: int = 200, tolerance: float = 1e-9) -> np.ndarray:
    """Solve the discrete Riccati equation and return K for delta-u = -K e.

    The fixed-point implementation is intentional: scipy is available in the
    development image but is not made a new runtime dependency of the package.
    """
    p_matrix = np.asarray(q_matrix, dtype=float).copy()
    for _ in range(iterations):
        inverse_term = np.linalg.inv(r_matrix + b_matrix.T @ p_matrix @ b_matrix)
        next_p = a_matrix.T @ p_matrix @ a_matrix - (
            a_matrix.T @ p_matrix @ b_matrix @ inverse_term @ b_matrix.T @ p_matrix @ a_matrix
        ) + q_matrix
        if not np.all(np.isfinite(next_p)):
            raise FloatingPointError("non-finite Riccati iterate")
        if np.max(np.abs(next_p - p_matrix)) < tolerance:
            p_matrix = next_p
            break
        p_matrix = next_p
    return np.linalg.solve(r_matrix + b_matrix.T @ p_matrix @ b_matrix, b_matrix.T @ p_matrix @ a_matrix)


class LQRTracker:
    """Time-varying discrete LQR path tracker for the unicycle model."""

    def __init__(
        self,
        control_dt: float = 0.04,
        v_nominal: float = 0.45,
        min_speed: float = 0.05,
        max_speed: float = 0.50,
        max_omega: float = 2.0,
        goal_tolerance: float = 0.25,
        yaw_tolerance: float = math.radians(12.0),
        goal_yaw_gain: float = 1.5,
        goal_progress_fraction: float = 0.95,
        lookahead_distance: float = 0.25,
        q_diagonal: Sequence[float] = (1.0, 6.0, 3.0),
        r_diagonal: Sequence[float] = (0.8, 0.6),
    ) -> None:
        self.dt = max(float(control_dt), 1e-3)
        self.v_nominal = max(0.0, float(v_nominal))
        self.min_speed = max(0.0, float(min_speed))
        self.max_speed = max(self.min_speed, float(max_speed))
        self.max_omega = max(0.0, float(max_omega))
        self.goal_tolerance = max(0.0, float(goal_tolerance))
        self.yaw_tolerance = max(0.0, float(yaw_tolerance))
        self.goal_yaw_gain = max(0.0, float(goal_yaw_gain))
        self.goal_progress_fraction = clamp(float(goal_progress_fraction), 0.0, 1.0)
        self.lookahead_distance = max(0.0, float(lookahead_distance))
        if len(q_diagonal) != 3 or len(r_diagonal) != 2:
            raise ValueError("LQR Q needs 3 values and R needs 2")
        self.q_matrix = np.diag(np.maximum(np.asarray(q_diagonal, dtype=float), 1e-9))
        self.r_matrix = np.diag(np.maximum(np.asarray(r_diagonal, dtype=float), 1e-9))
        self.last_index = 0

    def reset(self) -> None:
        self.last_index = 0

    def command(self, path: Sequence[Pose2D], pose: Pose2D) -> TrackingCommand:
        if not path:
            raise ValueError("path is empty")
        goal = path[-1]
        goal_distance = math.hypot(goal.x - pose.x, goal.y - pose.y)
        goal_heading_error = wrap_to_pi(pose.yaw - goal.yaw)
        nearest = nearest_forward_index(path, pose, self.last_index)
        self.last_index = max(self.last_index, nearest)
        closed_path = len(path) > 2 and math.hypot(
            path[0].x - goal.x, path[0].y - goal.y) <= self.goal_tolerance
        minimum_goal_index = int(math.ceil((len(path) - 1) * self.goal_progress_fraction))
        may_finish = not closed_path or self.last_index >= minimum_goal_index
        if goal_distance <= self.goal_tolerance and may_finish:
            if abs(goal_heading_error) <= self.yaw_tolerance:
                return TrackingCommand(0.0, 0.0, goal, len(path) - 1, 0.0,
                                       goal_heading_error, goal_distance, goal_reached=True,
                                       closest_index=len(path) - 1)
            return TrackingCommand(
                0.0,
                clamp(-self.goal_yaw_gain * goal_heading_error, -self.max_omega, self.max_omega),
                goal, len(path) - 1, 0.0, goal_heading_error, goal_distance,
                closest_index=len(path) - 1,
            )
        # LQR is used for path segments.  Near the endpoint, repeatedly
        # switching between a reference just behind and just ahead of the
        # axle can make a forward-only robot orbit its goal.  A unicycle
        # terminal stabilizer turns toward the goal first, then advances at a
        # distance-proportional speed; the final-yaw branch above stops it.
        terminal_radius = max(2.0 * self.goal_tolerance, self.lookahead_distance)
        if goal_distance <= terminal_radius and may_finish:
            goal_bearing = math.atan2(goal.y - pose.y, goal.x - pose.x)
            bearing_error = wrap_to_pi(goal_bearing - pose.yaw)
            if abs(bearing_error) > self.yaw_tolerance:
                return TrackingCommand(
                    0.0,
                    clamp(1.5 * bearing_error, -self.max_omega, self.max_omega),
                    goal, len(path) - 1, 0.0, bearing_error, goal_distance,
                    closest_index=nearest, lookahead_distance=self.lookahead_distance,
                )
            raw_v = min(self.v_nominal, 0.9 * goal_distance)
            v_cmd = clamp(raw_v, self.min_speed, self.max_speed)
            return TrackingCommand(
                v_cmd,
                clamp(1.5 * bearing_error, -self.max_omega, self.max_omega),
                goal, len(path) - 1, 0.0, bearing_error, goal_distance,
                saturated_v=not math.isclose(raw_v, v_cmd, abs_tol=1e-10),
                closest_index=nearest, lookahead_distance=self.lookahead_distance,
            )
        index = lookahead_index(path, pose, nearest, self.lookahead_distance)
        reference = path[index]
        target_x, _ = target_in_robot_frame(pose, reference)
        e_x, e_y, e_theta = local_errors(pose, reference)
        # A forward-only differential tracker cannot recover a reference that
        # is entirely behind it by continuing to drive forward.  Turn first,
        # then reacquire it on a following control cycle.
        if target_x <= 0.0:
            _, target_y = target_in_robot_frame(pose, reference)
            target_heading = math.atan2(target_y, target_x)
            return TrackingCommand(
                0.0, clamp(target_heading, -self.max_omega, self.max_omega),
                reference, index, e_y, e_theta, goal_distance,
                closest_index=nearest, lookahead_distance=self.lookahead_distance,
            )
        v_ref = self.v_nominal * min(1.0, goal_distance / max(self.goal_tolerance * 3.0, 1e-6))
        omega_ref = clamp(v_ref * reference_curvature(path, index), -self.max_omega, self.max_omega)
        a_matrix, b_matrix = discrete_error_model(v_ref, omega_ref, self.dt)
        gain = solve_discrete_lqr(a_matrix, b_matrix, self.q_matrix, self.r_matrix)
        delta_u = -gain @ np.array([e_x, e_y, e_theta])
        raw_v = v_ref + float(delta_u[0])
        raw_omega = omega_ref + float(delta_u[1])
        v_cmd = clamp(raw_v, self.min_speed, self.max_speed)
        omega_cmd = clamp(raw_omega, -self.max_omega, self.max_omega)
        return TrackingCommand(
            v_cmd, omega_cmd, reference, index, e_y, e_theta, goal_distance,
            saturated_v=not math.isclose(raw_v, v_cmd, abs_tol=1e-10),
            saturated_omega=not math.isclose(raw_omega, omega_cmd, abs_tol=1e-10),
            closest_index=nearest, lookahead_distance=self.lookahead_distance,
        )


class PurePursuitReference:
    """Offline mathematical counterpart of the existing Pure Pursuit baseline."""

    def __init__(self, lookahead_l0: float = 0.6, lookahead_kv: float = 1.3,
                 lookahead_min: float = 0.3, lookahead_max: float = 3.5,
                 v_nominal: float = 0.45, max_speed: float = 0.5,
                 max_omega: float = 2.0, max_curvature: float = 1.6,
                 goal_tolerance: float = 0.25, yaw_tolerance: float = math.radians(12.0),
                 goal_yaw_gain: float = 1.5, slow_radius: float = 1.2,
                 min_speed: float = 0.05, goal_progress_fraction: float = 0.95,
                 max_progress_jump_distance: float = 1.5) -> None:
        self.l0, self.kv, self.lmin, self.lmax = lookahead_l0, lookahead_kv, lookahead_min, lookahead_max
        self.v_nominal, self.max_speed, self.max_omega = v_nominal, max_speed, max_omega
        self.max_curvature, self.goal_tolerance, self.slow_radius = max_curvature, goal_tolerance, slow_radius
        self.yaw_tolerance, self.goal_yaw_gain, self.min_speed = yaw_tolerance, goal_yaw_gain, min_speed
        self.goal_progress_fraction = clamp(float(goal_progress_fraction), 0.0, 1.0)
        self.max_progress_jump_distance = max(0.0, float(max_progress_jump_distance))
        self.last_closest_index = 0
        self.last_target_index = 0
        self.previous_omega = 0.0

    def reset(self) -> None:
        self.last_closest_index, self.last_target_index, self.previous_omega = 0, 0, 0.0

    def command(self, path: Sequence[Pose2D], pose: Pose2D) -> TrackingCommand:
        if not path:
            raise ValueError("path is empty")
        goal = path[-1]
        goal_distance = math.hypot(goal.x - pose.x, goal.y - pose.y)
        goal_heading_error = wrap_to_pi(pose.yaw - goal.yaw)
        nearest = nearest_forward_index(
            path, pose, self.last_closest_index,
            max_arc_length=self.max_progress_jump_distance)
        self.last_closest_index = max(self.last_closest_index, nearest)
        minimum_goal_index = int(math.ceil((len(path) - 1) * self.goal_progress_fraction))
        # The final XY can also be visited during an earlier lap (or on an
        # open route that crosses itself).  Finishing based on distance alone
        # would stop a multi-lap mission on that first visit.  Always require
        # progress through the configured final fraction of the full path.
        may_finish = self.last_closest_index >= minimum_goal_index
        if goal_distance <= self.goal_tolerance and may_finish:
            if abs(goal_heading_error) <= self.yaw_tolerance:
                return TrackingCommand(0.0, 0.0, goal, len(path) - 1, 0.0,
                                       goal_heading_error, goal_distance, goal_reached=True,
                                       closest_index=len(path) - 1)
            return TrackingCommand(
                0.0,
                clamp(-self.goal_yaw_gain * goal_heading_error, -self.max_omega, self.max_omega),
                goal, len(path) - 1, 0.0, goal_heading_error, goal_distance,
                closest_index=len(path) - 1,
            )
        v_initial = self.v_nominal * min(1.0, goal_distance / max(self.slow_radius, 1e-6))
        lookahead = clamp(self.l0 + self.kv * abs(v_initial), self.lmin, self.lmax)
        # Once the end of an open path is close, the pure-pursuit circle can
        # become tangent to the goal and orbit forever.  The final approach
        # is a unicycle stabilizer: turn toward the goal, advance only when
        # it is in front, then use the final-yaw branch above.
        terminal_radius = max(2.0 * self.goal_tolerance, lookahead)
        if goal_distance <= terminal_radius and may_finish:
            goal_bearing = math.atan2(goal.y - pose.y, goal.x - pose.x)
            bearing_error = wrap_to_pi(goal_bearing - pose.yaw)
            if abs(bearing_error) > self.yaw_tolerance:
                return TrackingCommand(
                    0.0,
                    clamp(self.goal_yaw_gain * bearing_error, -self.max_omega, self.max_omega),
                    goal, len(path) - 1, 0.0, bearing_error, goal_distance,
                    closest_index=nearest, lookahead_distance=lookahead,
                )
            raw_v = min(self.v_nominal, 0.9 * goal_distance)
            v_cmd = clamp(raw_v, self.min_speed, self.max_speed)
            return TrackingCommand(
                v_cmd,
                clamp(self.goal_yaw_gain * bearing_error, -self.max_omega, self.max_omega),
                goal, len(path) - 1, 0.0, bearing_error, goal_distance,
                saturated_v=not math.isclose(raw_v, v_cmd, abs_tol=1e-10),
                closest_index=nearest, lookahead_distance=lookahead,
            )
        index = lookahead_index(path, pose, nearest, lookahead)
        self.last_target_index = max(self.last_target_index, index)
        reference = path[index]
        body_x, body_y = target_in_robot_frame(pose, reference)
        _, cte, heading = local_errors(pose, reference)
        if body_x <= 0.0:
            target_heading = math.atan2(body_y, body_x)
            return TrackingCommand(
                0.0, clamp(target_heading, -self.max_omega, self.max_omega),
                reference, index, cte, heading, goal_distance,
                closest_index=nearest, lookahead_distance=lookahead,
            )
        curvature = clamp(2.0 * body_y / (lookahead * lookahead + 1e-6), -self.max_curvature, self.max_curvature)
        raw_v = self.v_nominal * min(1.0, goal_distance / max(self.slow_radius, 1e-6))
        raw_v /= 1.0 + 1.5 * abs(body_y)
        v_cmd = clamp(raw_v, self.min_speed, self.max_speed)
        raw_omega = v_cmd * curvature
        limited_omega = clamp(raw_omega, -self.max_omega, self.max_omega)
        omega = 0.5 * limited_omega + 0.5 * self.previous_omega
        self.previous_omega = omega
        return TrackingCommand(v_cmd, omega, reference, index, cte, heading, goal_distance,
                               saturated_v=not math.isclose(raw_v, v_cmd, abs_tol=1e-10),
                               saturated_omega=not math.isclose(raw_omega, limited_omega, abs_tol=1e-10),
                               closest_index=nearest, lookahead_distance=lookahead)


class AdaptivePurePursuitReference:
    """Pure Pursuit with preview speed planning and physical rate limits.

    The original ``PurePursuitReference`` is intentionally retained unchanged.
    This controller uses the same monotonic path-progress safeguards, while
    exposing a compact, interpretable set of terms:

    * speed falls as the maximum curvature in a future arc-length window rises;
    * lookahead grows with commanded speed and shrinks in curved sections;
    * lateral/heading error reduce speed multiplicatively; and
    * linear and angular output changes are rate limited.
    """

    def __init__(
        self,
        lookahead_min: float = 0.30,
        lookahead_base: float = 0.60,
        lookahead_max: float = 1.20,
        lookahead_gain: float = 1.30,
        lookahead_curvature_gain: float = 0.45,
        min_speed: float = 0.05,
        nominal_speed: float | None = None,
        max_speed: float = 0.50,
        max_omega: float = 2.0,
        max_curvature: float = 1.6,
        adaptive_speed_enabled: bool = True,
        curvature_speed_gain: float = 1.5,
        curvature_preview_distance: float = 1.0,
        lateral_error_speed_gain: float = 1.5,
        lateral_error_slowdown_threshold: float = 0.15,
        heading_error_speed_gain: float = 0.45,
        heading_error_slowdown_threshold: float = math.radians(10.0),
        acceleration_limit: float = 0.55,
        deceleration_limit: float = 0.90,
        angular_acceleration_limit: float = 3.0,
        wheel_radius: float = 0.05,
        wheel_separation: float = 0.44,
        wheel_max_angular_velocity: float = 20.0,
        goal_tolerance: float = 0.25,
        yaw_tolerance: float = math.radians(12.0),
        goal_yaw_gain: float = 1.5,
        goal_progress_fraction: float = 0.95,
        max_progress_jump_distance: float = 1.5,
        slow_radius: float = 1.2,
    ) -> None:
        self.lmin = max(0.05, float(lookahead_min))
        self.lbase = clamp(float(lookahead_base), self.lmin, max(self.lmin, float(lookahead_max)))
        self.lmax = max(self.lmin, float(lookahead_max))
        self.lookahead_gain = max(0.0, float(lookahead_gain))
        self.lookahead_curvature_gain = max(0.0, float(lookahead_curvature_gain))
        self.min_speed = max(0.0, float(min_speed))
        self.wheel_radius = max(1e-6, float(wheel_radius))
        self.half_track = max(1e-6, 0.5 * float(wheel_separation))
        self.wheel_linear_limit = max(0.0, self.wheel_radius * float(wheel_max_angular_velocity))
        self.requested_max_speed = max(self.min_speed, float(max_speed))
        self.max_speed = min(self.requested_max_speed, self.wheel_linear_limit)
        requested_nominal = (
            self.requested_max_speed if nominal_speed is None else float(nominal_speed))
        self.requested_nominal_speed = max(self.min_speed, requested_nominal)
        self.nominal_speed = min(self.requested_nominal_speed, self.max_speed)
        self.requested_max_omega = max(0.0, float(max_omega))
        self.physical_max_omega = self.wheel_linear_limit / self.half_track
        self.max_omega = min(self.requested_max_omega, self.physical_max_omega)
        self.max_curvature = max(0.0, float(max_curvature))
        self.adaptive_speed_enabled = bool(adaptive_speed_enabled)
        self.curvature_speed_gain = max(0.0, float(curvature_speed_gain))
        self.preview_distance = max(0.0, float(curvature_preview_distance))
        self.lateral_error_speed_gain = max(0.0, float(lateral_error_speed_gain))
        self.lateral_error_slowdown_threshold = max(0.0, float(lateral_error_slowdown_threshold))
        self.heading_error_speed_gain = max(0.0, float(heading_error_speed_gain))
        self.heading_error_slowdown_threshold = max(0.0, float(heading_error_slowdown_threshold))
        self.acceleration_limit = max(0.0, float(acceleration_limit))
        self.deceleration_limit = max(0.0, float(deceleration_limit))
        self.angular_acceleration_limit = max(0.0, float(angular_acceleration_limit))
        self.goal_tolerance = max(0.0, float(goal_tolerance))
        self.yaw_tolerance = max(0.0, float(yaw_tolerance))
        self.goal_yaw_gain = max(0.0, float(goal_yaw_gain))
        self.goal_progress_fraction = clamp(float(goal_progress_fraction), 0.0, 1.0)
        self.max_progress_jump_distance = max(0.0, float(max_progress_jump_distance))
        self.slow_radius = max(1e-3, float(slow_radius))
        self._path_key: tuple[int, int] | None = None
        self._arc_lengths: list[float] = []
        self._path_curvatures: list[float] = []
        self.last_target_speed = 0.0
        self.last_curvature = 0.0
        self.last_path_curvature = 0.0
        self.last_future_curvature = 0.0
        self.last_speed_limit_curvature = 0.0
        self.last_speed_limit_preview = 0.0
        self.last_speed_limit_omega = 0.0
        self.last_speed_limit_lateral_error = 0.0
        self.last_speed_limit_heading_error = 0.0
        self.last_speed_limit_wheel = 0.0
        self.last_closest_index = 0
        self.last_target_index = 0
        self.previous_linear_velocity = 0.0
        self.previous_angular_velocity = 0.0

    def reset(self) -> None:
        self.last_closest_index = 0
        self.last_target_index = 0
        self.previous_linear_velocity = 0.0
        self.previous_angular_velocity = 0.0
        self.last_target_speed = 0.0
        self.last_curvature = 0.0
        self.last_path_curvature = 0.0
        self.last_future_curvature = 0.0
        self.last_speed_limit_curvature = 0.0
        self.last_speed_limit_preview = 0.0
        self.last_speed_limit_omega = 0.0
        self.last_speed_limit_lateral_error = 0.0
        self.last_speed_limit_heading_error = 0.0
        self.last_speed_limit_wheel = 0.0

    def _update_path_profile(self, path: Sequence[Pose2D]) -> None:
        key = (id(path), len(path))
        if self._path_key == key:
            return
        self._path_key = key
        self._arc_lengths = [0.0]
        for previous, current in zip(path, path[1:]):
            self._arc_lengths.append(
                self._arc_lengths[-1] + math.hypot(current.x - previous.x, current.y - previous.y))
        self._path_curvatures = [reference_curvature(path, index) for index in range(len(path))]

    def _preview_curvature(self, start_index: int) -> float:
        if not self._arc_lengths:
            return 0.0
        start_index = min(max(0, start_index), len(self._arc_lengths) - 1)
        end_distance = self._arc_lengths[start_index] + self.preview_distance
        result = 0.0
        for index in range(start_index, len(self._arc_lengths)):
            if self._arc_lengths[index] > end_distance:
                break
            result = max(result, abs(self._path_curvatures[index]))
        return result

    def progress_fraction(self, index: int) -> float:
        """Arc-length progress for diagnostics, constrained to [0, 1]."""
        if not self._arc_lengths or self._arc_lengths[-1] <= 1e-9:
            return 0.0
        safe_index = min(max(0, index), len(self._arc_lengths) - 1)
        return self._arc_lengths[safe_index] / self._arc_lengths[-1]

    def _curvature_speed_limit(self, curvature: float) -> float:
        """Interpretable speed ceiling for a path-geometry curvature."""
        if not self.adaptive_speed_enabled:
            return self.max_speed
        return self.max_speed / (1.0 + self.curvature_speed_gain * abs(curvature))

    def _error_speed_limit(self, error: float, threshold: float, gain: float) -> float:
        """Progressive error reduction; a zero gain cleanly disables it."""
        if not self.adaptive_speed_enabled or gain <= 0.0:
            return self.max_speed
        excess = max(0.0, abs(error) - threshold)
        return self.max_speed / (1.0 + gain * excess)

    def _omega_speed_limit(self, curvature: float) -> float:
        """Linear-speed ceiling needed to keep |omega| within its usable cap."""
        if abs(curvature) <= 1e-9:
            return self.max_speed
        return min(self.max_speed, self.max_omega / abs(curvature))

    def _wheel_speed_limit(self, curvature: float) -> float:
        """Differential-wheel bound for v +/- omega * track/2."""
        return self.wheel_linear_limit / (1.0 + self.half_track * abs(curvature))

    @staticmethod
    def _rate_limit(
        target: float, previous: float, up_limit: float, down_limit: float, dt: float,
    ) -> float:
        if dt <= 0.0:
            return previous
        delta = target - previous
        if delta >= 0.0:
            return previous + min(delta, up_limit * dt)
        return previous + max(delta, -down_limit * dt)

    def _linear_rate_limit(self, target: float, dt: float) -> float:
        return self._rate_limit(
            target, self.previous_linear_velocity,
            self.acceleration_limit, self.deceleration_limit, dt)

    def _angular_rate_limit(self, target: float, dt: float) -> float:
        if self.angular_acceleration_limit <= 0.0:
            return target
        return self._rate_limit(
            target, self.previous_angular_velocity,
            self.angular_acceleration_limit, self.angular_acceleration_limit, dt)

    def _final_command(
        self, raw_v: float, raw_omega: float, reference: Pose2D, reference_index: int,
        cte: float, heading: float, goal_distance: float, closest: int,
        lookahead: float, dt: float, goal_reached: bool = False,
    ) -> TrackingCommand:
        if goal_reached:
            self.previous_linear_velocity = 0.0
            self.previous_angular_velocity = 0.0
            self.last_target_speed = 0.0
            return TrackingCommand(0.0, 0.0, reference, reference_index, cte, heading,
                                   goal_distance, goal_reached=True, closest_index=closest,
                                   lookahead_distance=lookahead)
        raw_v = clamp(raw_v, 0.0, self.max_speed)
        # v +/- omega * half_track must never exceed wheel_linear_limit.
        turning_in_place = raw_v <= 1e-8
        curvature = 0.0 if turning_in_place else raw_omega / raw_v
        wheel_speed_cap = self._wheel_speed_limit(curvature)
        v_target = min(raw_v, wheel_speed_cap)
        v_command = self._linear_rate_limit(v_target, dt)
        omega_unlimited = raw_omega if turning_in_place else v_command * curvature
        omega_wheel_cap = max(0.0, (self.wheel_linear_limit - abs(v_command)) / self.half_track)
        omega_target = clamp(omega_unlimited, -min(self.max_omega, omega_wheel_cap),
                             min(self.max_omega, omega_wheel_cap))
        omega_command = self._angular_rate_limit(omega_target, dt)
        # A rate-limited omega can still be infeasible after a large speed drop.
        omega_command = clamp(omega_command, -omega_wheel_cap, omega_wheel_cap)
        self.previous_linear_velocity = v_command
        self.previous_angular_velocity = omega_command
        self.last_target_speed = v_target
        self.last_curvature = curvature
        return TrackingCommand(
            v_command, omega_command, reference, reference_index, cte, heading,
            goal_distance,
            saturated_v=not math.isclose(raw_v, v_target, abs_tol=1e-10),
            saturated_omega=not math.isclose(raw_omega, omega_target, abs_tol=1e-10),
            closest_index=closest, lookahead_distance=lookahead,
        )

    def command(self, path: Sequence[Pose2D], pose: Pose2D, dt: float = 0.04) -> TrackingCommand:
        if not path:
            raise ValueError('path is empty')
        self._update_path_profile(path)
        goal = path[-1]
        goal_distance = math.hypot(goal.x - pose.x, goal.y - pose.y)
        goal_heading_error = wrap_to_pi(pose.yaw - goal.yaw)
        nearest = nearest_forward_index(
            path, pose, self.last_closest_index,
            max_arc_length=self.max_progress_jump_distance)
        self.last_closest_index = max(self.last_closest_index, nearest)
        minimum_goal_index = int(math.ceil((len(path) - 1) * self.goal_progress_fraction))
        may_finish = self.last_closest_index >= minimum_goal_index
        preview = self._preview_curvature(nearest)
        self.last_future_curvature = preview
        lookahead = clamp(
            (self.lbase + self.lookahead_gain * abs(self.previous_linear_velocity)) /
            (1.0 + self.lookahead_curvature_gain * preview), self.lmin, self.lmax)
        if goal_distance <= self.goal_tolerance and may_finish:
            if abs(goal_heading_error) <= self.yaw_tolerance:
                return self._final_command(0.0, 0.0, goal, len(path) - 1, 0.0,
                                           goal_heading_error, goal_distance, nearest,
                                           lookahead, dt, goal_reached=True)
            return self._final_command(0.0, self.goal_yaw_gain * -goal_heading_error,
                                       goal, len(path) - 1, 0.0, goal_heading_error,
                                       goal_distance, nearest, lookahead, dt)
        terminal_radius = max(2.0 * self.goal_tolerance, lookahead)
        if goal_distance <= terminal_radius and may_finish:
            bearing_error = wrap_to_pi(math.atan2(goal.y - pose.y, goal.x - pose.x) - pose.yaw)
            if abs(bearing_error) > self.yaw_tolerance:
                return self._final_command(0.0, self.goal_yaw_gain * bearing_error,
                                           goal, len(path) - 1, 0.0, bearing_error,
                                           goal_distance, nearest, lookahead, dt)
            return self._final_command(min(self.max_speed, 0.9 * goal_distance),
                                       self.goal_yaw_gain * bearing_error, goal, len(path) - 1,
                                       0.0, bearing_error, goal_distance, nearest, lookahead, dt)
        index = lookahead_index(path, pose, nearest, lookahead)
        self.last_target_index = max(self.last_target_index, index)
        reference = path[index]
        body_x, body_y = target_in_robot_frame(pose, reference)
        _, cte, heading = local_errors(pose, reference)
        if body_x <= 0.0:
            target_heading = math.atan2(body_y, body_x)
            return self._final_command(0.0, target_heading, reference, index, cte, heading,
                                       goal_distance, nearest, lookahead, dt)
        pursuit_curvature = clamp(
            2.0 * body_y / (lookahead * lookahead + 1e-6),
            -self.max_curvature, self.max_curvature)
        self.last_path_curvature = self._path_curvatures[nearest]
        self.last_speed_limit_curvature = self._curvature_speed_limit(pursuit_curvature)
        self.last_speed_limit_preview = self._curvature_speed_limit(preview)
        self.last_speed_limit_omega = self._omega_speed_limit(pursuit_curvature)
        self.last_speed_limit_lateral_error = self._error_speed_limit(
            cte, self.lateral_error_slowdown_threshold, self.lateral_error_speed_gain)
        self.last_speed_limit_heading_error = self._error_speed_limit(
            heading, self.heading_error_slowdown_threshold, self.heading_error_speed_gain)
        self.last_speed_limit_wheel = self._wheel_speed_limit(pursuit_curvature)
        geometric_target = min(
            self.nominal_speed,
            self.last_speed_limit_curvature,
            self.last_speed_limit_preview,
            self.last_speed_limit_omega,
            self.last_speed_limit_lateral_error,
            self.last_speed_limit_heading_error,
        )
        # ``min_speed`` prevents unnecessary crawl in benign geometry, but it
        # is never allowed to override the physical wheel or omega limits.
        raw_v = min(
            max(self.min_speed, geometric_target),
            self.last_speed_limit_omega,
            self.last_speed_limit_wheel,
            self.max_speed,
        )
        return self._final_command(raw_v, raw_v * pursuit_curvature, reference, index, cte,
                                   heading, goal_distance, nearest, lookahead, dt)

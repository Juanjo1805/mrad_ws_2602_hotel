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

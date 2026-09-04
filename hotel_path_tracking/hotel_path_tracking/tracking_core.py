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


def nearest_forward_index(path: Sequence[Pose2D], pose: Pose2D, last_index: int) -> int:
    """Nearest waypoint no farther back than ``last_index``.

    Monotonic progress avoids reacquiring a waypoint after the robot has
    passed it, which is important for paths that fold back close to themselves.
    """
    if not path:
        raise ValueError("path is empty")
    begin = min(max(0, last_index), len(path) - 1)
    return min(range(begin, len(path)), key=lambda i: (path[i].x - pose.x) ** 2 + (path[i].y - pose.y) ** 2)


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

      e_x_dot = -omega_ref e_y + delta_v
      e_y_dot =  omega_ref e_x + v_ref e_theta
      e_theta_dot = delta_omega.
    """
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    a_cont = np.array([[0.0, -omega_ref, 0.0], [omega_ref, 0.0, v_ref], [0.0, 0.0, 0.0]])
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
        if goal_distance <= self.goal_tolerance:
            return TrackingCommand(0.0, 0.0, goal, len(path) - 1, 0.0,
                                   wrap_to_pi(pose.yaw - goal.yaw), goal_distance, goal_reached=True)
        nearest = nearest_forward_index(path, pose, self.last_index)
        index = lookahead_index(path, pose, nearest, self.lookahead_distance)
        self.last_index = max(self.last_index, nearest)
        reference = path[index]
        e_x, e_y, e_theta = local_errors(pose, reference)
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
        )


class PurePursuitReference:
    """Offline mathematical counterpart of the existing Pure Pursuit baseline."""

    def __init__(self, lookahead_l0: float = 0.6, lookahead_kv: float = 1.3,
                 lookahead_min: float = 0.3, lookahead_max: float = 3.5,
                 v_nominal: float = 0.45, max_speed: float = 0.5,
                 max_omega: float = 2.0, max_curvature: float = 1.6,
                 goal_tolerance: float = 0.25, slow_radius: float = 1.2) -> None:
        self.l0, self.kv, self.lmin, self.lmax = lookahead_l0, lookahead_kv, lookahead_min, lookahead_max
        self.v_nominal, self.max_speed, self.max_omega = v_nominal, max_speed, max_omega
        self.max_curvature, self.goal_tolerance, self.slow_radius = max_curvature, goal_tolerance, slow_radius
        self.last_index = 0
        self.previous_omega = 0.0

    def reset(self) -> None:
        self.last_index, self.previous_omega = 0, 0.0

    def command(self, path: Sequence[Pose2D], pose: Pose2D) -> TrackingCommand:
        if not path:
            raise ValueError("path is empty")
        goal = path[-1]
        goal_distance = math.hypot(goal.x - pose.x, goal.y - pose.y)
        if goal_distance <= self.goal_tolerance:
            return TrackingCommand(0.0, 0.0, goal, len(path) - 1, 0.0,
                                   wrap_to_pi(pose.yaw - goal.yaw), goal_distance, goal_reached=True)
        v_initial = self.v_nominal * min(1.0, goal_distance / max(self.slow_radius, 1e-6))
        lookahead = clamp(self.l0 + self.kv * abs(v_initial), self.lmin, self.lmax)
        nearest = nearest_forward_index(path, pose, self.last_index)
        index = lookahead_index(path, pose, nearest, lookahead)
        self.last_index = max(self.last_index, index)
        reference = path[index]
        dx, dy = reference.x - pose.x, reference.y - pose.y
        body_y = -math.sin(pose.yaw) * dx + math.cos(pose.yaw) * dy
        body_x = math.cos(pose.yaw) * dx + math.sin(pose.yaw) * dy
        curvature = clamp(2.0 * body_y / (lookahead * lookahead + 1e-6), -self.max_curvature, self.max_curvature)
        raw_v = self.v_nominal / (1.0 + 1.5 * abs(body_y))
        v_cmd = clamp(raw_v, 0.2, self.max_speed)
        raw_omega = v_cmd * curvature
        omega = clamp(raw_omega, -self.max_omega, self.max_omega)
        omega = 0.5 * omega + 0.5 * self.previous_omega
        self.previous_omega = omega
        _, cte, heading = local_errors(pose, reference)
        return TrackingCommand(v_cmd, omega, reference, index, cte, heading, goal_distance,
                               saturated_v=not math.isclose(raw_v, v_cmd, abs_tol=1e-10),
                               saturated_omega=not math.isclose(raw_omega, omega, abs_tol=1e-10))

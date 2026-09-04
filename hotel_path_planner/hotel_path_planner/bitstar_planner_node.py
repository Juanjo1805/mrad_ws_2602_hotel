#!/usr/bin/env python3
"""
BIT* nonholonomic planner for ROS 2 in a single file.

Features included in this version:
- Informed sampling with the prolate ellipse used after the first solution is found.
- Tree pruning between batches.
- Rewiring for cost improvement.
- Adaptive connection radius.
- Nonholonomic local steering using a unicycle model.

Topics:
- Subscribes: /map, /planner/start, /initialpose, /goal_pose
- Publishes:  /bitstar/path, /bitstar/tree
"""

from __future__ import annotations

import heapq
import itertools
import math
import random
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Set, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


# -----------------------------------------------------------------------------
# Basic geometry and state types
# -----------------------------------------------------------------------------


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class State:
    x: float
    y: float
    yaw: float = 0.0

    def distance_xy(self, other: "State") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def yaw_error(self, other: "State") -> float:
        return abs(wrap_angle(self.yaw - other.yaw))


@dataclass
class TrajectorySegment:
    states: List[State] = field(default_factory=list)
    cost: float = math.inf


# -----------------------------------------------------------------------------
# Collision checking over OccupancyGrid
# -----------------------------------------------------------------------------


class OccupancyGridCollisionChecker:
    def __init__(
        self,
        grid: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        robot_radius: float,
        occupancy_threshold: int = 50,
        unknown_is_occupied: bool = True,
    ) -> None:
        self.grid = grid
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.robot_radius = robot_radius
        self.occupancy_threshold = occupancy_threshold
        self.unknown_is_occupied = unknown_is_occupied
        self.height, self.width = grid.shape
        self._inflation_offsets = self._build_inflation_offsets()

    def bounds(self) -> Tuple[float, float, float, float]:
        x_min = self.origin_x
        x_max = self.origin_x + self.width * self.resolution
        y_min = self.origin_y
        y_max = self.origin_y + self.height * self.resolution
        return x_min, x_max, y_min, y_max

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def is_state_valid(self, state: State) -> bool:
        gx, gy = self.world_to_grid(state.x, state.y)
        for ox, oy in self._inflation_offsets:
            cx = gx + ox
            cy = gy + oy
            if cx < 0 or cy < 0 or cx >= self.width or cy >= self.height:
                return False
            value = int(self.grid[cy, cx])
            if value < 0:
                if self.unknown_is_occupied:
                    return False
                continue
            if value >= self.occupancy_threshold:
                return False
        return True

    def is_trajectory_valid(self, states: Iterable[State]) -> bool:
        return all(self.is_state_valid(state) for state in states)

    def free_area(self) -> float:
        if self.unknown_is_occupied:
            free_mask = (self.grid >= 0) & (self.grid < self.occupancy_threshold)
        else:
            free_mask = self.grid < self.occupancy_threshold
        return float(np.count_nonzero(free_mask)) * (self.resolution ** 2)

    def _build_inflation_offsets(self) -> List[Tuple[int, int]]:
        radius_cells = int(math.ceil(self.robot_radius / self.resolution))
        offsets: List[Tuple[int, int]] = []
        for oy in range(-radius_cells, radius_cells + 1):
            for ox in range(-radius_cells, radius_cells + 1):
                if math.hypot(ox, oy) * self.resolution <= self.robot_radius:
                    offsets.append((ox, oy))
        return offsets or [(0, 0)]


# -----------------------------------------------------------------------------
# Search space, heuristic and informed sampling
# -----------------------------------------------------------------------------


class SearchSpace:
    """
    Search space in SE(2).

    The heuristic is the Euclidean distance in x-y, which is admissible for the
    nonholonomic local connection used by the planner.
    """

    def __init__(
        self,
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
        yaw_weight: float = 0.25,
        rng_seed: Optional[int] = None,
    ) -> None:
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.yaw_weight = max(1e-6, yaw_weight)
        self.rng = random.Random(rng_seed)

    def heuristic(self, a: State, b: State) -> float:
        return a.distance_xy(b)

    def edge_cost_lower_bound(self, a: State, b: State) -> float:
        return self.heuristic(a, b)

    def metric(self, a: State, b: State) -> float:
        dyaw = wrap_angle(a.yaw - b.yaw)
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (self.yaw_weight * dyaw) ** 2)

    def volume(self) -> float:
        dx = max(1e-6, self.x_max - self.x_min)
        dy = max(1e-6, self.y_max - self.y_min)
        dyaw = 2.0 * math.pi * self.yaw_weight
        return dx * dy * dyaw

    def sample_free(
        self,
        count: int,
        collision_checker: OccupancyGridCollisionChecker,
        start: State,
        goal: State,
        c_best: float,
    ) -> List[State]:
        samples: List[State] = []
        max_attempts = max(100, 80 * count)
        attempts = 0

        while len(samples) < count and attempts < max_attempts:
            attempts += 1
            candidate = self._sample_informed(start, goal, c_best) if math.isfinite(c_best) else self._sample_uniform()
            if collision_checker.is_state_valid(candidate):
                samples.append(candidate)
        return samples

    def _sample_uniform(self) -> State:
        return State(
            x=self.rng.uniform(self.x_min, self.x_max),
            y=self.rng.uniform(self.y_min, self.y_max),
            yaw=self.rng.uniform(-math.pi, math.pi),
        )

    def _sample_informed(self, start: State, goal: State, c_best: float) -> State:
        """
        Informed sampling inside the prolate ellipse defined by start, goal and c_best.
        The x-y sample is taken from the ellipse and yaw is sampled uniformly.
        """
        c_min = max(1e-9, self.heuristic(start, goal))
        if c_best <= c_min + 1e-9:
            return self._sample_uniform()

        center_x = 0.5 * (start.x + goal.x)
        center_y = 0.5 * (start.y + goal.y)
        heading = math.atan2(goal.y - start.y, goal.x - start.x)

        semi_major = 0.5 * c_best
        semi_minor_sq = max(semi_major ** 2 - (0.5 * c_min) ** 2, 0.0)
        semi_minor = math.sqrt(semi_minor_sq)

        r = math.sqrt(self.rng.random())
        theta = self.rng.uniform(0.0, 2.0 * math.pi)
        local_x = r * math.cos(theta)
        local_y = r * math.sin(theta)

        x_local = semi_major * local_x
        y_local = semi_minor * local_y

        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        x = center_x + cos_h * x_local - sin_h * y_local
        y = center_y + sin_h * x_local + cos_h * y_local

        x = min(max(x, self.x_min), self.x_max)
        y = min(max(y, self.y_min), self.y_max)
        yaw = self.rng.uniform(-math.pi, math.pi)
        return State(x=x, y=y, yaw=yaw)


# -----------------------------------------------------------------------------
# Nonholonomic local steering
# -----------------------------------------------------------------------------


class UnicycleSteering:
    """
    Local planner that respects nonholonomic unicycle kinematics.

    A small set of controller gains is tested and the lowest-cost valid
    trajectory is retained.
    """

    def __init__(
        self,
        max_linear_speed: float = 0.8,
        max_angular_speed: float = 1.5,
        integration_dt: float = 0.05,
        max_integration_steps: int = 300,
        goal_tolerance_xy: float = 0.20,
        goal_tolerance_yaw: float = 0.20,
        allow_reverse: bool = False,
        angular_weight: float = 0.05,
    ) -> None:
        self.max_linear_speed = max(1e-3, max_linear_speed)
        self.max_angular_speed = max(1e-3, max_angular_speed)
        self.integration_dt = max(1e-3, integration_dt)
        self.max_integration_steps = max(1, max_integration_steps)
        self.goal_tolerance_xy = max(1e-3, goal_tolerance_xy)
        self.goal_tolerance_yaw = max(1e-3, goal_tolerance_yaw)
        self.allow_reverse = allow_reverse
        self.angular_weight = max(0.0, angular_weight)

    def steer(
        self,
        start: State,
        goal: State,
        collision_checker: OccupancyGridCollisionChecker,
    ) -> Optional[TrajectorySegment]:
        candidates: List[TrajectorySegment] = []
        gain_sets = [
            (1.2, 4.0, -1.0, False),
            (0.8, 3.0, -0.7, False),
            (1.5, 5.0, -1.5, False),
        ]
        if self.allow_reverse:
            gain_sets.extend(
                [
                    (1.2, 4.0, -1.0, True),
                    (0.8, 3.0, -0.7, True),
                ]
            )

        for k_rho, k_alpha, k_beta, reverse_mode in gain_sets:
            segment = self._simulate_controller(
                start=start,
                goal=goal,
                collision_checker=collision_checker,
                k_rho=k_rho,
                k_alpha=k_alpha,
                k_beta=k_beta,
                reverse_mode=reverse_mode,
            )
            if segment is not None:
                candidates.append(segment)

        if not candidates:
            return None
        return min(candidates, key=lambda segment: segment.cost)

    def _simulate_controller(
        self,
        start: State,
        goal: State,
        collision_checker: OccupancyGridCollisionChecker,
        k_rho: float,
        k_alpha: float,
        k_beta: float,
        reverse_mode: bool,
    ) -> Optional[TrajectorySegment]:
        current = State(start.x, start.y, start.yaw)
        states = [current]
        cost = 0.0

        for _ in range(self.max_integration_steps):
            dx = goal.x - current.x
            dy = goal.y - current.y
            rho = math.hypot(dx, dy)
            yaw_error = wrap_angle(goal.yaw - current.yaw)
            heading_to_goal = math.atan2(dy, dx) if rho > 1e-9 else goal.yaw

            if rho <= self.goal_tolerance_xy and abs(yaw_error) <= self.goal_tolerance_yaw:
                if current != goal and collision_checker.is_state_valid(goal):
                    cost += current.distance_xy(goal) + self.angular_weight * current.yaw_error(goal)
                    states.append(goal)
                return TrajectorySegment(states=states, cost=cost)

            if rho <= self.goal_tolerance_xy:
                v = 0.0
                omega = max(-self.max_angular_speed, min(self.max_angular_speed, 2.5 * yaw_error))
            else:
                alpha = wrap_angle(heading_to_goal - current.yaw)
                beta = wrap_angle(goal.yaw - heading_to_goal)
                direction = 1.0

                if reverse_mode:
                    direction = -1.0
                    alpha = wrap_angle(alpha + math.pi)
                    beta = wrap_angle(beta + math.pi)
                elif self.allow_reverse and abs(alpha) > math.pi / 2.0:
                    direction = -1.0
                    alpha = wrap_angle(alpha + math.pi)
                    beta = wrap_angle(beta + math.pi)

                v_cmd = direction * k_rho * rho
                omega_cmd = k_alpha * alpha + k_beta * beta

                v = max(-self.max_linear_speed, min(self.max_linear_speed, v_cmd))
                if not self.allow_reverse and v < 0.0:
                    v = 0.0
                omega = max(-self.max_angular_speed, min(self.max_angular_speed, omega_cmd))

            next_state = State(
                x=current.x + v * math.cos(current.yaw) * self.integration_dt,
                y=current.y + v * math.sin(current.yaw) * self.integration_dt,
                yaw=wrap_angle(current.yaw + omega * self.integration_dt),
            )

            if not collision_checker.is_state_valid(next_state):
                return None

            cost += abs(v) * self.integration_dt + self.angular_weight * abs(omega) * self.integration_dt
            states.append(next_state)
            current = next_state

        if current.distance_xy(goal) <= 1.5 * self.goal_tolerance_xy and current.yaw_error(goal) <= 1.5 * self.goal_tolerance_yaw:
            if current != goal and collision_checker.is_state_valid(goal):
                cost += current.distance_xy(goal) + self.angular_weight * current.yaw_error(goal)
                states.append(goal)
            return TrajectorySegment(states=states, cost=cost)
        return None


# -----------------------------------------------------------------------------
# Search tree
# -----------------------------------------------------------------------------


@dataclass
class TreeNode:
    node_id: int
    state: State
    parent_id: Optional[int]
    cost: float
    traj_from_parent: TrajectorySegment = field(default_factory=TrajectorySegment)


class SearchTree:
    def __init__(self, root_state: State) -> None:
        self.nodes: Dict[int, TreeNode] = {}
        self.children: Dict[int, Set[int]] = {}
        self._next_id = 0
        self.root_id = self.add_node(root_state, None, 0.0, TrajectorySegment(states=[root_state], cost=0.0))

    def add_node(
        self,
        state: State,
        parent_id: Optional[int],
        cost: float,
        traj_from_parent: TrajectorySegment,
    ) -> int:
        node_id = self._next_id
        self._next_id += 1
        self.nodes[node_id] = TreeNode(
            node_id=node_id,
            state=state,
            parent_id=parent_id,
            cost=cost,
            traj_from_parent=traj_from_parent,
        )
        self.children.setdefault(node_id, set())
        if parent_id is not None:
            self.children.setdefault(parent_id, set()).add(node_id)
        return node_id

    def is_ancestor(self, ancestor_id: int, node_id: int) -> bool:
        current = self.nodes[node_id].parent_id
        while current is not None:
            if current == ancestor_id:
                return True
            current = self.nodes[current].parent_id
        return False

    def rewire(
        self,
        node_id: int,
        new_parent_id: int,
        new_cost: float,
        traj_from_parent: TrajectorySegment,
    ) -> None:
        node = self.nodes[node_id]
        old_parent_id = node.parent_id
        old_cost = node.cost

        if old_parent_id is not None:
            self.children[old_parent_id].discard(node_id)

        node.parent_id = new_parent_id
        node.cost = new_cost
        node.traj_from_parent = traj_from_parent
        self.children.setdefault(new_parent_id, set()).add(node_id)

        delta = new_cost - old_cost
        if abs(delta) > 1e-9:
            self._propagate_cost_delta(node_id, delta)

    def _propagate_cost_delta(self, node_id: int, delta: float) -> None:
        queue = deque(self.children.get(node_id, set()))
        while queue:
            child_id = queue.popleft()
            self.nodes[child_id].cost += delta
            queue.extend(self.children.get(child_id, set()))

    def extract_path_states(self, node_id: int) -> List[State]:
        ordered_ids: List[int] = []
        current = node_id
        while current is not None:
            ordered_ids.append(current)
            current = self.nodes[current].parent_id
        ordered_ids.reverse()

        path: List[State] = []
        for idx, nid in enumerate(ordered_ids):
            segment = self.nodes[nid].traj_from_parent.states
            if idx == 0:
                path.extend(segment)
            else:
                path.extend(segment[1:] if segment else [self.nodes[nid].state])
        return path

    def remove_subtree(self, root_id: int) -> List[State]:
        ordered: List[int] = []
        queue = deque([root_id])
        while queue:
            node_id = queue.popleft()
            ordered.append(node_id)
            queue.extend(self.children.get(node_id, set()))

        removed_states = [self.nodes[node_id].state for node_id in ordered]

        for node_id in reversed(ordered):
            parent_id = self.nodes[node_id].parent_id
            if parent_id is not None:
                self.children[parent_id].discard(node_id)
            self.children.pop(node_id, None)
            self.nodes.pop(node_id, None)

        return removed_states


# -----------------------------------------------------------------------------
# BIT* planner
# -----------------------------------------------------------------------------


@dataclass
class PlannerResult:
    path: List[State]
    tree: SearchTree
    best_cost: float
    iterations: int
    solved: bool


class BITStarPlanner:
    def __init__(
        self,
        collision_checker: OccupancyGridCollisionChecker,
        search_space: SearchSpace,
        steering: UnicycleSteering,
        batch_size: int = 180,
        max_iterations: int = 4000,
        max_planning_time: float = 2.5,
        eta_radius: float = 1.1,
        min_connection_radius: float = 0.4,
        max_connection_radius: float = 2.5,
        goal_tolerance_xy: float = 0.25,
        goal_tolerance_yaw: float = 0.25,
        prune_epsilon: float = 1e-6,
    ) -> None:
        self.collision_checker = collision_checker
        self.search_space = search_space
        self.steering = steering
        self.batch_size = max(1, batch_size)
        self.max_iterations = max(1, max_iterations)
        self.max_planning_time = max(1e-3, max_planning_time)
        self.eta_radius = max(1.0, eta_radius)
        self.min_connection_radius = max(1e-3, min_connection_radius)
        self.max_connection_radius = max(self.min_connection_radius, max_connection_radius)
        self.goal_tolerance_xy = goal_tolerance_xy
        self.goal_tolerance_yaw = goal_tolerance_yaw
        self.prune_epsilon = max(0.0, prune_epsilon)

        self._counter = itertools.count()
        self._qv: List[Tuple[float, float, int, int]] = []
        self._qe: List[Tuple[float, float, int, int, str, object]] = []

        self.start: Optional[State] = None
        self.goal: Optional[State] = None
        self.tree: Optional[SearchTree] = None
        self.samples: Dict[State, State] = {}
        self.expanded_in_batch: Set[int] = set()
        self.best_cost = math.inf
        self.best_goal_node_id: Optional[int] = None

    def plan(self, start: State, goal: State) -> PlannerResult:
        self.start = start
        self.goal = goal
        self.tree = SearchTree(start)
        self.samples = {goal: goal}
        self.expanded_in_batch = set()
        self.best_cost = math.inf
        self.best_goal_node_id = None
        self._counter = itertools.count()
        self._qv = []
        self._qe = []

        start_time = time.perf_counter()
        iterations = 0
        self._start_new_batch()

        while iterations < self.max_iterations and (time.perf_counter() - start_time) < self.max_planning_time:
            if not self._qv and not self._qe:
                self._start_new_batch()
                if not self._qv and not self._qe:
                    break

            while self._qv and (not self._qe or self._best_vertex_queue_value() <= self._best_edge_queue_value()):
                vertex_id = self._pop_best_vertex()
                if vertex_id is None:
                    break
                self._expand_vertex(vertex_id)

            edge = self._pop_best_edge()
            if edge is None:
                iterations += 1
                continue

            self._process_edge(edge)
            iterations += 1

        path: List[State] = []
        solved = self.best_goal_node_id is not None
        if solved and self.tree is not None and self.best_goal_node_id is not None:
            path = self.tree.extract_path_states(self.best_goal_node_id)

        return PlannerResult(
            path=path,
            tree=self.tree,
            best_cost=self.best_cost,
            iterations=iterations,
            solved=solved,
        )

    # -------------------------
    # Batch management
    # -------------------------

    def _start_new_batch(self) -> None:
        assert self.start is not None and self.goal is not None and self.tree is not None

        if math.isfinite(self.best_cost):
            self._prune_tree_and_samples()

        new_samples = self.search_space.sample_free(
            count=self.batch_size,
            collision_checker=self.collision_checker,
            start=self.start,
            goal=self.goal,
            c_best=self.best_cost,
        )
        for state in new_samples:
            self.samples[state] = state

        if self.goal not in self.samples and self.best_goal_node_id is None:
            self.samples[self.goal] = self.goal

        self.expanded_in_batch.clear()
        self._qv = []
        self._qe = []

        for node_id in list(self.tree.nodes.keys()):
            if self._solution_lower_bound(self.tree.nodes[node_id].state) <= self.best_cost + self.prune_epsilon:
                self._enqueue_vertex(node_id)

    def _prune_tree_and_samples(self) -> None:
        assert self.tree is not None
        if not math.isfinite(self.best_cost):
            return

        # 1) Remove useless samples.
        self.samples = {
            state: state
            for state in self.samples.values()
            if self._solution_lower_bound(state) <= self.best_cost + self.prune_epsilon
        }

        # 2) Remove tree sub-branches that cannot improve the current solution.
        removable_roots: List[int] = []
        for node_id, node in list(self.tree.nodes.items()):
            if node_id == self.tree.root_id:
                continue
            if node_id == self.best_goal_node_id:
                continue

            node_bound = self._solution_lower_bound(node.state)
            if node_bound <= self.best_cost + self.prune_epsilon:
                continue

            parent_id = node.parent_id
            if parent_id is None:
                removable_roots.append(node_id)
                continue

            parent_bound = self._solution_lower_bound(self.tree.nodes[parent_id].state)
            if parent_bound <= self.best_cost + self.prune_epsilon:
                removable_roots.append(node_id)

        for node_id in removable_roots:
            if node_id not in self.tree.nodes:
                continue
            removed_states = self.tree.remove_subtree(node_id)
            for state in removed_states:
                if self._solution_lower_bound(state) <= self.best_cost + self.prune_epsilon:
                    self.samples[state] = state

        if self.best_goal_node_id is not None and self.best_goal_node_id not in self.tree.nodes:
            self.best_goal_node_id = None
            self.best_cost = math.inf

    # -------------------------
    # Radius and lower bounds
    # -------------------------

    def _connection_radius(self) -> float:
        """
        Adaptive radius using the standard random-geometric-graph BIT*/RRT* form:
            r(q) = eta * gamma * (log(q) / q)^(1/d)
        with d=3 for SE(2) under the weighted metric.
        """
        d = 3.0
        q = max(2.0, float(len(self.samples) + len(self.tree.nodes)))
        zeta_d = 4.0 * math.pi / 3.0  # unit ball volume in R^3

        free_area = max(self.collision_checker.free_area(), 1e-6)
        yaw_extent = 2.0 * math.pi * max(self.search_space.yaw_weight, 1e-6)
        free_volume = max(free_area * yaw_extent, 1e-6)

        gamma_star = 2.0 * ((1.0 + 1.0 / d) ** (1.0 / d)) * ((free_volume / zeta_d) ** (1.0 / d))
        radius = self.eta_radius * gamma_star * ((math.log(q) / q) ** (1.0 / d))

        return min(self.max_connection_radius, max(self.min_connection_radius, radius))

    def _solution_lower_bound(self, state: State) -> float:
        assert self.start is not None and self.goal is not None
        return self.search_space.heuristic(self.start, state) + self.search_space.heuristic(state, self.goal)

    def _vertex_queue_value(self, node_id: int) -> float:
        assert self.goal is not None and self.tree is not None
        node = self.tree.nodes[node_id]
        return node.cost + self.search_space.heuristic(node.state, self.goal)

    # -------------------------
    # Queue management
    # -------------------------

    def _enqueue_vertex(self, node_id: int) -> None:
        assert self.tree is not None
        node = self.tree.nodes[node_id]
        total_est = self._vertex_queue_value(node_id)
        heapq.heappush(self._qv, (total_est, node.cost, next(self._counter), node_id))

    def _enqueue_edge(self, parent_id: int, target_kind: str, target_ref: object) -> None:
        assert self.tree is not None and self.goal is not None
        parent = self.tree.nodes[parent_id]
        target_state = target_ref if target_kind == "sample" else self.tree.nodes[int(target_ref)].state
        edge_lb = self.search_space.edge_cost_lower_bound(parent.state, target_state)
        total_est = parent.cost + edge_lb + self.search_space.heuristic(target_state, self.goal)
        edge_est = parent.cost + edge_lb
        heapq.heappush(self._qe, (total_est, edge_est, next(self._counter), parent_id, target_kind, target_ref))

    def _best_vertex_queue_value(self) -> float:
        value = self._clean_vertex_queue(peek_only=True)
        return math.inf if value is None else value

    def _best_edge_queue_value(self) -> float:
        value = self._clean_edge_queue(peek_only=True)
        return math.inf if value is None else value

    def _clean_vertex_queue(self, peek_only: bool = False) -> Optional[float]:
        assert self.tree is not None
        while self._qv:
            total_est, _, _, node_id = self._qv[0]
            if node_id not in self.tree.nodes:
                heapq.heappop(self._qv)
                continue
            if node_id in self.expanded_in_batch:
                heapq.heappop(self._qv)
                continue
            if total_est > self.best_cost + self.prune_epsilon:
                heapq.heappop(self._qv)
                continue
            return total_est if peek_only else None
        return None

    def _clean_edge_queue(self, peek_only: bool = False) -> Optional[float]:
        assert self.tree is not None
        while self._qe:
            total_est, _, _, parent_id, target_kind, target_ref = self._qe[0]
            if parent_id not in self.tree.nodes:
                heapq.heappop(self._qe)
                continue
            if target_kind == "sample" and target_ref not in self.samples:
                heapq.heappop(self._qe)
                continue
            if target_kind == "vertex" and int(target_ref) not in self.tree.nodes:
                heapq.heappop(self._qe)
                continue
            if total_est > self.best_cost + self.prune_epsilon:
                heapq.heappop(self._qe)
                continue
            return total_est if peek_only else None
        return None

    def _pop_best_vertex(self) -> Optional[int]:
        assert self.tree is not None
        while self._qv:
            total_est, _, _, node_id = heapq.heappop(self._qv)
            if node_id not in self.tree.nodes:
                continue
            if node_id in self.expanded_in_batch:
                continue
            if total_est > self.best_cost + self.prune_epsilon:
                continue
            return node_id
        return None

    def _pop_best_edge(self) -> Optional[Tuple[int, str, object]]:
        assert self.tree is not None
        while self._qe:
            total_est, _, _, parent_id, target_kind, target_ref = heapq.heappop(self._qe)
            if parent_id not in self.tree.nodes:
                continue
            if target_kind == "sample" and target_ref not in self.samples:
                continue
            if target_kind == "vertex" and int(target_ref) not in self.tree.nodes:
                continue
            if total_est > self.best_cost + self.prune_epsilon:
                self._qv.clear()
                self._qe.clear()
                return None
            return parent_id, target_kind, target_ref
        return None

    # -------------------------
    # Vertex expansion and edge processing
    # -------------------------

    def _expand_vertex(self, vertex_id: int) -> None:
        assert self.tree is not None and self.goal is not None
        if vertex_id in self.expanded_in_batch:
            return

        vertex = self.tree.nodes[vertex_id]
        radius = self._connection_radius()

        # Candidate connections to samples.
        for sample in list(self.samples.values()):
            if self.search_space.metric(vertex.state, sample) > radius:
                continue
            est_total = vertex.cost + self.search_space.edge_cost_lower_bound(vertex.state, sample) + self.search_space.heuristic(sample, self.goal)
            if est_total > self.best_cost + self.prune_epsilon:
                continue
            self._enqueue_edge(vertex_id, "sample", sample)

        # Candidate rewiring connections to already-added vertices.
        for other_id, other in list(self.tree.nodes.items()):
            if other_id == vertex_id:
                continue
            if self.search_space.metric(vertex.state, other.state) > radius:
                continue
            if self.tree.is_ancestor(other_id, vertex_id):
                continue

            edge_lb = self.search_space.edge_cost_lower_bound(vertex.state, other.state)
            if vertex.cost + edge_lb >= other.cost - 1e-9:
                continue
            est_total = vertex.cost + edge_lb + self.search_space.heuristic(other.state, self.goal)
            if est_total > self.best_cost + self.prune_epsilon:
                continue
            self._enqueue_edge(vertex_id, "vertex", other_id)

        self.expanded_in_batch.add(vertex_id)

    def _process_edge(self, edge: Tuple[int, str, object]) -> None:
        assert self.tree is not None and self.goal is not None
        parent_id, target_kind, target_ref = edge

        if parent_id not in self.tree.nodes:
            return
        parent = self.tree.nodes[parent_id]

        if target_kind == "sample":
            if target_ref not in self.samples:
                return
            target_state = self.samples[target_ref]
            current_target_cost = math.inf
        else:
            target_id = int(target_ref)
            if target_id not in self.tree.nodes:
                return
            if self.tree.is_ancestor(target_id, parent_id):
                return
            target_state = self.tree.nodes[target_id].state
            current_target_cost = self.tree.nodes[target_id].cost

        edge_lb = self.search_space.edge_cost_lower_bound(parent.state, target_state)
        if parent.cost + edge_lb + self.search_space.heuristic(target_state, self.goal) > self.best_cost + self.prune_epsilon:
            return
        if parent.cost + edge_lb >= current_target_cost - 1e-9:
            return

        segment = self.steering.steer(parent.state, target_state, self.collision_checker)
        if segment is None or not segment.states:
            return

        new_cost = parent.cost + segment.cost
        if new_cost >= current_target_cost - 1e-9:
            return
        if new_cost + self.search_space.heuristic(target_state, self.goal) > self.best_cost + self.prune_epsilon:
            return

        if target_kind == "sample":
            new_node_id = self.tree.add_node(target_state, parent_id, new_cost, segment)
            self.samples.pop(target_state, None)
            self._enqueue_vertex(new_node_id)
            self._try_update_goal(new_node_id)
        else:
            target_id = int(target_ref)
            self.tree.rewire(target_id, parent_id, new_cost, segment)
            self._enqueue_vertex(target_id)
            self._try_update_goal(target_id)

    def _try_update_goal(self, node_id: int) -> None:
        assert self.tree is not None and self.goal is not None
        node = self.tree.nodes[node_id]
        if node.state.distance_xy(self.goal) <= self.goal_tolerance_xy and node.state.yaw_error(self.goal) <= self.goal_tolerance_yaw:
            if node.cost < self.best_cost:
                self.best_cost = node.cost
                self.best_goal_node_id = node_id


# -----------------------------------------------------------------------------
# ROS 2 node
# -----------------------------------------------------------------------------


class BITStarPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("bitstar_planner_node")

        # Topics and frames
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("start_topic", "/planner/start")
        self.declare_parameter("path_topic", "/bitstar_path")
        self.declare_parameter("tree_topic", "/bitstar_tree")
        self.declare_parameter("frame_id", "map")

        # Environment and robot
        self.declare_parameter("robot_radius", 0.20)
        self.declare_parameter("occupancy_threshold", 50)
        self.declare_parameter("unknown_is_occupied", True)

        # BIT*
        self.declare_parameter("batch_size", 180)
        self.declare_parameter("max_iterations", 4000)
        self.declare_parameter("max_planning_time", 2.5)
        self.declare_parameter("eta_radius", 1.1)
        self.declare_parameter("min_connection_radius", 0.4)
        self.declare_parameter("max_connection_radius", 2.5)
        self.declare_parameter("goal_tolerance_xy", 0.25)
        self.declare_parameter("goal_tolerance_yaw", 0.25)
        self.declare_parameter("sample_yaw_weight", 0.25)
        self.declare_parameter("prune_epsilon", 1e-6)

        # Local nonholonomic steering
        self.declare_parameter("max_linear_speed", 0.8)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("integration_dt", 0.05)
        self.declare_parameter("max_integration_steps", 320)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("angular_weight", 0.05)

        self.map_msg: Optional[OccupancyGrid] = None
        self.map_array: Optional[np.ndarray] = None
        self.start_state: Optional[State] = None
        self.goal_state: Optional[State] = None

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        default_qos = QoSProfile(depth=10)

        map_topic = str(self.get_parameter("map_topic").value)
        goal_topic = str(self.get_parameter("goal_topic").value)
        start_topic = str(self.get_parameter("start_topic").value)

        self.create_subscription(OccupancyGrid, map_topic, self._map_callback, map_qos)
        self.create_subscription(PoseStamped, goal_topic, self._goal_callback, default_qos)
        self.create_subscription(PoseStamped, start_topic, self._start_callback, default_qos)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._initialpose_callback, default_qos)

        self.path_pub = self.create_publisher(Path, str(self.get_parameter("path_topic").value), 10)
        self.tree_pub = self.create_publisher(Marker, str(self.get_parameter("tree_topic").value), 10)

        self.get_logger().info("BIT* nonholonomic planner ready")

    # -------------------------
    # Callbacks
    # -------------------------

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_array = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.get_logger().info(
            f"Map received | size={msg.info.width}x{msg.info.height} | resolution={msg.info.resolution:.3f} m/cell"
        )
        self._try_plan()

    def _start_callback(self, msg: PoseStamped) -> None:
        self.start_state = self._pose_to_state(msg.pose.position.x, msg.pose.position.y, msg.pose.orientation)
        self.get_logger().info(
            f"Start updated | x={self.start_state.x:.2f}, y={self.start_state.y:.2f}, yaw={self.start_state.yaw:.2f}"
        )
        self._try_plan()

    def _initialpose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.start_state = self._pose_to_state(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation)
        self.get_logger().info(
            f"Start updated from /initialpose | x={self.start_state.x:.2f}, y={self.start_state.y:.2f}, yaw={self.start_state.yaw:.2f}"
        )
        self._try_plan()

    def _goal_callback(self, msg: PoseStamped) -> None:
        self.goal_state = self._pose_to_state(msg.pose.position.x, msg.pose.position.y, msg.pose.orientation)
        self.get_logger().info(
            f"Goal updated | x={self.goal_state.x:.2f}, y={self.goal_state.y:.2f}, yaw={self.goal_state.yaw:.2f}"
        )
        self._try_plan()

    # -------------------------
    # Helpers
    # -------------------------

    def _pose_to_state(self, x: float, y: float, q: Quaternion) -> State:
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return State(x=x, y=y, yaw=yaw)

    def _state_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(0.5 * yaw)
        q.w = math.cos(0.5 * yaw)
        return q

    def _try_plan(self) -> None:
        if self.map_msg is None or self.map_array is None:
            return
        if self.start_state is None or self.goal_state is None:
            return

        collision_checker = OccupancyGridCollisionChecker(
            grid=self.map_array,
            resolution=self.map_msg.info.resolution,
            origin_x=self.map_msg.info.origin.position.x,
            origin_y=self.map_msg.info.origin.position.y,
            robot_radius=float(self.get_parameter("robot_radius").value),
            occupancy_threshold=int(self.get_parameter("occupancy_threshold").value),
            unknown_is_occupied=bool(self.get_parameter("unknown_is_occupied").value),
        )

        if not collision_checker.is_state_valid(self.start_state):
            self.get_logger().warn("Invalid start state: outside map or in collision")
            return
        if not collision_checker.is_state_valid(self.goal_state):
            self.get_logger().warn("Invalid goal state: outside map or in collision")
            return

        x_min, x_max, y_min, y_max = collision_checker.bounds()
        search_space = SearchSpace(
            x_min=x_min,
            x_max=x_max,
            y_min=y_min,
            y_max=y_max,
            yaw_weight=float(self.get_parameter("sample_yaw_weight").value),
        )
        steering = UnicycleSteering(
            max_linear_speed=float(self.get_parameter("max_linear_speed").value),
            max_angular_speed=float(self.get_parameter("max_angular_speed").value),
            integration_dt=float(self.get_parameter("integration_dt").value),
            max_integration_steps=int(self.get_parameter("max_integration_steps").value),
            goal_tolerance_xy=float(self.get_parameter("goal_tolerance_xy").value),
            goal_tolerance_yaw=float(self.get_parameter("goal_tolerance_yaw").value),
            allow_reverse=bool(self.get_parameter("allow_reverse").value),
            angular_weight=float(self.get_parameter("angular_weight").value),
        )
        planner = BITStarPlanner(
            collision_checker=collision_checker,
            search_space=search_space,
            steering=steering,
            batch_size=int(self.get_parameter("batch_size").value),
            max_iterations=int(self.get_parameter("max_iterations").value),
            max_planning_time=float(self.get_parameter("max_planning_time").value),
            eta_radius=float(self.get_parameter("eta_radius").value),
            min_connection_radius=float(self.get_parameter("min_connection_radius").value),
            max_connection_radius=float(self.get_parameter("max_connection_radius").value),
            goal_tolerance_xy=float(self.get_parameter("goal_tolerance_xy").value),
            goal_tolerance_yaw=float(self.get_parameter("goal_tolerance_yaw").value),
            prune_epsilon=float(self.get_parameter("prune_epsilon").value),
        )

        self.get_logger().info("Planning started")
        result = planner.plan(self.start_state, self.goal_state)

        if not result.solved:
            self.get_logger().warn(
                f"No feasible trajectory found | iterations={result.iterations} | best_cost=inf"
            )
            return

        self._publish_path(result.path)
        self._publish_tree(result.tree)
        self.get_logger().info(
            f"Trajectory planned | states={len(result.path)} | cost={result.best_cost:.3f} | iterations={result.iterations}"
        )

    def _publish_path(self, path_states: List[State]) -> None:
        frame_id = str(self.get_parameter("frame_id").value)
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        for state in path_states:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = state.x
            pose.pose.position.y = state.y
            pose.pose.orientation = self._state_to_quaternion(state.yaw)
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def _publish_tree(self, tree: SearchTree) -> None:
        frame_id = str(self.get_parameter("frame_id").value)
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame_id
        marker.ns = "bitstar_tree"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.015
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 0.3

        for node_id, node in tree.nodes.items():
            if node_id == tree.root_id:
                continue
            segment = node.traj_from_parent.states
            if len(segment) < 2:
                continue
            for s0, s1 in zip(segment[:-1], segment[1:]):
                marker.points.append(Point(x=s0.x, y=s0.y, z=0.0))
                marker.points.append(Point(x=s1.x, y=s1.y, z=0.0))

        self.tree_pub.publish(marker)


# -----------------------------------------------------------------------------
# Main entry point
# -----------------------------------------------------------------------------


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BITStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
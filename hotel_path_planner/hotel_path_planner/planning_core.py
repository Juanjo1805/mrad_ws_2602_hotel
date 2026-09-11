"""ROS-independent grid planning utilities used by the Hotel assignment.

The two planners deliberately share this map representation and metric code so
that a benchmark changes the planning method, not the collision policy.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


INF_CLEARANCE = 1_000_000


def wrap_to_pi(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(q) -> float:
    """Return planar yaw from an object exposing x, y, z and w."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quaternion(yaw: float):
    """Return quaternion components (x, y, z, w), avoiding a ROS dependency."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass
class PlannerResult:
    success: bool
    poses: List[Pose2D] = field(default_factory=list)
    planning_time_ms: float = 0.0
    expanded_nodes: int = 0
    reason: str = ""
    path_length_m: float = 0.0
    min_clearance_m: Optional[float] = None
    sharp_direction_changes: int = 0


class GridMap:
    """Occupancy grid with a shared inflation and clearance policy.

    ``clearance_cells`` is an 8-connected chamfer distance.  It is reported as
    an approximation, rather than claimed as an exact Euclidean distance.
    """

    def __init__(
        self,
        occupancy: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        occupied_threshold: int = 65,
        treat_unknown_as_obstacle: bool = True,
        inflate_radius: float = 0.25,
        origin_yaw: float = 0.0,
    ) -> None:
        if occupancy.ndim != 2:
            raise ValueError("occupancy must be a (height, width) array")
        if resolution <= 0.0:
            raise ValueError("resolution must be positive")
        self.occupancy = np.asarray(occupancy, dtype=np.int16)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        # OccupancyGrid.origin is a full pose.  Most map-server maps use a
        # zero yaw, but silently ignoring a non-zero yaw swaps/rotates the
        # world-grid conversion and produces a route that only looks right in
        # RViz by accident.
        self.origin_yaw = float(origin_yaw)
        self._origin_cos = math.cos(self.origin_yaw)
        self._origin_sin = math.sin(self.origin_yaw)
        self.height, self.width = self.occupancy.shape
        self.occupied_threshold = int(occupied_threshold)
        self.treat_unknown_as_obstacle = bool(treat_unknown_as_obstacle)

        self.raw_obstacles = self.occupancy >= self.occupied_threshold
        if self.treat_unknown_as_obstacle:
            self.raw_obstacles = np.logical_or(self.raw_obstacles, self.occupancy < 0)
        self.clearance_cells = self._distance_to_obstacles(self.raw_obstacles)
        inflation_cells = int(math.ceil(max(0.0, inflate_radius) / self.resolution))
        self.obstacles = np.logical_or(self.raw_obstacles, self.clearance_cells <= inflation_cells)

    @staticmethod
    def _distance_to_obstacles(obstacles: np.ndarray) -> np.ndarray:
        """8-neighbour brushfire transform, stored in cells as float values."""
        height, width = obstacles.shape
        dist = np.full((height, width), float(INF_CLEARANCE), dtype=np.float64)
        queue: List[Tuple[float, int, int]] = []
        ys, xs = np.nonzero(obstacles)
        for y, x in zip(ys, xs):
            dist[y, x] = 0.0
            heapq.heappush(queue, (0.0, int(x), int(y)))
        if not queue:
            return dist
        neighbors = (
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
        )
        while queue:
            current, x, y = heapq.heappop(queue)
            if current != dist[y, x]:
                continue
            for dx, dy, step in neighbors:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    candidate = current + step
                    if candidate < dist[ny, nx]:
                        dist[ny, nx] = candidate
                        heapq.heappush(queue, (candidate, nx, ny))
        return dist

    def world_to_local(self, x: float, y: float) -> Tuple[float, float]:
        """Express a world point in the unrotated grid-origin coordinates."""
        dx, dy = x - self.origin_x, y - self.origin_y
        return (
            self._origin_cos * dx + self._origin_sin * dy,
            -self._origin_sin * dx + self._origin_cos * dy,
        )

    def world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        local_x, local_y = self.world_to_local(x, y)
        ix = int(math.floor(local_x / self.resolution))
        iy = int(math.floor(local_y / self.resolution))
        if self.in_bounds(ix, iy):
            return ix, iy
        return None

    def grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        local_x = (ix + 0.5) * self.resolution
        local_y = (iy + 0.5) * self.resolution
        return (
            self.origin_x + self._origin_cos * local_x - self._origin_sin * local_y,
            self.origin_y + self._origin_sin * local_x + self._origin_cos * local_y,
        )

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height

    def is_free_world(self, x: float, y: float) -> bool:
        grid = self.world_to_grid(x, y)
        return grid is not None and not bool(self.obstacles[grid[1], grid[0]])

    def is_free_grid(self, ix: int, iy: int) -> bool:
        return self.in_bounds(ix, iy) and not bool(self.obstacles[iy, ix])

    def clearance_at_world(self, x: float, y: float) -> Optional[float]:
        grid = self.world_to_grid(x, y)
        if grid is None:
            return None
        value = float(self.clearance_cells[grid[1], grid[0]])
        if value >= INF_CLEARANCE:
            return None
        return value * self.resolution

    def occupancy_penalty(self, x: float, y: float, proximity_radius: float) -> float:
        grid = self.world_to_grid(x, y)
        if grid is None:
            return 1.0
        ix, iy = grid
        occ = int(self.occupancy[iy, ix])
        occ_norm = 1.0 if occ < 0 else min(1.0, max(0.0, occ / 100.0))
        clearance = self.clearance_at_world(x, y)
        proximity = 0.0
        if clearance is not None and proximity_radius > 1e-9:
            proximity = max(0.0, 1.0 - clearance / proximity_radius)
        return 0.5 * (occ_norm + proximity)


def path_metrics(grid: GridMap, poses: Sequence[Pose2D]) -> Tuple[float, Optional[float], int]:
    """Return length, approximate clearance and sharp yaw-change count."""
    if not poses:
        return 0.0, None, 0
    length = 0.0
    clearances: List[float] = []
    sharp = 0
    for i, pose in enumerate(poses):
        clearance = grid.clearance_at_world(pose.x, pose.y)
        if clearance is not None:
            clearances.append(clearance)
        if i:
            length += math.hypot(pose.x - poses[i - 1].x, pose.y - poses[i - 1].y)
            if abs(wrap_to_pi(pose.yaw - poses[i - 1].yaw)) > math.radians(35.0):
                sharp += 1
    return length, (min(clearances) if clearances else None), sharp


def densify_path(
    poses: Sequence[Pose2D],
    resolution: float,
    angular_resolution: float = math.radians(10.0),
) -> List[Pose2D]:
    """Interpolate a path without losing terminal heading information.

    Grid Dijkstra already produces samples at the map resolution, while a
    Hybrid A* primitive can be much longer.  A common output resolution keeps
    a tracker from treating sparse planner states as a polyline with large,
    discontinuous steering requests.  Pure rotations are sampled by yaw too.
    """
    if not poses:
        return []
    step = max(float(resolution), 1e-4)
    yaw_step = max(float(angular_resolution), 1e-4)
    dense: List[Pose2D] = [poses[0]]
    for start, finish in zip(poses, poses[1:]):
        distance = math.hypot(finish.x - start.x, finish.y - start.y)
        yaw_delta = wrap_to_pi(finish.yaw - start.yaw)
        count = max(1, int(math.ceil(distance / step)), int(math.ceil(abs(yaw_delta) / yaw_step)))
        for index in range(1, count + 1):
            ratio = index / count
            dense.append(Pose2D(
                start.x + (finish.x - start.x) * ratio,
                start.y + (finish.y - start.y) * ratio,
                wrap_to_pi(start.yaw + yaw_delta * ratio),
            ))
    # Avoid an accumulated numerical yaw error at the final orientation.
    dense[-1] = Pose2D(poses[-1].x, poses[-1].y, poses[-1].yaw)
    return dense


class DijkstraPlanner:
    """Grid Dijkstra baseline with the same occupancy policy as Hybrid A*."""

    def __init__(
        self,
        grid: GridMap,
        use_8_connected: bool = True,
        prevent_corner_cutting: bool = True,
        traversal_cost_weight: float = 0.0,
    ) -> None:
        self.grid = grid
        self.use_8_connected = bool(use_8_connected)
        self.prevent_corner_cutting = bool(prevent_corner_cutting)
        self.traversal_cost_weight = max(0.0, float(traversal_cost_weight))

    def plan(self, start: Pose2D, goal: Pose2D) -> PlannerResult:
        started = time.perf_counter()
        start_grid, goal_grid = self.grid.world_to_grid(start.x, start.y), self.grid.world_to_grid(goal.x, goal.y)
        if start_grid is None or goal_grid is None:
            return self._failure(started, "start_or_goal_outside_map")
        if not self.grid.is_free_grid(*start_grid):
            return self._failure(started, "start_occupied")
        if not self.grid.is_free_grid(*goal_grid):
            return self._failure(started, "goal_occupied")

        steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
        if self.use_8_connected:
            diagonal = math.sqrt(2.0)
            steps.extend([(1, 1, diagonal), (1, -1, diagonal), (-1, 1, diagonal), (-1, -1, diagonal)])
        dist = np.full((self.grid.height, self.grid.width), np.inf, dtype=np.float64)
        parent: Dict[Tuple[int, int], Tuple[int, int]] = {}
        visited = np.zeros((self.grid.height, self.grid.width), dtype=bool)
        queue: List[Tuple[float, Tuple[int, int]]] = [(0.0, start_grid)]
        dist[start_grid[1], start_grid[0]] = 0.0
        expanded = 0

        while queue:
            cost, (cx, cy) = heapq.heappop(queue)
            if visited[cy, cx]:
                continue
            visited[cy, cx] = True
            expanded += 1
            if (cx, cy) == goal_grid:
                break
            for dx, dy, step_cost in steps:
                nx, ny = cx + dx, cy + dy
                if not self.grid.is_free_grid(nx, ny):
                    continue
                if self.prevent_corner_cutting and dx and dy:
                    if not self.grid.is_free_grid(cx + dx, cy) or not self.grid.is_free_grid(cx, cy + dy):
                        continue
                x, y = self.grid.grid_to_world(nx, ny)
                extra = self.traversal_cost_weight * self.grid.occupancy_penalty(
                    x, y, proximity_radius=max(self.grid.resolution, 0.25)
                ) * step_cost
                candidate = cost + step_cost + extra
                if candidate < dist[ny, nx]:
                    dist[ny, nx] = candidate
                    parent[(nx, ny)] = (cx, cy)
                    heapq.heappush(queue, (candidate, (nx, ny)))

        if not visited[goal_grid[1], goal_grid[0]]:
            return self._failure(started, "no_path", expanded)
        cells = [goal_grid]
        while cells[-1] != start_grid:
            cells.append(parent[cells[-1]])
        cells.reverse()
        poses: List[Pose2D] = []
        previous_yaw = start.yaw
        for index, (ix, iy) in enumerate(cells):
            x, y = self.grid.grid_to_world(ix, iy)
            if index + 1 < len(cells):
                nx, ny = self.grid.grid_to_world(*cells[index + 1])
                previous_yaw = math.atan2(ny - y, nx - x)
            elif len(cells) == 1:
                previous_yaw = goal.yaw
            poses.append(Pose2D(x, y, previous_yaw))
        # Dijkstra has no heading state, but a PoseStamped goal does.  Keep
        # its requested terminal orientation instead of the last grid edge.
        poses[-1] = Pose2D(poses[-1].x, poses[-1].y, goal.yaw)
        elapsed = (time.perf_counter() - started) * 1000.0
        length, clearance, sharp = path_metrics(self.grid, poses)
        return PlannerResult(True, poses, elapsed, expanded, "", length, clearance, sharp)

    @staticmethod
    def _failure(started: float, reason: str, expanded: int = 0) -> PlannerResult:
        return PlannerResult(False, planning_time_ms=(time.perf_counter() - started) * 1000.0,
                             expanded_nodes=expanded, reason=reason)


@dataclass(frozen=True)
class HybridState:
    x: float
    y: float
    yaw: float
    direction: int = 1


class HybridAStarPlanner:
    """Hybrid A* for a differential-drive/unicycle robot.

    Translational primitives integrate ``xdot=v cos(theta)``,
    ``ydot=v sin(theta)``, ``thetadot=omega``.  Optional in-place rotations
    are physically feasible on differential drive and are intentionally not
    used as an Ackermann steering approximation.
    """

    def __init__(
        self,
        grid: GridMap,
        xy_resolution: float = 0.05,
        theta_resolution: float = math.radians(15.0),
        motion_step: float = 0.20,
        heuristic_weight: float = 1.0,
        max_curvature: float = 1.6,
        allow_reverse: bool = False,
        allow_in_place_rotation: bool = True,
        reverse_penalty: float = 1.4,
        turn_penalty: float = 0.10,
        direction_change_penalty: float = 0.25,
        rotation_penalty: float = 0.12,
        goal_position_tolerance: float = 0.15,
        goal_yaw_tolerance: float = math.radians(20.0),
        max_iterations: int = 100000,
        traversal_cost_weight: float = 0.0,
    ) -> None:
        if xy_resolution <= 0.0 or theta_resolution <= 0.0 or motion_step <= 0.0:
            raise ValueError("resolutions and motion_step must be positive")
        self.grid = grid
        self.xy_resolution = float(xy_resolution)
        self.theta_resolution = float(theta_resolution)
        self.theta_bins = max(4, int(round(2.0 * math.pi / self.theta_resolution)))
        self.theta_resolution = 2.0 * math.pi / self.theta_bins
        self.motion_step = float(motion_step)
        self.heuristic_weight = max(0.0, float(heuristic_weight))
        self.max_curvature = max(0.0, float(max_curvature))
        self.allow_reverse = bool(allow_reverse)
        self.allow_in_place_rotation = bool(allow_in_place_rotation)
        self.reverse_penalty = max(1.0, float(reverse_penalty))
        self.turn_penalty = max(0.0, float(turn_penalty))
        self.direction_change_penalty = max(0.0, float(direction_change_penalty))
        self.rotation_penalty = max(0.0, float(rotation_penalty))
        self.goal_position_tolerance = max(0.0, float(goal_position_tolerance))
        self.goal_yaw_tolerance = max(0.0, float(goal_yaw_tolerance))
        self.max_iterations = max(1, int(max_iterations))
        self.traversal_cost_weight = max(0.0, float(traversal_cost_weight))

    def theta_to_bin(self, yaw: float) -> int:
        return int(math.floor((wrap_to_pi(yaw) + math.pi) / self.theta_resolution + 0.5)) % self.theta_bins

    def bin_to_theta(self, theta_bin: int) -> float:
        return wrap_to_pi(theta_bin * self.theta_resolution - math.pi)

    def state_key(self, state: HybridState) -> Tuple[int, int, int, int]:
        local_x, local_y = self.grid.world_to_local(state.x, state.y)
        ix = int(math.floor(local_x / self.xy_resolution))
        iy = int(math.floor(local_y / self.xy_resolution))
        return ix, iy, self.theta_to_bin(state.yaw), state.direction

    def _translation_primitive(self, state: HybridState, curvature: float, direction: int) -> Optional[HybridState]:
        distance = direction * self.motion_step
        sample_distance = max(self.grid.resolution * 0.5, 0.02)
        samples = max(1, int(math.ceil(self.motion_step / sample_distance)))
        x, y, yaw = state.x, state.y, state.yaw
        ds = distance / samples
        for _ in range(samples):
            if abs(curvature) < 1e-9:
                x += ds * math.cos(yaw)
                y += ds * math.sin(yaw)
            else:
                next_yaw = wrap_to_pi(yaw + curvature * ds)
                x += (math.sin(next_yaw) - math.sin(yaw)) / curvature
                y += (-math.cos(next_yaw) + math.cos(yaw)) / curvature
                yaw = next_yaw
            if not self.grid.is_free_world(x, y):
                return None
        return HybridState(x, y, yaw, direction)

    def successors(self, state: HybridState) -> Iterable[Tuple[HybridState, float]]:
        curvatures = (-self.max_curvature, 0.0, self.max_curvature) if self.max_curvature else (0.0,)
        directions = (1, -1) if self.allow_reverse else (1,)
        for direction in directions:
            for curvature in curvatures:
                child = self._translation_primitive(state, curvature, direction)
                if child is None:
                    continue
                cost = self.motion_step
                if direction < 0:
                    cost *= self.reverse_penalty
                cost += self.turn_penalty * abs(curvature) * self.motion_step
                if direction != state.direction:
                    cost += self.direction_change_penalty
                cost += self.traversal_cost_weight * self.grid.occupancy_penalty(
                    child.x, child.y, proximity_radius=max(0.25, self.motion_step)
                ) * self.motion_step
                yield child, cost
        if self.allow_in_place_rotation:
            for sign in (-1.0, 1.0):
                child = HybridState(state.x, state.y, wrap_to_pi(state.yaw + sign * self.theta_resolution), state.direction)
                yield child, self.rotation_penalty * self.theta_resolution

    def _heuristic(self, state: HybridState, goal: Pose2D) -> float:
        # Euclidean distance is admissible for the non-negative translational costs.
        return math.hypot(goal.x - state.x, goal.y - state.y)

    def _is_goal(self, state: HybridState, goal: Pose2D) -> bool:
        return (
            math.hypot(goal.x - state.x, goal.y - state.y) <= self.goal_position_tolerance
            and abs(wrap_to_pi(goal.yaw - state.yaw)) <= self.goal_yaw_tolerance
        )

    def plan(self, start: Pose2D, goal: Pose2D) -> PlannerResult:
        started = time.perf_counter()
        if not self.grid.is_free_world(start.x, start.y):
            return self._failure(started, "start_outside_or_occupied")
        if not self.grid.is_free_world(goal.x, goal.y):
            return self._failure(started, "goal_outside_or_occupied")
        start_state = HybridState(start.x, start.y, wrap_to_pi(start.yaw), 1)
        start_key = self.state_key(start_state)
        open_heap: List[Tuple[float, int, Tuple[int, int, int, int]]] = []
        states: Dict[Tuple[int, int, int, int], HybridState] = {start_key: start_state}
        parent: Dict[Tuple[int, int, int, int], Optional[Tuple[int, int, int, int]]] = {start_key: None}
        g_cost: Dict[Tuple[int, int, int, int], float] = {start_key: 0.0}
        serial = 0
        heapq.heappush(open_heap, (self.heuristic_weight * self._heuristic(start_state, goal), serial, start_key))
        closed: set[Tuple[int, int, int, int]] = set()
        goal_key: Optional[Tuple[int, int, int, int]] = None
        expanded = 0

        while open_heap and expanded < self.max_iterations:
            _, _, key = heapq.heappop(open_heap)
            if key in closed:
                continue
            state = states[key]
            closed.add(key)
            expanded += 1
            if self._is_goal(state, goal):
                goal_key = key
                break
            for child, transition_cost in self.successors(state):
                child_key = self.state_key(child)
                # State discretisation can map a tiny primitive to itself.
                if child_key == key or child_key in closed:
                    continue
                candidate = g_cost[key] + transition_cost
                if candidate < g_cost.get(child_key, math.inf):
                    states[child_key] = child
                    parent[child_key] = key
                    g_cost[child_key] = candidate
                    serial += 1
                    priority = candidate + self.heuristic_weight * self._heuristic(child, goal)
                    heapq.heappush(open_heap, (priority, serial, child_key))

        if goal_key is None:
            reason = "max_iterations" if expanded >= self.max_iterations else "no_path"
            return self._failure(started, reason, expanded)
        keys = []
        current: Optional[Tuple[int, int, int, int]] = goal_key
        while current is not None:
            keys.append(current)
            current = parent[current]
        keys.reverse()
        poses = [Pose2D(states[key].x, states[key].y, states[key].yaw) for key in keys]
        # Preserve the requested final yaw exactly when the terminal tolerance allows it.
        if poses:
            last = poses[-1]
            poses[-1] = Pose2D(last.x, last.y, goal.yaw)
        elapsed = (time.perf_counter() - started) * 1000.0
        length, clearance, sharp = path_metrics(self.grid, poses)
        return PlannerResult(True, poses, elapsed, expanded, "", length, clearance, sharp)

    @staticmethod
    def _failure(started: float, reason: str, expanded: int = 0) -> PlannerResult:
        return PlannerResult(False, planning_time_ms=(time.perf_counter() - started) * 1000.0,
                             expanded_nodes=expanded, reason=reason)

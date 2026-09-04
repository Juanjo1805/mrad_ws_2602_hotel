#!/usr/bin/env python3
"""Dijkstra grid planner (ROS 2) with practical improvements.

Changes vs v2:
- Optional diagonal "no corner cutting" safety.
- Optional soft traversal cost using occupancy + distance-to-obstacle field.
- Faster, reusable obstacle distance field (brushfire BFS) used for inflation and costs.
- Minor robustness: start/goal blocked checks, last pose yaw kept.

Keeps the same topics/frames and message types.
"""

import math
import heapq
from collections import deque
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.qos import DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_geometry_msgs import do_transform_pose

import tf2_ros


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


class DijkstraNode(Node):
    def __init__(self):
        super().__init__('dijkstra_node')

        # --- Parameters ---
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')

        # Occupancy handling
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('treat_unknown_as_obstacle', True)

        # Graph connectivity
        self.declare_parameter('use_8_connected', True)
        self.declare_parameter('prevent_corner_cutting', True)

        # Obstacle inflation (meters)
        self.declare_parameter('inflate_radius', 0.15)

        # Soft costs (0 => pure geometric Dijkstra)
        # Interpretation: multiplier on [0..1] cell penalty (occupancy + proximity)
        self.declare_parameter('traversal_cost_weight', 0.0)

        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value

        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        qos_map = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_cb, qos_map)

        # Cached map data
        self._map: Optional[OccupancyGrid] = None
        self._grid: Optional[np.ndarray] = None            # int16 occupancy [-1, 0..100], shape (H, W)
        self._obstacles: Optional[np.ndarray] = None       # bool mask, shape (H, W)
        self._dist_cells: Optional[np.ndarray] = None      # int32 distance to nearest obstacle (cells), shape (H, W)

        # TF buffer/listener to get start pose from robot
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Dijkstra planner ready (v3).')

    # ---------------- Map processing ----------------

    def map_cb(self, msg: OccupancyGrid):
        self._map = msg
        W = msg.info.width
        H = msg.info.height
        res = msg.info.resolution

        grid = np.array(msg.data, dtype=np.int16).reshape((H, W))  # row-major: y first
        self._grid = grid

        occ_th = self.get_parameter('occupied_threshold').get_parameter_value().integer_value
        unknown_as_obs = self.get_parameter('treat_unknown_as_obstacle').get_parameter_value().bool_value

        obstacles = (grid >= occ_th)
        if unknown_as_obs:
            obstacles = np.logical_or(obstacles, grid == -1)

        # Precompute distance-to-obstacle field (in cells) from the raw obstacles.
        # This is useful for both inflation and soft traversal costs.
        dist_cells = self.compute_distance_to_obstacles(obstacles)
        self._dist_cells = dist_cells

        # Inflate obstacles if requested (uses distance field: dist <= R).
        inflate_radius = float(self.get_parameter('inflate_radius').get_parameter_value().double_value)
        if inflate_radius > 1e-6:
            inflation_cells = int(math.ceil(inflate_radius / res))
            obstacles = np.logical_or(obstacles, dist_cells <= inflation_cells)

        self._obstacles = obstacles
        self.get_logger().info(f'Map received: {W}x{H}, res={res:.3f} m/px')

    def compute_distance_to_obstacles(self, obstacles: np.ndarray) -> np.ndarray:
        """Brushfire / multi-source BFS distance transform (4-connected).

        Returns dist[y,x] in *cells* to the nearest obstacle cell.
        Obstacle cells have distance 0.
        """
        H, W = obstacles.shape
        INF = 1_000_000
        dist = np.full((H, W), INF, dtype=np.int32)

        q = deque()
        ys, xs = np.nonzero(obstacles)
        for y, x in zip(ys, xs):
            dist[y, x] = 0
            q.append((x, y))

        # If there are no obstacles, dist stays INF everywhere.
        if not q:
            return dist

        nbr4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        while q:
            x, y = q.popleft()
            d = dist[y, x]
            nd = d + 1
            for dx, dy in nbr4:
                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H and dist[ny, nx] > nd:
                    dist[ny, nx] = nd
                    q.append((nx, ny))
        return dist

    # ---------------- Goal handling ----------------

    def goal_cb(self, goal_msg: PoseStamped):
        if self._map is None or self._grid is None or self._obstacles is None:
            self.get_logger().warn('Goal received but map not available yet.')
            return

        global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # Ensure goal is in map frame
        if goal_msg.header.frame_id != global_frame:
            try:
                goal_msg = self.transform_pose(goal_msg, global_frame)
            except Exception as e:
                self.get_logger().error(f'Failed to transform goal to {global_frame}: {e}')
                return

        # Current robot pose (start)
        try:
            start_pose = self.lookup_robot_pose(global_frame, base_frame)
        except Exception as e:
            self.get_logger().error(f'Could not get robot pose: {e}')
            return

        path = self.plan_path(start_pose, goal_msg)
        if path is None:
            self.get_logger().warn('No path found.')
            return

        self.path_pub.publish(path)
        self.get_logger().info(f'Path published with {len(path.poses)} poses.')

    def lookup_robot_pose(self, target_frame: str, base_frame: str) -> PoseStamped:
        stamp = rclpy.time.Time()  # latest
        trans = self.tf_buffer.lookup_transform(target_frame, base_frame, stamp)
        p = PoseStamped()
        p.header.stamp = trans.header.stamp
        p.header.frame_id = target_frame
        p.pose.position.x = trans.transform.translation.x
        p.pose.position.y = trans.transform.translation.y
        p.pose.position.z = trans.transform.translation.z
        p.pose.orientation = trans.transform.rotation
        return p

    def transform_pose(self, pose: PoseStamped, target_frame: str) -> PoseStamped:
        trans = self.tf_buffer.lookup_transform(target_frame, pose.header.frame_id, rclpy.time.Time())
        return do_transform_pose(pose, trans)

    # ---------------- Planner (Dijkstra) ----------------

    def plan_path(self, start: PoseStamped, goal: PoseStamped) -> Optional[Path]:
        """Plan with Dijkstra on an occupancy grid.

        Improvements included:
        - Optional diagonal "corner cutting" prevention.
        - Optional soft traversal cost (occupancy + proximity), controlled by traversal_cost_weight.
        """
        info = self._map.info
        res = info.resolution
        x0 = info.origin.position.x
        y0 = info.origin.position.y
        W, H = info.width, info.height

        s = self.world_to_map(start.pose.position.x, start.pose.position.y, x0, y0, res, W, H)
        g = self.world_to_map(goal.pose.position.x,  goal.pose.position.y,  x0, y0, res, W, H)
        if s is None or g is None:
            self.get_logger().warn('Start or goal outside map bounds.')
            return None
        sx, sy = s
        gx, gy = g

        # Start/goal validity
        if self._obstacles[sy, sx]:
            self.get_logger().warn('Start cell is occupied.')
            return None
        if self._obstacles[gy, gx]:
            self.get_logger().warn('Goal cell is occupied.')
            return None

        use_eight = self.get_parameter('use_8_connected').get_parameter_value().bool_value
        prevent_corner = self.get_parameter('prevent_corner_cutting').get_parameter_value().bool_value

        # Neighbor set (dx, dy, geometric_step_cost)
        neighbors = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
        if use_eight:
            d = math.sqrt(2.0)
            neighbors += [(1, 1, d), (1, -1, d), (-1, 1, d), (-1, -1, d)]

        # Soft-cost weight
        w_soft = float(self.get_parameter('traversal_cost_weight').get_parameter_value().double_value)

        dist = np.full((H, W), np.inf, dtype=np.float64)
        visited = np.zeros((H, W), dtype=bool)
        parent_x = -np.ones((H, W), dtype=np.int32)
        parent_y = -np.ones((H, W), dtype=np.int32)

        dist[sy, sx] = 0.0
        pq: List[Tuple[float, Tuple[int, int]]] = []
        heapq.heappush(pq, (0.0, (sx, sy)))

        # Precompute proximity normalization if available
        dist_cells = self._dist_cells
        prox_scale = 1.0
        if dist_cells is not None:
            # For proximity penalty, distances beyond ~inflate radius are "safe".
            inflate_radius = float(self.get_parameter('inflate_radius').get_parameter_value().double_value)
            inflate_cells = max(1, int(math.ceil(inflate_radius / res)))
            prox_scale = float(inflate_cells)

        while pq:
            cost, (cx, cy) = heapq.heappop(pq)
            if visited[cy, cx]:
                continue
            visited[cy, cx] = True

            if (cx, cy) == (gx, gy):
                break

            for dx, dy, step_cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < W and 0 <= ny < H):
                    continue
                if self._obstacles[ny, nx]:
                    continue

                # Diagonal corner cutting prevention:
                # For a diagonal move, require both adjacent cardinal cells to be free.
                if prevent_corner and dx != 0 and dy != 0:
                    if self._obstacles[cy, nx] or self._obstacles[ny, cx]:
                        continue

                extra = 0.0
                if w_soft > 1e-9:
                    # (A) Occupancy-based penalty (0 for free, up to ~1 near threshold).
                    occ = int(self._grid[ny, nx])
                    if occ < 0:
                        occ_norm = 1.0  # unknown (if not treated as obstacle, still penalize)
                    else:
                        occ_norm = min(1.0, max(0.0, occ / 100.0))

                    # (B) Proximity penalty (1 near obstacles, 0 far away)
                    prox = 0.0
                    if dist_cells is not None:
                        dcell = float(dist_cells[ny, nx])
                        if dcell >= 1_000_000:
                            prox = 0.0
                        else:
                            # within prox_scale cells => penalty close to 1, decays to 0 beyond it
                            prox = max(0.0, 1.0 - (dcell / prox_scale))

                    # Combine both into a [0..1] penalty
                    cell_penalty = 0.5 * occ_norm + 0.5 * prox

                    # Convert to additive cost proportional to step length
                    extra = w_soft * cell_penalty * step_cost

                new_cost = cost + step_cost + extra
                if new_cost < dist[ny, nx]:
                    dist[ny, nx] = new_cost
                    parent_x[ny, nx] = cx
                    parent_y[ny, nx] = cy
                    heapq.heappush(pq, (new_cost, (nx, ny)))

        if not visited[gy, gx]:
            return None

        # Reconstruct path cells
        cells: List[Tuple[int, int]] = []
        cx, cy = gx, gy
        while cx != -1 and cy != -1:
            cells.append((cx, cy))
            px, py = parent_x[cy, cx], parent_y[cy, cx]
            if px == -1 and py == -1:
                break
            cx, cy = px, py
        cells.reverse()

        # Build Path message
        path = Path()
        path.header.frame_id = self.get_parameter('global_frame').get_parameter_value().string_value
        path.header.stamp = self.get_clock().now().to_msg()

        last_yaw = 0.0
        for i, (ix, iy) in enumerate(cells):
            x, y = self.map_to_world(ix, iy, x0, y0, res)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y

            if i + 1 < len(cells):
                nxw, nyw = self.map_to_world(cells[i + 1][0], cells[i + 1][1], x0, y0, res)
                last_yaw = math.atan2(nyw - y, nxw - x)
            pose.pose.orientation = yaw_to_quaternion(last_yaw)

            path.poses.append(pose)

        return path

    # ---------------- Utilities ----------------

    def world_to_map(
        self,
        x: float,
        y: float,
        x0: float,
        y0: float,
        res: float,
        W: int,
        H: int,
    ) -> Optional[Tuple[int, int]]:
        """World (meters) -> grid indices (ix, iy)."""
        ix = int(math.floor((x - x0) / res))
        iy = int(math.floor((y - y0) / res))
        if 0 <= ix < W and 0 <= iy < H:
            return ix, iy
        return None

    def map_to_world(self, ix: int, iy: int, x0: float, y0: float, res: float) -> Tuple[float, float]:
        """Grid indices -> world (meters), at cell center."""
        x = x0 + (ix + 0.5) * res
        y = y0 + (iy + 0.5) * res
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = DijkstraNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
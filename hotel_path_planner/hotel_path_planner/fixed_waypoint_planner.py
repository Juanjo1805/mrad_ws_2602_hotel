#!/usr/bin/env python3
"""Plan one collision-checked route through all fixed waypoints and laps."""

from __future__ import annotations

import math
from pathlib import Path as FilePath
from typing import Any

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import OccupancyGrid, Path

import numpy as np

import rclpy
from rclpy.node import Node

import tf2_ros

import yaml

from .planner_node_utils import map_qos, path_qos
from .planning_core import (DijkstraPlanner, GridMap, HybridAStarPlanner,
                            Pose2D, densify_path, quaternion_to_yaw,
                            yaw_to_quaternion)


class FixedWaypointPlanner(Node):
    """Concatenate a global plan for every leg of a fixed multi-lap route."""

    def __init__(self) -> None:
        """Configure map input, planner selection, and route output."""
        super().__init__('fixed_waypoint_planner')
        config_dir = FilePath(__file__).resolve().parents[1] / 'config'
        self.declare_parameter(
            'waypoints_file', str(config_dir / 'fixed_waypoints.yaml'))
        self.declare_parameter('planner', 'dijkstra')
        self.declare_parameter('laps', 2)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('route_topic', '/planned_path')
        self.declare_parameter('path_resolution', 0.05)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('treat_unknown_as_obstacle', True)
        self.declare_parameter('inflate_radius', 0.25)
        self.declare_parameter('traversal_cost_weight', 0.0)
        self.declare_parameter('use_8_connected', True)
        self.declare_parameter('prevent_corner_cutting', True)
        self.declare_parameter('xy_resolution', 0.05)
        self.declare_parameter('theta_resolution', math.radians(15.0))
        self.declare_parameter('motion_step', 0.20)
        self.declare_parameter('heuristic_weight', 1.0)
        self.declare_parameter('max_curvature', 1.6)
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('allow_in_place_rotation', True)
        self.declare_parameter('reverse_penalty', 1.4)
        self.declare_parameter('turn_penalty', 0.10)
        self.declare_parameter('direction_change_penalty', 0.25)
        self.declare_parameter('rotation_penalty', 0.12)
        self.declare_parameter('goal_position_tolerance', 0.15)
        self.declare_parameter('goal_yaw_tolerance', math.radians(20.0))
        self.declare_parameter('max_iterations', 100000)

        waypoints_file = self.get_parameter('waypoints_file').value
        self.waypoints_file = FilePath(waypoints_file).expanduser()
        self.planner_name = str(self.get_parameter('planner').value)
        self.laps = max(1, int(self.get_parameter('laps').value))
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.route_topic = str(self.get_parameter('route_topic').value)
        self.frame_id = 'map'
        self.waypoints: list[tuple[float, float]] = []
        self.grid: GridMap | None = None
        self.finished = False
        self._load_waypoints()

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._map_callback,
            map_qos(),
        )
        self.route_pub = self.create_publisher(Path, self.route_topic, path_qos())
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self._try_plan)
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints x {self.laps} laps; '
            f'waiting for map and TF to run {self.planner_name}.')

    def _load_waypoints(self) -> None:
        """Read valid XY points and their map frame from the YAML file."""
        try:
            with self.waypoints_file.open('r', encoding='utf-8') as stream:
                data: dict[str, Any] = yaml.safe_load(stream) or {}
        except (OSError, yaml.YAMLError) as error:
            self.get_logger().error(
                f'Unable to load {self.waypoints_file}: {error}')
            return
        yaml_frame = data.get('frame_id', 'map')
        if isinstance(yaml_frame, str) and yaml_frame:
            self.frame_id = yaml_frame
        for index, item in enumerate(data.get('waypoints', [])):
            try:
                x, y = float(item['x']), float(item['y'])
                if not math.isfinite(x) or not math.isfinite(y):
                    raise ValueError('coordinate is not finite')
                self.waypoints.append((x, y))
            except (KeyError, TypeError, ValueError):
                self.get_logger().warning(
                    f'Ignoring invalid waypoint {index}.')

    def _map_callback(self, message: OccupancyGrid) -> None:
        """Convert the ROS occupancy grid to the shared representation."""
        try:
            if message.header.frame_id != self.frame_id:
                raise ValueError(
                    f"map frame '{message.header.frame_id}' differs from route frame '{self.frame_id}'"
                )
            occupancy = np.asarray(message.data, dtype=np.int16).reshape(
                (message.info.height, message.info.width))
            self.grid = GridMap(
                occupancy,
                message.info.resolution,
                message.info.origin.position.x,
                message.info.origin.position.y,
                int(self.get_parameter('occupied_threshold').value),
                bool(self.get_parameter('treat_unknown_as_obstacle').value),
                float(self.get_parameter('inflate_radius').value),
                quaternion_to_yaw(message.info.origin.orientation),
            )
        except (TypeError, ValueError) as error:
            self.grid = None
            self.get_logger().error(f'Invalid map: {error}')

    def _planner(self):
        """Instantiate the selected planner with the A/B parameters."""
        if self.planner_name == 'dijkstra':
            return DijkstraPlanner(
                self.grid,
                use_8_connected=bool(
                    self.get_parameter('use_8_connected').value),
                prevent_corner_cutting=bool(
                    self.get_parameter('prevent_corner_cutting').value),
                traversal_cost_weight=float(
                    self.get_parameter('traversal_cost_weight').value),
            )
        if self.planner_name == 'hybrid_astar':
            return HybridAStarPlanner(
                self.grid,
                xy_resolution=float(
                    self.get_parameter('xy_resolution').value),
                theta_resolution=float(
                    self.get_parameter('theta_resolution').value),
                motion_step=float(self.get_parameter('motion_step').value),
                heuristic_weight=float(
                    self.get_parameter('heuristic_weight').value),
                max_curvature=float(
                    self.get_parameter('max_curvature').value),
                allow_reverse=bool(
                    self.get_parameter('allow_reverse').value),
                allow_in_place_rotation=bool(
                    self.get_parameter('allow_in_place_rotation').value),
                reverse_penalty=float(
                    self.get_parameter('reverse_penalty').value),
                turn_penalty=float(self.get_parameter('turn_penalty').value),
                direction_change_penalty=float(
                    self.get_parameter('direction_change_penalty').value),
                rotation_penalty=float(
                    self.get_parameter('rotation_penalty').value),
                goal_position_tolerance=float(
                    self.get_parameter('goal_position_tolerance').value),
                goal_yaw_tolerance=float(
                    self.get_parameter('goal_yaw_tolerance').value),
                max_iterations=int(
                    self.get_parameter('max_iterations').value),
                traversal_cost_weight=float(
                    self.get_parameter('traversal_cost_weight').value),
            )
        raise ValueError("planner must be 'dijkstra' or 'hybrid_astar'")

    @staticmethod
    def _target_yaw(
            target: tuple[float, float],
            next_target: tuple[float, float],
    ) -> float:
        """Set a continuous target heading toward the next waypoint."""
        return math.atan2(
            next_target[1] - target[1], next_target[0] - target[0])

    def _try_plan(self) -> None:
        """Wait for prerequisites and publish the concatenated plan once."""
        if self.finished or self.grid is None:
            return
        if len(self.waypoints) < 2:
            self.get_logger().error(
                'At least two valid waypoints are required.')
            self.finished = True
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id, self.base_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return

        robot = transform.transform
        sin_yaw = 2.0 * (
            robot.rotation.w * robot.rotation.z +
            robot.rotation.x * robot.rotation.y)
        cos_yaw = 1.0 - 2.0 * (
            robot.rotation.y * robot.rotation.y +
            robot.rotation.z * robot.rotation.z)
        current = Pose2D(
            robot.translation.x,
            robot.translation.y,
            math.atan2(sin_yaw, cos_yaw),
        )
        targets = self.waypoints * self.laps
        planner = self._planner()
        complete_path: list[Pose2D] = []
        total_time_ms = 0.0
        for index, target in enumerate(targets):
            next_target = targets[(index + 1) % len(targets)]
            goal = Pose2D(
                target[0], target[1], self._target_yaw(target, next_target))
            result = planner.plan(current, goal)
            total_time_ms += result.planning_time_ms
            if not result.success:
                self.get_logger().error(
                    f'Route aborted at segment {index + 1}/{len(targets)}: '
                    f'{result.reason}')
                self.finished = True
                return
            segment = result.poses
            if complete_path:
                segment = segment[1:]
            complete_path.extend(segment)
            current = result.poses[-1]

        dense_path = densify_path(
            complete_path, float(self.get_parameter('path_resolution').value))
        route = Path()
        route.header.frame_id = self.frame_id
        route.header.stamp = self.get_clock().now().to_msg()
        for item in dense_path:
            pose = PoseStamped()
            pose.header = route.header
            pose.pose.position.x = item.x
            pose.pose.position.y = item.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x, pose.pose.orientation.y, \
                pose.pose.orientation.z, pose.pose.orientation.w = \
                yaw_to_quaternion(item.yaw)
            route.poses.append(pose)
        self.route_pub.publish(route)
        self.finished = True
        self.get_logger().info(
            f'{self.planner_name} published {len(route.poses)} dense poses for '
            f'{len(targets)} segments in {total_time_ms:.1f} ms on '
            f'{self.route_topic}.')


def main(args: list[str] | None = None) -> None:
    """Start the multi-waypoint global planner."""
    rclpy.init(args=args)
    node = FixedWaypointPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

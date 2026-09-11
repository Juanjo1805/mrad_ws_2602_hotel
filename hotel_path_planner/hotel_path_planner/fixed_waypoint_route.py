#!/usr/bin/env python3
"""Publish all editable waypoints as one repeated route for a path tracker."""

from __future__ import annotations

import math
from pathlib import Path as FilePath
from typing import Any

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Path

import rclpy
from rclpy.node import Node
import yaml

from .planner_node_utils import path_qos
from .planning_core import Pose2D, densify_path, yaw_to_quaternion


class FixedWaypointRoute(Node):
    """Load XY points and publish a complete multi-lap ``nav_msgs/Path``."""

    def __init__(self) -> None:
        """Configure the route file, number of laps, and output topic."""
        super().__init__('fixed_waypoint_route')
        config_dir = FilePath(__file__).resolve().parents[1] / 'config'
        self.declare_parameter(
            'waypoints_file', str(config_dir / 'fixed_waypoints.yaml'))
        self.declare_parameter('route_topic', '/fixed_waypoints/route')
        self.declare_parameter('laps', 2)
        self.declare_parameter('path_resolution', 0.05)

        waypoints_file = self.get_parameter('waypoints_file').value
        self.waypoints_file = FilePath(waypoints_file).expanduser()
        self.route_topic = str(self.get_parameter('route_topic').value)
        self.laps = max(1, int(self.get_parameter('laps').value))
        self.frame_id = 'map'
        self.waypoints: list[tuple[float, float]] = []
        self._load_waypoints()

        self.path_resolution = max(
            0.01, float(self.get_parameter('path_resolution').value))
        self.route_pub = self.create_publisher(Path, self.route_topic, path_qos())
        self._publish_route()

    def _load_waypoints(self) -> None:
        """Load XY points from the fixed-waypoint YAML configuration."""
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
        raw_waypoints = data.get('waypoints', [])
        if not isinstance(raw_waypoints, list):
            self.get_logger().error("'waypoints' must be a YAML list.")
            return
        for index, item in enumerate(raw_waypoints):
            try:
                x, y = float(item['x']), float(item['y'])
                if not math.isfinite(x) or not math.isfinite(y):
                    raise ValueError('coordinate is not finite')
                self.waypoints.append((x, y))
            except (KeyError, TypeError, ValueError):
                self.get_logger().warning(
                    f'Ignoring invalid waypoint {index}.')

    @staticmethod
    def _yaw_to_next(
            point: tuple[float, float],
            next_point: tuple[float, float],
    ) -> float:
        """Return the tangent yaw from one route point to the following one."""
        return math.atan2(next_point[1] - point[1], next_point[0] - point[0])

    def _publish_route(self) -> None:
        """Publish all waypoints repeated by ``laps`` in one Path message."""
        if len(self.waypoints) < 2:
            self.get_logger().error(
                'At least two valid waypoints are required.')
            return
        route_points = self.waypoints * self.laps
        coarse_path = [
            Pose2D(
                point[0], point[1],
                self._yaw_to_next(point, route_points[(index + 1) % len(route_points)]),
            )
            for index, point in enumerate(route_points)
        ]
        dense_path = densify_path(coarse_path, self.path_resolution)
        route = Path()
        route.header.frame_id = self.frame_id
        route.header.stamp = self.get_clock().now().to_msg()
        for point in dense_path:
            pose = PoseStamped()
            pose.header = route.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x, pose.pose.orientation.y, \
                pose.pose.orientation.z, pose.pose.orientation.w = \
                yaw_to_quaternion(point.yaw)
            route.poses.append(pose)
        self.route_pub.publish(route)
        self.get_logger().info(
            f'Published {len(route.poses)} dense poses at '
            f'{self.path_resolution:.2f} m: '
            f'{len(self.waypoints)} points '
            f'x {self.laps} lap(s) on {self.route_topic}.')


def main(args: list[str] | None = None) -> None:
    """Start the one-shot fixed waypoint route publisher."""
    rclpy.init(args=args)
    node = FixedWaypointRoute()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

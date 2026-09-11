#!/usr/bin/env python3
"""Send editable YAML waypoints one at a time to the global planner."""

from __future__ import annotations

import math
from pathlib import Path as FilePath
from typing import Any

from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node

import tf2_ros

import yaml


class FixedWaypointMission(Node):
    """Advance through fixed map points after the robot reaches each one."""

    def __init__(self) -> None:
        """Load the YAML mission and configure its planner goal publisher."""
        super().__init__('fixed_waypoint_mission')
        config_dir = FilePath(__file__).resolve().parents[1] / 'config'
        self.declare_parameter(
            'waypoints_file', str(config_dir / 'fixed_waypoints.yaml'))
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter(
            'current_goal_topic', '/fixed_waypoints/current_goal')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('goal_tolerance', 0.45)
        self.declare_parameter('loop', False)
        self.declare_parameter('autostart', True)

        self.waypoints_file = FilePath(
            self.get_parameter('waypoints_file').value).expanduser()
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.frame_id = 'map'
        self.waypoints: list[dict[str, float]] = []
        self.current_index = 0
        self.goal_sent = False
        self.active = bool(self.get_parameter('autostart').value)

        goal_topic = str(self.get_parameter('goal_topic').value)
        current_goal_topic = str(
            self.get_parameter('current_goal_topic').value)
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.current_goal_pub = self.create_publisher(
            PoseStamped, current_goal_topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._load_waypoints()
        self.timer = self.create_timer(0.2, self._tick)

        state = 'started' if self.active else 'paused'
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} fixed waypoints; mission {state}.')
        self.get_logger().info(
            f'Publishing each goal on {goal_topic}; arrival tolerance is '
            f'{self.goal_tolerance:.2f} m.')

    def _load_waypoints(self) -> None:
        """Load valid XY waypoint mappings from the YAML configuration."""
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
            if not isinstance(item, dict):
                self.get_logger().warning(
                    f'Ignoring waypoint {index}: expected a mapping.')
                continue
            try:
                self.waypoints.append({
                    'x': float(item['x']),
                    'y': float(item['y']),
                })
            except (KeyError, TypeError, ValueError):
                self.get_logger().warning(
                    f'Ignoring invalid waypoint {index}.')

    def _goal_message(self) -> PoseStamped:
        """Build the current planner goal as a pose in the YAML frame."""
        waypoint = self.waypoints[self.current_index]
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = waypoint['x']
        goal.pose.position.y = waypoint['y']
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        return goal

    def _send_current_goal(self) -> None:
        """Publish the next mission target to the planner exactly once."""
        goal = self._goal_message()
        self.goal_pub.publish(goal)
        self.current_goal_pub.publish(goal)
        self.goal_sent = True
        self.get_logger().info(
            f'Sent waypoint {self.current_index + 1}/{len(self.waypoints)}: '
            f'x={goal.pose.position.x:.3f}, y={goal.pose.position.y:.3f}')

    def _robot_reached_goal(self) -> bool:
        """Return whether the robot's map position is close to this goal."""
        transform = self.tf_buffer.lookup_transform(
            self.frame_id, self.base_frame, rclpy.time.Time())
        robot = transform.transform.translation
        waypoint = self.waypoints[self.current_index]
        distance = math.hypot(waypoint['x'] - robot.x, waypoint['y'] - robot.y)
        return distance <= self.goal_tolerance

    def _advance(self) -> None:
        """Advance to the following target or finish the mission."""
        self.current_index += 1
        if self.current_index < len(self.waypoints):
            self.goal_sent = False
            return
        if self.loop:
            self.current_index = 0
            self.goal_sent = False
            self.get_logger().info('Completed lap; restarting at waypoint 1.')
            return
        self.active = False
        self.get_logger().info('Fixed waypoint mission completed.')

    def _tick(self) -> None:
        """Send a target or wait until the robot reaches the active target."""
        if not self.active:
            return
        if not self.waypoints:
            self.get_logger().error('Mission stopped: no valid waypoints.')
            self.active = False
            return
        if not self.goal_sent:
            self._send_current_goal()
            return
        try:
            if self._robot_reached_goal():
                self.get_logger().info(
                    f'Reached waypoint {self.current_index + 1}.')
                self._advance()
        except tf2_ros.TransformException as error:
            self.get_logger().warning(
                f'Waiting for TF {self.frame_id} -> {self.base_frame}: '
                f'{error}')


def main(args: list[str] | None = None) -> None:
    """Start the fixed waypoint mission node."""
    rclpy.init(args=args)
    node = FixedWaypointMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

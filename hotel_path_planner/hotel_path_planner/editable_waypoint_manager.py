#!/usr/bin/env python3
"""Editable fixed waypoints, persisted to YAML and displayed in RViz."""

from __future__ import annotations

import os
from pathlib import Path as FilePath
from typing import Any

from geometry_msgs.msg import Pose, PoseArray, PoseStamped

from interactive_markers.interactive_marker_server import (
    InteractiveMarkerServer,
)

from nav_msgs.msg import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)

import yaml


class EditableWaypointManager(Node):
    """Maintain map waypoints and expose them as RViz interactive markers."""

    def __init__(self) -> None:
        """Configure waypoint persistence, ROS publishers, and RViz markers."""
        super().__init__('editable_waypoint_manager')

        source_config = FilePath(__file__).resolve().parents[1] / 'config'
        default_file = source_config / 'fixed_waypoints.yaml'
        self.declare_parameter('waypoints_file', str(default_file))
        self.declare_parameter('topic_prefix', '/fixed_waypoints')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('marker_scale', 0.28)

        self.waypoints_file = FilePath(
            self.get_parameter('waypoints_file').value).expanduser()
        self.topic_prefix = str(
            self.get_parameter('topic_prefix').value).rstrip('/')
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.marker_scale = float(self.get_parameter('marker_scale').value)
        self.waypoints: list[dict[str, float]] = []

        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, f'{self.topic_prefix}/markers', latched_qos)
        self.path_pub = self.create_publisher(
            Path, f'{self.topic_prefix}/path', latched_qos)
        self.poses_pub = self.create_publisher(
            PoseArray, f'{self.topic_prefix}/poses', latched_qos)

        # This namespace makes RViz's update topic /fixed_waypoints/update.
        self.server = InteractiveMarkerServer(self, self.topic_prefix)
        self._load_waypoints()
        self._refresh_visualization()
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from '
            f'{self.waypoints_file}')
        self.get_logger().info(
            f'RViz topics: {self.topic_prefix}/markers, '
            f'{self.topic_prefix}/path, '
            f'{self.topic_prefix}/poses and {self.topic_prefix}/update')

    def _load_waypoints(self) -> None:
        """Load and validate the waypoint YAML file."""
        try:
            with self.waypoints_file.open('r', encoding='utf-8') as stream:
                data: dict[str, Any] = yaml.safe_load(stream) or {}
        except (OSError, yaml.YAMLError) as error:
            self.get_logger().error(
                f'Unable to load {self.waypoints_file}: {error}; '
                'starting empty.')
            data = {}

        yaml_frame = data.get('frame_id')
        if isinstance(yaml_frame, str) and yaml_frame:
            self.frame_id = yaml_frame

        raw_waypoints = data.get('waypoints', [])
        if not isinstance(raw_waypoints, list):
            self.get_logger().error(
                "'waypoints' must be a YAML list; starting empty.")
            raw_waypoints = []

        for index, item in enumerate(raw_waypoints):
            if not isinstance(item, dict):
                self.get_logger().warn(
                    f'Ignoring waypoint {index}: expected a mapping.')
                continue
            try:
                self.waypoints.append({
                    'x': float(item['x']),
                    'y': float(item['y']),
                })
            except (KeyError, TypeError, ValueError):
                self.get_logger().warn(f'Ignoring invalid waypoint {index}.')

    def _save_waypoints(self) -> None:
        """Atomically save current waypoints to their configured YAML path."""
        document = {'frame_id': self.frame_id, 'waypoints': self.waypoints}
        temporary_file = self.waypoints_file.with_suffix(
            f'{self.waypoints_file.suffix}.tmp')
        try:
            self.waypoints_file.parent.mkdir(parents=True, exist_ok=True)
            with temporary_file.open('w', encoding='utf-8') as stream:
                yaml.safe_dump(document, stream, sort_keys=False)
            os.replace(temporary_file, self.waypoints_file)
        except OSError as error:
            self.get_logger().error(
                f'Unable to save {self.waypoints_file}: {error}')

    def _pose_from_waypoint(self, waypoint: dict[str, float]) -> Pose:
        """Build a Pose message from an internal waypoint."""
        pose = Pose()
        pose.position.x = waypoint['x']
        pose.position.y = waypoint['y']
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        return pose

    def _make_interactive_marker(
            self, index: int, waypoint: dict[str, float]) -> InteractiveMarker:
        """Create an XY-draggable marker for one waypoint."""
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = f'waypoint_{index}'
        marker.description = f'Waypoint {index + 1}'
        marker.scale = max(self.marker_scale * 3.0, 0.8)
        marker.pose = self._pose_from_waypoint(waypoint)

        visual = Marker()
        visual.type = Marker.SPHERE
        visual.pose.position.z = 0.0
        visual.scale.x = self.marker_scale
        visual.scale.y = self.marker_scale
        visual.scale.z = self.marker_scale
        visual.color.r = 0.1
        visual.color.g = 0.75
        visual.color.b = 1.0
        visual.color.a = 0.9

        visible_control = InteractiveMarkerControl()
        visible_control.always_visible = True
        visible_control.markers.append(visual)
        marker.controls.append(visible_control)

        move_control = InteractiveMarkerControl()
        move_control.name = 'move_xy'
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        # MOVE_PLANE uses the control X axis as the plane normal.  Rotate
        # that axis onto global Z so the draggable plane is strictly XY.
        move_control.orientation.w = 0.70710678
        move_control.orientation.y = 0.70710678
        marker.controls.append(move_control)

        return marker

    def _feedback_callback(self, feedback: InteractiveMarkerFeedback) -> None:
        """Persist an XY waypoint while forcing it back onto the map plane."""
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        try:
            index = int(feedback.marker_name.removeprefix('waypoint_'))
            waypoint = self.waypoints[index]
        except (IndexError, ValueError):
            self.get_logger().warn(
                f'Ignoring feedback from unknown marker '
                f'{feedback.marker_name}.')
            return

        fixed_pose = feedback.pose
        fixed_pose.position.z = 0.0
        self.server.setPose(feedback.marker_name, fixed_pose)
        self.server.applyChanges()
        waypoint['x'] = fixed_pose.position.x
        waypoint['y'] = fixed_pose.position.y
        self._save_waypoints()
        self._publish_standard_messages()
        self.get_logger().info(
            f'Saved waypoint {index + 1}: x={waypoint["x"]:.3f}, '
            f'y={waypoint["y"]:.3f}')

    def _publish_standard_messages(self) -> None:
        """Publish regular RViz messages for all waypoints."""
        now = self.get_clock().now().to_msg()
        poses = [self._pose_from_waypoint(item) for item in self.waypoints]

        pose_array = PoseArray()
        pose_array.header.frame_id = self.frame_id
        pose_array.header.stamp = now
        pose_array.poses = poses
        self.poses_pub.publish(pose_array)

        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = now
        for pose in poses:
            item = PoseStamped()
            item.header = path.header
            item.pose = pose
            path.poses.append(item)
        self.path_pub.publish(path)

        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)
        for index, pose in enumerate(poses):
            sphere = Marker()
            sphere.header = path.header
            sphere.ns = 'fixed_waypoints'
            sphere.id = index * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose
            sphere.scale.x = self.marker_scale
            sphere.scale.y = self.marker_scale
            sphere.scale.z = self.marker_scale
            sphere.color.r = 0.1
            sphere.color.g = 0.75
            sphere.color.b = 1.0
            sphere.color.a = 0.95
            marker_array.markers.append(sphere)

            label = Marker()
            label.header = path.header
            label.ns = 'fixed_waypoints_labels'
            label.id = index * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = pose
            label.pose.position.z = 0.02
            label.scale.z = self.marker_scale
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'WP {index + 1}'
            marker_array.markers.append(label)
        self.markers_pub.publish(marker_array)

    def _refresh_visualization(self) -> None:
        """Rebuild RViz interactive markers and regular visualizations."""
        self.server.clear()
        for index, waypoint in enumerate(self.waypoints):
            marker = self._make_interactive_marker(index, waypoint)
            self.server.insert(
                marker, feedback_callback=self._feedback_callback)
        self.server.applyChanges()
        self._publish_standard_messages()


def main(args: list[str] | None = None) -> None:
    """Start the editable waypoint manager."""
    rclpy.init(args=args)
    node = EditableWaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

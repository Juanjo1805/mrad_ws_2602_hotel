"""Small RViz-facing diagnostics shared by the path trackers."""

from __future__ import annotations

import math
from typing import Sequence

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

from .tracking_core import Pose2D, TrackingCommand


def visualization_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class TrackingVisualization:
    """Publish the executed trace and the controller's active geometry."""

    def __init__(self, node, prefix: str = '/path_tracking') -> None:
        prefix = prefix.rstrip('/') or '/path_tracking'
        qos = visualization_qos()
        self.node = node
        self.executed_pub = node.create_publisher(Path, f'{prefix}/executed_path', qos)
        self.closest_pub = node.create_publisher(Marker, f'{prefix}/closest_point', qos)
        self.lookahead_pub = node.create_publisher(Marker, f'{prefix}/lookahead_point', qos)
        self.goal_pub = node.create_publisher(Marker, f'{prefix}/goal', qos)
        self.executed = Path()
        self.frame_id = ''

    @staticmethod
    def _pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _marker(self, frame_id: str, stamp, marker_id: int, namespace: str,
                point: Pose2D, red: float, green: float, blue: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = self._pose(point.x, point.y, point.yaw).pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.16
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.95
        return marker

    def reset(self, frame_id: str) -> None:
        self.frame_id = frame_id
        self.executed = Path()
        self.executed.header.frame_id = frame_id

    def publish(self, frame_id: str, stamp, robot: Pose2D,
                path: Sequence[Pose2D], command: TrackingCommand) -> None:
        if frame_id != self.frame_id:
            self.reset(frame_id)
        self.executed.header.frame_id = frame_id
        self.executed.header.stamp = stamp
        robot_pose = self._pose(robot.x, robot.y, robot.yaw)
        robot_pose.header = self.executed.header
        self.executed.poses.append(robot_pose)
        # Keep the diagnostic bounded on an indefinitely looping mission.
        if len(self.executed.poses) > 10000:
            self.executed.poses.pop(0)
        self.executed_pub.publish(self.executed)

        target = command.reference
        closest = path[min(max(command.closest_index, 0), len(path) - 1)]
        goal = path[-1]
        self.closest_pub.publish(
            self._marker(frame_id, stamp, 0, 'closest', closest, 0.1, 0.9, 1.0))
        self.lookahead_pub.publish(
            self._marker(frame_id, stamp, 0, 'lookahead', target, 1.0, 0.85, 0.0))
        self.goal_pub.publish(
            self._marker(frame_id, stamp, 0, 'goal', goal, 1.0, 0.1, 0.2))

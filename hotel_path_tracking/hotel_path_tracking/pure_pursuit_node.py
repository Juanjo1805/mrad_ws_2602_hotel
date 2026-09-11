#!/usr/bin/env python3
"""Frame-safe Pure Pursuit tracker for the differential Hotel robot."""

from __future__ import annotations

import math
from typing import List

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros
from tf2_ros import TransformException

from .tracking_core import Pose2D, PurePursuitReference, TrackingCommand, target_in_robot_frame
from .tracking_metrics import TrackingCsvLogger
from .tracking_visualization import TrackingVisualization


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def path_qos() -> QoSProfile:
    """Match the planner's latched Path publication."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class PurePursuitNode(Node):
    """Follow a Path in its own frame using a periodic TF-based control loop."""

    def __init__(self, node_name: str = 'pure_pursuit_node') -> None:
        super().__init__(node_name)
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('control_frequency', 40.0)
        self.declare_parameter('control_rate_hz', float('nan'))  # legacy alias
        self.declare_parameter('linear_velocity', 0.45)
        self.declare_parameter('v_nominal', float('nan'))  # legacy alias
        self.declare_parameter('max_linear_velocity', 0.50)
        self.declare_parameter('max_speed', float('nan'))  # legacy alias
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('max_omega', float('nan'))  # legacy alias
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('yaw_tolerance', math.radians(12.0))
        self.declare_parameter('goal_yaw_gain', 1.5)
        self.declare_parameter('goal_progress_fraction', 0.95)
        self.declare_parameter('max_progress_jump_distance', 1.5)
        self.declare_parameter('slow_radius', 1.2)
        self.declare_parameter('lookahead_distance', 0.60)
        self.declare_parameter('lookahead_L0', float('nan'))  # legacy alias
        self.declare_parameter('lookahead_gain', 1.3)
        self.declare_parameter('lookahead_kv', float('nan'))  # legacy alias
        self.declare_parameter('minimum_lookahead', 0.30)
        self.declare_parameter('lookahead_min', float('nan'))  # legacy alias
        self.declare_parameter('maximum_lookahead', 1.20)
        self.declare_parameter('lookahead_max', float('nan'))  # legacy alias
        self.declare_parameter('max_curvature', 1.6)
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('visualization_prefix', '/path_tracking')
        self.declare_parameter('scenario', 'manual')
        self.declare_parameter('trial', 0)
        self.declare_parameter('environment', 'gazebo')
        self.declare_parameter('trace_csv', 'results/tracker_trace.csv')
        self.declare_parameter('results_csv', 'results/tracker_results.csv')

        self.path_topic = str(self.get_parameter('path_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.rate_hz = max(1.0, self._value('control_frequency', 'control_rate_hz'))
        self.tf_timeout = max(0.0, float(self.get_parameter('tf_timeout_sec').value))
        self.tracker = PurePursuitReference(
            lookahead_l0=self._value('lookahead_distance', 'lookahead_L0'),
            lookahead_kv=self._value('lookahead_gain', 'lookahead_kv'),
            lookahead_min=self._value('minimum_lookahead', 'lookahead_min'),
            lookahead_max=self._value('maximum_lookahead', 'lookahead_max'),
            v_nominal=self._value('linear_velocity', 'v_nominal'),
            min_speed=max(0.0, float(self.get_parameter('min_speed').value)),
            max_speed=self._value('max_linear_velocity', 'max_speed'),
            max_omega=self._value('max_angular_velocity', 'max_omega'),
            max_curvature=max(0.0, float(self.get_parameter('max_curvature').value)),
            goal_tolerance=max(0.0, float(self.get_parameter('goal_tolerance').value)),
            yaw_tolerance=max(0.0, float(self.get_parameter('yaw_tolerance').value)),
            goal_yaw_gain=max(0.0, float(self.get_parameter('goal_yaw_gain').value)),
            goal_progress_fraction=float(self.get_parameter('goal_progress_fraction').value),
            max_progress_jump_distance=max(
                0.0, float(self.get_parameter('max_progress_jump_distance').value)),
            slow_radius=max(1e-3, float(self.get_parameter('slow_radius').value)),
        )
        if self.tracker.lmax < self.tracker.lmin:
            self.tracker.lmax = self.tracker.lmin

        self.metrics = TrackingCsvLogger(
            'pure_pursuit_pt_2602_hotel',
            str(self.get_parameter('scenario').value),
            int(self.get_parameter('trial').value),
            str(self.get_parameter('trace_csv').value),
            str(self.get_parameter('results_csv').value),
            environment=str(self.get_parameter('environment').value),
        )
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, path_qos())
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.visualization = TrackingVisualization(
            self, str(self.get_parameter('visualization_prefix').value))

        self.path: List[Pose2D] = []
        self.path_frame = ''
        self.has_path = False
        self.goal_reached = False
        self.last_path_timestamp = None
        self.last_diagnostic_time = -math.inf
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.get_logger().info(
            f'Pure Pursuit ready: path={self.path_topic}, cmd={self.cmd_topic}, '
            f'rate={self.rate_hz:.1f} Hz, Ld={self.tracker.l0:.2f} m.'
        )

    def _value(self, canonical: str, legacy: str) -> float:
        """Prefer a configured legacy parameter without making it mandatory."""
        old_value = float(self.get_parameter(legacy).value)
        if math.isfinite(old_value):
            return old_value
        return float(self.get_parameter(canonical).value)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _decode_path(self, message: Path) -> List[Pose2D]:
        frame = message.header.frame_id
        if not frame:
            raise ValueError('Path.header.frame_id is empty')
        decoded: List[Pose2D] = []
        for index, pose_stamped in enumerate(message.poses):
            pose_frame = pose_stamped.header.frame_id
            if pose_frame and pose_frame != frame:
                raise ValueError(
                    f'pose {index} has frame {pose_frame!r}, expected {frame!r}')
            pose = pose_stamped.pose
            values = (pose.position.x, pose.position.y, pose.orientation.x,
                      pose.orientation.y, pose.orientation.z, pose.orientation.w)
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f'pose {index} contains a non-finite value')
            norm = math.sqrt(sum(value * value for value in values[2:]))
            if norm < 1e-6:
                raise ValueError(f'pose {index} has an invalid zero quaternion')
            decoded.append(Pose2D(pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)))
        if not decoded:
            raise ValueError('Path contains no poses')
        return decoded

    def on_path(self, message: Path) -> None:
        self.has_path = False
        self.goal_reached = False
        self.tracker.reset()
        try:
            self.path = self._decode_path(message)
            self.path_frame = message.header.frame_id
        except ValueError as error:
            self.path = []
            self.path_frame = ''
            self.publish_stop()
            self.get_logger().error(f'Rejected planned path: {error}')
            return
        self.has_path = True
        self.last_path_timestamp = message.header.stamp
        self.visualization.reset(self.path_frame)
        self.metrics.start(self.now_s(), len(self.path))
        self.get_logger().info(
            f'Received Path: poses={len(self.path)}, frame={self.path_frame}, '
            f'stamp={message.header.stamp.sec}.{message.header.stamp.nanosec:09d}.'
        )

    def robot_pose_in_path_frame(self) -> Pose2D:
        transform = self.tf_buffer.lookup_transform(
            self.path_frame,
            self.base_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=self.tf_timeout),
        )
        translation = transform.transform.translation
        return Pose2D(
            translation.x,
            translation.y,
            yaw_from_quaternion(transform.transform.rotation),
        )

    def _diagnostic(self, robot: Pose2D, command: TrackingCommand) -> None:
        now = self.now_s()
        if now - self.last_diagnostic_time < 1.0:
            return
        self.last_diagnostic_time = now
        target_x, target_y = target_in_robot_frame(robot, command.reference)
        alpha = math.atan2(target_y, target_x)
        curvature = 0.0 if abs(command.linear_velocity) < 1e-6 else \
            command.angular_velocity / command.linear_velocity
        self.get_logger().info(
            '[PurePursuit] '
            f'path_points={len(self.path)} robot=({robot.x:.3f},{robot.y:.3f},{robot.yaw:.3f}) '
            f'closest_idx={command.closest_index} target_idx={command.reference_index} '
            f'target_global=({command.reference.x:.3f},{command.reference.y:.3f}) '
            f'target_robot=({target_x:.3f},{target_y:.3f}) Ld={command.lookahead_distance:.3f} '
            f'alpha={alpha:.3f} curvature={curvature:.3f} '
            f'cmd=(v={command.linear_velocity:.3f},w={command.angular_velocity:.3f}).'
        )

    def _warning(self, text: str) -> None:
        now = self.now_s()
        if now - self.last_diagnostic_time >= 1.0:
            self.last_diagnostic_time = now
            self.get_logger().warning(text)

    def on_timer(self) -> None:
        if not self.has_path:
            self.publish_stop()
            return
        try:
            robot = self.robot_pose_in_path_frame()
            command = self.tracker.command(self.path, robot)
        except (TransformException, ValueError, FloatingPointError) as error:
            self.publish_stop()
            self._warning(
                f'Pure Pursuit stopped: cannot obtain {self.path_frame} -> {self.base_frame}: {error}')
            return

        self.publish(command)
        stamp = self.get_clock().now().to_msg()
        self.visualization.publish(self.path_frame, stamp, robot, self.path, command)
        self.metrics.sample(self.now_s(), robot, command)
        self._diagnostic(robot, command)
        if command.goal_reached:
            self.goal_reached = True
            self.has_path = False
            self.metrics.finish(self.now_s(), True, 'goal_position_and_yaw_reached')
            self.get_logger().info('Pure Pursuit reached the final pose; stop published.')

    def publish(self, command: TrackingCommand) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = float(command.linear_velocity)
        message.twist.angular.z = float(command.angular_velocity)
        self.cmd_pub.publish(message)

    def publish_stop(self) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_2602(args=None) -> None:
    rclpy.init(args=args)
    node = PurePursuitNode('pure_pursuit_pt_2602_hotel')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

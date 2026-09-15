#!/usr/bin/env python3
"""ROS wrapper for the optional curvature-aware Pure Pursuit controller."""

from __future__ import annotations

import math
from typing import List

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32
import tf2_ros
from tf2_ros import TransformException

from .pure_pursuit_node import yaw_from_quaternion
from .tracking_core import AdaptivePurePursuitReference, Pose2D, TrackingCommand
from .tracking_metrics import TrackingCsvLogger
from .tracking_visualization import TrackingVisualization


def path_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class AdaptivePurePursuitNode(Node):
    """Follow a path with adaptive speed/lookahead without changing baseline PP."""

    def __init__(self) -> None:
        super().__init__('adaptive_pure_pursuit')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('control_frequency', 25.0)
        # Canonical adaptive parameters.  The older names remain aliases so a
        # previous launch file can select this node without breaking.
        self.declare_parameter('adaptive_speed_enabled', True)
        self.declare_parameter('min_linear_velocity', float('nan'))
        self.declare_parameter('nominal_linear_velocity', float('nan'))
        self.declare_parameter('max_linear_velocity', 1.00)
        self.declare_parameter('max_angular_velocity', 4.00)
        self.declare_parameter('linear_velocity', 0.90)  # legacy alias
        self.declare_parameter('min_speed', 0.25)  # legacy alias
        self.declare_parameter('max_curvature', 1.6)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('yaw_tolerance', math.radians(12.0))
        self.declare_parameter('goal_yaw_gain', 1.5)
        self.declare_parameter('goal_progress_fraction', 0.95)
        self.declare_parameter('max_progress_jump_distance', 1.5)
        self.declare_parameter('slow_radius', 1.2)
        self.declare_parameter('lookahead_min', float('nan'))
        self.declare_parameter('lookahead_base', float('nan'))
        self.declare_parameter('lookahead_max', float('nan'))
        self.declare_parameter('lookahead_speed_gain', float('nan'))
        self.declare_parameter('minimum_lookahead', 0.30)  # legacy alias
        self.declare_parameter('maximum_lookahead', 1.20)  # legacy alias
        self.declare_parameter('lookahead_distance', 0.60)  # legacy alias
        self.declare_parameter('lookahead_gain', 0.60)  # legacy alias
        self.declare_parameter('lookahead_curvature_gain', 0.45)
        self.declare_parameter('curvature_speed_gain', 1.5)
        self.declare_parameter('curvature_preview_distance', 1.0)
        self.declare_parameter('lateral_error_speed_gain', 1.5)
        self.declare_parameter('lateral_error_slowdown_threshold', 0.15)
        self.declare_parameter('heading_error_speed_gain', 0.45)
        self.declare_parameter('heading_error_slowdown_threshold', math.radians(10.0))
        self.declare_parameter('acceleration_limit', 0.55)
        self.declare_parameter('deceleration_limit', 0.90)
        self.declare_parameter('angular_acceleration_limit', 3.0)
        self.declare_parameter('wheel_radius_m', 0.05)
        self.declare_parameter('wheel_separation_m', 0.44)
        self.declare_parameter('wheel_max_angular_velocity', 20.0)
        self.declare_parameter('odom_topic', '/ekf/odometry')
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('visualization_prefix', '/path_tracking')
        self.declare_parameter('scenario', 'manual')
        self.declare_parameter('trial', 0)
        self.declare_parameter('environment', 'gazebo')
        self.declare_parameter('trace_csv', 'results/tracker_trace.csv')
        self.declare_parameter('results_csv', 'results/tracker_results.csv')
        # Zero means disabled.  The reactive supervisor uses this only in
        # REJOINING; TRACKING keeps the exact nominal adaptive profile.
        self.declare_parameter('external_speed_limit_topic', '/reactive_avoidance/rejoin_speed_limit')

        self.path_topic = str(self.get_parameter('path_topic').value)
        self.command_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.rate_hz = max(1.0, float(self.get_parameter('control_frequency').value))
        self.tf_timeout = max(0.0, float(self.get_parameter('tf_timeout_sec').value))
        self.tracker = AdaptivePurePursuitReference(
            lookahead_min=self._value('lookahead_min', ('minimum_lookahead',), 0.30),
            lookahead_base=self._value('lookahead_base', ('lookahead_distance',), 0.60),
            lookahead_max=self._value('lookahead_max', ('maximum_lookahead',), 1.20),
            lookahead_gain=self._value('lookahead_speed_gain', ('lookahead_gain',), 0.60),
            lookahead_curvature_gain=float(self.get_parameter('lookahead_curvature_gain').value),
            min_speed=self._value('min_linear_velocity', ('min_speed',), 0.25),
            nominal_speed=self._value('nominal_linear_velocity', ('linear_velocity',), 0.90),
            max_speed=float(self.get_parameter('max_linear_velocity').value),
            max_omega=float(self.get_parameter('max_angular_velocity').value),
            max_curvature=float(self.get_parameter('max_curvature').value),
            adaptive_speed_enabled=bool(self.get_parameter('adaptive_speed_enabled').value),
            curvature_speed_gain=float(self.get_parameter('curvature_speed_gain').value),
            curvature_preview_distance=float(
                self.get_parameter('curvature_preview_distance').value),
            lateral_error_speed_gain=float(self.get_parameter('lateral_error_speed_gain').value),
            lateral_error_slowdown_threshold=float(
                self.get_parameter('lateral_error_slowdown_threshold').value),
            heading_error_speed_gain=float(self.get_parameter('heading_error_speed_gain').value),
            heading_error_slowdown_threshold=float(
                self.get_parameter('heading_error_slowdown_threshold').value),
            acceleration_limit=float(self.get_parameter('acceleration_limit').value),
            deceleration_limit=float(self.get_parameter('deceleration_limit').value),
            angular_acceleration_limit=float(
                self.get_parameter('angular_acceleration_limit').value),
            wheel_radius=float(self.get_parameter('wheel_radius_m').value),
            wheel_separation=float(self.get_parameter('wheel_separation_m').value),
            wheel_max_angular_velocity=float(
                self.get_parameter('wheel_max_angular_velocity').value),
            goal_tolerance=float(self.get_parameter('goal_tolerance').value),
            yaw_tolerance=float(self.get_parameter('yaw_tolerance').value),
            goal_yaw_gain=float(self.get_parameter('goal_yaw_gain').value),
            goal_progress_fraction=float(self.get_parameter('goal_progress_fraction').value),
            max_progress_jump_distance=float(
                self.get_parameter('max_progress_jump_distance').value),
            slow_radius=float(self.get_parameter('slow_radius').value),
        )
        self.metrics = TrackingCsvLogger(
            'adaptive_pure_pursuit', str(self.get_parameter('scenario').value),
            int(self.get_parameter('trial').value), str(self.get_parameter('trace_csv').value),
            str(self.get_parameter('results_csv').value),
            environment=str(self.get_parameter('environment').value),
        )
        self.command_publisher = self.create_publisher(TwistStamped, self.command_topic, 10)
        self.path_subscription = self.create_subscription(
            Path, self.path_topic, self.on_path, path_qos())
        self.actual_v: float | None = None
        self.actual_omega: float | None = None
        self.external_speed_limit = 0.0
        self.external_speed_limit_subscription = self.create_subscription(
            Float32, str(self.get_parameter('external_speed_limit_topic').value),
            self.on_external_speed_limit, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.on_odom, 30)
        self._create_diagnostic_publishers()
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.visualization = TrackingVisualization(
            self, str(self.get_parameter('visualization_prefix').value))
        self.path: List[Pose2D] = []
        self.path_frame = ''
        self.has_path = False
        self.last_time: float | None = None
        self.last_log_time = -math.inf
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.get_logger().info(
            'Adaptive Pure Pursuit ready: '
            f'requested(v_nom={self.tracker.requested_nominal_speed:.2f}, '
            f'v_max={self.tracker.requested_max_speed:.2f}, '
            f'w_max={self.tracker.requested_max_omega:.2f}) -> '
            f'effective(v_nom={self.tracker.nominal_speed:.2f}, '
            f'v_max={self.tracker.max_speed:.2f}, '
            f'w_max={self.tracker.max_omega:.2f}); '
            f'preview={self.tracker.preview_distance:.2f} m.')

    def _value(self, canonical: str, aliases: tuple[str, ...], default: float) -> float:
        """Prefer the canonical parameter, then a finite legacy alias."""
        value = float(self.get_parameter(canonical).value)
        if math.isfinite(value):
            return value
        for alias in aliases:
            value = float(self.get_parameter(alias).value)
            if math.isfinite(value):
                return value
        return default

    def _create_diagnostic_publishers(self) -> None:
        prefix = str(self.get_parameter('visualization_prefix').value).rstrip('/')
        prefix = prefix or '/path_tracking'
        self.current_speed_pub = self.create_publisher(Float32, f'{prefix}/current_speed', 10)
        self.target_speed_pub = self.create_publisher(Float32, f'{prefix}/target_speed', 10)
        self.speed_limit_curvature_pub = self.create_publisher(
            Float32, f'{prefix}/speed_limit_curvature', 10)
        self.speed_limit_preview_pub = self.create_publisher(
            Float32, f'{prefix}/speed_limit_preview', 10)
        self.speed_limit_omega_pub = self.create_publisher(
            Float32, f'{prefix}/speed_limit_omega', 10)
        self.speed_limit_lateral_pub = self.create_publisher(
            Float32, f'{prefix}/speed_limit_lateral_error', 10)
        self.lookahead_distance_pub = self.create_publisher(
            Float32, f'{prefix}/lookahead_distance', 10)
        self.curvature_pub = self.create_publisher(Float32, f'{prefix}/curvature', 10)
        self.future_curvature_pub = self.create_publisher(
            Float32, f'{prefix}/future_curvature', 10)
        self.lateral_error_pub = self.create_publisher(Float32, f'{prefix}/lateral_error', 10)
        self.heading_error_pub = self.create_publisher(Float32, f'{prefix}/heading_error', 10)
        self.path_progress_pub = self.create_publisher(Float32, f'{prefix}/path_progress', 10)
        self.closest_index_pub = self.create_publisher(Int32, f'{prefix}/closest_index', 10)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_path(self, message: Path) -> None:
        self.has_path = False
        self.tracker.reset()
        try:
            if not message.header.frame_id or not message.poses:
                raise ValueError('path is empty or has no frame_id')
            decoded = []
            for index, item in enumerate(message.poses):
                if item.header.frame_id and item.header.frame_id != message.header.frame_id:
                    raise ValueError(
                        f'pose {index} has frame {item.header.frame_id!r}, '
                        f'expected {message.header.frame_id!r}')
                pose = item.pose
                values = (pose.position.x, pose.position.y, pose.orientation.x,
                          pose.orientation.y, pose.orientation.z, pose.orientation.w)
                if not all(math.isfinite(value) for value in values):
                    raise ValueError(f'pose {index} is not finite')
                if math.sqrt(sum(value * value for value in values[2:])) < 1e-6:
                    raise ValueError(f'pose {index} has an invalid zero quaternion')
                decoded.append(Pose2D(
                    pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)))
            if len(decoded) < 2:
                raise ValueError('path needs at least two poses')
        except ValueError as error:
            self.path = []
            self.path_frame = ''
            self.publish_stop()
            self.get_logger().error(f'Rejected planned path: {error}')
            return
        self.path = decoded
        self.path_frame = message.header.frame_id
        self.has_path = True
        self.last_time = None
        self.visualization.reset(self.path_frame)
        self.metrics.start(self.now_s(), len(self.path))
        self.get_logger().info(f'Received adaptive path with {len(self.path)} poses.')

    def on_odom(self, message: Odometry) -> None:
        linear = float(message.twist.twist.linear.x)
        angular = float(message.twist.twist.angular.z)
        self.actual_v = linear if math.isfinite(linear) else None
        self.actual_omega = angular if math.isfinite(angular) else None

    def on_external_speed_limit(self, message: Float32) -> None:
        """Accept a temporary positive cap without changing nominal settings."""
        value = float(message.data)
        self.external_speed_limit = value if math.isfinite(value) and value > 0.0 else 0.0

    def robot_pose(self) -> Pose2D:
        transform = self.tf_buffer.lookup_transform(
            self.path_frame, self.base_frame, rclpy.time.Time(),
            timeout=Duration(seconds=self.tf_timeout))
        translation = transform.transform.translation
        return Pose2D(translation.x, translation.y, _yaw(transform.transform.rotation))

    def on_timer(self) -> None:
        if not self.has_path:
            self.publish_stop()
            return
        now = self.now_s()
        dt = 1.0 / self.rate_hz if self.last_time is None else max(
            1e-3, min(0.25, now - self.last_time))
        self.last_time = now
        try:
            robot = self.robot_pose()
            command = self.tracker.command(self.path, robot, dt)
            command = self.apply_external_speed_limit(command)
        except (TransformException, ValueError, FloatingPointError) as error:
            self.publish_stop()
            if now - self.last_log_time >= 1.0:
                self.last_log_time = now
                self.get_logger().warning(f'Adaptive Pure Pursuit stopped: {error}')
            return
        self.publish(command)
        stamp = self.get_clock().now().to_msg()
        self.visualization.publish(self.path_frame, stamp, robot, self.path, command)
        self.metrics.sample(now, robot, command)
        self.publish_diagnostics(command)
        if now - self.last_log_time >= 1.0:
            self.last_log_time = now
            progress = self._path_progress(command.closest_index)
            current_v = command.linear_velocity if self.actual_v is None else self.actual_v
            self.get_logger().info(
                f'[AdaptivePP] progress={progress:.3f} '
                f'idx={command.closest_index}/{len(self.path) - 1} '
                f'v_real={current_v:.2f} v_target={self.tracker.last_target_speed:.2f} '
                f'v_lim(k={self.tracker.last_speed_limit_curvature:.2f}, '
                f'preview={self.tracker.last_speed_limit_preview:.2f}, '
                f'omega={self.tracker.last_speed_limit_omega:.2f}, '
                f'cte={self.tracker.last_speed_limit_lateral_error:.2f}) '
                f'Ld={command.lookahead_distance:.2f} '
                f'k={self.tracker.last_curvature:.2f} '
                f'k_preview={self.tracker.last_future_curvature:.2f} '
                f'cte={command.cross_track_error:.2f} '
                f'omega_cmd={command.angular_velocity:.2f}.')
        if command.goal_reached:
            self.has_path = False
            self.metrics.finish(now, True, 'goal_position_and_yaw_reached')
            self.get_logger().info('Adaptive Pure Pursuit reached the final pose.')

    def publish(self, command: TrackingCommand) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = float(command.linear_velocity)
        message.twist.angular.z = float(command.angular_velocity)
        self.command_publisher.publish(message)

    def apply_external_speed_limit(self, command: TrackingCommand) -> TrackingCommand:
        """Cap rejoin speed while preserving commanded curvature and safety."""
        if self.external_speed_limit <= 0.0 or command.linear_velocity <= self.external_speed_limit:
            return command
        ratio = self.external_speed_limit / max(command.linear_velocity, 1e-9)
        command.linear_velocity = self.external_speed_limit
        command.angular_velocity *= ratio
        return command

    def _path_progress(self, closest_index: int) -> float:
        return self.tracker.progress_fraction(closest_index)

    def publish_diagnostics(self, command: TrackingCommand) -> None:
        """Publish controller-native diagnostics; values are rosbag-ready."""
        current_v = command.linear_velocity if self.actual_v is None else self.actual_v
        self.current_speed_pub.publish(Float32(data=float(current_v)))
        self.target_speed_pub.publish(Float32(data=float(self.tracker.last_target_speed)))
        self.speed_limit_curvature_pub.publish(
            Float32(data=float(self.tracker.last_speed_limit_curvature)))
        self.speed_limit_preview_pub.publish(
            Float32(data=float(self.tracker.last_speed_limit_preview)))
        self.speed_limit_omega_pub.publish(
            Float32(data=float(self.tracker.last_speed_limit_omega)))
        self.speed_limit_lateral_pub.publish(
            Float32(data=float(self.tracker.last_speed_limit_lateral_error)))
        self.lookahead_distance_pub.publish(Float32(data=float(command.lookahead_distance)))
        self.curvature_pub.publish(Float32(data=float(self.tracker.last_curvature)))
        self.future_curvature_pub.publish(Float32(data=float(self.tracker.last_future_curvature)))
        self.lateral_error_pub.publish(Float32(data=float(command.cross_track_error)))
        self.heading_error_pub.publish(Float32(data=float(command.heading_error)))
        self.path_progress_pub.publish(
            Float32(data=float(self._path_progress(command.closest_index))))
        self.closest_index_pub.publish(Int32(data=int(command.closest_index)))

    def publish_stop(self) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        self.command_publisher.publish(message)


def _yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AdaptivePurePursuitNode()
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

#!/usr/bin/env python3
"""
Non-invasive instrumentation for repeatable path-tracking experiments.

The monitor is intentionally separate from both Pure Pursuit implementations.
It observes the planned path, TF/odometry, command chain, and AEB diagnostics;
therefore a baseline is measured with exactly the original controller logic.
"""

from __future__ import annotations

import csv
import hashlib
import math
from pathlib import Path as FilePath
import struct
from typing import Any

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32, String
import tf2_ros
from tf2_ros import TransformException
from visualization_msgs.msg import Marker
import yaml

from .tracking_core import clamp, wrap_to_pi


TRACE_COLUMNS = [
    'time_s', 'x_m', 'y_m', 'yaw_rad', 'path_s_m', 'progress_percent', 'lap',
    'closest_index', 'target_index',
    'lookahead_x_m', 'lookahead_y_m',
    'lateral_error_m', 'heading_error_rad', 'curvature_1_m',
    'future_curvature_1_m', 'actual_v_m_s', 'actual_omega_rad_s',
    'target_v_m_s', 'target_omega_rad_s', 'mux_v_m_s', 'mux_omega_rad_s',
    'output_v_m_s', 'output_omega_rad_s', 'lookahead_m', 'aeb_active',
    'aeb_ttc_s', 'aeb_critical_distance_m', 'aeb_command_blocked',
]


def _yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _stamp_to_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _finite(value: float | None) -> float | None:
    return float(value) if value is not None and math.isfinite(value) else None


def _pose_snapshot(message: PoseWithCovarianceStamped | None) -> dict[str, float | str] | None:
    """Serialize AMCL / initialpose evidence without changing either source."""
    if message is None:
        return None
    pose = message.pose.pose
    return {
        'frame_id': message.header.frame_id,
        'timestamp_s': _stamp_to_s(message.header.stamp),
        'x_m': float(pose.position.x),
        'y_m': float(pose.position.y),
        'yaw_rad': _yaw(pose.orientation),
        'covariance_x_m2': float(message.pose.covariance[0]),
        'covariance_y_m2': float(message.pose.covariance[7]),
        'covariance_yaw_rad2': float(message.pose.covariance[35]),
    }


def _odom_snapshot(message: Odometry | None) -> dict[str, float | str] | None:
    if message is None:
        return None
    pose = message.pose.pose
    twist = message.twist.twist
    return {
        'frame_id': message.header.frame_id,
        'child_frame_id': message.child_frame_id,
        'timestamp_s': _stamp_to_s(message.header.stamp),
        'x_m': float(pose.position.x),
        'y_m': float(pose.position.y),
        'yaw_rad': _yaw(pose.orientation),
        'linear_x_m_s': float(twist.linear.x),
        'angular_z_rad_s': float(twist.angular.z),
    }


def _transform_snapshot(transform) -> dict[str, float | str]:
    translation = transform.transform.translation
    return {
        'parent_frame': transform.header.frame_id,
        'child_frame': transform.child_frame_id,
        'timestamp_s': _stamp_to_s(transform.header.stamp),
        'x_m': float(translation.x),
        'y_m': float(translation.y),
        'z_m': float(translation.z),
        'yaw_rad': _yaw(transform.transform.rotation),
    }


def _safe_mean(values: list[float]) -> float | None:
    return None if not values else sum(values) / len(values)


def _safe_rmse(values: list[float]) -> float | None:
    return None if not values else math.sqrt(sum(value * value for value in values) / len(values))


def assess_stuck_window(
        rows: list[dict[str, float | int | bool | None]],
        minimum_progress_m: float,
        minimum_distance_m: float,
        minimum_index_delta: int,
        maximum_linear_speed_m_s: float,
        recovery_angular_speed_rad_s: float,
        command_speed_threshold_m_s: float,
) -> tuple[bool, dict[str, float | int | bool]]:
    """Classify physical immobility from a completed measurement window.

    AEB state is intentionally absent from this decision.  An intervention can
    preserve angular motion or release after a slow recovery, neither of which
    is a stuck robot.  The caller is responsible for selecting rows spanning
    the configurable temporal window.
    """
    if len(rows) < 2:
        return False, {'window_ready': False}
    first, last = rows[0], rows[-1]
    travelled = 0.0
    for previous, current in zip(rows, rows[1:]):
        travelled += math.hypot(
            float(current['x_m']) - float(previous['x_m']),
            float(current['y_m']) - float(previous['y_m']),
        )
    progress_delta = float(last['path_s_m']) - float(first['path_s_m'])
    index_delta = int(last['closest_index']) - int(first['closest_index'])
    max_actual_linear = max(abs(float(row['actual_v_m_s'])) for row in rows)
    max_actual_angular = max(abs(float(row['actual_omega_rad_s'])) for row in rows)
    max_requested_linear = max(
        max(abs(float(row['target_v_m_s'])), abs(float(row['mux_v_m_s']))) for row in rows)
    controller_attempting = max_requested_linear >= command_speed_threshold_m_s
    no_progress = progress_delta < minimum_progress_m
    no_displacement = travelled < minimum_distance_m
    no_index_progress = index_delta < minimum_index_delta
    no_linear_motion = max_actual_linear < maximum_linear_speed_m_s
    not_reorienting = max_actual_angular < recovery_angular_speed_rad_s
    stuck = bool(controller_attempting and no_progress and no_displacement and
                 no_index_progress and no_linear_motion and not_reorienting)
    return stuck, {
        'window_ready': True,
        'window_duration_s': float(last['time_s']) - float(first['time_s']),
        'progress_delta_m': progress_delta,
        'distance_travelled_m': travelled,
        'closest_index_delta': index_delta,
        'maximum_actual_linear_speed_m_s': max_actual_linear,
        'maximum_actual_angular_speed_rad_s': max_actual_angular,
        'maximum_requested_linear_speed_m_s': max_requested_linear,
        'controller_attempting': controller_attempting,
        'no_progress': no_progress,
        'no_displacement': no_displacement,
        'no_index_progress': no_index_progress,
        'no_linear_motion': no_linear_motion,
        'not_reorienting': not_reorienting,
    }


def path_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class PathProfile:
    """Arc-length and curvature representation used only for measurement."""

    def __init__(self, message: Path) -> None:
        if len(message.poses) < 2:
            raise ValueError('planned path needs at least two poses')
        self.frame_id = message.header.frame_id
        if not self.frame_id:
            raise ValueError('planned path has no frame_id')
        self.xy = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        self.yaw = [_yaw(pose.pose.orientation) for pose in message.poses]
        if not all(math.isfinite(x) and math.isfinite(y) for x, y in self.xy):
            raise ValueError('planned path contains non-finite coordinates')
        self.s = [0.0]
        for (x0, y0), (x1, y1) in zip(self.xy, self.xy[1:]):
            self.s.append(self.s[-1] + math.hypot(x1 - x0, y1 - y0))
        self.length = self.s[-1]
        if self.length <= 1e-6:
            raise ValueError('planned path has zero length')
        self.curvature = self._curvature()
        digest = hashlib.sha256()
        digest.update(self.frame_id.encode('utf-8'))
        for (x, y), yaw in zip(self.xy, self.yaw):
            digest.update(struct.pack('<ddd', float(x), float(y), float(yaw)))
        self.sha256 = digest.hexdigest()

    def _curvature(self) -> list[float]:
        values: list[float] = []
        for index in range(len(self.xy)):
            left = max(0, index - 1)
            right = min(len(self.xy) - 1, index + 1)
            ds = self.s[right] - self.s[left]
            values.append(0.0 if ds <= 1e-6 else wrap_to_pi(
                self.yaw[right] - self.yaw[left]) / ds)
        return values

    def point_at(self, distance: float) -> tuple[float, float]:
        distance = clamp(distance, 0.0, self.length)
        for index in range(1, len(self.s)):
            if self.s[index] >= distance:
                segment = max(1e-9, self.s[index] - self.s[index - 1])
                ratio = (distance - self.s[index - 1]) / segment
                x0, y0 = self.xy[index - 1]
                x1, y1 = self.xy[index]
                return x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0)
        return self.xy[-1]

    def _index_at(self, distance: float) -> int:
        distance = clamp(distance, 0.0, self.length)
        low, high = 0, len(self.s) - 1
        while low < high:
            middle = (low + high) // 2
            if self.s[middle] < distance:
                low = middle + 1
            else:
                high = middle
        return low

    def index_at(self, distance: float) -> int:
        """Return the planned-path index associated with an arc length."""
        return self._index_at(distance)

    def curvature_at(self, distance: float) -> float:
        return self.curvature[self._index_at(distance)]

    def future_max_curvature(self, distance: float, preview_m: float) -> float:
        begin = self._index_at(distance)
        end = self._index_at(min(self.length, distance + max(0.0, preview_m)))
        return max((abs(value) for value in self.curvature[begin:end + 1]), default=0.0)

    def project(
            self, x: float, y: float, min_distance: float,
            backward_m: float, forward_m: float,
    ) -> tuple[float, float, float]:
        """
        Project a pose near monotonic progress onto the polyline.

        Restricting the search window prevents an overlapping second lap from
        being mistaken for a future point merely because it is geometrically
        close.  This is measurement-only and cannot affect tracking.
        """
        lower = max(0.0, min_distance - max(0.0, backward_m))
        upper = min(self.length, min_distance + max(0.0, forward_m))
        first = max(0, self._index_at(lower) - 1)
        last = min(len(self.xy) - 2, self._index_at(upper))
        best: tuple[float, float, float, float] | None = None
        for index in range(first, last + 1):
            x0, y0 = self.xy[index]
            x1, y1 = self.xy[index + 1]
            dx, dy = x1 - x0, y1 - y0
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-12:
                continue
            ratio = clamp(((x - x0) * dx + (y - y0) * dy) / length_sq, 0.0, 1.0)
            px, py = x0 + ratio * dx, y0 + ratio * dy
            error_x, error_y = x - px, y - py
            norm = math.hypot(error_x, error_y)
            signed = math.copysign(norm, dx * error_y - dy * error_x)
            s_value = self.s[index] + ratio * math.sqrt(length_sq)
            candidate = (norm, s_value, signed, math.atan2(dy, dx))
            if best is None or candidate[0] < best[0]:
                best = candidate
        if best is None:
            raise ValueError('cannot project onto planned path')
        _, progress, lateral_error, tangent = best
        return progress, lateral_error, tangent

    def as_yaml(self) -> dict[str, Any]:
        return {
            'frame_id': self.frame_id,
            'points': len(self.xy),
            'length_m': self.length,
            'curvature_abs_mean_1_m': _safe_mean([abs(value) for value in self.curvature]),
            'curvature_abs_max_1_m': max(abs(value) for value in self.curvature),
            'sha256': self.sha256,
            'first_20_poses': [
                {
                    'index': index,
                    'x_m': x,
                    'y_m': y,
                    'yaw_rad': yaw,
                    'curvature_1_m': self.curvature[index],
                }
                for index, ((x, y), yaw) in enumerate(zip(self.xy[:20], self.yaw[:20]))
            ],
        }

    def write_csv(self, destination: FilePath) -> None:
        """Write the geometric profile used to interpret a measured run."""
        with destination.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(
                stream, fieldnames=['index', 's_m', 'x_m', 'y_m', 'heading_rad', 'curvature_1_m'])
            writer.writeheader()
            for index, ((x, y), heading, distance, curvature) in enumerate(
                    zip(self.xy, self.yaw, self.s, self.curvature)):
                writer.writerow({
                    'index': index, 's_m': distance, 'x_m': x, 'y_m': y,
                    'heading_rad': heading, 'curvature_1_m': curvature,
                })


class OptimizationMonitor(Node):
    """Observe a single two-lap attempt and persist a complete result file."""

    def __init__(self) -> None:
        super().__init__('optimization_monitor')
        self.declare_parameter('run_id', 0)
        self.declare_parameter('controller', 'pure_pursuit_standard')
        self.declare_parameter('results_file', '')
        self.declare_parameter('trace_file', '')
        self.declare_parameter('initial_state_file', '')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('odom_topic', '/ekf/odometry')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('cmd_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_mux_topic', '/cmd_vel_mux')
        self.declare_parameter('cmd_output_topic', '/diffdrive_controller/cmd_vel')
        self.declare_parameter('lookahead_marker_topic', '/path_tracking/lookahead_point')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('sample_frequency', 25.0)
        self.declare_parameter('expected_laps', 2)
        self.declare_parameter('start_speed_threshold', 0.04)
        self.declare_parameter('stop_speed_threshold', 0.025)
        self.declare_parameter('finish_radius_m', 0.60)
        self.declare_parameter('lap_progress_fraction', 0.95)
        self.declare_parameter('completion_hold_s', 0.50)
        self.declare_parameter('maximum_duration_s', 900.0)
        self.declare_parameter('progress_stall_timeout_s', 20.0)
        self.declare_parameter('critical_lateral_error_m', 1.00)
        self.declare_parameter('critical_error_duration_s', 1.5)
        # Kept only so an older experiment YAML can still be read.  AEB state
        # is a safety metric, never a terminal condition by itself.
        self.declare_parameter('aeb_abort_duration_s', 0.0)
        self.declare_parameter('stuck_window_s', 45.0)
        self.declare_parameter('stuck_min_progress_m', 0.12)
        self.declare_parameter('stuck_min_distance_m', 0.12)
        self.declare_parameter('stuck_min_closest_index_delta', 3)
        self.declare_parameter('stuck_max_actual_linear_speed_m_s', 0.025)
        self.declare_parameter('stuck_recovery_angular_speed_rad_s', 0.08)
        self.declare_parameter('stuck_command_speed_threshold_m_s', 0.04)
        self.declare_parameter('projection_backward_m', 0.50)
        self.declare_parameter('projection_forward_m', 1.50)
        self.declare_parameter('curvature_preview_distance', 1.0)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('wheel_radius_m', 0.05)
        self.declare_parameter('wheel_separation_m', 0.44)
        self.declare_parameter('wheel_max_angular_velocity', 20.0)

        self.run_id = int(self.get_parameter('run_id').value)
        self.controller = str(self.get_parameter('controller').value)
        results_file_value = str(self.get_parameter('results_file').value)
        trace_file_value = str(self.get_parameter('trace_file').value)
        if not results_file_value:
            raise ValueError('results_file parameter is required')
        self.results_file = FilePath(results_file_value)
        self.trace_file = FilePath(trace_file_value) if trace_file_value else \
            self.results_file.with_name('monitor_trace.csv')
        initial_state_value = str(self.get_parameter('initial_state_file').value)
        self.initial_state_file = FilePath(initial_state_value) if initial_state_value else \
            self.results_file.with_name('initial_state.yaml')
        self.expected_laps = max(1, int(self.get_parameter('expected_laps').value))
        self.start_speed = max(0.0, float(self.get_parameter('start_speed_threshold').value))
        self.stop_speed = max(0.0, float(self.get_parameter('stop_speed_threshold').value))
        self.finish_radius = max(0.05, float(self.get_parameter('finish_radius_m').value))
        self.lap_fraction = clamp(float(self.get_parameter('lap_progress_fraction').value), 0.5, 1.0)
        self.preview_m = max(0.0, float(self.get_parameter('curvature_preview_distance').value))
        self.max_omega = max(0.0, float(self.get_parameter('max_angular_velocity').value))
        self.results_file.parent.mkdir(parents=True, exist_ok=True)
        self.trace_file.parent.mkdir(parents=True, exist_ok=True)

        self.profile: PathProfile | None = None
        self.last_s = 0.0
        self.path_received = False
        self.started_at: float | None = None
        self.last_sample_at: float | None = None
        self.last_progress_at: float | None = None
        self.last_pose: tuple[float, float] | None = None
        self.laps_completed = 0
        self.completed = False
        self.failure_reason: str | None = None
        self.finish_candidate_at: float | None = None
        self.critical_error_at: float | None = None
        self.aeb_active_since: float | None = None
        self.was_moving = False
        self.stopped_since: float | None = None
        self.stop_count = 0
        self.aeb_previous = False
        self.aeb_events = 0
        self.aeb_active_time_s = 0.0
        self.aeb_command_blocked_time_s = 0.0
        self.distance_travelled_m = 0.0
        self.stuck_evidence: dict[str, float | int | bool] | None = None
        self.samples: list[dict[str, float | int | bool | None]] = []
        self.status_pub = self.create_publisher(String, '/path_tracking/mission_status', 10)
        self.current_speed_pub = self.create_publisher(Float32, '/path_tracking/current_speed', 10)
        self.target_speed_pub = self.create_publisher(Float32, '/path_tracking/target_speed', 10)
        self.lookahead_pub = self.create_publisher(Float32, '/path_tracking/lookahead_distance', 10)
        self.curvature_pub = self.create_publisher(Float32, '/path_tracking/curvature', 10)
        self.future_curvature_pub = self.create_publisher(Float32, '/path_tracking/future_curvature', 10)
        self.lateral_error_pub = self.create_publisher(Float32, '/path_tracking/lateral_error', 10)
        self.heading_error_pub = self.create_publisher(Float32, '/path_tracking/heading_error', 10)
        self.progress_pub = self.create_publisher(Float32, '/path_tracking/path_progress', 10)
        self.lap_pub = self.create_publisher(Int32, '/path_tracking/lap', 10)
        self.closest_index_pub = self.create_publisher(Int32, '/path_tracking/closest_index', 10)
        self.target_index_pub = self.create_publisher(Int32, '/path_tracking/target_index', 10)

        self.nav_command: TwistStamped | None = None
        self.mux_command: TwistStamped | None = None
        self.output_command: TwistStamped | None = None
        self.odom: Odometry | None = None
        self.lookahead_marker: Marker | None = None
        self.aeb_active = False
        self.aeb_blocked = False
        self.aeb_ttc = math.inf
        self.aeb_distance = math.inf
        self.amcl_pose: PoseWithCovarianceStamped | None = None
        self.initial_pose_message: PoseWithCovarianceStamped | None = None
        self.initial_state_written = False

        self.create_subscription(Path, str(self.get_parameter('path_topic').value), self._on_path, path_qos())
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 30)
        self.create_subscription(PoseWithCovarianceStamped,
                                 str(self.get_parameter('amcl_pose_topic').value),
                                 self._on_amcl_pose, 30)
        self.create_subscription(PoseWithCovarianceStamped,
                                 str(self.get_parameter('initial_pose_topic').value),
                                 self._on_initial_pose, 30)
        self.create_subscription(TwistStamped, str(self.get_parameter('cmd_nav_topic').value), self._on_nav, 30)
        self.create_subscription(TwistStamped, str(self.get_parameter('cmd_mux_topic').value), self._on_mux, 30)
        self.create_subscription(TwistStamped, str(self.get_parameter('cmd_output_topic').value), self._on_output, 30)
        self.create_subscription(Marker, str(self.get_parameter('lookahead_marker_topic').value), self._on_lookahead, path_qos())
        self.create_subscription(Bool, '/aeb/active', lambda msg: setattr(self, 'aeb_active', bool(msg.data)), 30)
        self.create_subscription(Bool, '/aeb/command_blocked', lambda msg: setattr(self, 'aeb_blocked', bool(msg.data)), 30)
        self.create_subscription(Float32, '/aeb/ttc', lambda msg: setattr(self, 'aeb_ttc', float(msg.data)), 30)
        self.create_subscription(Float32, '/aeb/critical_distance', lambda msg: setattr(self, 'aeb_distance', float(msg.data)), 30)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        frequency = max(1.0, float(self.get_parameter('sample_frequency').value))
        self.timer = self.create_timer(1.0 / frequency, self._on_timer)
        self._publish_status('waiting_for_path')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_status(self, value: str) -> None:
        self.status_pub.publish(String(data=value))

    @staticmethod
    def _twist(message: TwistStamped | None) -> tuple[float, float]:
        if message is None:
            return 0.0, 0.0
        return float(message.twist.linear.x), float(message.twist.angular.z)

    def _on_path(self, message: Path) -> None:
        if self.started_at is not None:
            return
        try:
            self.profile = PathProfile(message)
        except ValueError as error:
            self.get_logger().error(f'Rejected path for optimization monitor: {error}')
            return
        self.path_received = True
        self.last_s = 0.0
        self.profile.write_csv(self.results_file.with_name('path_profile.csv'))
        self._publish_status('waiting_for_motion')
        self.get_logger().info(
            f'Monitor received path: {len(message.poses)} points, {self.profile.length:.3f} m.')

    def _on_odom(self, message: Odometry) -> None:
        self.odom = message

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        self.amcl_pose = message

    def _on_initial_pose(self, message: PoseWithCovarianceStamped) -> None:
        self.initial_pose_message = message

    def _on_nav(self, message: TwistStamped) -> None:
        self.nav_command = message

    def _on_mux(self, message: TwistStamped) -> None:
        self.mux_command = message

    def _on_output(self, message: TwistStamped) -> None:
        self.output_command = message

    def _on_lookahead(self, message: Marker) -> None:
        if message.action == Marker.ADD:
            self.lookahead_marker = message

    def _target_index(self, progress_s: float, closest_index: int) -> int:
        """Map the tracker marker to a planned-path index for diagnostics."""
        if self.profile is None or self.lookahead_marker is None:
            return closest_index
        marker = self.lookahead_marker.pose.position
        try:
            target_s, _, _ = self.profile.project(
                marker.x, marker.y, progress_s, backward_m=0.50, forward_m=5.0)
        except ValueError:
            return closest_index
        return self.profile.index_at(target_s)

    def _pose(self) -> tuple[float, float, float] | None:
        if self.profile is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.profile.frame_id,
                str(self.get_parameter('base_frame').value),
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        return translation.x, translation.y, _yaw(transform.transform.rotation)

    def _lookup_transform_snapshot(self, parent: str, child: str) -> dict[str, float | str] | None:
        """Capture a TF sample for comparison; failure is recorded as missing."""
        try:
            transform = self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except TransformException:
            return None
        return _transform_snapshot(transform)

    def _write_initial_state(self, event: str) -> None:
        """Persist the latest pre-mission localization evidence exactly once.

        The monitor is started before the automatic initialpose publication and
        before manual navigation.  It never publishes any of these messages;
        this file simply records what the active stack believed when motion
        began.  A later readiness snapshot provides the equivalent state just
        before the automatic navigation launch.
        """
        if self.initial_state_written:
            return
        self.initial_state_written = True
        profile_frame = None if self.profile is None else self.profile.frame_id
        content = {
            'capture_event': event,
            'capture_time_s': self._now(),
            'gazebo_ground_truth': {
                'status': 'not_available_through_current_ros_bridge',
                'note': 'The runner records a Gazebo service query separately when available.',
            },
            'ekf_odometry': _odom_snapshot(self.odom),
            'amcl_pose': _pose_snapshot(self.amcl_pose),
            'initialpose_observed': _pose_snapshot(self.initial_pose_message),
            'transforms': {
                'map->odom': self._lookup_transform_snapshot('map', 'odom'),
                'odom->base_link': self._lookup_transform_snapshot('odom', 'base_link'),
                'map->base_link': self._lookup_transform_snapshot('map', 'base_link'),
            },
            'path_frame': profile_frame,
        }
        with self.initial_state_file.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(content, stream, sort_keys=False)

    def _begin(self, now: float) -> None:
        self.started_at = now
        self.last_sample_at = now
        self.last_progress_at = now
        self._write_initial_state('mission_motion_detected')
        self._publish_status('running')
        self.get_logger().info('Optimization monitor detected mission start.')

    def _lap_from_progress(self, progress_s: float) -> int:
        if self.profile is None:
            return 0
        ratio = clamp(progress_s / self.profile.length, 0.0, 1.0)
        return min(self.expected_laps, int(ratio * self.expected_laps) + 1)

    def _check_laps(self, now: float, x: float, y: float, progress_s: float) -> None:
        if self.profile is None:
            return
        while self.laps_completed < self.expected_laps:
            next_lap = self.laps_completed + 1
            seam_s = self.profile.length * next_lap / self.expected_laps
            seam_x, seam_y = self.profile.point_at(seam_s)
            near_seam = math.hypot(x - seam_x, y - seam_y) <= self.finish_radius
            past_threshold = progress_s >= seam_s * self.lap_fraction
            if not (past_threshold and near_seam):
                break
            self.laps_completed = next_lap
            self.get_logger().info(
                f'Lap {self.laps_completed}/{self.expected_laps} detected at progress '
                f'{100.0 * progress_s / self.profile.length:.1f}%.')
            self._publish_status(f'lap_{self.laps_completed}_complete')

    def _is_stuck(self, now: float) -> tuple[bool, dict[str, float | int | bool]]:
        """Evaluate the configurable no-progress window without using AEB state."""
        window_s = max(1.0, float(self.get_parameter('stuck_window_s').value))
        rows = [row for row in self.samples if now - float(row['time_s']) <= window_s]
        if len(rows) < 2 or float(rows[-1]['time_s']) - float(rows[0]['time_s']) < 0.95 * window_s:
            return False, {'window_ready': False}
        return assess_stuck_window(
            rows,
            minimum_progress_m=max(0.0, float(self.get_parameter('stuck_min_progress_m').value)),
            minimum_distance_m=max(0.0, float(self.get_parameter('stuck_min_distance_m').value)),
            minimum_index_delta=max(0, int(self.get_parameter('stuck_min_closest_index_delta').value)),
            maximum_linear_speed_m_s=max(
                0.0, float(self.get_parameter('stuck_max_actual_linear_speed_m_s').value)),
            recovery_angular_speed_rad_s=max(
                0.0, float(self.get_parameter('stuck_recovery_angular_speed_rad_s').value)),
            command_speed_threshold_m_s=max(
                0.0, float(self.get_parameter('stuck_command_speed_threshold_m_s').value)),
        )

    def _check_terminal_conditions(
            self, now: float, x: float, y: float, lateral_error: float,
            progress_s: float, target_v: float,
    ) -> None:
        if self.profile is None or self.started_at is None:
            return
        elapsed = now - self.started_at
        if elapsed > float(self.get_parameter('maximum_duration_s').value):
            self._finish(now, False, 'maximum_duration_exceeded')
            return
        critical_error = abs(lateral_error) > float(self.get_parameter('critical_lateral_error_m').value)
        if critical_error:
            self.critical_error_at = self.critical_error_at or now
            if now - self.critical_error_at >= float(self.get_parameter('critical_error_duration_s').value):
                self._finish(now, False, 'critical_lateral_error')
                return
        else:
            self.critical_error_at = None
        stuck, evidence = self._is_stuck(now)
        if stuck:
            self.stuck_evidence = evidence
            self._finish(now, False, 'robot_stuck_no_progress')
            return
        final_x, final_y = self.profile.xy[-1]
        near_final = math.hypot(x - final_x, y - final_y) <= self.finish_radius
        final_progress = progress_s >= self.profile.length * self.lap_fraction
        if self.laps_completed >= self.expected_laps and near_final and final_progress:
            self.finish_candidate_at = self.finish_candidate_at or now
            if now - self.finish_candidate_at >= float(self.get_parameter('completion_hold_s').value):
                self._finish(now, True, 'two_laps_complete')
        else:
            self.finish_candidate_at = None

    def _sample(self, now: float, x: float, y: float, yaw: float) -> None:
        if self.profile is None:
            return
        progress_s, lateral_error, tangent = self.profile.project(
            x, y, self.last_s,
            float(self.get_parameter('projection_backward_m').value),
            float(self.get_parameter('projection_forward_m').value),
        )
        progress_s = max(self.last_s, progress_s)
        if progress_s > self.last_s + 0.005:
            self.last_progress_at = now
        self.last_s = progress_s
        closest_index = self.profile.index_at(progress_s)
        target_index = self._target_index(progress_s, closest_index)
        heading_error = wrap_to_pi(yaw - tangent)
        curvature = self.profile.curvature_at(progress_s)
        future_curvature = self.profile.future_max_curvature(progress_s, self.preview_m)
        nav_v, nav_w = self._twist(self.nav_command)
        mux_v, mux_w = self._twist(self.mux_command)
        output_v, output_w = self._twist(self.output_command)
        actual_v = 0.0 if self.odom is None else float(self.odom.twist.twist.linear.x)
        actual_w = 0.0 if self.odom is None else float(self.odom.twist.twist.angular.z)
        lookahead_x = None if self.lookahead_marker is None else float(self.lookahead_marker.pose.position.x)
        lookahead_y = None if self.lookahead_marker is None else float(self.lookahead_marker.pose.position.y)
        lookahead = None if self.lookahead_marker is None else math.hypot(
            lookahead_x - x,
            lookahead_y - y,
        )
        if self.last_pose is not None:
            self.distance_travelled_m += math.hypot(x - self.last_pose[0], y - self.last_pose[1])
        self.last_pose = (x, y)
        last_time = self.last_sample_at if self.last_sample_at is not None else now
        dt = max(0.0, now - last_time)
        self.last_sample_at = now
        if self.aeb_active:
            self.aeb_active_time_s += dt
        if self.aeb_blocked:
            self.aeb_command_blocked_time_s += dt
        if self.aeb_active and not self.aeb_previous:
            self.aeb_events += 1
        self.aeb_previous = self.aeb_active
        if abs(actual_v) >= self.start_speed:
            self.was_moving = True
            self.stopped_since = None
        elif self.was_moving:
            self.stopped_since = self.stopped_since or now
            if now - self.stopped_since >= 0.50:
                self.stop_count += 1
                self.was_moving = False
        self._check_laps(now, x, y, progress_s)
        lap = self._lap_from_progress(progress_s)
        self.samples.append({
            'time_s': now, 'x_m': x, 'y_m': y, 'yaw_rad': yaw,
            'path_s_m': progress_s, 'progress_percent': 100.0 * progress_s / self.profile.length,
            'lap': lap, 'closest_index': closest_index, 'target_index': target_index,
            'lookahead_x_m': lookahead_x, 'lookahead_y_m': lookahead_y,
            'lateral_error_m': lateral_error, 'heading_error_rad': heading_error,
            'curvature_1_m': curvature, 'future_curvature_1_m': future_curvature,
            'actual_v_m_s': actual_v, 'actual_omega_rad_s': actual_w,
            'target_v_m_s': nav_v, 'target_omega_rad_s': nav_w,
            'mux_v_m_s': mux_v, 'mux_omega_rad_s': mux_w,
            'output_v_m_s': output_v, 'output_omega_rad_s': output_w,
            'lookahead_m': lookahead, 'aeb_active': self.aeb_active,
            'aeb_ttc_s': _finite(self.aeb_ttc), 'aeb_critical_distance_m': _finite(self.aeb_distance),
            'aeb_command_blocked': self.aeb_blocked,
        })
        self.current_speed_pub.publish(Float32(data=float(actual_v)))
        self.target_speed_pub.publish(Float32(data=float(nav_v)))
        self.lookahead_pub.publish(Float32(data=float(lookahead or 0.0)))
        self.curvature_pub.publish(Float32(data=float(curvature)))
        self.future_curvature_pub.publish(Float32(data=float(future_curvature)))
        self.lateral_error_pub.publish(Float32(data=float(lateral_error)))
        self.heading_error_pub.publish(Float32(data=float(heading_error)))
        self.progress_pub.publish(Float32(data=float(progress_s / self.profile.length)))
        self.lap_pub.publish(Int32(data=int(lap)))
        self.closest_index_pub.publish(Int32(data=int(closest_index)))
        self.target_index_pub.publish(Int32(data=int(target_index)))
        self._check_terminal_conditions(now, x, y, lateral_error, progress_s, nav_v)

    def _on_timer(self) -> None:
        if self.completed:
            return
        pose = self._pose()
        if pose is None or not self.path_received:
            return
        now = self._now()
        nav_v, _ = self._twist(self.nav_command)
        actual_v = 0.0 if self.odom is None else float(self.odom.twist.twist.linear.x)
        if self.started_at is None:
            if max(abs(nav_v), abs(actual_v)) < self.start_speed:
                return
            self._begin(now)
        self._sample(now, *pose)

    def _lap_summary(self, lap: int) -> dict[str, float | int | None]:
        data = [row for row in self.samples if int(row['lap']) == lap]
        if not data:
            return {'samples': 0, 'time_s': None, 'average_velocity_m_s': None,
                    'maximum_velocity_m_s': None, 'lateral_rmse_m': None,
                    'lateral_max_error_m': None}
        first, last = float(data[0]['time_s']), float(data[-1]['time_s'])
        errors = [float(row['lateral_error_m']) for row in data]
        velocities = [abs(float(row['actual_v_m_s'])) for row in data]
        return {
            'samples': len(data), 'time_s': max(0.0, last - first),
            'average_velocity_m_s': _safe_mean(velocities),
            'maximum_velocity_m_s': max(velocities),
            'lateral_rmse_m': _safe_rmse(errors),
            'lateral_max_error_m': max(abs(value) for value in errors),
        }

    def _section_summary(self) -> dict[str, dict[str, float | int | None]]:
        output: dict[str, dict[str, float | int | None]] = {}
        if self.profile is None:
            return output
        for index in range(4):
            lower, upper = 25.0 * index, 25.0 * (index + 1)
            data = [row for row in self.samples if lower <= float(row['progress_percent']) < upper or
                    (index == 3 and float(row['progress_percent']) == 100.0)]
            if not data:
                output[f'{lower:.0f}-{upper:.0f}%'] = {'samples': 0}
                continue
            velocities = [abs(float(row['actual_v_m_s'])) for row in data]
            errors = [abs(float(row['lateral_error_m'])) for row in data]
            curvatures = [abs(float(row['curvature_1_m'])) for row in data]
            output[f'{lower:.0f}-{upper:.0f}%'] = {
                'samples': len(data),
                'time_s': max(0.0, float(data[-1]['time_s']) - float(data[0]['time_s'])),
                'average_velocity_m_s': _safe_mean(velocities),
                'average_abs_curvature_1_m': _safe_mean(curvatures),
                'lateral_rmse_m': _safe_rmse(errors),
            }
        return output

    def _aeb_interventions(self, now: float) -> list[dict[str, float | int | None]]:
        """Return sampled AEB intervals; an active final interval ends at ``now``."""
        if self.started_at is None:
            return []
        output: list[dict[str, float | int | None]] = []
        active_rows: list[dict[str, float | int | bool | None]] = []

        def close(end_time: float) -> None:
            if not active_rows:
                return
            distances = [
                float(row['aeb_critical_distance_m']) for row in active_rows
                if _finite(row['aeb_critical_distance_m']) is not None
            ]
            first = active_rows[0]
            last = active_rows[-1]
            output.append({
                'start_time_s': float(first['time_s']) - self.started_at,
                'end_time_s': end_time - self.started_at,
                'duration_s': max(0.0, end_time - float(first['time_s'])),
                'start_path_s_m': float(first['path_s_m']),
                'end_path_s_m': float(last['path_s_m']),
                'progress_delta_m': float(last['path_s_m']) - float(first['path_s_m']),
                'minimum_front_lidar_distance_m': min(distances) if distances else None,
                'maximum_actual_angular_speed_rad_s': max(
                    abs(float(row['actual_omega_rad_s'])) for row in active_rows),
            })

        for row in self.samples:
            if bool(row['aeb_active']):
                active_rows.append(row)
            elif active_rows:
                close(float(row['time_s']))
                active_rows = []
        if active_rows:
            close(now)
        return output

    def _safety_summary(self, elapsed: float | None, interventions: list[dict[str, float | int | None]]) -> dict[str, Any]:
        distances = [
            float(row['aeb_critical_distance_m']) for row in self.samples
            if _finite(row['aeb_critical_distance_m']) is not None
        ]
        longest = max((float(item['duration_s']) for item in interventions), default=0.0)
        return {
            # No Gazebo contact sensor is currently bridged.  Do not turn a
            # LiDAR clearance into a fabricated collision label.
            'collision_detected': None,
            'collision_detection_status': 'not_instrumented_no_contact_topic',
            'minimum_lidar_distance_semantics': 'minimum valid range in lidar_data front sector [-20,+20] deg',
            'minimum_front_lidar_distance_m': min(distances) if distances else None,
            'aeb_events': self.aeb_events,
            'aeb_total_active_time_s': self.aeb_active_time_s,
            'aeb_longest_intervention_s': longest,
            'aeb_active_fraction': None if not elapsed else self.aeb_active_time_s / elapsed,
            'aeb_command_blocked_time_s': self.aeb_command_blocked_time_s,
            'aeb_interventions': interventions,
        }

    def _summary(self, completed: bool, reason: str, now: float) -> dict[str, Any]:
        errors = [float(row['lateral_error_m']) for row in self.samples]
        velocities = [abs(float(row['actual_v_m_s'])) for row in self.samples]
        omegas = [abs(float(row['actual_omega_rad_s'])) for row in self.samples]
        nav_omegas = [abs(float(row['target_omega_rad_s'])) for row in self.samples]
        target_saturated = [value >= 0.995 * self.max_omega for value in nav_omegas] if self.max_omega else []
        wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        half_track = 0.5 * float(self.get_parameter('wheel_separation_m').value)
        wheel_limit = wheel_radius * float(self.get_parameter('wheel_max_angular_velocity').value)
        physically_excessive = [
            abs(float(row['target_v_m_s'])) + half_track * abs(float(row['target_omega_rad_s'])) > wheel_limit + 1e-6
            for row in self.samples
        ]
        elapsed = None if self.started_at is None else max(0.0, now - self.started_at)
        interventions = self._aeb_interventions(now)
        safety = self._safety_summary(elapsed, interventions)
        return {
            'completed': bool(completed), 'reason': reason, 'laps': self.laps_completed,
            'expected_laps': self.expected_laps, 'total_time_s': elapsed,
            'average_velocity_m_s': None if not elapsed or elapsed <= 0.0 else self.distance_travelled_m / elapsed,
            'mean_sampled_velocity_m_s': _safe_mean(velocities),
            'maximum_velocity_m_s': None if not velocities else max(velocities),
            'mean_abs_angular_velocity_rad_s': _safe_mean(omegas),
            'maximum_abs_angular_velocity_rad_s': None if not omegas else max(omegas),
            'lateral_rmse_m': _safe_rmse(errors),
            'lateral_max_error_m': None if not errors else max(abs(value) for value in errors),
            'distance_travelled_m': self.distance_travelled_m,
            'path_progress_percent': None if self.profile is None else 100.0 * self.last_s / self.profile.length,
            'aeb_events': self.aeb_events,
            'aeb_active_time_s': self.aeb_active_time_s,
            'aeb_longest_intervention_s': safety['aeb_longest_intervention_s'],
            'aeb_active_fraction': safety['aeb_active_fraction'],
            'aeb_command_blocked_time_s': self.aeb_command_blocked_time_s,
            'minimum_lidar_distance_m': safety['minimum_front_lidar_distance_m'],
            'stops': self.stop_count,
            'omega_saturated_time_fraction': None if not target_saturated else sum(target_saturated) / len(target_saturated),
            'wheel_limit_exceeded_time_fraction': None if not physically_excessive else sum(physically_excessive) / len(physically_excessive),
            'lap_1': self._lap_summary(1), 'lap_2': self._lap_summary(2),
            'sections': self._section_summary(),
            'stuck_evidence': self.stuck_evidence,
        }

    def _write_trace(self) -> None:
        with self.trace_file.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=TRACE_COLUMNS)
            writer.writeheader()
            for row in self.samples:
                writer.writerow({key: row.get(key) for key in TRACE_COLUMNS})

    def _finish(self, now: float, completed: bool, reason: str) -> None:
        if self.completed:
            return
        self.completed = True
        self.failure_reason = None if completed else reason
        self._write_trace()
        content = {
            'run_id': self.run_id, 'controller': self.controller,
            'path_profile': {} if self.profile is None else self.profile.as_yaml(),
            'result': self._summary(completed, reason, now),
        }
        content['mission'] = {
            'completed': content['result']['completed'],
            'laps': content['result']['laps'],
            'expected_laps': content['result']['expected_laps'],
            'total_time_s': content['result']['total_time_s'],
            'path_progress_percent': content['result']['path_progress_percent'],
        }
        content['safety'] = self._safety_summary(
            content['result']['total_time_s'], self._aeb_interventions(now))
        with self.results_file.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(content, stream, sort_keys=False)
        state = 'completed' if completed else f'failed:{reason}'
        self._publish_status(state)
        self.get_logger().info(f'Optimization monitor {state}; wrote {self.results_file}.')

    def destroy_node(self) -> bool:
        if not self.initial_state_written:
            self._write_initial_state('monitor_shutdown_before_motion')
        if not self.completed and self.started_at is not None:
            self._finish(self._now(), False, 'monitor_shutdown_before_completion')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OptimizationMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

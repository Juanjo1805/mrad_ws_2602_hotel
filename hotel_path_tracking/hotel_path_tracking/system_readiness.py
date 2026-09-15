#!/usr/bin/env python3
"""Observe ROS readiness without changing the simulation or localization state.

The experiment runner calls this helper before publishing an initial pose and
again immediately before launching navigation.  It is deliberately a
subscriber / TF listener only: no commands, parameters, poses, lifecycle
transitions, or Gazebo services are sent by this module.
"""

from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timezone
import math
from pathlib import Path
import time
from typing import Any

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_ros import TransformException
import yaml


def _yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _stamp(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _finite_pose(pose) -> bool:
    values = [
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
    ]
    return all(math.isfinite(float(value)) for value in values)


def _pose_yaml(message: PoseWithCovarianceStamped | None) -> dict[str, Any] | None:
    if message is None:
        return None
    pose = message.pose.pose
    return {
        'frame_id': message.header.frame_id,
        'timestamp_s': _stamp(message.header.stamp),
        'x_m': float(pose.position.x),
        'y_m': float(pose.position.y),
        'z_m': float(pose.position.z),
        'yaw_rad': _yaw(pose.orientation),
        'covariance_x_m2': float(message.pose.covariance[0]),
        'covariance_y_m2': float(message.pose.covariance[7]),
        'covariance_yaw_rad2': float(message.pose.covariance[35]),
    }


def _odom_yaml(message: Odometry | None) -> dict[str, Any] | None:
    if message is None:
        return None
    pose = message.pose.pose
    twist = message.twist.twist
    return {
        'frame_id': message.header.frame_id,
        'child_frame_id': message.child_frame_id,
        'timestamp_s': _stamp(message.header.stamp),
        'x_m': float(pose.position.x),
        'y_m': float(pose.position.y),
        'z_m': float(pose.position.z),
        'yaw_rad': _yaw(pose.orientation),
        'linear_x_m_s': float(twist.linear.x),
        'angular_z_rad_s': float(twist.angular.z),
    }


def _transform_yaml(transform) -> dict[str, Any]:
    translation = transform.transform.translation
    return {
        'parent_frame': transform.header.frame_id,
        'child_frame': transform.child_frame_id,
        'timestamp_s': _stamp(transform.header.stamp),
        'x_m': float(translation.x),
        'y_m': float(translation.y),
        'z_m': float(translation.z),
        'yaw_rad': _yaw(transform.transform.rotation),
    }


class SystemReadiness(Node):
    """Wait for a sustained, finite sensor / TF / AMCL state and save it."""

    def __init__(self) -> None:
        super().__init__('system_readiness')
        self.declare_parameter('output_file', '')
        self.declare_parameter('phase', 'bootstrap')
        self.declare_parameter('timeout_s', 75.0)
        self.declare_parameter('stable_window_s', 2.0)
        self.declare_parameter('sample_frequency_hz', 10.0)
        self.declare_parameter('odom_topic', '/ekf/odometry')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('minimum_odom_samples', 15)
        self.declare_parameter('minimum_scan_samples', 5)
        self.declare_parameter('minimum_tf_samples', 10)
        self.declare_parameter('minimum_amcl_samples', 1)
        self.declare_parameter('minimum_odom_rate_hz', 5.0)
        self.declare_parameter('minimum_scan_rate_hz', 1.0)
        # Values <= 0 explicitly disable a covariance threshold.  The default
        # rejects only obviously diffuse localization, while the exact values
        # measured before navigation are retained in the YAML output.
        self.declare_parameter('maximum_amcl_x_covariance_m2', 2.0)
        self.declare_parameter('maximum_amcl_y_covariance_m2', 2.0)
        self.declare_parameter('maximum_amcl_yaw_covariance_rad2', 1.0)

        output = str(self.get_parameter('output_file').value)
        if not output:
            raise ValueError('output_file is required')
        self.output_file = Path(output)
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        self.phase = str(self.get_parameter('phase').value)
        self.require_localization = self.phase.lower() in {'localized', 'navigation', 'pre_navigation'}
        self.started_wall = time.monotonic()
        self.started_utc = datetime.now(timezone.utc).isoformat()
        self.finished = False
        self.succeeded = False
        self.failure_reason: str | None = None

        self.odom: Odometry | None = None
        self.scan: LaserScan | None = None
        self.amcl_pose: PoseWithCovarianceStamped | None = None
        self.initial_pose: PoseWithCovarianceStamped | None = None
        self.received: dict[str, list[float]] = defaultdict(list)
        self.first_seen: dict[str, float] = {}
        self.transforms: dict[str, Any] = {}
        self.tf_seen: dict[str, list[float]] = defaultdict(list)
        self.tf_first_seen: dict[str, float] = {}

        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 30)
        self.create_subscription(LaserScan, str(self.get_parameter('scan_topic').value), self._on_scan, 30)
        self.create_subscription(PoseWithCovarianceStamped,
                                 str(self.get_parameter('amcl_pose_topic').value), self._on_amcl, 30)
        self.create_subscription(PoseWithCovarianceStamped,
                                 str(self.get_parameter('initial_pose_topic').value), self._on_initial_pose, 30)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        frequency = max(2.0, float(self.get_parameter('sample_frequency_hz').value))
        self.timer = self.create_timer(1.0 / frequency, self._on_timer)

    def _mark(self, key: str) -> None:
        now = time.monotonic()
        self.received[key].append(now)
        self.first_seen.setdefault(key, now)
        # Only a short recent history is necessary for rate and stability.
        cutoff = now - max(10.0, 3.0 * float(self.get_parameter('stable_window_s').value))
        self.received[key] = [value for value in self.received[key] if value >= cutoff]

    def _on_odom(self, message: Odometry) -> None:
        if _finite_pose(message.pose.pose) and all(math.isfinite(float(value)) for value in (
                message.twist.twist.linear.x, message.twist.twist.angular.z)):
            self.odom = message
            self._mark('ekf_odometry')

    def _on_scan(self, message: LaserScan) -> None:
        valid = [value for value in message.ranges
                 if math.isfinite(float(value)) and message.range_min <= value <= message.range_max]
        if valid:
            self.scan = message
            self._mark('scan')

    def _on_amcl(self, message: PoseWithCovarianceStamped) -> None:
        covariance = (message.pose.covariance[0], message.pose.covariance[7], message.pose.covariance[35])
        if _finite_pose(message.pose.pose) and all(math.isfinite(float(value)) for value in covariance):
            self.amcl_pose = message
            self._mark('amcl_pose')

    def _on_initial_pose(self, message: PoseWithCovarianceStamped) -> None:
        if _finite_pose(message.pose.pose):
            self.initial_pose = message
            self._mark('initialpose')

    @staticmethod
    def _rate(values: list[float]) -> float:
        if len(values) < 2:
            return 0.0
        duration = values[-1] - values[0]
        return 0.0 if duration <= 1e-6 else (len(values) - 1) / duration

    def _tf(self, parent: str, child: str) -> Any | None:
        key = f'{parent}->{child}'
        try:
            transform = self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.02))
        except TransformException:
            return None
        values = (
            transform.transform.translation.x, transform.transform.translation.y,
            transform.transform.translation.z, transform.transform.rotation.x,
            transform.transform.rotation.y, transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return None
        now = time.monotonic()
        self.transforms[key] = transform
        self.tf_seen[key].append(now)
        self.tf_first_seen.setdefault(key, now)
        cutoff = now - max(10.0, 3.0 * float(self.get_parameter('stable_window_s').value))
        self.tf_seen[key] = [value for value in self.tf_seen[key] if value >= cutoff]
        return transform

    def _stable(self, values: list[float], minimum: int, minimum_rate: float = 0.0) -> bool:
        if len(values) < minimum:
            return False
        window = float(self.get_parameter('stable_window_s').value)
        if values[-1] - values[0] < 0.80 * window:
            return False
        return minimum_rate <= 0.0 or self._rate(values) >= minimum_rate

    def _amcl_covariance_ready(self) -> bool:
        if self.amcl_pose is None:
            return False
        values = (self.amcl_pose.pose.covariance[0], self.amcl_pose.pose.covariance[7],
                  self.amcl_pose.pose.covariance[35])
        limits = (
            float(self.get_parameter('maximum_amcl_x_covariance_m2').value),
            float(self.get_parameter('maximum_amcl_y_covariance_m2').value),
            float(self.get_parameter('maximum_amcl_yaw_covariance_rad2').value),
        )
        return all(math.isfinite(float(value)) and (limit <= 0.0 or value <= limit)
                   for value, limit in zip(values, limits))

    def _conditions(self) -> dict[str, dict[str, Any]]:
        map_frame = str(self.get_parameter('map_frame').value)
        odom_frame = str(self.get_parameter('odom_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        self._tf(odom_frame, base_frame)
        if self.require_localization:
            self._tf(map_frame, odom_frame)
            self._tf(map_frame, base_frame)
        min_tf = int(self.get_parameter('minimum_tf_samples').value)
        conditions = {
            'ekf_odometry': {
                'required': True,
                'samples': len(self.received['ekf_odometry']),
                'rate_hz': self._rate(self.received['ekf_odometry']),
                'ready': self._stable(self.received['ekf_odometry'],
                                      int(self.get_parameter('minimum_odom_samples').value),
                                      float(self.get_parameter('minimum_odom_rate_hz').value)),
            },
            'scan': {
                'required': True,
                'samples': len(self.received['scan']),
                'rate_hz': self._rate(self.received['scan']),
                'ready': self._stable(self.received['scan'],
                                      int(self.get_parameter('minimum_scan_samples').value),
                                      float(self.get_parameter('minimum_scan_rate_hz').value)),
            },
            f'{odom_frame}->{base_frame}': {
                'required': True,
                'samples': len(self.tf_seen[f'{odom_frame}->{base_frame}']),
                'ready': self._stable(self.tf_seen[f'{odom_frame}->{base_frame}'], min_tf),
            },
        }
        if self.require_localization:
            conditions[f'{map_frame}->{odom_frame}'] = {
                'required': True,
                'samples': len(self.tf_seen[f'{map_frame}->{odom_frame}']),
                'ready': self._stable(self.tf_seen[f'{map_frame}->{odom_frame}'], min_tf),
            }
            conditions[f'{map_frame}->{base_frame}'] = {
                'required': True,
                'samples': len(self.tf_seen[f'{map_frame}->{base_frame}']),
                'ready': self._stable(self.tf_seen[f'{map_frame}->{base_frame}'], min_tf),
            }
            conditions['amcl_pose'] = {
                'required': True,
                'samples': len(self.received['amcl_pose']),
                'covariance_ready': self._amcl_covariance_ready(),
                'ready': len(self.received['amcl_pose']) >= int(
                    self.get_parameter('minimum_amcl_samples').value) and self._amcl_covariance_ready(),
            }
        return conditions

    def _snapshot(self) -> dict[str, Any]:
        transforms = {key: _transform_yaml(value) for key, value in self.transforms.items()}
        return {
            'gazebo_ground_truth': {
                'status': 'not_available_through_current_ros_bridge',
                'note': 'Physical Gazebo pose is captured separately by the runner when the Gazebo service is available.',
            },
            'ekf_odometry': _odom_yaml(self.odom),
            'amcl_pose': _pose_yaml(self.amcl_pose),
            'initialpose_observed': _pose_yaml(self.initial_pose),
            'transforms': transforms,
        }

    def _write(self, ready: bool, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        self.succeeded = bool(ready)
        self.failure_reason = None if ready else reason
        now = time.monotonic()
        conditions = self._conditions()
        content = {
            'phase': self.phase,
            'ready': bool(ready),
            'reason': reason,
            'started_utc': self.started_utc,
            'completed_utc': datetime.now(timezone.utc).isoformat(),
            'elapsed_wall_s': now - self.started_wall,
            'requirements': {
                'stable_window_s': float(self.get_parameter('stable_window_s').value),
                'minimum_odom_samples': int(self.get_parameter('minimum_odom_samples').value),
                'minimum_scan_samples': int(self.get_parameter('minimum_scan_samples').value),
                'minimum_tf_samples': int(self.get_parameter('minimum_tf_samples').value),
                'minimum_amcl_samples': int(self.get_parameter('minimum_amcl_samples').value),
            },
            'conditions': conditions,
            'snapshot': self._snapshot(),
        }
        with self.output_file.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(content, stream, sort_keys=False)
        self.get_logger().info(f'Readiness {"passed" if ready else "failed"}: {reason}; wrote {self.output_file}.')

    def _on_timer(self) -> None:
        if self.finished:
            return
        conditions = self._conditions()
        if all(bool(item.get('ready')) for item in conditions.values() if item.get('required')):
            self._write(True, 'all_required_conditions_stable')
            rclpy.shutdown()
            return
        if time.monotonic() - self.started_wall > float(self.get_parameter('timeout_s').value):
            self._write(False, 'readiness_timeout')
            rclpy.shutdown()

    def destroy_node(self) -> bool:
        if not self.finished:
            self._write(False, self.failure_reason or 'readiness_shutdown_before_completion')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemReadiness()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ready = node.succeeded
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    # ``rclpy.shutdown`` is the normal success path.  The YAML is the source
    # of truth for the runner; the process exit code mirrors it for scripting.
    raise SystemExit(0 if ready else 1)


if __name__ == '__main__':
    main()

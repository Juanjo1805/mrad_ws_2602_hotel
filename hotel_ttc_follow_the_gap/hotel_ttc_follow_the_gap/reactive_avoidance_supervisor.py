#!/usr/bin/env python3
"""Path-aware reactive supervisor using the project's Follow-The-Gap logic."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32, String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros import TransformException

from .reactive_core import (ReactiveState, ReactiveStateMachine, point_segment_distance,
                            analyze_gap, reactive_preview, safe_gap_command)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class ReactiveAvoidanceSupervisor(Node):
    """Only publishes `/cmd_vel_gap` while a path blockage is persistent."""

    def __init__(self) -> None:
        super().__init__('reactive_avoidance_supervisor')
        self._declare_parameters()
        self._read_parameters()
        self.path: Path | None = None
        self.scan: LaserScan | None = None
        self.grid: OccupancyGrid | None = None
        self.speed = 0.0
        self.closest_index = 0
        self.lateral_error = math.inf
        self.heading_error = math.inf
        self.last_rejoin_index = 0
        self.last_log = -math.inf
        self.machine = ReactiveStateMachine(self.activation_persistence, self.clear_persistence,
                                            self.rejoin_stable, self.minimum_avoiding)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._interfaces()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)
        # A steady-time heartbeat makes liveness visible even when an isolated
        # `use_sim_time:=true` node has no /clock publisher yet.
        self.state_heartbeat = self.create_timer(
            1.0, self._publish_state_heartbeat,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._publish_state_heartbeat()
        self.get_logger().info('Reactive avoidance ready; FTG is gated until TRACKING -> AVOIDING.')

    def _declare_parameters(self) -> None:
        values = {
            'enabled': True, 'path_topic': '/planned_path', 'scan_topic': '/scan',
            'odom_topic': '/ekf/odometry', 'base_frame': 'base_link', 'rate_hz': 10.0,
            'preview_distance_base': 1.10, 'preview_distance_speed_gain': 0.55,
            'minimum_preview_distance': 1.00, 'maximum_preview_distance': 2.50,
            'deceleration_limit': 1.40, 'control_latency_s': 0.20,
            'robot_front_extent': 0.20, 'corridor_half_robot_width': 0.20,
            'corridor_safety_margin': 0.15, 'obstacle_activation_distance': 2.20,
            'obstacle_points_threshold': 3, 'activation_persistence_time': 0.30,
            'clear_persistence_time': 0.60, 'minimum_avoiding_time': 0.80,
            'ftg_max_linear_velocity': 0.68, 'ftg_max_angular_velocity': 2.00,
            'ftg_steering_gain': 1.20, 'ftg_steering_slowdown': 1.20,
            'wheel_linear_limit': 1.00, 'wheel_separation_m': 0.44,
            'rejoin_max_linear_velocity': 0.68, 'rejoin_lateral_error_threshold': 0.15,
            'rejoin_heading_error_threshold': 0.22, 'rejoin_stable_time': 0.80,
            'rejoin_offset_distance': 0.70, 'ftg_path_bias_enabled': True,
            'ftg_path_alignment_gain': 0.20, 'ftg_progress_gain': 0.10,
            'minimum_gap_width': 0.55, 'bubble_radius': 0.35, 'ftg_fov_deg': 120.0,
            'minimum_clearance': 0.12, 'map_topic': '/map',
        }
        for key, value in values.items():
            self.declare_parameter(key, value)

    def _read_parameters(self) -> None:
        get = lambda name: self.get_parameter(name).value
        self.enabled = bool(get('enabled')); self.path_topic = str(get('path_topic')); self.base_frame = str(get('base_frame'))
        self.rate_hz = max(1.0, float(get('rate_hz'))); self.base_preview = float(get('preview_distance_base'))
        self.preview_gain = float(get('preview_distance_speed_gain')); self.preview_min = float(get('minimum_preview_distance'))
        self.preview_max = float(get('maximum_preview_distance')); self.deceleration = float(get('deceleration_limit'))
        self.latency = float(get('control_latency_s')); self.front_extent = float(get('robot_front_extent'))
        self.corridor_width = float(get('corridor_half_robot_width')) + float(get('corridor_safety_margin'))
        self.activation_distance = float(get('obstacle_activation_distance')); self.points_threshold = int(get('obstacle_points_threshold'))
        self.activation_persistence = float(get('activation_persistence_time')); self.clear_persistence = float(get('clear_persistence_time'))
        self.minimum_avoiding = float(get('minimum_avoiding_time')); self.ftg_vmax = float(get('ftg_max_linear_velocity'))
        self.ftg_wmax = float(get('ftg_max_angular_velocity')); self.steering_gain = float(get('ftg_steering_gain'))
        self.steering_slowdown = float(get('ftg_steering_slowdown')); self.wheel_limit = float(get('wheel_linear_limit'))
        self.half_track = 0.5 * float(get('wheel_separation_m')); self.rejoin_vmax = float(get('rejoin_max_linear_velocity'))
        self.rejoin_lateral = float(get('rejoin_lateral_error_threshold')); self.rejoin_heading = float(get('rejoin_heading_error_threshold'))
        self.rejoin_stable = float(get('rejoin_stable_time')); self.rejoin_offset = float(get('rejoin_offset_distance'))
        self.path_bias = float(get('ftg_path_alignment_gain')) if bool(get('ftg_path_bias_enabled')) else 0.0
        self.min_gap_width = float(get('minimum_gap_width')); self.bubble_radius = float(get('bubble_radius'))
        self.ftg_half_fov = math.radians(float(get('ftg_fov_deg')) / 2.0)
        self.progress_bias = float(get('ftg_progress_gain')) if bool(get('ftg_path_bias_enabled')) else 0.0
        self.minimum_clearance = float(get('minimum_clearance'))

    def _interfaces(self) -> None:
        self.create_subscription(Path, self.path_topic, self._path, 10)
        self.create_subscription(LaserScan, str(self.get_parameter('scan_topic').value), self._scan, 20)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value), self._odom, 20)
        self.create_subscription(Int32, '/path_tracking/closest_index', self._index, 20)
        self.create_subscription(Float32, '/path_tracking/lateral_error', self._lateral, 20)
        self.create_subscription(Float32, '/path_tracking/heading_error', self._heading, 20)
        self.create_subscription(OccupancyGrid, str(self.get_parameter('map_topic').value), self._map, 2)
        self.cmd = self.create_publisher(TwistStamped, '/cmd_vel_gap', 10)
        self.rejoin_limit = self.create_publisher(Float32, '/reactive_avoidance/rejoin_speed_limit', 10)
        self.state_pub = self.create_publisher(String, '/reactive_avoidance/state', 10)
        self.active_pub = self.create_publisher(Bool, '/reactive_avoidance/active', 10)
        self.blocking_pub = self.create_publisher(Bool, '/reactive_avoidance/blocking_obstacle', 10)
        self.unmapped_pub = self.create_publisher(Bool, '/reactive_avoidance/unmapped_obstacle', 10)
        self.distance_pub = self.create_publisher(Float32, '/reactive_avoidance/obstacle_distance', 10)
        self.clear_pub = self.create_publisher(Bool, '/reactive_avoidance/path_clear', 10)
        self.preview_pub = self.create_publisher(Float32, '/reactive_avoidance/preview_distance', 10)
        self.width_pub = self.create_publisher(Float32, '/reactive_avoidance/corridor_width', 10)
        self.rejoin_index_pub = self.create_publisher(Int32, '/reactive_avoidance/rejoin_index', 10)
        self.rejoin_distance_pub = self.create_publisher(Float32, '/reactive_avoidance/rejoin_distance', 10)
        self.rejoin_heading_pub = self.create_publisher(Float32, '/reactive_avoidance/rejoin_heading_error', 10)
        self.target_pub = self.create_publisher(Float32, '/reactive_avoidance/ftg_target_angle', 10)
        self.gap_pub = self.create_publisher(Float32, '/reactive_avoidance/ftg_gap_width', 10)
        self.valid_gap_pub = self.create_publisher(Bool, '/reactive_avoidance/ftg_valid_gap', 10)
        self.ftg_stats_pub = {
            'scan_beams_total': self.create_publisher(Int32, '/reactive_avoidance/ftg_scan_beams_total', 10),
            'scan_beams_in_fov': self.create_publisher(Int32, '/reactive_avoidance/ftg_scan_beams_in_fov', 10),
            'valid_ranges': self.create_publisher(Int32, '/reactive_avoidance/ftg_valid_ranges', 10),
            'invalid_ranges': self.create_publisher(Int32, '/reactive_avoidance/ftg_invalid_ranges', 10),
            'fov_start_index': self.create_publisher(Int32, '/reactive_avoidance/ftg_fov_start_index', 10),
            'fov_end_index': self.create_publisher(Int32, '/reactive_avoidance/ftg_fov_end_index', 10),
            'closest_obstacle_index': self.create_publisher(Int32, '/reactive_avoidance/ftg_closest_obstacle_index', 10),
            'bubble_removed_beams': self.create_publisher(Int32, '/reactive_avoidance/ftg_bubble_removed_beams', 10),
            'candidate_gap_count': self.create_publisher(Int32, '/reactive_avoidance/ftg_candidate_gap_count', 10),
            'valid_gap_count': self.create_publisher(Int32, '/reactive_avoidance/ftg_valid_gap_count', 10),
        }
        self.fov_start_pub = self.create_publisher(Float32, '/reactive_avoidance/ftg_fov_start_angle', 10)
        self.fov_end_pub = self.create_publisher(Float32, '/reactive_avoidance/ftg_fov_end_angle', 10)
        self.closest_range_pub = self.create_publisher(Float32, '/reactive_avoidance/ftg_closest_obstacle_range', 10)
        self.gap_details_pub = self.create_publisher(String, '/reactive_avoidance/ftg_gap_details', 10)
        self.markers = self.create_publisher(MarkerArray, '/reactive_avoidance/markers', 10)

    def _publish_state_heartbeat(self) -> None:
        self.state_pub.publish(String(data=self.machine.state.value))
        self.active_pub.publish(Bool(data=self.machine.state != ReactiveState.TRACKING))

    def _path(self, message: Path) -> None: self.path = message if message.poses else None
    def _scan(self, message: LaserScan) -> None: self.scan = message
    def _odom(self, message: Odometry) -> None: self.speed = abs(float(message.twist.twist.linear.x))
    def _index(self, message: Int32) -> None: self.closest_index = max(self.closest_index, int(message.data))
    def _lateral(self, message: Float32) -> None: self.lateral_error = float(message.data)
    def _heading(self, message: Float32) -> None: self.heading_error = float(message.data)
    def _map(self, message: OccupancyGrid) -> None: self.grid = message

    def _robot_pose(self) -> tuple[float, float, float] | None:
        if self.path is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(self.path.header.frame_id, self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.10))
        except TransformException:
            return None
        return transform.transform.translation.x, transform.transform.translation.y, _yaw(transform.transform.rotation)

    def _future_path_base(self, pose: tuple[float, float, float], preview: float) -> tuple[list[tuple[float, float]], int]:
        assert self.path is not None
        px, py, yaw = pose; c, s = math.cos(yaw), math.sin(yaw)
        start = min(max(0, self.closest_index), len(self.path.poses) - 1)
        points: list[tuple[float, float]] = []
        travelled = 0.0; previous = None; final_index = start
        for index in range(start, len(self.path.poses)):
            item = self.path.poses[index].pose.position
            if previous is not None:
                travelled += math.hypot(item.x - previous.x, item.y - previous.y)
            previous = item
            local_x = c * (item.x - px) + s * (item.y - py)
            local_y = -s * (item.x - px) + c * (item.y - py)
            points.append((local_x, local_y)); final_index = index
            if travelled >= preview:
                break
        return points, final_index

    def _blocking(self, pose: tuple[float, float, float], points: Sequence[tuple[float, float]], preview: float) -> tuple[bool, float, list[tuple[float, float]], bool]:
        if self.scan is None or len(points) < 2:
            return False, math.inf, [], False
        ranges = np.asarray(self.scan.ranges, dtype=float)
        angles = self.scan.angle_min + np.arange(ranges.size) * self.scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= self.scan.range_min) & (ranges <= min(self.scan.range_max, preview))
        hits: list[tuple[float, float]] = []; minimum = math.inf
        for distance, angle in zip(ranges[valid], angles[valid]):
            hit = (float(distance * math.cos(angle)), float(distance * math.sin(angle)))
            if hit[0] < -0.05:
                continue
            near = min(point_segment_distance(hit, a, b)[0] for a, b in zip(points, points[1:]))
            if near <= self.corridor_width:
                hits.append(hit); minimum = min(minimum, float(distance))
        blocking = len(hits) >= self.points_threshold and minimum <= min(preview, self.activation_distance)
        unmapped = any(self._is_map_free(pose, hit) for hit in hits) if blocking else False
        return blocking, minimum, hits, unmapped

    def _is_map_free(self, pose: tuple[float, float, float], hit: tuple[float, float]) -> bool:
        """Classify only for diagnostics; it never gates physical avoidance."""
        if self.grid is None or self.grid.info.resolution <= 0.0:
            return False
        px, py, yaw = pose; hx, hy = hit
        map_x = px + math.cos(yaw) * hx - math.sin(yaw) * hy
        map_y = py + math.sin(yaw) * hx + math.cos(yaw) * hy
        origin = self.grid.info.origin.position
        column = int((map_x - origin.x) / self.grid.info.resolution)
        row = int((map_y - origin.y) / self.grid.info.resolution)
        if column < 0 or row < 0 or column >= self.grid.info.width or row >= self.grid.info.height:
            return False
        occupancy = self.grid.data[row * self.grid.info.width + column]
        return 0 <= occupancy < 10

    def _gap(self, preview_points: Sequence[tuple[float, float]]) -> tuple[float, float, bool, object, dict[str, float | int]]:
        assert self.scan is not None
        angles = self.scan.angle_min + np.arange(len(self.scan.ranges)) * self.scan.angle_increment
        raw = np.asarray(self.scan.ranges, dtype=float)
        fov_mask = np.abs(angles) <= self.ftg_half_fov
        indices = np.flatnonzero(fov_mask)
        ranges = raw.copy(); ranges[~fov_mask] = 0.0
        target = math.atan2(preview_points[-1][1], preview_points[-1][0]) if preview_points else 0.0
        result, diagnostics = analyze_gap(angles, ranges, self.scan.range_max,
                            self.bubble_radius + 0.15 * self.speed, self.minimum_clearance,
                            self.min_gap_width, target, self.path_bias, self.progress_bias,
                            self.activation_distance, True)
        obstacle_mask = fov_mask & np.isfinite(raw) & (raw >= self.scan.range_min) & (raw <= self.activation_distance)
        closest = int(np.argmin(np.where(obstacle_mask, raw, np.inf))) if np.any(obstacle_mask) else -1
        stats: dict[str, float | int] = {
            'scan_beams_total': int(raw.size), 'scan_beams_in_fov': int(indices.size),
            'valid_ranges': int(np.count_nonzero(fov_mask & np.isfinite(raw) & (raw >= self.scan.range_min) & (raw <= self.scan.range_max))),
            'invalid_ranges': int(np.count_nonzero(fov_mask & ~np.isfinite(raw))),
            'fov_start_index': int(indices[0]) if indices.size else -1, 'fov_end_index': int(indices[-1]) if indices.size else -1,
            'fov_start_angle': float(angles[indices[0]]) if indices.size else 0.0, 'fov_end_angle': float(angles[indices[-1]]) if indices.size else 0.0,
            'closest_obstacle_index': closest, 'closest_obstacle_range': float(raw[closest]) if closest >= 0 else -1.0,
        }
        return result.angle, result.width, result.valid, diagnostics, stats

    def _publish_gap_diagnostics(self, valid: bool, diagnostics, stats: dict[str, float | int]) -> None:
        self.valid_gap_pub.publish(Bool(data=valid))
        for name, publisher in self.ftg_stats_pub.items():
            value = diagnostics.bubble_removed_beams if name == 'bubble_removed_beams' else (diagnostics.candidate_gap_count if name == 'candidate_gap_count' else (diagnostics.valid_gap_count if name == 'valid_gap_count' else stats[name]))
            publisher.publish(Int32(data=int(value)))
        self.fov_start_pub.publish(Float32(data=float(stats['fov_start_angle']))); self.fov_end_pub.publish(Float32(data=float(stats['fov_end_angle'])))
        self.closest_range_pub.publish(Float32(data=float(stats['closest_obstacle_range'])))
        detail = '; '.join(f'[{c.start_index}:{c.end_index}] a={c.start_angle:.2f}:{c.end_angle:.2f} aw={c.angular_width:.2f} r={c.representative_range:.2f} w={c.physical_width:.2f} accepted={c.accepted} {c.rejection_reason}' for c in diagnostics.candidates)
        self.gap_details_pub.publish(String(data=detail or 'no_candidate_gaps'))

    def _rejoin_index(self, points: Sequence[tuple[float, float]]) -> int:
        if self.path is None:
            return self.last_rejoin_index
        accumulated = 0.0; previous = None; index = self.closest_index
        for index, current in enumerate(self.path.poses[self.closest_index:], self.closest_index):
            if previous is not None:
                accumulated += math.hypot(current.pose.position.x - previous.pose.position.x, current.pose.position.y - previous.pose.position.y)
            previous = current
            if accumulated >= self.rejoin_offset:
                break
        self.last_rejoin_index = max(self.last_rejoin_index, self.closest_index, index)
        return self.last_rejoin_index

    def _publish_markers(self, points: Sequence[tuple[float, float]], hits: Sequence[tuple[float, float]],
                         angle: float, rejoin: tuple[float, float], state: ReactiveState) -> None:
        now = self.get_clock().now().to_msg(); output = MarkerArray()
        for marker_id, namespace, color, scale in ((0, 'corridor', (0.1, 0.8, 1.0), 0.03), (1, 'blocking_hits', (1.0, 0.1, 0.1), 0.08)):
            marker = Marker(); marker.header.frame_id = self.base_frame; marker.header.stamp = now; marker.ns = namespace; marker.id = marker_id
            marker.type = Marker.LINE_STRIP if marker_id == 0 else Marker.POINTS; marker.action = Marker.ADD; marker.scale.x = scale; marker.scale.y = scale
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 0.9
            marker.points = [Point(x=x, y=y, z=0.05) for x, y in (points if marker_id == 0 else hits)]; output.markers.append(marker)
        arrow = Marker(); arrow.header.frame_id = self.base_frame; arrow.header.stamp = now; arrow.ns = 'ftg_target'; arrow.id = 2; arrow.type = Marker.ARROW; arrow.action = Marker.ADD
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.8, 0.08, 0.08; arrow.color.g, arrow.color.a = 1.0, 1.0
        arrow.points = [Point(), Point(x=0.8 * math.cos(angle), y=0.8 * math.sin(angle), z=0.05)]; output.markers.append(arrow)
        rejoin_marker = Marker(); rejoin_marker.header.frame_id = self.base_frame; rejoin_marker.header.stamp = now; rejoin_marker.ns = 'rejoin_point'; rejoin_marker.id = 3
        rejoin_marker.type = Marker.SPHERE; rejoin_marker.action = Marker.ADD; rejoin_marker.scale.x = rejoin_marker.scale.y = rejoin_marker.scale.z = 0.16
        rejoin_marker.color.r, rejoin_marker.color.g, rejoin_marker.color.a = 1.0, 0.8, 1.0; rejoin_marker.pose.position.x, rejoin_marker.pose.position.y = rejoin; rejoin_marker.pose.position.z = 0.08; output.markers.append(rejoin_marker)
        text = Marker(); text.header.frame_id = self.base_frame; text.header.stamp = now; text.ns = 'state'; text.id = 4; text.type = Marker.TEXT_VIEW_FACING; text.action = Marker.ADD
        text.scale.z = 0.22; text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0; text.pose.position.z = 0.55; text.text = state.value; output.markers.append(text)
        self.markers.publish(output)

    def on_timer(self) -> None:
        if not self.enabled or self.path is None or self.scan is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        preview = reactive_preview(self.speed, self.base_preview, self.preview_gain, self.preview_min,
                                   self.preview_max, self.deceleration, self.latency, self.front_extent,
                                   self.corridor_width - float(self.get_parameter('corridor_half_robot_width').value))
        pose = self._robot_pose()
        if pose is None:
            return
        points, _ = self._future_path_base(pose, preview)
        previous_state = self.machine.state
        blocking, obstacle_distance, hits, unmapped = self._blocking(pose, points, preview)
        path_clear = not blocking
        rejoin_aligned = path_clear and abs(self.lateral_error) < self.rejoin_lateral and abs(self.heading_error) < self.rejoin_heading
        state, changed = self.machine.update(now, blocking, path_clear, rejoin_aligned)
        rejoin_index = self._rejoin_index(points)
        rejoin_pose = self.path.poses[min(rejoin_index, len(self.path.poses) - 1)].pose.position
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rejoin_local = (c * (rejoin_pose.x - pose[0]) + s * (rejoin_pose.y - pose[1]),
                        -s * (rejoin_pose.x - pose[0]) + c * (rejoin_pose.y - pose[1]))
        gap_angle, gap_width, valid_gap, gap_diagnostics, gap_stats = self._gap(points)
        self._publish_gap_diagnostics(valid_gap, gap_diagnostics, gap_stats)
        if state == ReactiveState.AVOIDING:
            velocity, omega = safe_gap_command(
                gap_angle, self.ftg_vmax, self.ftg_wmax, self.steering_gain,
                self.steering_slowdown, self.wheel_limit, self.half_track)
            command = TwistStamped(); command.header.stamp = self.get_clock().now().to_msg(); command.header.frame_id = self.base_frame
            command.twist.linear.x = float(velocity); command.twist.angular.z = float(omega); self.cmd.publish(command)
        self.rejoin_limit.publish(Float32(data=float(self.rejoin_vmax if state == ReactiveState.REJOINING else 0.0)))
        self.state_pub.publish(String(data=state.value)); self.active_pub.publish(Bool(data=state != ReactiveState.TRACKING))
        self.blocking_pub.publish(Bool(data=blocking)); self.unmapped_pub.publish(Bool(data=unmapped)); self.distance_pub.publish(Float32(data=float(obstacle_distance if math.isfinite(obstacle_distance) else -1.0)))
        self.clear_pub.publish(Bool(data=path_clear)); self.preview_pub.publish(Float32(data=float(preview))); self.width_pub.publish(Float32(data=float(self.corridor_width)))
        self.rejoin_index_pub.publish(Int32(data=int(rejoin_index))); self.rejoin_distance_pub.publish(Float32(data=float(self.rejoin_offset))); self.rejoin_heading_pub.publish(Float32(data=float(self.heading_error)))
        self.target_pub.publish(Float32(data=float(gap_angle))); self.gap_pub.publish(Float32(data=float(gap_width))); self._publish_markers(points, hits, gap_angle, rejoin_local, state)
        if changed:
            self.get_logger().info(f'[Reactive] {previous_state.value} -> {state.value} blocking={blocking} distance={obstacle_distance:.2f}')
        if now - self.last_log >= 1.0:
            details = '; '.join(f'[{c.start_index}:{c.end_index}] aw={c.angular_width:.2f} r={c.representative_range:.2f} w={c.physical_width:.2f} accepted={c.accepted} {c.rejection_reason}' for c in gap_diagnostics.candidates)
            self.last_log = now; self.get_logger().info(f'[Reactive] state={state.value} v={self.speed:.2f} preview={preview:.2f} blocking={blocking} gap={gap_angle:.2f} valid={valid_gap} candidates={gap_diagnostics.candidate_gap_count} accepted={gap_diagnostics.valid_gap_count} bubble_removed={gap_diagnostics.bubble_removed_beams} details={details or "no_candidate_gaps"}')


def main(args=None) -> None:
    rclpy.init(args=args); node = ReactiveAvoidanceSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

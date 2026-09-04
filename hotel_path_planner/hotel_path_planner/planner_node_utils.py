"""ROS adapter and CSV instrumentation shared by assignment planner nodes."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path as FilePath
from typing import Dict, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from .planning_core import GridMap, PlannerResult, Pose2D, quaternion_to_yaw, yaw_to_quaternion


PLANNER_COLUMNS = [
    "method", "scenario", "trial", "execution_mode", "environment", "success", "reason", "planning_time_ms",
    "expanded_nodes", "path_length_m", "waypoints", "min_clearance_m",
    "clearance_metric", "sharp_direction_changes", "start_x", "start_y", "start_yaw",
    "goal_x", "goal_y", "goal_yaw", "terminal_yaw_error_rad", "parameters",
]


def map_qos() -> QoSProfile:
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def pose_to_core(pose: PoseStamped) -> Pose2D:
    return Pose2D(pose.pose.position.x, pose.pose.position.y, quaternion_to_yaw(pose.pose.orientation))


def result_to_path(result: PlannerResult, frame_id: str, stamp) -> Path:
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for item in result.poses:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(item.x)
        pose.pose.position.y = float(item.y)
        qx, qy, qz, qw = yaw_to_quaternion(item.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        path.poses.append(pose)
    return path


class PlannerCsvLogger:
    """Append planning records without overwriting prior experimental evidence."""

    def __init__(self, path: str) -> None:
        self.path = FilePath(path)

    def append(self, row: Dict[str, object]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        write_header = not self.path.exists() or self.path.stat().st_size == 0
        with self.path.open("a", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=PLANNER_COLUMNS, extrasaction="ignore")
            if write_header:
                writer.writeheader()
            writer.writerow(row)


class PlannerNodeBase(rclpy.node.Node):
    """Common ROS concerns; subclasses only create a planner from ``GridMap``."""

    method_name = "planner"

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("treat_unknown_as_obstacle", True)
        # 0.25 m is the enclosing circular radius of the 0.40 x 0.44 m chassis.
        self.declare_parameter("inflate_radius", 0.25)
        self.declare_parameter("traversal_cost_weight", 0.0)
        self.declare_parameter("scenario", "manual")
        self.declare_parameter("trial", 0)
        # Set to ``gazebo`` for the supplied simulator, ``robot`` for a
        # hardware run, and ``offline`` only in the benchmark scripts.
        self.declare_parameter("environment", "gazebo")
        self.declare_parameter("results_csv", "results/planner_results.csv")

        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value,
                                                self.on_map, map_qos())
        self.goal_sub = self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value,
                                                 self.on_goal, 10)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.grid: Optional[GridMap] = None
        self._map: Optional[OccupancyGrid] = None
        self.csv = PlannerCsvLogger(str(self.get_parameter("results_csv").value))

    def on_map(self, msg: OccupancyGrid) -> None:
        try:
            occupancy = np.asarray(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
            self.grid = GridMap(
                occupancy, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y,
                int(self.get_parameter("occupied_threshold").value),
                bool(self.get_parameter("treat_unknown_as_obstacle").value),
                float(self.get_parameter("inflate_radius").value),
            )
            self._map = msg
            self.get_logger().info(
                f"Map ready: {msg.info.width}x{msg.info.height}, {msg.info.resolution:.3f} m/cell"
            )
        except Exception as error:
            self.grid = None
            self.get_logger().error(f"Invalid OccupancyGrid: {error}")

    def get_start_pose(self) -> PoseStamped:
        global_frame = str(self.get_parameter("global_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        transform = self.tf_buffer.lookup_transform(global_frame, base_frame, rclpy.time.Time())
        pose = PoseStamped()
        pose.header.frame_id = global_frame
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def goal_in_global_frame(self, goal: PoseStamped) -> PoseStamped:
        global_frame = str(self.get_parameter("global_frame").value)
        if not goal.header.frame_id or goal.header.frame_id == global_frame:
            goal.header.frame_id = global_frame
            return goal
        transform = self.tf_buffer.lookup_transform(global_frame, goal.header.frame_id, rclpy.time.Time())
        return do_transform_pose(goal, transform)

    def build_planner(self):  # pragma: no cover - supplied by subclasses
        raise NotImplementedError

    def planner_parameters(self) -> Dict[str, object]:
        names = self.list_parameters([], depth=10).names
        ignored = {"scenario", "trial", "results_csv", "map_topic", "goal_topic", "path_topic"}
        return {name: self.get_parameter(name).value for name in names if name not in ignored}

    def on_goal(self, goal: PoseStamped) -> None:
        if self.grid is None:
            self.get_logger().warning("Goal ignored: map is not available yet.")
            return
        try:
            start_msg = self.get_start_pose()
            goal_msg = self.goal_in_global_frame(goal)
        except Exception as error:
            self.get_logger().error(f"Planning skipped because TF is unavailable: {error}")
            return
        start, target = pose_to_core(start_msg), pose_to_core(goal_msg)
        result = self.build_planner().plan(start, target)
        self._log_result(result, start, target)
        if not result.success:
            # Clear a previous route so either tracker stops instead of
            # continuing toward an obsolete goal after planning fails.
            empty_path = Path()
            empty_path.header.frame_id = str(self.get_parameter("global_frame").value)
            empty_path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(empty_path)
            self.get_logger().warning(f"{self.method_name} failed: {result.reason}")
            return
        path = result_to_path(result, str(self.get_parameter("global_frame").value), self.get_clock().now().to_msg())
        self.path_pub.publish(path)
        self.get_logger().info(
            f"{self.method_name}: {result.planning_time_ms:.2f} ms, {result.expanded_nodes} expanded, "
            f"{result.path_length_m:.2f} m, {len(path.poses)} poses"
        )

    def _log_result(self, result: PlannerResult, start: Pose2D, target: Pose2D) -> None:
        parameters = self.planner_parameters()
        row = {
            "method": self.method_name,
            "scenario": self.get_parameter("scenario").value,
            "trial": self.get_parameter("trial").value,
            "execution_mode": "ros_live",
            "environment": self.get_parameter("environment").value,
            "success": int(result.success),
            "reason": result.reason,
            "planning_time_ms": f"{result.planning_time_ms:.6f}",
            "expanded_nodes": result.expanded_nodes,
            "path_length_m": f"{result.path_length_m:.6f}",
            "waypoints": len(result.poses),
            "min_clearance_m": "" if result.min_clearance_m is None else f"{result.min_clearance_m:.6f}",
            "clearance_metric": "8_connected_chamfer_approx_m",
            "sharp_direction_changes": result.sharp_direction_changes,
            "start_x": f"{start.x:.6f}", "start_y": f"{start.y:.6f}", "start_yaw": f"{start.yaw:.6f}",
            "goal_x": f"{target.x:.6f}", "goal_y": f"{target.y:.6f}", "goal_yaw": f"{target.yaw:.6f}",
            "terminal_yaw_error_rad": "" if not result.poses else f"{abs((result.poses[-1].yaw - target.yaw + math.pi) % (2 * math.pi) - math.pi):.6f}",
            "parameters": json.dumps(parameters, sort_keys=True),
        }
        self.csv.append(row)

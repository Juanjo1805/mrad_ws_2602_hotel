#!/usr/bin/env python3
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path

import tf2_ros
from tf2_ros import TransformException

from tf2_geometry_msgs import do_transform_pose_stamped

from .tracking_core import Pose2D, TrackingCommand, wrap_to_pi
from .tracking_metrics import TrackingCsvLogger


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class PurePursuitNode(Node):

    def __init__(self, node_name="pure_pursuit_node"):
        super().__init__(node_name)

        # ---------- PARAMETERS ----------

        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("base_frame", "base_link")

        # control frequency
        self.declare_parameter("control_rate_hz", 40.0)

        # velocities
        self.declare_parameter("v_nominal", 0.9)
        self.declare_parameter("max_speed", 1.1)
        self.declare_parameter("max_omega", 2.0)

        # goal handling
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("slow_radius", 1.2)

        # lookahead
        self.declare_parameter("lookahead_L0", 1.1)
        self.declare_parameter("lookahead_kv", 1.3)
        self.declare_parameter("lookahead_min", 0.9)
        self.declare_parameter("lookahead_max", 3.5)

        # curvature limit
        self.declare_parameter("max_curvature", 1.6)

        self.declare_parameter("eps", 1e-6)
        self.declare_parameter("tf_timeout_sec", 0.2)

        # Optional, append-only assignment instrumentation.  These do not
        # alter the baseline Pure Pursuit control law.
        self.declare_parameter("scenario", "manual")
        self.declare_parameter("trial", 0)
        self.declare_parameter("environment", "gazebo")
        self.declare_parameter("trace_csv", "results/tracker_trace.csv")
        self.declare_parameter("results_csv", "results/tracker_results.csv")

        # ---------- READ PARAMETERS ----------

        self.path_topic = self.get_parameter("path_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.base_frame = self.get_parameter("base_frame").value

        self.rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.v_nominal = float(self.get_parameter("v_nominal").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_omega = float(self.get_parameter("max_omega").value)

        self.goal_tol = float(self.get_parameter("goal_tolerance").value)
        self.slow_radius = float(self.get_parameter("slow_radius").value)

        self.L0 = float(self.get_parameter("lookahead_L0").value)
        self.kv = float(self.get_parameter("lookahead_kv").value)
        self.Lmin = float(self.get_parameter("lookahead_min").value)
        self.Lmax = float(self.get_parameter("lookahead_max").value)

        self.max_curvature = float(self.get_parameter("max_curvature").value)

        self.eps = float(self.get_parameter("eps").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)

        self.metrics = TrackingCsvLogger(
            "pure_pursuit_pt_2602_hotel",
            str(self.get_parameter("scenario").value),
            int(self.get_parameter("trial").value),
            str(self.get_parameter("trace_csv").value),
            str(self.get_parameter("results_csv").value),
            environment=str(self.get_parameter("environment").value),
        )

        # ---------- ROS INTERFACES ----------

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, 10)

        # ---------- TF ----------

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- STATE ----------

        self.path: List[PoseStamped] = []
        self.path_frame: Optional[str] = None
        self.has_path = False

        self.last_target_index = 0
        self.prev_omega = 0.0

        dt = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(dt, self.on_timer)

        self.get_logger().info("Stable Pure Pursuit controller started")

    # -----------------------------------
    # PATH CALLBACK
    # -----------------------------------

    def on_path(self, msg: Path):

        self.path = list(msg.poses)
        self.path_frame = msg.header.frame_id
        self.has_path = len(self.path) > 0

        self.last_target_index = 0
        self.metrics.start(self.get_clock().now().nanoseconds * 1e-9, len(self.path))

        if self.has_path:
            self.get_logger().info(f"Received path with {len(self.path)} poses")
        else:
            self.metrics.finish(self.get_clock().now().nanoseconds * 1e-9, False, "empty_path")

    # -----------------------------------
    # MAIN CONTROL LOOP
    # -----------------------------------

    def on_timer(self):

        if not self.has_path:
            self.publish_stop()
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.path_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout)
            )
        except TransformException:
            self.publish_stop()
            return

        # ---------- GOAL CHECK ----------

        goal_pose = self.transform_pose(self.path[-1], tf)

        if goal_pose is None:
            self.publish_stop()
            return

        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y

        goal_dist = math.hypot(gx, gy)

        if goal_dist <= self.goal_tol:
            self.publish_stop()
            self.metrics.finish(self.get_clock().now().nanoseconds * 1e-9, True, "goal_tolerance_reached")
            self.has_path = False
            return

        # ---------- SPEED ----------

        v_cmd = self.v_nominal

        if goal_dist < self.slow_radius:
            v_cmd *= goal_dist / self.slow_radius

        # ---------- LOOKAHEAD ----------

        Ld = clamp(self.L0 + self.kv * abs(v_cmd), self.Lmin, self.Lmax)

        # ---------- TARGET ----------

        target = self.find_lookahead_target(tf, Ld)

        if target is None:
            self.publish_stop()
            return

        bx, by, idx = target
        self.last_target_index = idx

        # ---------- CURVATURE ----------

        kappa = (2.0 * by) / (Ld * Ld + self.eps)
        kappa = clamp(kappa, -self.max_curvature, self.max_curvature)

        # ---------- SPEED ADAPTATION ----------

        error_lat = abs(by)
        v_cmd = self.v_nominal / (1 + 1.5 * error_lat)
        v_cmd = clamp(v_cmd, 0.2, self.max_speed)

        # ---------- OMEGA ----------

        omega = v_cmd * kappa
        omega = clamp(omega, -self.max_omega, self.max_omega)

        # smoothing
        alpha = 0.5
        omega = alpha * omega + (1 - alpha) * self.prev_omega
        self.prev_omega = omega

        # ---------- PUBLISH ----------

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.twist.linear.x = float(v_cmd)
        cmd.twist.angular.z = float(omega)

        self.cmd_pub.publish(cmd)

        # The TF looked up above is T_base_path.  Invert it to log the robot
        # pose in path coordinates, while keeping the baseline command intact.
        robot = self.robot_pose_in_path_frame(tf)
        reference_msg = self.path[idx]
        reference = Pose2D(
            reference_msg.pose.position.x,
            reference_msg.pose.position.y,
            self.yaw_from_quaternion(reference_msg.pose.orientation),
        )
        command = TrackingCommand(
            float(v_cmd), float(omega), reference, idx, float(by),
            wrap_to_pi(robot.yaw - reference.yaw), float(goal_dist),
            saturated_v=bool(v_cmd >= self.max_speed - self.eps),
            saturated_omega=bool(abs(omega) >= self.max_omega - self.eps),
        )
        self.metrics.sample(self.get_clock().now().nanoseconds * 1e-9, robot, command)

    # -----------------------------------
    # TF
    # -----------------------------------

    def transform_pose(self, pose, tf):

        try:
            return do_transform_pose_stamped(pose, tf)
        except Exception:
            return None

    @staticmethod
    def yaw_from_quaternion(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @classmethod
    def robot_pose_in_path_frame(cls, tf) -> Pose2D:
        """Invert T_base_path for instrumentation only."""
        yaw = cls.yaw_from_quaternion(tf.transform.rotation)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        c, s = math.cos(yaw), math.sin(yaw)
        return Pose2D(-(c * tx + s * ty), s * tx - c * ty, wrap_to_pi(-yaw))

    # -----------------------------------
    # LOOKAHEAD
    # -----------------------------------

    def find_lookahead_target(self, tf, Ld):

        n = len(self.path)
        best = None

        for i in range(self.last_target_index, n):

            pose_b = self.transform_pose(self.path[i], tf)

            if pose_b is None:
                continue

            bx = pose_b.pose.position.x
            by = pose_b.pose.position.y

            d = math.hypot(bx, by)

            best = (bx, by, i)

            if d >= Ld:
                return (bx, by, i)

        return best

    # -----------------------------------
    # STOP
    # -----------------------------------

    def publish_stop(self):

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main():

    rclpy.init()

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


def main_2602(args=None):
    """Convention-named entry point while preserving ``pure_pursuit_node``."""
    rclpy.init(args=args)
    node = PurePursuitNode("pure_pursuit_pt_2602_hotel")
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


if __name__ == "__main__":
    main()

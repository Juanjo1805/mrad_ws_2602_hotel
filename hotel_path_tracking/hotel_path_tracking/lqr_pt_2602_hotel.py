"""True discrete-LQR tracker for the differential-drive Hotel robot."""

from __future__ import annotations

import math
from typing import List

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
import tf2_ros
from tf2_ros import TransformException

from .tracking_core import LQRTracker, Pose2D, TrackingCommand, wrap_to_pi
from .tracking_metrics import TrackingCsvLogger


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LQRPT2602Hotel(Node):
    """LQR tracker using map/path-frame pose recovered from TF.

    Error state is e=[e_x,e_y,e_theta] in the reference frame.  Every cycle
    derives A_d,B_d from the reference unicycle velocity and applies
    ``[v-v_r, omega-omega_r] = -K e`` with saturated TwistStamped output.
    """

    method_name = "lqr_pt_2602_hotel"

    def __init__(self) -> None:
        super().__init__(self.method_name)
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("control_rate_hz", 25.0)
        # Wheel limit is 10 rad/s * 0.05 m = 0.50 m/s in the URDF.
        self.declare_parameter("v_nominal", 0.45)
        self.declare_parameter("min_speed", 0.05)
        self.declare_parameter("max_speed", 0.50)
        self.declare_parameter("max_omega", 2.0)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("lookahead_distance", 0.25)
        self.declare_parameter("q_x", 1.0)
        self.declare_parameter("q_lateral", 6.0)
        self.declare_parameter("q_heading", 3.0)
        self.declare_parameter("r_linear", 0.8)
        self.declare_parameter("r_angular", 0.6)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("scenario", "manual")
        self.declare_parameter("trial", 0)
        self.declare_parameter("environment", "gazebo")
        self.declare_parameter("trace_csv", "results/tracker_trace.csv")
        self.declare_parameter("results_csv", "results/tracker_results.csv")

        self.path_topic = str(self.get_parameter("path_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.tf_timeout = max(0.0, float(self.get_parameter("tf_timeout_sec").value))
        self.tracker = LQRTracker(
            control_dt=1.0 / self.rate_hz,
            v_nominal=float(self.get_parameter("v_nominal").value),
            min_speed=float(self.get_parameter("min_speed").value),
            max_speed=float(self.get_parameter("max_speed").value),
            max_omega=float(self.get_parameter("max_omega").value),
            goal_tolerance=float(self.get_parameter("goal_tolerance").value),
            lookahead_distance=float(self.get_parameter("lookahead_distance").value),
            q_diagonal=(float(self.get_parameter("q_x").value), float(self.get_parameter("q_lateral").value),
                        float(self.get_parameter("q_heading").value)),
            r_diagonal=(float(self.get_parameter("r_linear").value), float(self.get_parameter("r_angular").value)),
        )
        self.logger = TrackingCsvLogger(
            self.method_name, str(self.get_parameter("scenario").value), int(self.get_parameter("trial").value),
            str(self.get_parameter("trace_csv").value), str(self.get_parameter("results_csv").value),
            environment=str(self.get_parameter("environment").value),
        )
        self.path: List[Pose2D] = []
        self.path_frame = ""
        self.has_path = False
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, 10)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.get_logger().info("Differential-drive discrete LQR tracker started.")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_path(self, message: Path) -> None:
        frame = message.header.frame_id
        self.path = [
            Pose2D(item.pose.position.x, item.pose.position.y, yaw_from_quaternion(item.pose.orientation))
            for item in message.poses
        ]
        self.path_frame = frame
        self.has_path = bool(self.path) and bool(frame)
        self.tracker.reset()
        self.logger.start(self.now_s(), len(self.path))
        if not self.has_path:
            self.publish_stop()
            self.logger.finish(self.now_s(), False, "empty_path")
            self.get_logger().warning("Empty path or empty frame received; stop published.")
        else:
            self.get_logger().info(f"Received path with {len(self.path)} poses in {frame}.")

    @staticmethod
    def robot_pose_in_path_frame(transform) -> Pose2D:
        """Invert T_base_path to recover the base pose in path coordinates."""
        yaw_t = yaw_from_quaternion(transform.transform.rotation)
        tx, ty = transform.transform.translation.x, transform.transform.translation.y
        c, s = math.cos(yaw_t), math.sin(yaw_t)
        x = -(c * tx + s * ty)
        y = s * tx - c * ty
        return Pose2D(x, y, wrap_to_pi(-yaw_t))

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

    def on_timer(self) -> None:
        if not self.has_path:
            self.publish_stop()
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.path_frame, rclpy.time.Time(), timeout=Duration(seconds=self.tf_timeout)
            )
            robot = self.robot_pose_in_path_frame(transform)
            command = self.tracker.command(self.path, robot)
        except (TransformException, ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
            self.publish_stop()
            self.get_logger().warning(f"LQR reference/TF failure; stop published: {error}")
            return
        self.publish(command)
        timestamp = self.now_s()
        self.logger.sample(timestamp, robot, command)
        if command.goal_reached:
            self.logger.finish(timestamp, True, "goal_tolerance_reached")
            # The stop command above is valid, but the completed path must
            # not keep adding zero-command samples to the same run.
            self.has_path = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LQRPT2602Hotel()
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

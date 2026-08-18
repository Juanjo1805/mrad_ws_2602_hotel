#!/usr/bin/env python3

import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


PARAMETERS = (
    ('kp', 1.1),
    ('kd', 1.5),
    ('max_steering', 2.2),
    ('max_velocity', 1.7),
    ('min_velocity', 1.33),
    ('kv', 2.1),
)


class WallFollowerControl(Node):

    def __init__(self):
        super().__init__('wall_follower_control')

        self._load_parameters()
        self._reset_state()
        self._create_ros_interfaces()
        self.get_logger().info('Wall follower control node started')

    def _load_parameters(self):
        for name, default in PARAMETERS:
            self.declare_parameter(name, default)
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.max_steering = self.get_parameter('max_steering').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.min_velocity = self.get_parameter('min_velocity').value
        self.kv = self.get_parameter('kv').value

    def _reset_state(self):
        self.prev_error = 0.0
        self.prev_time = time.time()
        self.front_yaw = 0.0
        self.diagiz = 0.0
        self.dist_min = float('inf')
        self.rb_pressed = False
        self.error = 0.0

    def _create_ros_interfaces(self):
        self.error_sub = self.create_subscription(
            Float32, '/error', self.error_callback, 10
        )
        self.dist_diagiz_sub = self.create_subscription(
            Float32, '/diagiz_dist', self.dist_diagiz_callback, 10
        )
        self.dist_min_sub = self.create_subscription(
            Twist, '/dist_min', self.front_callback, 10
        )
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_ctrl', 10)

    def error_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.prev_time

        if dt <= 0.0:
            return

        error = msg.data
        self.error = error
        steering = self._pd_steering(error, dt)
        velocity = self._adaptive_velocity(error)
        self._publish_command(velocity, steering)
        self._remember_control_state(error, current_time)

    def _pd_steering(self, error, dt):
        derivative = (error - self.prev_error) / dt
        steering = self.kp * error + self.kd * derivative + self.front_yaw
        return self._clamp(steering, -self.max_steering, self.max_steering)

    def _adaptive_velocity(self, error):
        velocity = self.max_velocity / (1 + self.kv * abs(error))
        return max(self.min_velocity, velocity)

    def _publish_command(self, velocity, steering):
        if not self.rb_pressed:
            velocity = 0.0
            steering = 0.0
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = velocity
        cmd.twist.angular.z = steering
        self.cmd_pub.publish(cmd)

    def _remember_control_state(self, error, current_time):
        self.prev_error = error
        self.prev_time = current_time

    def front_callback(self, msg):
        self.dist_min = msg.linear.x
        if (
            msg.linear.x < 2.5
            and msg.linear.y > 1.0
            and self.diagiz > self.dist_min
        ):
            self.front_yaw = (
                0.6 * msg.linear.y
                + 0.65 / (msg.linear.x * msg.linear.x)
            )
            if self.diagiz - self.dist_min < 0.4:
                self.front_yaw += 1.5 * (self.diagiz - self.dist_min)
        else:
            self.front_yaw = 0.0

    def dist_diagiz_callback(self, msg):
        self.diagiz = msg.data

    def joy_callback(self, msg):
        self.rb_pressed = (msg.buttons[5] == 1)
        if not self.rb_pressed:
            self.get_logger().info('Dead man activado')

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

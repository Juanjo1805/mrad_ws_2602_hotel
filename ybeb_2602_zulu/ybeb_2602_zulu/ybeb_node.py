"""Last RC stage: stamped command to calibrated Rosmaster PWM channels."""

import math
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node


class YbebNode(Node):
    """Convert commands to PWM and neutralize if AEB stops publishing."""

    def __init__(self):
        super().__init__('ybeb_node')
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('throttle_limit', 0.4)
        self.declare_parameter('steering_limit', 0.5)
        self.declare_parameter('throttle_center', 91.0)
        self.declare_parameter('throttle_gain', 27.0)
        self.declare_parameter('steering_left', 74.0)
        self.declare_parameter('steering_center', 103.0)
        self.declare_parameter('steering_right', 129.0)
        self.declare_parameter('aux_servo_2', 111.0)
        self.timeout = float(self.get_parameter('command_timeout').value)
        self.throttle_limit = float(self.get_parameter('throttle_limit').value)
        self.steering_limit = float(self.get_parameter('steering_limit').value)
        self.throttle_center = float(self.get_parameter('throttle_center').value)
        self.throttle_gain = float(self.get_parameter('throttle_gain').value)
        self.steering_left = float(self.get_parameter('steering_left').value)
        self.steering_center = float(self.get_parameter('steering_center').value)
        self.steering_right = float(self.get_parameter('steering_right').value)
        self.aux_servo_2 = float(self.get_parameter('aux_servo_2').value)
        values = (self.timeout, self.throttle_limit, self.steering_limit,
                  self.throttle_center, self.throttle_gain, self.steering_left,
                  self.steering_center, self.steering_right, self.aux_servo_2)
        if not all(math.isfinite(v) for v in values) or min(values[:3]) <= 0:
            raise ValueError('hardware parameters must be finite and limits positive')
        if not 0 <= self.steering_left < self.steering_center < self.steering_right <= 180:
            raise ValueError('steering_left < steering_center < steering_right required')
        if (self.throttle_center - self.throttle_gain * self.throttle_limit < 0
                or self.throttle_center + self.throttle_gain * self.throttle_limit > 180):
            raise ValueError('throttle PWM exceeds [0, 180] degrees')
        if not 0 <= self.aux_servo_2 <= 180:
            raise ValueError('aux_servo_2 must be in [0, 180] degrees')
        self.command = (0.0, 0.0)
        self.last_command = -math.inf
        # The only place the serial library is imported or constructed.
        from Rosmaster_Lib import Rosmaster
        self.robot = Rosmaster()
        try:
            self.write_pwm(0.0, 0.0)
            self.robot.create_receive_threading()
            self.create_subscription(TwistStamped, '/cmd_vel_stamped', self.on_command, 10)
            self.create_timer(0.05, self.on_timer)
        except Exception:
            self.shutdown_hardware()
            raise

    def pwm(self, throttle, steering):
        throttle_pwm = self.throttle_center + self.throttle_gain * throttle
        if steering < 0:
            steering_pwm = self.steering_center + (
                steering / self.steering_limit) * (self.steering_center - self.steering_left)
        else:
            steering_pwm = self.steering_center + (
                steering / self.steering_limit) * (self.steering_right - self.steering_center)
        return throttle_pwm, steering_pwm

    def write_pwm(self, throttle, steering):
        throttle_pwm, steering_pwm = self.pwm(throttle, steering)
        self.robot.set_pwm_servo(1, throttle_pwm)
        self.robot.set_pwm_servo(4, steering_pwm)
        # The physically used 2601 node also held channel 2 at 111 degrees.
        self.robot.set_pwm_servo(2, self.aux_servo_2)

    def on_command(self, message):
        throttle = message.twist.linear.x
        steering = message.twist.angular.z
        if not math.isfinite(throttle) or not math.isfinite(steering):
            self.command = (0.0, 0.0)
            self.last_command = -math.inf
            self.write_pwm(0.0, 0.0)
            return
        bounded = (max(-self.throttle_limit, min(self.throttle_limit, throttle)),
                   max(-self.steering_limit, min(self.steering_limit, steering)))
        if bounded != (throttle, steering):
            self.get_logger().warn(f'Hardware limit: {(throttle, steering)} -> {bounded}')
        self.command = bounded
        self.last_command = time.monotonic()

    def on_timer(self):
        command = self.command if time.monotonic() - self.last_command < self.timeout else (0.0, 0.0)
        self.write_pwm(*command)

    def shutdown_hardware(self):
        if getattr(self, 'robot', None) is not None:
            try:
                self.write_pwm(0.0, 0.0)
            finally:
                if hasattr(self.robot, 'ser'):
                    self.robot.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YbebNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.shutdown_hardware()
            finally:
                node.destroy_node()
        rclpy.try_shutdown()

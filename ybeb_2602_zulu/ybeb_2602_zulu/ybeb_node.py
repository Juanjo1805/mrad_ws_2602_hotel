"""The single RC hardware interface, retaining Rosmaster PWM calibration."""

import math
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32

from ybeb_2602_zulu.ros_support import finite_twist, read_parameter, stamp_lifetime
from ybeb_2602_zulu.safety_core import clamp, CommandWatchdog, Limits, servo_values


class YbebNode(Node):
    """Guard actuator writes independently of AEB, using a monotonic watchdog."""

    def __init__(self, robot_factory=None, **kwargs):
        """Open hardware only here; tests inject an in-memory fake explicitly."""
        super().__init__('ybeb_node', **kwargs)
        self.robot = None
        self.closed = False
        self.limits = Limits(read_parameter(self, 'throttle_limit', 0.4),
                             read_parameter(self, 'steering_limit', 0.5))
        timeout = read_parameter(self, 'command_timeout', 0.25)
        self.watchdog = CommandWatchdog(self.limits, timeout)
        self.safe_steering = read_parameter(self, 'watchdog_steering', 0.0)
        self.future_tolerance = read_parameter(self, 'future_stamp_tolerance', 0.1)
        rate = read_parameter(self, 'output_rate', 20.0)
        if not math.isfinite(self.safe_steering) or abs(self.safe_steering) > 0.5:
            raise ValueError('watchdog_steering must be within [-0.5, 0.5]')
        if not math.isfinite(rate) or not 10 <= rate <= 100:
            raise ValueError('hardware output_rate must be between 10 and 100 Hz')
        if not math.isfinite(self.future_tolerance) or not 0 <= self.future_tolerance <= 0.5:
            raise ValueError('future_stamp_tolerance must be in [0, 0.5]')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('RC hardware requires use_sim_time=false')
        try:
            if robot_factory is None:
                from Rosmaster_Lib import Rosmaster
                robot_factory = Rosmaster
            self.robot = robot_factory()
            if hasattr(self.robot, 'ser'):
                self.robot.ser.write_timeout = 0.05
            self.neutral()
            self.robot.create_receive_threading()
            self.create_subscription(TwistStamped, '/cmd_vel_stamped', self.twist_callback, 1)
            self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
            self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
            self.voltage_pub = self.create_publisher(Float32, '/voltage', 10)
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(1.0 / rate, self.actuate, clock=self.steady_clock)
            self.telemetry_timer = self.create_timer(0.1, self.telemetry, clock=self.steady_clock)
        except Exception:
            self.safe_shutdown()
            self.destroy_node()
            raise

    def twist_callback(self, message):
        """Reject nonfinite and stale input; clamp both actuator axes."""
        lifetime = stamp_lifetime(message, self.get_clock().now().nanoseconds,
                                  self.watchdog.timeout, self.future_tolerance)
        self.watchdog.update(message.twist.linear.x, message.twist.angular.z,
                             time.monotonic(), finite_twist(message), lifetime)
        # Invalid commands neutralize immediately, not only on the next timer tick.
        if self.watchdog.reason != 'ok':
            self.neutral()

    def neutral(self):
        """Write calibrated PWM 91 for throttle and configurable safe steering."""
        if self.robot is not None:
            throttle, steering = servo_values(0.0, self.safe_steering, self.limits)
            self.robot.set_pwm_servo(1, throttle)
            self.robot.set_pwm_servo(4, steering)

    def actuate(self):
        """Apply a bounded command or neutral before any telemetry processing."""
        command, reason = self.watchdog.sample(time.monotonic())
        if reason != 'ok':
            command = (0.0, clamp(self.safe_steering, self.limits.steering_limit))
        throttle, steering = servo_values(*command, self.limits)
        try:
            self.robot.set_pwm_servo(1, throttle)
            self.robot.set_pwm_servo(4, steering)
        except Exception:
            self.watchdog.update(0.0, 0.0, time.monotonic(), valid=False)
            self.neutral()
            raise

    def telemetry(self):
        """Keep the existing cached IMU, magnetic field and voltage telemetry."""
        try:
            stamp = self.get_clock().now().to_msg()
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = 'imu_link'
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (
                float(v) for v in self.robot.get_accelerometer_data())
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (
                float(v) for v in self.robot.get_gyroscope_data())
            mag = MagneticField()
            mag.header.stamp = stamp
            mag.header.frame_id = 'imu_link'
            mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z = (
                float(v) for v in self.robot.get_magnetometer_data())
            self.imu_pub.publish(imu)
            self.mag_pub.publish(mag)
            self.voltage_pub.publish(Float32(data=float(self.robot.get_battery_voltage())))
        except Exception as error:
            self.get_logger().error(f'Telemetry failure: {error}')

    def safe_shutdown(self):
        """Best-effort neutral on normal shutdown or exceptions, before closing serial."""
        if self.closed:
            return
        self.closed = True
        try:
            self.neutral()
        finally:
            if self.robot is not None and hasattr(self.robot, 'ser'):
                self.robot.ser.close()


# Keep the historical class name available to code importing it.
TWIST_CMD_NODE = YbebNode


def main(args=None):
    """Guarantee neutral writes on SIGINT/SIGTERM and normal Python exceptions."""
    rclpy.init(args=args)
    node = None
    try:
        node = YbebNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.safe_shutdown()
            finally:
                node.destroy_node()
        rclpy.try_shutdown()

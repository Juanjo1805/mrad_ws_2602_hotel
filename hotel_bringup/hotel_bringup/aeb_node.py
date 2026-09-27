"""RC AEB: selected TwistStamped and real LaserScan to guarded TwistStamped."""

from dataclasses import fields
import math
import time

from geometry_msgs.msg import TwistStamped
from hotel_bringup.safety_core import AEBConfig, AEBController
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from ybeb_2602_zulu.ros_support import finite_twist, read_parameter, stamp_lifetime
from ybeb_2602_zulu.safety_core import Limits
from ybeb_2602_zulu.scan_support import Scan


class AEBNode(Node):
    """Publish a fresh decision periodically; input callbacks never bypass AEB."""

    def __init__(self, **kwargs):
        """Configure immutable parameters and real-time safety timers."""
        super().__init__('rc_aeb', **kwargs)
        defaults = AEBConfig()
        config = AEBConfig(**{
            f.name: read_parameter(self, f.name, getattr(defaults, f.name))
            for f in fields(defaults)})
        limits = Limits(read_parameter(self, 'throttle_limit', 0.4),
                        read_parameter(self, 'steering_limit', 0.5))
        self.core = AEBController(config, limits)
        self.future_tolerance = read_parameter(self, 'future_stamp_tolerance', 0.1)
        self.scan_frame = read_parameter(self, 'scan_frame', 'laser_frame')
        self.output_frame = read_parameter(self, 'output_frame', 'base_link')
        rate = read_parameter(self, 'output_rate', 25.0)
        if not math.isfinite(rate) or not 10 <= rate <= 100:
            raise ValueError('output_rate must be between 10 and 100 Hz')
        if not math.isfinite(self.future_tolerance) or not 0 <= self.future_tolerance <= 0.5:
            raise ValueError('future_stamp_tolerance must be in [0, 0.5]')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('RC safety nodes require use_sim_time=false')
        self.last_scan_stamp = -1
        self.last_reason = None
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 1)
        self.active_pub = self.create_publisher(Bool, '/aeb/active', 1)
        self.reason_pub = self.create_publisher(String, '/aeb/reason', 1)
        self.ttc_pub = self.create_publisher(Float32, '/aeb/ttc', 1)
        self.create_subscription(TwistStamped, '/cmd_vel_mux', self.command_callback, 1)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / rate, self.publish_decision, clock=self.steady_clock)

    def command_callback(self, message):
        """Reject nonfinite commands and stale source stamps before storing them."""
        lifetime = stamp_lifetime(message, self.get_clock().now().nanoseconds,
                                  self.core.config.command_timeout, self.future_tolerance)
        self.core.watchdog.update(message.twist.linear.x, message.twist.angular.z,
                                  time.monotonic(), finite_twist(message), lifetime)

    def scan_callback(self, message):
        """Invalidate stale/repeated scans; BEST_EFFORT supports real sensor QoS."""
        lifetime = stamp_lifetime(message, self.get_clock().now().nanoseconds,
                                  self.core.config.scan_timeout, self.future_tolerance)
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        valid = message.header.frame_id == self.scan_frame and stamp > self.last_scan_stamp
        if valid and lifetime > 0:
            self.last_scan_stamp = stamp
        scan = Scan(tuple(message.ranges), message.angle_min, message.angle_increment,
                    message.angle_max, message.range_min, message.range_max)
        self.core.update_scan(scan, time.monotonic(), valid, lifetime)

    def publish_decision(self):
        """Output neutral on expiry, invalid input or a latched TTC hazard."""
        decision = self.core.evaluate(time.monotonic())
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.output_frame
        out.twist.linear.x = decision.throttle
        out.twist.angular.z = decision.steering
        self.pub.publish(out)
        self.active_pub.publish(Bool(data=decision.active))
        self.reason_pub.publish(String(data=decision.reason))
        self.ttc_pub.publish(Float32(data=decision.ttc))
        if decision.reason != self.last_reason:
            self.get_logger().info(f'AEB: {decision.reason}')
            self.last_reason = decision.reason


def main(args=None):
    """Run AEB; hardware independently neutralizes if this process exits."""
    rclpy.init(args=args)
    node = None
    try:
        node = AEBNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

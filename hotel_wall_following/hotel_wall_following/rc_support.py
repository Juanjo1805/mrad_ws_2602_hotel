"""Optional Wall/Gap source with enable lease, scan expiry and final saturation."""

from dataclasses import fields
import math
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from ybeb_2602_zulu.ros_support import read_parameter, stamp_lifetime
from ybeb_2602_zulu.safety_core import clamp
from ybeb_2602_zulu.scan_support import Scan


class EnableLease:
    """An operator must refresh enable; disabling never keeps a mux source alive."""

    def __init__(self, timeout=0.5):
        """Start disabled with a finite positive enable timeout."""
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('enable_timeout must be positive')
        self.timeout = timeout
        self.deadline = -math.inf

    def update(self, enabled, now):
        """Renew enable or revoke it immediately."""
        self.deadline = now + self.timeout if enabled else -math.inf

    def active(self, now):
        """Return false when no recent enable heartbeat exists."""
        return now < self.deadline


class ScanCommandSource(Node):
    """Publish a source only while explicitly enabled and its scan is fresh."""

    def __init__(self, mode, config_type, command_function, **kwargs):
        """Configure a single optional mode; no actuator or final-command publisher."""
        super().__init__('rc_' + mode, **kwargs)
        self.mode = mode
        self.command_function = command_function
        defaults = config_type()
        self.config = config_type(**{
            f.name: read_parameter(self, f.name, getattr(defaults, f.name))
            for f in fields(defaults)})
        self.lease = EnableLease(read_parameter(self, 'enable_timeout', 0.5))
        self.scan_timeout = read_parameter(self, 'scan_timeout', 0.3)
        if not math.isfinite(self.scan_timeout) or self.scan_timeout <= 0:
            raise ValueError('scan_timeout must be positive')
        self.scan_frame = read_parameter(self, 'scan_frame', 'laser_frame')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('RC sources require use_sim_time=false')
        self.scan = None
        self.scan_deadline = -math.inf
        self.last_stamp = -1
        self.was_active = False
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_' + self.mode, 1)
        self.create_subscription(Bool, '/' + self.mode + '/enable', self.enable_callback, 1)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.tick, clock=self.steady_clock)

    def enable_callback(self, message):
        """Accept an explicit operator heartbeat; absence expires the source."""
        self.lease.update(message.data, time.monotonic())

    def scan_callback(self, message):
        """Invalidate stale, repeated or unexpected-frame data."""
        lifetime = stamp_lifetime(message, self.get_clock().now().nanoseconds,
                                  self.scan_timeout, 0.1)
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        if message.header.frame_id != self.scan_frame or stamp <= self.last_stamp:
            lifetime = 0.0
        if lifetime > 0:
            self.last_stamp = stamp
        self.scan_deadline = time.monotonic() + lifetime
        self.scan = Scan(tuple(message.ranges), message.angle_min, message.angle_increment,
                         message.angle_max, message.range_min, message.range_max)

    def tick(self):
        """Send one neutral on deactivation, then stay silent so the mux can expire."""
        now = time.monotonic()
        active = self.lease.active(now) and self.scan is not None and now < self.scan_deadline
        if not active and not self.was_active:
            return
        x, z = self.command_function(self.scan, self.config) if active else (0.0, 0.0)
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.twist.linear.x = clamp(x, 0.4)
        message.twist.angular.z = clamp(z, 0.5)
        self.pub.publish(message)
        self.was_active = active


def spin_source(node_class, args=None):
    """Run an optional prototype; final safety remains downstream of the mux."""
    rclpy.init(args=args)
    node = None
    try:
        node = node_class()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

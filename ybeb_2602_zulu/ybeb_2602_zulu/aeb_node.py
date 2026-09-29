"""Simple TTC brake on the Pi: /cmd_vel_mux + /scan -> /cmd_vel_stamped."""

import copy
import math
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def minimum_ttc(scan, throttle, speed_bound, clearance):
    """Return (TTC, usable); NaN, infinity and out-of-range rays are ignored."""
    if (not scan.ranges or not all(math.isfinite(v) for v in
                                  (scan.angle_min, scan.angle_increment,
                                   scan.range_min, scan.range_max))
            or scan.angle_increment == 0 or scan.range_min < 0
            or scan.range_max <= scan.range_min):
        return math.inf, False
    direction = 1 if throttle >= 0 else -1
    best = math.inf
    usable = False
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or distance < scan.range_min or distance > scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        projection = direction * math.cos(angle)
        if projection <= 0.5:
            continue
        usable = True
        closing_speed = speed_bound * projection
        if closing_speed > 0:
            best = min(best, max(0.0, distance - clearance) / closing_speed)
    return best, usable


class AEBNode(Node):
    """Pass fresh commands unchanged unless TTC or input freshness requires neutral."""

    def __init__(self):
        super().__init__('aeb_node')
        self.declare_parameter('ttc_threshold', 0.45)
        self.declare_parameter('clearance_m', 0.45)
        self.declare_parameter('speed_bound_mps', 2.0)
        self.declare_parameter('scan_timeout', 0.4)
        self.declare_parameter('command_timeout', 0.3)
        self.ttc_threshold = float(self.get_parameter('ttc_threshold').value)
        self.clearance = float(self.get_parameter('clearance_m').value)
        self.speed_bound = float(self.get_parameter('speed_bound_mps').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        values = (self.ttc_threshold, self.clearance, self.speed_bound,
                  self.scan_timeout, self.command_timeout)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('AEB parameters must be finite and positive')
        self.command = None
        self.scan = None
        self.last_command = -math.inf
        self.last_scan = -math.inf
        self.last_reason = None
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)
        self.create_subscription(TwistStamped, '/cmd_vel_mux', self.on_command, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.publish_decision)

    def on_command(self, message):
        x, z = message.twist.linear.x, message.twist.angular.z
        if not math.isfinite(x) or not math.isfinite(z):
            self.command = None
            self.last_command = -math.inf
            return
        self.command = message
        self.last_command = time.monotonic()

    def on_scan(self, message):
        self.scan = message
        self.last_scan = time.monotonic()

    def decision(self, now):
        """Return exact x,z pass-through on clear path and neutral on faults."""
        if self.command is None or now - self.last_command >= self.command_timeout:
            return 0.0, 0.0, 'command_timeout'
        if self.scan is None or now - self.last_scan >= self.scan_timeout:
            return 0.0, 0.0, 'scan_timeout'
        x, z = self.command.twist.linear.x, self.command.twist.angular.z
        ttc, usable = minimum_ttc(self.scan, x, self.speed_bound, self.clearance)
        if not usable:
            return 0.0, 0.0, 'invalid_scan'
        if x != 0.0 and ttc <= self.ttc_threshold:
            return 0.0, z, 'ttc'
        return x, z, 'clear'

    def publish_decision(self):
        x, z, reason = self.decision(time.monotonic())
        output = TwistStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'base_link'
        if reason == 'clear':
            output.twist = copy.deepcopy(self.command.twist)
        output.twist.linear.x = x
        output.twist.angular.z = z
        self.publisher.publish(output)
        if reason != self.last_reason:
            self.get_logger().info(f'AEB: {reason}')
            self.last_reason = reason


def main(args=None):
    rclpy.init(args=args)
    node = AEBNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

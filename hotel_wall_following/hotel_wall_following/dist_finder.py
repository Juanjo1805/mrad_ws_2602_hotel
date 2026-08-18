#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


PARAMETERS = (
    ('theta_deg', 48.0),
    ('lookahead_dist', 1.5),
    ('desired_distance', 0.72),
)


class DistFinder(Node):

    def __init__(self):
        super().__init__('dist_finder')

        self._load_parameters()
        self._create_ros_interfaces()
        self.get_logger().info('dist_finder started')

    def _load_parameters(self):
        for name, default in PARAMETERS:
            self.declare_parameter(name, default)
        self.theta = math.radians(
            self.get_parameter('theta_deg').value
        )
        self.L = self.get_parameter('lookahead_dist').value
        self.desired_distance = self.get_parameter('desired_distance').value

    def _create_ros_interfaces(self):
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.error_pub = self.create_publisher(Float32, '/error', 10)
        self.dist_diagiz_pub = self.create_publisher(Float32, '/diagiz_dist', 10)

    def getRange(self, data, angle):
        index = self._range_index(data, angle)
        r = data.ranges[index]
        if math.isinf(r) or math.isnan(r):
            return data.range_max
        return r

    def scan_callback(self, data):
        a, b, left_upper = self._sample_wall_ranges(data)

        if a == 0.0 or b == 0.0:
            return

        alpha = self._wall_angle(a, b)
        error = self._projected_error(alpha, b)
        self._publish_error(error, left_upper)
        self.get_logger().info(
            f'Error: {error: .3f}, Alpha: {math.degrees(alpha): .3f}'
        )

    def _range_index(self, data, angle):
        index = int((angle - data.angle_min) / data.angle_increment)
        return max(0, min(index, len(data.ranges) - 1))

    def _sample_wall_ranges(self, data):
        b = self.getRange(data, -math.pi / 2)
        a = self.getRange(data, -math.pi / 2 + self.theta)
        left_upper = self.getRange(data, math.pi / 8)
        return a, b, left_upper

    def _wall_angle(self, a, b):
        return math.atan2(
            a * math.cos(self.theta) - b,
            a * math.sin(self.theta)
        )

    def _projected_error(self, alpha, b):
        y = b * math.cos(alpha)
        y_future = y + self.L * math.sin(alpha)
        return self.desired_distance - y_future

    def _publish_error(self, error, left_upper):
        msg = Float32()
        msg.data = float(error)
        self.error_pub.publish(msg)
        self.dist_diagiz_pub.publish(Float32(data=left_upper))


def main(args=None):
    rclpy.init(args=args)
    node = DistFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

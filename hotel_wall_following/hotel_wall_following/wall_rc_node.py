"""Right-wall follower for the RC car: /scan -> /cmd_vel_wall."""

import math
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def wall_error(scan, desired_distance, theta_deg, lookahead_dist,
               front_angle_deg=180.0):
    """Reuse the Hotel right-wall geometry; return None for unusable rays."""
    if (not scan.ranges or not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_increment) or scan.angle_increment == 0
            or not math.isfinite(scan.range_min) or scan.range_max <= scan.range_min):
        return None

    def sample(angle):
        # Accept both [-pi, pi] and [0, 2*pi] LaserScan angle conventions.
        for turns in (0, 1, -1):
            target = angle + turns * 2 * math.pi
            index = round((target - scan.angle_min) / scan.angle_increment)
            if 0 <= index < len(scan.ranges):
                actual = scan.angle_min + index * scan.angle_increment
                value = scan.ranges[index]
                if (abs(actual - target) <= 1.5 * abs(scan.angle_increment)
                        and math.isfinite(value)
                        and scan.range_min <= value <= scan.range_max):
                    return value
        return None

    theta = math.radians(theta_deg)
    # The Pi AEB confirms that 180 degrees in /scan points forward on this car.
    # Measure right and forward-right relative to that physical direction.
    right = math.radians(front_angle_deg) - math.pi / 2
    b = sample(right)
    a = sample(right + theta)
    if a is None or b is None:
        return None

    # Same equations as the original dist_finder.py (right wall).
    alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
    future_distance = b * math.cos(alpha) + lookahead_dist * math.sin(alpha)
    return desired_distance - future_distance


def wall_command(error, previous_error, dt, kp, kd, max_velocity,
                 min_velocity, kv, max_steering, steering_sign):
    """Original Hotel PD/adaptive-speed law, bounded to RC command units."""
    derivative = 0.0 if previous_error is None or dt <= 0 else (error - previous_error) / dt
    steering = max(-max_steering, min(max_steering,
                                    steering_sign * (kp * error + kd * derivative)))
    velocity = max(min_velocity, max_velocity / (1 + kv * abs(error)))
    return velocity, steering


class WallRC(Node):
    def __init__(self):
        super().__init__('wall_rc')
        defaults = {
            'desired_distance': 0.72,
            'front_angle_deg': 180.0,
            'theta_deg': 48.0,
            'lookahead_dist': 1.5,
            'kp': 1.5,
            'kd': 0.48,
            'kv': 2.1,
            'max_velocity': 0.30,
            'min_velocity': 0.25,
            'max_steering': 0.5,
            'steering_sign': 1.0,
        }
        self.params = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            self.params[name] = float(self.get_parameter(name).value)
        p = self.params
        if (not all(math.isfinite(v) for v in p.values())
                or not 0 < p['theta_deg'] < 90
                or min(p['desired_distance'], p['lookahead_dist'], p['max_velocity'],
                       p['min_velocity'], p['max_steering']) <= 0
                or not 0 <= p['kd'] or not 0 <= p['kp'] or not 0 <= p['kv']
                or not p['min_velocity'] <= p['max_velocity'] <= 0.4
                or p['max_steering'] > 0.5 or p['steering_sign'] not in (-1.0, 1.0)):
            raise ValueError('Invalid Wall RC parameters; check RC limits and gains')

        self.previous_error = None
        self.previous_time = None
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel_wall', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback,
                                 qos_profile_sensor_data)
        self.get_logger().info('Following RIGHT wall from /scan; publishing /cmd_vel_wall')

    def scan_callback(self, scan):
        now = time.monotonic()
        p = self.params
        error = wall_error(scan, p['desired_distance'], p['theta_deg'],
                           p['lookahead_dist'], p['front_angle_deg'])
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        if error is None:
            self.previous_error = None
            self.previous_time = None
            # A missing wall must not keep the previous moving command alive.
        else:
            dt = 0.0 if self.previous_time is None else now - self.previous_time
            velocity, steering = wall_command(
                error, self.previous_error, dt, p['kp'], p['kd'],
                p['max_velocity'], p['min_velocity'], p['kv'], p['max_steering'],
                p['steering_sign'])
            cmd.twist.linear.x = velocity
            cmd.twist.angular.z = steering
            self.previous_error = error
            self.previous_time = now
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WallRC()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

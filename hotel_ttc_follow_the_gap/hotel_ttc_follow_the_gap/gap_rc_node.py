"""Simple RC Follow the Gap on the PC: /scan -> /cmd_vel_gap."""

import math
from bisect import bisect_left, bisect_right

from geometry_msgs.msg import TwistStamped

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


def select_gap(scan, front_angle_deg, fov_deg, min_clearance,
               bubble_radius, bubble_trigger_distance, min_gap_width_deg):
    """Return a gap-center angle relative to the car front, or None."""
    metadata = (scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max)
    if (not scan.ranges or not all(math.isfinite(v) for v in metadata)
            or scan.angle_increment == 0 or scan.range_min < 0
            or scan.range_max <= scan.range_min):
        return None

    front = math.radians(front_angle_deg)
    half_fov = math.radians(fov_deg) / 2
    step = abs(scan.angle_increment)
    rays = []
    finite_count = 0
    for index, distance in enumerate(scan.ranges):
        raw_angle = scan.angle_min + index * scan.angle_increment
        angle = math.atan2(math.sin(raw_angle - front),
                           math.cos(raw_angle - front))
        if abs(angle) > half_fov:
            continue
        valid = (math.isfinite(distance)
                 and scan.range_min <= distance <= scan.range_max)
        if valid:
            finite_count += 1
            depth = distance
        elif math.isinf(distance) and distance > 0:
            depth = scan.range_max  # No return: open up to the sensor range.
        else:
            depth = 0.0  # NaN or malformed return: blocked.
        rays.append((angle, depth, valid))
    rays.sort(key=lambda ray: ray[0])
    if (finite_count == 0 or len(rays) < 2
            or min(abs(angle) for angle, _, _ in rays) > 1.5 * step):
        return None

    # Expand nearby obstacles, as in the original Hotel gap finder.
    angles = [angle for angle, _, _ in rays]
    bubble_blocked = [False] * len(rays)
    for obstacle_angle, depth, valid in rays:
        if not valid or depth >= bubble_trigger_distance:
            continue
        radius = math.atan2(bubble_radius, max(depth, 1e-6))
        start = bisect_left(angles, obstacle_angle - radius)
        stop = bisect_right(angles, obstacle_angle + radius)
        for index in range(start, stop):
            bubble_blocked[index] = True

    gaps = []
    current = []
    previous_angle = None
    for index, (angle, depth, _) in enumerate(rays):
        continuous = (previous_angle is None
                      or angle - previous_angle <= 1.5 * step)
        if depth >= min_clearance and not bubble_blocked[index]:
            if not continuous and current:
                gaps.append(current)
                current = []
            current.append((angle, depth))
        elif current:
            gaps.append(current)
            current = []
        previous_angle = angle
    if current:
        gaps.append(current)

    candidates = []
    minimum_width = math.radians(min_gap_width_deg)
    for gap in gaps:
        width = gap[-1][0] - gap[0][0]
        if width < minimum_width:
            continue
        center = (gap[0][0] + gap[-1][0]) / 2
        mean_depth = sum(depth for _, depth in gap) / len(gap)
        # Hotel scoring: prefer a wide, deep gap; break close ties forward.
        score = (0.4 * width / math.radians(fov_deg)
                 + 0.4 * mean_depth / scan.range_max
                 - 0.05 * abs(center) / half_fov)
        candidates.append((score, -abs(center), center))
    return max(candidates)[2] if candidates else None


def gap_command(angle, steering_gain, steering_sign, angle_deadband,
                max_steering, max_velocity, min_velocity, steering_slowdown):
    """Use the original softened steering and slow down in turns."""
    angle = 0.0 if abs(angle) < angle_deadband else angle
    raw = steering_gain * angle
    steering = steering_sign * max_steering * raw / (1.0 + abs(raw))
    throttle = max(min_velocity,
                   max_velocity / (1.0 + steering_slowdown * abs(steering)))
    return throttle, steering


class GapRC(Node):
    """Publish one bounded RC command per fresh LaserScan."""

    def __init__(self):
        """Read Gap tuning without starting any sensor or hardware node."""
        super().__init__('gap_rc')
        defaults = {
            'front_angle_deg': 180.0,
            'fov_deg': 120.0,
            'min_clearance': 0.75,
            'bubble_radius': 0.35,
            'bubble_trigger_distance': 1.0,
            'min_gap_width_deg': 12.0,
            'smooth_alpha': 0.2,
            'steering_gain': 1.5,
            'steering_sign': 1.0,
            'angle_deadband': 0.05,
            'max_steering': 0.5,
            'max_velocity': 0.30,
            'min_velocity': 0.25,
            'steering_slowdown': 1.5,
        }
        self.params = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            self.params[name] = float(self.get_parameter(name).value)
        p = self.params
        if (not all(math.isfinite(v) for v in p.values())
                or not 0 < p['fov_deg'] <= 180
                or not 0 < p['min_gap_width_deg'] < p['fov_deg']
                or min(p['min_clearance'], p['bubble_radius'],
                       p['bubble_trigger_distance'], p['steering_gain'],
                       p['max_steering'], p['max_velocity'],
                       p['min_velocity']) <= 0
                or not 0 <= p['smooth_alpha'] < 1
                or not 0 <= p['angle_deadband'] <
                math.radians(p['fov_deg']) / 2
                or p['steering_slowdown'] < 0
                or p['steering_sign'] not in (-1.0, 1.0)
                or p['max_steering'] > 0.5
                or not p['min_velocity'] <= p['max_velocity'] <= 0.4):
            raise ValueError('Invalid Gap RC parameters; check RC limits')
        self.previous_angle = None
        self.publisher = self.create_publisher(
            TwistStamped, '/cmd_vel_gap', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback,
                                 qos_profile_sensor_data)
        self.get_logger().info('Follow the Gap uses front=180 deg on /scan')

    def scan_callback(self, scan):
        """Send neutral with no gap; otherwise follow its center."""
        p = self.params
        target = select_gap(scan, p['front_angle_deg'], p['fov_deg'],
                            p['min_clearance'], p['bubble_radius'],
                            p['bubble_trigger_distance'],
                            p['min_gap_width_deg'])
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        if target is None:
            self.previous_angle = None
        else:
            if self.previous_angle is not None:
                target = (p['smooth_alpha'] * self.previous_angle
                          + (1.0 - p['smooth_alpha']) * target)
            self.previous_angle = target
            throttle, steering = gap_command(
                target, p['steering_gain'], p['steering_sign'],
                p['angle_deadband'], p['max_steering'], p['max_velocity'],
                p['min_velocity'], p['steering_slowdown'])
            cmd.twist.linear.x = throttle
            cmd.twist.angular.z = steering
        self.publisher.publish(cmd)


def main(args=None):
    """Start the standalone PC Gap source."""
    rclpy.init(args=args)
    node = None
    try:
        node = GapRC()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

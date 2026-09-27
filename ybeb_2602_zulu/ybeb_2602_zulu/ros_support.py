"""Small shared ROS boundary checks; no hardware imports or side effects."""

import math

from rcl_interfaces.msg import ParameterDescriptor


def read_parameter(node, name, default):
    """Declare immutable safety parameters, configured before startup."""
    return node.declare_parameter(
        name, default, ParameterDescriptor(read_only=True)).value


def stamp_lifetime(message, ros_now_ns, timeout, future_tolerance):
    """Bound usable lifetime by source age as well as monotonic reception age."""
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        return 0.0
    source = stamp.sec * 1000000000 + stamp.nanosec
    if source == 0:
        return 0.0
    age = (ros_now_ns - source) / 1e9
    if age < -future_tolerance:
        return 0.0
    return max(0.0, timeout - max(0.0, age))


def finite_twist(message):
    """Validate every Twist component, even fields unused by this vehicle."""
    t = message.twist
    return all(math.isfinite(v) for v in (
        t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z))

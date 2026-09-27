"""Minimum Wall/Gap acceptance: geometry, hard limits and relinquishing the mux."""

import math
import random
import time

from geometry_msgs.msg import TwistStamped
from hotel_ttc_follow_the_gap.rc_core import gap_command, GapConfig
from hotel_ttc_follow_the_gap.rc_node import GapNode
from hotel_wall_following.rc_core import wall_command, WallConfig
from hotel_wall_following.rc_node import WallNode
from hotel_wall_following.rc_support import EnableLease
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from ybeb_2602_zulu.scan_support import Scan


def reactive_config(mode='wall', **kwargs):
    """Preserve acceptance inputs while selecting the migrated owner package."""
    return (WallConfig if mode == 'wall' else GapConfig)(**kwargs)


def reactive_command(scan, config):
    """Exercise the actual implementation in each existing algorithm package."""
    return (wall_command if isinstance(config, WallConfig) else gap_command)(scan, config)


def make_scan(ranges):
    """Build a full-angle synthetic scan."""
    return Scan(tuple(ranges), -math.pi, math.pi / 180, math.pi, 0.1, 12.0)


def test_right_wall_geometry():
    """Parallel right wall at the requested distance produces zero correction."""
    ranges = [4.0] * 361
    for angle in range(-90, 0):
        ranges[angle + 180] = 0.72 / -math.sin(math.radians(angle))
    assert reactive_command(make_scan(ranges), reactive_config()) == pytest.approx((0.08, 0.0))
    ranges[90] = math.nan
    assert reactive_command(make_scan(ranges), reactive_config()) == (0.0, 0.0)


def test_gap_center_no_gap_and_avoidance():
    """A clear gap goes straight, no gap stops, an offset gap produces bounded steering."""
    c = reactive_config(mode='gap')
    assert reactive_command(make_scan([4.0] * 361), c) == pytest.approx((0.08, 0.0))
    assert reactive_command(make_scan([0.2] * 361), c) == (0.0, 0.0)
    values = [0.2 if -60 <= i - 180 <= 5 else 4.0 for i in range(361)]
    x, z = reactive_command(make_scan(values), c)
    assert x == 0.08 and 0 < z <= 0.25


@pytest.mark.parametrize('mode', ['wall', 'gap'])
def test_reactive_outputs_always_obey_rc_contract(mode):
    """Random scans and extreme gains cannot recreate the historical +/-1.4 path."""
    rng = random.Random(19)
    c = reactive_config(mode=mode, throttle=0.4, steering_limit=0.5, steering_gain=1e5)
    for _ in range(100):
        values = [rng.choice([math.nan, math.inf, -1.0, 0.0, rng.uniform(0.1, 15.0)])
                  for _ in range(361)]
        x, z = reactive_command(make_scan(values), c)
        assert math.isfinite(x) and math.isfinite(z)
        assert abs(x) <= 0.4 and abs(z) <= 0.5
    with pytest.raises(ValueError):
        reactive_config(steering_limit=1.4)


def test_enable_lease_expires():
    """An enable heartbeat is required, and explicit false disables immediately."""
    lease = EnableLease()
    assert not lease.active(0.0)
    lease.update(True, 0.0)
    assert lease.active(0.49) and not lease.active(0.5)
    lease.update(True, 1.0)
    lease.update(False, 1.1)
    assert not lease.active(1.1)


@pytest.mark.parametrize('mode', ['wall', 'gap'])
def test_ros_inactive_sources_stop_publishing(mode, monkeypatch, tmp_path):
    """Actual ROS publishers remain silent when disabled or their enable lease expires."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '188')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    context = Context()
    rclpy.init(context=context, domain_id=188)
    executor = SingleThreadedExecutor(context=context)
    behavior = (WallNode if mode == 'wall' else GapNode)(context=context)
    probe = Node('reactive_test_probe', context=context)
    executor.add_node(behavior)
    executor.add_node(probe)
    messages = []
    probe.create_subscription(TwistStamped, '/cmd_vel_' + mode, messages.append, 1)
    scans = probe.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    enable = probe.create_publisher(Bool, '/' + mode + '/enable', 1)

    def pump(duration, enabled=None):
        end, next_input = time.monotonic() + duration, 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_input:
                next_input = time.monotonic() + 0.04
                message = LaserScan()
                message.header.stamp = probe.get_clock().now().to_msg()
                message.header.frame_id = 'laser_frame'
                message.angle_min, message.angle_max = -math.pi, math.pi
                message.angle_increment = math.pi / 180
                message.range_min, message.range_max = 0.1, 12.0
                message.ranges = [4.0] * 361
                scans.publish(message)
                if enabled is not None:
                    enable.publish(Bool(data=enabled))
            executor.spin_once(timeout_sec=0.005)

    try:
        pump(0.25)
        assert not messages
        pump(0.4, True)
        assert messages and messages[-1].twist.linear.x == 0.08
        pump(0.15, False)
        assert messages[-1].twist.linear.x == 0.0
        count = len(messages)
        pump(0.6, False)
        assert len(messages) == count
        pump(0.3, True)
        assert messages[-1].twist.linear.x == 0.08
        pump(0.65)
        assert messages[-1].twist.linear.x == 0.0
        count = len(messages)
        pump(0.3)
        assert len(messages) == count
    finally:
        executor.shutdown()
        behavior.destroy_node()
        probe.destroy_node()
        context.try_shutdown()

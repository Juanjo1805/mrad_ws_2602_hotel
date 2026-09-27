"""Exercise installed mux/AEB/teleop on loopback with an in-memory Rosmaster."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String
import yaml

from ybeb_2602_zulu.ybeb_node import YbebNode


class FakeRobot:
    """Record PWM in memory; never import a serial or Rosmaster driver."""

    def __init__(self):
        """Initialize the virtual actuator register."""
        self.pwm = {}
        self.writes = []

    def set_pwm_servo(self, channel, value):
        """Reject invalid values just as the physical boundary must."""
        assert math.isfinite(value)
        self.pwm[channel] = value
        self.writes.append((time.monotonic(), channel, value))

    def create_receive_threading(self):
        """Provide the hardware interface without creating a serial thread."""

    def get_accelerometer_data(self):
        """Return stationary synthetic telemetry."""
        return (0.0, 0.0, 0.0)

    get_gyroscope_data = get_accelerometer_data
    get_magnetometer_data = get_accelerometer_data

    def get_battery_voltage(self):
        """Return synthetic voltage."""
        return 12.0


def test_installed_ros_pipeline(monkeypatch, tmp_path):
    """Verify real ROS types, arbitration, timers, QoS and fake hardware PWM."""
    # A separate localhost-only domain prevents contact with the RC network.
    monkeypatch.setenv('ROS_DOMAIN_ID', '187')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    share = Path(get_package_share_directory('hotel_bringup'))
    context = Context()
    rclpy.init(context=context, domain_id=187)
    executor = SingleThreadedExecutor(context=context)
    probe = Node('rc_test_probe', context=context)
    robot = FakeRobot()
    hardware = YbebNode(robot_factory=lambda: robot, context=context)
    executor.add_node(probe)
    executor.add_node(hardware)
    processes = []
    log_files = []

    def start(args, name):
        stream = (tmp_path / (name + '.log')).open('w')
        log_files.append(stream)
        proc = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT,
                                env=os.environ.copy(), start_new_session=True)
        processes.append(proc)
        return proc

    def stop(proc):
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=3)

    outputs = []
    reasons = []
    joy_outputs = []
    probe.create_subscription(TwistStamped, '/cmd_vel_stamped', outputs.append, 1)
    probe.create_subscription(String, '/aeb/reason', lambda m: reasons.append(m.data), 1)
    probe.create_subscription(TwistStamped, '/cmd_vel_joy', joy_outputs.append, 1)
    scan_pub = probe.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    publishers = {name: probe.create_publisher(TwistStamped, '/cmd_vel_' + name, 1)
                  for name in ('joy', 'wall', 'gap', 'nav')}
    joy_pub = probe.create_publisher(Joy, '/joy', 1)
    state = {'commands': {'joy': (0.1, 0.2)}, 'scan': True, 'obstacles': [],
             'range': 4.0, 'frame': 'laser_frame', 'stamp_age': 0.0, 'joy': None}

    def pump(duration):
        end = time.monotonic() + duration
        next_input = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_input:
                next_input = time.monotonic() + 0.04
                now = probe.get_clock().now()
                for name, (x, z) in state['commands'].items():
                    message = TwistStamped()
                    message.header.stamp = (now - rclpy.duration.Duration(
                        seconds=state['stamp_age'])).to_msg()
                    message.header.frame_id = 'base_link'
                    message.twist.linear.x, message.twist.angular.z = float(x), float(z)
                    publishers[name].publish(message)
                if state['joy'] is not None:
                    message = Joy()
                    message.header.stamp = now.to_msg()
                    message.axes = [0.0, 1.0, 0.0, -1.0, 0.0, 0.0]
                    message.buttons = [0, 0, 0, 0, state['joy'], 0, 0, 0, 0, 0, 0]
                    joy_pub.publish(message)
                if state['scan']:
                    scan = LaserScan()
                    scan.header.stamp = now.to_msg()
                    scan.header.frame_id = state['frame']
                    scan.angle_min, scan.angle_max = -math.pi, math.pi
                    scan.angle_increment = math.pi / 180
                    scan.range_min, scan.range_max = 0.1, 12.0
                    values = [state['range']] * 361
                    for angle, distance in state['obstacles']:
                        values[angle + 180] = distance
                    scan.ranges = values
                    scan_pub.publish(scan)
            executor.spin_once(timeout_sec=0.005)

    def check(x, z, reason, duration=0.22):
        pump(duration)
        assert outputs and reasons, 'AEB did not publish periodically'
        assert reasons[-1] == reason, (reason, reasons[-10:])
        assert outputs[-1].twist.linear.x == pytest.approx(x)
        assert outputs[-1].twist.angular.z == pytest.approx(z)
        assert robot.pwm[1] == pytest.approx(91.0 + 27.0 * x)
        assert robot.pwm[4] == pytest.approx(127.5 + 105.0 * z)

    try:
        overrides = {}
        for mode, package, gain in (('wall', 'hotel_wall_following', 0.31),
                                    ('gap', 'hotel_ttc_follow_the_gap', 0.42)):
            source = Path(get_package_share_directory(package)) / 'config' / (mode + '_rc.yaml')
            config = yaml.safe_load(source.read_text())
            config['rc_' + mode]['ros__parameters']['steering_gain'] = gain
            overrides[mode] = tmp_path / (mode + '_override.yaml')
            overrides[mode].write_text(yaml.safe_dump(config))
        bringup = start([
            'ros2', 'launch', 'hotel_bringup', 'rc_pc_bringup.launch.py',
            'start_joystick:=false', 'start_wall:=true', 'start_gap:=true',
            'calibration_confirmed:=true',
            'wall_config:=' + str(overrides['wall']),
            'gap_config:=' + str(overrides['gap']),
        ], 'bringup')
        pump(6.0)
        assert bringup.poll() is None, (tmp_path / 'bringup.log').read_text()
        check(0.1, 0.2, 'clear')

        # Inspect all requested topics via both graph API and the actual CLI.
        graph = dict(probe.get_topic_names_and_types())
        for topic in ('/cmd_vel_joy', '/cmd_vel_mux', '/cmd_vel_stamped', '/scan'):
            expected = ('sensor_msgs/msg/LaserScan' if topic == '/scan'
                        else 'geometry_msgs/msg/TwistStamped')
            assert graph[topic] == [expected]
            result = subprocess.run(['ros2', 'topic', 'info', topic, '--no-daemon',
                                     '--spin-time', '5'],
                                    capture_output=True, text=True, timeout=10)
            assert result.returncode == 0, result.stderr
            assert expected in result.stdout
            print(f'ros2 topic info {topic} --no-daemon\n{result.stdout}')
        assert probe.count_publishers('/cmd_vel_stamped') == 1
        assert probe.count_publishers('/cmd_vel_mux') == 1
        # Distinct include arguments must load each source's own YAML.
        for mode, expected_gain in (('wall', 0.31), ('gap', 0.42)):
            client = AsyncParameterClient(probe, '/rc_' + mode)
            future = client.get_parameters(['steering_gain'])
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                pump(0.05)
            assert future.done(), 'Reactive parameter service did not respond'
            assert future.result().values[0].double_value == pytest.approx(expected_gain)
        scan_qos = probe.get_subscriptions_info_by_topic('/scan')
        assert any(info.node_name == 'rc_aeb' and info.qos_profile.reliability ==
                   qos_profile_sensor_data.reliability for info in scan_qos)
        check(0.1, 0.2, 'clear')
        state['obstacles'] = [(0, 1.0)]
        check(0.0, 0.2, 'collision')
        state['obstacles'] = [(0, 1.55)]
        check(0.0, 0.2, 'collision', 0.4)
        state['obstacles'] = []
        check(0.1, 0.2, 'clear', 0.55)
        state['obstacles'] = [(90, 0.2), (-90, 0.2)]
        check(0.1, 0.2, 'clear')
        for value in (math.inf, math.nan):
            state['obstacles'] = [(0, value)]
            check(0.1, 0.2, 'clear')
            state['obstacles'], state['range'] = [], value
            check(0.0, 0.0, 'insufficient_scan_returns')
            state['range'] = 4.0
            check(0.1, 0.2, 'clear')
        state['scan'] = False
        check(0.0, 0.0, 'scan_timeout', 0.5)
        state['scan'] = True
        check(0.1, 0.2, 'clear')
        state['commands'] = {}
        check(0.0, 0.0, 'command_timeout', 0.5)
        state['commands'] = {'joy': (8.0, -8.0)}
        check(0.4, -0.5, 'clear')
        for pair in ((math.nan, 0.1), (0.1, math.inf)):
            state['commands'] = {'joy': pair}
            check(0.0, 0.0, 'invalid_command')
        state['commands'] = {'joy': (0.1, 0.2)}
        state['stamp_age'] = 2.0
        check(0.0, 0.0, 'invalid_command')
        state['stamp_age'], state['frame'] = 0.0, 'incorrect_frame'
        check(0.0, 0.0, 'invalid_scan')
        state['frame'] = 'laser_frame'
        check(0.1, 0.2, 'clear')

        # Highest priority wins; inactive sources actually relinquish after 0.5 s.
        state['commands'] = {'nav': (0.04, 0.04), 'wall': (0.05, 0.05),
                             'gap': (0.06, 0.06), 'joy': (0.07, 0.07)}
        check(0.07, 0.07, 'clear')
        for removed, value in (('joy', 0.06), ('gap', 0.05), ('wall', 0.04)):
            del state['commands'][removed]
            check(value, value, 'clear', 0.75)

        # Run the installed teleop executable with the shipped RC YAML.
        state['commands'] = {}
        teleop = start([
            'ros2', 'run', 'teleop_twist_joy', 'teleop_node', '--ros-args',
            '--params-file', str(share / 'config' / 'joystick_rc.yaml'),
            '-r', '__node:=teleop_node', '-r', 'cmd_vel:=/cmd_vel_joy',
        ], 'teleop')
        state['joy'] = 1
        check(0.1, -0.25, 'clear', 5.0)
        assert joy_outputs[-1].header.stamp.sec > 0
        state['joy'] = 0
        pump(0.15)
        assert joy_outputs[-1].twist.linear.x == 0.0
        assert joy_outputs[-1].twist.angular.z == 0.0
        pump(0.55)
        assert robot.pwm[1] == 91.0
        stop(teleop)
        state['joy'] = None

        # Killing both upstream processes leaves the independent ybeb timer alive.
        state['commands'] = {'joy': (0.1, 0.2)}
        check(0.1, 0.2, 'clear', 0.7)
        stop(bringup)
        pump(0.5)
        assert robot.pwm == {1: 91.0, 4: 127.5}

        # Test the hardware subscriber itself, with no AEB publisher remaining.
        final_pub = probe.create_publisher(TwistStamped, '/cmd_vel_stamped', 1)
        pump(0.2)
        message = TwistStamped()
        message.header.stamp = probe.get_clock().now().to_msg()
        message.twist.linear.x, message.twist.angular.z = 9.0, -9.0
        final_pub.publish(message)
        pump(0.12)
        assert robot.pwm[1] == pytest.approx(101.8)
        assert robot.pwm[4] == 75.0
        message.header.stamp = probe.get_clock().now().to_msg()
        message.twist.linear.x = math.nan
        final_pub.publish(message)
        pump(0.12)
        assert robot.pwm == {1: 91.0, 4: 127.5}
        message.header.stamp = probe.get_clock().now().to_msg()
        message.twist.linear.x, message.twist.angular.z = 0.2, 0.3
        final_pub.publish(message)
        pump(0.12)
        assert robot.pwm[1] > 91.0
        hardware.safe_shutdown()
        assert robot.pwm == {1: 91.0, 4: 127.5}
        print('PASS: mux/AEB/teleop + fake ybeb watchdog, clamps and shutdown')
    finally:
        for proc in processes:
            stop(proc)
        hardware.safe_shutdown()
        executor.shutdown()
        hardware.destroy_node()
        probe.destroy_node()
        context.try_shutdown()
        for stream in log_files:
            stream.close()

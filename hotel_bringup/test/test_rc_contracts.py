"""Check source-age guards, installed assets and actuator-independent imports."""

import ast
import importlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
import pytest
import yaml

from ybeb_2602_zulu.ros_support import finite_twist, stamp_lifetime


@pytest.mark.parametrize('stamp,expected', [
    (10.0, 0.3), (9.8, 0.1), (9.0, 0.0), (0.0, 0.0),
    (10.05, 0.3), (10.2, 0.0),
])
def test_header_age_cannot_rejuvenate_a_stale_command(stamp, expected):
    """Delayed, missing and future stamps are not fresh reception evidence."""
    message = TwistStamped()
    nanoseconds = round(stamp * 1e9)
    message.header.stamp.sec = nanoseconds // 1000000000
    message.header.stamp.nanosec = nanoseconds % 1000000000
    assert stamp_lifetime(message, 10000000000, 0.3, 0.1) == pytest.approx(expected)


def test_invalid_unused_axis_is_rejected():
    """A corrupt unused component still invalidates the whole command."""
    message = TwistStamped()
    message.twist.linear.y = math.nan
    assert not finite_twist(message)


def test_package_and_installed_assets():
    """Validate actual installation paths, YAML, package dependencies and imports."""
    share = Path(get_package_share_directory('hotel_bringup'))
    hardware = Path(get_package_share_directory('ybeb_2602_zulu'))
    required = ['rclpy', 'geometry_msgs', 'sensor_msgs', 'std_msgs', 'launch_ros',
                'joy', 'teleop_twist_joy', 'twist_mux', 'ybeb_2602_zulu']
    root = ET.parse(share / 'package.xml').getroot()
    deps = {entry.text for entry in root.findall('exec_depend')}
    assert set(required) <= deps
    for name in ('aeb_rc', 'twist_mux_rc', 'joystick_rc'):
        assert isinstance(yaml.safe_load((share / 'config' / (name + '.yaml')).read_text()), dict)
    assert (hardware / 'config/hardware_rc.yaml').is_file()
    for folder, names in ((share, ('rc_pc_bringup', 'joystick_rc')),
                          (hardware, ('rc_pi_bringup', 'rplidar'))):
        for name in names:
            path = folder / 'launch' / (name + '.launch.py')
            ast.parse(path.read_text(), filename=str(path))
    for module in ('hotel_bringup.aeb_node', 'hotel_bringup.safety_core',
                   'ybeb_2602_zulu.ybeb_node', 'ybeb_2602_zulu.ros_support'):
        importlib.import_module(module)
    mux = yaml.safe_load((share / 'config' / 'twist_mux_rc.yaml').read_text())
    assert mux['twist_mux']['ros__parameters']['use_stamped'] is True
    params = yaml.safe_load((share / 'config' / 'aeb_rc.yaml').read_text())
    assert params['rc_aeb']['ros__parameters']['calibration_confirmed'] is False

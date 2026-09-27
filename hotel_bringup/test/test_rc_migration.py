"""Protect package ownership, PC/Pi separation and the former fixed-turn regression."""

import ast
from dataclasses import fields
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from ament_flake8.main import main_with_errors
from ament_index_python.packages import get_package_share_directory
from ament_pep257.main import main as pep257
from hotel_bringup.safety_core import AEBConfig
from hotel_ttc_follow_the_gap.ttc_control import TTCControl
import pytest
from ybeb_2602_zulu.scan_support import ScanQuality


def test_pc_and_pi_launch_responsibilities():
    """The PC must never start hardware; the Pi package must not depend on PC logic."""
    pc = Path(get_package_share_directory('hotel_bringup'))
    pi = Path(get_package_share_directory('ybeb_2602_zulu'))
    pc_source = (pc / 'launch/rc_pc_bringup.launch.py').read_text()
    pi_source = (pi / 'launch/rc_pi_bringup.launch.py').read_text()
    assert "executable='aeb_node'" in pc_source
    assert "package='twist_mux'" in pc_source
    for forbidden in ('Rosmaster', "executable='ybeb_node'", "package='rplidar_ros'",
                      "package='gazebo_ros'"):
        assert forbidden not in pc_source
    assert "executable='ybeb_node'" in pi_source
    for forbidden in ("executable='aeb_node'", "package='twist_mux'", "package='joy'"):
        assert forbidden not in pi_source
    deps = {item.text for item in ET.parse(pi / 'package.xml').getroot().findall('exec_depend')}
    assert not deps.intersection({'hotel_bringup', 'hotel_wall_following',
                                  'hotel_ttc_follow_the_gap', 'twist_mux', 'joy'})
    ast.parse(pc_source)
    ast.parse(pi_source)


def test_shared_scan_quality_preserves_validated_defaults():
    """Extracting shared geometry must not change the defaults used by Wall/Gap."""
    aeb, quality = AEBConfig(), ScanQuality()
    for field in fields(quality):
        assert getattr(aeb, field.name) == getattr(quality, field.name)


@pytest.mark.parametrize('angle', [-0.2, 0.2])
def test_legacy_gap_no_longer_forces_fixed_turn(angle):
    """The previous +/-1.4 branch is removed even from the historical controller."""
    control = SimpleNamespace(gap_angle=angle, deadband=0.05, kp=1.2,
                              max_steering=0.5, _clamp=TTCControl._clamp)
    actual = TTCControl._calculate_steering(control)
    assert 0 < abs(actual) < 0.5


def test_rc_python_style():
    """Check migrated code separately from the documented pre-existing lint debt."""
    source = Path(__file__).resolve().parents[2]
    files = []
    for package, names in {
        'hotel_bringup': [
            'hotel_bringup/aeb_node.py', 'hotel_bringup/safety_core.py',
            'launch/rc_pc_bringup.launch.py', 'launch/joystick_rc.launch.py'],
        'hotel_wall_following': [
            'hotel_wall_following/rc_core.py', 'hotel_wall_following/rc_node.py',
            'hotel_wall_following/rc_support.py', 'launch/wall_rc.launch.py'],
        'hotel_ttc_follow_the_gap': [
            'hotel_ttc_follow_the_gap/rc_core.py', 'hotel_ttc_follow_the_gap/rc_node.py',
            'launch/gap_rc.launch.py'],
    }.items():
        files.extend(str(source / package / name) for name in names)
    files.extend(str(path) for path in (source / 'hotel_bringup/test').glob('test_rc_*.py'))
    code, errors = main_with_errors(argv=files)
    assert code == 0, '\n'.join(errors)
    assert pep257(argv=files) == 0

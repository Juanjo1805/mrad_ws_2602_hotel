"""Optional bounded RC Gap source running on the PC."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start with no active command until enable heartbeats arrive."""
    config = os.path.join(get_package_share_directory('hotel_ttc_follow_the_gap'),
                          'config', 'gap_rc.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('gap_config', default_value=config),
        Node(package='hotel_ttc_follow_the_gap', executable='rc_gap_node', name='rc_gap',
             parameters=[LaunchConfiguration('gap_config')], output='screen'),
    ])

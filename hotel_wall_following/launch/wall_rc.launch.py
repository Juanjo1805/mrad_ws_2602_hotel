"""Optional RC right-wall source; disabled until operator enable heartbeats."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Keep the RC source separate from the historical simulation controller."""
    config = os.path.join(get_package_share_directory('hotel_wall_following'),
                          'config', 'wall_rc.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('wall_config', default_value=config),
        Node(package='hotel_wall_following', executable='rc_wall_node', name='rc_wall',
             parameters=[LaunchConfiguration('wall_config')], output='screen'),
    ])

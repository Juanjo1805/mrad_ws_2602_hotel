"""Raspberry-only drivers: optional real LiDAR and the single hardware interface."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Never run mux or AEB here; the PC supplies the guarded command over DDS."""
    share = get_package_share_directory('ybeb_2602_zulu')
    return LaunchDescription([
        DeclareLaunchArgument('start_lidar', default_value='false'),
        DeclareLaunchArgument('start_hardware', default_value='false'),
        DeclareLaunchArgument(
            'hardware_config', default_value=os.path.join(share, 'config', 'hardware_rc.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'rplidar.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_lidar'))),
        Node(package='ybeb_2602_zulu', executable='ybeb_node', name='ybeb_node',
             parameters=[LaunchConfiguration('hardware_config')],
             condition=IfCondition(LaunchConfiguration('start_hardware')), output='screen'),
    ])

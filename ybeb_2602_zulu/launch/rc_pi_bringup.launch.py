"""Independent Pi stages: real LiDAR, simple AEB, and Rosmaster hardware."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Hardware remains off by default."""
    share = get_package_share_directory('ybeb_2602_zulu')
    return LaunchDescription([
        DeclareLaunchArgument('start_lidar', default_value='false'),
        DeclareLaunchArgument('start_aeb', default_value='false'),
        DeclareLaunchArgument('start_hardware', default_value='false'),
        DeclareLaunchArgument(
            'aeb_config', default_value=os.path.join(share, 'config', 'aeb_rc.yaml')),
        DeclareLaunchArgument(
            'hardware_config', default_value=os.path.join(share, 'config', 'hardware_rc.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'rplidar.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_lidar'))),
        Node(package='ybeb_2602_zulu', executable='aeb_node', name='aeb_node',
             parameters=[LaunchConfiguration('aeb_config')],
             condition=IfCondition(LaunchConfiguration('start_aeb')), output='screen'),
        Node(package='ybeb_2602_zulu', executable='ybeb_node', name='ybeb_node',
             parameters=[LaunchConfiguration('hardware_config')],
             condition=IfCondition(LaunchConfiguration('start_hardware')), output='screen'),
    ])

"""Manual RC control on the PC: joystick, teleop, and stamped twist_mux."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hotel_bringup')
    joystick = os.path.join(share, 'config', 'joystick_rc.yaml')
    mux = os.path.join(share, 'config', 'twist_mux_rc.yaml')
    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[joystick], output='screen'),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_node',
             parameters=[joystick], remappings=[('/cmd_vel', '/cmd_vel_joy')],
             output='screen'),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             parameters=[mux], remappings=[('/cmd_vel_out', '/cmd_vel_mux')],
             output='screen'),
    ])

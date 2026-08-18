#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -------- Launch Arguments --------
    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='1.5',
        description='Proportional gain'
    )

    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='0.48',
        description='Derivative gain'
    )

    vel_arg = DeclareLaunchArgument(
        'velocity',
        default_value='1.0',
        description='Forward velocity'
    )

    # -------- Launch Configurations --------
    kp = LaunchConfiguration('kp')
    kd = LaunchConfiguration('kd')
    velocity = LaunchConfiguration('velocity')

    # -------- Nodes --------
    dist_finder_node = Node(
        package='hotel_wall_following',
        executable='dist_finder',
        name='dist_finder',
        output='screen'
    )

    control_node = Node(
        package='hotel_wall_following',
        executable='control',
        name='wall_follower_control',
        output='screen',
        parameters=[
            {'kp': kp},
            {'kd': kd},
            {'velocity': velocity}
        ]
    )

    return LaunchDescription([
        kp_arg,
        kd_arg,
        vel_arg,
        dist_finder_node,
        control_node
    ])

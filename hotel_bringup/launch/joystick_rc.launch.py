"""Computer-side deadman joystick publishing only /cmd_vel_joy."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def joystick_nodes(context):
    """Reject unsafe launch scales before starting either joystick node."""
    linear = float(LaunchConfiguration('linear_scale').perform(context))
    steering = float(LaunchConfiguration('steering_scale').perform(context))
    if not math.isfinite(linear) or not 0 < linear <= 0.4:
        raise ValueError('linear_scale must be in (0, 0.4]')
    if not math.isfinite(steering) or not 0 < steering <= 0.5:
        raise ValueError('steering_scale must be in (0, 0.5]')
    axis = int(LaunchConfiguration('steering_axis').perform(context))
    if axis < 0:
        raise ValueError('steering_axis must be nonnegative')
    config = os.path.join(get_package_share_directory('hotel_bringup'),
                          'config', 'joystick_rc.yaml')
    return [
        Node(package='joy', executable='joy_node', name='joy_node', parameters=[config]),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_node',
             parameters=[config, {'scale_linear.x': linear, 'scale_linear_turbo.x': linear,
                                  'scale_angular.yaw': steering,
                                  'scale_angular_turbo.yaw': steering,
                                  'axis_angular.yaw': axis}],
             remappings=[('/cmd_vel', '/cmd_vel_joy')], output='screen'),
    ]


def generate_launch_description():
    """Expose conservative scales and the physically verified steering axis."""
    return LaunchDescription([
        DeclareLaunchArgument('linear_scale', default_value='0.1'),
        DeclareLaunchArgument('steering_scale', default_value='0.25'),
        DeclareLaunchArgument('steering_axis', default_value='3'),
        OpaqueFunction(function=joystick_nodes),
    ])

"""PC-only RC chain: optional sources -> mux -> AEB -> DDS to Raspberry."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Run no hardware driver, serial library or simulator on the PC."""
    share = get_package_share_directory('hotel_bringup')
    actions = [
        DeclareLaunchArgument('start_joystick', default_value='false'),
        DeclareLaunchArgument('start_wall', default_value='false'),
        DeclareLaunchArgument('start_gap', default_value='false'),
        DeclareLaunchArgument('linear_scale', default_value='0.1'),
        DeclareLaunchArgument('steering_scale', default_value='0.25'),
        DeclareLaunchArgument('steering_axis', default_value='3'),
        DeclareLaunchArgument('calibration_confirmed', default_value='false'),
        DeclareLaunchArgument(
            'aeb_config', default_value=os.path.join(share, 'config', 'aeb_rc.yaml')),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             parameters=[os.path.join(share, 'config', 'twist_mux_rc.yaml')],
             remappings=[('/cmd_vel_out', '/cmd_vel_mux')], output='screen'),
        Node(package='hotel_bringup', executable='aeb_node', name='rc_aeb',
             parameters=[LaunchConfiguration('aeb_config'), {
                 'use_sim_time': False,
                 'calibration_confirmed': ParameterValue(
                     LaunchConfiguration('calibration_confirmed'), value_type=bool)}],
             output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'joystick_rc.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_joystick')),
            launch_arguments={name: LaunchConfiguration(name) for name in (
                'linear_scale', 'steering_scale', 'steering_axis')}.items()),
    ]
    for mode, package in (('wall', 'hotel_wall_following'), ('gap', 'hotel_ttc_follow_the_gap')):
        actions.append(DeclareLaunchArgument(
            mode + '_config', default_value=PathJoinSubstitution([
                FindPackageShare(package), 'config', mode + '_rc.yaml'])))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare(package), 'launch', mode + '_rc.launch.py'])),
            condition=IfCondition(LaunchConfiguration('start_' + mode)),
            launch_arguments={mode + '_config': LaunchConfiguration(mode + '_config')}.items()))
    return LaunchDescription(actions)

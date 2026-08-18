#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -------- Launch Arguments --------
    # Parametros que se pueden cambiar desde ros2 launch.
    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='1.5',
        description='Ganancia proporcional: corrige segun el error actual'
    )

    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='0.48',
        description='Ganancia derivativa: corrige cambios rapidos del error'
    )

    max_steering_arg = DeclareLaunchArgument(
        'max_steering',
        default_value='2.2',
        description='Limite maximo del giro enviado al robot, en rad/s'
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='1.7',
        description='Velocidad maxima cuando el robot esta alineado, en m/s'
    )

    min_velocity_arg = DeclareLaunchArgument(
        'min_velocity',
        default_value='1.33',
        description='Velocidad minima durante el seguimiento de pared, en m/s'
    )

    kv_arg = DeclareLaunchArgument(
        'kv',
        default_value='2.1',
        description='Ganancia que baja la velocidad cuando aumenta el error'
    )

    theta_arg = DeclareLaunchArgument(
        'theta_deg',
        default_value='48.0',
        description='Angulo diagonal del LiDAR usado para estimar la pared'
    )

    lookahead_arg = DeclareLaunchArgument(
        'lookahead_dist',
        default_value='1.5',
        description='Distancia de anticipacion para calcular el error futuro'
    )

    desired_distance_arg = DeclareLaunchArgument(
        'desired_distance',
        default_value='0.72',
        description='Distancia objetivo que se quiere mantener con la pared'
    )

    # -------- Launch Configurations --------
    kp = LaunchConfiguration('kp')
    kd = LaunchConfiguration('kd')
    max_steering = LaunchConfiguration('max_steering')
    max_velocity = LaunchConfiguration('max_velocity')
    min_velocity = LaunchConfiguration('min_velocity')
    kv = LaunchConfiguration('kv')
    theta_deg = LaunchConfiguration('theta_deg')
    lookahead_dist = LaunchConfiguration('lookahead_dist')
    desired_distance = LaunchConfiguration('desired_distance')

    # -------- Nodes --------
    dist_finder_node = Node(
        package='hotel_wall_following',
        executable='dist_finder',
        name='dist_finder',
        output='screen',
        parameters=[
            {'theta_deg': theta_deg},
            {'lookahead_dist': lookahead_dist},
            {'desired_distance': desired_distance},
        ]
    )

    control_node = Node(
        package='hotel_wall_following',
        executable='control',
        name='wall_follower_control',
        output='screen',
        parameters=[
            {'kp': kp},
            {'kd': kd},
            {'max_steering': max_steering},
            {'max_velocity': max_velocity},
            {'min_velocity': min_velocity},
            {'kv': kv},
        ]
    )

    return LaunchDescription([
        kp_arg,
        kd_arg,
        max_steering_arg,
        max_velocity_arg,
        min_velocity_arg,
        kv_arg,
        theta_arg,
        lookahead_arg,
        desired_distance_arg,
        dist_finder_node,
        control_node
    ])

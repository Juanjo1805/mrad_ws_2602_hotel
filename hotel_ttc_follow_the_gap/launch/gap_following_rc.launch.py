"""Only the RC Follow the Gap node on the PC."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import yaml


TUNING_ARGUMENTS = (
    ('front_angle_deg', 'Frente en /scan, grados. Es 180 en este carro.'),
    ('fov_deg', 'Sector frontal total usado para buscar gaps, grados.'),
    ('min_clearance', 'Distancia minima de un rayo libre, metros.'),
    ('bubble_radius', 'Margen sobre cada obstaculo cercano, metros.'),
    ('bubble_trigger_distance', 'Distancia que activa la burbuja, metros.'),
    ('min_gap_width_deg', 'Ancho angular minimo del gap, grados.'),
    ('smooth_alpha', 'Suavizado del angulo: 0 inmediato, cercano a 1 lento.'),
    ('steering_gain', 'Intensidad del giro hacia el gap elegido.'),
    ('steering_sign', 'Direccion del giro: +1 normal; -1 invierte.'),
    ('angle_deadband', 'Error angular ignorado cerca del frente, radianes.'),
    ('max_steering', 'Limite absoluto de angular.z RC; maximo 0.5.'),
    ('max_velocity', 'Throttle RC maximo en camino libre; maximo 0.4.'),
    ('min_velocity', 'Throttle RC minimo con gap valido; <= max_velocity.'),
    ('steering_slowdown', 'Reduccion de throttle al aumentar el giro.'),
)


def generate_launch_description():
    """Expose Gap RC tuning with defaults from one YAML file."""
    share = get_package_share_directory('hotel_ttc_follow_the_gap')
    config = os.path.join(share, 'config', 'gap_following_rc.yaml')
    with open(config, encoding='utf-8') as stream:
        defaults = yaml.safe_load(stream)['gap_rc']['ros__parameters']
    arguments = [
        DeclareLaunchArgument(name, default_value=str(defaults[name]),
                              description=description)
        for name, description in TUNING_ARGUMENTS
    ]
    overrides = {
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name, _ in TUNING_ARGUMENTS
    }
    return LaunchDescription(arguments + [
        Node(package='hotel_ttc_follow_the_gap', executable='gap_rc_node',
             name='gap_rc', parameters=[config, overrides], output='screen'),
    ])

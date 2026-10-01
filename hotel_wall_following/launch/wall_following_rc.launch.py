"""Only the RC right-wall follower; PC bringup runs separately."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import yaml


TUNING_ARGUMENTS = (
    ('front_angle_deg',
     'Frente en /scan, grados. Es 180; verificar antes de cambiar.'),
    ('desired_distance',
     'Distancia a la pared derecha, metros. Subir para ir mas lejos.'),
    ('theta_deg',
     'Angulo entre el rayo derecho y el diagonal delantero, grados.'),
    ('lookahead_dist',
     'Anticipacion en metros. Subir da mas peso a la inclinacion.'),
    ('kp',
     'Ganancia proporcional: intensidad del giro ante el error lateral.'),
    ('kd',
     'Ganancia derivativa: responde a cambios; subir amplifica ruido.'),
    ('kv',
     'Reduce el throttle cuando aumenta el error; subir frena mas.'),
    ('max_velocity',
     'Throttle maximo alineado, en unidades RC. Entre min_velocity y 0.4.'),
    ('min_velocity',
     'Throttle minimo con pared valida; debe bastar para mover el carro.'),
    ('max_steering',
     'Limite absoluto de angular.z, en unidades RC; maximo 0.5.'),
    ('steering_sign',
     'Sentido del giro: +1 normal, -1 invierte si la prueba fisica lo exige.'),
)


def generate_launch_description():
    """Expose Wall RC tuning with defaults read from its YAML file."""
    config = os.path.join(get_package_share_directory('hotel_wall_following'),
                          'config', 'wall_following_rc.yaml')
    with open(config, encoding='utf-8') as stream:
        defaults = yaml.safe_load(stream)['wall_rc']['ros__parameters']
    arguments = [
        DeclareLaunchArgument(name, default_value=str(defaults[name]),
                              description=help_text)
        for name, help_text in TUNING_ARGUMENTS
    ]
    overrides = {
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name, _ in TUNING_ARGUMENTS
    }
    return LaunchDescription(arguments + [
        Node(package='hotel_wall_following', executable='wall_rc_node',
             name='wall_rc',
             parameters=[config, overrides], output='screen'),
    ])

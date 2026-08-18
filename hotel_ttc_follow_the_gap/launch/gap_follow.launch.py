from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -------- ARGUMENTOS --------
    args = [
        DeclareLaunchArgument('ttc_min',          default_value='2.0',   description='TTC mínimo (s)'),
        DeclareLaunchArgument('v_max',             default_value='1.9',   description='Velocidad máxima (m/s)'),
        DeclareLaunchArgument('v_min',             default_value='1.65',  description='Velocidad mínima garantizada (m/s)'),
        DeclareLaunchArgument('kp_steering',       default_value='0.5',   description='Ganancia proporcional de steering'),
        DeclareLaunchArgument('max_steering',      default_value='2.5',   description='Saturación de steering (rad/s)'),
        DeclareLaunchArgument('steering_slowdown', default_value='1.5',   description='Reducción de vel. en curva'),
        DeclareLaunchArgument('fov_deg',           default_value='60.0', description='Campo de visión LiDAR (grados)'),
        DeclareLaunchArgument('bubble_base',       default_value='0.45',  description='Radio base de burbuja (m)'),
        DeclareLaunchArgument('bubble_vel_k',      default_value='0.25',   description='Ganancia de burbuja por velocidad'),
        DeclareLaunchArgument('min_clearance',     default_value='1.5',  description='Distancia mínima a paredes (m)'),
        DeclareLaunchArgument('smooth_alpha',      default_value='0.0',  description='Suavizado de ángulo (0=raw, 1=fijo)'),
    ]

    cfg = {k: LaunchConfiguration(k) for k in [
        'ttc_min', 'v_max', 'v_min', 'kp_steering', 'max_steering',
        'steering_slowdown', 'fov_deg', 'bubble_base', 'bubble_vel_k',
        'min_clearance', 'smooth_alpha',
    ]}

    # -------- NODOS --------
    gap_finder_node = Node(
        package    = 'hotel_ttc_follow_the_gap',
        executable = 'ttc_gap_finder',
        name       = 'ttc_gap_finder',
        output     = 'screen',
        parameters = [{
            'ttc_min':       cfg['ttc_min'],
            'fov_deg':       cfg['fov_deg'],
            'bubble_base':   cfg['bubble_base'],
            'bubble_vel_k':  cfg['bubble_vel_k'],
            'min_clearance': cfg['min_clearance'],
            'smooth_alpha':  cfg['smooth_alpha'],
        }]
    )

    control_node = Node(
        package    = 'hotel_ttc_follow_the_gap',
        executable = 'ttc_control',
        name       = 'ttc_control',
        output     = 'screen',
        parameters = [{
            'ttc_min':          cfg['ttc_min'],
            'v_max':            cfg['v_max'],
            'v_min':            cfg['v_min'],
            'kp_steering':      cfg['kp_steering'],
            'max_steering':     cfg['max_steering'],
            'steering_slowdown':cfg['steering_slowdown'],
        }]
    )

    return LaunchDescription(args + [gap_finder_node, control_node])
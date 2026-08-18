from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -------- ARGUMENTOS CONFIGURABLES --------
    # Estos valores se pueden cambiar al lanzar el nodo con ros2 launch.
    args = [
        DeclareLaunchArgument('ttc_min',          default_value='2.0',   description='Tiempo minimo antes de colisionar; mas alto = mas conservador'),
        DeclareLaunchArgument('v_max',             default_value='1.9',   description='Velocidad maxima del robot en camino libre, en m/s'),
        DeclareLaunchArgument('v_min',             default_value='1.65',  description='Velocidad minima garantizada cuando el robot esta habilitado, en m/s'),
        DeclareLaunchArgument('kp_steering',       default_value='0.5',   description='Ganancia que convierte el angulo del gap en giro'),
        DeclareLaunchArgument('max_steering',      default_value='2.5',   description='Limite maximo del giro enviado, en rad/s'),
        DeclareLaunchArgument('steering_slowdown', default_value='1.5',   description='Cuanto baja la velocidad cuando el giro aumenta'),
        DeclareLaunchArgument('fov_deg',           default_value='60.0',  description='Campo de vision frontal usado del LiDAR, en grados'),
        DeclareLaunchArgument('bubble_base',       default_value='0.45',  description='Radio base de seguridad alrededor de obstaculos, en metros'),
        DeclareLaunchArgument('bubble_vel_k',      default_value='0.25',  description='Crecimiento de la burbuja segun la velocidad'),
        DeclareLaunchArgument('min_clearance',     default_value='1.5',   description='Distancia minima libre para aceptar un punto como seguro, en metros'),
        DeclareLaunchArgument('smooth_alpha',      default_value='0.0',   description='Suavizado del angulo; 0 = sin suavizar, 1 = mantiene el anterior'),
        DeclareLaunchArgument('min_depth_threshold', default_value='0.9', description='Profundidad minima del gap antes de penalizarlo como callejon'),
        DeclareLaunchArgument('deadend_weight',    default_value='1.1',   description='Peso de penalizacion para gaps poco profundos'),
        DeclareLaunchArgument('min_gap_width_deg', default_value='3.0',   description='Ancho angular minimo para aceptar un gap, en grados'),
        DeclareLaunchArgument('angle_deadband',    default_value='0.05',  description='Zona muerta del angulo para evitar correcciones pequenas'),
    ]

    cfg = {k: LaunchConfiguration(k) for k in [
        'ttc_min', 'v_max', 'v_min', 'kp_steering', 'max_steering',
        'steering_slowdown', 'fov_deg', 'bubble_base', 'bubble_vel_k',
        'min_clearance', 'smooth_alpha', 'min_depth_threshold',
        'deadend_weight', 'min_gap_width_deg', 'angle_deadband',
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
            'min_depth_threshold': cfg['min_depth_threshold'],
            'deadend_weight': cfg['deadend_weight'],
            'min_gap_width_deg': cfg['min_gap_width_deg'],
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
            'angle_deadband':   cfg['angle_deadband'],
        }]
    )

    return LaunchDescription(args + [gap_finder_node, control_node])

"""
ackermann.launch.py
hotel_bringup/launch/

Launch file para el robot Ackermann en Gazebo Harmonic + ROS 2 Jazzy.

Cambios respecto al launch de diferencial:
  - URDF:       ackerman_urdf/ackerman_urdf.xacro  (en hotel_description)
  - Ctrl:       ackermann_steering_controller  (en lugar de diffdrive_controller)
  - twist_mux:  redirige /cmd_vel_out → /ackermann_steering_controller/reference
                (TwistStamped es aceptado directamente por el controlador)
  - Bridge:     topic_bridge_ackermann.yaml
  - Spawn name: hotel_ackermann

Uso:
  ros2 launch hotel_bringup ackermann.launch.py
  ros2 launch hotel_bringup ackermann.launch.py gz_mode:=false          # headless
  ros2 launch hotel_bringup ackermann.launch.py world:=/ruta/mundo.sdf
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Nombres de paquetes ───────────────────────────────────────────────
    description_pkg = "hotel_description"
    bringup_pkg     = "hotel_bringup"
    gazebo_pkg      = "hotel_gazebo"

    # ── Argumentos del launch ─────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Usar reloj de Gazebo si true",
    )
    gz_mode_arg = DeclareLaunchArgument(
        "gz_mode",
        default_value="true",
        description="true = GUI, false = headless",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            get_package_share_directory(gazebo_pkg), "worlds", "RaceTrack.sdf"
        ),
        description="Ruta completa al archivo SDF del mundo",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    world        = LaunchConfiguration("world")

    # ── Descripción del robot (xacro → URDF XML) ──────────────────────────
    xacro_file = os.path.join(
        get_package_share_directory(description_pkg),
        "ackerman_urdf",
        "ackerman_urdf.xacro",
    )
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    # ── Robot State Publisher ─────────────────────────────────────────────
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_xml},
            {"use_sim_time": use_sim_time},
        ],
    )

    # ── Gazebo Harmonic (GUI) ─────────────────────────────────────────────
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("gz_mode")),
    )

    # ── Gazebo Harmonic (headless) ────────────────────────────────────────
    gz_headless_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": ["-r -s -v4 ", world],
            "headless-rendering": "true",
            "on_exit_shutdown": "true",
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("gz_mode")),
    )

    # ── Spawn del robot en Gazebo ─────────────────────────────────────────
    # z=0.10 → las ruedas (radio 0.052 m) quedan sobre el suelo con holgura
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",  "hotel_ackermann",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.10",
            "-R", "0",   "-P", "0",   "-Y", "0.0",
        ],
    )

    # ── Bridge Gazebo ↔ ROS 2 (scan, imu, clock) ─────────────────────────
    bridge_config = os.path.join(
        get_package_share_directory(gazebo_pkg),
        "config",
        "topic_bridge_ackermann.yaml",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "--ros-args",
            "-p", f"config_file:={bridge_config}",
        ],
    )

    # ── Spawner: ackermann_steering_controller ────────────────────────────
    ackermann_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller"],
        output="screen",
    )

    # ── Spawner: joint_state_broadcaster ─────────────────────────────────
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # ── Joystick ──────────────────────────────────────────────────────────
    joy_params = os.path.join(
        get_package_share_directory(bringup_pkg), "config", "joystick.yaml"
    )
    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_params],
        output="screen",
    )
    teleop_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[joy_params],
        remappings=[("/cmd_vel", "/cmd_vel_joy")],
        output="screen",
    )

    # ── Twist Mux ─────────────────────────────────────────────────────────
    twist_mux_params = os.path.join(
        get_package_share_directory(bringup_pkg), "config", "twist_mux.yaml"
    )
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        # /cmd_vel_out → referencia directa al controlador Ackermann
        # El ackermann_steering_controller acepta TwistStamped en ~/reference:
        #   linear.x  → velocidad deseada [m/s]
        #   angular.z → velocidad angular deseada [rad/s]  (ctrl convierte a ángulo)
        remappings=[
            ("/cmd_vel_out", "/ackermann_steering_controller/reference")
        ],
        output="screen",
    )

    # ── AEB ───────────────────────────────────────────────────────────────
    aeb_node = Node(
        package=bringup_pkg,
        executable="aeb_node",
        output="screen",
    )

    # ── LaunchDescription ─────────────────────────────────────────────────
    return LaunchDescription(
        [
            use_sim_time_arg,
            gz_mode_arg,
            world_arg,
            # Gazebo
            gz_launch,
            gz_headless_launch,
            # Robot
            rsp,
            spawn,
            bridge,
            # Controllers
            ackermann_ctrl_spawner,
            joint_broad_spawner,
            # Teleop
            joy_node,
            teleop_joy_node,
            twist_mux_node,
            # Seguridad
            aeb_node,
        ]
    )

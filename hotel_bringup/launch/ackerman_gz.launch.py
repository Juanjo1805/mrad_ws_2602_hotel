import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

import xacro

def generate_launch_description():
    gazebo_pkg_name = "hotel_gazebo"
    bringup_pkg_name = "hotel_bringup"
    description_pkg_name = "hotel_description"

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    # --- Robot description (xacro -> URDF XML string) ---
    xacro_file = os.path.join(get_package_share_directory(description_pkg_name), "diffdrive_urdf", "ackerman.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": use_sim_time}],
    )

    # --- Launch Gazebo (via ros_gz_sim launch file) ---
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('gz_mode')),
    )

    # --- Launch Gazebo Headless (via ros_gz_sim launch file) ---
    gz_headless_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": ['-r -s -v4 ', world], 'headless-rendering': 'true', 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(LaunchConfiguration('gz_mode')),
    )

    # --- Spawn entity into Gazebo from robot_description topic ---
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "diffbot",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.5", "-R", "0", "-P", "0", "-Y", "0.0"
        ],
    )
   
    bridge_params = os.path.join(get_package_share_directory(gazebo_pkg_name),'config','topic_bridge.yaml')

    bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    output="screen",
    arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
    )

    depth_cloud_tf = Node( package="tf2_ros", 
                          executable="static_transform_publisher", 
                          arguments=[ "0", "0", "0", "0", "0", "0", 
                                     "depth_camera_link", 
                                     "diffbot/base_link/depth_camera", ], 
                          output="screen", )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffdrive_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster_controller"],
    )

# ------
# Code
# ------

    joy_params = os.path.join(get_package_share_directory(bringup_pkg_name),'config','joystick.yaml')

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    joy_node = Node(package='joy', 
                    executable='joy_node',
                    parameters=[joy_params],
    )

    teleop_node = Node(package='teleop_twist_joy', 
                    executable='teleop_node',
                    name="teleop_node",
                    parameters=[joy_params],
                    remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    twist_mux_params = os.path.join(get_package_share_directory(bringup_pkg_name),'config','twist_mux.yaml')
    
    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': True}],
                    remappings=[('/cmd_vel_out','/diffdrive_controller/cmd_vel')]
    )

    aeb_node = Node(
        package=bringup_pkg_name,
        executable='aeb_node',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            "gz_mode",
            default_value="true",
            description="Run Gazebo with GUI if true, headless if false",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(get_package_share_directory(gazebo_pkg_name), "worlds", "RaceTrack.sdf"),
            description="Full path to world SDF file",
        ),
        gz_launch,
        gz_headless_launch,
        rsp,
        spawn,
        bridge,
        depth_cloud_tf,
        diff_drive_spawner,
        joint_broad_spawner,
        joy_node,
        teleop_node,
        twist_mux_node,
        aeb_node,
    ])
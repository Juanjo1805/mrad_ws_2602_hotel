import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

import xacro


def launch_gazebo(context, gazebo_pkg_name):
    world_value = LaunchConfiguration("world").perform(context)
    if not os.path.isabs(world_value):
        world_value = os.path.join(
            get_package_share_directory(gazebo_pkg_name),
            "worlds",
            world_value,
        )

    gz_sim_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("ros_gz_sim"),
            "launch",
            "gz_sim.launch.py",
        )
    )

    return [
        IncludeLaunchDescription(
            gz_sim_launch,
            launch_arguments={
                "gz_args": f"-r -v4 {world_value}",
                "on_exit_shutdown": "true",
            }.items(),
            condition=IfCondition(LaunchConfiguration("gz_mode")),
        ),
        IncludeLaunchDescription(
            gz_sim_launch,
            launch_arguments={
                "gz_args": f"-r -s -v4 {world_value}",
                "headless-rendering": "true",
                "on_exit_shutdown": "true",
            }.items(),
            condition=UnlessCondition(LaunchConfiguration("gz_mode")),
        ),
    ]


def generate_launch_description():
    gazebo_pkg_name = "hotel_gazebo"
    bringup_pkg_name = "hotel_bringup"
    description_pkg_name = "hotel_description"

    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    # --- Robot description (xacro -> URDF XML string) ---
    xacro_file = os.path.join(get_package_share_directory(description_pkg_name), "diffdrive_urdf", "robot.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": use_sim_time}],
    )

    gz_launch = OpaqueFunction(function=launch_gazebo, args=[gazebo_pkg_name])

    # --- Spawn entity into Gazebo from robot_description topic ---
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "diffbot",
            "-topic", "robot_description",
            "-x", x_pose,
            "-y", y_pose,
            "-z", z_pose,
            "-R", roll,
            "-P", pitch,
            "-Y", yaw
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
                    remappings=[('/cmd_vel_out','/cmd_vel_mux')]
    )

    aeb_node = Node(
        package=bringup_pkg_name,
        executable='aeb_node',
        output='screen',
        remappings=[('/cmd_vel_out','/diffdrive_controller/cmd_vel')]
    )

    lidar_data_node = Node(
        package=bringup_pkg_name,
        executable='lidar_data',
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
            default_value="track.sdf",
            description="World SDF filename from hotel_gazebo/worlds or a full path",
        ),
        DeclareLaunchArgument("x_pose", default_value="-10.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        DeclareLaunchArgument("z_pose", default_value="0.5"),
        DeclareLaunchArgument("roll", default_value="0.0"),
        DeclareLaunchArgument("pitch", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="1.57"),
        gz_launch,
        rsp,
        TimerAction(period=3.0, actions=[spawn]),
        bridge,
        depth_cloud_tf,
        TimerAction(period=5.0, actions=[diff_drive_spawner, joint_broad_spawner]),
        joy_node,
        teleop_node,
        twist_mux_node,
        lidar_data_node,
        aeb_node,
    ])

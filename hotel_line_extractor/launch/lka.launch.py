from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("image_topic", default_value="/left_camera/image"),
        DeclareLaunchArgument("target_speed", default_value="0.5"),
        DeclareLaunchArgument("min_speed", default_value="0.2"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_lka"),
        DeclareLaunchArgument("kp_lateral", default_value="1.0"),
        DeclareLaunchArgument("kp_heading", default_value="0.8"),
        DeclareLaunchArgument("max_steering", default_value="1.4"),
        DeclareLaunchArgument("lane_width_px", default_value="260.0"),
        DeclareLaunchArgument("roi_top_ratio", default_value="0.55"),
        DeclareLaunchArgument("black_v_max", default_value="140"),
        DeclareLaunchArgument("dark_percentile", default_value="35.0"),
        DeclareLaunchArgument("dark_margin", default_value="25"),
        DeclareLaunchArgument("min_black_area", default_value="80.0"),
        DeclareLaunchArgument("min_mask_pixels", default_value="80"),
        DeclareLaunchArgument("use_joy_enable", default_value="true"),
        DeclareLaunchArgument("joy_button_index", default_value="5"),
    ]

    lka = Node(
        package="hotel_line_extractor",
        executable="lka_node",
        name="line_keeping_assist",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "target_speed": ParameterValue(
                    LaunchConfiguration("target_speed"), value_type=float
                ),
                "min_speed": ParameterValue(LaunchConfiguration("min_speed"), value_type=float),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "kp_lateral": ParameterValue(LaunchConfiguration("kp_lateral"), value_type=float),
                "kp_heading": ParameterValue(LaunchConfiguration("kp_heading"), value_type=float),
                "max_steering": ParameterValue(LaunchConfiguration("max_steering"), value_type=float),
                "lane_width_px": ParameterValue(
                    LaunchConfiguration("lane_width_px"), value_type=float
                ),
                "roi_top_ratio": ParameterValue(
                    LaunchConfiguration("roi_top_ratio"), value_type=float
                ),
                "black_v_max": ParameterValue(
                    LaunchConfiguration("black_v_max"), value_type=int
                ),
                "dark_percentile": ParameterValue(
                    LaunchConfiguration("dark_percentile"), value_type=float
                ),
                "dark_margin": ParameterValue(
                    LaunchConfiguration("dark_margin"), value_type=int
                ),
                "min_black_area": ParameterValue(
                    LaunchConfiguration("min_black_area"), value_type=float
                ),
                "min_mask_pixels": ParameterValue(
                    LaunchConfiguration("min_mask_pixels"), value_type=int
                ),
                "use_joy_enable": ParameterValue(
                    LaunchConfiguration("use_joy_enable"), value_type=bool
                ),
                "joy_button_index": ParameterValue(
                    LaunchConfiguration("joy_button_index"), value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(args + [lka])

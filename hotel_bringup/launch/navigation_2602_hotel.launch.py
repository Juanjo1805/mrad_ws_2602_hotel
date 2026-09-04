"""Select one assignment planner and tracker for the differential robot.

Start the existing differential simulation and localization first.  The
selected tracker publishes TwistStamped to /cmd_vel_nav, the navigation input
already configured in twist_mux.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    planner = LaunchConfiguration("planner")
    tracker = LaunchConfiguration("tracker")
    use_sim_time = LaunchConfiguration("use_sim_time")
    scenario = LaunchConfiguration("scenario")
    trial = LaunchConfiguration("trial")
    environment = LaunchConfiguration("environment")
    common = {"use_sim_time": use_sim_time, "scenario": scenario, "trial": trial,
              "environment": environment}
    return LaunchDescription([
        DeclareLaunchArgument("planner", default_value="dijkstra", description="dijkstra | hybrid_astar"),
        DeclareLaunchArgument("tracker", default_value="pure_pursuit", description="pure_pursuit | lqr"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("scenario", default_value="manual"),
        DeclareLaunchArgument("trial", default_value="0"),
        DeclareLaunchArgument("environment", default_value="gazebo",
                              description="gazebo | robot; offline is reserved for benchmark scripts"),
        Node(
            package="path_planner_2602_hotel", executable="dijkstra_pp_2602_hotel",
            name="dijkstra_pp_2602_hotel", output="screen",
            condition=IfCondition(PythonExpression(["'", planner, "' == 'dijkstra'"])),
            parameters=[common],
        ),
        Node(
            package="path_planner_2602_hotel", executable="hybrid_astar_pp_2602_hotel",
            name="hybrid_astar_pp_2602_hotel", output="screen",
            condition=IfCondition(PythonExpression(["'", planner, "' == 'hybrid_astar'"])),
            parameters=[common],
        ),
        Node(
            package="path_tracker_2602_hotel", executable="pure_pursuit_pt_2602_hotel",
            name="pure_pursuit_pt_2602_hotel", output="screen",
            condition=IfCondition(PythonExpression(["'", tracker, "' == 'pure_pursuit'"])),
            parameters=[{**common, "v_nominal": 0.45, "max_speed": 0.50, "max_omega": 2.0,
                         "lookahead_L0": 0.6, "lookahead_min": 0.3}],
        ),
        Node(
            package="path_tracker_2602_hotel", executable="lqr_pt_2602_hotel",
            name="lqr_pt_2602_hotel", output="screen",
            condition=IfCondition(PythonExpression(["'", tracker, "' == 'lqr'"])),
            parameters=[common],
        ),
    ])

"""Launch one planner and one differential-drive tracker.

The default mode plans a single ``/goal_pose``.  ``mission_mode:=fixed_waypoints``
instead connects every YAML waypoint with the selected planner and publishes
one dense ``/planned_path`` for the same tracker interface.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def mode_condition(planner, selected, mission_mode, expected_mode):
    return IfCondition(PythonExpression([
        "'", planner, "' == '", selected, "' and '", mission_mode,
        "' == '", expected_mode, "'",
    ]))


def tracker_condition(tracker, selected):
    return IfCondition(PythonExpression(["'", tracker, "' == '", selected, "'"]))


def generate_launch_description():
    planner = LaunchConfiguration('planner')
    tracker = LaunchConfiguration('tracker')
    mission_mode = LaunchConfiguration('mission_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scenario = LaunchConfiguration('scenario')
    trial = LaunchConfiguration('trial')
    environment = LaunchConfiguration('environment')
    waypoints_file = LaunchConfiguration('waypoints_file')
    laps = LaunchConfiguration('laps')
    planner_results_csv = LaunchConfiguration('planner_results_csv')
    tracker_trace_csv = LaunchConfiguration('tracker_trace_csv')
    tracker_results_csv = LaunchConfiguration('tracker_results_csv')
    common = {
        'use_sim_time': use_sim_time,
        'scenario': scenario,
        'trial': trial,
        'environment': environment,
        'results_csv': planner_results_csv,
    }
    tracker_parameters = {
        **common,
        'control_frequency': 25.0,
        'linear_velocity': 0.45,
        'max_linear_velocity': 0.50,
        'max_angular_velocity': 2.0,
        'lookahead_distance': 0.60,
        'minimum_lookahead': 0.30,
        'maximum_lookahead': 1.20,
        'goal_tolerance': 0.25,
        'yaw_tolerance': 0.21,
        'goal_progress_fraction': 0.95,
        'trace_csv': tracker_trace_csv,
        'results_csv': tracker_results_csv,
    }
    default_waypoints = os.path.join(
        get_package_share_directory('path_planner_2602_hotel'),
        'config',
        'fixed_waypoints.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='dijkstra',
                              description='dijkstra | hybrid_astar'),
        DeclareLaunchArgument('tracker', default_value='pure_pursuit',
                              description='pure_pursuit | lqr'),
        DeclareLaunchArgument('mission_mode', default_value='goal',
                              description='goal | fixed_waypoints'),
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints,
                              description='YAML used only by fixed_waypoints mode'),
        DeclareLaunchArgument('laps', default_value='1',
                              description='Number of fixed-waypoint laps'),
        DeclareLaunchArgument('planner_results_csv', default_value='results/planner_results.csv'),
        DeclareLaunchArgument('tracker_trace_csv', default_value='results/tracker_trace.csv'),
        DeclareLaunchArgument('tracker_results_csv', default_value='results/tracker_results.csv'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='manual'),
        DeclareLaunchArgument('trial', default_value='0'),
        DeclareLaunchArgument('environment', default_value='gazebo'),
        Node(
            package='path_planner_2602_hotel',
            executable='dijkstra_pp_2602_hotel',
            name='dijkstra_pp_2602_hotel',
            output='screen',
            condition=mode_condition(planner, 'dijkstra', mission_mode, 'goal'),
            parameters=[{**common, 'path_resolution': 0.05}],
        ),
        Node(
            package='path_planner_2602_hotel',
            executable='hybrid_astar_pp_2602_hotel',
            name='hybrid_astar_pp_2602_hotel',
            output='screen',
            condition=mode_condition(planner, 'hybrid_astar', mission_mode, 'goal'),
            parameters=[{**common, 'path_resolution': 0.05}],
        ),
        Node(
            package='path_planner_2602_hotel',
            executable='fixed_waypoint_planner',
            name='fixed_waypoint_planner',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", mission_mode, "' == 'fixed_waypoints'",
            ])),
            parameters=[{
                'use_sim_time': use_sim_time,
                'planner': planner,
                'waypoints_file': waypoints_file,
                'laps': laps,
                'path_resolution': 0.05,
            }],
        ),
        Node(
            package='path_tracker_2602_hotel',
            executable='pure_pursuit_pt_2602_hotel',
            name='pure_pursuit_pt_2602_hotel',
            output='screen',
            condition=tracker_condition(tracker, 'pure_pursuit'),
            parameters=[tracker_parameters],
        ),
        Node(
            package='path_tracker_2602_hotel',
            executable='lqr_pt_2602_hotel',
            name='lqr_pt_2602_hotel',
            output='screen',
            condition=tracker_condition(tracker, 'lqr'),
            parameters=[tracker_parameters],
        ),
    ])

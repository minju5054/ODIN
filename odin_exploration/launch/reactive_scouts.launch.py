from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


COMMON_PARAMS = {
    'use_sim_time': True,
    'forward_speed': 0.28,
    'slow_speed': 0.11,
    'max_turn_speed': 0.90,
    'stop_distance': 0.34,
    'free_distance': 0.55,
    'slow_distance': 0.90,
    'field_of_view_deg': 170.0,
    'front_angle_deg': 18.0,
    'target_smoothing': 0.35,
    'backup_speed': -0.04,
    'escape_duration': 1.4,
    'backup_duration': 0.35,
    'turn_commit_duration': 0.9,
    'explore_bias_speed': 0.12,
    'explore_bias_period': 12.0,
    'side_clearance_weight': 0.45,
    'enable_center_spiral': True,
    'center_x': 0.0,
    'center_y': 0.0,
    'center_spiral_weight': 0.35,
    'center_spiral_turn_direction': -1.0,
    'center_spiral_max_angle_deg': 45.0,
    'center_spiral_min_radius': 1.5,
    'hostage_event_topic': '/hostage_events',
    'enable_outer_explore_after_event': True,
    'outer_explore_weight': 0.45,
    'outer_explore_tangent_weight': 0.35,
    'outer_explore_max_angle_deg': 55.0,
    'outer_explore_min_radius': 1.2,
    'map_min_x': -9.2,
    'map_max_x': 9.2,
    'map_min_y': -9.2,
    'map_max_y': 9.2,
    'boundary_margin_m': 1.0,
    'red_zone_min_x': 4.0,
    'red_zone_max_x': 10.0,
    'red_zone_min_y': 4.0,
    'red_zone_max_y': 10.0,
    'red_zone_avoid_margin_m': 1.5,
    'red_zone_predict_horizon_m': 1.4,
    'red_zone_turn_away_weight': 0.85,
}


def _scout_node(namespace, node_name, preferred_turn_direction):
    params = dict(COMMON_PARAMS)
    params['preferred_turn_direction'] = preferred_turn_direction

    return Node(
        package='odin_exploration',
        executable='reactive_scout',
        namespace=namespace,
        name=node_name,
        output='screen',
        parameters=[params],
    )


def generate_launch_description():
    enable = LaunchConfiguration('enable')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable',
            default_value='true',
            description='Start simple reactive scout motion for robot_1 and robot_2.',
        ),
        _scout_node('robot_1', 'reactive_scout_robot_1', 1.0),
        _scout_node('robot_2', 'reactive_scout_robot_2', 1.0),
    ])

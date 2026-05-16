import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


LIFECYCLE_NODES = [
    'planner_server',
    'smoother_server',
    'controller_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]


def _configured_params(namespace, params_file, use_sim_time):
    return ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            },
            convert_types=True,
        ),
        allow_substs=True,
    )


def _robot_state_publisher(use_sim_time):
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        'turtlebot3_burger.urdf',
    )
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'frame_prefix': 'robot_3/',
        }],
    )


def _nav2_nodes(params, use_sim_time):
    common = {
        'namespace': 'robot_3',
        'output': 'screen',
        'parameters': [params],
        'arguments': ['--ros-args', '--log-level', 'info'],
    }

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            remappings=[('cmd_vel', 'cmd_vel_nav')],
            **common,
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            **common,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            **common,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            **common,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            **common,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            **common,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
            **common,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace='robot_3',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': LIFECYCLE_NODES,
            }],
        ),
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    dispatch_params_file = LaunchConfiguration('dispatch_params_file')
    share = get_package_share_directory('odin_navigation')
    default_nav2_params = os.path.join(share, 'config', 'nav2_robot_3.yaml')
    default_dispatch_params = os.path.join(share, 'config', 'robot_3_simple_dispatch.yaml')
    params = _configured_params('robot_3', nav2_params_file, use_sim_time)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params,
            description='Nav2 parameter file for robot_3.',
        ),
        DeclareLaunchArgument(
            'dispatch_params_file',
            default_value=default_dispatch_params,
            description='Spawn manager parameter file for robot_3.',
        ),
        Node(
            package='odin_navigation',
            executable='robot_3_spawn_on_goal',
            name='robot_3_spawn_on_goal',
            output='screen',
            parameters=[
                dispatch_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'spawn_trigger_topic': '/robot_3/spawn_trigger',
                },
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot_3_map_to_odom_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_3/odom'],
        ),
        GroupAction([
            PushRosNamespace('robot_3'),
            _robot_state_publisher(use_sim_time),
        ]),
        Node(
            package='odin_navigation',
            executable='robot_scan_filter',
            name='robot_3_scan_filter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_scan_topic': '/robot_3/scan',
                'filtered_scan_topic': '/robot_3/scan_nav2',
                'robot_3_odom_topic': '/robot_3/odom',
                'ignored_robot_odom_topics': ['/robot_1/odom', '/robot_2/odom'],
                'ignore_distance_m': 1.1,
                'ignored_robot_radius_m': 0.24,
                'range_tolerance_m': 0.18,
                'angle_margin_rad': 0.08,
                'odom_timeout_sec': 1.5,
                'dispatch_status_topic': '/robot_3/dispatch_status',
                'blind_start_distance_m': 3.0,
            }],
        ),
        *_nav2_nodes(params, use_sim_time),
        Node(
            package='odin_navigation',
            executable='nav2_goal_dispatcher',
            name='robot_3_nav2_goal_dispatcher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'goal_topic': '/robot_3/goal_pose',
                'selected_path_topic': '/ai/selected_path',
                'status_topic': '/robot_3/dispatch_status',
                'navigate_action': '/robot_3/navigate_to_pose',
                'follow_selected_path': True,
                'selected_path_waypoint_spacing_m': 1.0,
                'selected_path_goal_tolerance_m': 0.35,
            }],
        ),
        Node(
            package='odin_navigation',
            executable='mission_success_marker',
            name='mission_success_marker',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'dispatch_status_topic': '/robot_3/dispatch_status',
                'mission_status_topic': '/mission_status',
                'marker_topic': '/mission_marker',
                'frame_id': 'map',
                'text': 'MISSION SUCCESS',
                'x': 0.0,
                'y': 0.0,
                'z': 1.2,
            }],
        ),
        Node(
            package='odin_navigation',
            executable='mission_status_panel',
            name='mission_status_panel',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'hostage_event_topic': '/hostage_events',
                'coordinator_status_topic': '/coordinator/status',
                'dispatch_status_topic': '/robot_3/dispatch_status',
                'mission_status_topic': '/mission_status',
                'window_title': 'ODIN Mission Timeline',
            }],
        ),
        Node(
            package='odin_navigation',
            executable='qwen_route_map_panel',
            name='qwen_route_map_panel',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'merged_map_topic': '/merged_map',
                'hostage_event_topic': '/hostage_events',
                'coordinator_path_topic': '/coordinator/candidate_path',
                'candidate_routes_topic': '/coordinator/candidate_routes',
                'selected_path_topic': '/ai/selected_path',
                'robot_3_odom_topic': '/robot_3/odom',
                'red_zone_min_x': 4.0,
                'red_zone_max_x': 10.0,
                'red_zone_min_y': 4.0,
                'red_zone_max_y': 10.0,
                'window_title': 'ODIN Qwen Route Decision',
            }],
        ),
        Node(
            package='odin_navigation',
            executable='qwen_dialog_panel',
            name='qwen_dialog_panel',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'hostage_event_topic': '/hostage_events',
                'coordinator_status_topic': '/coordinator/status',
                'ai_status_topic': '/ai/status',
                'window_title': 'ODIN Coordinator / Qwen Dialog',
                'max_lines': 140,
            }],
        ),
        Node(
            package='odin_navigation',
            executable='mission_success_popup',
            name='mission_success_popup',
            output='screen',
            parameters=[{
                'mission_status_topic': '/mission_status',
                'window_title': 'ODIN Mission Status',
                'message': 'MISSION SUCCESS',
                'subtitle': 'HOSTAGE RESCUE COMPLETE',
            }],
        ),
    ])

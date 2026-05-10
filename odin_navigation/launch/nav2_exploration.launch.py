import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


ROBOTS = ('robot_1', 'robot_2')
ROBOT_START_DELAYS = {
    'robot_1': 0.0,
    'robot_2': 4.0,
}
LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
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


def _nav2_nodes(namespace, params_file, use_sim_time):
    params = _configured_params(namespace, params_file, use_sim_time)
    common = {
        'namespace': namespace,
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
            namespace=namespace,
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


def _explore_node(namespace, params_file):
    return Node(
        package='explore_lite',
        executable='explore',
        namespace=namespace,
        name='explore_node',
        output='screen',
        parameters=[_configured_params(namespace, params_file, LaunchConfiguration('use_sim_time'))],
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    share = get_package_share_directory('odin_navigation')

    actions = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
    ]

    for robot in ROBOTS:
        robot_actions = []
        robot_actions.extend(
            _nav2_nodes(
                robot,
                os.path.join(share, 'config', f'nav2_{robot}.yaml'),
                use_sim_time,
            )
        )
        robot_actions.append(
            _explore_node(
                robot,
                os.path.join(share, 'config', f'explore_{robot}.yaml'),
            )
        )
        actions.append(
            TimerAction(
                period=ROBOT_START_DELAYS[robot],
                actions=robot_actions,
            )
        )

    return LaunchDescription(actions)

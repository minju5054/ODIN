import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


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


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('dispatch_params_file')
    default_params = os.path.join(
        get_package_share_directory('odin_navigation'),
        'config',
        'robot_3_simple_dispatch.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'dispatch_params_file',
            default_value=default_params,
            description='Simple robot_3 dispatch follower parameter file.',
        ),
        Node(
            package='odin_navigation',
            executable='robot_3_spawn_on_goal',
            name='robot_3_spawn_on_goal',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
        GroupAction([
            PushRosNamespace('robot_3'),
            _robot_state_publisher(use_sim_time),
            Node(
                package='odin_navigation',
                executable='simple_goal_follower',
                name='simple_goal_follower',
                output='screen',
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time},
                ],
            ),
        ]),
    ])

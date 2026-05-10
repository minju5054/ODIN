import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = os.path.join(
        get_package_share_directory('odin_map_merge'),
        'config',
        'scenario_scan_map_merge.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        Node(
            package='odin_map_merge',
            executable='scenario_scan_map_merge',
            name='scenario_scan_map_merge',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        ),
    ])

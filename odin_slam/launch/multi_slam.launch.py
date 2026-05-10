import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _slam_node(namespace, params_file, use_sim_time):
    return Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace=namespace,
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'scan_topic': 'scan',
                'map_frame': f'{namespace}/map',
                'odom_frame': f'{namespace}/odom',
                'base_frame': f'{namespace}/base_footprint',
            },
        ],
        remappings=[
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
        ],
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = os.path.join(
        get_package_share_directory('odin_slam'),
        'config',
        'slam_toolbox.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        _slam_node('robot_1', params_file, use_sim_time),
        _slam_node('robot_2', params_file, use_sim_time),
    ])

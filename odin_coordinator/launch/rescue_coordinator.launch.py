import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('coordinator_params_file')
    battlefield_config_file = LaunchConfiguration('battlefield_config_file')
    default_params = os.path.join(
        get_package_share_directory('odin_coordinator'),
        'config',
        'rescue_coordinator.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'coordinator_params_file',
            default_value=default_params,
            description='Rescue coordinator parameter file.',
        ),
        DeclareLaunchArgument(
            'battlefield_config_file',
            default_value='',
            description='Shared battlefield rules YAML file.',
        ),
        Node(
            package='odin_coordinator',
            executable='rescue_coordinator',
            name='rescue_coordinator',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'battlefield_config_file': battlefield_config_file,
                },
            ],
        ),
    ])

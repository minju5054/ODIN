import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    detector_params_file = LaunchConfiguration('detector_params_file')

    default_params = os.path.join(
        get_package_share_directory('odin_detection'),
        'config',
        'rgb_aruco_event_detector.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'detector_params_file',
            default_value=default_params,
            description='RGB ArUco event detector parameters.',
        ),
        Node(
            package='odin_detection',
            executable='rgb_aruco_event_detector',
            name='rgb_aruco_event_detector',
            output='screen',
            parameters=[
                detector_params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])

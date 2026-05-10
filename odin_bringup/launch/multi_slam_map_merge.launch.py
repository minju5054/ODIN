from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_launch = PathJoinSubstitution([
        FindPackageShare('odin_slam'),
        'launch',
        'multi_slam.launch.py',
    ])
    map_merge_launch = PathJoinSubstitution([
        FindPackageShare('odin_map_merge'),
        'launch',
        'scenario_scan_map_merge.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_merge_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])

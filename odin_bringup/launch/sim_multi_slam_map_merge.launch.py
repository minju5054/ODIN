from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')

    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('odin_gazebo'),
        'launch',
        'house_easier_three_robots.launch.py',
    ])
    slam_merge_launch = PathJoinSubstitution([
        FindPackageShare('odin_bringup'),
        'launch',
        'multi_slam_map_merge.launch.py',
    ])
    exploration_launch = PathJoinSubstitution([
        FindPackageShare('odin_exploration'),
        'launch',
        'reactive_scouts.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo client.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('odin_gazebo'),
                'worlds',
                'odin_rescue_20x20_c.world',
            ]),
            description='Optional Gazebo world file passed through to odin_gazebo.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'gui': gui,
                'world': world,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_merge_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(exploration_launch),
                ),
            ],
        ),
    ])

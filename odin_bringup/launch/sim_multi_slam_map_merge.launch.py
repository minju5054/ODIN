from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    start_detection = LaunchConfiguration('start_detection')
    start_coordinator = LaunchConfiguration('start_coordinator')
    start_ai = LaunchConfiguration('start_ai')
    start_robot_3_dispatch = LaunchConfiguration('start_robot_3_dispatch')

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
    detection_launch = PathJoinSubstitution([
        FindPackageShare('odin_detection'),
        'launch',
        'gazebo_aruco_event_detector.launch.py',
    ])
    coordinator_launch = PathJoinSubstitution([
        FindPackageShare('odin_coordinator'),
        'launch',
        'rescue_coordinator.launch.py',
    ])
    ai_launch = PathJoinSubstitution([
        FindPackageShare('odin_ai'),
        'launch',
        'heuristic_waypoint_recommender.launch.py',
    ])
    robot_3_dispatch_launch = PathJoinSubstitution([
        FindPackageShare('odin_navigation'),
        'launch',
        'robot_3_simple_dispatch.launch.py',
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
        DeclareLaunchArgument(
            'start_detection',
            default_value='true',
            description='Start Gazebo-based ArUco hostage event detector.',
        ),
        DeclareLaunchArgument(
            'start_coordinator',
            default_value='true',
            description='Start hostage event validation and rescue candidate coordinator.',
        ),
        DeclareLaunchArgument(
            'start_ai',
            default_value='true',
            description='Start local heuristic AI waypoint recommender.',
        ),
        DeclareLaunchArgument(
            'start_robot_3_dispatch',
            default_value='true',
            description='Start conservative robot_3 goal follower.',
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
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(detection_launch),
                    launch_arguments={'use_sim_time': use_sim_time}.items(),
                    condition=IfCondition(start_detection),
                ),
            ],
        ),
        TimerAction(
            period=9.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(coordinator_launch),
                    launch_arguments={'use_sim_time': use_sim_time}.items(),
                    condition=IfCondition(start_coordinator),
                ),
            ],
        ),
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(ai_launch),
                    launch_arguments={'use_sim_time': use_sim_time}.items(),
                    condition=IfCondition(start_ai),
                ),
            ],
        ),
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(robot_3_dispatch_launch),
                    launch_arguments={'use_sim_time': use_sim_time}.items(),
                    condition=IfCondition(start_robot_3_dispatch),
                ),
            ],
        ),
    ])

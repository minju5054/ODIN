import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('ai_params_file')
    battlefield_config_file = LaunchConfiguration('battlefield_config_file')
    start_mission_intent_gui = LaunchConfiguration('start_mission_intent_gui')
    default_params = os.path.join(
        get_package_share_directory('odin_ai'),
        'config',
        'virtual_qwen_planner.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'ai_params_file',
            default_value=default_params,
            description='Virtual Qwen planner parameter file.',
        ),
        DeclareLaunchArgument(
            'battlefield_config_file',
            default_value='',
            description='Shared battlefield rules YAML file.',
        ),
        DeclareLaunchArgument(
            'start_mission_intent_gui',
            default_value='true',
            description='Start the desktop mission intent input panel.',
        ),
        Node(
            package='odin_ai',
            executable='mission_intent_panel',
            name='mission_intent_panel',
            output='screen',
            condition=IfCondition(start_mission_intent_gui),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='odin_ai',
            executable='virtual_qwen_planner',
            name='virtual_qwen_planner',
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

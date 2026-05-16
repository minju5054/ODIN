import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


ROBOTS = (
    {'name': 'robot_1', 'x': '-7.5', 'y': '7.5', 'yaw': '-1.5708'},
    {'name': 'robot_2', 'x': '7.5', 'y': '-7.5', 'yaw': '1.5708'},
)


def _add_rgb_camera(model, namespace):
    camera_link = ET.SubElement(model, 'link', {'name': 'camera_link'})

    visual = ET.SubElement(camera_link, 'visual', {'name': 'camera_visual'})
    visual_pose = ET.SubElement(visual, 'pose')
    visual_pose.text = '0 0 0 0 0 0'
    visual_geometry = ET.SubElement(visual, 'geometry')
    visual_box = ET.SubElement(visual_geometry, 'box')
    visual_size = ET.SubElement(visual_box, 'size')
    visual_size.text = '0.035 0.055 0.035'
    visual_material = ET.SubElement(visual, 'material')
    visual_ambient = ET.SubElement(visual_material, 'ambient')
    visual_ambient.text = '0.02 0.02 0.02 1'
    visual_diffuse = ET.SubElement(visual_material, 'diffuse')
    visual_diffuse.text = '0.02 0.02 0.02 1'

    sensor = ET.SubElement(camera_link, 'sensor', {'name': 'rgb_camera', 'type': 'camera'})
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'visualize').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '15'
    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'horizontal_fov').text = '2.0943951'
    image = ET.SubElement(camera, 'image')
    ET.SubElement(image, 'width').text = '640'
    ET.SubElement(image, 'height').text = '480'
    ET.SubElement(image, 'format').text = 'R8G8B8'
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '0.05'
    ET.SubElement(clip, 'far').text = '5.0'

    plugin = ET.SubElement(
        sensor,
        'plugin',
        {'name': 'rgb_camera_controller', 'filename': 'libgazebo_ros_camera.so'},
    )
    ros = ET.SubElement(plugin, 'ros')
    ET.SubElement(ros, 'remapping').text = 'image_raw:=camera/image_raw'
    ET.SubElement(ros, 'remapping').text = 'camera_info:=camera/camera_info'
    ET.SubElement(plugin, 'camera_name').text = 'camera'
    ET.SubElement(plugin, 'frame_name').text = f'{namespace}/camera_link'

    joint = ET.SubElement(model, 'joint', {'name': 'camera_joint', 'type': 'fixed'})
    ET.SubElement(joint, 'parent').text = 'base_link'
    ET.SubElement(joint, 'child').text = 'camera_link'
    ET.SubElement(joint, 'pose').text = '0.10 0 0.28 0 0.3491 0'


def _set_lidar_range(model, max_range):
    if model is None:
        return
    for sensor in model.iter('sensor'):
        if sensor.get('type') != 'ray':
            continue
        visualize_tag = sensor.find('visualize')
        if visualize_tag is not None:
            visualize_tag.text = 'true'
        range_tag = sensor.find('./ray/range/max')
        if range_tag is not None:
            range_tag.text = f'{max_range:.1f}'


def _patched_turtlebot_sdf(namespace):
    source = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_burger',
        'model.sdf',
    )
    tree = ET.parse(source)
    root = tree.getroot()

    model = root.find('model')
    if model is not None:
        model.set('name', namespace)

    for odom_frame_tag in root.iter('odometry_frame'):
        odom_frame_tag.text = f'{namespace}/odom'
    for base_frame_tag in root.iter('robot_base_frame'):
        base_frame_tag.text = f'{namespace}/base_footprint'
    for scan_frame_tag in root.iter('frame_name'):
        scan_frame_tag.text = f'{namespace}/base_scan'
    if model is not None:
        _set_lidar_range(model, 1.9)
        _add_rgb_camera(model, namespace)
    patched = ET.tostring(root, encoding='unicode')
    path = os.path.join(tempfile.gettempdir(), f'odin_{namespace}_burger.sdf')
    with open(path, 'w', encoding='utf-8') as sdf_file:
        sdf_file.write('<?xml version="1.0" ?>\n')
        sdf_file.write(patched)
    return path


def _robot_state_publisher(namespace, use_sim_time):
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
            'frame_prefix': f'{namespace}/',
        }],
    )


def _camera_static_transform(namespace, use_sim_time):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_transform_publisher',
        output='screen',
        arguments=[
            '0.10', '0', '0.28',
            '0', '0.3491', '0',
            f'{namespace}/base_link',
            f'{namespace}/camera_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )


def _spawn_robot(robot, sdf_path):
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot['name'],
            '-file', sdf_path,
            '-x', robot['x'],
            '-y', robot['y'],
            '-z', '0.01',
            '-Y', robot['yaw'],
            '-robot_namespace', robot['name'],
        ],
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    odin_gazebo_share = get_package_share_directory('odin_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    world = LaunchConfiguration('world')
    default_world = os.path.join(odin_gazebo_share, 'worlds', 'odin_rescue_20x20_c.world')

    patched_sdf_paths = {
        robot['name']: _patched_turtlebot_sdf(robot['name'])
        for robot in ROBOTS
    }

    actions = [
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
            default_value=default_world,
            description='Gazebo world file. Defaults to tactical map C.',
        ),
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger'),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                os.path.join(odin_gazebo_share, 'models'),
                ':',
                os.path.join(turtlebot3_gazebo_share, 'models'),
                ':',
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
            ),
            condition=IfCondition(gui),
        ),
    ]

    for robot in ROBOTS:
        namespace = robot['name']
        actions.append(
            GroupAction([
                PushRosNamespace(namespace),
                _robot_state_publisher(namespace, use_sim_time),
                _camera_static_transform(namespace, use_sim_time),
                _spawn_robot(robot, patched_sdf_paths[namespace]),
            ])
        )

    actions.append(
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda event, context: [
                    os.remove(path) for path in patched_sdf_paths.values()
                    if os.path.exists(path)
                ]
            )
        )
    )

    return LaunchDescription(actions)

import os
import xml.etree.ElementTree as ET
import math
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Robot3SpawnOnGoal(Node):
    """Spawn robot_3 only after a validated dispatch goal is published."""

    def __init__(self) -> None:
        super().__init__('robot_3_spawn_on_goal')

        self.declare_parameter('goal_topic', '/robot_3/goal_pose')
        self.declare_parameter('status_topic', '/robot_3/dispatch_status')
        self.declare_parameter('robot_name', 'robot_3')
        self.declare_parameter('spawn_x', -7.5)
        self.declare_parameter('spawn_y', -7.5)
        self.declare_parameter('spawn_z', 0.01)
        self.declare_parameter('spawn_yaw', 0.785398)
        self.declare_parameter('spawn_service', '/spawn_entity')

        self.robot_name = str(self.get_parameter('robot_name').value)
        self.spawn_requested = False
        self.spawned = False
        self.pending_goal: Optional[PoseStamped] = None

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.spawn_client = self.create_client(
            SpawnEntity,
            str(self.get_parameter('spawn_service').value),
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._goal_callback,
            10,
        )
        self.create_timer(0.5, self._spawn_loop)

        self._publish_status('spawn_manager_ready waiting_for_validated_goal')

    def _goal_callback(self, msg: PoseStamped) -> None:
        self.pending_goal = msg
        if self.spawned:
            self._publish_status('spawn_skipped reason=robot_3_already_spawned')
        else:
            self._publish_status(
                f'spawn_trigger_received goal_x={msg.pose.position.x:.2f} '
                f'goal_y={msg.pose.position.y:.2f}'
            )

    def _spawn_loop(self) -> None:
        if self.pending_goal is None or self.spawned or self.spawn_requested:
            return

        if not self.spawn_client.service_is_ready():
            self._publish_status('spawn_waiting reason=spawn_entity_service_unavailable')
            return

        request = SpawnEntity.Request()
        request.name = self.robot_name
        request.xml = self._patched_turtlebot_sdf(self.robot_name)
        request.robot_namespace = self.robot_name
        request.initial_pose = self._spawn_pose()

        self.spawn_requested = True
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self._spawn_done_callback)
        self._publish_status('spawn_requested robot=robot_3')

    def _spawn_done_callback(self, future) -> None:
        self.spawn_requested = False
        try:
            response = future.result()
        except Exception as exc:
            self._publish_status(f'spawn_failed exception={exc}')
            return

        if response.success:
            self.spawned = True
            self._publish_status('spawn_succeeded robot=robot_3')
        else:
            message = response.status_message.replace(' ', '_')
            if 'already' in response.status_message.lower():
                self.spawned = True
                self._publish_status(f'spawn_assumed_ready status={message}')
            else:
                self._publish_status(f'spawn_failed status={message}')

    def _patched_turtlebot_sdf(self, namespace: str) -> str:
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
            self._set_lidar_range(model, 2.2)
            self._add_rescue_visual_marker(model)

        return '<?xml version="1.0" ?>\n' + ET.tostring(root, encoding='unicode')

    @staticmethod
    def _set_lidar_range(model, max_range: float) -> None:
        for sensor in model.iter('sensor'):
            if sensor.get('type') != 'ray':
                continue
            visualize_tag = sensor.find('visualize')
            if visualize_tag is not None:
                visualize_tag.text = 'false'
            range_tag = sensor.find('./ray/range/max')
            if range_tag is not None:
                range_tag.text = f'{max_range:.1f}'

    @staticmethod
    def _add_rescue_visual_marker(model) -> None:
        link = ET.SubElement(model, 'link', {'name': 'rescue_beacon_link'})
        visual = ET.SubElement(link, 'visual', {'name': 'rescue_beacon_visual'})
        geometry = ET.SubElement(visual, 'geometry')
        cylinder = ET.SubElement(geometry, 'cylinder')
        ET.SubElement(cylinder, 'radius').text = '0.11'
        ET.SubElement(cylinder, 'length').text = '0.32'
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = '0.0 0.35 1.0 1'
        ET.SubElement(material, 'diffuse').text = '0.0 0.45 1.0 1'
        ET.SubElement(material, 'emissive').text = '0.0 0.25 1.0 1'

        joint = ET.SubElement(model, 'joint', {'name': 'rescue_beacon_joint', 'type': 'fixed'})
        ET.SubElement(joint, 'parent').text = 'base_link'
        ET.SubElement(joint, 'child').text = 'rescue_beacon_link'
        ET.SubElement(joint, 'pose').text = '0 0 0.42 0 0 0'

    def _spawn_pose(self) -> Pose:
        pose = Pose()
        pose.position.x = float(self.get_parameter('spawn_x').value)
        pose.position.y = float(self.get_parameter('spawn_y').value)
        pose.position.z = float(self.get_parameter('spawn_z').value)
        yaw = float(self.get_parameter('spawn_yaw').value)
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = Robot3SpawnOnGoal()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

import math
from typing import Dict, List, Optional

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from odin_interfaces.msg import HostageEvent
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class GazeboArucoEventDetector(Node):
    """Publish hostage events when scouts can see the Gazebo ArUco surrogate."""

    def __init__(self) -> None:
        super().__init__('gazebo_aruco_event_detector')

        self.declare_parameter('robot_names', ['robot_1', 'robot_2'])
        self.declare_parameter('marker_model_name', 'hostage_aruco_marker_0')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('model_states_topic', '/model_states')
        self.declare_parameter('event_topic', '/hostage_events')
        self.declare_parameter('victim_event_topic', '')
        self.declare_parameter('detection_range_m', 4.0)
        self.declare_parameter('detection_fov_deg', 100.0)
        self.declare_parameter('event_min_period_sec', 3.0)

        self.robot_names: List[str] = [
            str(name) for name in self.get_parameter('robot_names').value
        ]
        self.marker_model_name = str(self.get_parameter('marker_model_name').value)
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.detection_range_m = float(self.get_parameter('detection_range_m').value)
        self.detection_half_fov = math.radians(
            float(self.get_parameter('detection_fov_deg').value) / 2.0
        )
        self.event_min_period_sec = float(self.get_parameter('event_min_period_sec').value)
        self.last_event_time_by_robot: Dict[str, float] = {}

        event_topic = str(self.get_parameter('event_topic').value)
        victim_event_topic = str(self.get_parameter('victim_event_topic').value)
        model_states_topic = str(self.get_parameter('model_states_topic').value)

        self.event_pub = self.create_publisher(HostageEvent, event_topic, 10)
        self.victim_event_pub = (
            self.create_publisher(HostageEvent, victim_event_topic, 10)
            if victim_event_topic
            else None
        )
        self.create_subscription(ModelStates, model_states_topic, self._model_states_callback, 10)

        self.get_logger().info(
            'Gazebo ArUco event detector started: '
            f'robots={self.robot_names}, marker={self.marker_model_name}, '
            f'event_topic={event_topic}'
        )

    def _model_states_callback(self, msg: ModelStates) -> None:
        poses = dict(zip(msg.name, msg.pose))
        marker_pose = poses.get(self.marker_model_name)
        if marker_pose is None:
            self.get_logger().warn(
                f'Marker model "{self.marker_model_name}" not found in {self.get_parameter("model_states_topic").value}.',
                throttle_duration_sec=5.0,
            )
            return

        for robot_name in self.robot_names:
            robot_pose = poses.get(robot_name)
            if robot_pose is None:
                continue
            if not self._is_marker_visible(robot_pose, marker_pose):
                continue
            if self._event_is_throttled(robot_name):
                continue
            self._publish_event(robot_name, marker_pose)

    def _is_marker_visible(self, robot_pose: Pose, marker_pose: Pose) -> bool:
        dx = marker_pose.position.x - robot_pose.position.x
        dy = marker_pose.position.y - robot_pose.position.y
        distance = math.hypot(dx, dy)
        if distance > self.detection_range_m:
            return False

        robot_yaw = self._yaw_from_quaternion(
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w,
        )
        marker_bearing = math.atan2(dy, dx)
        relative_bearing = self._normalize_angle(marker_bearing - robot_yaw)
        return abs(relative_bearing) <= self.detection_half_fov

    def _publish_event(self, robot_name: str, marker_pose: Pose) -> None:
        event = HostageEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.header.frame_id = self.global_frame
        event.marker_id = self.marker_id
        event.detecting_robot = robot_name
        event.pose = marker_pose

        self.event_pub.publish(event)
        if self.victim_event_pub is not None:
            self.victim_event_pub.publish(event)

        self.last_event_time_by_robot[robot_name] = self._now_seconds()
        self.get_logger().info(
            f'Published hostage event: marker_id={event.marker_id}, '
            f'robot={robot_name}, frame={event.header.frame_id}, '
            f'x={event.pose.position.x:.2f}, y={event.pose.position.y:.2f}'
        )

    def _event_is_throttled(self, robot_name: str) -> bool:
        last_event_time = self.last_event_time_by_robot.get(robot_name)
        if last_event_time is None:
            return False
        return (self._now_seconds() - last_event_time) < self.event_min_period_sec

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = GazeboArucoEventDetector()
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

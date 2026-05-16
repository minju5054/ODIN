import math
from typing import Dict, List, Optional

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
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
        self.declare_parameter('merged_map_topic', '/merged_map')
        self.declare_parameter('detection_range_m', 4.0)
        self.declare_parameter('detection_fov_deg', 100.0)
        self.declare_parameter('event_min_period_sec', 3.0)
        self.declare_parameter('require_line_of_sight', True)
        self.declare_parameter('line_of_sight_step_m', 0.05)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('allow_unknown_line_of_sight', False)

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
        self.require_line_of_sight = bool(self.get_parameter('require_line_of_sight').value)
        self.line_of_sight_step_m = float(self.get_parameter('line_of_sight_step_m').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.allow_unknown_line_of_sight = bool(
            self.get_parameter('allow_unknown_line_of_sight').value
        )
        self.last_event_time_by_robot: Dict[str, float] = {}
        self.published_marker_ids = set()
        self.latest_map: Optional[OccupancyGrid] = None

        event_topic = str(self.get_parameter('event_topic').value)
        victim_event_topic = str(self.get_parameter('victim_event_topic').value)
        model_states_topic = str(self.get_parameter('model_states_topic').value)
        merged_map_topic = str(self.get_parameter('merged_map_topic').value)

        self.event_pub = self.create_publisher(HostageEvent, event_topic, 10)
        self.victim_event_pub = (
            self.create_publisher(HostageEvent, victim_event_topic, 10)
            if victim_event_topic
            else None
        )
        self.create_subscription(ModelStates, model_states_topic, self._model_states_callback, 10)
        self.create_subscription(OccupancyGrid, merged_map_topic, self._map_callback, 1)

        self.get_logger().info(
            'Gazebo ArUco event detector started: '
            f'robots={self.robot_names}, marker={self.marker_model_name}, '
            f'event_topic={event_topic}, require_line_of_sight={self.require_line_of_sight}'
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def _model_states_callback(self, msg: ModelStates) -> None:
        if self.marker_id in self.published_marker_ids:
            return

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
        if abs(relative_bearing) > self.detection_half_fov:
            return False

        if self.require_line_of_sight:
            return self._has_line_of_sight(
                robot_pose.position.x,
                robot_pose.position.y,
                marker_pose.position.x,
                marker_pose.position.y,
            )
        return True

    def _has_line_of_sight(self, start_x: float, start_y: float, end_x: float, end_y: float) -> bool:
        if self.latest_map is None:
            self.get_logger().debug('Line-of-sight rejected because /merged_map is not ready.')
            return False

        dx = end_x - start_x
        dy = end_y - start_y
        distance = math.hypot(dx, dy)
        if distance <= 1e-6:
            return True

        step_m = max(self.line_of_sight_step_m, self.latest_map.info.resolution)
        steps = max(1, int(math.ceil(distance / step_m)))
        # Skip the exact endpoints so the robot footprint and marker wall mounting do not reject itself.
        for index in range(1, steps):
            ratio = index / steps
            x = start_x + dx * ratio
            y = start_y + dy * ratio
            value = self._map_value_at(x, y)
            if value is None:
                return False
            if value < 0:
                if not self.allow_unknown_line_of_sight:
                    return False
                continue
            if value >= self.occupied_threshold:
                return False
        return True

    def _map_value_at(self, x: float, y: float) -> Optional[int]:
        if self.latest_map is None:
            return None
        origin = self.latest_map.info.origin.position
        resolution = self.latest_map.info.resolution
        cell_x = int(math.floor((x - origin.x) / resolution))
        cell_y = int(math.floor((y - origin.y) / resolution))
        if not (0 <= cell_x < self.latest_map.info.width and 0 <= cell_y < self.latest_map.info.height):
            return None
        return int(self.latest_map.data[cell_y * self.latest_map.info.width + cell_x])

    def _publish_event(self, robot_name: str, marker_pose: Pose) -> None:
        if self.marker_id in self.published_marker_ids:
            return

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
        self.published_marker_ids.add(self.marker_id)
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

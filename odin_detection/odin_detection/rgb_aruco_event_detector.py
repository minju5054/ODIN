import math
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from odin_interfaces.msg import HostageEvent
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class RgbArucoEventDetector(Node):
    """Publish hostage events when scout RGB cameras see ArUco marker ID 0."""

    def __init__(self) -> None:
        super().__init__('rgb_aruco_event_detector')

        self.declare_parameter('robot_names', ['robot_1', 'robot_2'])
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size_m', 0.40)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('event_topic', '/hostage_events')
        self.declare_parameter('victim_event_topic', '')
        self.declare_parameter('merged_map_topic', '/merged_map')
        self.declare_parameter('image_topic_suffix', 'camera/image_raw')
        self.declare_parameter('camera_info_topic_suffix', 'camera/camera_info')
        self.declare_parameter('odom_topic_suffix', 'odom')
        self.declare_parameter('detection_range_m', 4.0)
        self.declare_parameter('event_min_period_sec', 3.0)
        self.declare_parameter('camera_forward_offset_m', 0.10)
        self.declare_parameter('require_line_of_sight', False)
        self.declare_parameter('line_of_sight_step_m', 0.05)
        self.declare_parameter('line_of_sight_endpoint_skip_m', 0.25)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('allow_unknown_line_of_sight', True)

        self.robot_names: List[str] = [
            str(name) for name in self.get_parameter('robot_names').value
        ]
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.detection_range_m = float(self.get_parameter('detection_range_m').value)
        self.event_min_period_sec = float(self.get_parameter('event_min_period_sec').value)
        self.camera_forward_offset_m = float(
            self.get_parameter('camera_forward_offset_m').value
        )
        self.require_line_of_sight = bool(self.get_parameter('require_line_of_sight').value)
        self.line_of_sight_step_m = float(self.get_parameter('line_of_sight_step_m').value)
        self.line_of_sight_endpoint_skip_m = float(
            self.get_parameter('line_of_sight_endpoint_skip_m').value
        )
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.allow_unknown_line_of_sight = bool(
            self.get_parameter('allow_unknown_line_of_sight').value
        )

        dictionary_name = str(self.get_parameter('aruco_dictionary').value)
        dictionary_id = getattr(cv2.aruco, dictionary_name)
        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 35
        self.aruco_params.adaptiveThreshWinSizeStep = 4
        self.aruco_params.minMarkerPerimeterRate = 0.02
        self.aruco_params.polygonalApproxAccuracyRate = 0.08

        self.bridge = CvBridge()
        self.camera_info_by_robot: Dict[str, CameraInfo] = {}
        self.odom_by_robot: Dict[str, Odometry] = {}
        self.last_event_time_by_robot: Dict[str, float] = {}
        self.published_marker_ids = set()
        self.latest_map: Optional[OccupancyGrid] = None

        event_topic = str(self.get_parameter('event_topic').value)
        victim_event_topic = str(self.get_parameter('victim_event_topic').value)
        merged_map_topic = str(self.get_parameter('merged_map_topic').value)

        self.event_pub = self.create_publisher(HostageEvent, event_topic, 10)
        self.victim_event_pub = (
            self.create_publisher(HostageEvent, victim_event_topic, 10)
            if victim_event_topic
            else None
        )
        self.create_subscription(OccupancyGrid, merged_map_topic, self._map_callback, 1)

        image_suffix = str(self.get_parameter('image_topic_suffix').value).strip('/')
        info_suffix = str(self.get_parameter('camera_info_topic_suffix').value).strip('/')
        odom_suffix = str(self.get_parameter('odom_topic_suffix').value).strip('/')
        for robot_name in self.robot_names:
            self.create_subscription(
                Image,
                f'/{robot_name}/{image_suffix}',
                lambda msg, name=robot_name: self._image_callback(name, msg),
                10,
            )
            self.create_subscription(
                CameraInfo,
                f'/{robot_name}/{info_suffix}',
                lambda msg, name=robot_name: self._camera_info_callback(name, msg),
                10,
            )
            self.create_subscription(
                Odometry,
                f'/{robot_name}/{odom_suffix}',
                lambda msg, name=robot_name: self._odom_callback(name, msg),
                10,
            )

        self.get_logger().info(
            'RGB ArUco event detector started: '
            f'robots={self.robot_names}, marker_id={self.marker_id}, '
            f'event_topic={event_topic}, require_line_of_sight={self.require_line_of_sight}'
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def _camera_info_callback(self, robot_name: str, msg: CameraInfo) -> None:
        self.camera_info_by_robot[robot_name] = msg

    def _odom_callback(self, robot_name: str, msg: Odometry) -> None:
        self.odom_by_robot[robot_name] = msg

    def _image_callback(self, robot_name: str, msg: Image) -> None:
        if self.marker_id in self.published_marker_ids:
            return
        if self._event_is_throttled(robot_name):
            return

        camera_info = self.camera_info_by_robot.get(robot_name)
        odom = self.odom_by_robot.get(robot_name)
        if camera_info is None or odom is None:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image from {robot_name}: {exc}')
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dictionary,
            parameters=self.aruco_params,
        )
        if ids is None:
            return

        for index, marker_id in enumerate(ids.flatten()):
            if int(marker_id) != self.marker_id:
                continue

            pose = self._estimate_marker_pose(corners[index], camera_info, odom)
            if pose is None:
                continue

            distance = math.hypot(
                pose.position.x - odom.pose.pose.position.x,
                pose.position.y - odom.pose.pose.position.y,
            )
            if distance > self.detection_range_m:
                continue

            if self.require_line_of_sight and not self._has_line_of_sight(
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                pose.position.x,
                pose.position.y,
            ):
                continue

            self._publish_event(robot_name, pose)
            return

    def _estimate_marker_pose(
        self,
        marker_corners,
        camera_info: CameraInfo,
        odom: Odometry,
    ) -> Optional[Pose]:
        camera_matrix = self._camera_matrix(camera_info)
        if camera_matrix is None:
            return None

        dist_coeffs = self._distortion_coefficients(camera_info)
        _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [marker_corners],
            self.marker_size_m,
            camera_matrix,
            dist_coeffs,
        )
        if tvecs is None or len(tvecs) == 0:
            return None

        tvec = tvecs[0][0]
        right_m = float(tvec[0])
        forward_m = float(tvec[2])
        if forward_m <= 0.0:
            return None

        left_m = -right_m
        base_forward_m = forward_m + self.camera_forward_offset_m

        robot_pose = odom.pose.pose
        robot_yaw = self._yaw_from_quaternion(
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w,
        )

        pose = Pose()
        pose.position.x = (
            robot_pose.position.x
            + math.cos(robot_yaw) * base_forward_m
            - math.sin(robot_yaw) * left_m
        )
        pose.position.y = (
            robot_pose.position.y
            + math.sin(robot_yaw) * base_forward_m
            + math.cos(robot_yaw) * left_m
        )
        pose.position.z = 0.0
        yaw = math.atan2(
            pose.position.y - robot_pose.position.y,
            pose.position.x - robot_pose.position.x,
        )
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = self._quaternion_from_yaw(yaw)
        return pose

    def _camera_matrix(self, camera_info: CameraInfo):
        if len(camera_info.k) != 9 or camera_info.k[0] <= 0.0:
            return None
        return np.array(
            [
                [camera_info.k[0], camera_info.k[1], camera_info.k[2]],
                [camera_info.k[3], camera_info.k[4], camera_info.k[5]],
                [camera_info.k[6], camera_info.k[7], camera_info.k[8]],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _distortion_coefficients(camera_info: CameraInfo):
        return np.array(camera_info.d if camera_info.d else [0.0], dtype=np.float64)

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
        endpoint_skip_steps = int(
            math.ceil(max(self.line_of_sight_endpoint_skip_m, 0.0) / step_m)
        )
        end_index = max(1, steps - endpoint_skip_steps)
        for index in range(1, end_index):
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
            f'Published RGB hostage event: marker_id={event.marker_id}, '
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
    def _quaternion_from_yaw(yaw: float):
        return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = RgbArucoEventDetector()
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

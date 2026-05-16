import math
from typing import Dict, List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class RobotScanFilter(Node):
    """Remove known scout robot returns from robot_3 LaserScan for Nav2 costmaps."""

    def __init__(self) -> None:
        super().__init__('robot_scan_filter')

        self.declare_parameter('input_scan_topic', '/robot_3/scan')
        self.declare_parameter('filtered_scan_topic', '/robot_3/scan_nav2')
        self.declare_parameter('robot_3_odom_topic', '/robot_3/odom')
        self.declare_parameter('ignored_robot_odom_topics', ['/robot_1/odom', '/robot_2/odom'])
        self.declare_parameter('ignore_distance_m', 2.2)
        self.declare_parameter('ignored_robot_radius_m', 0.36)
        self.declare_parameter('range_tolerance_m', 0.30)
        self.declare_parameter('angle_margin_rad', 0.12)
        self.declare_parameter('odom_timeout_sec', 1.5)
        self.declare_parameter('dispatch_status_topic', '/robot_3/dispatch_status')
        self.declare_parameter('blind_start_distance_m', 3.0)

        self.ignore_distance_m = float(self.get_parameter('ignore_distance_m').value)
        self.ignored_robot_radius_m = float(self.get_parameter('ignored_robot_radius_m').value)
        self.range_tolerance_m = float(self.get_parameter('range_tolerance_m').value)
        self.angle_margin_rad = float(self.get_parameter('angle_margin_rad').value)
        self.odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self.blind_start_distance_m = float(self.get_parameter('blind_start_distance_m').value)
        self.robot_3_odom: Optional[Odometry] = None
        self.ignored_odoms: Dict[str, Odometry] = {}
        self.blind_start_active = False
        self.blind_start_origin: Optional[tuple] = None

        self.filtered_scan_pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter('filtered_scan_topic').value),
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('robot_3_odom_topic').value),
            self._robot_3_odom_callback,
            10,
        )
        for topic in self.get_parameter('ignored_robot_odom_topics').value:
            topic_name = str(topic)
            self.create_subscription(
                Odometry,
                topic_name,
                lambda msg, name=topic_name: self._ignored_odom_callback(name, msg),
                10,
            )
        self.create_subscription(
            String,
            str(self.get_parameter('dispatch_status_topic').value),
            self._dispatch_status_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('input_scan_topic').value),
            self._scan_callback,
            10,
        )

        self.get_logger().info(
            'Robot scan filter ready: '
            f'{self.get_parameter("input_scan_topic").value} -> '
            f'{self.get_parameter("filtered_scan_topic").value}, '
            f'blind_start_distance={self.blind_start_distance_m:.2f}m'
        )

    def _robot_3_odom_callback(self, msg: Odometry) -> None:
        self.robot_3_odom = msg
        if self.blind_start_active and self.blind_start_origin is None:
            pose = msg.pose.pose.position
            self.blind_start_origin = (pose.x, pose.y)
            self.get_logger().info(
                'robot_3_blind_start_origin_set '
                f'x={pose.x:.2f} y={pose.y:.2f} distance={self.blind_start_distance_m:.2f}'
            )

    def _ignored_odom_callback(self, topic_name: str, msg: Odometry) -> None:
        self.ignored_odoms[topic_name] = msg

    def _dispatch_status_callback(self, msg: String) -> None:
        if (
            'selected_path_follow_queued' not in msg.data
            and 'nav2_goal_queued' not in msg.data
        ):
            return
        if self.blind_start_distance_m <= 0.0:
            return
        self.blind_start_active = True
        self.blind_start_origin = None
        if self.robot_3_odom is not None:
            pose = self.robot_3_odom.pose.pose.position
            self.blind_start_origin = (pose.x, pose.y)
        self.get_logger().info(
            'robot_3_blind_start_enabled '
            f'distance={self.blind_start_distance_m:.2f} status={msg.data}'
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.intensities = list(msg.intensities)
        filtered.ranges = list(msg.ranges)

        if self._blind_start_should_suppress_scan():
            self._clear_all_ranges(filtered)
            self.filtered_scan_pub.publish(filtered)
            return

        if self.robot_3_odom is None or msg.angle_increment == 0.0:
            self.filtered_scan_pub.publish(filtered)
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        robot_3_pose = self.robot_3_odom.pose.pose
        robot_3_stamp = self._stamp_to_sec(self.robot_3_odom)
        if now_sec - robot_3_stamp > self.odom_timeout_sec:
            self.filtered_scan_pub.publish(filtered)
            return

        robot_3_x = robot_3_pose.position.x
        robot_3_y = robot_3_pose.position.y
        robot_3_yaw = self._yaw_from_odom(self.robot_3_odom)

        masks = self._robot_masks(robot_3_x, robot_3_y, robot_3_yaw, now_sec)
        for center_angle, distance, half_width in masks:
            self._clear_scan_window(filtered, center_angle, distance, half_width)

        self.filtered_scan_pub.publish(filtered)

    def _blind_start_should_suppress_scan(self) -> bool:
        if not self.blind_start_active:
            return False
        if self.robot_3_odom is None:
            return True
        pose = self.robot_3_odom.pose.pose.position
        if self.blind_start_origin is None:
            self.blind_start_origin = (pose.x, pose.y)
            return True
        traveled = math.hypot(pose.x - self.blind_start_origin[0], pose.y - self.blind_start_origin[1])
        if traveled < self.blind_start_distance_m:
            return True
        self.blind_start_active = False
        self.blind_start_origin = None
        self.get_logger().info(
            f'robot_3_blind_start_complete traveled={traveled:.2f}m; scan obstacle filtering restored'
        )
        return False

    @staticmethod
    def _clear_all_ranges(scan: LaserScan) -> None:
        scan.ranges = [
            float('inf') if math.isfinite(value) and value >= scan.range_min else value
            for value in scan.ranges
        ]

    def _robot_masks(
        self,
        robot_3_x: float,
        robot_3_y: float,
        robot_3_yaw: float,
        now_sec: float,
    ) -> List[tuple]:
        masks = []
        for odom in self.ignored_odoms.values():
            if now_sec - self._stamp_to_sec(odom) > self.odom_timeout_sec:
                continue
            pose = odom.pose.pose
            dx = pose.position.x - robot_3_x
            dy = pose.position.y - robot_3_y
            distance = math.hypot(dx, dy)
            if distance <= 1e-6 or distance > self.ignore_distance_m:
                continue
            center_angle = self._normalize_angle(math.atan2(dy, dx) - robot_3_yaw)
            half_width = math.asin(min(0.95, self.ignored_robot_radius_m / distance))
            half_width += self.angle_margin_rad
            masks.append((center_angle, distance, half_width))
        return masks

    def _clear_scan_window(
        self,
        scan: LaserScan,
        center_angle: float,
        distance: float,
        half_width: float,
    ) -> None:
        min_range = max(scan.range_min, distance - self.ignored_robot_radius_m - self.range_tolerance_m)
        max_range = min(scan.range_max, distance + self.ignored_robot_radius_m + self.range_tolerance_m)
        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value) or value < min_range or value > max_range:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            if abs(self._normalize_angle(angle - center_angle)) <= half_width:
                scan.ranges[index] = float('inf')

    @staticmethod
    def _stamp_to_sec(msg: Odometry) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    @staticmethod
    def _yaw_from_odom(msg: Odometry) -> float:
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
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
    node = RobotScanFilter()
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

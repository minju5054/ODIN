import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SimpleGoalFollower(Node):
    """Conservative goal follower for the initial robot_3 dispatch milestone."""

    def __init__(self) -> None:
        super().__init__('simple_goal_follower')

        self.declare_parameter('goal_topic', 'goal_pose')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('status_topic', 'dispatch_status')
        self.declare_parameter('goal_tolerance_m', 0.25)
        self.declare_parameter('yaw_tolerance_rad', 0.35)
        self.declare_parameter('max_linear_speed', 0.16)
        self.declare_parameter('max_angular_speed', 0.75)
        self.declare_parameter('heading_gain', 1.4)
        self.declare_parameter('distance_slowdown_m', 1.0)
        self.declare_parameter('obstacle_stop_distance_m', 0.34)
        self.declare_parameter('front_fov_deg', 35.0)
        self.declare_parameter('side_fov_deg', 70.0)
        self.declare_parameter('avoid_turn_speed', 0.55)
        self.declare_parameter('avoid_clear_distance_m', 0.55)

        self.goal_tolerance_m = float(self.get_parameter('goal_tolerance_m').value)
        self.yaw_tolerance_rad = float(self.get_parameter('yaw_tolerance_rad').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.distance_slowdown_m = float(self.get_parameter('distance_slowdown_m').value)
        self.obstacle_stop_distance_m = float(self.get_parameter('obstacle_stop_distance_m').value)
        self.front_fov = math.radians(float(self.get_parameter('front_fov_deg').value))
        self.side_fov = math.radians(float(self.get_parameter('side_fov_deg').value))
        self.avoid_turn_speed = float(self.get_parameter('avoid_turn_speed').value)
        self.avoid_clear_distance_m = float(self.get_parameter('avoid_clear_distance_m').value)

        self.goal: Optional[PoseStamped] = None
        self.pose: Optional[Pose] = None
        self.front_clearance = math.inf
        self.left_clearance = math.inf
        self.right_clearance = math.inf
        self.avoid_turn_direction = 1.0
        self.active = False

        self.cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._goal_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._scan_callback,
            10,
        )
        self.create_timer(0.1, self._control_loop)

        self._publish_status('dispatch_follower_ready waiting_for_goal')

    def _goal_callback(self, msg: PoseStamped) -> None:
        self.goal = msg
        self.active = True
        self._publish_status(
            f'goal_received x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self.pose = msg.pose.pose

    def _scan_callback(self, msg: LaserScan) -> None:
        front_ranges = []
        left_ranges = []
        right_ranges = []
        half_fov = self.front_fov / 2.0
        for index, value in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            normalized = self._normalize_angle(angle)
            if not (math.isfinite(value) and msg.range_min <= value <= msg.range_max):
                continue
            if abs(normalized) <= half_fov:
                front_ranges.append(value)
            elif 0.0 < normalized <= self.side_fov:
                left_ranges.append(value)
            elif -self.side_fov <= normalized < 0.0:
                right_ranges.append(value)
        self.front_clearance = min(front_ranges) if front_ranges else math.inf
        self.left_clearance = min(left_ranges) if left_ranges else math.inf
        self.right_clearance = min(right_ranges) if right_ranges else math.inf

    def _control_loop(self) -> None:
        cmd = Twist()
        if not self.active or self.goal is None or self.pose is None:
            self.cmd_pub.publish(cmd)
            return

        dx = self.goal.pose.position.x - self.pose.position.x
        dy = self.goal.pose.position.y - self.pose.position.y
        distance = math.hypot(dx, dy)
        current_yaw = self._yaw_from_pose(self.pose)
        target_yaw = math.atan2(dy, dx)
        heading_error = self._normalize_angle(target_yaw - current_yaw)

        if distance <= self.goal_tolerance_m:
            final_yaw = self._yaw_from_pose(self.goal.pose)
            yaw_error = self._normalize_angle(final_yaw - current_yaw)
            if abs(yaw_error) <= self.yaw_tolerance_rad:
                self.active = False
                self.cmd_pub.publish(cmd)
                self._publish_status('goal_reached')
                return
            cmd.angular.z = self._clamp(
                self.heading_gain * yaw_error,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            self.cmd_pub.publish(cmd)
            return

        if self.front_clearance < self.obstacle_stop_distance_m:
            self.avoid_turn_direction = 1.0 if self.left_clearance >= self.right_clearance else -1.0
            cmd.angular.z = self.avoid_turn_direction * min(
                abs(self.avoid_turn_speed),
                self.max_angular_speed,
            )
            self.cmd_pub.publish(cmd)
            self._publish_status(
                'dispatch_avoiding_obstacle '
                f'front={self.front_clearance:.2f} left={self.left_clearance:.2f} '
                f'right={self.right_clearance:.2f}'
            )
            return

        if self.front_clearance < self.avoid_clear_distance_m and abs(heading_error) < 0.45:
            self.avoid_turn_direction = 1.0 if self.left_clearance >= self.right_clearance else -1.0
            cmd.linear.x = self.max_linear_speed * 0.35
            cmd.angular.z = self.avoid_turn_direction * min(
                abs(self.avoid_turn_speed) * 0.65,
                self.max_angular_speed,
            )
            self.cmd_pub.publish(cmd)
            return

        cmd.angular.z = self._clamp(
            self.heading_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        heading_scale = max(0.0, 1.0 - min(abs(heading_error) / 1.2, 1.0))
        distance_scale = min(distance / max(self.distance_slowdown_m, 0.1), 1.0)
        cmd.linear.x = self.max_linear_speed * heading_scale * distance_scale
        self.cmd_pub.publish(cmd)

    @staticmethod
    def _yaw_from_pose(pose: Pose) -> float:
        q = pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(value, high))

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = SimpleGoalFollower()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

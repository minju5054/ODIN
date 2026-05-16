import math
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from odin_interfaces.msg import HostageEvent
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class DriveMode(Enum):
    CRUISE = 'cruise'
    ESCAPE = 'escape'
    COMMITTED_TURN = 'committed_turn'


class ReactiveScout(Node):
    """Reactive SLAM scout with gap following, committed turns, and escape behavior."""

    def __init__(self) -> None:
        super().__init__('reactive_scout')

        self.declare_parameter('forward_speed', 0.20)
        self.declare_parameter('slow_speed', 0.08)
        self.declare_parameter('max_turn_speed', 0.75)
        self.declare_parameter('stop_distance', 0.34)
        self.declare_parameter('free_distance', 0.55)
        self.declare_parameter('slow_distance', 0.90)
        self.declare_parameter('field_of_view_deg', 170.0)
        self.declare_parameter('front_angle_deg', 18.0)
        self.declare_parameter('target_smoothing', 0.35)
        self.declare_parameter('preferred_turn_direction', 1.0)
        self.declare_parameter('backup_speed', -0.04)
        self.declare_parameter('escape_duration', 1.4)
        self.declare_parameter('backup_duration', 0.35)
        self.declare_parameter('turn_commit_duration', 0.9)
        self.declare_parameter('explore_bias_speed', 0.12)
        self.declare_parameter('explore_bias_period', 12.0)
        self.declare_parameter('side_clearance_weight', 0.45)
        self.declare_parameter('enable_center_spiral', False)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('center_spiral_weight', 0.35)
        self.declare_parameter('center_spiral_turn_direction', -1.0)
        self.declare_parameter('center_spiral_max_angle_deg', 45.0)
        self.declare_parameter('center_spiral_min_radius', 1.5)
        self.declare_parameter('hostage_event_topic', '/hostage_events')
        self.declare_parameter('enable_outer_explore_after_event', True)
        self.declare_parameter('outer_explore_weight', 0.45)
        self.declare_parameter('outer_explore_tangent_weight', 0.35)
        self.declare_parameter('outer_explore_max_angle_deg', 55.0)
        self.declare_parameter('outer_explore_min_radius', 1.2)
        self.declare_parameter('map_min_x', -9.2)
        self.declare_parameter('map_max_x', 9.2)
        self.declare_parameter('map_min_y', -9.2)
        self.declare_parameter('map_max_y', 9.2)
        self.declare_parameter('boundary_margin_m', 1.0)
        self.declare_parameter('red_zone_min_x', 4.0)
        self.declare_parameter('red_zone_max_x', 10.0)
        self.declare_parameter('red_zone_min_y', 4.0)
        self.declare_parameter('red_zone_max_y', 10.0)
        self.declare_parameter('red_zone_avoid_margin_m', 1.5)
        self.declare_parameter('red_zone_predict_horizon_m', 1.4)
        self.declare_parameter('red_zone_turn_away_weight', 0.85)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.slow_speed = float(self.get_parameter('slow_speed').value)
        self.max_turn_speed = float(self.get_parameter('max_turn_speed').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.free_distance = float(self.get_parameter('free_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.field_of_view = math.radians(float(self.get_parameter('field_of_view_deg').value))
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.target_smoothing = float(self.get_parameter('target_smoothing').value)
        preferred_turn = float(self.get_parameter('preferred_turn_direction').value)
        self.preferred_turn_direction = -1.0 if preferred_turn < 0.0 else 1.0
        self.backup_speed = float(self.get_parameter('backup_speed').value)
        self.escape_duration = float(self.get_parameter('escape_duration').value)
        self.backup_duration = float(self.get_parameter('backup_duration').value)
        self.turn_commit_duration = float(self.get_parameter('turn_commit_duration').value)
        self.explore_bias_speed = float(self.get_parameter('explore_bias_speed').value)
        self.explore_bias_period = float(self.get_parameter('explore_bias_period').value)
        self.side_clearance_weight = float(self.get_parameter('side_clearance_weight').value)
        self.enable_center_spiral = bool(self.get_parameter('enable_center_spiral').value)
        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.center_spiral_weight = float(self.get_parameter('center_spiral_weight').value)
        spiral_turn = float(self.get_parameter('center_spiral_turn_direction').value)
        self.center_spiral_turn_direction = -1.0 if spiral_turn < 0.0 else 1.0
        self.center_spiral_max_angle = math.radians(
            float(self.get_parameter('center_spiral_max_angle_deg').value)
        )
        self.center_spiral_min_radius = float(
            self.get_parameter('center_spiral_min_radius').value
        )
        self.enable_outer_explore_after_event = bool(
            self.get_parameter('enable_outer_explore_after_event').value
        )
        self.outer_explore_weight = float(self.get_parameter('outer_explore_weight').value)
        self.outer_explore_tangent_weight = float(
            self.get_parameter('outer_explore_tangent_weight').value
        )
        self.outer_explore_max_angle = math.radians(
            float(self.get_parameter('outer_explore_max_angle_deg').value)
        )
        self.outer_explore_min_radius = float(
            self.get_parameter('outer_explore_min_radius').value
        )
        self.map_min_x = float(self.get_parameter('map_min_x').value)
        self.map_max_x = float(self.get_parameter('map_max_x').value)
        self.map_min_y = float(self.get_parameter('map_min_y').value)
        self.map_max_y = float(self.get_parameter('map_max_y').value)
        self.boundary_margin_m = float(self.get_parameter('boundary_margin_m').value)
        self.red_zone_min_x = float(self.get_parameter('red_zone_min_x').value)
        self.red_zone_max_x = float(self.get_parameter('red_zone_max_x').value)
        self.red_zone_min_y = float(self.get_parameter('red_zone_min_y').value)
        self.red_zone_max_y = float(self.get_parameter('red_zone_max_y').value)
        self.red_zone_avoid_margin_m = float(
            self.get_parameter('red_zone_avoid_margin_m').value
        )
        self.red_zone_predict_horizon_m = float(
            self.get_parameter('red_zone_predict_horizon_m').value
        )
        self.red_zone_turn_away_weight = float(
            self.get_parameter('red_zone_turn_away_weight').value
        )

        self.front_distance: Optional[float] = None
        self.left_distance = math.inf
        self.right_distance = math.inf
        self.target_angle: Optional[float] = None
        self.smoothed_target_angle = 0.0
        self.mode = DriveMode.CRUISE
        self.mode_until = 0.0
        self.mode_started_at = 0.0
        self.turn_direction = self.preferred_turn_direction
        self.last_turn_sign = self.preferred_turn_direction
        self.sign_flip_count = 0
        self.loop_window_started_at = 0.0
        self.start_time = self._now_seconds()
        self.odom_pose: Optional[Tuple[float, float, float]] = None
        self.hostage_event_received = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self._scan_callback, 10)
        self.create_subscription(Odometry, 'odom', self._odom_callback, 10)
        self.create_subscription(
            HostageEvent,
            str(self.get_parameter('hostage_event_topic').value),
            self._hostage_event_callback,
            10,
        )
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            'Reactive scout started: '
            f'forward={self.forward_speed:.2f}, turn={self.max_turn_speed:.2f}, '
            f'stop={self.stop_distance:.2f}, free={self.free_distance:.2f}'
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        if not msg.ranges:
            self.front_distance = None
            self.target_angle = None
            return

        samples: List[Tuple[float, float]] = []
        front_ranges = []
        left_ranges = []
        right_ranges = []
        half_fov = self.field_of_view / 2.0

        for index, value in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            normalized_angle = self._normalize_angle(angle)
            if abs(normalized_angle) > half_fov:
                continue

            if not math.isfinite(value) or value < msg.range_min:
                distance = msg.range_max
            else:
                distance = min(value, msg.range_max)

            samples.append((normalized_angle, distance))
            if abs(normalized_angle) <= self.front_angle:
                front_ranges.append(distance)
            if math.radians(35.0) <= normalized_angle <= math.radians(95.0):
                left_ranges.append(distance)
            elif math.radians(-95.0) <= normalized_angle <= math.radians(-35.0):
                right_ranges.append(distance)

        self.front_distance = min(front_ranges) if front_ranges else math.inf
        self.left_distance = self._percentile(left_ranges, 35) if left_ranges else math.inf
        self.right_distance = self._percentile(right_ranges, 35) if right_ranges else math.inf
        self.target_angle = self._best_gap_angle(samples)

    def _control_loop(self) -> None:
        cmd = Twist()

        if self.front_distance is None or self.target_angle is None:
            self.cmd_pub.publish(cmd)
            return

        now = self._now_seconds()
        self._update_mode(now)

        alpha = self._clamp(self.target_smoothing, 0.0, 1.0)
        self.smoothed_target_angle = (
            alpha * self.target_angle + (1.0 - alpha) * self.smoothed_target_angle
        )

        if self.mode == DriveMode.ESCAPE:
            elapsed = now - self.mode_started_at
            cmd.linear.x = self.backup_speed if elapsed < self.backup_duration else 0.0
            cmd.angular.z = self.turn_direction * self.max_turn_speed
            self.cmd_pub.publish(cmd)
            return

        target_angle = self.smoothed_target_angle
        if self.mode == DriveMode.COMMITTED_TURN:
            target_angle = self.turn_direction * max(abs(target_angle), math.radians(35.0))
        elif self.front_distance > self.slow_distance and abs(target_angle) < math.radians(10.0):
            target_angle += self._explore_bias(now)

        target_angle = self._apply_scenario_bias(target_angle)
        cmd.angular.z = self._clamp(1.25 * target_angle, -self.max_turn_speed, self.max_turn_speed)

        if self.front_distance < self.stop_distance:
            cmd.linear.x = 0.0
            if abs(cmd.angular.z) < 0.25:
                cmd.angular.z = self.preferred_turn_direction * self.max_turn_speed
        elif self.front_distance < self.slow_distance:
            clearance_ratio = (
                (self.front_distance - self.stop_distance)
                / max(self.slow_distance - self.stop_distance, 0.01)
            )
            angle_ratio = 1.0 - min(abs(self.smoothed_target_angle) / (math.pi / 2.0), 1.0)
            cmd.linear.x = self.slow_speed + (self.forward_speed - self.slow_speed) * (
                0.35 + 0.65 * min(clearance_ratio, angle_ratio)
            )
        else:
            turn_ratio = min(abs(self.smoothed_target_angle) / (math.pi / 2.0), 1.0)
            cmd.linear.x = self.forward_speed * (1.0 - 0.45 * turn_ratio)

        self.cmd_pub.publish(cmd)

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = self._yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self.odom_pose = (pose.position.x, pose.position.y, yaw)

    def _hostage_event_callback(self, msg: HostageEvent) -> None:
        if self.hostage_event_received:
            return
        self.hostage_event_received = True
        self.get_logger().info(
            'Hostage event received; switching scout bias to safe outer exploration '
            f'marker_id={msg.marker_id}'
        )

    def _update_mode(self, now: float) -> None:
        if now < self.mode_until:
            return

        if self.mode != DriveMode.CRUISE:
            self.mode = DriveMode.CRUISE

        if self.front_distance is not None and self.front_distance < self.stop_distance:
            self._enter_escape(now)
            return

        turn_sign = self._sign(self.target_angle or 0.0)
        if abs(self.target_angle or 0.0) > math.radians(25.0) and turn_sign != 0.0:
            if turn_sign != self.last_turn_sign:
                if now - self.loop_window_started_at > 6.0:
                    self.loop_window_started_at = now
                    self.sign_flip_count = 0
                self.sign_flip_count += 1
            self.last_turn_sign = turn_sign

            if self.sign_flip_count >= 4:
                self.sign_flip_count = 0
                self._enter_escape(now)
                return

            if self.front_distance is not None and self.front_distance < self.slow_distance:
                self.mode = DriveMode.COMMITTED_TURN
                self.mode_started_at = now
                self.mode_until = now + self.turn_commit_duration
                self.turn_direction = turn_sign

    def _enter_escape(self, now: float) -> None:
        self.mode = DriveMode.ESCAPE
        self.mode_started_at = now
        self.mode_until = now + self.escape_duration
        if abs(self.left_distance - self.right_distance) > 0.12:
            self.turn_direction = 1.0 if self.left_distance > self.right_distance else -1.0
        else:
            self.turn_direction = self.preferred_turn_direction

    def _explore_bias(self, now: float) -> float:
        period = max(self.explore_bias_period, 1.0)
        phase = int((now - self.start_time) / period)
        direction = self.preferred_turn_direction if phase % 2 == 0 else -self.preferred_turn_direction
        return direction * self.explore_bias_speed

    def _apply_center_spiral_bias(self, target_angle: float) -> float:
        if not self.enable_center_spiral or self.odom_pose is None:
            return target_angle
        if self.front_distance is None or self.front_distance < self.stop_distance:
            return target_angle

        x, y, yaw = self.odom_pose
        inward_x = self.center_x - x
        inward_y = self.center_y - y
        radius = math.hypot(inward_x, inward_y)
        if radius < self.center_spiral_min_radius:
            return target_angle

        inward_x /= radius
        inward_y /= radius
        tangent_x = self.center_spiral_turn_direction * -inward_y
        tangent_y = self.center_spiral_turn_direction * inward_x
        desired_x = inward_x + 0.55 * tangent_x
        desired_y = inward_y + 0.55 * tangent_y

        desired_heading = math.atan2(desired_y, desired_x)
        desired_angle = self._normalize_angle(desired_heading - yaw)
        desired_angle = self._clamp(
            desired_angle,
            -self.center_spiral_max_angle,
            self.center_spiral_max_angle,
        )

        weight = self._clamp(self.center_spiral_weight, 0.0, 1.0)
        return self._normalize_angle((1.0 - weight) * target_angle + weight * desired_angle)

    def _apply_scenario_bias(self, target_angle: float) -> float:
        if (
            self.hostage_event_received
            and self.enable_outer_explore_after_event
            and self.odom_pose is not None
        ):
            return self._apply_outer_explore_bias(target_angle)
        return self._apply_center_spiral_bias(target_angle)

    def _apply_outer_explore_bias(self, target_angle: float) -> float:
        if self.front_distance is None or self.front_distance < self.stop_distance:
            return target_angle

        x, y, yaw = self.odom_pose
        outward_x = x - self.center_x
        outward_y = y - self.center_y
        radius = math.hypot(outward_x, outward_y)
        if radius < self.outer_explore_min_radius:
            outward_x = math.cos(yaw)
            outward_y = math.sin(yaw)
            radius = 1.0

        outward_x /= radius
        outward_y /= radius

        tangent_x = self.preferred_turn_direction * -outward_y
        tangent_y = self.preferred_turn_direction * outward_x
        desired_x = outward_x + self.outer_explore_tangent_weight * tangent_x
        desired_y = outward_y + self.outer_explore_tangent_weight * tangent_y

        avoid_x, avoid_y = self._boundary_avoidance_vector(x, y)
        desired_x += avoid_x
        desired_y += avoid_y

        red_avoid_x, red_avoid_y = self._red_zone_avoidance_vector(x, y)
        desired_x += red_avoid_x
        desired_y += red_avoid_y
        desired_x, desired_y = self._keep_heading_out_of_red_zone(
            x,
            y,
            desired_x,
            desired_y,
        )

        desired_norm = math.hypot(desired_x, desired_y)
        if desired_norm < 1e-6:
            return target_angle

        desired_heading = math.atan2(desired_y, desired_x)
        desired_angle = self._normalize_angle(desired_heading - yaw)
        desired_angle = self._clamp(
            desired_angle,
            -self.outer_explore_max_angle,
            self.outer_explore_max_angle,
        )
        weight = self._clamp(self.outer_explore_weight, 0.0, 1.0)
        return self._normalize_angle((1.0 - weight) * target_angle + weight * desired_angle)

    def _boundary_avoidance_vector(self, x: float, y: float) -> Tuple[float, float]:
        margin = max(self.boundary_margin_m, 0.01)
        avoid_x = 0.0
        avoid_y = 0.0
        if x - self.map_min_x < margin:
            avoid_x += (margin - (x - self.map_min_x)) / margin
        if self.map_max_x - x < margin:
            avoid_x -= (margin - (self.map_max_x - x)) / margin
        if y - self.map_min_y < margin:
            avoid_y += (margin - (y - self.map_min_y)) / margin
        if self.map_max_y - y < margin:
            avoid_y -= (margin - (self.map_max_y - y)) / margin
        return avoid_x, avoid_y

    def _red_zone_avoidance_vector(self, x: float, y: float) -> Tuple[float, float]:
        min_x = self.red_zone_min_x - self.red_zone_avoid_margin_m
        max_x = self.red_zone_max_x + self.red_zone_avoid_margin_m
        min_y = self.red_zone_min_y - self.red_zone_avoid_margin_m
        max_y = self.red_zone_max_y + self.red_zone_avoid_margin_m
        if not (min_x <= x <= max_x and min_y <= y <= max_y):
            return 0.0, 0.0

        center_x = (self.red_zone_min_x + self.red_zone_max_x) / 2.0
        center_y = (self.red_zone_min_y + self.red_zone_max_y) / 2.0
        away_x = x - center_x
        away_y = y - center_y
        distance = max(math.hypot(away_x, away_y), 0.01)
        strength = self._clamp(
            1.0 - distance / max(self.red_zone_avoid_margin_m + 3.0, 0.01),
            0.2,
            1.0,
        )
        return strength * away_x / distance, strength * away_y / distance

    def _keep_heading_out_of_red_zone(
        self,
        x: float,
        y: float,
        desired_x: float,
        desired_y: float,
    ) -> Tuple[float, float]:
        norm = math.hypot(desired_x, desired_y)
        if norm < 1e-6:
            return desired_x, desired_y

        direction_x = desired_x / norm
        direction_y = desired_y / norm
        if not self._path_points_toward_red_zone(x, y, direction_x, direction_y):
            return desired_x, desired_y

        center_x = (self.red_zone_min_x + self.red_zone_max_x) / 2.0
        center_y = (self.red_zone_min_y + self.red_zone_max_y) / 2.0
        away_x = x - center_x
        away_y = y - center_y
        away_norm = math.hypot(away_x, away_y)
        if away_norm < 1e-6:
            away_x = -1.0
            away_y = -1.0
            away_norm = math.sqrt(2.0)

        away_x /= away_norm
        away_y /= away_norm
        tangent_x = self.preferred_turn_direction * -away_y
        tangent_y = self.preferred_turn_direction * away_x
        safe_x = away_x + 0.45 * tangent_x
        safe_y = away_y + 0.45 * tangent_y

        weight = self._clamp(self.red_zone_turn_away_weight, 0.0, 1.0)
        mixed_x = (1.0 - weight) * direction_x + weight * safe_x
        mixed_y = (1.0 - weight) * direction_y + weight * safe_y
        mixed_norm = math.hypot(mixed_x, mixed_y)
        if mixed_norm < 1e-6:
            return desired_x, desired_y
        return norm * mixed_x / mixed_norm, norm * mixed_y / mixed_norm

    def _path_points_toward_red_zone(
        self,
        x: float,
        y: float,
        direction_x: float,
        direction_y: float,
    ) -> bool:
        min_x = self.red_zone_min_x - self.red_zone_avoid_margin_m
        max_x = self.red_zone_max_x + self.red_zone_avoid_margin_m
        min_y = self.red_zone_min_y - self.red_zone_avoid_margin_m
        max_y = self.red_zone_max_y + self.red_zone_avoid_margin_m
        horizon = max(self.red_zone_predict_horizon_m, 0.0)

        if min_x <= x <= max_x and min_y <= y <= max_y:
            return True

        steps = max(1, int(math.ceil(horizon / 0.20)))
        for index in range(1, steps + 1):
            distance = horizon * index / steps
            next_x = x + direction_x * distance
            next_y = y + direction_y * distance
            if min_x <= next_x <= max_x and min_y <= next_y <= max_y:
                return True
        return False

    def _best_gap_angle(self, samples: List[Tuple[float, float]]) -> Optional[float]:
        if not samples:
            return None

        free_mask = [distance >= self.free_distance for _, distance in samples]
        best_start = None
        best_end = None
        index = 0

        while index < len(free_mask):
            if not free_mask[index]:
                index += 1
                continue

            start = index
            while index < len(free_mask) and free_mask[index]:
                index += 1
            end = index - 1

            if best_start is None or (end - start) > (best_end - best_start):
                best_start = start
                best_end = end

        if best_start is None or best_end is None:
            return self.preferred_turn_direction * self.field_of_view / 2.0

        best_index = best_start
        best_score = -math.inf
        center = (best_start + best_end) / 2.0
        width = max(best_end - best_start, 1)

        for index in range(best_start, best_end + 1):
            angle, distance = samples[index]
            center_bias = 1.0 - abs(index - center) / width
            forward_bias = 1.0 - min(abs(angle) / (self.field_of_view / 2.0), 1.0)
            side_clearance = self.left_distance if angle > 0.0 else self.right_distance
            side_score = min(side_clearance, self.slow_distance) / self.slow_distance
            score = distance + 0.8 * center_bias + 0.5 * forward_bias + self.side_clearance_weight * side_score
            if score > best_score:
                best_score = score
                best_index = index

        return samples[best_index][0]

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)

    @staticmethod
    def _percentile(values: List[float], percentile: int) -> float:
        if not values:
            return math.inf
        ordered = sorted(values)
        index = int((len(ordered) - 1) * percentile / 100.0)
        return ordered[index]

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _sign(value: float) -> float:
        if value > 0.0:
            return 1.0
        if value < 0.0:
            return -1.0
        return 0.0

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReactiveScout()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

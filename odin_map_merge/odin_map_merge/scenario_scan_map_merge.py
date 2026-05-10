import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    has_odom: bool = False


class ScenarioScanMapMerge(Node):
    """Build a shared occupancy grid directly from known-scenario odom and scans."""

    def __init__(self) -> None:
        super().__init__('scenario_scan_map_merge')

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('output_topic', '/merged_map')
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('origin_x', -10.0)
        self.declare_parameter('origin_y', -10.0)
        self.declare_parameter('width_m', 20.0)
        self.declare_parameter('height_m', 20.0)
        self.declare_parameter('free_value', 0)
        self.declare_parameter('occupied_value', 100)
        self.declare_parameter('unknown_value', -1)
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('scan_decimation', 2)
        self.declare_parameter('robot_clear_radius', 0.42)
        self.declare_parameter('dynamic_obstacle_filter_radius', 0.55)
        self.declare_parameter('robot_names', ['robot_1', 'robot_2'])

        self.global_frame = str(self.get_parameter('global_frame').value)
        output_topic = str(self.get_parameter('output_topic').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        width_m = float(self.get_parameter('width_m').value)
        height_m = float(self.get_parameter('height_m').value)
        self.free_value = int(self.get_parameter('free_value').value)
        self.occupied_value = int(self.get_parameter('occupied_value').value)
        self.unknown_value = int(self.get_parameter('unknown_value').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.scan_decimation = max(1, int(self.get_parameter('scan_decimation').value))
        self.robot_clear_radius = float(self.get_parameter('robot_clear_radius').value)
        self.dynamic_obstacle_filter_radius = float(
            self.get_parameter('dynamic_obstacle_filter_radius').value
        )
        robot_names = list(self.get_parameter('robot_names').value)

        self.width = max(1, int(round(width_m / self.resolution)))
        self.height = max(1, int(round(height_m / self.resolution)))
        self.grid = [self.unknown_value] * (self.width * self.height)
        self.robot_states: Dict[str, RobotState] = {
            robot_name: RobotState() for robot_name in robot_names
        }

        self.publisher = self.create_publisher(OccupancyGrid, output_topic, 1)

        for robot_name in robot_names:
            self.create_subscription(
                Odometry,
                f'/{robot_name}/odom',
                lambda msg, name=robot_name: self._odom_callback(name, msg),
                10,
            )
            self.create_subscription(
                LaserScan,
                f'/{robot_name}/scan',
                lambda msg, name=robot_name: self._scan_callback(name, msg),
                10,
            )
            self.get_logger().info(
                f'Using /{robot_name}/odom + /{robot_name}/scan for scenario merged map'
            )

        timer_period = 1.0 / max(publish_rate_hz, 0.1)
        self.create_timer(timer_period, self._publish_map)

    def _odom_callback(self, robot_name: str, msg: Odometry) -> None:
        pose = msg.pose.pose
        state = self.robot_states[robot_name]
        state.x = pose.position.x
        state.y = pose.position.y
        state.yaw = self._yaw_from_pose(pose)
        state.has_odom = True

    def _scan_callback(self, robot_name: str, msg: LaserScan) -> None:
        state = self.robot_states[robot_name]
        if not state.has_odom:
            return

        robot_cell = self._world_to_cell(state.x, state.y)
        if robot_cell is None:
            return

        range_max = min(self.max_range, msg.range_max)
        for index in range(0, len(msg.ranges), self.scan_decimation):
            raw_range = msg.ranges[index]
            angle = state.yaw + msg.angle_min + index * msg.angle_increment
            has_hit = math.isfinite(raw_range) and msg.range_min <= raw_range <= range_max
            scan_range = raw_range if has_hit else range_max
            scan_range = max(msg.range_min, min(scan_range, range_max))

            end_x = state.x + scan_range * math.cos(angle)
            end_y = state.y + scan_range * math.sin(angle)
            end_cell = self._world_to_cell(end_x, end_y)
            if end_cell is None:
                end_cell = self._clipped_cell_from_ray(state.x, state.y, end_x, end_y)
                has_hit = False

            if end_cell is None:
                continue

            self._raytrace_free(robot_cell, end_cell)
            if has_hit and not self._is_other_robot_hit(robot_name, end_x, end_y):
                self._mark_cell(end_cell[0], end_cell[1], self.occupied_value)

    def _publish_map(self) -> None:
        self._clear_robot_footprints()

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin = self._origin_pose()
        msg.data = self.grid
        self.publisher.publish(msg)

    def _is_other_robot_hit(self, source_robot_name: str, hit_x: float, hit_y: float) -> bool:
        if self.dynamic_obstacle_filter_radius <= 0.0:
            return False

        for robot_name, state in self.robot_states.items():
            if robot_name == source_robot_name or not state.has_odom:
                continue
            distance = math.hypot(hit_x - state.x, hit_y - state.y)
            if distance <= self.dynamic_obstacle_filter_radius:
                return True

        return False

    def _clear_robot_footprints(self) -> None:
        if self.robot_clear_radius <= 0.0:
            return

        radius_cells = max(1, int(math.ceil(self.robot_clear_radius / self.resolution)))
        for state in self.robot_states.values():
            if not state.has_odom:
                continue

            center = self._world_to_cell(state.x, state.y)
            if center is None:
                continue

            center_x, center_y = center
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy > radius_cells * radius_cells:
                        continue
                    self._mark_cell(center_x + dx, center_y + dy, self.free_value)

    def _raytrace_free(self, start: Tuple[int, int], end: Tuple[int, int]) -> None:
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x = x0
        y = y0

        while True:
            if (x, y) == (x1, y1):
                break
            self._mark_free(x, y)
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def _mark_free(self, x: int, y: int) -> None:
        index = y * self.width + x
        if self.grid[index] != self.occupied_value:
            self.grid[index] = self.free_value

    def _mark_cell(self, x: int, y: int, value: int) -> None:
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y * self.width + x] = value

    def _world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        cell_x = int(math.floor((x - self.origin_x) / self.resolution))
        cell_y = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= cell_x < self.width and 0 <= cell_y < self.height:
            return cell_x, cell_y
        return None

    def _clipped_cell_from_ray(
        self,
        start_x: float,
        start_y: float,
        end_x: float,
        end_y: float,
    ) -> Optional[Tuple[int, int]]:
        steps = max(1, int(math.ceil(self.max_range / self.resolution)))
        last_cell = self._world_to_cell(start_x, start_y)
        for step in range(1, steps + 1):
            ratio = step / steps
            x = start_x + (end_x - start_x) * ratio
            y = start_y + (end_y - start_y) * ratio
            cell = self._world_to_cell(x, y)
            if cell is None:
                return last_cell
            last_cell = cell
        return last_cell

    def _origin_pose(self) -> Pose:
        pose = Pose()
        pose.position.x = self.origin_x
        pose.position.y = self.origin_y
        pose.orientation.w = 1.0
        return pose

    @staticmethod
    def _yaw_from_pose(pose: Pose) -> float:
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioScanMapMerge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class RectangleZone:
    name: str
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    hard_reject: bool = True


@dataclass
class CircleZone:
    name: str
    center_x: float
    center_y: float
    radius_m: float
    hard_reject: bool = True


class HeuristicWaypointRecommender(Node):
    """Local stand-in for Qwen/VLM waypoint recommendations."""

    def __init__(self) -> None:
        super().__init__('heuristic_waypoint_recommender')

        self.declare_parameter('candidate_path_topic', '/coordinator/candidate_path')
        self.declare_parameter('merged_map_topic', '/merged_map')
        self.declare_parameter('recommendation_topic', '/ai/waypoint_recommendation')
        self.declare_parameter('status_topic', '/ai/status')
        self.declare_parameter('battlefield_config_file', '')
        self.declare_parameter('prefer_goal_index_from_end', 0)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('allow_unknown_cells', True)

        self.latest_map: Optional[OccupancyGrid] = None
        self.prefer_goal_index_from_end = max(
            0,
            int(self.get_parameter('prefer_goal_index_from_end').value),
        )
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.allow_unknown_cells = bool(self.get_parameter('allow_unknown_cells').value)
        self.red_zones = []
        self.enemy_vision_zones = []
        self._load_battlefield_config(str(self.get_parameter('battlefield_config_file').value))

        self.recommendation_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('recommendation_topic').value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('merged_map_topic').value),
            self._map_callback,
            1,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter('candidate_path_topic').value),
            self._path_callback,
            10,
        )

        self._publish_status('heuristic_ai_ready mode=local_fallback')

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def _path_callback(self, path: Path) -> None:
        if not path.poses:
            self._publish_status('rejected_candidate_path reason=empty')
            return

        start_index = max(0, len(path.poses) - 1 - self.prefer_goal_index_from_end)
        for index in range(start_index, -1, -1):
            waypoint = path.poses[index]
            if self._pose_is_acceptable(waypoint):
                self.recommendation_pub.publish(waypoint)
                self._publish_status(
                    'recommended_waypoint '
                    f'index={index} x={waypoint.pose.position.x:.2f} '
                    f'y={waypoint.pose.position.y:.2f}'
                )
                return

        self._publish_status('rejected_candidate_path reason=no_accessible_waypoint')

    def _pose_is_acceptable(self, waypoint: PoseStamped) -> bool:
        x = waypoint.pose.position.x
        y = waypoint.pose.position.y
        forbidden_zone = self._forbidden_zone_name(x, y)
        if forbidden_zone is not None:
            return False

        if self.latest_map is None:
            return True

        cell = self._world_to_cell(x, y)
        if cell is None:
            return False

        value = int(self.latest_map.data[cell[1] * self.latest_map.info.width + cell[0]])
        if value < 0:
            return self.allow_unknown_cells
        return value < self.occupied_threshold

    def _world_to_cell(self, x: float, y: float):
        assert self.latest_map is not None
        origin = self.latest_map.info.origin.position
        resolution = self.latest_map.info.resolution
        cell_x = int((x - origin.x) / resolution)
        cell_y = int((y - origin.y) / resolution)
        if 0 <= cell_x < self.latest_map.info.width and 0 <= cell_y < self.latest_map.info.height:
            return cell_x, cell_y
        return None

    def _forbidden_zone_name(self, x: float, y: float) -> Optional[str]:
        for zone in self.red_zones:
            if not zone.hard_reject:
                continue
            if zone.min_x <= x <= zone.max_x and zone.min_y <= y <= zone.max_y:
                return zone.name

        for zone in self.enemy_vision_zones:
            if not zone.hard_reject:
                continue
            if math.hypot(x - zone.center_x, y - zone.center_y) <= zone.radius_m:
                return zone.name

        return None

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _load_battlefield_config(self, config_file: str) -> None:
        if not config_file:
            self.get_logger().info('No battlefield_config_file provided; using map checks only')
            return

        try:
            with open(config_file, 'r', encoding='utf-8') as config:
                battlefield = yaml.safe_load(config).get('battlefield', {})
        except (OSError, yaml.YAMLError, AttributeError) as exc:
            self.get_logger().warning(f'Failed to load battlefield config {config_file}: {exc}')
            return

        self.red_zones = [
            RectangleZone(
                name=str(zone.get('name', 'red_zone')),
                min_x=float(zone['min_x']),
                max_x=float(zone['max_x']),
                min_y=float(zone['min_y']),
                max_y=float(zone['max_y']),
                hard_reject=bool(zone.get('hard_reject', True)),
            )
            for zone in battlefield.get('red_zones', [])
            if zone.get('type', 'rectangle') == 'rectangle'
        ]
        self.enemy_vision_zones = [
            CircleZone(
                name=str(zone.get('name', 'enemy_vision_zone')),
                center_x=float(zone['center_x']),
                center_y=float(zone['center_y']),
                radius_m=float(zone['radius_m']),
                hard_reject=bool(zone.get('hard_reject', True)),
            )
            for zone in battlefield.get('enemy_vision_zones', [])
            if zone.get('type', 'circle') == 'circle'
        ]

        self.get_logger().info(
            'Loaded battlefield config: '
            f'{len(self.red_zones)} red zone(s), '
            f'{len(self.enemy_vision_zones)} enemy vision zone(s)'
        )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = HeuristicWaypointRecommender()
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

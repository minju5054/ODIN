from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class HeuristicWaypointRecommender(Node):
    """Local stand-in for Qwen/VLM waypoint recommendations."""

    def __init__(self) -> None:
        super().__init__('heuristic_waypoint_recommender')

        self.declare_parameter('candidate_path_topic', '/coordinator/candidate_path')
        self.declare_parameter('merged_map_topic', '/merged_map')
        self.declare_parameter('recommendation_topic', '/ai/waypoint_recommendation')
        self.declare_parameter('status_topic', '/ai/status')
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
        if self.latest_map is None:
            return True

        cell = self._world_to_cell(
            waypoint.pose.position.x,
            waypoint.pose.position.y,
        )
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

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)


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

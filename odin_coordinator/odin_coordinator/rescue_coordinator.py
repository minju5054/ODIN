import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from odin_interfaces.msg import HostageEvent
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class SeenEvent:
    marker_id: int
    x: float
    y: float
    stamp_sec: float


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    stamp_sec: float = 0.0
    has_odom: bool = False


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


class RescueCoordinator(Node):
    """Validate hostage events and prepare safe robot_3 rescue candidates."""

    def __init__(self) -> None:
        super().__init__('rescue_coordinator')

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('hostage_event_topic', '/hostage_events')
        self.declare_parameter('merged_map_topic', '/merged_map')
        self.declare_parameter('status_topic', '/coordinator/status')
        self.declare_parameter('candidate_path_topic', '/coordinator/candidate_path')
        self.declare_parameter('rescue_goal_topic', '/robot_3/goal_pose')
        self.declare_parameter('ai_waypoint_topic', '/ai/waypoint_recommendation')
        self.declare_parameter('validated_waypoint_topic', '/coordinator/validated_waypoint')
        self.declare_parameter('battlefield_config_file', '')
        self.declare_parameter('robot_names', ['robot_1', 'robot_2', 'robot_3'])
        self.declare_parameter('safe_insertion_x', -7.5)
        self.declare_parameter('safe_insertion_y', -7.5)
        self.declare_parameter('safe_insertion_yaw', 0.785398)
        self.declare_parameter('map_min_x', -10.0)
        self.declare_parameter('map_max_x', 10.0)
        self.declare_parameter('map_min_y', -10.0)
        self.declare_parameter('map_max_y', 10.0)
        self.declare_parameter('duplicate_distance_m', 0.75)
        self.declare_parameter('duplicate_window_sec', 60.0)
        self.declare_parameter('rescue_standoff_m', 0.9)
        self.declare_parameter('rescue_standoff_samples', 16)
        self.declare_parameter('rescue_standoff_extra_radii_m', [1.2, 1.5])
        self.declare_parameter('path_waypoint_spacing_m', 0.5)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('accessibility_radius_m', 0.25)
        self.declare_parameter('allow_unknown_cells', True)
        self.declare_parameter('require_robot_3_available_for_goal', True)
        self.declare_parameter('use_nav2_path_planning', True)
        self.declare_parameter('compute_path_action', '/robot_3/compute_path_to_pose')
        self.declare_parameter('fallback_to_straight_path', True)

        self.global_frame = str(self.get_parameter('global_frame').value)
        self.safe_insertion_x = float(self.get_parameter('safe_insertion_x').value)
        self.safe_insertion_y = float(self.get_parameter('safe_insertion_y').value)
        self.safe_insertion_yaw = float(self.get_parameter('safe_insertion_yaw').value)
        self.map_min_x = float(self.get_parameter('map_min_x').value)
        self.map_max_x = float(self.get_parameter('map_max_x').value)
        self.map_min_y = float(self.get_parameter('map_min_y').value)
        self.map_max_y = float(self.get_parameter('map_max_y').value)
        self.duplicate_distance_m = float(self.get_parameter('duplicate_distance_m').value)
        self.duplicate_window_sec = float(self.get_parameter('duplicate_window_sec').value)
        self.rescue_standoff_m = float(self.get_parameter('rescue_standoff_m').value)
        self.rescue_standoff_samples = max(
            4,
            int(self.get_parameter('rescue_standoff_samples').value),
        )
        self.rescue_standoff_extra_radii_m = [
            float(radius)
            for radius in self.get_parameter('rescue_standoff_extra_radii_m').value
        ]
        self.path_waypoint_spacing_m = float(self.get_parameter('path_waypoint_spacing_m').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.accessibility_radius_m = float(self.get_parameter('accessibility_radius_m').value)
        self.allow_unknown_cells = bool(self.get_parameter('allow_unknown_cells').value)
        self.require_robot_3_available_for_goal = bool(
            self.get_parameter('require_robot_3_available_for_goal').value
        )
        self.use_nav2_path_planning = bool(self.get_parameter('use_nav2_path_planning').value)
        self.fallback_to_straight_path = bool(
            self.get_parameter('fallback_to_straight_path').value
        )

        self.latest_map: Optional[OccupancyGrid] = None
        self.seen_events: List[SeenEvent] = []
        self.robot_states: Dict[str, RobotState] = {}
        self.red_zones: List[RectangleZone] = []
        self.enemy_vision_zones: List[CircleZone] = []
        self._load_battlefield_config(str(self.get_parameter('battlefield_config_file').value))

        status_topic = str(self.get_parameter('status_topic').value)
        candidate_path_topic = str(self.get_parameter('candidate_path_topic').value)
        rescue_goal_topic = str(self.get_parameter('rescue_goal_topic').value)
        validated_waypoint_topic = str(self.get_parameter('validated_waypoint_topic').value)

        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.candidate_path_pub = self.create_publisher(Path, candidate_path_topic, 10)
        self.rescue_goal_pub = self.create_publisher(PoseStamped, rescue_goal_topic, 10)
        self.validated_waypoint_pub = self.create_publisher(
            PoseStamped,
            validated_waypoint_topic,
            10,
        )
        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            str(self.get_parameter('compute_path_action').value),
        )

        self.create_subscription(
            HostageEvent,
            str(self.get_parameter('hostage_event_topic').value),
            self._hostage_event_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('merged_map_topic').value),
            self._map_callback,
            1,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('ai_waypoint_topic').value),
            self._ai_waypoint_callback,
            10,
        )

        robot_names = [str(name) for name in self.get_parameter('robot_names').value]
        for robot_name in robot_names:
            self.robot_states[robot_name] = RobotState()
            self.create_subscription(
                Odometry,
                f'/{robot_name}/odom',
                lambda msg, name=robot_name: self._robot_odom_callback(name, msg),
                10,
            )

        self._publish_status('coordinator_ready')
        self.get_logger().info(
            'Rescue coordinator started: '
            f'events={self.get_parameter("hostage_event_topic").value}, '
            f'map={self.get_parameter("merged_map_topic").value}, '
            f'robot_3_goal={rescue_goal_topic}'
        )

    def _hostage_event_callback(self, event: HostageEvent) -> None:
        valid, reason = self._validate_hostage_event(event)
        if not valid:
            self._publish_status(f'rejected_hostage_event reason={reason}')
            return

        rescue_goal = self._find_rescue_goal(event.pose)
        if rescue_goal is None:
            self._publish_status('rejected_rescue_goal reason=no_accessible_standoff')
            return

        path = self._make_candidate_path(rescue_goal)
        if self.use_nav2_path_planning and self.compute_path_client.server_is_ready():
            self._request_nav2_path(event, rescue_goal, path)
            return

        if self.use_nav2_path_planning and not self.compute_path_client.server_is_ready():
            self._publish_status(
                'nav2_planner_unavailable '
                f'fallback_to_straight_path={self.fallback_to_straight_path}'
            )
            if not self.fallback_to_straight_path:
                return

        self._approve_candidate(event, rescue_goal, path, planner='straight_fallback')

    def _ai_waypoint_callback(self, waypoint: PoseStamped) -> None:
        valid, reason = self._validate_pose_stamped(waypoint, require_map=True)
        if not valid:
            self._publish_status(f'rejected_ai_waypoint reason={reason}')
            return

        self.validated_waypoint_pub.publish(waypoint)
        self.rescue_goal_pub.publish(waypoint)
        self._publish_status(
            'ai_waypoint_validated dispatch_requested '
            f'x={waypoint.pose.position.x:.2f} y={waypoint.pose.position.y:.2f}'
        )

    def _request_nav2_path(
        self,
        event: HostageEvent,
        rescue_goal: PoseStamped,
        fallback_path: Path,
    ) -> None:
        request = ComputePathToPose.Goal()
        request.goal = rescue_goal
        request.start = self._make_safe_insertion_pose()
        request.use_start = True

        send_future = self.compute_path_client.send_goal_async(request)
        send_future.add_done_callback(
            lambda future: self._nav2_path_goal_response(
                future,
                event,
                rescue_goal,
                fallback_path,
            )
        )
        self._publish_status(
            'nav2_path_requested '
            f'x={rescue_goal.pose.position.x:.2f} y={rescue_goal.pose.position.y:.2f}'
        )

    def _nav2_path_goal_response(
        self,
        future,
        event: HostageEvent,
        rescue_goal: PoseStamped,
        fallback_path: Path,
    ) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._publish_status('nav2_path_rejected_by_server')
            if self.fallback_to_straight_path:
                self._approve_candidate(event, rescue_goal, fallback_path, planner='straight_fallback')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self._nav2_path_result(
                done,
                event,
                rescue_goal,
                fallback_path,
            )
        )

    def _nav2_path_result(
        self,
        future,
        event: HostageEvent,
        rescue_goal: PoseStamped,
        fallback_path: Path,
    ) -> None:
        result = future.result().result
        path = result.path
        if path.poses:
            self._approve_candidate(event, rescue_goal, path, planner='nav2')
            return

        self._publish_status('nav2_path_empty')
        if self.fallback_to_straight_path:
            self._approve_candidate(event, rescue_goal, fallback_path, planner='straight_fallback')

    def _approve_candidate(
        self,
        event: HostageEvent,
        rescue_goal: PoseStamped,
        path: Path,
        planner: str,
    ) -> None:
        valid_path, path_reason = self._validate_path(path)
        if not valid_path:
            self._publish_status(f'rejected_candidate_path reason={path_reason} planner={planner}')
            return

        self._remember_event(event)
        self.candidate_path_pub.publish(path)

        robot_3_available = self._robot_3_available()
        if self.require_robot_3_available_for_goal and not robot_3_available:
            self._publish_status(
                'candidate_ready dispatch_blocked reason=robot_3_unavailable '
                f'marker_id={event.marker_id} detecting_robot={event.detecting_robot}'
            )
            return

        self.rescue_goal_pub.publish(rescue_goal)
        self._publish_status(
            'rescue_goal_validated '
            f'planner={planner} marker_id={event.marker_id} '
            f'detecting_robot={event.detecting_robot} '
            f'x={rescue_goal.pose.position.x:.2f} y={rescue_goal.pose.position.y:.2f}'
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def _robot_odom_callback(self, robot_name: str, msg: Odometry) -> None:
        pose = msg.pose.pose
        state = self.robot_states[robot_name]
        state.x = pose.position.x
        state.y = pose.position.y
        state.yaw = self._yaw_from_pose(pose)
        state.stamp_sec = self._stamp_to_seconds(msg.header.stamp)
        state.has_odom = True

    def _validate_hostage_event(self, event: HostageEvent) -> Tuple[bool, str]:
        if event.header.frame_id != self.global_frame:
            return False, f'bad_frame:{event.header.frame_id}'
        if event.marker_id < 0:
            return False, 'invalid_marker_id'
        if not event.detecting_robot:
            return False, 'missing_detecting_robot'
        if not self._pose_values_are_finite(event.pose):
            return False, 'nonfinite_pose'
        if not self._within_bounds(event.pose.position.x, event.pose.position.y):
            return False, 'out_of_bounds'
        if self._is_duplicate_event(event):
            return False, 'duplicate_event'
        return True, 'ok'

    def _validate_pose_stamped(
        self,
        pose_stamped: PoseStamped,
        require_map: bool,
    ) -> Tuple[bool, str]:
        if pose_stamped.header.frame_id != self.global_frame:
            return False, f'bad_frame:{pose_stamped.header.frame_id}'
        if not self._pose_values_are_finite(pose_stamped.pose):
            return False, 'nonfinite_pose'
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        if not self._within_bounds(x, y):
            return False, 'out_of_bounds'
        forbidden_zone = self._forbidden_zone_name(x, y)
        if forbidden_zone is not None:
            return False, f'forbidden_zone:{forbidden_zone}'
        if require_map and not self._pose_is_accessible(x, y):
            return False, 'map_inaccessible'
        return True, 'ok'

    def _validate_path(self, path: Path) -> Tuple[bool, str]:
        if not path.poses:
            return False, 'empty_path'
        for pose in path.poses:
            valid, reason = self._validate_pose_stamped(pose, require_map=False)
            if not valid:
                return False, f'path_{reason}'
        return True, 'ok'

    def _find_rescue_goal(self, marker_pose: Pose) -> Optional[PoseStamped]:
        radii = [self.rescue_standoff_m] + self.rescue_standoff_extra_radii_m
        preferred_angle = math.atan2(
            self.safe_insertion_y - marker_pose.position.y,
            self.safe_insertion_x - marker_pose.position.x,
        )
        angle_offsets = [0.0]
        for index in range(1, self.rescue_standoff_samples):
            step = math.ceil(index / 2)
            sign = 1.0 if index % 2 else -1.0
            angle_offsets.append(sign * step * (2.0 * math.pi / self.rescue_standoff_samples))

        best_goal: Optional[PoseStamped] = None
        best_score = math.inf
        for radius in radii:
            for offset in angle_offsets:
                angle = preferred_angle + offset
                goal = self._make_rescue_goal(marker_pose, radius, angle)
                valid_goal, _ = self._validate_pose_stamped(goal, require_map=True)
                if not valid_goal:
                    continue
                score = math.hypot(
                    goal.pose.position.x - self.safe_insertion_x,
                    goal.pose.position.y - self.safe_insertion_y,
                ) + abs(offset) * 0.1
                if score < best_score:
                    best_score = score
                    best_goal = goal

        return best_goal

    def _make_rescue_goal(self, marker_pose: Pose, radius: float, angle: float) -> PoseStamped:
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.global_frame
        goal.pose.position.x = marker_pose.position.x + math.cos(angle) * radius
        goal.pose.position.y = marker_pose.position.y + math.sin(angle) * radius
        yaw = math.atan2(marker_pose.position.y - goal.pose.position.y,
                         marker_pose.position.x - goal.pose.position.x)
        self._set_yaw(goal.pose, yaw)
        return goal

    def _make_candidate_path(self, goal: PoseStamped) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.global_frame

        start_x = self.safe_insertion_x
        start_y = self.safe_insertion_y
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        steps = max(1, int(math.ceil(distance / max(self.path_waypoint_spacing_m, 0.1))))

        for index in range(steps + 1):
            ratio = index / steps
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = start_x + (goal_x - start_x) * ratio
            pose.pose.position.y = start_y + (goal_y - start_y) * ratio
            yaw = math.atan2(goal_y - start_y, goal_x - start_x)
            self._set_yaw(pose.pose, yaw)
            path.poses.append(pose)

        path.poses[-1] = goal
        return path

    def _make_safe_insertion_pose(self) -> PoseStamped:
        start = PoseStamped()
        start.header.stamp = self.get_clock().now().to_msg()
        start.header.frame_id = self.global_frame
        start.pose.position.x = self.safe_insertion_x
        start.pose.position.y = self.safe_insertion_y
        self._set_yaw(start.pose, self.safe_insertion_yaw)
        return start

    def _pose_is_accessible(self, x: float, y: float) -> bool:
        if self.latest_map is None:
            return False

        center = self._world_to_cell(self.latest_map, x, y)
        if center is None:
            return False

        radius_cells = max(
            0,
            int(math.ceil(self.accessibility_radius_m / self.latest_map.info.resolution)),
        )
        saw_candidate_cell = False
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                cell_x = center[0] + dx
                cell_y = center[1] + dy
                value = self._cell_value(self.latest_map, cell_x, cell_y)
                if value is None:
                    continue
                saw_candidate_cell = True
                if value >= self.occupied_threshold:
                    return False
                if value < 0 and not self.allow_unknown_cells:
                    return False

        return saw_candidate_cell

    def _world_to_cell(
        self,
        grid: OccupancyGrid,
        x: float,
        y: float,
    ) -> Optional[Tuple[int, int]]:
        origin = grid.info.origin.position
        cell_x = int(math.floor((x - origin.x) / grid.info.resolution))
        cell_y = int(math.floor((y - origin.y) / grid.info.resolution))
        if 0 <= cell_x < grid.info.width and 0 <= cell_y < grid.info.height:
            return cell_x, cell_y
        return None

    @staticmethod
    def _cell_value(grid: OccupancyGrid, cell_x: int, cell_y: int) -> Optional[int]:
        if 0 <= cell_x < grid.info.width and 0 <= cell_y < grid.info.height:
            return int(grid.data[cell_y * grid.info.width + cell_x])
        return None

    def _is_duplicate_event(self, event: HostageEvent) -> bool:
        now_sec = self._stamp_to_seconds(event.header.stamp)
        self.seen_events = [
            seen for seen in self.seen_events
            if now_sec - seen.stamp_sec <= self.duplicate_window_sec
        ]

        for seen in self.seen_events:
            if seen.marker_id != event.marker_id:
                continue
            distance = math.hypot(event.pose.position.x - seen.x, event.pose.position.y - seen.y)
            if distance <= self.duplicate_distance_m:
                return True
        return False

    def _remember_event(self, event: HostageEvent) -> None:
        self.seen_events.append(
            SeenEvent(
                marker_id=event.marker_id,
                x=event.pose.position.x,
                y=event.pose.position.y,
                stamp_sec=self._stamp_to_seconds(event.header.stamp),
            )
        )

    def _robot_3_available(self) -> bool:
        state = self.robot_states.get('robot_3')
        if state is None or not state.has_odom:
            return False
        return self._within_bounds(state.x, state.y)

    def _within_bounds(self, x: float, y: float) -> bool:
        return self.map_min_x <= x <= self.map_max_x and self.map_min_y <= y <= self.map_max_y

    def _load_battlefield_config(self, config_file: str) -> None:
        if not config_file:
            self.get_logger().info('No battlefield_config_file provided; using coordinator params only')
            return

        try:
            with open(config_file, 'r', encoding='utf-8') as config:
                battlefield = yaml.safe_load(config).get('battlefield', {})
        except (OSError, yaml.YAMLError, AttributeError) as exc:
            self.get_logger().warning(f'Failed to load battlefield config {config_file}: {exc}')
            return

        bounds = battlefield.get('map_bounds', {})
        self.map_min_x = float(bounds.get('min_x', self.map_min_x))
        self.map_max_x = float(bounds.get('max_x', self.map_max_x))
        self.map_min_y = float(bounds.get('min_y', self.map_min_y))
        self.map_max_y = float(bounds.get('max_y', self.map_max_y))

        insertion = battlefield.get('safe_insertion', {})
        self.safe_insertion_x = float(insertion.get('x', self.safe_insertion_x))
        self.safe_insertion_y = float(insertion.get('y', self.safe_insertion_y))
        self.safe_insertion_yaw = float(insertion.get('yaw', self.safe_insertion_yaw))

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

    @staticmethod
    def _pose_values_are_finite(pose: Pose) -> bool:
        values = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        if not all(math.isfinite(value) for value in values):
            return False
        norm = math.sqrt(
            pose.orientation.x * pose.orientation.x
            + pose.orientation.y * pose.orientation.y
            + pose.orientation.z * pose.orientation.z
            + pose.orientation.w * pose.orientation.w
        )
        return 0.5 <= norm <= 1.5

    @staticmethod
    def _yaw_from_pose(pose: Pose) -> float:
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _set_yaw(pose: Pose, yaw: float) -> None:
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = RescueCoordinator()
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

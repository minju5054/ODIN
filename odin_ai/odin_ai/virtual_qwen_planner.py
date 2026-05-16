import heapq
import json
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from urllib import error, request

import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from odin_interfaces.msg import HostageEvent
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


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


@dataclass
class CandidateRoute:
    name: str
    path: Path
    score: float
    reason: str
    metrics: Dict[str, float]


class VirtualQwenPlanner(Node):
    """Scenario-aware stand-in for Qwen route reasoning.

    This node receives coordinator-generated candidate routes and merged map,
    scores them against battlefield constraints, and publishes the selected
    rescue waypoint. Replace the scoring method with a real Qwen call later
    without changing the ROS interface.
    """

    def __init__(self) -> None:
        super().__init__('virtual_qwen_planner')

        self.declare_parameter('candidate_routes_topic', '/coordinator/candidate_routes')
        self.declare_parameter('hostage_event_topic', '/hostage_events')
        self.declare_parameter('merged_map_topic', '/merged_map')
        self.declare_parameter('recommendation_topic', '/ai/waypoint_recommendation')
        self.declare_parameter('selected_path_topic', '/ai/selected_path')
        self.declare_parameter('status_topic', '/ai/status')
        self.declare_parameter('mission_intent_topic', '/mission/intent')
        self.declare_parameter('mission_policy_topic', '/ai/mission_policy')
        self.declare_parameter('battlefield_config_file', '')
        self.declare_parameter('safe_insertion_x', 7.5)
        self.declare_parameter('safe_insertion_y', -7.5)
        self.declare_parameter('safe_insertion_yaw', 1.5708)
        self.declare_parameter('map_min_x', -10.0)
        self.declare_parameter('map_max_x', 10.0)
        self.declare_parameter('map_min_y', -10.0)
        self.declare_parameter('map_max_y', 10.0)
        self.declare_parameter('detour_offsets_m', [1.2, -1.2, 2.0, -2.0])
        self.declare_parameter('rule_hostage_standoff_m', 0.9)
        self.declare_parameter('path_waypoint_spacing_m', 0.45)
        self.declare_parameter('prefer_map_astar_routes', True)
        self.declare_parameter('astar_unknown_penalties', [3.0, 8.0, 15.0])
        self.declare_parameter('astar_obstacle_inflation_m', 0.18)
        self.declare_parameter('astar_max_expansions', 120000)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('allow_unknown_cells', True)
        self.declare_parameter('unknown_cell_penalty', 10.0)
        self.declare_parameter('occupied_cell_penalty', 1000.0)
        self.declare_parameter('route_segment_sample_step_m', 0.08)
        self.declare_parameter('fallback_route_penalty', 80.0)
        self.declare_parameter('red_zone_penalty', 1000.0)
        self.declare_parameter('enemy_vision_penalty', 600.0)
        self.declare_parameter('length_weight', 2.0)
        self.declare_parameter('turn_weight', 0.18)
        self.declare_parameter('use_remote_qwen', True)
        self.declare_parameter(
            'qwen_api_url',
            'http://10.42.0.14:8081/v1/chat/completions',
        )
        self.declare_parameter('qwen_model', 'Qwen3-VL-4B-Instruct-Q4_K_M.gguf')
        self.declare_parameter('qwen_timeout_sec', 25.0)
        self.declare_parameter('qwen_max_tokens', 80)
        self.declare_parameter('qwen_temperature', 0.1)
        self.declare_parameter('qwen_max_candidate_routes', 4)
        self.declare_parameter('mission_mode', 'SAFE_RESCUE')

        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_hostage_event: Optional[HostageEvent] = None
        self.safe_insertion_x = float(self.get_parameter('safe_insertion_x').value)
        self.safe_insertion_y = float(self.get_parameter('safe_insertion_y').value)
        self.safe_insertion_yaw = float(self.get_parameter('safe_insertion_yaw').value)
        self.map_min_x = float(self.get_parameter('map_min_x').value)
        self.map_max_x = float(self.get_parameter('map_max_x').value)
        self.map_min_y = float(self.get_parameter('map_min_y').value)
        self.map_max_y = float(self.get_parameter('map_max_y').value)
        self.detour_offsets_m = [
            float(offset) for offset in self.get_parameter('detour_offsets_m').value
        ]
        self.rule_hostage_standoff_m = float(
            self.get_parameter('rule_hostage_standoff_m').value
        )
        self.path_waypoint_spacing_m = float(self.get_parameter('path_waypoint_spacing_m').value)
        self.prefer_map_astar_routes = bool(self.get_parameter('prefer_map_astar_routes').value)
        self.astar_unknown_penalties = [
            float(value) for value in self.get_parameter('astar_unknown_penalties').value
        ]
        self.astar_obstacle_inflation_m = float(
            self.get_parameter('astar_obstacle_inflation_m').value
        )
        self.astar_max_expansions = int(self.get_parameter('astar_max_expansions').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.allow_unknown_cells = bool(self.get_parameter('allow_unknown_cells').value)
        self.unknown_cell_penalty = float(self.get_parameter('unknown_cell_penalty').value)
        self.occupied_cell_penalty = float(self.get_parameter('occupied_cell_penalty').value)
        self.route_segment_sample_step_m = float(
            self.get_parameter('route_segment_sample_step_m').value
        )
        self.fallback_route_penalty = float(self.get_parameter('fallback_route_penalty').value)
        self.red_zone_penalty = float(self.get_parameter('red_zone_penalty').value)
        self.enemy_vision_penalty = float(self.get_parameter('enemy_vision_penalty').value)
        self.length_weight = float(self.get_parameter('length_weight').value)
        self.turn_weight = float(self.get_parameter('turn_weight').value)
        self.use_remote_qwen = bool(self.get_parameter('use_remote_qwen').value)
        self.qwen_api_url = str(self.get_parameter('qwen_api_url').value)
        self.qwen_model = str(self.get_parameter('qwen_model').value)
        self.qwen_timeout_sec = float(self.get_parameter('qwen_timeout_sec').value)
        self.qwen_max_tokens = int(self.get_parameter('qwen_max_tokens').value)
        self.qwen_temperature = float(self.get_parameter('qwen_temperature').value)
        self.qwen_max_candidate_routes = int(self.get_parameter('qwen_max_candidate_routes').value)
        self.mission_mode = str(self.get_parameter('mission_mode').value).upper()
        self.score_weights = self._mission_weights(self.mission_mode)
        self.score_weights['fallback'] = self.fallback_route_penalty
        self.red_zones: List[RectangleZone] = []
        self.enemy_vision_zones: List[CircleZone] = []
        self._load_battlefield_config(str(self.get_parameter('battlefield_config_file').value))

        self.recommendation_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('recommendation_topic').value),
            10,
        )
        self.selected_path_pub = self.create_publisher(
            Path,
            str(self.get_parameter('selected_path_topic').value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.mission_policy_pub = self.create_publisher(
            String,
            str(self.get_parameter('mission_policy_topic').value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mission_intent_topic').value),
            self._mission_intent_callback,
            10,
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
            MarkerArray,
            str(self.get_parameter('candidate_routes_topic').value),
            self._candidate_routes_callback,
            10,
        )

        self._publish_status(
            'virtual_qwen_ready '
            f'mode={"remote_qwen" if self.use_remote_qwen else "local_heuristic"} '
            f'default_mission_mode={self.mission_mode} waiting_for_intent=true '
            f'api={self.qwen_api_url}'
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def _mission_intent_callback(self, msg: String) -> None:
        intent = msg.data.strip()
        if not intent:
            self._publish_status('mission_intent_ignored reason=empty')
            return

        policy, reason, mode = self._select_mission_policy(intent)
        self._set_mission_mode(policy)
        self._publish_policy(reason)
        self._publish_status(
            'mission_policy_selected '
            f'mode={self.mission_mode} selector={mode} reason={reason.replace(" ", "_")[:180]} '
            f'intent={intent.replace(" ", "_")[:180]}'
        )

    def _hostage_event_callback(self, msg: HostageEvent) -> None:
        self.latest_hostage_event = msg
        self._publish_status(
            'qwen_hostage_context_updated '
            f'marker_id={msg.marker_id} detecting_robot={msg.detecting_robot} '
            f'x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}'
        )

    def _candidate_routes_callback(self, msg: MarkerArray) -> None:
        candidate_routes = self._routes_from_markers(msg)
        if not candidate_routes:
            self._publish_status('qwen_rejected reason=no_candidate_routes')
            return

        viable_routes = [
            route for route in candidate_routes if not self._has_hard_route_block(route.path)
        ]
        selectable_routes = viable_routes if viable_routes else candidate_routes
        best, selection_mode, qwen_note = self._select_route(selectable_routes, candidate_routes)
        selected_goal = best.path.poses[-1]
        self.selected_path_pub.publish(best.path)
        self.recommendation_pub.publish(selected_goal)
        self._publish_status(
            'qwen_selected_route '
            f'name={best.name} mode={selection_mode} score={best.score:.2f} reason={best.reason} '
            f'goal_x={selected_goal.pose.position.x:.2f} goal_y={selected_goal.pose.position.y:.2f} '
            f'candidates={len(candidate_routes)} viable={len(viable_routes)} {qwen_note}'
        )

    def _select_route(
        self,
        selectable_routes: List[CandidateRoute],
        all_routes: List[CandidateRoute],
    ) -> Tuple[CandidateRoute, str, str]:
        local_best = min(selectable_routes, key=lambda route: route.score)
        local_mode = 'viable' if selectable_routes != all_routes else 'best_effort'
        if self.mission_mode == 'STEALTH_RESCUE':
            stealth_route = self._select_stealth_lower_left_upward(selectable_routes)
            if stealth_route is not None:
                return (
                    stealth_route,
                    f'stealth_lower_left_upward_{local_mode}',
                    'qwen=stealth_rule_lower_left_upward',
                )

        if not self.use_remote_qwen:
            return local_best, f'local_{local_mode}', 'qwen=disabled'

        self._publish_status(
            'qwen_request_started '
            f'api={self.qwen_api_url} selectable={len(selectable_routes)} '
            f'candidates={len(all_routes)}'
        )
        try:
            route_name, raw_reply = self._request_qwen_route(selectable_routes, all_routes)
        except (OSError, TimeoutError, error.URLError, json.JSONDecodeError, KeyError, ValueError) as exc:
            self._publish_status(
                'qwen_request_failed '
                f'error={type(exc).__name__}:{str(exc).replace(" ", "_")[:160]} '
                f'fallback={local_best.name}'
            )
            return local_best, f'fallback_{local_mode}', 'qwen=fallback_request_failed'

        selected = self._find_route_by_name(route_name, selectable_routes)
        if selected is None:
            self._publish_status(
                'qwen_response_unmatched '
                f'route={route_name} fallback={local_best.name} '
                f'reply={raw_reply.replace(" ", "_")[:160]}'
            )
            return local_best, f'fallback_{local_mode}', 'qwen=fallback_unmatched'

        self._publish_status(
            'qwen_response_received '
            f'route={selected.name} reply={raw_reply.replace(" ", "_")[:220]}'
        )
        return selected, 'remote_qwen', 'qwen=ok'

    def _select_stealth_lower_left_upward(
        self,
        routes: List[CandidateRoute],
    ) -> Optional[CandidateRoute]:
        upward_routes = []
        lower_left_count = 0
        for route in routes:
            upward_gain = self._lower_left_upward_gain(route.path)
            if upward_gain is None:
                continue
            lower_left_count += 1
            if upward_gain <= 0.0:
                continue
            upward_routes.append((route, upward_gain))

        if not upward_routes:
            reason = (
                'no_lower_left_waypoint_route'
                if lower_left_count == 0
                else 'no_lower_left_upward_route'
            )
            self._publish_status(f'stealth_rule_skipped reason={reason}')
            return None
        return max(upward_routes, key=lambda item: item[1])[0]

    def _lower_left_upward_gain(self, path: Path) -> Optional[float]:
        lower_left_x = self.map_min_x + (self.map_max_x - self.map_min_x) * 0.25
        lower_left_y = self.map_min_y + (self.map_max_y - self.map_min_y) * 0.35
        best_gain: Optional[float] = None
        for index, pose in enumerate(path.poses):
            point = pose.pose.position
            if point.x > lower_left_x or point.y > lower_left_y:
                continue
            for next_pose in path.poses[index + 1:]:
                next_point = next_pose.pose.position
                if next_point.x > lower_left_x:
                    continue
                gain = next_point.y - point.y
                if gain <= 0.5:
                    continue
                if best_gain is None or gain > best_gain:
                    best_gain = gain
        return best_gain

    def _select_mission_policy(self, intent: str) -> Tuple[str, str, str]:
        fallback_policy, fallback_reason = self._keyword_policy(intent)
        if not self.use_remote_qwen:
            return fallback_policy, fallback_reason, 'local_keyword'

        self._publish_status('mission_policy_request_started api=' + self.qwen_api_url)
        try:
            policy, reason, raw_reply = self._request_qwen_policy(intent)
        except (OSError, TimeoutError, error.URLError, json.JSONDecodeError, KeyError, ValueError) as exc:
            self._publish_status(
                'mission_policy_request_failed '
                f'error={type(exc).__name__}:{str(exc).replace(" ", "_")[:160]} '
                f'fallback={fallback_policy}'
            )
            return fallback_policy, fallback_reason, 'fallback_keyword'

        if policy not in self._known_mission_modes():
            self._publish_status(
                'mission_policy_unmatched '
                f'policy={policy} fallback={fallback_policy} '
                f'reply={raw_reply.replace(" ", "_")[:160]}'
            )
            return fallback_policy, fallback_reason, 'fallback_unmatched'

        return policy, reason, 'remote_qwen'

    def _request_qwen_policy(self, intent: str) -> Tuple[str, str, str]:
        payload = {
            'model': self.qwen_model,
            'messages': [
                {
                    'role': 'system',
                    'content': (
                        'Select one ODIN rescue policy from FAST_RESCUE, SAFE_RESCUE, '
                        'STEALTH_RESCUE. JSON only: {"policy":"name","reason":"short"}.'
                    ),
                },
                {
                    'role': 'user',
                    'content': (
                        f'mission_intent={intent}; '
                        'FAST_RESCUE means time critical and shortest route. '
                        'SAFE_RESCUE means use known/scouted areas and avoid unknown hazards. '
                        'STEALTH_RESCUE means never be detected; avoid enemy area and vision. '
                        'Choose exactly one policy.'
                    ),
                },
            ],
            'temperature': self.qwen_temperature,
            'max_tokens': 80,
            'stream': False,
        }
        body = json.dumps(payload).encode('utf-8')
        http_request = request.Request(
            self.qwen_api_url,
            data=body,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        with request.urlopen(http_request, timeout=self.qwen_timeout_sec) as response:
            result = json.loads(response.read().decode('utf-8'))
        content = result['choices'][0]['message']['content'].strip()
        parsed = self._parse_qwen_json(content)
        policy = str(parsed.get('policy', '')).strip().upper()
        reason = str(parsed.get('reason', content)).strip()
        if not policy:
            raise ValueError('Qwen response did not include a policy field')
        return policy, reason, content

    def _request_qwen_route(
        self,
        selectable_routes: List[CandidateRoute],
        all_routes: List[CandidateRoute],
    ) -> Tuple[str, str]:
        payload = {
            'model': self.qwen_model,
            'messages': [
                {
                    'role': 'system',
                    'content': (
                        'Pick one route using provided mode, formula, weights, and calc scores. '
                        'JSON only: {"route":"name","reason":"short"}.'
                    ),
                },
                {
                    'role': 'user',
                    'content': self._make_qwen_prompt(selectable_routes, all_routes),
                },
            ],
            'temperature': self.qwen_temperature,
            'max_tokens': self.qwen_max_tokens,
            'stream': False,
        }
        body = json.dumps(payload).encode('utf-8')
        http_request = request.Request(
            self.qwen_api_url,
            data=body,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        with request.urlopen(http_request, timeout=self.qwen_timeout_sec) as response:
            result = json.loads(response.read().decode('utf-8'))
        content = result['choices'][0]['message']['content'].strip()
        parsed = self._parse_qwen_json(content)
        route_name = str(parsed.get('route', '')).strip()
        if not route_name:
            raise ValueError('Qwen response did not include a route field')
        return route_name, content

    def _make_qwen_prompt(
        self,
        selectable_routes: List[CandidateRoute],
        all_routes: List[CandidateRoute],
    ) -> str:
        hostage = 'unknown'
        if self.latest_hostage_event is not None:
            hostage = (
                f'robot={self.latest_hostage_event.detecting_robot},'
                f'x={self.latest_hostage_event.pose.position.x:.2f},'
                f'y={self.latest_hostage_event.pose.position.y:.2f}'
            )

        route_limit = max(1, self.qwen_max_candidate_routes)
        route_summaries = [
            self._route_summary(route, route in selectable_routes)
            for route in self._routes_for_qwen(selectable_routes, all_routes, route_limit)
        ]
        return (
            f'mode={self.mission_mode}; '
            'score=w_len*length+w_unknown*unknown+w_occ*occupied+w_red*red+'
            'w_vision*vision+w_turn*turn+w_fallback*fallback+'
            'w_red_clearance*red_clearance; '
            f'weights={json.dumps(self.score_weights, separators=(",", ":"))}; '
            f'start=({self.safe_insertion_x:.1f},{self.safe_insertion_y:.1f}); '
            f'hostage={hostage}. '
            'Choose route name with lowest safe calc score. '
            f'routes={json.dumps(route_summaries, separators=(",", ":"))}'
        )

    @staticmethod
    def _routes_for_qwen(
        selectable_routes: List[CandidateRoute],
        all_routes: List[CandidateRoute],
        route_limit: int,
    ) -> List[CandidateRoute]:
        selectable_ids = {id(route) for route in selectable_routes}
        prioritized = [
            route for route in all_routes
            if route.name == 'planner_candidate'
        ]
        selected_ids = {id(route) for route in prioritized}
        remaining = sorted(
            all_routes,
            key=lambda route: (id(route) not in selectable_ids, route.score),
        )
        for route in remaining:
            if len(prioritized) >= route_limit:
                break
            if id(route) in selected_ids:
                continue
            prioritized.append(route)
            selected_ids.add(id(route))
        return prioritized[:route_limit]

    def _route_summary(self, route: CandidateRoute, selectable: bool) -> Dict[str, object]:
        poses = route.path.poses
        preview = []
        if poses:
            indexes = sorted(set([0, len(poses) // 2, len(poses) - 1]))
            for index in indexes:
                pose = poses[index].pose.position
                preview.append([round(pose.x, 2), round(pose.y, 2)])
        goal = poses[-1].pose.position if poses else None
        metrics = self._compact_metrics(route.reason)
        return {
            'n': route.name,
            'ok': selectable,
            'calc': round(route.score, 1),
            'p': preview,
            'v': {
                'length': round(route.metrics['length'], 2),
                'unknown': int(route.metrics['unknown']),
                'occupied': int(route.metrics['occupied']),
                'red': int(route.metrics['red']),
                'vision': int(route.metrics['vision']),
                'turn': round(route.metrics['turn'], 2),
                'red_clearance': round(route.metrics.get('red_clearance', 0.0), 2),
                'fallback': int(route.metrics['fallback']),
            },
        }

    @staticmethod
    def _compact_metrics(reason: str) -> str:
        wanted = []
        for item in reason.split(','):
            key = item.split('=', 1)[0]
            if key in ('length', 'unknown', 'occupied', 'red', 'vision', 'red_clearance'):
                wanted.append(item)
        return ','.join(wanted)

    def _map_summary(self) -> Dict[str, object]:
        if self.latest_map is None:
            return {'available': False}
        data = self.latest_map.data
        total = max(len(data), 1)
        unknown = sum(1 for value in data if value < 0)
        occupied = sum(1 for value in data if value >= self.occupied_threshold)
        free = total - unknown - occupied
        return {
            'available': True,
            'frame_id': self.latest_map.header.frame_id,
            'width': self.latest_map.info.width,
            'height': self.latest_map.info.height,
            'resolution': round(self.latest_map.info.resolution, 3),
            'free_ratio': round(free / total, 3),
            'unknown_ratio': round(unknown / total, 3),
            'occupied_ratio': round(occupied / total, 3),
        }

    @staticmethod
    def _parse_qwen_json(content: str) -> Dict[str, object]:
        try:
            return json.loads(content)
        except json.JSONDecodeError:
            start = content.find('{')
            end = content.rfind('}')
            if start < 0 or end <= start:
                raise
            return json.loads(content[start:end + 1])

    @staticmethod
    def _find_route_by_name(
        route_name: str,
        routes: List[CandidateRoute],
    ) -> Optional[CandidateRoute]:
        normalized = route_name.strip().lower()
        for route in routes:
            if route.name.strip().lower() == normalized:
                return route
        for route in routes:
            if normalized and normalized in route.name.strip().lower():
                return route
        return None

    def _routes_from_markers(self, msg: MarkerArray) -> List[CandidateRoute]:
        routes: List[CandidateRoute] = []
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL or marker.type != Marker.LINE_STRIP:
                continue
            if len(marker.points) < 2:
                continue
            path = Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = marker.header.frame_id or 'map'
            for index, point in enumerate(marker.points):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = point.x
                pose.pose.position.y = point.y
                pose.pose.position.z = point.z
                if index + 1 < len(marker.points):
                    next_point = marker.points[index + 1]
                    yaw = math.atan2(next_point.y - point.y, next_point.x - point.x)
                elif index > 0:
                    previous = marker.points[index - 1]
                    yaw = math.atan2(point.y - previous.y, point.x - previous.x)
                else:
                    yaw = self.safe_insertion_yaw
                self._set_yaw(pose, yaw)
                path.poses.append(pose)
            score, reason = self._score_path(path)
            metrics = self._metrics_from_reason(reason)
            name = marker.text or f'route_{marker.id}'
            routes.append(
                CandidateRoute(name=name, path=path, score=score, reason=reason, metrics=metrics)
            )
        return routes

    @staticmethod
    def _point_from_pose(pose: PoseStamped) -> Point:
        point = Point()
        point.x = pose.pose.position.x
        point.y = pose.pose.position.y
        point.z = pose.pose.position.z
        return point

    def _generate_candidate_routes(self, goal: PoseStamped) -> List[CandidateRoute]:
        routes: List[Tuple[str, Path]] = []
        if self.prefer_map_astar_routes:
            routes.extend(self._generate_astar_routes(goal))

        routes.extend(self._generate_strategic_routes(goal))
        routes.append(('direct', self._make_path([self._safe_start_pose(), goal])))

        start = self._safe_start_pose()
        start_x = start.pose.position.x
        start_y = start.pose.position.y
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.hypot(dx, dy)
        if distance > 1e-6:
            normal_x = -dy / distance
            normal_y = dx / distance
            for offset in self.detour_offsets_m:
                mid = PoseStamped()
                mid.header = goal.header
                mid.pose.position.x = (start_x + goal_x) / 2.0 + normal_x * offset
                mid.pose.position.y = (start_y + goal_y) / 2.0 + normal_y * offset
                self._set_yaw(mid, math.atan2(goal_y - start_y, goal_x - start_x))
                routes.append((f'detour_{offset:+.1f}m', self._make_path([start, mid, goal])))

        evaluated = []
        for name, route_path in routes:
            score, reason = self._score_path(route_path)
            metrics = self._metrics_from_reason(reason)
            if not name.startswith('map_astar_'):
                metrics['fallback'] = 1.0
                score = self._score_metrics(metrics)
                reason = self._reason_from_metrics(metrics)
            evaluated.append(
                CandidateRoute(name=name, path=route_path, score=score, reason=reason, metrics=metrics)
            )
        return evaluated

    def _generate_strategic_routes(self, goal: PoseStamped) -> List[Tuple[str, Path]]:
        start = self._safe_start_pose()
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        safe_x = min(max(self.safe_insertion_x, self.map_min_x + 1.0), self.map_max_x - 1.0)
        safe_y = min(max(self.safe_insertion_y, self.map_min_y + 1.0), self.map_max_y - 1.0)
        left_lane_x = min(max(self.map_min_x + 2.0, self.map_min_x), self.map_max_x)
        bottom_lane_y = min(max(self.map_min_y + 2.0, self.map_min_y), self.map_max_y)

        waypoints = [
            ('left_lane_then_goal', [(left_lane_x, safe_y), (left_lane_x, goal_y)]),
            ('bottom_lane_then_goal', [(safe_x, bottom_lane_y), (goal_x, bottom_lane_y)]),
            ('staged_safe_arc', [(left_lane_x, safe_y), (left_lane_x, bottom_lane_y), (goal_x, bottom_lane_y)]),
        ]
        routes: List[Tuple[str, Path]] = []
        for name, points in waypoints:
            anchors = [start]
            blocked = False
            for x, y in points:
                waypoint = PoseStamped()
                waypoint.header = goal.header
                waypoint.pose.position.x = x
                waypoint.pose.position.y = y
                self._set_yaw(waypoint, 0.0)
                if self._inside_red_zone(x, y):
                    blocked = True
                    break
                anchors.append(waypoint)
            if blocked:
                continue
            anchors.append(goal)
            routes.append((name, self._make_path(anchors)))
        return routes

    def _enemy_opposite_point(self) -> Tuple[float, float]:
        margin = 2.0
        center_x = (self.map_min_x + self.map_max_x) / 2.0
        center_y = (self.map_min_y + self.map_max_y) / 2.0
        enemy_x = center_x
        enemy_y = center_y
        if self.red_zones:
            red_zone = self.red_zones[0]
            enemy_x = (red_zone.min_x + red_zone.max_x) / 2.0
            enemy_y = (red_zone.min_y + red_zone.max_y) / 2.0
        elif self.enemy_vision_zones:
            enemy_x = self.enemy_vision_zones[0].center_x
            enemy_y = self.enemy_vision_zones[0].center_y

        opposite_x = self.map_min_x + margin if enemy_x >= center_x else self.map_max_x - margin
        opposite_y = self.map_min_y + margin if enemy_y >= center_y else self.map_max_y - margin
        return opposite_x, opposite_y

    def _generate_astar_routes(self, goal: PoseStamped) -> List[Tuple[str, Path]]:
        if self.latest_map is None:
            return []

        start_cell = self._world_to_cell(self.safe_insertion_x, self.safe_insertion_y)
        goal_cell = self._world_to_cell(goal.pose.position.x, goal.pose.position.y)
        if start_cell is None or goal_cell is None:
            return []
        start_cell = self._nearest_traversable_cell(start_cell) or start_cell
        goal_cell = self._nearest_traversable_cell(goal_cell) or goal_cell

        routes: List[Tuple[str, Path]] = []
        for penalty in self.astar_unknown_penalties:
            cells = self._astar_cells(start_cell, goal_cell, penalty)
            if not cells:
                continue
            path = self._path_from_cells(cells, goal.header.frame_id or 'map')
            if path.poses:
                path.poses[-1] = goal
                routes.append((f'map_astar_unknown_{penalty:.1f}', path))
        return routes

    def _nearest_traversable_cell(
        self,
        origin_cell: Tuple[int, int],
        max_radius_cells: int = 16,
    ) -> Optional[Tuple[int, int]]:
        if self._astar_cell_cost(origin_cell, unknown_penalty=3.0) is not None:
            return origin_cell

        best_cell = None
        best_distance = math.inf
        ox, oy = origin_cell
        for radius in range(1, max_radius_cells + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    cell = (ox + dx, oy + dy)
                    if self._astar_cell_cost(cell, unknown_penalty=3.0) is None:
                        continue
                    distance = math.hypot(dx, dy)
                    if distance < best_distance:
                        best_distance = distance
                        best_cell = cell
            if best_cell is not None:
                return best_cell
        return None

    def _astar_cells(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        unknown_penalty: float,
    ) -> List[Tuple[int, int]]:
        grid = self.latest_map
        if grid is None:
            return []

        queue: List[Tuple[float, int, Tuple[int, int]]] = []
        heapq.heappush(queue, (0.0, 0, start))
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        cost_so_far: Dict[Tuple[int, int], float] = {start: 0.0}
        counter = 0
        expansions = 0
        neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ]

        while queue and expansions < self.astar_max_expansions:
            _, _, current = heapq.heappop(queue)
            expansions += 1
            if current == goal:
                return self._reconstruct_cells(came_from, current)

            for dx, dy, distance_cost in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                cell_cost = self._astar_cell_cost(neighbor, unknown_penalty)
                if cell_cost is None:
                    continue
                new_cost = cost_so_far[current] + distance_cost + cell_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self._cell_distance(neighbor, goal)
                    counter += 1
                    heapq.heappush(queue, (priority, counter, neighbor))

        return []

    def _astar_cell_cost(
        self,
        cell: Tuple[int, int],
        unknown_penalty: float,
    ) -> Optional[float]:
        grid = self.latest_map
        if grid is None:
            return None
        cell_x, cell_y = cell
        value = self._cell_value(cell_x, cell_y)
        if value is None:
            return None

        x, y = self._cell_to_world(cell_x, cell_y)
        if self._inside_red_zone(x, y):
            return None

        if value >= self.occupied_threshold:
            return None
        if value < 0 and not self.allow_unknown_cells:
            return None

        obstacle_cost = self._inflated_obstacle_cost(cell_x, cell_y)
        if obstacle_cost is None:
            return None

        cost = obstacle_cost
        if value < 0:
            cost += unknown_penalty
        if self._inside_enemy_vision_zone(x, y):
            cost += 20.0
        return cost

    def _inflated_obstacle_cost(self, cell_x: int, cell_y: int) -> Optional[float]:
        grid = self.latest_map
        if grid is None:
            return None
        radius_cells = max(0, int(math.ceil(self.astar_obstacle_inflation_m / grid.info.resolution)))
        if radius_cells <= 0:
            return 0.0

        nearest_sq: Optional[int] = None
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx == 0 and dy == 0:
                    continue
                value = self._cell_value(cell_x + dx, cell_y + dy)
                if value is None:
                    continue
                if value < self.occupied_threshold:
                    continue
                dist_sq = dx * dx + dy * dy
                if dist_sq <= radius_cells * radius_cells:
                    if dist_sq <= 1:
                        return None
                    if nearest_sq is None or dist_sq < nearest_sq:
                        nearest_sq = dist_sq

        if nearest_sq is None:
            return 0.0
        clearance = math.sqrt(nearest_sq) / max(radius_cells, 1)
        return max(0.0, 4.0 * (1.0 - clearance))

    def _path_from_cells(self, cells: List[Tuple[int, int]], frame_id: str) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id

        for index, (cell_x, cell_y) in enumerate(cells):
            x, y = self._cell_to_world(cell_x, cell_y)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if index + 1 < len(cells):
                nx, ny = self._cell_to_world(*cells[index + 1])
                yaw = math.atan2(ny - y, nx - x)
            elif index > 0:
                px, py = self._cell_to_world(*cells[index - 1])
                yaw = math.atan2(y - py, x - px)
            else:
                yaw = self.safe_insertion_yaw
            self._set_yaw(pose, yaw)
            path.poses.append(pose)
        return path

    @staticmethod
    def _reconstruct_cells(
        came_from: Dict[Tuple[int, int], Tuple[int, int]],
        current: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        cells = [current]
        while current in came_from:
            current = came_from[current]
            cells.append(current)
        cells.reverse()
        return cells

    def _simplify_cells(self, cells: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        if len(cells) <= 2:
            return cells

        simplified = [cells[0]]
        previous_direction = (
            cells[1][0] - cells[0][0],
            cells[1][1] - cells[0][1],
        )
        for index in range(2, len(cells)):
            direction = (
                cells[index][0] - cells[index - 1][0],
                cells[index][1] - cells[index - 1][1],
            )
            if direction != previous_direction:
                simplified.append(cells[index - 1])
                previous_direction = direction
        simplified.append(cells[-1])
        return simplified

    @staticmethod
    def _cell_distance(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _make_path(self, anchors: List[PoseStamped]) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = anchors[-1].header.frame_id or 'map'

        for start, goal in zip(anchors[:-1], anchors[1:]):
            sx = start.pose.position.x
            sy = start.pose.position.y
            gx = goal.pose.position.x
            gy = goal.pose.position.y
            distance = math.hypot(gx - sx, gy - sy)
            steps = max(1, int(math.ceil(distance / max(self.path_waypoint_spacing_m, 0.1))))
            for index in range(steps):
                ratio = index / steps
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = sx + (gx - sx) * ratio
                pose.pose.position.y = sy + (gy - sy) * ratio
                self._set_yaw(pose, math.atan2(gy - sy, gx - sx))
                path.poses.append(pose)

        final_goal = anchors[-1]
        final_goal.header = path.header
        path.poses.append(final_goal)
        return path

    def _score_path(self, path: Path) -> Tuple[float, str]:
        unknown_cells = 0
        occupied_cells = 0
        red_hits = 0
        vision_hits = 0
        length = 0.0
        turn_cost = 0.0
        red_clearance = 0.0
        min_red_distance: Optional[float] = None
        previous_yaw = None
        previous_pose = None

        for pose in self._sample_path(path):
            x = pose.pose.position.x
            y = pose.pose.position.y
            if previous_pose is not None:
                px = previous_pose.pose.position.x
                py = previous_pose.pose.position.y
                segment = math.hypot(x - px, y - py)
                length += segment
                yaw = math.atan2(y - py, x - px)
                if previous_yaw is not None:
                    turn_cost += abs(self._normalize_angle(yaw - previous_yaw))
                previous_yaw = yaw
            previous_pose = pose

            if self._inside_red_zone(x, y):
                red_hits += 1
            if self._inside_enemy_vision_zone(x, y):
                vision_hits += 1
            red_distance = self._distance_to_nearest_red_zone(x, y)
            if red_distance is not None:
                if min_red_distance is None or red_distance < min_red_distance:
                    min_red_distance = red_distance

            value = self._map_value_at(x, y)
            if value is None:
                occupied_cells += 1
            elif value < 0:
                unknown_cells += 1
                if not self.allow_unknown_cells:
                    occupied_cells += 1
            elif value >= self.occupied_threshold:
                occupied_cells += 1

        if min_red_distance is not None:
            red_clearance = min(min_red_distance, 8.0)

        metrics = {
            'length': length,
            'turn': turn_cost,
            'unknown': float(unknown_cells),
            'occupied': float(occupied_cells),
            'red': float(red_hits),
            'vision': float(vision_hits),
            'red_clearance': red_clearance,
            'fallback': 0.0,
        }
        score = self._score_metrics(metrics)
        reason = self._reason_from_metrics(metrics)
        return score, reason

    def _score_metrics(self, metrics: Dict[str, float]) -> float:
        return (
            self.score_weights['length'] * metrics.get('length', 0.0)
            + self.score_weights['turn'] * metrics.get('turn', 0.0)
            + self.score_weights['unknown'] * metrics.get('unknown', 0.0)
            + self.score_weights['occupied'] * metrics.get('occupied', 0.0)
            + self.score_weights['red'] * metrics.get('red', 0.0)
            + self.score_weights['vision'] * metrics.get('vision', 0.0)
            + self.score_weights.get('red_clearance', 0.0) * metrics.get('red_clearance', 0.0)
            + self.score_weights['fallback'] * metrics.get('fallback', 0.0)
        )

    @staticmethod
    def _reason_from_metrics(metrics: Dict[str, float]) -> str:
        return (
            f'length={metrics.get("length", 0.0):.2f},'
            f'unknown={int(metrics.get("unknown", 0.0))},'
            f'occupied={int(metrics.get("occupied", 0.0))},'
            f'red={int(metrics.get("red", 0.0))},'
            f'vision={int(metrics.get("vision", 0.0))},'
            f'turn={metrics.get("turn", 0.0):.2f},'
            f'red_clearance={metrics.get("red_clearance", 0.0):.2f},'
            f'fallback={int(metrics.get("fallback", 0.0))}'
        )

    def _set_mission_mode(self, mode: str) -> None:
        normalized = mode.upper()
        if normalized not in self._known_mission_modes():
            normalized = 'SAFE_RESCUE'
        self.mission_mode = normalized
        self.score_weights = self._mission_weights(self.mission_mode)
        self.score_weights['fallback'] = self.fallback_route_penalty

    def _publish_policy(self, reason: str) -> None:
        msg = String()
        msg.data = self.mission_mode
        self.mission_policy_pub.publish(msg)
        self._publish_status(
            f'mission_policy_active mode={self.mission_mode} reason={reason.replace(" ", "_")[:180]}'
        )

    @staticmethod
    def _known_mission_modes() -> Tuple[str, ...]:
        return ('FAST_RESCUE', 'SAFE_RESCUE', 'STEALTH_RESCUE', 'BALANCED')

    @staticmethod
    def _keyword_policy(intent: str) -> Tuple[str, str]:
        text = intent.lower()
        stealth_keywords = (
            'stealth', 'hidden', 'undetected', 'avoid detection', 'covert',
            '들키', '은밀', '잠입', '발각', '적에게', '시야',
        )
        fast_keywords = (
            'fast', 'urgent', 'quick', 'shortest', 'asap', 'time critical',
            '빨리', '급해', '긴급', '최단', '신속', '시간',
        )
        safe_keywords = (
            'safe', 'careful', 'known', 'scouted', 'avoid unknown', 'mine', 'ambush',
            '안전', '조심', '알려진', '정찰된', '미탐사', '지뢰', '매복',
        )
        if any(keyword in text for keyword in stealth_keywords):
            return 'STEALTH_RESCUE', 'keyword matched stealth / avoid detection intent'
        if any(keyword in text for keyword in fast_keywords):
            return 'FAST_RESCUE', 'keyword matched time critical rescue intent'
        if any(keyword in text for keyword in safe_keywords):
            return 'SAFE_RESCUE', 'keyword matched safe known-route rescue intent'
        return 'SAFE_RESCUE', 'defaulted to safe known-route rescue'

    @staticmethod
    def _metrics_from_reason(reason: str) -> Dict[str, float]:
        metrics = {
            'length': 0.0,
            'unknown': 0.0,
            'occupied': 0.0,
            'red': 0.0,
            'vision': 0.0,
            'turn': 0.0,
            'red_clearance': 0.0,
            'fallback': 0.0,
        }
        for item in reason.split(','):
            if '=' not in item:
                continue
            key, value = item.split('=', 1)
            if key not in metrics:
                continue
            try:
                metrics[key] = float(value)
            except ValueError:
                continue
        return metrics

    @staticmethod
    def _mission_weights(mode: str) -> Dict[str, float]:
        weight_sets = {
            'FAST_RESCUE': {
                'length': 12.0,
                'unknown': 0.5,
                'occupied': 1000.0,
                'red': 1000.0,
                'vision': 450.0,
                'turn': 0.1,
                'red_clearance': 0.0,
                'fallback': 80.0,
            },
            'SAFE_RESCUE': {
                'length': 1.0,
                'unknown': 18.0,
                'occupied': 1000.0,
                'red': 1000.0,
                'vision': 600.0,
                'turn': 0.18,
                'red_clearance': 0.0,
                'fallback': 80.0,
            },
            'STEALTH_RESCUE': {
                'length': 1.0,
                'unknown': 18.0,
                'occupied': 1000.0,
                'red': 1500.0,
                'vision': 1200.0,
                'turn': 0.2,
                'red_clearance': -120.0,
                'fallback': 80.0,
            },
            'BALANCED': {
                'length': 2.0,
                'unknown': 6.0,
                'occupied': 1000.0,
                'red': 1000.0,
                'vision': 600.0,
                'turn': 0.18,
                'red_clearance': 0.0,
                'fallback': 80.0,
            },
        }
        return weight_sets.get(mode, weight_sets['SAFE_RESCUE'])

    def _has_hard_route_block(self, path: Path) -> bool:
        for pose in self._sample_path(path):
            x = pose.pose.position.x
            y = pose.pose.position.y
            if self._inside_red_zone(x, y):
                return True

            value = self._map_value_at(x, y)
            if value is None:
                return True
            if value < 0 and not self.allow_unknown_cells:
                return True
            if value >= self.occupied_threshold:
                return True
        return False

    def _sample_path(self, path: Path) -> List[PoseStamped]:
        if len(path.poses) < 2:
            return list(path.poses)

        samples: List[PoseStamped] = []
        step = max(self.route_segment_sample_step_m, 0.03)
        for start, goal in zip(path.poses[:-1], path.poses[1:]):
            sx = start.pose.position.x
            sy = start.pose.position.y
            gx = goal.pose.position.x
            gy = goal.pose.position.y
            distance = math.hypot(gx - sx, gy - sy)
            steps = max(1, int(math.ceil(distance / step)))
            for index in range(steps):
                ratio = index / steps
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = sx + (gx - sx) * ratio
                pose.pose.position.y = sy + (gy - sy) * ratio
                self._set_yaw(pose, math.atan2(gy - sy, gx - sx))
                samples.append(pose)
        samples.append(path.poses[-1])
        return samples

    def _safe_start_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.safe_insertion_x
        pose.pose.position.y = self.safe_insertion_y
        self._set_yaw(pose, self.safe_insertion_yaw)
        return pose

    def _map_value_at(self, x: float, y: float) -> Optional[int]:
        if self.latest_map is None:
            return -1 if self.allow_unknown_cells else None
        origin = self.latest_map.info.origin.position
        resolution = self.latest_map.info.resolution
        cell_x = int(math.floor((x - origin.x) / resolution))
        cell_y = int(math.floor((y - origin.y) / resolution))
        if not (0 <= cell_x < self.latest_map.info.width and 0 <= cell_y < self.latest_map.info.height):
            return None
        return int(self.latest_map.data[cell_y * self.latest_map.info.width + cell_x])

    def _world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self.latest_map is None:
            return None
        origin = self.latest_map.info.origin.position
        resolution = self.latest_map.info.resolution
        cell_x = int(math.floor((x - origin.x) / resolution))
        cell_y = int(math.floor((y - origin.y) / resolution))
        if 0 <= cell_x < self.latest_map.info.width and 0 <= cell_y < self.latest_map.info.height:
            return cell_x, cell_y
        return None

    def _cell_to_world(self, cell_x: int, cell_y: int) -> Tuple[float, float]:
        if self.latest_map is None:
            return 0.0, 0.0
        origin = self.latest_map.info.origin.position
        resolution = self.latest_map.info.resolution
        return (
            origin.x + (cell_x + 0.5) * resolution,
            origin.y + (cell_y + 0.5) * resolution,
        )

    def _cell_value(self, cell_x: int, cell_y: int) -> Optional[int]:
        if self.latest_map is None:
            return None
        if 0 <= cell_x < self.latest_map.info.width and 0 <= cell_y < self.latest_map.info.height:
            return int(self.latest_map.data[cell_y * self.latest_map.info.width + cell_x])
        return None

    def _inside_red_zone(self, x: float, y: float) -> bool:
        for zone in self.red_zones:
            if not zone.hard_reject:
                continue
            if zone.min_x <= x <= zone.max_x and zone.min_y <= y <= zone.max_y:
                return True
        return False

    def _distance_to_nearest_red_zone(self, x: float, y: float) -> Optional[float]:
        nearest: Optional[float] = None
        for zone in self.red_zones:
            if not zone.hard_reject:
                continue
            dx = max(zone.min_x - x, 0.0, x - zone.max_x)
            dy = max(zone.min_y - y, 0.0, y - zone.max_y)
            distance = math.hypot(dx, dy)
            if nearest is None or distance < nearest:
                nearest = distance
        return nearest

    def _inside_enemy_vision_zone(self, x: float, y: float) -> bool:
        for zone in self.enemy_vision_zones:
            if not zone.hard_reject:
                continue
            if math.hypot(x - zone.center_x, y - zone.center_y) <= zone.radius_m:
                return True
        return False

    def _load_battlefield_config(self, config_file: str) -> None:
        if not config_file:
            self.get_logger().info('No battlefield_config_file provided; using AI params only')
            return

        try:
            with open(config_file, 'r', encoding='utf-8') as config:
                battlefield = yaml.safe_load(config).get('battlefield', {})
        except (OSError, yaml.YAMLError, AttributeError) as exc:
            self.get_logger().warning(f'Failed to load battlefield config {config_file}: {exc}')
            return

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
            'Loaded virtual Qwen battlefield context: '
            f'{len(self.red_zones)} red zone(s), {len(self.enemy_vision_zones)} vision zone(s)'
        )

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    @staticmethod
    def _set_yaw(pose_stamped: PoseStamped, yaw: float) -> None:
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
        pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = VirtualQwenPlanner()
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

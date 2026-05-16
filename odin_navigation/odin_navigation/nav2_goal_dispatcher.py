import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Nav2GoalDispatcher(Node):
    """Forward validated robot_3 goals to Nav2 NavigateToPose."""

    def __init__(self) -> None:
        super().__init__('nav2_goal_dispatcher')

        self.declare_parameter('goal_topic', '/robot_3/goal_pose')
        self.declare_parameter('selected_path_topic', '/ai/selected_path')
        self.declare_parameter('status_topic', '/robot_3/dispatch_status')
        self.declare_parameter('navigate_action', '/robot_3/navigate_to_pose')
        self.declare_parameter('server_check_period_sec', 0.5)
        self.declare_parameter('feedback_period_sec', 1.0)
        self.declare_parameter('follow_selected_path', True)
        self.declare_parameter('selected_path_waypoint_spacing_m', 1.0)
        self.declare_parameter('selected_path_goal_tolerance_m', 0.35)
        self.declare_parameter('selected_path_sync_timeout_sec', 1.0)

        self.pending_goal: Optional[PoseStamped] = None
        self.pending_final_goal: Optional[PoseStamped] = None
        self.pending_waypoints: List[PoseStamped] = []
        self.latest_selected_path: Optional[Path] = None
        self.goal_active = False
        self.goal_wait_started_sec = 0.0
        self.last_feedback_time_sec = 0.0
        self.feedback_period_sec = float(self.get_parameter('feedback_period_sec').value)
        self.follow_selected_path = bool(self.get_parameter('follow_selected_path').value)
        self.selected_path_waypoint_spacing_m = float(
            self.get_parameter('selected_path_waypoint_spacing_m').value
        )
        self.selected_path_goal_tolerance_m = float(
            self.get_parameter('selected_path_goal_tolerance_m').value
        )
        self.selected_path_sync_timeout_sec = float(
            self.get_parameter('selected_path_sync_timeout_sec').value
        )

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('navigate_action').value),
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._goal_callback,
            10,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter('selected_path_topic').value),
            self._selected_path_callback,
            10,
        )
        self.create_timer(
            float(self.get_parameter('server_check_period_sec').value),
            self._dispatch_loop,
        )

        self._publish_status('nav2_dispatcher_ready waiting_for_goal')

    def _selected_path_callback(self, msg: Path) -> None:
        if not msg.poses:
            return
        self.latest_selected_path = msg
        self._publish_status(f'selected_path_cached waypoints={len(msg.poses)}')
        if self.pending_final_goal is not None and not self.goal_active:
            self._queue_selected_path_goal(self.pending_final_goal)

    def _goal_callback(self, msg: PoseStamped) -> None:
        self.pending_final_goal = msg
        self.pending_goal = None
        self.pending_waypoints = []
        self.goal_active = False
        self.goal_wait_started_sec = self._now_sec()
        if not self._queue_goal(msg, allow_wait=True):
            self._publish_status(
                'goal_waiting_for_selected_path '
                f'timeout={self.selected_path_sync_timeout_sec:.2f} '
                f'x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}'
            )

    def _queue_goal(self, msg: PoseStamped, allow_wait: bool) -> bool:
        if self._queue_selected_path_goal(msg):
            return True

        if allow_wait and self.follow_selected_path and self.selected_path_sync_timeout_sec > 0.0:
            return False

        self.pending_goal = msg
        self.pending_final_goal = None
        self._publish_status(
            f'nav2_goal_queued x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}'
        )
        return True

    def _queue_selected_path_goal(self, msg: PoseStamped) -> bool:
        self.pending_waypoints = self._make_selected_path_waypoints(msg)
        if not self.pending_waypoints:
            return False
        self.pending_goal = self.pending_waypoints.pop(0)
        self.pending_final_goal = None
        self._publish_status(
            'selected_path_follow_queued '
            f'waypoints_remaining={len(self.pending_waypoints) + 1} '
            f'final_x={msg.pose.position.x:.2f} final_y={msg.pose.position.y:.2f}'
        )
        return True

    def _dispatch_loop(self) -> None:
        if self.pending_final_goal is not None and not self.goal_active:
            elapsed = self._now_sec() - self.goal_wait_started_sec
            if elapsed >= self.selected_path_sync_timeout_sec:
                goal = self.pending_final_goal
                self._publish_status(
                    'selected_path_sync_timeout '
                    f'elapsed={elapsed:.2f} fallback=goal_pose'
                )
                self._queue_goal(goal, allow_wait=False)

        if self.pending_goal is None or self.goal_active:
            return
        if not self.navigate_client.server_is_ready():
            self._publish_status('nav2_waiting reason=navigate_to_pose_unavailable')
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.pending_goal
        self.goal_active = True
        send_future = self.navigate_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        send_future.add_done_callback(self._goal_response_callback)
        self._publish_status('nav2_goal_sent')

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_active = False
            self.pending_waypoints = []
            self._publish_status('nav2_goal_rejected')
            return

        self._publish_status('nav2_goal_accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_feedback_time_sec < self.feedback_period_sec:
            return
        self.last_feedback_time_sec = now_sec
        feedback = feedback_msg.feedback
        self._publish_status(
            'nav2_feedback '
            f'distance_remaining={feedback.distance_remaining:.2f} '
            f'nav_time={feedback.navigation_time.sec}s'
        )

    def _result_callback(self, future) -> None:
        result = future.result()
        self.goal_active = False
        self._publish_status(f'nav2_result status={result.status}')
        if result.status == 4 and self.pending_waypoints:
            self.pending_goal = self.pending_waypoints.pop(0)
            self._publish_status(
                'selected_path_waypoint_advance '
                f'waypoints_remaining={len(self.pending_waypoints)} '
                f'x={self.pending_goal.pose.position.x:.2f} '
                f'y={self.pending_goal.pose.position.y:.2f}'
            )
            return
        if result.status != 4 and self.pending_waypoints:
            self.pending_goal = self.pending_waypoints.pop(0)
            self._publish_status(
                'selected_path_waypoint_skipped '
                f'failed_status={result.status} '
                f'waypoints_remaining={len(self.pending_waypoints)} '
                f'x={self.pending_goal.pose.position.x:.2f} '
                f'y={self.pending_goal.pose.position.y:.2f}'
            )
            return

        self.pending_goal = None
        self.pending_final_goal = None
        self.pending_waypoints = []
        if result.status == 4:
            self._publish_status('selected_path_follow_complete')
            self._publish_status('dispatch_complete')

    def _make_selected_path_waypoints(self, final_goal: PoseStamped) -> List[PoseStamped]:
        if not self.follow_selected_path or self.latest_selected_path is None:
            return []
        path = self.latest_selected_path
        if not path.poses or not self._path_matches_goal(path, final_goal):
            return []

        waypoints: List[PoseStamped] = []
        last_x = path.poses[0].pose.position.x
        last_y = path.poses[0].pose.position.y
        spacing = max(self.selected_path_waypoint_spacing_m, 0.2)
        for pose in path.poses[1:]:
            x = pose.pose.position.x
            y = pose.pose.position.y
            if math.hypot(x - last_x, y - last_y) < spacing:
                continue
            waypoints.append(pose)
            last_x = x
            last_y = y

        if not waypoints or self._distance(waypoints[-1], final_goal) > 0.05:
            waypoints.append(final_goal)
        else:
            waypoints[-1] = final_goal

        return waypoints

    def _path_matches_goal(self, path: Path, goal: PoseStamped) -> bool:
        return self._distance(path.poses[-1], goal) <= self.selected_path_goal_tolerance_m

    @staticmethod
    def _distance(a: PoseStamped, b: PoseStamped) -> float:
        return math.hypot(
            a.pose.position.x - b.pose.position.x,
            a.pose.position.y - b.pose.position.y,
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = Nav2GoalDispatcher()
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

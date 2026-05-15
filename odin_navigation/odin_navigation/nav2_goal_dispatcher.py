from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Nav2GoalDispatcher(Node):
    """Forward validated robot_3 goals to Nav2 NavigateToPose."""

    def __init__(self) -> None:
        super().__init__('nav2_goal_dispatcher')

        self.declare_parameter('goal_topic', '/robot_3/goal_pose')
        self.declare_parameter('status_topic', '/robot_3/dispatch_status')
        self.declare_parameter('navigate_action', '/robot_3/navigate_to_pose')
        self.declare_parameter('server_check_period_sec', 0.5)
        self.declare_parameter('feedback_period_sec', 1.0)

        self.pending_goal: Optional[PoseStamped] = None
        self.goal_active = False
        self.last_feedback_time_sec = 0.0
        self.feedback_period_sec = float(self.get_parameter('feedback_period_sec').value)

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
        self.create_timer(
            float(self.get_parameter('server_check_period_sec').value),
            self._dispatch_loop,
        )

        self._publish_status('nav2_dispatcher_ready waiting_for_goal')

    def _goal_callback(self, msg: PoseStamped) -> None:
        self.pending_goal = msg
        self.goal_active = False
        self._publish_status(
            f'nav2_goal_queued x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}'
        )

    def _dispatch_loop(self) -> None:
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
        self.pending_goal = None
        self._publish_status(f'nav2_result status={result.status}')

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

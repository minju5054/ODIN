from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class MissionSuccessMarker(Node):
    """Publish an RViz text marker when robot_3 completes its Nav2 dispatch."""

    def __init__(self) -> None:
        super().__init__('mission_success_marker')

        self.declare_parameter('dispatch_status_topic', '/robot_3/dispatch_status')
        self.declare_parameter('mission_status_topic', '/mission_status')
        self.declare_parameter('marker_topic', '/mission_marker')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('text', 'MISSION SUCCESS')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 1.2)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.text = str(self.get_parameter('text').value)
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.z = float(self.get_parameter('z').value)
        self.mission_succeeded = False

        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.marker_pub = self.create_publisher(
            Marker,
            str(self.get_parameter('marker_topic').value),
            marker_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('mission_status_topic').value),
            marker_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('dispatch_status_topic').value),
            self._status_callback,
            10,
        )
        self.create_timer(1.0, self._publish_marker_if_needed)

        self.get_logger().info('Mission success marker ready.')

    def _status_callback(self, msg: String) -> None:
        if self.mission_succeeded:
            return
        if 'nav2_result status=4' not in msg.data and 'goal_reached' not in msg.data:
            return

        self.mission_succeeded = True
        status = String()
        status.data = 'mission_success'
        self.status_pub.publish(status)
        self._publish_marker()
        self.get_logger().info('Mission success detected.')

    def _publish_marker_if_needed(self) -> None:
        if self.mission_succeeded:
            self._publish_marker()

    def _publish_marker(self) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = 'odin_mission'
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = self.z
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.85
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.25
        marker.color.a = 1.0
        marker.text = self.text
        marker.lifetime = Duration(seconds=0.0).to_msg()
        self.marker_pub.publish(marker)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = MissionSuccessMarker()
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

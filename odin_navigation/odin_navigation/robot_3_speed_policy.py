from typing import Dict, Optional

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Robot3SpeedPolicy(Node):
    """Adjust robot_3 Nav2 speed limits from the active mission policy."""

    def __init__(self) -> None:
        super().__init__('robot_3_speed_policy')

        self.declare_parameter('mission_policy_topic', '/ai/mission_policy')
        self.declare_parameter('controller_set_parameters_service', '/robot_3/controller_server/set_parameters')
        self.declare_parameter('velocity_smoother_set_parameters_service', '/robot_3/velocity_smoother/set_parameters')
        self.declare_parameter('normal_linear_speed_mps', 0.26)
        self.declare_parameter('stealth_linear_speed_mps', 0.40)
        self.declare_parameter('angular_speed_radps', 0.85)
        self.declare_parameter('retry_period_sec', 1.0)

        self.normal_linear_speed_mps = float(self.get_parameter('normal_linear_speed_mps').value)
        self.stealth_linear_speed_mps = float(self.get_parameter('stealth_linear_speed_mps').value)
        self.angular_speed_radps = float(self.get_parameter('angular_speed_radps').value)
        self.target_speed_mps = self.normal_linear_speed_mps
        self.pending_policy: Optional[str] = None
        self.applied_policy: Optional[str] = None
        self.pending_clients: Dict[str, bool] = {}

        self.controller_client = self.create_client(
            SetParameters,
            str(self.get_parameter('controller_set_parameters_service').value),
        )
        self.smoother_client = self.create_client(
            SetParameters,
            str(self.get_parameter('velocity_smoother_set_parameters_service').value),
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mission_policy_topic').value),
            self._policy_callback,
            10,
        )
        self.create_timer(float(self.get_parameter('retry_period_sec').value), self._retry_pending)
        self.get_logger().info(
            'robot_3_speed_policy_ready '
            f'normal={self.normal_linear_speed_mps:.2f}mps '
            f'stealth={self.stealth_linear_speed_mps:.2f}mps'
        )

    def _policy_callback(self, msg: String) -> None:
        policy = msg.data.strip().upper()
        if not policy:
            return
        speed = (
            self.stealth_linear_speed_mps
            if policy == 'STEALTH_RESCUE'
            else self.normal_linear_speed_mps
        )
        if self.applied_policy == policy and abs(self.target_speed_mps - speed) < 1e-6:
            return
        self.pending_policy = policy
        self.target_speed_mps = speed
        self.pending_clients = {'controller': False, 'smoother': False}
        self._apply_policy()

    def _retry_pending(self) -> None:
        if self.pending_policy is None:
            return
        self._apply_policy()

    def _apply_policy(self) -> None:
        if self.pending_policy is None:
            return

        if not self.pending_clients.get('controller', True):
            self._set_controller_speed(self.target_speed_mps)
        if not self.pending_clients.get('smoother', True):
            self._set_smoother_speed(self.target_speed_mps)

        if all(self.pending_clients.values()):
            self.applied_policy = self.pending_policy
            self.get_logger().info(
                'robot_3_speed_policy_applied '
                f'policy={self.applied_policy} linear={self.target_speed_mps:.2f}mps'
            )
            self.pending_policy = None

    def _set_controller_speed(self, speed_mps: float) -> None:
        if not self.controller_client.service_is_ready():
            self.get_logger().info('robot_3_speed_policy_waiting target=controller_server')
            return
        request = SetParameters.Request()
        request.parameters = [
            self._double_parameter('FollowPath.max_vel_x', speed_mps),
            self._double_parameter('FollowPath.max_speed_xy', speed_mps),
        ]
        future = self.controller_client.call_async(request)
        future.add_done_callback(lambda result: self._set_done('controller', result))

    def _set_smoother_speed(self, speed_mps: float) -> None:
        if not self.smoother_client.service_is_ready():
            self.get_logger().info('robot_3_speed_policy_waiting target=velocity_smoother')
            return
        request = SetParameters.Request()
        request.parameters = [
            self._double_array_parameter(
                'max_velocity',
                [speed_mps, 0.0, self.angular_speed_radps],
            ),
        ]
        future = self.smoother_client.call_async(request)
        future.add_done_callback(lambda result: self._set_done('smoother', result))

    def _set_done(self, client_name: str, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - ROS service failure path
            self.get_logger().warn(
                f'robot_3_speed_policy_failed target={client_name} error={exc}'
            )
            return

        if not response.results or not all(result.successful for result in response.results):
            reason = response.results[0].reason if response.results else 'empty response'
            self.get_logger().warn(
                f'robot_3_speed_policy_rejected target={client_name} reason={reason}'
            )
            return
        self.pending_clients[client_name] = True

    @staticmethod
    def _double_parameter(name: str, value: float) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=float(value),
        )
        return parameter

    @staticmethod
    def _double_array_parameter(name: str, values) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            double_array_value=[float(value) for value in values],
        )
        return parameter


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = Robot3SpeedPolicy()
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

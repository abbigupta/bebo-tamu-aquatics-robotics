"""Forward the common ASV velocity command to ArduPilot through MAVROS."""

from __future__ import annotations

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MavrosVelocityAdapter(Node):
    """Publish fresh common velocity commands on the MAVROS setpoint topic."""

    def __init__(self) -> None:
        super().__init__('mavros_velocity_adapter')
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter(
            'mavros_command_topic', '/mavros/setpoint_velocity/cmd_vel'
        )
        self.timeout_ns = int(
            float(self.get_parameter('command_timeout_s').value) * 1e9
        )
        self.last_command: TwistStamped | None = None
        self.received_ns: int | None = None
        self.publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('mavros_command_topic').value),
            10,
        )
        self.create_subscription(TwistStamped, '/asv/cmd_vel', self._command, 10)
        self.create_timer(0.1, self._tick)

    def _command(self, message: TwistStamped) -> None:
        self.last_command = message
        self.received_ns = self.get_clock().now().nanoseconds

    def _tick(self) -> None:
        now = self.get_clock().now()
        if (
            self.last_command is not None
            and self.received_ns is not None
            and now.nanoseconds - self.received_ns <= self.timeout_ns
        ):
            output = self.last_command
        else:
            output = TwistStamped()
            output.header.frame_id = 'base_link'
        output.header.stamp = now.to_msg()
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MavrosVelocityAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

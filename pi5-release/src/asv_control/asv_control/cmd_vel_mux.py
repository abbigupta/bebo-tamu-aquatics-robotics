"""Priority and timeout mux for autonomous and teleoperation commands."""

from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


@dataclass
class Source:
    message: TwistStamped | None = None
    received_ns: int | None = None


class CmdVelMux(Node):
    """Prefer fresh teleoperation commands over fresh autonomy commands."""

    def __init__(self) -> None:
        super().__init__('cmd_vel_mux')
        self.declare_parameter('teleop_timeout_s', 0.35)
        self.declare_parameter('autonomy_timeout_s', 0.60)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.teleop_timeout_ns = int(
            float(self.get_parameter('teleop_timeout_s').value) * 1e9
        )
        self.autonomy_timeout_ns = int(
            float(self.get_parameter('autonomy_timeout_s').value) * 1e9
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.teleop = Source()
        self.autonomy = Source()
        self.publisher = self.create_publisher(TwistStamped, '/asv/cmd_vel', 10)
        self.create_subscription(
            TwistStamped, '/asv/cmd_vel/teleop', self._teleop, 10
        )
        self.create_subscription(
            TwistStamped, '/asv/cmd_vel/autonomy', self._autonomy, 10
        )
        self.create_timer(1.0 / rate, self._publish)

    def _teleop(self, message: TwistStamped) -> None:
        self.teleop = Source(message, self.get_clock().now().nanoseconds)
        self._publish()

    def _autonomy(self, message: TwistStamped) -> None:
        self.autonomy = Source(message, self.get_clock().now().nanoseconds)
        self._publish()

    @staticmethod
    def _active(source: Source, timeout_ns: int, now_ns: int) -> bool:
        return (
            source.message is not None
            and source.received_ns is not None
            and now_ns - source.received_ns <= timeout_ns
        )

    def _publish(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if self._active(self.teleop, self.teleop_timeout_ns, now_ns):
            output = self.teleop.message
        elif self._active(self.autonomy, self.autonomy_timeout_ns, now_ns):
            output = self.autonomy.message
        else:
            output = TwistStamped()
            output.header.frame_id = 'base_link'
        output.header.stamp = now.to_msg()
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""Safe ROS interface for independently commanding two Gazebo thrusters."""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger


@dataclass
class ThrusterCommand:
    """Latest requested command for one thruster."""

    thrust_newtons: float = 0.0
    received_at_ns: int | None = None


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a numeric value to an inclusive interval."""
    return max(lower, min(value, upper))


class TwinThrusterController(Node):
    """Clamp, time out, and forward port/starboard thrust commands."""

    def __init__(self) -> None:
        super().__init__('twin_thruster_controller')

        self.declare_parameter('max_forward_thrust_n', 25.0)
        self.declare_parameter('max_reverse_thrust_n', 15.0)
        self.declare_parameter('command_timeout_s', 0.75)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('gazebo_thrust_sign', -1.0)

        self.max_forward = float(
            self.get_parameter('max_forward_thrust_n').value
        )
        self.max_reverse = float(
            self.get_parameter('max_reverse_thrust_n').value
        )
        self.timeout_ns = int(
            float(self.get_parameter('command_timeout_s').value) * 1e9
        )
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.gazebo_thrust_sign = float(
            self.get_parameter('gazebo_thrust_sign').value
        )

        self.port = ThrusterCommand()
        self.starboard = ThrusterCommand()
        self.estop_latched = False

        self.port_gazebo_pub = self.create_publisher(
            Float64,
            '/model/asv/joint/port_thruster_joint/cmd_thrust',
            10,
        )
        self.starboard_gazebo_pub = self.create_publisher(
            Float64,
            '/model/asv/joint/starboard_thruster_joint/cmd_thrust',
            10,
        )
        self.port_applied_pub = self.create_publisher(
            Float64, '/asv/thrusters/port/applied', 10
        )
        self.starboard_applied_pub = self.create_publisher(
            Float64, '/asv/thrusters/starboard/applied', 10
        )

        self.create_subscription(
            Float64,
            '/asv/thrusters/port/command',
            self._on_port_command,
            10,
        )
        self.create_subscription(
            Float64,
            '/asv/thrusters/starboard/command',
            self._on_starboard_command,
            10,
        )
        self.create_service(Trigger, '/asv/thrusters/stop', self._stop)
        self.create_service(SetBool, '/asv/control/estop', self._estop)
        self.create_timer(1.0 / publish_rate, self._publish_commands)

        self.get_logger().info(
            'Twin-thruster controller ready: '
            f'forward limit={self.max_forward:.1f} N, '
            f'reverse limit={self.max_reverse:.1f} N, '
            f'timeout={self.timeout_ns / 1e9:.2f} s'
        )

    def _bounded(self, requested: float) -> float:
        return clamp(requested, -self.max_reverse, self.max_forward)

    def _on_port_command(self, message: Float64) -> None:
        if self.estop_latched:
            return
        self.port.thrust_newtons = self._bounded(message.data)
        self.port.received_at_ns = self.get_clock().now().nanoseconds

    def _on_starboard_command(self, message: Float64) -> None:
        if self.estop_latched:
            return
        self.starboard.thrust_newtons = self._bounded(message.data)
        self.starboard.received_at_ns = self.get_clock().now().nanoseconds

    def _active_value(self, command: ThrusterCommand, now_ns: int) -> float:
        if command.received_at_ns is None:
            return 0.0
        if now_ns - command.received_at_ns > self.timeout_ns:
            command.thrust_newtons = 0.0
            command.received_at_ns = None
            return 0.0
        return command.thrust_newtons

    def _publish(self, publisher, feedback_publisher, value: float) -> None:
        gazebo_message = Float64()
        gazebo_message.data = self.gazebo_thrust_sign * value
        feedback_message = Float64()
        feedback_message.data = value
        publisher.publish(gazebo_message)
        feedback_publisher.publish(feedback_message)

    def _publish_commands(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        port = 0.0 if self.estop_latched else self._active_value(self.port, now_ns)
        starboard = (
            0.0 if self.estop_latched
            else self._active_value(self.starboard, now_ns)
        )
        self._publish(self.port_gazebo_pub, self.port_applied_pub, port)
        self._publish(
            self.starboard_gazebo_pub,
            self.starboard_applied_pub,
            starboard,
        )

    def _stop(self, _request, response):
        self.port = ThrusterCommand()
        self.starboard = ThrusterCommand()
        self._publish_commands()
        response.success = True
        response.message = 'Both thruster commands set to zero.'
        return response

    def _estop(self, request, response):
        self.estop_latched = bool(request.data)
        self.port = ThrusterCommand()
        self.starboard = ThrusterCommand()
        self._publish_commands()
        response.success = True
        response.message = (
            'Emergency stop latched.' if self.estop_latched
            else 'Emergency stop cleared; thrusters remain at zero.'
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwinThrusterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

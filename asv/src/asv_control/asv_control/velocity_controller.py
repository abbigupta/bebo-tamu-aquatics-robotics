"""Closed-loop surge-speed and yaw-rate controller for twin thrusters."""

from __future__ import annotations

import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

from .control_math import allocate_thrust, Pid, scale_pair_to_limits


class VelocityController(Node):
    """Control surge speed and yaw rate through differential thrust."""

    def __init__(self) -> None:
        super().__init__('velocity_controller')
        parameters = {
            'surge_kp': 18.0, 'surge_ki': 2.0, 'surge_kd': 0.0,
            'yaw_kp': 12.0, 'yaw_ki': 0.2, 'yaw_kd': 4.0,
            'max_yaw_moment_nm': 12.0,
            'yaw_moment_sign': 1.0,
            'thruster_offset_y_m': 0.29,
            'max_forward_thrust_n': 25.0,
            'max_reverse_thrust_n': 15.0,
            'command_timeout_s': 0.75,
            'odometry_timeout_s': 0.75,
            'control_rate_hz': 50.0,
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)

        def value(name: str) -> float:
            return float(self.get_parameter(name).value)

        self.surge_pid = Pid(
            value('surge_kp'), value('surge_ki'), value('surge_kd'), 5.0, 40.0
        )
        self.yaw_pid = Pid(
            value('yaw_kp'), value('yaw_ki'), value('yaw_kd'), 1.0,
            value('max_yaw_moment_nm')
        )
        self.offset = value('thruster_offset_y_m')
        self.yaw_moment_sign = value('yaw_moment_sign')
        self.forward_limit = value('max_forward_thrust_n')
        self.reverse_limit = value('max_reverse_thrust_n')
        self.command_timeout_ns = int(value('command_timeout_s') * 1e9)
        self.odom_timeout_ns = int(value('odometry_timeout_s') * 1e9)
        rate = value('control_rate_hz')

        self.command: TwistStamped | None = None
        self.command_received_ns: int | None = None
        self.odometry: Odometry | None = None
        self.odom_received_ns: int | None = None
        self.last_control_ns: int | None = None
        self.port_pub = self.create_publisher(
            Float64, '/asv/thrusters/port/command', 10
        )
        self.starboard_pub = self.create_publisher(
            Float64, '/asv/thrusters/starboard/command', 10
        )
        self.create_subscription(TwistStamped, '/asv/cmd_vel', self._cmd, 10)
        self.create_subscription(
            Odometry, '/asv/odometry/filtered', self._odom, 10
        )
        self.create_timer(1.0 / rate, self._control)

    def _cmd(self, message: TwistStamped) -> None:
        self.command = message
        self.command_received_ns = self.get_clock().now().nanoseconds

    def _odom(self, message: Odometry) -> None:
        self.odometry = message
        self.odom_received_ns = self.get_clock().now().nanoseconds

    def _publish(self, port_value: float, starboard_value: float) -> None:
        port = Float64(data=port_value)
        starboard = Float64(data=starboard_value)
        self.port_pub.publish(port)
        self.starboard_pub.publish(starboard)

    def _stop(self) -> None:
        self.surge_pid.reset()
        self.yaw_pid.reset()
        self._publish(0.0, 0.0)

    def _control(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if (
            self.command is None or self.odometry is None
            or self.command_received_ns is None or self.odom_received_ns is None
            or now_ns - self.command_received_ns > self.command_timeout_ns
            or now_ns - self.odom_received_ns > self.odom_timeout_ns
        ):
            self._stop()
            self.last_control_ns = now_ns
            return

        dt = 0.05 if self.last_control_ns is None else (
            now_ns - self.last_control_ns
        ) / 1e9
        self.last_control_ns = now_ns
        desired_u = self.command.twist.linear.x
        desired_r = self.command.twist.angular.z
        measured_u = self.odometry.twist.twist.linear.x
        measured_r = self.odometry.twist.twist.angular.z
        if not all(math.isfinite(v) for v in (
            desired_u, desired_r, measured_u, measured_r
        )):
            self._stop()
            return

        force = self.surge_pid.update(desired_u - measured_u, dt)
        moment = self.yaw_pid.update(desired_r - measured_r, dt)
        port, starboard = allocate_thrust(
            force, self.yaw_moment_sign * moment, self.offset
        )
        port, starboard = scale_pair_to_limits(
            port, starboard, self.reverse_limit, self.forward_limit
        )
        self._publish(port, starboard)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            try:
                node._stop()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

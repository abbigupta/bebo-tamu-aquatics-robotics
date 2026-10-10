"""Keyboard teleoperation for the ASV's twin thrusters."""

from __future__ import annotations

import select
import sys
import termios
import tty

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node


HELP = """
Twin-thruster keyboard control

  W / Up Arrow       forward
  S / Down Arrow     reverse
  A / Left Arrow     pivot left
  D / Right Arrow    pivot right
  Space or X         stop
  Q                   stop and quit
"""


def command_for_key(
    key: str,
    forward_speed: float = 0.8,
    reverse_speed: float = 0.5,
    turn_rate: float = 0.8,
) -> tuple[float, float] | None:
    """Return linear speed and yaw rate for a supported key."""
    commands = {
        'w': (forward_speed, 0.0),
        '\x1b[A': (forward_speed, 0.0),
        's': (-reverse_speed, 0.0),
        '\x1b[B': (-reverse_speed, 0.0),
        'a': (0.0, turn_rate),
        '\x1b[D': (0.0, turn_rate),
        'd': (0.0, -turn_rate),
        '\x1b[C': (0.0, -turn_rate),
        ' ': (0.0, 0.0),
        'x': (0.0, 0.0),
    }
    return commands.get(key.lower() if len(key) == 1 else key)


def read_key() -> str:
    """Read one key, including a complete terminal arrow-key sequence."""
    if not select.select([sys.stdin], [], [], 0.0)[0]:
        return ''

    key = sys.stdin.read(1)
    if key != '\x1b':
        return key

    # Arrow keys arrive as ESC, [, and a direction letter.
    sequence = key
    for _ in range(2):
        if not select.select([sys.stdin], [], [], 0.02)[0]:
            break
        sequence += sys.stdin.read(1)
    return sequence


class KeyboardTeleop(Node):
    """Publish a latched keyboard command at a watchdog-safe rate."""

    def __init__(self) -> None:
        super().__init__('keyboard_teleop')
        self.declare_parameter('forward_speed_mps', 0.8)
        self.declare_parameter('reverse_speed_mps', 0.5)
        self.declare_parameter('turn_rate_rad_s', 0.8)
        self.declare_parameter('publish_rate_hz', 30.0)

        self.forward_speed = float(
            self.get_parameter('forward_speed_mps').value
        )
        self.reverse_speed = float(
            self.get_parameter('reverse_speed_mps').value
        )
        self.turn_rate = float(
            self.get_parameter('turn_rate_rad_s').value
        )
        publish_rate = float(self.get_parameter('publish_rate_hz').value)

        self.linear_speed = 0.0
        self.yaw_rate = 0.0
        self.port_pub = self.create_publisher(
            TwistStamped, '/asv/cmd_vel/teleop', 10
        )
        self.create_timer(1.0 / publish_rate, self.publish_command)

    def set_from_key(self, key: str) -> bool:
        """Apply a key; return True when it requests program exit."""
        if key.lower() == 'q' or key == '\x03':
            self.stop()
            return True

        command = command_for_key(
            key,
            self.forward_speed,
            self.reverse_speed,
            self.turn_rate,
        )
        if command is not None:
            self.linear_speed, self.yaw_rate = command
            self.publish_command()
        return False

    def publish_command(self) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.twist.linear.x = self.linear_speed
        message.twist.angular.z = self.yaw_rate
        self.port_pub.publish(message)

    def stop(self) -> None:
        self.linear_speed = 0.0
        self.yaw_rate = 0.0
        # Publish more than once so a final stop is unlikely to be dropped.
        self.publish_command()
        self.publish_command()


def main(args=None) -> None:
    if not sys.stdin.isatty():
        raise RuntimeError('keyboard_teleop must run in an interactive terminal')

    rclpy.init(args=args)
    node = KeyboardTeleop()
    terminal_settings = termios.tcgetattr(sys.stdin)
    print(HELP, flush=True)

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = read_key()
            if key and node.set_from_key(key):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

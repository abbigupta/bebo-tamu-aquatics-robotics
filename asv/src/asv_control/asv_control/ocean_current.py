"""Publish a bounded, slowly varying ocean current for Gazebo Sim."""

from __future__ import annotations

import math
import random

from geometry_msgs.msg import Vector3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .control_math import clamp


def gauss_markov_step(
    value: float,
    mean: float,
    lower: float,
    upper: float,
    reversion_rate: float,
    noise_stddev: float,
    dt: float,
    normal_sample: float,
) -> float:
    """Advance a bounded Ornstein-Uhlenbeck / Gauss-Markov process."""
    if dt <= 0.0:
        return clamp(value, lower, upper)
    next_value = value + reversion_rate * (mean - value) * dt
    next_value += noise_stddev * math.sqrt(dt) * normal_sample
    return clamp(next_value, lower, upper)


def current_vector(speed: float, direction_rad: float, vertical: float) -> Vector3:
    """Convert speed and ENU heading into a Gazebo world-frame vector."""
    return Vector3(
        x=speed * math.cos(direction_rad),
        y=speed * math.sin(direction_rad),
        z=vertical,
    )


class OceanCurrent(Node):
    """DAVE-style first-order stochastic current with deterministic bounds."""

    def __init__(self) -> None:
        super().__init__('ocean_current')
        defaults = {
            'mean_speed_mps': 0.25,
            'min_speed_mps': 0.0,
            'max_speed_mps': 0.60,
            'speed_noise_stddev': 0.015,
            'speed_reversion_rate': 0.20,
            'mean_direction_deg': 30.0,
            'direction_noise_stddev_deg': 0.8,
            'direction_reversion_rate': 0.15,
            'vertical_speed_mps': 0.0,
            'publish_rate_hz': 10.0,
            'random_seed': 42,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def number(name: str) -> float:
            return float(self.get_parameter(name).value)

        self.mean_speed = number('mean_speed_mps')
        self.min_speed = number('min_speed_mps')
        self.max_speed = number('max_speed_mps')
        self.speed_noise = number('speed_noise_stddev')
        self.speed_reversion = number('speed_reversion_rate')
        self.mean_direction = math.radians(number('mean_direction_deg'))
        self.direction_noise = math.radians(
            number('direction_noise_stddev_deg')
        )
        self.direction_reversion = number('direction_reversion_rate')
        self.vertical_speed = number('vertical_speed_mps')
        rate = number('publish_rate_hz')
        seed = int(self.get_parameter('random_seed').value)

        if self.min_speed < 0.0 or self.max_speed < self.min_speed:
            raise ValueError('ocean-current speed bounds are invalid')
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')

        self.speed = clamp(
            self.mean_speed, self.min_speed, self.max_speed
        )
        self.direction = self.mean_direction
        self.dt = 1.0 / rate
        self.random = random.Random(seed)
        self.publisher = self.create_publisher(
            Vector3, '/model/asv/ocean_current', 10
        )
        self.create_timer(self.dt, self._publish)
        self.get_logger().info(
            'Ocean current active: '
            f'{self.mean_speed:.2f} m/s at '
            f'{math.degrees(self.mean_direction):.1f} deg ENU'
        )

    def _publish(self) -> None:
        self.speed = gauss_markov_step(
            self.speed,
            self.mean_speed,
            self.min_speed,
            self.max_speed,
            self.speed_reversion,
            self.speed_noise,
            self.dt,
            self.random.gauss(0.0, 1.0),
        )
        # Direction is deliberately unbounded around its mean; sin/cos wrap it.
        self.direction = gauss_markov_step(
            self.direction,
            self.mean_direction,
            self.mean_direction - math.pi,
            self.mean_direction + math.pi,
            self.direction_reversion,
            self.direction_noise,
            self.dt,
            self.random.gauss(0.0, 1.0),
        )
        self.publisher.publish(
            current_vector(self.speed, self.direction, self.vertical_speed)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OceanCurrent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

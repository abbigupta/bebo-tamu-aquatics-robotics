"""Pure control helpers shared by the ASV controller and tests."""

from __future__ import annotations

from dataclasses import dataclass
import math


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def allocate_thrust(
    total_force_n: float,
    yaw_moment_nm: float,
    thruster_offset_y_m: float,
) -> tuple[float, float]:
    """Map surge force and positive-CCW yaw moment to port/starboard thrust."""
    if thruster_offset_y_m <= 0.0:
        raise ValueError('thruster_offset_y_m must be positive')
    differential = yaw_moment_nm / (2.0 * thruster_offset_y_m)
    return total_force_n / 2.0 - differential, total_force_n / 2.0 + differential


def scale_pair_to_limits(
    port: float,
    starboard: float,
    reverse_limit: float,
    forward_limit: float,
) -> tuple[float, float]:
    """Scale a pair together, preserving its force / moment ratio."""
    scale = 1.0
    for value in (port, starboard):
        if value > forward_limit:
            scale = min(scale, forward_limit / value)
        elif value < -reverse_limit:
            scale = min(scale, -reverse_limit / value)
    return port * scale, starboard * scale


@dataclass
class Pid:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float
    integral: float = 0.0
    previous_error: float | None = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None

    def update(self, error: float, dt: float) -> float:
        if not math.isfinite(error) or dt <= 0.0:
            return 0.0
        self.integral = clamp(
            self.integral + error * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        derivative = 0.0
        if self.previous_error is not None:
            derivative = (error - self.previous_error) / dt
        self.previous_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(output, -self.output_limit, self.output_limit)

"""Pure waypoint-guidance functions."""

from __future__ import annotations

import math


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def guidance_command(
    dx: float,
    dy: float,
    yaw: float,
    cruise_speed: float,
    heading_gain: float,
    max_yaw_rate: float,
) -> tuple[float, float, float, float]:
    """Return speed, yaw-rate, distance, and heading error to a target."""
    distance = math.hypot(dx, dy)
    desired_heading = math.atan2(dy, dx)
    error = normalize_angle(desired_heading - yaw)
    yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, heading_gain * error))
    # Surface vessels have substantial yaw inertia. Turn in place when the
    # bow is more than 20 degrees off course, then approach slowly enough not
    # to orbit a nearby waypoint.
    heading_window = math.radians(45.0)
    alignment = max(0.0, 1.0 - abs(error) / heading_window)
    approach_speed = min(cruise_speed, 0.60 * distance)
    speed = approach_speed * alignment
    return speed, yaw_rate, distance, error

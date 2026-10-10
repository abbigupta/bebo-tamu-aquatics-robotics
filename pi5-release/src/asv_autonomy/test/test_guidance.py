import math

from asv_autonomy.guidance import guidance_command, normalize_angle


def test_angle_wrap():
    assert math.isclose(normalize_angle(3.0 * math.pi), math.pi, abs_tol=1e-9)


def test_guidance_forward():
    speed, yaw_rate, distance, error = guidance_command(5, 0, 0, 1, 2, 1)
    assert speed == 1
    assert yaw_rate == 0
    assert distance == 5
    assert error == 0


def test_guidance_turns_before_driving_behind():
    speed, yaw_rate, _, _ = guidance_command(-5, 0, 0, 1, 2, 1)
    assert speed == 0
    assert abs(yaw_rate) == 1


def test_guidance_slows_near_waypoint():
    speed, _, _, _ = guidance_command(1, 0, 0, 1, 2, 1)
    assert math.isclose(speed, 0.60)

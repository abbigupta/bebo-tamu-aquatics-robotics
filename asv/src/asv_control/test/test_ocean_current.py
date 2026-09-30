"""Tests for the ocean-current process."""

import math

from asv_control.ocean_current import current_vector, gauss_markov_step


def test_current_vector_uses_enu_heading():
    east = current_vector(0.4, 0.0, 0.0)
    north = current_vector(0.4, math.pi / 2.0, 0.0)
    assert math.isclose(east.x, 0.4)
    assert math.isclose(east.y, 0.0, abs_tol=1e-12)
    assert math.isclose(north.x, 0.0, abs_tol=1e-12)
    assert math.isclose(north.y, 0.4)


def test_gauss_markov_process_reverts_toward_mean():
    value = gauss_markov_step(0.5, 0.2, 0.0, 1.0, 0.5, 0.0, 0.1, 0.0)
    assert value < 0.5
    assert value > 0.2


def test_gauss_markov_process_is_bounded():
    value = gauss_markov_step(0.9, 0.5, 0.0, 1.0, 0.0, 10.0, 1.0, 1.0)
    assert value == 1.0

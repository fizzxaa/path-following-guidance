import math
import numpy as np
import pytest

from ctrack.dubins import dubins_shortest, dubins_all, wrap_pi


def _random_pose(rng, span=100.0):
    return (rng.uniform(-span, span), rng.uniform(-span, span), rng.uniform(-math.pi, math.pi))


def test_endpoint_matches_goal_pose():
    """Sampling the path must land on the goal position and heading."""
    rng = np.random.default_rng(0)
    for _ in range(500):
        q0, q1 = _random_pose(rng), _random_pose(rng)
        if math.hypot(q1[0] - q0[0], q1[1] - q0[1]) < 1.0:
            continue
        rho = rng.uniform(5, 30)
        path = dubins_shortest(q0, q1, rho)
        x, y, th, _ = path.sample(0.25)
        assert math.hypot(x[-1] - q1[0], y[-1] - q1[1]) < 1e-3
        assert abs(wrap_pi(th[-1] - q1[2])) < 1e-6
        assert abs(x[0] - q0[0]) < 1e-9 and abs(y[0] - q0[1]) < 1e-9


def test_length_at_least_straight_line():
    rng = np.random.default_rng(1)
    for _ in range(300):
        q0, q1 = _random_pose(rng), _random_pose(rng)
        D = math.hypot(q1[0] - q0[0], q1[1] - q0[1])
        if D < 1.0:
            continue
        assert dubins_shortest(q0, q1, 10.0).length >= D - 1e-9


def test_chosen_word_is_shortest_of_all():
    rng = np.random.default_rng(2)
    for _ in range(200):
        q0, q1 = _random_pose(rng), _random_pose(rng)
        if math.hypot(q1[0] - q0[0], q1[1] - q0[1]) < 1.0:
            continue
        best = dubins_shortest(q0, q1, 12.0)
        assert best.length <= min(p.length for p in dubins_all(q0, q1, 12.0)) + 1e-9


def test_sampled_curvature_never_exceeds_limit():
    rng = np.random.default_rng(3)
    rho = 15.0
    for _ in range(100):
        q0, q1 = _random_pose(rng), _random_pose(rng)
        if math.hypot(q1[0] - q0[0], q1[1] - q0[1]) < 1.0:
            continue
        _, _, _, k = dubins_shortest(q0, q1, rho).sample(0.5)
        assert np.max(np.abs(k)) <= 1.0 / rho + 1e-12


def test_known_straight_line_case():
    path = dubins_shortest((0, 0, 0.0), (100, 0, 0.0), 10.0)
    assert path.length == pytest.approx(100.0, abs=1e-6)


def test_known_u_turn_case():
    """Two poses one diameter apart, facing opposite ways: a half circle, length pi*rho."""
    rho = 10.0
    path = dubins_shortest((0, 0, 0.0), (0, 2 * rho, math.pi), rho)
    assert path.length == pytest.approx(math.pi * rho, rel=1e-6)


def test_heading_is_tangent_to_path():
    path = dubins_shortest((0, 0, 0.3), (80, 40, -1.0), 12.0)
    x, y, th, _ = path.sample(0.1)
    ang = np.arctan2(np.diff(y), np.diff(x))
    assert np.max(np.abs(wrap_pi(ang - th[:-1]))) < 0.05

import numpy as np
import pytest

from ctrack.route_kinds import build_path, curvature_jump, max_curvature_ratio
from ctrack.scenarios import TEST_ROUTE_SEEDS
from ctrack.route import random_route
from ctrack.vehicles import QUADCOPTER as Q


def wps(seed=2000):
    return random_route(Q.r_min, seed)


def test_dubins_has_a_curvature_jump_and_smooth_does_not():
    d = build_path("dubins", wps(), 1.5 * Q.r_min)
    m = build_path("dubins_smooth", wps(), 1.5 * Q.r_min)
    assert curvature_jump(d) > 0.9
    assert curvature_jump(m) < 0.1


@pytest.mark.parametrize("seed", TEST_ROUTE_SEEDS)
def test_smooth_stays_close_and_never_tighter_than_the_limit(seed):
    rho = 1.5 * Q.r_min
    d = build_path("dubins", wps(seed), rho)
    m = build_path("dubins_smooth", wps(seed), rho)
    assert max_curvature_ratio(m) <= 1.02            # never tighter than the plan
    assert -0.025 < (m.length - d.length) / d.length <= 0.0     # smoothing cuts inside arcs: up to ~2% shorter
    # endpoints preserved
    assert np.hypot(m.x[0] - d.x[0], m.y[0] - d.y[0]) < 0.5
    assert np.hypot(m.x[-1] - d.x[-1], m.y[-1] - d.y[-1]) < 0.5
    # every smoothed point is within a fraction of a radius of the Dubins route
    dd = np.sqrt(((m.x[::10, None] - d.x[None, :]) ** 2 + (m.y[::10, None] - d.y[None, :]) ** 2).min(axis=1))
    assert dd.max() < 0.35 * rho


def test_smooth_of_straight_line_is_the_straight_line():
    m = build_path("dubins_smooth", [[0, 0], [100, 0]], 20.0)
    assert np.abs(m.y).max() < 1e-6 and np.abs(m.kappa).max() < 1e-6

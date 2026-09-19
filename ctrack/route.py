"""Turn a list of waypoints into one smooth, curvature-limited reference path."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .dubins import dubins_shortest


@dataclass
class Path:
    x: np.ndarray
    y: np.ndarray
    psi: np.ndarray     # path heading at each sample (rad)
    kappa: np.ndarray   # signed curvature at each sample (1/m), left = +
    s: np.ndarray       # arc length at each sample (m)
    rho: float          # turning radius the path was planned with

    @property
    def n(self) -> int:
        return len(self.x)

    @property
    def length(self) -> float:
        return float(self.s[-1])


def default_headings(waypoints):
    """Heading at each waypoint = direction of the chord between its neighbours."""
    w = np.asarray(waypoints, dtype=float)
    n = len(w)
    heads = []
    for i in range(n):
        a = w[max(i - 1, 0)]
        b = w[min(i + 1, n - 1)]
        heads.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    return heads


def build_route(waypoints, rho: float, ds: float = 0.5, headings=None) -> Path:
    """Chain Dubins paths through the waypoints.

    Every joint has a matching heading, so the route is heading-continuous.
    Curvature never exceeds 1/rho.
    """
    w = np.asarray(waypoints, dtype=float)
    if len(w) < 2:
        raise ValueError("need at least two waypoints")
    heads = list(headings) if headings is not None else default_headings(w)
    xs, ys, ps, ks = [], [], [], []
    for i in range(len(w) - 1):
        q0 = (w[i][0], w[i][1], heads[i])
        q1 = (w[i + 1][0], w[i + 1][1], heads[i + 1])
        x, y, th, k = dubins_shortest(q0, q1, rho).sample(ds)
        if i > 0:
            x, y, th, k = x[1:], y[1:], th[1:], k[1:]
        xs.append(x); ys.append(y); ps.append(th); ks.append(k)
    x = np.concatenate(xs)
    y = np.concatenate(ys)
    psi = np.unwrap(np.concatenate(ps))
    kappa = np.concatenate(ks)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    return Path(x, y, psi, kappa, s, rho)


# A test route, in units of the vehicle's nominal minimum turning radius.
# Chosen so consecutive waypoints are ~6-8 radii apart: a mix of long
# straights, gentle bends and a few tight turns.
ROUTE_UNITS = [(0, 0), (8, 0), (14, 6), (10, 13), (2, 12), (-3, 5), (2, -2), (10, -4)]


def standard_route(r_nom: float):
    """The benchmark route scaled to a vehicle's nominal minimum turn radius."""
    return [(x * r_nom, y * r_nom) for x, y in ROUTE_UNITS]


def random_route(r_nom: float, seed: int, n_waypoints: int = 7, spacing=(6.0, 9.0),
                 max_turn_deg: float = 100.0, min_gap: float = 5.0, max_detour: float = 1.6):
    """A random route, in metres, for tuning and testing on many different routes.

    Built in units of the turning radius and then scaled, so the same seed gives
    the same shape for every vehicle. Each step turns by up to max_turn_deg and
    goes 6-9 radii. Routes are rejected if they come back too close to an earlier
    waypoint, or if the Dubins path at 2x the radius is much longer than the
    straight-line legs (a sign of big loops).
    """
    rng = np.random.default_rng(seed)
    for _ in range(500):
        pts = [np.zeros(2)]
        heading = rng.uniform(-math.pi, math.pi)
        ok = True
        for _ in range(n_waypoints - 1):
            heading += math.radians(rng.uniform(-max_turn_deg, max_turn_deg))
            step = rng.uniform(*spacing)
            p = pts[-1] + step * np.array([math.cos(heading), math.sin(heading)])
            if any(np.linalg.norm(p - q) < min_gap for q in pts[:-1]):
                ok = False
                break
            pts.append(p)
        if not ok:
            continue
        chords = sum(np.linalg.norm(pts[i + 1] - pts[i]) for i in range(len(pts) - 1))
        coarse = build_route(pts, rho=2.0, ds=1.0)
        if coarse.length / chords > max_detour:
            continue
        return [(float(q[0] * r_nom), float(q[1] * r_nom)) for q in pts]
    raise RuntimeError(f"could not build a route for seed {seed}")

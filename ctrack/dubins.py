"""Dubins shortest-path planner (all six words: LSL, RSR, LSR, RSL, RLR, LRL).

A Dubins path is the shortest path between two poses (x, y, heading) for a
vehicle that moves forward at constant speed and cannot turn tighter than a
minimum radius `rho`. It is made of at most three pieces: arcs (L = left turn,
R = right turn) and straight lines (S).

Angles are in radians, measured counter-clockwise from the +x axis (east).
The closed-form solutions follow Shkel & Lumelsky (2001), in the same
normalised form used by the widely used `dubins-curves` C library.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

TWO_PI = 2.0 * math.pi


def mod2pi(a: float) -> float:
    """Wrap an angle into [0, 2*pi)."""
    return a - TWO_PI * math.floor(a / TWO_PI)


def wrap_pi(a):
    """Wrap an angle (scalar or array) into [-pi, pi)."""
    return (a + np.pi) % TWO_PI - np.pi


# Each solver returns (t, p, q) = normalised segment lengths, or None.
# Arcs are in radians of turning; the straight piece is in units of rho.
def _lsl(alpha, beta, d):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb)
    if p_sq < 0:
        return None
    tmp1 = math.atan2(cb - ca, d + sa - sb)
    return mod2pi(tmp1 - alpha), math.sqrt(p_sq), mod2pi(beta - tmp1)


def _rsr(alpha, beta, d):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa)
    if p_sq < 0:
        return None
    tmp1 = math.atan2(ca - cb, d - sa + sb)
    return mod2pi(alpha - tmp1), math.sqrt(p_sq), mod2pi(-beta + tmp1)


def _lsr(alpha, beta, d):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    p_sq = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp0 = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    return mod2pi(tmp0 - alpha), p, mod2pi(tmp0 - mod2pi(beta))


def _rsl(alpha, beta, d):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    p_sq = -2 + d * d + 2 * c_ab - 2 * d * (sa + sb)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp0 = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    return mod2pi(alpha - tmp0), p, mod2pi(beta - tmp0)


def _rlr(alpha, beta, d):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    tmp0 = (6.0 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp0) > 1:
        return None
    phi = math.atan2(ca - cb, d - sa + sb)
    p = mod2pi(TWO_PI - math.acos(tmp0))
    t = mod2pi(alpha - phi + mod2pi(p / 2.0))
    q = mod2pi(alpha - beta - t + mod2pi(p))
    return t, p, q


def _lrl(alpha, beta, d):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    tmp0 = (6.0 - d * d + 2 * c_ab + 2 * d * (sb - sa)) / 8.0
    if abs(tmp0) > 1:
        return None
    phi = math.atan2(ca - cb, d + sa - sb)
    p = mod2pi(TWO_PI - math.acos(tmp0))
    t = mod2pi(-alpha - phi + p / 2.0)
    q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p))
    return t, p, q


_WORDS = {
    "LSL": _lsl, "RSR": _rsr, "LSR": _lsr,
    "RSL": _rsl, "RLR": _rlr, "LRL": _lrl,
}


@dataclass
class DubinsPath:
    q0: tuple          # start pose (x, y, heading)
    rho: float         # turning radius used
    word: str          # e.g. "LSL"
    lengths: tuple     # normalised (t, p, q)

    @property
    def length(self) -> float:
        return sum(self.lengths) * self.rho

    def sample(self, ds: float):
        """Return arrays x, y, heading, curvature sampled every ~ds metres.

        curvature is +1/rho on left arcs, -1/rho on right arcs, 0 on straights.
        """
        x0, y0, th0 = self.q0
        cx = cy = 0.0
        cth = th0
        xs, ys, ths, ks = [], [], [], []
        total = self.length
        n_total = max(2, int(math.ceil(total / ds)) + 1)
        for k, (kind, L) in enumerate(zip(self.word, self.lengths)):
            n = max(2, int(round(n_total * (L * self.rho) / max(total, 1e-9))) + 1)
            s = np.linspace(0.0, L, n)
            if kind == "L":
                x = cx + np.sin(cth + s) - math.sin(cth)
                y = cy - np.cos(cth + s) + math.cos(cth)
                th = cth + s
                kap = np.full_like(s, 1.0 / self.rho)
                ex, ey, eth = x[-1], y[-1], th[-1]
            elif kind == "R":
                x = cx - np.sin(cth - s) + math.sin(cth)
                y = cy + np.cos(cth - s) - math.cos(cth)
                th = cth - s
                kap = np.full_like(s, -1.0 / self.rho)
                ex, ey, eth = x[-1], y[-1], th[-1]
            else:  # "S"
                x = cx + s * math.cos(cth)
                y = cy + s * math.sin(cth)
                th = np.full_like(s, cth)
                kap = np.zeros_like(s)
                ex, ey, eth = x[-1], y[-1], cth
            if k > 0:  # drop the duplicated joint sample
                x, y, th, kap = x[1:], y[1:], th[1:], kap[1:]
            xs.append(x); ys.append(y); ths.append(th); ks.append(kap)
            cx, cy, cth = ex, ey, eth
        x = np.concatenate(xs) * self.rho + x0
        y = np.concatenate(ys) * self.rho + y0
        th = np.concatenate(ths)
        kap = np.concatenate(ks)
        return x, y, th, kap


def dubins_shortest(q0, q1, rho: float) -> DubinsPath:
    """Shortest Dubins path from pose q0 to pose q1 with turning radius rho."""
    if rho <= 0:
        raise ValueError("rho must be positive")
    dx, dy = q1[0] - q0[0], q1[1] - q0[1]
    d = math.hypot(dx, dy) / rho
    theta = mod2pi(math.atan2(dy, dx)) if d > 0 else 0.0
    alpha = mod2pi(q0[2] - theta)
    beta = mod2pi(q1[2] - theta)
    best = None
    for word, fn in _WORDS.items():
        res = fn(alpha, beta, d)
        if res is None:
            continue
        total = sum(res)
        if best is None or total < best[0]:
            best = (total, word, res)
    if best is None:  # should not happen for distinct poses
        raise RuntimeError("no Dubins path found")
    return DubinsPath(tuple(q0), rho, best[1], best[2])


def dubins_all(q0, q1, rho: float):
    """Every feasible word (used in tests to check the shortest is chosen)."""
    dx, dy = q1[0] - q0[0], q1[1] - q0[1]
    d = math.hypot(dx, dy) / rho
    theta = mod2pi(math.atan2(dy, dx)) if d > 0 else 0.0
    alpha = mod2pi(q0[2] - theta)
    beta = mod2pi(q1[2] - theta)
    out = []
    for word, fn in _WORDS.items():
        res = fn(alpha, beta, d)
        if res is not None:
            out.append(DubinsPath(tuple(q0), rho, word, res))
    return out

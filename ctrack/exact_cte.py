"""Cross-track error without the path-sampling floor.

metrics.cross_track_errors returns the distance to the nearest path SAMPLE, which has a floor
of about ds/2 * (typical) when the vehicle is exactly on the path between samples. This module
projects onto the two segments next to the nearest sample instead, so a vehicle exactly on the
path reads ~0. The default metric in metrics.py is left unchanged so older results reproduce.
"""
from __future__ import annotations

import numpy as np

from .route import Path


def _seg_dist(px, py, ax, ay, bx, by):
    abx, aby = bx - ax, by - ay
    L2 = abx * abx + aby * aby
    t = np.where(L2 > 0, ((px - ax) * abx + (py - ay) * aby) / np.where(L2 > 0, L2, 1.0), 0.0)
    t = np.clip(t, 0.0, 1.0)
    return np.hypot(px - (ax + t * abx), py - (ay + t * aby))


def exact_cross_track_errors(path: Path, x, y, chunk: int = 512):
    x = np.asarray(x, float); y = np.asarray(y, float)
    out = np.empty(len(x))
    for a in range(0, len(x), chunk):
        b = min(a + chunk, len(x))
        d2 = (x[a:b, None] - path.x[None, :]) ** 2 + (y[a:b, None] - path.y[None, :]) ** 2
        i = d2.argmin(axis=1)
        lo = np.maximum(i - 1, 0); hi = np.minimum(i + 1, path.n - 1)
        px, py = x[a:b], y[a:b]
        d_prev = _seg_dist(px, py, path.x[lo], path.y[lo], path.x[i], path.y[i])
        d_next = _seg_dist(px, py, path.x[i], path.y[i], path.x[hi], path.y[hi])
        out[a:b] = np.minimum(d_prev, d_next)
    return out


def floor_of(path: Path) -> float:
    """RMS of the nearest-sample metric for a point exactly on the path (the metric's floor)."""
    from .metrics import cross_track_errors
    mids = 0.5 * (path.x[:-1] + path.x[1:]), 0.5 * (path.y[:-1] + path.y[1:])
    return float(np.sqrt(np.mean(cross_track_errors(path, *mids) ** 2)))

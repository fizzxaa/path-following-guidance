"""Split tracking error by what the path is doing at that moment.

Dubins routes are straight lines joined by constant-curvature arcs, so curvature jumps from 0 to
1/rho at each joint. Whole-path RMS error mixes behaviour in those two very different situations.
This module labels every path sample STRAIGHT / TRANSITION / ARC and reports error per label.

  ARC         : |kappa| > frac * (1/rho)          (the planner's arcs; frac=0.5)
  TRANSITION  : a straight sample within `window` turning radii (path arc length) of an ARC sample
  STRAIGHT    : everything else

A vehicle sample takes the label of its nearest path sample.
"""
from __future__ import annotations

import numpy as np

from .exact_cte import exact_cross_track_errors
from .route import Path

STRAIGHT, TRANSITION, ARC = 0, 1, 2
NAMES = {STRAIGHT: "straight", TRANSITION: "transition", ARC: "arc"}


def arc_mask(path: Path, frac: float = 0.5, min_turn_rad: float = 0.02) -> np.ndarray:
    """Samples on a real arc. The Dubins sampler reports kappa = 1/rho on zero-length arcs (a few
    samples at a joint where the arc has no extent), so a run of high-curvature samples only counts
    if the heading actually changes by at least `min_turn_rad` across it."""
    hi = np.abs(path.kappa) > frac / path.rho
    out = np.zeros(path.n, dtype=bool)
    edges = np.flatnonzero(np.diff(np.r_[False, hi, False].astype(np.int8)))
    for a, b in zip(edges[::2], edges[1::2]):          # run is samples a..b-1
        turn = abs(float(np.unwrap(path.psi[a:b])[-1] - path.psi[a])) if b - a > 1 else 0.0
        if turn >= min_turn_rad:
            out[a:b] = True
    return out


def label_path(path: Path, window_radii: float = 1.0, frac: float = 0.5) -> np.ndarray:
    lab = np.full(path.n, STRAIGHT, dtype=np.int8)
    arc = arc_mask(path, frac)
    if arc.any() and window_radii > 0:
        s_arc = path.s[arc]
        # distance along the path from every sample to the nearest arc sample
        k = np.searchsorted(s_arc, path.s)
        left = np.where(k > 0, path.s - s_arc[np.maximum(k - 1, 0)], np.inf)
        right = np.where(k < len(s_arc), s_arc[np.minimum(k, len(s_arc) - 1)] - path.s, np.inf)
        near = np.minimum(left, right)
        lab[near <= window_radii * path.rho] = TRANSITION
    lab[arc] = ARC
    return lab


def nearest_index(path: Path, x, y, chunk: int = 512):
    out = np.empty(len(x), dtype=int)
    for a in range(0, len(x), chunk):
        b = min(a + chunk, len(x))
        d2 = (x[a:b, None] - path.x[None, :]) ** 2 + (y[a:b, None] - path.y[None, :]) ** 2
        out[a:b] = d2.argmin(axis=1)
    return out


def segment_rms(path: Path, x, y, mask=None, window_radii: float = 1.0, frac: float = 0.5):
    """RMS exact cross-track error per label (m). `mask` selects samples (e.g. after warm-up).
    Returns dict label_name -> (rms, n_samples); 'all' is the whole-path value."""
    x = np.asarray(x); y = np.asarray(y)
    if mask is not None:
        x, y = x[mask], y[mask]
    cte = exact_cross_track_errors(path, x, y)
    lab = label_path(path, window_radii, frac)[nearest_index(path, x, y)]
    out = {"all": (float(np.sqrt(np.mean(cte ** 2))), len(cte))}
    for k, name in NAMES.items():
        sel = lab == k
        out[name] = (float(np.sqrt(np.mean(cte[sel] ** 2))) if sel.any() else float("nan"), int(sel.sum()))
    return out


def turning_share(path: Path, frac: float = 0.5) -> float:
    return float(np.mean(arc_mask(path, frac)))


def segment_rms_windows(path: Path, x, y, mask=None, windows=(1.0,), frac: float = 0.5):
    """As segment_rms for several transition windows, computing the error and nearest sample once.
    Returns {window: {label: (rms, n)}}."""
    x = np.asarray(x); y = np.asarray(y)
    if mask is not None:
        x, y = x[mask], y[mask]
    cte = exact_cross_track_errors(path, x, y)
    ni = nearest_index(path, x, y)
    out = {}
    for w in windows:
        lab = label_path(path, w, frac)[ni]
        d = {"all": (float(np.sqrt(np.mean(cte ** 2))), len(cte))}
        for k, name in NAMES.items():
            sel = lab == k
            d[name] = (float(np.sqrt(np.mean(cte[sel] ** 2))) if sel.any() else float("nan"), int(sel.sum()))
        out[w] = d
    return out

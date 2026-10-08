"""Reference-path variants, to separate the effect of the PLANNER from the effect of the CONTROLLER.

  dubins        : straight lines and constant-curvature arcs; curvature jumps 0 -> 1/rho at joints.
  dubins_smooth : the same route smoothed by a Gaussian along arc length (sigma = sigma_radii * rho).
                  Curvature is continuous, the peak curvature drops below 1/rho, the path stays close
                  to the Dubins route. NOT a clothoid; it is a smoothed Dubins route.
"""
from __future__ import annotations

import numpy as np
from scipy.ndimage import gaussian_filter1d

from .route import Path, build_route

ROUTE_KINDS = ("dubins", "dubins_smooth")


def smooth_path(path: Path, sigma_radii: float = 0.5, ds: float | None = None) -> Path:
    s_u, keep = np.unique(path.s, return_index=True)          # drop zero-length duplicates
    xs, ys = path.x[keep], path.y[keep]
    ds = ds or float(np.median(np.diff(s_u)))
    grid = np.arange(0.0, s_u[-1] + 1e-9, ds)
    x = np.interp(grid, s_u, xs); y = np.interp(grid, s_u, ys)
    sig = sigma_radii * path.rho / ds                           # in samples
    pad = int(np.ceil(4 * sig)) + 2
    # extend both ends along their tangents so the filter does not shrink the route
    t0 = np.array([x[1] - x[0], y[1] - y[0]]) / ds
    t1 = np.array([x[-1] - x[-2], y[-1] - y[-2]]) / ds
    k = np.arange(pad, 0, -1)[:, None]
    head = np.c_[x[0], y[0]] - k * ds * t0
    tail = np.c_[x[-1], y[-1]] + np.arange(1, pad + 1)[:, None] * ds * t1
    xy = np.vstack([head, np.c_[x, y], tail])
    xf = gaussian_filter1d(xy[:, 0], sig, mode="nearest")[pad:-pad]
    yf = gaussian_filter1d(xy[:, 1], sig, mode="nearest")[pad:-pad]
    dx, dy = np.gradient(xf, ds), np.gradient(yf, ds)
    psi = np.unwrap(np.arctan2(dy, dx))
    kappa = np.gradient(psi, ds)
    s = np.r_[0.0, np.cumsum(np.hypot(np.diff(xf), np.diff(yf)))]
    return Path(xf, yf, psi, kappa, s, path.rho)


def build_path(kind: str, waypoints, rho: float, ds: float = 0.5, sigma_radii: float = 0.5) -> Path:
    base = build_route(waypoints, rho, ds)
    if kind == "dubins":
        return base
    if kind == "dubins_smooth":
        return smooth_path(base, sigma_radii)
    raise ValueError(f"unknown route kind {kind}")


def max_curvature_ratio(path: Path) -> float:
    return float(np.max(np.abs(path.kappa)) * path.rho)


def curvature_jump(path: Path) -> float:
    """Largest sample-to-sample curvature change, in units of 1/rho. ~1 for Dubins, small if smooth."""
    return float(np.max(np.abs(np.diff(path.kappa))) * path.rho)

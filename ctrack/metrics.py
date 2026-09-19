"""Numbers that describe how well a run followed the path."""
from __future__ import annotations

import numpy as np

from .route import Path
from .sim import SimResult
from .vehicles import VehicleParams


def cross_track_errors(path: Path, x: np.ndarray, y: np.ndarray, chunk: int = 512):
    """Distance from each point to the nearest path sample.

    Uses the whole path (not the controller's own progress estimate) so the
    error is measured independently of the controller. Accuracy is about half
    the path sample spacing (0.25 m for the default 0.5 m spacing).
    """
    out = np.empty(len(x))
    for a in range(0, len(x), chunk):
        b = min(a + chunk, len(x))
        d2 = (x[a:b, None] - path.x[None, :]) ** 2 + (y[a:b, None] - path.y[None, :]) ** 2
        out[a:b] = np.sqrt(d2.min(axis=1))
    return out


def summarize(path: Path, vehicle: VehicleParams, res: SimResult, limit_mismatch: float = 1.0,
              transient_rmin: float = 4.0, cte_stride: int = 1):
    """Summary numbers for one run.

    The run starts with a deliberate random offset from the path. To keep that
    from dominating the error numbers, cross-track error is measured only after
    the first `transient_rmin` turning radii of travel (about 4 * r_min / V seconds).
    """
    st = max(1, int(cte_stride))   # >1 speeds up tuning (checks every st-th step)
    cte_all = cross_track_errors(path, res.x[::st], res.y[::st])
    t_skip = transient_rmin * vehicle.r_min / vehicle.airspeed
    keep = res.t[::st] >= t_skip
    cte = cte_all[keep] if keep.sum() > 10 else cte_all
    a_lim = limit_mismatch * vehicle.a_max
    sat = np.abs(res.a_cmd) > a_lim
    rms = float(np.sqrt(np.mean(cte ** 2)))
    mx = float(cte.max())
    # One number used for tuning and ranking (lower is better):
    #   rms error + 0.25 * worst error, both in turning radii, +1 if the run did not finish.
    score = rms / vehicle.r_min + 0.25 * mx / vehicle.r_min + (0.0 if res.completed else 1.0)
    return {
        "score": float(score),
        "max_cte_over_rmin": float(mx / vehicle.r_min),
        "completed": bool(res.completed),
        "finish_time_s": res.finish_time,
        "rms_cte_m": float(np.sqrt(np.mean(cte ** 2))),
        "max_cte_m": float(cte.max()),
        "rms_cte_over_rmin": float(np.sqrt(np.mean(cte ** 2)) / vehicle.r_min),
        "sat_fraction": float(sat.mean()),                       # share of time the limit was hit
        "effort": float(np.sqrt(np.mean(res.a_sat ** 2)) / vehicle.a_max),  # rms command / a_max
    }

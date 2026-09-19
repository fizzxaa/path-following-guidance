"""The control step used by the SITL runner, kept free of MAVLink so it can be tested.

Coordinates in this repo: x = east, y = north, angles counter-clockwise from east.
MAVLink LOCAL_POSITION_NED: x = north, y = east, z = down (velocities the same way).
Only this file and the runner convert between the two.
"""
from __future__ import annotations

import math


def velocity_command(law, path, x_east, y_north, v_east, v_north, speed, horizon,
                     min_speed: float = 0.5, max_turn: float = 1.0):
    """Turn a guidance law's lateral acceleration into a velocity command.

    The current ground velocity is rotated by (a / Vg) * horizon and scaled to `speed`.
    `horizon` acts like a gain: roughly the response time of the vehicle's velocity loop.
    Returns (v_north_cmd, v_east_cmd, a_cmd) in the MAVLink (NED) order.
    """
    vg = math.hypot(v_east, v_north)
    if vg < min_speed:                       # too slow for a meaningful course: pretend we fly along the path
        chi0 = path.psi[min(law.idx, path.n - 1)]
        v_east, v_north = speed * math.cos(chi0), speed * math.sin(chi0)
        vg = speed
    chi = math.atan2(v_north, v_east)
    a_cmd = law.command(path, x_east, y_north, v_east, v_north)
    turn = max(-max_turn, min(max_turn, a_cmd / vg * horizon))
    chi_cmd = chi + turn
    return speed * math.sin(chi_cmd), speed * math.cos(chi_cmd), a_cmd


def finished(law, path, x_east, y_north, tol: float) -> bool:
    return law.idx >= path.n - 3 and math.hypot(path.x[-1] - x_east, path.y[-1] - y_north) < tol


def save_run(out_csv, rows, path, r_min, speed, transient_rmin: float = 4.0):
    """Write the flown track and the planned path to CSV; return summary numbers.

    rows: list of (t, x_east, y_north, v_east, v_north, a_cmd, path_idx).
    Errors ignore the first `transient_rmin` turning radii of travel, like the simulator metrics.
    """
    import os
    import numpy as np
    from .metrics import cross_track_errors

    rows = np.asarray(rows, dtype=float)
    cte = cross_track_errors(path, rows[:, 1], rows[:, 2])
    os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)
    np.savetxt(out_csv, np.column_stack([rows, cte]), delimiter=",", comments="",
               header="t_s,x_east,y_north,v_east,v_north,a_cmd,path_idx,cte_m")
    stem, _ = os.path.splitext(out_csv)
    np.savetxt(stem + "_path.csv", np.column_stack([path.x, path.y]), delimiter=",", comments="",
               header="x_east,y_north")
    keep = rows[:, 0] >= transient_rmin * r_min / speed
    c = cte[keep] if keep.sum() > 10 else cte
    rms, mx = float(np.sqrt(np.mean(c ** 2))), float(c.max())
    return {"rms_cte_m": rms, "max_cte_m": mx,
            "rms_cte_over_rmin": rms / r_min, "max_cte_over_rmin": mx / r_min}

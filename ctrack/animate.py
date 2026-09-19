"""Watch a run: an animated GIF of the vehicle(s) following the planned path.

    python -m ctrack.animate                                   # quadcopter, all 3 laws, hard case
    python -m ctrack.animate --vehicle fixed_wing --wind 0.15 --radius-factor 1.5 --mismatch 1.0
    python -m ctrack.animate --tuned                           # use the tuned gains
    python -m ctrack.animate --csv-dir results/export          # also write CSVs for MATLAB

The three panels use the SAME route, start and wind, so you can compare the laws directly.
A red "LIMIT HIT" label appears when a law asks for more turn than the vehicle can give.
"""
from __future__ import annotations

import argparse
import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

from .guidance import LAW_NAMES, make_law
from .metrics import cross_track_errors
from .scenarios import get_path
from .sim import SimConfig, simulate
from .vehicles import VEHICLES

COLORS = {"pure_pursuit": "#1f77b4", "l1": "#2ca02c", "vector_field": "#d62728", "lead_vf": "#9467bd"}


def _tuned(params_path, vehicle, law):
    if params_path and os.path.exists(params_path):
        return json.load(open(params_path))[vehicle][law]["params"]
    return None


def run_all(vehicle, laws, wind, radius_factor, mismatch, route_seed, seed, tuned_path=None):
    veh = VEHICLES[vehicle]
    path = get_path(veh, route_seed, radius_factor)
    cfg = SimConfig(wind_speed=wind * veh.airspeed, limit_mismatch=mismatch)
    runs = {}
    for law in laws:
        obj = make_law(law, veh.r_min, _tuned(tuned_path, vehicle, law))
        res = simulate(path, veh, obj, cfg, seed=seed)
        cte = cross_track_errors(path, res.x, res.y)
        runs[law] = (res, cte)
    return veh, path, cfg, runs


def make_animation(out_gif, vehicle="quadcopter", laws=None, wind=0.30, radius_factor=1.0,
                   mismatch=0.8, route_seed=None, seed=0, fps=20, speedup=8.0, dpi=70,
                   tuned_path=None, csv_dir=None):
    laws = laws or list(LAW_NAMES)
    veh, path, cfg, runs = run_all(vehicle, laws, wind, radius_factor, mismatch, route_seed, seed, tuned_path)
    a_true = mismatch * veh.a_max
    step = max(1, int(round(speedup / (fps * cfg.dt))))
    n_max = max(len(r.t) for r, _ in runs.values())
    frames = list(range(0, n_max, step)) + [n_max - 1] * (fps * 2)   # hold the last frame ~2 s

    n = len(laws)
    fig, axes = plt.subplots(1, n, figsize=(4.6 * n, 4.9), squeeze=False)
    axes = axes[0]
    size = 0.28 * veh.r_min
    tri = np.array([[1.0, 0.0], [-0.7, 0.5], [-0.7, -0.5]]) * size
    pad = 2.0 * veh.r_min
    xlim = (path.x.min() - pad, path.x.max() + pad)
    ylim = (path.y.min() - pad, path.y.max() + pad)
    parts = {}
    for ax, law in zip(axes, laws):
        res, cte = runs[law]
        ax.plot(path.x, path.y, color="0.55", ls="--", lw=1.2, label="planned path")
        trail, = ax.plot([], [], color=COLORS[law], lw=1.5)
        poly = Polygon(tri, closed=True, color=COLORS[law], zorder=5)
        ax.add_patch(poly)
        txt = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=8,
                      family="monospace", bbox=dict(fc="white", ec="0.8", alpha=0.85))
        lim = ax.text(0.98, 0.98, "", transform=ax.transAxes, va="top", ha="right",
                      fontsize=9, color="red", weight="bold")
        ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect("equal")
        ax.set_title(law, color=COLORS[law], fontsize=11)
        ax.set_xticks([]); ax.set_yticks([])
        # wind arrow (fixed in a corner of the panel)
        if cfg.wind_speed > 0:
            ax.annotate("", xy=(0.90 + 0.07 * math.cos(res.wind_dir), 0.07 + 0.07 * math.sin(res.wind_dir)),
                        xytext=(0.90, 0.07), xycoords="axes fraction",
                        arrowprops=dict(arrowstyle="->", color="0.3", lw=1.5))
            ax.text(0.90, 0.015, f"wind {cfg.wind_speed:.1f} m/s", transform=ax.transAxes,
                    ha="center", fontsize=7, color="0.3")
        parts[law] = (trail, poly, txt, lim)
    fig.suptitle(f"{vehicle}: planned radius x{radius_factor}, true limit x{mismatch}, "
                 f"r_min = {veh.r_min:.1f} m", fontsize=10)
    fig.tight_layout()

    def update(f):
        for law in laws:
            res, cte = runs[law]
            trail, poly, txt, lim = parts[law]
            i = min(f, len(res.t) - 1)
            trail.set_data(res.x[:i + 1], res.y[:i + 1])
            j0, j1 = max(i - 1, 0), min(i + 1, len(res.t) - 1)
            course = math.atan2(res.y[j1] - res.y[j0], res.x[j1] - res.x[j0])
            c, s = math.cos(course), math.sin(course)
            R = np.array([[c, -s], [s, c]])
            poly.set_xy(tri @ R.T + np.array([res.x[i], res.y[i]]))
            done = i >= len(res.t) - 1
            txt.set_text(f"t = {res.t[i]:6.1f} s\nerror = {cte[i]:5.2f} m\n"
                         f"max so far = {cte[:i + 1].max():5.2f} m" + ("\nFINISHED" if done and res.completed else ""))
            lim.set_text("LIMIT HIT" if abs(res.a_cmd[i]) > a_true else "")
        return []

    os.makedirs(os.path.dirname(out_gif) or ".", exist_ok=True)
    anim = FuncAnimation(fig, update, frames=frames, blit=False)
    anim.save(out_gif, writer=PillowWriter(fps=fps), dpi=dpi)
    update(frames[-1])
    still = os.path.splitext(out_gif)[0] + ".png"
    fig.savefig(still, dpi=130)
    plt.close(fig)

    if csv_dir:
        os.makedirs(csv_dir, exist_ok=True)
        np.savetxt(os.path.join(csv_dir, f"{vehicle}_path.csv"),
                   np.column_stack([path.x, path.y, path.psi, path.kappa, path.s]),
                   delimiter=",", header="x,y,psi,kappa,s", comments="")
        for law in laws:
            res, cte = runs[law]
            np.savetxt(os.path.join(csv_dir, f"{vehicle}_{law}.csv"),
                       np.column_stack([res.t, res.x, res.y, res.a_cmd, res.a_sat, cte]),
                       delimiter=",", header="t,x,y,a_cmd,a_sat,cte", comments="")
    for law in laws:
        res, cte = runs[law]
        print(f"{law:13s} finished={res.completed}  rms error={np.sqrt(np.mean(cte**2)):.2f} m  "
              f"max error={cte.max():.2f} m")
    return out_gif, still


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", choices=list(VEHICLES), default="quadcopter")
    ap.add_argument("--law", choices=LAW_NAMES + ["all"], default="all")
    ap.add_argument("--wind", type=float, default=0.30, help="steady wind / airspeed")
    ap.add_argument("--radius-factor", type=float, default=1.0)
    ap.add_argument("--mismatch", type=float, default=0.8)
    ap.add_argument("--route", default="standard", help="'standard' or a route seed number")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--speedup", type=float, default=8.0, help="simulated seconds per video second")
    ap.add_argument("--tuned", action="store_true", help="use results/tuned_params.json")
    ap.add_argument("--csv-dir", default=None, help="also write CSVs for MATLAB")
    ap.add_argument("--out", default="results/run.gif")
    args = ap.parse_args()
    route_seed = None if args.route == "standard" else int(args.route)
    laws = list(LAW_NAMES) if args.law == "all" else [args.law]
    gif, still = make_animation(args.out, args.vehicle, laws, args.wind, args.radius_factor,
                                args.mismatch, route_seed, args.seed, args.fps, args.speedup,
                                tuned_path="results/tuned_params.json" if args.tuned else None,
                                csv_dir=args.csv_dir)
    print("wrote", gif, "and", still)


if __name__ == "__main__":
    main()

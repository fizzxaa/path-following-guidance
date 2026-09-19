"""Make the three figures used in the README / report.

    python -m ctrack.plots
"""
from __future__ import annotations

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .guidance import LAW_NAMES, make_law
from .route import build_route, standard_route
from .sim import SimConfig, simulate
from .vehicles import VEHICLES

COLORS = {"pure_pursuit": "#1f77b4", "l1": "#2ca02c", "vector_field": "#d62728", "lead_vf": "#9467bd"}


def fig_trajectories(outdir, vehicle="quadcopter", wind_frac=0.3, radius_factor=1.0, mismatch=0.8, seed=0):
    """The route and the path each law actually flew, in one hard scenario."""
    veh = VEHICLES[vehicle]
    path = build_route(standard_route(veh.r_min), radius_factor * veh.r_min)
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(path.x, path.y, "k--", lw=1.5, label="planned Dubins path")
    for name in LAW_NAMES:
        cfg = SimConfig(wind_speed=wind_frac * veh.airspeed, limit_mismatch=mismatch)
        res = simulate(path, veh, make_law(name, veh.r_min), cfg, seed=seed)
        ax.plot(res.x, res.y, color=COLORS[name], lw=1.2, label=f"{name} (done={res.completed})")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(f"{vehicle}: wind {wind_frac:.0%} of airspeed, planned radius x{radius_factor}, "
                 f"true limit x{mismatch}")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig1_trajectories.png"), dpi=150)
    plt.close(fig)


def fig_wind(df, outdir):
    """Which law handles wind best? Median RMS error (in turning radii) with spread."""
    d = df[(df.completed) & (df.mismatch == 1.0) & (df.radius_factor == 1.5)]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=False)
    for ax, veh in zip(axes, ["quadcopter", "fixed_wing"]):
        for name in LAW_NAMES:
            g = d[(d.vehicle == veh) & (d.law == name)].groupby("wind_frac").rms_cte_over_rmin
            med, q1, q3 = g.median(), g.quantile(0.25), g.quantile(0.75)
            ax.plot(med.index, med.values, "-o", color=COLORS[name], label=name)
            ax.fill_between(med.index, q1.values, q3.values, color=COLORS[name], alpha=0.15)
        ax.set_title(veh); ax.set_xlabel("steady wind / airspeed")
        ax.set_ylabel("RMS cross-track error / r_min")
        ax.legend(fontsize=8)
    fig.suptitle("Tracking error vs wind (finished runs, planned radius x1.5, no limit mismatch)")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig2_wind.png"), dpi=150)
    plt.close(fig)


def fig_limits(df, outdir):
    """What happens when the plan is tight and the vehicle is weaker than assumed?"""
    d = df[df.wind_frac == 0.15]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    for ax, veh in zip(axes, ["quadcopter", "fixed_wing"]):
        sub = d[d.vehicle == veh]
        xs = np.arange(len(sub.radius_factor.unique()) * 2)
        labels, w = [], 0.8 / len(LAW_NAMES)
        combos = [(rf, m) for rf in sorted(sub.radius_factor.unique()) for m in sorted(sub.mismatch.unique(), reverse=True)]
        for i, name in enumerate(LAW_NAMES):
            vals = [sub[(sub.law == name) & (sub.radius_factor == rf) & (sub.mismatch == m)].max_cte_m.median()
                    / VEHICLES[veh].r_min for rf, m in combos]
            ax.bar(xs + (i - (len(LAW_NAMES) - 1) / 2) * w, vals, w, color=COLORS[name], label=name)
        labels = [f"R x{rf}\nlimit x{m}" for rf, m in combos]
        ax.set_xticks(xs); ax.set_xticklabels(labels, fontsize=7)
        ax.set_ylabel("median MAX cross-track error / r_min")
        ax.set_title(veh); ax.legend(fontsize=8)
    fig.suptitle("Effect of planned radius and a weaker-than-assumed vehicle (wind 15%)")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig3_limits.png"), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="results/benchmark.csv")
    ap.add_argument("--out", default="results")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    fig_trajectories(args.out)
    if os.path.exists(args.csv):
        df = pd.read_csv(args.csv)
        fig_wind(df, args.out)
        fig_limits(df, args.out)
        print("wrote fig1..fig3 to", args.out)
    else:
        print("no benchmark csv yet; wrote fig1 only")


if __name__ == "__main__":
    main()

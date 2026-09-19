"""How much does each law degrade when the world is worse than the model it was tuned in?

    python -m ctrack.robustness          # ~3 min

Uses the TUNED gains (results/tuned_params.json) on the 12 test routes, in two conditions:
  typical  : wind 15%, planned radius x1.5, vehicle as rated
  at limit : wind 15%, planned radius x1.0, vehicle as rated
and applies one imperfection at a time (plus a combined one):
  position noise 2% of r_min, velocity noise 5% of airspeed, sensor delay 0.2 s,
  response lag x2, airspeed x0.85.
"""
from __future__ import annotations

import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .evaluate import COLORS, boot_ci
from .guidance import LAW_NAMES, make_law
from .metrics import summarize
from .scenarios import TEST_ROUTE_SEEDS, get_path
from .sim import SimConfig, simulate
from .vehicles import VEHICLES

CONDITIONS = {"typical": (0.15, 1.5), "at limit": (0.15, 1.0)}
PERTURBATIONS = {
    "none": {},
    "position noise": {"pos_noise_frac": 0.02},
    "velocity noise": {"vel_noise_frac": 0.05},
    "sensor delay 0.2 s": {"sensor_delay": 0.2},
    "response lag x2": {"lag_scale": 2.0},
    "airspeed x0.85": {"airspeed_scale": 0.85},
    "combined": {"pos_noise_frac": 0.02, "vel_noise_frac": 0.05, "sensor_delay": 0.1, "lag_scale": 1.5},
}


def run(params_path, out_csv, seeds_per_route=2):
    tuned = json.load(open(params_path))
    rows = []
    for vname, veh in VEHICLES.items():
        for cname, (wind, rf) in CONDITIONS.items():
            for pname, extra in PERTURBATIONS.items():
                for law in LAW_NAMES:
                    lawp = tuned[vname][law]["params"]
                    for rs in TEST_ROUTE_SEEDS:
                        path = get_path(veh, rs, rf)
                        for sd in range(seeds_per_route):
                            cfg = SimConfig(wind_speed=wind * veh.airspeed, **extra)
                            res = simulate(path, veh, make_law(law, veh.r_min, lawp), cfg, seed=rs * 1000 + sd)
                            s = summarize(path, veh, res)
                            rows.append({"vehicle": vname, "condition": cname, "perturbation": pname,
                                         "law": law, "route": rs, "seed": sd, "score": s["score"],
                                         "completed": s["completed"], "sat_fraction": s["sat_fraction"]})
        print(vname, "done", flush=True)
    df = pd.DataFrame(rows)
    os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)
    df.to_csv(out_csv, index=False)
    return df


def table(df, vehicle, condition):
    d = df[(df.vehicle == vehicle) & (df.condition == condition)]
    out = {}
    for law in LAW_NAMES:
        col = {}
        for pname in PERTURBATIONS:
            x = d[(d.law == law) & (d.perturbation == pname)].groupby("route").score.mean().values
            col[pname] = boot_ci(x)[0]
        out[law] = col
    return pd.DataFrame(out).round(4)


def figure(df, outdir):
    fig, axes = plt.subplots(2, 2, figsize=(13, 7.5), sharex=True)
    names = list(PERTURBATIONS)
    for r, vname in enumerate(VEHICLES):
        for c, cname in enumerate(CONDITIONS):
            ax = axes[r, c]
            for i, law in enumerate(LAW_NAMES):
                vals = [table(df, vname, cname).loc[p, law] for p in names]
                ax.bar(np.arange(len(names)) + (i - 1.5) * 0.2, vals, 0.2, color=COLORS[law], label=law)
            ax.set_title(f"{vname}, {cname}", fontsize=9)
            ax.set_xticks(range(len(names))); ax.set_xticklabels([n.replace(" ", "\n", 1) for n in names], fontsize=7)
            if c == 0:
                ax.set_ylabel("mean score (lower is better)")
    axes[0, 0].legend(fontsize=7)
    fig.suptitle("Robustness on the 12 test routes: one imperfection at a time (tuned gains)")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig5_robustness.png"), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", default="results/tuned_params.json")
    ap.add_argument("--csv", default="results/robustness.csv")
    ap.add_argument("--out", default="results")
    ap.add_argument("--reuse", action="store_true")
    args = ap.parse_args()
    df = pd.read_csv(args.csv) if args.reuse else run(args.params, args.csv)
    figure(df, args.out)
    pd.set_option("display.width", 200)
    for vname in VEHICLES:
        for cname in CONDITIONS:
            print(f"\\n{vname}, {cname}: mean score by perturbation (lower is better)")
            print(table(df, vname, cname).to_string())


if __name__ == "__main__":
    main()

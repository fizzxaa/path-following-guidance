"""Final test on routes that were never used for tuning, with honest error bars.

    python -m ctrack.evaluate            # ~3-4 min on one core

For every vehicle and law it runs the DEFAULT gains and the TUNED gains on the 12 test
routes x 18 conditions. Then it reports paired differences (same scenario, two variants).

Error bars: a cluster bootstrap over ROUTES. Scenarios on the same route are not
independent, so we average within each route first and resample the routes.
With 12 routes the intervals are wide. That is the honest size of the uncertainty.
"""
from __future__ import annotations

import argparse
import json
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .guidance import LAW_NAMES
from .scenarios import CONDITIONS, TEST_ROUTE_SEEDS, run_scenario
from .vehicles import VEHICLES

COLORS = {"pure_pursuit": "#1f77b4", "l1": "#2ca02c", "vector_field": "#d62728", "lead_vf": "#9467bd"}


def run_heldout(params_path, out_csv, n_standard_seeds=5):
    tuned = json.load(open(params_path))
    rows = []
    t0 = time.time()
    for vname, veh in VEHICLES.items():
        for variant in ("default", "tuned"):
            for law in LAW_NAMES:
                params = None if variant == "default" else tuned[vname][law]["params"]
                # 12 random test routes x 18 conditions
                for rs in TEST_ROUTE_SEEDS:
                    for ci, cond in enumerate(CONDITIONS):
                        s = run_scenario(veh, law, params, rs, cond, sim_seed=rs * 1000 + ci)
                        rows.append({"vehicle": vname, "variant": variant, "law": law, "route": rs,
                                     "cond_idx": ci, "wind_frac": cond[0], "radius_factor": cond[1],
                                     "mismatch": cond[2], **s})
                # the fixed standard route as an extra sanity check
                for ci, cond in enumerate(CONDITIONS):
                    for sd in range(n_standard_seeds):
                        s = run_scenario(veh, law, params, None, cond, sim_seed=5000 + ci * 10 + sd)
                        rows.append({"vehicle": vname, "variant": variant, "law": law, "route": -1,
                                     "cond_idx": ci, "wind_frac": cond[0], "radius_factor": cond[1],
                                     "mismatch": cond[2], **s})
            print(f"[{time.time() - t0:4.0f}s] {vname} done", flush=True)
    df = pd.DataFrame(rows)
    os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)
    df.to_csv(out_csv, index=False)
    print("wrote", out_csv, len(df), "rows")
    return df


def per_route_means(df, vehicle, variant, law, subset=None):
    d = df[(df.vehicle == vehicle) & (df.variant == variant) & (df.law == law) & (df.route >= 0)]
    if subset is not None:
        d = d[subset(d)]
    return d.groupby("route").score.mean().sort_index().values


def boot_ci(x, n=5000, seed=0):
    """Mean and 95% interval, resampling routes with replacement."""
    rng = np.random.default_rng(seed)
    x = np.asarray(x, dtype=float)
    idx = rng.integers(0, len(x), size=(n, len(x)))
    m = x[idx].mean(axis=1)
    return float(x.mean()), float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


SUBSETS = {
    "all conditions": None,
    "planned at limit (radius x1.0)": lambda d: d.radius_factor == 1.0,
    "planned with margin (radius >= x1.5)": lambda d: d.radius_factor >= 1.5,
}


def make_stats(df):
    rows = []
    for vname in VEHICLES:
        for sub_name, sub in SUBSETS.items():
            # 1) tuned vs default, per law (paired by route)
            for law in LAW_NAMES:
                a = per_route_means(df, vname, "default", law, sub)
                b = per_route_means(df, vname, "tuned", law, sub)
                m, lo, hi = boot_ci(b - a)
                rows.append({"vehicle": vname, "subset": sub_name, "comparison": f"{law}: tuned - default",
                             "mean_diff": m, "ci_lo": lo, "ci_hi": hi,
                             "reading": "tuned better" if hi < 0 else ("tuned worse" if lo > 0 else "no clear difference")})
            # 2) law vs law, tuned gains
            for x, y in (("pure_pursuit", "l1"), ("vector_field", "l1"), ("vector_field", "pure_pursuit"),
                         ("lead_vf", "l1"), ("lead_vf", "vector_field")):
                a = per_route_means(df, vname, "tuned", x, sub)
                b = per_route_means(df, vname, "tuned", y, sub)
                m, lo, hi = boot_ci(a - b)
                rows.append({"vehicle": vname, "subset": sub_name, "comparison": f"tuned {x} - tuned {y}",
                             "mean_diff": m, "ci_lo": lo, "ci_hi": hi,
                             "reading": f"{x} better" if hi < 0 else (f"{y} better" if lo > 0 else "no clear difference")})
    return pd.DataFrame(rows)


def fig_scores(df, outdir):
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharey="row")
    subs = ["all conditions", "planned at limit (radius x1.0)"]
    for r, vname in enumerate(VEHICLES):
        for c, sub_name in enumerate(subs):
            ax = axes[r, c]
            for i, law in enumerate(LAW_NAMES):
                for j, variant in enumerate(("default", "tuned")):
                    x = per_route_means(df, vname, variant, law, SUBSETS[sub_name])
                    m, lo, hi = boot_ci(x)
                    pos = i + (j - 0.5) * 0.35
                    ax.bar(pos, m, 0.33, color=COLORS[law], alpha=0.45 if variant == "default" else 1.0,
                           label=f"{law} ({variant})" if (r == 0 and c == 0) else None)
                    ax.errorbar(pos, m, yerr=[[m - lo], [hi - m]], color="k", capsize=3, lw=1)
            ax.set_xticks(range(len(LAW_NAMES))); ax.set_xticklabels([n.replace("_", chr(10), 1) for n in LAW_NAMES], fontsize=8)
            ax.set_title(f"{vname}: {sub_name}", fontsize=9)
            if c == 0:
                ax.set_ylabel("score (lower is better)")
    axes[0, 0].legend(fontsize=7, ncol=2)
    fig.suptitle("Held-out test routes: default vs tuned gains (bars: mean, lines: 95% interval over routes)")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig4_heldout_scores.png"), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", default="results/tuned_params.json")
    ap.add_argument("--csv", default="results/heldout.csv")
    ap.add_argument("--out", default="results")
    ap.add_argument("--reuse", action="store_true", help="skip the simulation, reuse the csv")
    args = ap.parse_args()
    df = pd.read_csv(args.csv) if args.reuse else run_heldout(args.params, args.csv)
    stats = make_stats(df)
    stats.to_csv(os.path.join(args.out, "heldout_stats.csv"), index=False)
    fig_scores(df, args.out)
    pd.set_option("display.width", 220); pd.set_option("display.max_colwidth", 60)
    print(stats.round(4).to_string(index=False))


if __name__ == "__main__":
    main()

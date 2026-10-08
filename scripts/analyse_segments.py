"""Tie-aware, segment-resolved report from results/heldout_segments.csv.

    PYTHONPATH=. python scripts/analyse_segments.py [csv] > results/segments_report.md

Uncertainty: bootstrap over the 12 test ROUTES (scenarios on one route are not independent).
A law is "tied with the best" in a region if the paired 95% interval of (law - best) includes 0.
A whole-path winner "flips" in a condition only if it is distinguishably WORSE than the best law in
that region (paired interval entirely above 0).
"""
import sys

import numpy as np
import pandas as pd

CSV = sys.argv[1] if len(sys.argv) > 1 else "results/heldout_segments.csv"
import os
W = os.environ.get("SEG_WINDOW", "1.0")
REGIONS = ["all", "straight", "transition", "arc"]
rng = np.random.default_rng(0)


def boot_idx(n, B=4000):
    return rng.integers(0, n, size=(B, n))


def ci(x, idx):
    m = x[idx].mean(axis=1)
    return x.mean(), np.percentile(m, 2.5), np.percentile(m, 97.5)


df = pd.read_csv(CSV)
laws = list(dict.fromkeys(df.law))
print(f"# Segment-resolved held-out report ({CSV})\n")
print(f"Window = {W} turning radius. {df.route.nunique()} routes, {df.cond_idx.nunique()} conditions. "
      "RMS cross-track error in metres (exact projection), mean over routes with 95% route-bootstrap interval.\n")

for veh, d in df.groupby("vehicle"):
    print(f"## {veh}\n")
    print("Turning share of the path (fraction of samples on arcs): "
          f"{d.groupby('route').turn_share.mean().min():.2f}-{d.groupby('route').turn_share.mean().max():.2f}\n")
    for sub_name, sub in (("all conditions", d), ("planned at limit (radius x1.0)", d[d.radius_factor == 1.0])):
        print(f"### {sub_name}\n")
        print("| law | " + " | ".join(REGIONS) + " |")
        print("|---|" + "---|" * len(REGIONS))
        per = {}
        routes = np.sort(sub.route.unique())
        idx = boot_idx(len(routes))
        for law in laws:
            row = []
            for r in REGIONS:
                col = f"rms_{r}_w{W}"
                v = sub[sub.law == law].groupby("route")[col].mean().reindex(routes).values
                per[(law, r)] = v
                m, lo, hi = ci(v, idx)
                row.append(f"{m:.3f} [{lo:.3f}, {hi:.3f}]")
            print(f"| {law} | " + " | ".join(row) + " |")
        print()
        for r in REGIONS:
            means = {l: per[(l, r)].mean() for l in laws}
            best = min(means, key=means.get)
            tied = [l for l in laws if l == best or ci(per[(l, r)] - per[(best, r)], idx)[1] <= 0]
            print(f"- {r}: best {best}; tie set (paired 95% interval includes 0): {', '.join(tied)}")
        print()
    # per-condition flips: whole-path winner vs best in arcs / straights
    flips = {"arc": 0, "straight": 0}
    n_cond = 0
    routes = np.sort(d.route.unique()); idx = boot_idx(len(routes), 2000)
    for c, g in d.groupby("cond_idx"):
        n_cond += 1
        pr = {(l, r): g[g.law == l].set_index("route")[f"rms_{r}_w{W}"].reindex(routes).values
              for l in laws for r in ("all", "arc", "straight")}
        win = min(laws, key=lambda l: pr[(l, "all")].mean())
        for r in ("arc", "straight"):
            best = min(laws, key=lambda l: pr[(l, r)].mean())
            if best != win and ci(pr[(win, r)] - pr[(best, r)], idx)[1] > 0:
                flips[r] += 1
    print(f"Whole-path winner is distinguishably worse than the best law in arcs in "
          f"{flips['arc']}/{n_cond} conditions, and on straights in {flips['straight']}/{n_cond}.\n")
    print("### Effort and saturation (mean over all runs)\n")
    print("| law | sat_fraction | effort (rms cmd / a_max) | not completed |")
    print("|---|---|---|---|")
    for law in laws:
        g = d[d.law == law]
        print(f"| {law} | {g.sat_fraction.mean():.3f} | {g.effort.mean():.3f} | {(~g.completed.astype(bool)).sum()} |")
    print()

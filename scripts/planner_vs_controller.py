"""Planner effect vs controller effect, on whole-path exact RMS (all 18 conditions, 12 routes).

  controller effect : spread between the best and worst law on one route kind
  planner effect    : mean over laws of |RMS(dubins) - RMS(dubins_smooth)|, same law, same gains origin

Gains: 'dubins-tuned' = tuned on Dubins routes, applied to smooth routes (transfer);
       'own-tuned'    = retuned on the smooth routes (fair to the smooth route).
    PYTHONPATH=. python scripts/planner_vs_controller.py
"""
import numpy as np
import pandas as pd

rng = np.random.default_rng(0)
FILES = {
    ("dubins", "dubins-tuned"): "results/heldout_segments_6laws.csv",
    ("smooth", "dubins-tuned"): "results/heldout_smooth_dubinsgains.csv",
    ("smooth", "own-tuned"): "results/heldout_smooth_owngains.csv",
}
data = {k: pd.read_csv(v) for k, v in FILES.items()}
laws = list(dict.fromkeys(data[("dubins", "dubins-tuned")].law))
print("# Planner vs controller (whole-path RMS, metres)\n")
for veh in ("quadcopter", "fixed_wing"):
    per = {}
    for k, d in data.items():
        g = d[d.vehicle == veh]
        per[k] = {l: g[g.law == l].groupby("route")["rms_all_w1.0"].mean().sort_index().values for l in laws}
    idx = rng.integers(0, 12, size=(4000, 12))
    print(f"## {veh}\n")
    print("| law | Dubins | smooth (Dubins gains) | smooth (own gains) |")
    print("|---|---|---|---|")
    for l in laws:
        print(f"| {l} | " + " | ".join(f"{per[k][l].mean():.3f}" for k in data) + " |")
    print()
    for k in data:
        m = {l: per[k][l].mean() for l in laws}
        best, worst = min(m, key=m.get), max(m, key=m.get)
        sp = per[k][worst] - per[k][best]
        b = sp[idx].mean(1)
        print(f"- controller effect on {k[0]} routes, {k[1]}: spread {sp.mean():.3f} m [{np.percentile(b, 2.5):.3f}, {np.percentile(b, 97.5):.3f}] "
              f"(best {best}, worst {worst}); ranking: {' < '.join(sorted(laws, key=m.get))}")
    for gk in (("smooth", "dubins-tuned"), ("smooth", "own-tuned")):
        diff = np.mean([np.abs(per[("dubins", "dubins-tuned")][l] - per[gk][l]) for l in laws], axis=0)
        signed = np.mean([per[gk][l] - per[("dubins", "dubins-tuned")][l] for l in laws], axis=0)
        b = diff[idx].mean(1)
        print(f"- planner effect (Dubins -> smooth, {gk[1]}): mean |change| {diff.mean():.3f} m [{np.percentile(b, 2.5):.3f}, {np.percentile(b, 97.5):.3f}]; "
              f"mean signed change {signed.mean():+.3f} m")
    print()

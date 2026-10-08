"""Do the headline claims survive a worse or different world?

Claims are fixed BEFORE running (paired 95% interval over the 12 test routes, tuned Dubins gains):
  C1a  carrot_chasing has lower straight-segment error than L1
  C1b  carrot_chasing has higher arc-segment error than L1
  C2a  vector_field has higher arc error than L1
  C2b  lead_vf has lower arc error than vector_field
  C3   vector_field and L1 are NOT distinguishable on straights (interval includes 0)
Each claim is checked per vehicle x condition x perturbation. Output: results/sensitivity.csv and
results/sensitivity_report.md.

    PYTHONPATH=. python scripts/sensitivity.py
"""
import json
import time

import numpy as np
import pandas as pd

from ctrack.guidance import ALL_LAW_NAMES, make_law
from ctrack.scenarios import TEST_ROUTE_SEEDS, get_path
from ctrack.segments import segment_rms_windows
from ctrack.sim import SimConfig, simulate
from ctrack.vehicles import VEHICLES

CONDS = {"typical (wind .15, r x1.5)": (0.15, 1.5), "at limit (wind .15, r x1.0)": (0.15, 1.0)}
PERT = {
    "none": {},
    "lag x0.5": {"lag_scale": 0.5},
    "lag x1.5": {"lag_scale": 1.5},
    "lag x2": {"lag_scale": 2.0},
    "true a_max x0.8": {"limit_mismatch": 0.8},
    "true a_max x0.6": {"limit_mismatch": 0.6},
    "position noise 2% r_min": {"pos_noise_frac": 0.02},
    "velocity noise 5%": {"vel_noise_frac": 0.05},
    "sensor delay 0.2 s": {"sensor_delay": 0.2},
    "gusts x3 (std 75% of wind)": {"gust_frac": 0.75},
    "wind 30%": {"wind_speed": "0.30"},
}
W = 1.0


def run(out="results/sensitivity.csv", params="results/tuned_params.json"):
    tuned = json.load(open(params))
    rows, t0 = [], time.time()
    for vname, veh in VEHICLES.items():
        t_skip = 4.0 * veh.r_min / veh.airspeed
        for cname, (wind, rf) in CONDS.items():
            for pname, extra in PERT.items():
                ex = dict(extra)
                wf = float(ex.pop("wind_speed", wind))
                for law in ALL_LAW_NAMES:
                    for rs in TEST_ROUTE_SEEDS:
                        path = get_path(veh, rs, rf)
                        cfg = SimConfig(wind_speed=wf * veh.airspeed, **ex)
                        mm = ex.get("limit_mismatch", 1.0)
                        res = simulate(path, veh, make_law(law, veh.r_min, tuned[vname][law]["params"]), cfg, seed=rs * 1000 + 7)
                        seg = segment_rms_windows(path, res.x, res.y, res.t >= t_skip, windows=(W,))[W]
                        rows.append({"vehicle": vname, "cond": cname, "pert": pname, "law": law, "route": rs,
                                     "completed": res.completed,
                                     **{f"rms_{k}": v[0] for k, v in seg.items()}})
        print(f"[{time.time() - t0:5.0f}s] {vname}", flush=True)
        pd.DataFrame(rows).to_csv(out, index=False)
    return pd.DataFrame(rows)


def check(df):
    rng = np.random.default_rng(0)
    out = []
    for (v, c, p), g in df.groupby(["vehicle", "cond", "pert"], sort=False):
        piv = {col: g.pivot(index="route", columns="law", values=col) for col in ("rms_straight", "rms_arc")}
        idx = rng.integers(0, len(piv["rms_arc"]), size=(3000, len(piv["rms_arc"])))

        def pci(col, a, b):
            d = (piv[col][a] - piv[col][b]).values
            m = d[idx].mean(axis=1)
            return np.percentile(m, 2.5), np.percentile(m, 97.5)
        r = {"vehicle": v, "cond": c, "pert": p}
        lo, hi = pci("rms_straight", "carrot_chasing", "l1"); r["C1a"] = hi < 0
        lo, hi = pci("rms_arc", "carrot_chasing", "l1"); r["C1b"] = lo > 0
        lo, hi = pci("rms_arc", "vector_field", "l1"); r["C2a"] = lo > 0
        lo, hi = pci("rms_arc", "lead_vf", "vector_field"); r["C2b"] = hi < 0
        lo, hi = pci("rms_straight", "vector_field", "l1"); r["C3"] = lo <= 0 <= hi
        r["not_completed"] = int((~g.completed.astype(bool)).sum())
        out.append(r)
    return pd.DataFrame(out)


if __name__ == "__main__":
    df = run()
    ck = check(df)
    ck.to_csv("results/sensitivity_checks.csv", index=False)
    claims = ["C1a", "C1b", "C2a", "C2b", "C3"]
    lines = ["# Sensitivity of the headline claims\n",
             "Each cell: does the claim hold (paired 95% interval over 12 routes) in that world?\n"]
    for v, g in ck.groupby("vehicle"):
        lines.append(f"\n## {v}\n\n| condition | perturbation | " + " | ".join(claims) + " | not completed |")
        lines.append("|---|---|" + "---|" * (len(claims) + 1))
        for _, r in g.iterrows():
            lines.append(f"| {r.cond} | {r.pert} | " + " | ".join("yes" if r[c] else "NO" for c in claims) + f" | {r.not_completed} |")
        lines.append("\nHold rate: " + ", ".join(f"{c} {g[c].mean() * 100:.0f}%" for c in claims))
    open("results/sensitivity_report.md", "w").write("\n".join(lines) + "\n")
    print("\n".join(lines[-4:]))

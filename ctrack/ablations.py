"""Sensitivity checks on the test routes: which ingredient of each law matters?

    python -m ctrack.ablations           # ~4 min

  lead_vf : sweep lead_time from 0 (= the plain vector-field law) to 1.2 s
  vector_field : feed-forward on / off
  l1      : sweep the lookahead time
All other gains stay at their DEFAULT values, so the sweep isolates one thing at a time.
"""
from __future__ import annotations

import argparse
import os

import numpy as np
import pandas as pd

from .evaluate import boot_ci
from .scenarios import CONDITIONS, TEST_ROUTE_SEEDS, run_scenario
from .vehicles import VEHICLES

SWEEPS = [
    ("lead_vf", "lead_time", [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2]),
    ("vector_field", "feedforward", [True, False]),
    ("l1", "lookahead_time", [1.0, 1.75, 2.5, 3.5, 5.0]),
]
GROUPS = {"limit respected": lambda c: c[1] >= 1.5 and c[2] == 1.0,
          "vehicle 20% weaker": lambda c: c[1] >= 1.5 and c[2] == 0.8,
          "planned at limit": lambda c: c[1] == 1.0}


def run(routes, out_csv):
    rows = []
    for vname, veh in VEHICLES.items():
        for law, key, values in SWEEPS:
            for v in values:
                for rs in TEST_ROUTE_SEEDS[:routes]:
                    for ci, cond in enumerate(CONDITIONS):
                        s = run_scenario(veh, law, {key: v}, rs, cond, sim_seed=rs * 1000 + ci, cte_stride=2)
                        g = next(n for n, f in GROUPS.items() if f(cond)) if any(f(cond) for f in GROUPS.values()) else None
                        for n, f in GROUPS.items():
                            if f(cond):
                                rows.append({"vehicle": vname, "law": law, "param": key, "value": str(v),
                                             "group": n, "route": rs, "score": s["score"]})
        print(vname, "done", flush=True)
    df = pd.DataFrame(rows)
    os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)
    df.to_csv(out_csv, index=False)
    return df


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--routes", type=int, default=6)
    ap.add_argument("--csv", default="results/ablations.csv")
    ap.add_argument("--reuse", action="store_true")
    args = ap.parse_args()
    df = pd.read_csv(args.csv) if args.reuse else run(args.routes, args.csv)
    pd.set_option("display.width", 200)
    for vname in VEHICLES:
        for law, key, values in SWEEPS:
            d = df[(df.vehicle == vname) & (df.law == law)]
            t = d.groupby(["value", "group"]).score.mean().unstack().reindex([str(v) for v in values])
            print(f"\n{vname}: {law}, sweep of {key} (mean score, lower is better)")
            print(t[list(GROUPS)].round(4).to_string())


if __name__ == "__main__":
    main()

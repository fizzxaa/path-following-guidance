"""Held-out runs (tuned gains) with exact, segment-resolved error. Writes one row per run.

    PYTHONPATH=. python scripts/heldout_segments.py --out results/heldout_segments.csv
"""
import argparse
import json
import time

import numpy as np
import pandas as pd

from ctrack.guidance import ALL_LAW_NAMES, make_law
from ctrack.metrics import summarize
from ctrack.scenarios import CONDITIONS, TEST_ROUTE_SEEDS, get_path
from ctrack.segments import segment_rms_windows, turning_share
from ctrack.sim import SimConfig, simulate
from ctrack.vehicles import VEHICLES

WINDOWS = (0.5, 1.0, 2.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", default="results/tuned_params.json")
    ap.add_argument("--out", default="results/heldout_segments.csv")
    ap.add_argument("--route", default="dubins", choices=["dubins", "dubins_smooth"])
    ap.add_argument("--laws", nargs="*", default=list(ALL_LAW_NAMES))
    ap.add_argument("--vehicles", nargs="*", default=list(VEHICLES))
    a = ap.parse_args()
    tuned = json.load(open(a.params))
    rows, t0 = [], time.time()
    for vname in a.vehicles:
        veh = VEHICLES[vname]
        t_skip = 4.0 * veh.r_min / veh.airspeed
        for law in a.laws:
            params = tuned[vname][law]["params"]
            for rs in TEST_ROUTE_SEEDS:
                for ci, cond in enumerate(CONDITIONS):
                    wf, rf, mm = cond
                    path = get_path(veh, rs, rf, a.route)
                    res = simulate(path, veh, make_law(law, veh.r_min, params),
                                   SimConfig(wind_speed=wf * veh.airspeed, limit_mismatch=mm),
                                   seed=rs * 1000 + ci)
                    s = summarize(path, veh, res, limit_mismatch=mm)
                    keep = res.t >= t_skip
                    row = {"vehicle": vname, "law": law, "route": rs, "cond_idx": ci,
                           "wind_frac": wf, "radius_factor": rf, "mismatch": mm,
                           "turn_share": turning_share(path), "completed": s["completed"],
                           "sat_fraction": s["sat_fraction"], "effort": s["effort"]}
                    allseg = segment_rms_windows(path, res.x, res.y, keep, windows=WINDOWS)
                    for w, seg in allseg.items():
                        for k, (v, n) in seg.items():
                            row[f"rms_{k}_w{w}"] = v
                            row[f"n_{k}_w{w}"] = n
                    rows.append(row)
            print(f"[{time.time() - t0:5.0f}s] {vname} {law}", flush=True)
            pd.DataFrame(rows).to_csv(a.out, index=False)
    print("wrote", a.out, len(rows))


if __name__ == "__main__":
    main()

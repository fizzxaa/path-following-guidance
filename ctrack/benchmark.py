"""Run the full comparison and write results/benchmark.csv.

    python -m ctrack.benchmark --quick        # a few minutes on a laptop
    python -m ctrack.benchmark --seeds 20     # the full run
"""
from __future__ import annotations

import argparse
import csv
import json
import itertools
import os
import time
from concurrent.futures import ProcessPoolExecutor

from .guidance import LAW_NAMES, make_law
from .metrics import summarize
from .route import build_route, standard_route
from .sim import SimConfig, simulate
from .vehicles import VEHICLES

from .scenarios import MISMATCHES, RADIUS_FACTORS, WIND_FRACS  # noqa: E402


def _run_one(job):
    vname, law_name, wind_frac, rf, mism, seed, params = job
    veh = VEHICLES[vname]
    rho = rf * veh.r_min
    path = build_route(standard_route(veh.r_min), rho)
    law = make_law(law_name, veh.r_min, params)
    cfg = SimConfig(wind_speed=wind_frac * veh.airspeed, limit_mismatch=mism)
    res = simulate(path, veh, law, cfg, seed=seed)
    row = {"vehicle": vname, "law": law_name, "wind_frac": wind_frac,
           "radius_factor": rf, "mismatch": mism, "seed": seed,
           "path_length_m": round(path.length, 1)}
    row.update(summarize(path, veh, res, limit_mismatch=mism))
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--quick", action="store_true", help="5 seeds only")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--out", default="results/benchmark.csv")
    ap.add_argument("--params", default=None, help="tuned_params.json to use instead of default gains")
    args = ap.parse_args()
    seeds = range(5 if args.quick else args.seeds)

    tuned = json.load(open(args.params)) if args.params else None
    jobs = [(v, l, w, rf, m, sd, tuned[v][l]["params"] if tuned else None)
            for v, l, w, rf, m, sd in itertools.product(VEHICLES, LAW_NAMES, WIND_FRACS,
                                                        RADIUS_FACTORS, MISMATCHES, seeds)]
    print(f"{len(jobs)} runs on {args.jobs} workers ...")
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        rows = list(ex.map(_run_one, jobs, chunksize=8))
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {args.out} ({len(rows)} rows) in {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()

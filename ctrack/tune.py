"""Tune each law's gains on TUNING routes only.  Nothing here touches the test routes.

    python -m ctrack.tune            # ~7 min on one core, writes results/tuned_params.json

Method: random search (grid for the one-parameter laws). Every candidate is scored
on the same 6 tuning routes x 6 conditions, with the same random seeds, so
candidates are compared fairly. The default gains are always one of the candidates.
Score = rms error + 0.25 * worst error (in turning radii) + 1 if the run did not finish.

Caveat: the winner out of ~60 candidates looks a little better on the tuning set than it
really is (selection bias). That is why evaluate.py re-tests on routes never used here.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time

import numpy as np

from .guidance import DEFAULT_PARAMS, LAW_NAMES
from .scenarios import TUNE_ROUTE_SEEDS, run_scenario
from .vehicles import VEHICLES

# (wind fraction, radius factor, limit mismatch): a spread of easy and hard conditions
TUNE_CONDITIONS = [
    (0.15, 1.0, 0.8), (0.30, 1.0, 1.0),
    (0.15, 1.5, 1.0), (0.30, 1.5, 0.8),
    (0.00, 2.0, 1.0), (0.30, 2.0, 0.8),
]


def tuning_score(veh, law_name, params, stride=4):
    scores = []
    for rs in TUNE_ROUTE_SEEDS:
        for ci, cond in enumerate(TUNE_CONDITIONS):
            s = run_scenario(veh, law_name, params, rs, cond, sim_seed=rs * 1000 + ci, cte_stride=stride)
            scores.append(s["score"])
    return float(np.mean(scores))


def candidates(law_name, rng, n_random):
    d = DEFAULT_PARAMS[law_name]
    if law_name in ("pure_pursuit", "l1"):
        vals = sorted(set(np.round(np.linspace(0.8, 5.0, 15), 2).tolist() + [d["lookahead_time"]]))
        return [{"lookahead_time": float(v)} for v in vals]
    out = [dict(d)]                       # the default gains are always candidate 0
    for _ in range(n_random):
        c = {
            "k_e_scale": float(math.exp(rng.uniform(math.log(0.1), math.log(2.0)))),
            "chi_inf_deg": float(rng.uniform(30.0, 85.0)),
            "k_chi": float(math.exp(rng.uniform(math.log(0.3), math.log(3.0)))),
            "feedforward": True if law_name == "lead_vf" else bool(rng.integers(0, 2)),
        }
        if law_name == "lead_vf":
            c["lead_time"] = float(rng.uniform(0.0, 1.5))
        out.append(c)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-random", type=int, default=60, help="random candidates for vector_field")
    ap.add_argument("--out", default="results/tuned_params.json")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    result = {}
    t0 = time.time()
    for vname, veh in VEHICLES.items():
        result[vname] = {}
        for law in LAW_NAMES:
            rng = np.random.default_rng(args.seed)
            cands = candidates(law, rng, args.n_random)
            default_score = None
            best = (float("inf"), None)
            for i, params in enumerate(cands):
                sc = tuning_score(veh, law, params)
                if i == 0 and law in ("vector_field", "lead_vf"):
                    default_score = sc
                if law in ("pure_pursuit", "l1") and params["lookahead_time"] == DEFAULT_PARAMS[law]["lookahead_time"]:
                    default_score = sc
                if sc < best[0]:
                    best = (sc, params)
            result[vname][law] = {"params": best[1], "tuning_score": best[0],
                                  "default_tuning_score": default_score, "n_candidates": len(cands)}
            print(f"[{time.time() - t0:5.0f}s] {vname:10s} {law:13s} default {default_score:.4f} -> "
                  f"tuned {best[0]:.4f}  {best[1]}", flush=True)
            os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
            with open(args.out, "w") as f:
                json.dump(result, f, indent=2)
    print("wrote", args.out)


if __name__ == "__main__":
    main()

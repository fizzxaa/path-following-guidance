"""One place that defines the test conditions and runs a single scenario.

A *scenario* = one route + one condition (wind, planned radius, limit mismatch)
+ one random seed. Tuning and final testing use DIFFERENT routes and seeds.
Route seeds:  tuning 1000-1005,  final test 2000-2011,  the fixed 'standard' route is separate.
"""
from __future__ import annotations

import itertools

from .guidance import make_law
from .metrics import summarize
from .route import build_route, random_route, standard_route
from .sim import SimConfig, simulate
from .vehicles import VehicleParams

WIND_FRACS = [0.0, 0.15, 0.30]       # steady wind as a fraction of airspeed
RADIUS_FACTORS = [1.0, 1.5, 2.0]     # planned radius / vehicle's true minimum radius
MISMATCHES = [1.0, 0.8]              # true lateral-accel limit vs. what the planner assumed
CONDITIONS = list(itertools.product(WIND_FRACS, RADIUS_FACTORS, MISMATCHES))   # 18

TUNE_ROUTE_SEEDS = list(range(1000, 1006))
TEST_ROUTE_SEEDS = list(range(2000, 2012))

_path_cache: dict = {}


def get_path(veh: VehicleParams, route_seed, rf: float):
    """Route for a vehicle. route_seed=None -> the fixed standard route."""
    key = (veh.name, route_seed, rf)
    if key not in _path_cache:
        wps = standard_route(veh.r_min) if route_seed is None else random_route(veh.r_min, route_seed)
        _path_cache[key] = build_route(wps, rf * veh.r_min)
    return _path_cache[key]


def run_scenario(veh: VehicleParams, law_name: str, params, route_seed, cond, sim_seed: int,
                 cte_stride: int = 1):
    wind_frac, rf, mism = cond
    path = get_path(veh, route_seed, rf)
    law = make_law(law_name, veh.r_min, params)
    cfg = SimConfig(wind_speed=wind_frac * veh.airspeed, limit_mismatch=mism)
    res = simulate(path, veh, law, cfg, seed=sim_seed)
    out = summarize(path, veh, res, limit_mismatch=mism, cte_stride=cte_stride)
    out["path_length_m"] = round(path.length, 1)
    return out

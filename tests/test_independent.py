"""Cross-check against an independent re-implementation written from tests/independent/SPEC.md
by a separate agent that was not shown the repo code. If the two disagree, either the code has a
bug or the spec is ambiguous; both are worth knowing."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "independent"))
from indep_impl import IndepLaw, indep_simulate  # noqa: E402

from ctrack.guidance import LAW_NAMES, make_law  # noqa: E402
from ctrack.scenarios import get_path  # noqa: E402
from ctrack.sim import SimConfig, simulate  # noqa: E402
from ctrack.vehicles import FIXED_WING, QUADCOPTER  # noqa: E402

SPEC_LAWS = ["pure_pursuit", "l1", "vector_field", "lead_vf"]


def indep_from(law_name, mine):
    kw = {}
    if law_name in ("vector_field", "lead_vf"):
        kw = dict(k_e=mine.k_e, chi_inf=mine.chi_inf, k_chi=mine.k_chi, feedforward=mine.feedforward)
        if law_name == "lead_vf":
            kw["lead_time"] = mine.lead_time
    else:
        kw = dict(lookahead_time=mine.lookahead_time, min_lookahead=mine.min_lookahead)
    return IndepLaw(law_name, **kw)


@pytest.mark.parametrize("law", SPEC_LAWS)
@pytest.mark.parametrize("veh", [QUADCOPTER, FIXED_WING], ids=lambda v: v.name)
def test_commands_agree_on_random_states(law, veh):
    rng = np.random.default_rng(42)
    worst = 0.0
    for seed in (2000, 2001, 2002):
        path = get_path(veh, seed, 1.5)
        for _ in range(400):
            i = rng.integers(0, path.n)
            off = rng.uniform(-2.0, 2.0) * veh.r_min
            x = path.x[i] - off * math.sin(path.psi[i])
            y = path.y[i] + off * math.cos(path.psi[i])
            vg = rng.uniform(0.7, 1.3) * veh.airspeed
            chi = path.psi[i] + rng.uniform(-math.pi / 2, math.pi / 2)
            vx, vy = vg * math.cos(chi), vg * math.sin(chi)
            a = make_law(law, veh.r_min)
            b = indep_from(law, a)
            a.reset(path, x, y); b.reset(path, x, y)
            ca, cb = a.command(path, x, y, vx, vy), b.command(path, x, y, vx, vy)
            worst = max(worst, abs(ca - cb))
            assert a.idx == b.idx
    assert worst < 1e-9, worst


@pytest.mark.parametrize("law", SPEC_LAWS)
@pytest.mark.parametrize("veh", [QUADCOPTER, FIXED_WING], ids=lambda v: v.name)
def test_trajectories_agree_in_ideal_sim(law, veh):
    for seed, rf in ((2000, 1.0), (2005, 1.5), (2011, 2.0)):
        path = get_path(veh, seed, rf)
        cfg = SimConfig(start_offset_frac=0.0, start_heading_deg=0.0, wind_speed=0.0, gust_frac=0.0)
        mine_law = make_law(law, veh.r_min)
        res = simulate(path, veh, mine_law, cfg, seed=1)
        other = indep_simulate(path, veh.a_max, veh.airspeed, veh.tau, indep_from(law, mine_law), cfg.dt)
        assert len(res.x) == len(other["x"])
        assert np.abs(res.x - other["x"]).max() < 1e-6
        assert np.abs(res.y - other["y"]).max() < 1e-6
        assert np.abs(res.a_cmd - other["a_cmd"]).max() < 1e-6

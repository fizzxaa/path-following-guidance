import math
import numpy as np
import pytest

from ctrack.guidance import LAW_NAMES, make_law
from ctrack.route import build_route, standard_route
from ctrack.sim import SimConfig, simulate
from ctrack.metrics import summarize
from ctrack.vehicles import QUADCOPTER, FIXED_WING


def straight_path():
    return build_route([(0, 0), (500, 0)], rho=10.0)


@pytest.mark.parametrize("name", LAW_NAMES)
def test_zero_command_when_on_a_straight_path(name):
    path = straight_path()
    law = make_law(name, r_nom=12.5)
    law.reset(path, 100.0, 0.0)
    a = law.command(path, 100.0, 0.0, 5.0, 0.0)
    assert abs(a) < 1e-6


@pytest.mark.parametrize("name", LAW_NAMES)
def test_steers_back_toward_path(name):
    """Left of an east-bound path -> steer right (negative); right of it -> steer left."""
    path = straight_path()
    law = make_law(name, r_nom=12.5)
    law.reset(path, 100.0, 6.0)
    assert law.command(path, 100.0, 6.0, 5.0, 0.0) < 0
    law2 = make_law(name, r_nom=12.5)
    law2.reset(path, 100.0, -6.0)
    assert law2.command(path, 100.0, -6.0, 5.0, 0.0) > 0


@pytest.mark.parametrize("name", LAW_NAMES)
def test_progress_index_never_goes_backwards(name):
    path = build_route(standard_route(12.5), 12.5)
    law = make_law(name, 12.5)
    law.reset(path, path.x[0], path.y[0])
    last = 0
    for i in range(0, path.n, 7):
        law.command(path, path.x[i], path.y[i] + 0.5, 5 * math.cos(path.psi[i]), 5 * math.sin(path.psi[i]))
        assert law.idx >= last
        last = law.idx


def test_route_curvature_respects_planned_radius():
    path = build_route(standard_route(12.5), 15.0)
    assert np.max(np.abs(path.kappa)) <= 1 / 15.0 + 1e-12


@pytest.mark.parametrize("veh", [QUADCOPTER, FIXED_WING])
@pytest.mark.parametrize("name", LAW_NAMES)
def test_no_wind_run_finishes_and_stays_close(veh, name):
    path = build_route(standard_route(veh.r_min), 1.5 * veh.r_min)
    res = simulate(path, veh, make_law(name, veh.r_min), SimConfig(), seed=1)
    s = summarize(path, veh, res)
    assert s["completed"]
    assert s["rms_cte_over_rmin"] < 0.1     # within a tenth of a turning radius


def test_same_seed_gives_same_result():
    veh = QUADCOPTER
    path = build_route(standard_route(veh.r_min), veh.r_min)
    cfg = SimConfig(wind_speed=1.5)
    a = simulate(path, veh, make_law("l1", veh.r_min), cfg, seed=7)
    b = simulate(path, veh, make_law("l1", veh.r_min), cfg, seed=7)
    assert np.array_equal(a.x, b.x) and np.array_equal(a.y, b.y)


def test_saturation_is_applied():
    veh = QUADCOPTER
    path = build_route(standard_route(veh.r_min), veh.r_min)
    cfg = SimConfig(limit_mismatch=0.5)
    res = simulate(path, veh, make_law("vector_field", veh.r_min), cfg, seed=0)
    assert np.max(np.abs(res.a_sat)) <= 0.5 * veh.a_max + 1e-9


# ---------------- lead vector field ----------------

def test_lead_law_with_zero_lead_equals_the_plain_vector_field():
    path = build_route(standard_route(12.5), 12.5)
    plain, lead = make_law("vector_field", 12.5), make_law("lead_vf", 12.5, {"lead_time": 0.0})
    plain.reset(path, path.x[0], path.y[0]); lead.reset(path, path.x[0], path.y[0])
    for i in range(0, path.n, 37):
        args = (path, path.x[i] + 0.7, path.y[i] - 0.4, 5 * math.cos(path.psi[i]), 5 * math.sin(path.psi[i]))
        assert plain.command(*args) == pytest.approx(lead.command(*args), abs=1e-12)


def test_lead_law_starts_turning_before_the_arc_begins():
    """On the path, a second before a turn: the lead law already commands the turn, the plain one does not."""
    path = build_route([(0, 0), (80, 0), (80, 80)], rho=12.5)
    i_arc = int(np.argmax(np.abs(path.kappa) > 0))
    i = i_arc - int(0.5 / 0.5 * 5 / 1.0)          # about 5 m (1 s at 5 m/s) before the arc
    args = (path, path.x[i], path.y[i], 5 * math.cos(path.psi[i]), 5 * math.sin(path.psi[i]))
    plain, lead = make_law("vector_field", 12.5), make_law("lead_vf", 12.5, {"lead_time": 1.0})
    plain.reset(path, path.x[i], path.y[i]); lead.reset(path, path.x[i], path.y[i])
    assert abs(plain.command(*args)) < 1e-6
    assert abs(lead.command(*args)) > 0.5 * 25.0 / 12.5     # a good part of the arc's V^2/rho


def test_lead_law_rejects_a_negative_lead():
    with pytest.raises(ValueError):
        make_law("lead_vf", 12.5, {"lead_time": -0.1})

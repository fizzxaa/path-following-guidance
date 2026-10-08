"""Known-answer tests: results that must hold from theory, independent of the tuned numbers.

Each expected value is derived by hand (see the comment above each test), not read off the code.
"""
import math

import numpy as np
import pytest

from ctrack.exact_cte import exact_cross_track_errors
from ctrack.guidance import ALL_LAW_NAMES, Guidance, make_law
from ctrack.route import Path, build_route
from ctrack.sim import SimConfig, simulate
from ctrack.vehicles import QUADCOPTER, VehicleParams

LAWS = list(ALL_LAW_NAMES) + ["carrot_tangent"]
CARROT = ("carrot_chasing", "carrot_tangent")
CIRCLE_EXACT = [l for l in LAWS if l not in CARROT]   # laws that are exact on a circle by construction
QUAD = QUADCOPTER


def line_path(length, ds=0.5, heading=0.0):
    s = np.arange(0.0, length + ds, ds)
    return Path(s * math.cos(heading), s * math.sin(heading), np.full_like(s, heading),
                np.zeros_like(s), s, rho=QUAD.r_min)


def circle_path(R, laps, ds=0.25):
    """Counter-clockwise circle of radius R starting at (R, 0) heading +y."""
    s = np.arange(0.0, 2 * math.pi * R * laps + ds, ds)
    th = s / R
    return Path(R * np.cos(th), R * np.sin(th), th + math.pi / 2, np.full_like(s, 1.0 / R), s,
                rho=R)


def ideal_cfg(**kw):
    base = dict(start_offset_frac=0.0, start_heading_deg=0.0, wind_speed=0.0, gust_frac=0.0)
    base.update(kw)
    return SimConfig(**base)


class ConstLaw(Guidance):
    name = "const"

    def __init__(self, a):
        super().__init__()
        self.a = a

    def command(self, path, x, y, vx, vy):
        return self.a


# --------------------------------------------------------------------------- simulator physics
def test_sim_lag_step_response_matches_closed_form():
    # a_act(t) = a (1 - exp(-t/tau)). Recover a_act from the heading rate: psi' = a_act / V.
    a, V, tau = 1.0, QUAD.airspeed, QUAD.tau
    res = simulate(line_path(2000), QUAD, ConstLaw(a), ideal_cfg(dt=0.005), seed=0)
    psi = np.unwrap(np.arctan2(np.gradient(res.y, res.t), np.gradient(res.x, res.t)))
    a_act = np.gradient(psi, res.t) * V
    for t_chk in (0.15, 0.3, 0.6, 1.2):
        k = int(round(t_chk / 0.005))
        assert a_act[k] == pytest.approx(a * (1 - math.exp(-t_chk / tau)), abs=0.02)


@pytest.mark.parametrize("a", [0.5, 1.0, 1.8])
def test_sim_steady_turn_radius_is_V2_over_a(a):
    # With a constant lateral acceleration a at speed V, the steady turn radius is V^2 / a.
    res = simulate(circle_path(1000, 1), QUAD, ConstLaw(a), ideal_cfg(dt=0.01), seed=0)
    k0 = int(8.0 / 0.01)                       # after the lag has died out
    x, y = res.x[k0:k0 + 600], res.y[k0:k0 + 600]
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]   # algebraic circle fit
    c = np.linalg.lstsq(A, x ** 2 + y ** 2, rcond=None)[0]
    r = math.sqrt(c[2] + c[0] ** 2 + c[1] ** 2)
    assert r == pytest.approx(QUAD.airspeed ** 2 / a, rel=0.01)


def test_sim_saturation_clips_at_a_max():
    res = simulate(line_path(500), QUAD, ConstLaw(10.0), ideal_cfg(), seed=0)
    assert np.max(res.a_sat) == pytest.approx(QUAD.a_max)
    assert np.max(res.a_cmd) == pytest.approx(10.0)


def test_sim_wind_moves_ground_track_by_wind_vector():
    # zero command, heading along +x, wind 1 m/s along +y at all times (no gusts): after T the
    # vehicle is displaced by exactly V*T along x and 1*T along y.
    class Cfg(SimConfig):
        pass
    cfg = ideal_cfg(wind_speed=1.0)
    res = simulate(line_path(2000), QUAD, ConstLaw(0.0), cfg, seed=3)
    wd_vec = np.array([math.cos(res.wind_dir), math.sin(res.wind_dir)])
    T = res.t[-1]
    disp = np.array([res.x[-1] - res.x[0], res.y[-1] - res.y[0]])
    assert disp == pytest.approx(np.array([QUAD.airspeed * T, 0.0]) + wd_vec * T, abs=0.5)


# ----------------------------------------------------------------------- straight line, ideal
@pytest.mark.parametrize("law", LAWS)
def test_straight_line_converges_to_zero_error(law):
    path = line_path(600)
    cfg = ideal_cfg(start_offset_frac=0.3, start_heading_deg=10.0)   # offset 3.75 m, heading error
    res = simulate(path, QUAD, make_law(law, QUAD.r_min), cfg, seed=5)
    cte = exact_cross_track_errors(path, res.x, res.y)
    tail = cte[res.t >= res.t[-1] - 10.0]
    assert tail.max() < 0.05, f"{law}: still {tail.max():.3f} m off after settling"


@pytest.mark.parametrize("law", LAWS)
def test_on_path_straight_gives_zero_command(law):
    path = line_path(300)
    g = make_law(law, QUAD.r_min)
    g.reset(path, 50.0, 0.0)
    assert g.command(path, 50.0, 0.0, 5.0, 0.0) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("law", LAWS)
def test_command_sign_pushes_back_toward_path(law):
    path = line_path(300)
    g = make_law(law, QUAD.r_min)
    g.reset(path, 50.0, 3.0)
    a_left_of_path = g.command(path, 50.0, 3.0, 5.0, 0.0)    # vehicle left (+y) of an +x path
    assert a_left_of_path < 0, "must turn right, back toward the path"
    g2 = make_law(law, QUAD.r_min)
    g2.reset(path, 50.0, -3.0)
    assert g2.command(path, 50.0, -3.0, 5.0, 0.0) > 0


# ------------------------------------------------------------------------- circle, closed form
# Pure pursuit / L1 on the path circle (radius R), zero error: the aim point is at arc angle phi,
# chord d = 2R sin(phi/2), eta = phi/2, so a = 2V^2 sin(eta)/d = V^2/R exactly. For L1 off the
# circle on a concentric circle of radius r, solving a = V^2/r gives r = R: the only steady state
# is zero error. The vector field with feed-forward gives a = V^2 kappa = V^2/R at e = 0.
@pytest.mark.parametrize("law", CIRCLE_EXACT)
def test_circle_command_at_zero_error_is_V2_over_R(law):
    R = 25.0
    path = circle_path(R, 2)
    g = make_law(law, QUAD.r_min)
    V = QUAD.airspeed
    g.reset(path, R, 0.0)
    a = g.command(path, R, 0.0, 0.0, V)
    assert a == pytest.approx(V ** 2 / R, rel=0.02)


@pytest.mark.parametrize("law", CIRCLE_EXACT)
def test_circle_steady_state_error_is_zero(law):
    R = 25.0                                   # 2 x r_min, so no saturation
    path = circle_path(R, 4)
    cfg = ideal_cfg()
    res = simulate(path, QUAD, make_law(law, QUAD.r_min), cfg, seed=0)
    r = np.hypot(res.x, res.y)
    tail = r[res.t >= res.t[-1] - 20.0]
    assert np.abs(tail - R).max() < 0.15, f"{law}: radial error {np.abs(tail - R).max():.3f} m"


# --------------------------------------------------------------------------- invariances
def _route(scale=1.0, rot=0.0, mirror=False):
    base = np.array([[0, 0], [60, 20], [110, -10], [150, 40], [200, 30]], float)
    w = base * scale
    if mirror:
        w = w * np.array([1.0, -1.0])
    c, s = math.cos(rot), math.sin(rot)
    w = w @ np.array([[c, s], [-s, c]])
    return build_route(w, QUAD.r_min * 1.5 * scale, ds=0.5 * scale)


@pytest.mark.parametrize("law", LAWS)
def test_scale_invariance(law):
    # Lengths and speeds x k, times unchanged => accelerations x k. Every length in a time-based
    # law scales, so the trajectory must scale exactly.
    k = 2.0
    v1 = QUAD
    v2 = VehicleParams("q2", QUAD.airspeed * k, QUAD.a_max * k, QUAD.tau)
    cfg1 = SimConfig(wind_speed=0.3 * v1.airspeed, limit_mismatch=0.9)
    cfg2 = SimConfig(wind_speed=0.3 * v2.airspeed, limit_mismatch=0.9)
    r1 = simulate(_route(1.0), v1, make_law(law, v1.r_min), cfg1, seed=11)
    r2 = simulate(_route(k), v2, make_law(law, v2.r_min), cfg2, seed=11)
    n = min(len(r1.x), len(r2.x))
    assert len(r1.x) == len(r2.x)
    assert np.abs(r2.x[:n] - k * r1.x[:n]).max() < 1e-6 * k * 200
    assert np.abs(r2.y[:n] - k * r1.y[:n]).max() < 1e-6 * k * 200


@pytest.mark.parametrize("law", LAWS)
def test_rotation_invariance(law):
    cfg = ideal_cfg()
    th = 0.7
    r1 = simulate(_route(), QUAD, make_law(law, QUAD.r_min), cfg, seed=1)
    r2 = simulate(_route(rot=th), QUAD, make_law(law, QUAD.r_min), cfg, seed=1)
    c, s = math.cos(th), math.sin(th)
    xr = c * r1.x - s * r1.y            # rotate trajectory 1 by th
    yr = s * r1.x + c * r1.y
    # _route rotates with the transpose convention; accept either rotation sense
    err_a = max(np.abs(r2.x - xr).max(), np.abs(r2.y - yr).max())
    xr2 = c * r1.x + s * r1.y
    yr2 = -s * r1.x + c * r1.y
    err_b = max(np.abs(r2.x - xr2).max(), np.abs(r2.y - yr2).max())
    assert min(err_a, err_b) < 1e-6


@pytest.mark.parametrize("law", LAWS)
def test_mirror_invariance(law):
    cfg = ideal_cfg()
    r1 = simulate(_route(), QUAD, make_law(law, QUAD.r_min), cfg, seed=1)
    r2 = simulate(_route(mirror=True), QUAD, make_law(law, QUAD.r_min), cfg, seed=1)
    assert np.abs(r2.x - r1.x).max() < 1e-6
    assert np.abs(r2.y + r1.y).max() < 1e-6


# ------------------------------------------------------------------------ step-size convergence
@pytest.mark.parametrize("law", LAWS)
def test_timestep_convergence(law):
    # Euler integration: halving dt must change the error by only a few percent.
    path = _route()
    rms = {}
    for dt in (0.05, 0.025, 0.0125):
        cfg = SimConfig(dt=dt, wind_speed=0.0, start_offset_frac=0.0, start_heading_deg=0.0)
        res = simulate(path, QUAD, make_law(law, QUAD.r_min), cfg, seed=2)
        skip = res.t >= 4 * QUAD.r_min / QUAD.airspeed
        rms[dt] = float(np.sqrt(np.mean(exact_cross_track_errors(path, res.x[skip], res.y[skip]) ** 2)))
    d1 = abs(rms[0.05] - rms[0.0125])
    assert d1 / max(rms[0.0125], 0.05) < 0.10, f"{law}: {rms}"


def test_exact_cte_floor_is_removed():
    from ctrack.metrics import cross_track_errors
    path = build_route([[0, 0], [100, 40], [200, 0]], 20.0)
    mid = 0.5 * (path.x[:-1] + path.x[1:]), 0.5 * (path.y[:-1] + path.y[1:])
    assert cross_track_errors(path, *mid).max() > 0.1
    assert exact_cross_track_errors(path, *mid).max() < 0.01


# Carrot chasing steers a = vg * kappa * eta (angle, not sin(eta)/chord), so it is NOT exact on a
# circle in general. Arc mode, vehicle on the circle: carrot at arc angle delta/R, chord angle
# eta = delta/(2R) exactly, so a0 = kappa * delta_time * V^2 / (2R) = (kappa*delta_time/2) * V^2/R:
# exact only when kappa*delta_time = 2. Tangent mode: the line to the carrot IS the tangent, so
# eta = 0 and the command at zero error is 0 (the original definition cannot turn without error).
@pytest.mark.parametrize("kappa,dt_", [(1.0, 2.0), (0.5, 2.0), (2.0, 2.0), (1.0, 3.0)])
def test_carrot_arc_command_at_zero_error_closed_form(kappa, dt_):
    R, V = 25.0, QUAD.airspeed
    path = circle_path(R, 2, ds=0.05)
    g = make_law("carrot_chasing", QUAD.r_min, {"kappa": kappa, "delta_time": dt_})
    g.reset(path, R, 0.0)
    assert g.command(path, R, 0.0, 0.0, V) == pytest.approx(kappa * dt_ / 2 * V ** 2 / R, rel=0.01)


def test_carrot_tangent_command_at_zero_error_is_zero():
    R, V = 25.0, QUAD.airspeed
    path = circle_path(R, 2, ds=0.05)
    g = make_law("carrot_tangent", QUAD.r_min)
    g.reset(path, R, 0.0)
    assert g.command(path, R, 0.0, 0.0, V) == pytest.approx(0.0, abs=1e-6)


def test_carrot_tangent_has_standing_outward_offset_on_a_circle_arc_does_not():
    R = 25.0
    path = circle_path(R, 4)
    out = {}
    for name in CARROT:
        res = simulate(path, QUAD, make_law(name, QUAD.r_min), ideal_cfg(), seed=0)
        r = np.hypot(res.x, res.y)[res.t >= res.t[-1] - 20.0]
        out[name] = r.mean() - R
    assert out["carrot_tangent"] > 1.0          # measured about +1.9 m
    assert abs(out["carrot_chasing"]) < 0.1     # default gains give kappa*delta_time = 2 (exact)

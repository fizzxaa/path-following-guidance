"""Compare the repo's laws with published formulas, on random states.

ArduPilot AP_L1_Control::update_waypoint (libraries/AP_L1_Control/AP_L1_Control.cpp), as
transcribed on 2026-10-05 from the master branch via a fetch summary (not byte-verified; re-check
against the source before citing):

    L1_dist  = max(0.3183099 * zeta * period * Vg, dist_min)
    xtrackVel = v x AB ; ltrackVel = v . AB            (AB = unit vector of the segment)
    Nu2 = atan2(xtrackVel, ltrackVel)
    sine_Nu1 = clamp(crosstrack_error / max(L1_dist, 0.1), -0.7071, 0.7071)
    Nu = asin(sine_Nu1) + Nu2
    latAccDem = 4 zeta^2 * Vg^2 / L1_dist * sin(Nu)

With zeta = 1/sqrt(2), 4 zeta^2 = 2, which is the Park/Deyst/How gain 2 V^2 sin(eta) / L1 used by
ctrack.guidance.L1. The two should then agree on a straight segment, apart from (a) the repo
picking the aim point from path samples, and (b) ArduPilot clamping |sin Nu1| at 0.7071.
"""
import math

import numpy as np
import pytest

from ctrack.guidance import L1, PurePursuit
from ctrack.route import Path


def ardupilot_l1_lat_acc(pos, vel, A, B, zeta, period):
    """ArduPilot's lateral acceleration demand, evaluated with the formula's own cross products in a
    right-handed x-y frame. ArduPilot works in NED (north, east), where the same cross products give
    the mirror sign, so here positive = turn left, the same as the repo. The sign was checked by hand
    on one case (vehicle right of the path, heading away from it: both give a large left turn)."""
    AB = np.array(B) - np.array(A)
    AB = AB / np.linalg.norm(AB)
    vg = float(np.hypot(*vel))
    L1_dist = 0.3183099 * zeta * period * vg
    A_air = np.array(pos) - np.array(A)
    cross = lambda u, v: u[0] * v[1] - u[1] * v[0]
    crosstrack_error = cross(A_air, AB)                     # ArduPilot: A_air % AB
    xtrack_vel = cross(np.array(vel), AB)                   # v % AB
    ltrack_vel = float(np.dot(vel, AB))
    nu2 = math.atan2(xtrack_vel, ltrack_vel)
    sine_nu1 = max(-0.7071, min(0.7071, crosstrack_error / max(L1_dist, 0.1)))
    nu = math.asin(sine_nu1) + nu2
    nu = max(-1.5708, min(1.5708, nu))
    return 4 * zeta ** 2 * vg ** 2 / L1_dist * math.sin(nu), L1_dist


def fine_line(length=400.0, ds=0.02):
    s = np.arange(0.0, length + ds, ds)
    return Path(s.copy(), np.zeros_like(s), np.zeros_like(s), np.zeros_like(s), s, rho=10.0)


def test_repo_l1_matches_ardupilot_on_straight_segment():
    rng = np.random.default_rng(0)
    path = fine_line()
    zeta, period = 1 / math.sqrt(2), 11.0
    worst = 0.0
    for _ in range(300):
        vg = rng.uniform(3.0, 20.0)
        _, L1d = ardupilot_l1_lat_acc((0, 0), (vg, 0), (0, 0), (1, 0), zeta, period)
        e = rng.uniform(-0.6, 0.6) * L1d                    # keep inside the 0.7071 clamp
        head = rng.uniform(-math.radians(50), math.radians(50))
        pos = (rng.uniform(20, 150), e)
        vel = (vg * math.cos(head), vg * math.sin(head))
        ap, L1d = ardupilot_l1_lat_acc(pos, vel, (0, 0), (1, 0), zeta, period)
        law = L1(lookahead_time=0.3183099 * zeta * period, min_lookahead=0.0, search_window=100_000)
        law.reset(path, *pos)
        mine = law.command(path, pos[0], pos[1], *vel)       # left = +
        assert mine == pytest.approx(ap, rel=0.02, abs=0.01), (pos, vel, mine, ap)
        worst = max(worst, abs(mine - ap))
    assert worst < 0.05


def test_ardupilot_gain_equals_two_only_for_zeta_inv_sqrt2():
    assert 4 * (1 / math.sqrt(2)) ** 2 == pytest.approx(2.0)
    assert 4 * 0.75 ** 2 == pytest.approx(2.25)        # ArduPilot default damping 0.75


def test_pure_pursuit_equals_l1_on_a_straight_line_for_small_errors():
    path = fine_line()
    pp = PurePursuit(lookahead_time=2.5, min_lookahead=0.0, search_window=100_000)
    l1 = L1(lookahead_time=2.5, min_lookahead=0.0, search_window=100_000)
    pos, vel = (50.0, 1.0), (5.0, 0.2)
    pp.reset(path, *pos); l1.reset(path, *pos)
    a, b = pp.command(path, *pos, *vel), l1.command(path, *pos, *vel)
    assert a == pytest.approx(b, rel=0.05)

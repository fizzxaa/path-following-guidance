"""Check the SITL control step against a fake copter that speaks MAVLink NED.

The fake copter follows velocity commands with a lag and an acceleration limit. This
does not prove the real ArduCopter behaves the same, but it does catch frame mix-ups
(north/east swaps, wrong signs), which are the most likely bugs in a first SITL run.
"""
import math
import numpy as np
import pytest

from ctrack.guidance import LAW_NAMES, make_law
from ctrack.metrics import cross_track_errors
from ctrack.route import build_route, standard_route
from ctrack.sitl_control import finished, velocity_command
from ctrack.vehicles import QUADCOPTER


class FakeCopterNED:
    """State in NED like LOCAL_POSITION_NED: x north, y east, vx north, vy east."""

    def __init__(self, north, east, tau=0.5, a_max=2.0):
        self.x, self.y, self.vx, self.vy = north, east, 0.0, 0.0
        self.tau, self.a_max = tau, a_max

    def message(self):
        return dict(x=self.x, y=self.y, vx=self.vx, vy=self.vy)

    def step(self, vn_cmd, ve_cmd, dt):
        ax = max(-self.a_max, min(self.a_max, (vn_cmd - self.vx) / self.tau))
        ay = max(-self.a_max, min(self.a_max, (ve_cmd - self.vy) / self.tau))
        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt


@pytest.mark.parametrize("law_name", LAW_NAMES)
def test_ned_conversions_track_the_path(law_name):
    veh = QUADCOPTER
    path = build_route(standard_route(veh.r_min), 2.0 * veh.r_min)   # gentle: needs ~1 m/s^2
    path.x = path.x + 30.0      # start somewhere other than the origin, like the real copter
    path.y = path.y - 20.0
    law = make_law(law_name, veh.r_min)
    law.reset(path, path.x[0], path.y[0])
    cop = FakeCopterNED(north=path.y[0], east=path.x[0])     # NED: north = y, east = x
    xs, ys, dt = [], [], 0.1
    ok = False
    for _ in range(int(4 * path.length / 5.0 / dt)):
        m = cop.message()
        east, north, ve, vn = m["y"], m["x"], m["vy"], m["vx"]      # NED -> repo frame
        vn_cmd, ve_cmd, _ = velocity_command(law, path, east, north, ve, vn, speed=5.0, horizon=0.5)
        xs.append(east); ys.append(north)
        if finished(law, path, east, north, 0.4 * veh.r_min):
            ok = True
            break
        cop.step(vn_cmd, ve_cmd, dt)
    assert ok, "did not reach the end of the path"
    cte = cross_track_errors(path, np.array(xs), np.array(ys))
    assert np.sqrt(np.mean(cte ** 2)) < 2.0       # under 2 m RMS on a 12.5 m minimum radius


def test_a_swapped_north_east_would_be_caught():
    """Guard against the classic bug: if x/y were swapped, tracking must fail."""
    veh = QUADCOPTER
    path = build_route(standard_route(veh.r_min), 2.0 * veh.r_min)
    law = make_law("l1", veh.r_min)
    law.reset(path, path.x[0], path.y[0])
    cop = FakeCopterNED(north=path.y[0], east=path.x[0])
    worst = 0.0
    for _ in range(600):
        m = cop.message()
        # WRONG on purpose: treat north as east
        vn_cmd, ve_cmd, _ = velocity_command(law, path, m["x"], m["y"], m["vx"], m["vy"], 5.0, 0.5)
        cop.step(vn_cmd, ve_cmd, 0.1)
        worst = max(worst, float(cross_track_errors(path, np.array([cop.y]), np.array([cop.x]))[0]))
    assert worst > 10.0


def test_slow_start_does_not_divide_by_zero():
    veh = QUADCOPTER
    path = build_route(standard_route(veh.r_min), 2.0 * veh.r_min)
    law = make_law("vector_field", veh.r_min)
    law.reset(path, path.x[0], path.y[0])
    vn, ve, a = velocity_command(law, path, path.x[0], path.y[0], 0.0, 0.0, 5.0, 0.5)
    assert math.isfinite(vn) and math.isfinite(ve) and math.isfinite(a)
    assert abs(math.hypot(vn, ve) - 5.0) < 1e-9


def test_saved_log_can_be_plotted(tmp_path):
    """Round trip: fake flight -> save_run -> plot_sitl, so the two file formats agree."""
    from ctrack.plot_sitl import main as plot_main
    from ctrack.sitl_control import save_run

    veh = QUADCOPTER
    outs = []
    for name in ("pure_pursuit", "vector_field"):
        path = build_route(standard_route(veh.r_min), 2.0 * veh.r_min)
        path.x = path.x + 5.0
        law = make_law(name, veh.r_min)
        law.reset(path, path.x[0], path.y[0])
        cop = FakeCopterNED(north=path.y[0], east=path.x[0])
        rows, t = [], 0.0
        for _ in range(2000):
            m = cop.message()
            vn_cmd, ve_cmd, a = velocity_command(law, path, m["y"], m["x"], m["vy"], m["vx"], 5.0, 0.5)
            rows.append((t, m["y"], m["x"], m["vy"], m["vx"], a, law.idx))
            if finished(law, path, m["y"], m["x"], 5.0):
                break
            cop.step(vn_cmd, ve_cmd, 0.1)
            t += 0.1
        s = save_run(str(tmp_path / f"{name}.csv"), rows, path, veh.r_min, 5.0)
        assert s["rms_cte_m"] < 2.0
        outs.append(str(tmp_path / f"{name}.csv"))
    plot_main(outs + ["--out", str(tmp_path / "p.png"), "--gif", str(tmp_path / "r.gif"), "--speedup", "60"])
    assert (tmp_path / "p.png").exists()
    assert (tmp_path / "r.gif").stat().st_size > 1000

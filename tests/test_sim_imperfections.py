import math
import numpy as np
import pytest

from ctrack.guidance import Guidance
from ctrack.route import build_route
from ctrack.sim import SimConfig, simulate
from ctrack.vehicles import QUADCOPTER as veh


class Probe(Guidance):
    """Records what the guidance law is shown and commands a fixed lateral acceleration."""
    name = "probe"

    def __init__(self, a=0.0):
        super().__init__()
        self.seen, self.a = [], a

    def command(self, path, x, y, vx, vy):
        self.seen.append((x, y, vx, vy))
        return self.a


def _path():
    return build_route([(0, 0), (2000, 0)], rho=veh.r_min)


def _cfg(**kw):
    return SimConfig(start_offset_frac=0.0, start_heading_deg=0.0, t_max_factor=0.05, **kw)


def test_sensor_delay_shows_the_law_an_older_state():
    probe = Probe(a=1.0)                                  # keep it turning so the state changes
    res = simulate(_path(), veh, probe, _cfg(sensor_delay=1.0), seed=0)
    seen = np.array(probe.seen)
    d = int(round(1.0 / 0.05))
    k = np.arange(len(seen))
    ref = np.maximum(0, k - d)
    assert np.allclose(seen[:, 0], res.x[ref]) and np.allclose(seen[:, 1], res.y[ref])


def test_position_noise_has_the_requested_size_and_does_not_touch_the_true_motion():
    clean = simulate(_path(), veh, Probe(1.0), _cfg(), seed=3)
    probe = Probe(1.0)
    noisy = simulate(_path(), veh, probe, _cfg(pos_noise_frac=0.04), seed=3)
    # the vehicle's true track is identical: noise only changes what the law sees (a=const here)
    assert np.allclose(clean.x, noisy.x) and np.allclose(clean.y, noisy.y)
    seen = np.array(probe.seen)
    err = np.concatenate([seen[:, 0] - noisy.x, seen[:, 1] - noisy.y])
    assert np.std(err) == pytest.approx(0.04 * veh.r_min, rel=0.15)


def test_noise_off_gives_identical_runs_to_before():
    a = simulate(_path(), veh, Probe(0.5), _cfg(), seed=5)
    b = simulate(_path(), veh, Probe(0.5), _cfg(pos_noise_frac=0.0, vel_noise_frac=0.0, sensor_delay=0.0,
                                                lag_scale=1.0, airspeed_scale=1.0), seed=5)
    assert np.array_equal(a.x, b.x) and np.array_equal(a.y, b.y)


def test_a_slower_true_response_turns_less_in_the_first_seconds():
    fast = simulate(_path(), veh, Probe(1.5), _cfg(lag_scale=1.0), seed=0)
    slow = simulate(_path(), veh, Probe(1.5), _cfg(lag_scale=4.0), seed=0)
    n = int(2.0 / 0.05)
    assert abs(fast.y[n]) > abs(slow.y[n]) > 0


def test_airspeed_scale_changes_the_distance_flown():
    slow = simulate(_path(), veh, Probe(0.0), _cfg(airspeed_scale=0.8), seed=0)
    norm = simulate(_path(), veh, Probe(0.0), _cfg(), seed=0)
    n = min(len(slow.x), len(norm.x)) - 1
    assert slow.x[n] == pytest.approx(0.8 * norm.x[n], rel=1e-6)

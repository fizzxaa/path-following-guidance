import numpy as np
import pytest

from ctrack.estimate_lag import estimate, fit, lateral_acceleration, low_pass, shift


def _synthetic_log(path, gain, tau, delay, v=5.0, seed=0, noise=0.0):
    rng = np.random.default_rng(seed)
    t = np.cumsum(rng.uniform(0.08, 0.12, 1500))                  # uneven ~10 Hz sampling, like SITL
    a_cmd = 0.8 * np.sin(0.4 * t) + 0.5 * np.sin(1.3 * t + 1.0) + 0.3 * np.sin(2.9 * t + 0.3)
    a_true = gain * shift(low_pass(a_cmd, t, tau), t, delay)
    chi = np.concatenate([[0.0], np.cumsum(0.5 * (a_true[1:] + a_true[:-1]) / v * np.diff(t))])
    v_e, v_n = v * np.cos(chi), v * np.sin(chi)
    if noise:
        v_e = v_e + rng.normal(0, noise, len(t)); v_n = v_n + rng.normal(0, noise, len(t))
    np.savetxt(path, np.column_stack([t, np.zeros_like(t), np.zeros_like(t), v_e, v_n, a_cmd, np.zeros_like(t)]),
               delimiter=",", comments="", header="t_s,x_east,y_north,v_east,v_north,a_cmd,path_idx")


@pytest.mark.parametrize("gain,tau,delay", [(0.8, 0.4, 0.2), (1.0, 0.3, 0.0), (0.6, 0.6, 0.3)])
def test_recovers_the_known_response(tmp_path, gain, tau, delay):
    p = tmp_path / "log.csv"
    _synthetic_log(p, gain, tau, delay)
    r = estimate(str(p))
    assert r["gain"] == pytest.approx(gain, rel=0.12)
    assert r["lag"] == pytest.approx(tau + delay, abs=0.12)      # only the SUM is well determined
    assert r["r2"] > 0.9


def test_still_close_with_measurement_noise(tmp_path):
    p = tmp_path / "log.csv"
    _synthetic_log(p, 0.8, 0.4, 0.2, noise=0.03)
    r = estimate(str(p))
    assert r["gain"] == pytest.approx(0.8, rel=0.2)
    assert r["lag"] == pytest.approx(0.6, abs=0.2)


def test_lateral_acceleration_of_a_circle():
    v, R = 5.0, 12.5
    t = np.linspace(0, 20, 400)
    chi = v / R * t
    a = lateral_acceleration(t, v * np.cos(chi), v * np.sin(chi))
    assert np.allclose(a[5:-5], v * v / R, rtol=1e-3)


def test_a_command_the_copter_ignores_gives_a_low_r2(tmp_path):
    rng = np.random.default_rng(1)
    t = np.arange(0, 100, 0.1)
    a_cmd = rng.normal(0, 1, len(t))
    chi = np.cumsum(rng.normal(0, 0.01, len(t)))               # motion unrelated to the command
    np.savetxt(tmp_path / "x.csv", np.column_stack([t, 0 * t, 0 * t, 5 * np.cos(chi), 5 * np.sin(chi), a_cmd, 0 * t]),
               delimiter=",", comments="", header="t_s,x_east,y_north,v_east,v_north,a_cmd,path_idx")
    assert estimate(str(tmp_path / "x.csv"))["r2"] < 0.3

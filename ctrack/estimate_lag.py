"""How does the real copter respond to the guidance command? Estimate it from a SITL log.

    python -m ctrack.estimate_lag results/sitl_l1_b.csv results/sitl_vector_field.csv ...

The runner logs the acceleration the law asked for (a_cmd) and the measured velocity. From the
velocity we get the sideways acceleration the copter actually produced:

    a_meas = ground_speed * d(course)/dt

and we fit   a_meas(t) = gain * LowPass_tau[ a_cmd ](t - delay).

  gain   : how much of the command the copter turns into real turning (1 = all of it).
           NOTE: the runner turns a_cmd into a velocity command with a gain of its own
           (the --horizon setting), so this "gain" describes command-to-motion overall.
  tau    : first-order time constant (s)
  delay  : pure delay (s)
  lag    : tau + delay, the number to compare with the best lead time of lead_vf.
           tau and delay trade off against each other; only their sum is well determined.
"""
from __future__ import annotations

import argparse
import os

import numpy as np


def load(csv_path):
    d = np.genfromtxt(csv_path, delimiter=",", names=True)
    return d["t_s"], d["v_east"], d["v_north"], d["a_cmd"]


def lateral_acceleration(t, v_e, v_n):
    chi = np.unwrap(np.arctan2(v_n, v_e))
    return np.hypot(v_e, v_n) * np.gradient(chi, t)


def low_pass(u, t, tau):
    """First-order low pass on an unevenly sampled signal (exact for piecewise-constant input)."""
    y = np.empty_like(u)
    y[0] = u[0]
    for k in range(1, len(u)):
        a = 1.0 - np.exp(-(t[k] - t[k - 1]) / tau)
        y[k] = y[k - 1] + a * (u[k] - y[k - 1])
    return y


def shift(y, t, delay):
    return np.interp(t - delay, t, y, left=y[0])


def fit(t, a_cmd, a_meas, skip_s=10.0, taus=None, delays=None):
    taus = np.linspace(0.02, 2.0, 50) if taus is None else taus
    delays = np.linspace(0.0, 1.0, 41) if delays is None else delays
    keep = t >= skip_s
    best = None
    for tau in taus:
        lp = low_pass(a_cmd, t, tau)
        for d in delays:
            pred = shift(lp, t, d)[keep]
            m = a_meas[keep]
            denom = float(pred @ pred)
            if denom < 1e-12:
                continue
            gain = float(pred @ m) / denom
            sse = float(np.sum((m - gain * pred) ** 2))
            if best is None or sse < best[0]:
                best = (sse, gain, tau, d)
    sse, gain, tau, d = best
    m = a_meas[keep]
    r2 = 1.0 - sse / float(np.sum((m - m.mean()) ** 2))
    return {"gain": gain, "tau": float(tau), "delay": float(d), "lag": float(tau + d), "r2": r2,
            "n_samples": int(keep.sum())}


def estimate(csv_path, skip_s=10.0):
    t, v_e, v_n, a_cmd = load(csv_path)
    return fit(t, a_cmd, lateral_acceleration(t, v_e, v_n), skip_s=skip_s)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--skip", type=float, default=10.0, help="seconds at the start to ignore")
    args = ap.parse_args(argv)
    print(f"{'log':34s} {'gain':>6s} {'tau':>6s} {'delay':>6s} {'lag':>6s} {'R^2':>6s}")
    for f in args.logs:
        r = estimate(f, args.skip)
        print(f"{os.path.basename(f):34s} {r['gain']:6.2f} {r['tau']:6.2f} {r['delay']:6.2f} "
              f"{r['lag']:6.2f} {r['r2']:6.2f}")
    print("\nlag = tau + delay (s). A low R^2 (< 0.5) means the fit is not trustworthy for that log.")


if __name__ == "__main__":
    main()

"""Fast 2-D simulator: vehicle + wind + guidance law, run against one path."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .guidance import Guidance
from .route import Path
from .vehicles import VehicleParams


@dataclass
class SimConfig:
    dt: float = 0.05
    wind_speed: float = 0.0          # m/s, steady wind (direction is random per seed)
    gust_frac: float = 0.25          # gust std-dev as a fraction of wind_speed
    gust_tau: float = 5.0            # s, gust correlation time
    limit_mismatch: float = 1.0      # true a_max = mismatch * planned a_max
    start_offset_frac: float = 0.3   # random start offset, as a fraction of r_min
    start_heading_deg: float = 15.0  # random start heading error
    finish_tol_frac: float = 0.4     # finished when within this * r_min of the end
    t_max_factor: float = 4.0        # give up after this * (path length / airspeed)
    # --- imperfections (all off by default, so earlier results are unchanged) ---
    pos_noise_frac: float = 0.0      # position measurement noise, std as a fraction of r_min
    vel_noise_frac: float = 0.0      # velocity measurement noise, std as a fraction of airspeed
    sensor_delay: float = 0.0        # s, the law sees the state this old
    lag_scale: float = 1.0           # true response lag = lag_scale * the lag the vehicle is rated for
    airspeed_scale: float = 1.0      # true airspeed = scale * rated airspeed


@dataclass
class SimResult:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    a_cmd: np.ndarray       # raw command from the law
    a_sat: np.ndarray       # command after the physical limit
    completed: bool
    finish_time: float
    wind_dir: float = 0.0   # direction the steady wind blows toward (rad, from +x)


def simulate(path: Path, vehicle: VehicleParams, law: Guidance,
             cfg: SimConfig, seed: int = 0) -> SimResult:
    rng = np.random.default_rng(seed)
    rng_meas = np.random.default_rng(seed + 10_000_019)   # separate stream: noise never shifts the wind
    V = vehicle.airspeed * cfg.airspeed_scale
    tau = vehicle.tau * cfg.lag_scale
    a_true = cfg.limit_mismatch * vehicle.a_max
    d_steps = int(round(cfg.sensor_delay / cfg.dt))
    pos_sd = cfg.pos_noise_frac * vehicle.r_min
    vel_sd = cfg.vel_noise_frac * vehicle.airspeed

    # random start: small offset from the path start and a small heading error
    off = rng.uniform(-cfg.start_offset_frac, cfg.start_offset_frac) * vehicle.r_min
    h_off = math.radians(rng.uniform(-cfg.start_heading_deg, cfg.start_heading_deg))
    p0 = path.psi[0]
    x = path.x[0] - off * math.sin(p0)
    y = path.y[0] + off * math.cos(p0)
    psi = p0 + h_off

    # wind: steady part in a random direction plus a correlated gust
    wd = rng.uniform(0.0, 2.0 * math.pi)
    wx0, wy0 = cfg.wind_speed * math.cos(wd), cfg.wind_speed * math.sin(wd)
    gust = np.zeros(2)
    sigma = cfg.gust_frac * cfg.wind_speed
    a_act = 0.0

    law.reset(path, x, y)
    t_max = cfg.t_max_factor * path.length / vehicle.airspeed
    n_max = int(t_max / cfg.dt)
    end_x, end_y = path.x[-1], path.y[-1]
    tol = cfg.finish_tol_frac * vehicle.r_min

    T, X, Y, AC, AS = [], [], [], [], []
    hist = []                      # true (x, y, vx, vy) at every step, for the sensor delay
    completed = False
    finish_time = float("nan")
    for k in range(n_max):
        t = k * cfg.dt
        wx, wy = wx0 + gust[0], wy0 + gust[1]
        vx, vy = V * math.cos(psi) + wx, V * math.sin(psi) + wy

        # what the guidance law gets to see: delayed, and possibly noisy
        hist.append((x, y, vx, vy))
        mx, my, mvx, mvy = hist[max(0, k - d_steps)]
        if pos_sd > 0:
            mx += rng_meas.normal(0.0, pos_sd)
            my += rng_meas.normal(0.0, pos_sd)
        if vel_sd > 0:
            mvx += rng_meas.normal(0.0, vel_sd)
            mvy += rng_meas.normal(0.0, vel_sd)
        a_cmd = law.command(path, mx, my, mvx, mvy)
        a_sat = max(-a_true, min(a_true, a_cmd))

        T.append(t); X.append(x); Y.append(y); AC.append(a_cmd); AS.append(a_sat)

        if law.idx >= path.n - 3 and math.hypot(end_x - x, end_y - y) < tol:
            completed, finish_time = True, t
            break

        a_act += cfg.dt * (a_sat - a_act) / tau
        psi += cfg.dt * a_act / V
        x += cfg.dt * vx
        y += cfg.dt * vy
        if sigma > 0:
            gust += (-gust / cfg.gust_tau) * cfg.dt + \
                sigma * math.sqrt(2.0 * cfg.dt / cfg.gust_tau) * rng.standard_normal(2)

    return SimResult(np.array(T), np.array(X), np.array(Y),
                     np.array(AC), np.array(AS), completed, finish_time, wd)

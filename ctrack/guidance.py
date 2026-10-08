"""Three path-following (guidance) laws.

Each law looks at the vehicle's position and ground velocity and returns ONE
number: the lateral acceleration to command (m/s^2, left = +).

  PurePursuit : aim at a point a fixed arc-length ahead along the path.
  L1          : aim at the first path point that is a fixed straight-line
                distance (L1) away (Park, Deyst & How, 2004).
  VectorField : build a desired-course field that points along the path far
                from it and turns toward it when off it (Nelson et al., 2007),
                then steer the course toward that field.

  LeadVF      : the vector-field law, but its curvature feed-forward is read about one
                response-lag AHEAD along the path. Fixes most of the vector-field law's
                weakness when the plan is at the vehicle's limit.

PurePursuit and L1 use the same steering formula, a = 2 V^2 sin(eta) / d. They
differ only in how the aim point is chosen. That is how they are usually
defined, so expect them to behave alike on gentle paths.
"""
from __future__ import annotations

import math

import numpy as np

from .dubins import wrap_pi
from .route import Path


class Guidance:
    name = "base"

    def __init__(self, lookahead_time: float = 2.5, min_lookahead: float = 3.0,
                 search_window: int = 800):
        self.lookahead_time = lookahead_time
        self.min_lookahead = min_lookahead
        self.search_window = search_window
        self.idx = 0

    def reset(self, path: Path, x: float, y: float):
        """Find the starting point on the path (a full search once)."""
        d2 = (path.x - x) ** 2 + (path.y - y) ** 2
        self.idx = int(np.argmin(d2))

    def _project(self, path: Path, x: float, y: float) -> int:
        """Nearest path sample, searched only forward of the last one.

        Searching only a forward window stops the vehicle from 'jumping' to
        another part of a path that loops back near itself.
        """
        lo = max(self.idx - 5, 0)
        hi = min(self.idx + self.search_window, path.n)
        d2 = (path.x[lo:hi] - x) ** 2 + (path.y[lo:hi] - y) ** 2
        self.idx = max(self.idx, lo + int(np.argmin(d2)))
        return self.idx

    def command(self, path: Path, x: float, y: float, vx: float, vy: float) -> float:
        raise NotImplementedError


class PurePursuit(Guidance):
    name = "pure_pursuit"

    def command(self, path, x, y, vx, vy):
        i = self._project(path, x, y)
        vg = math.hypot(vx, vy)
        chi = math.atan2(vy, vx)
        ld = max(self.lookahead_time * vg, self.min_lookahead)
        j = min(int(np.searchsorted(path.s, path.s[i] + ld)), path.n - 1)
        dx, dy = path.x[j] - x, path.y[j] - y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0.0
        eta = float(wrap_pi(math.atan2(dy, dx) - chi))
        return 2.0 * vg ** 2 * math.sin(eta) / dist


class L1(Guidance):
    name = "l1"

    def command(self, path, x, y, vx, vy):
        i = self._project(path, x, y)
        vg = math.hypot(vx, vy)
        chi = math.atan2(vy, vx)
        l1 = max(self.lookahead_time * vg, self.min_lookahead)
        hi = min(i + self.search_window, path.n)
        d2 = (path.x[i:hi] - x) ** 2 + (path.y[i:hi] - y) ** 2
        far = np.nonzero(d2 >= l1 * l1)[0]
        j = i + int(far[0]) if len(far) else path.n - 1
        dx, dy = path.x[j] - x, path.y[j] - y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0.0
        eta = float(wrap_pi(math.atan2(dy, dx) - chi))
        return 2.0 * vg ** 2 * math.sin(eta) / dist


class VectorField(Guidance):
    name = "vector_field"

    def __init__(self, k_e: float, chi_inf: float = math.radians(70.0),
                 k_chi: float = 1.0, feedforward: bool = True, **kw):
        super().__init__(**kw)
        self.k_e = k_e              # 1/m: how quickly the field turns toward the path
        self.chi_inf = chi_inf      # max approach angle when far from the path
        self.k_chi = k_chi          # 1/s: course-error gain
        self.feedforward = feedforward  # add path curvature as a feed-forward term

    def command(self, path, x, y, vx, vy):
        i = self._project(path, x, y)
        vg = math.hypot(vx, vy)
        chi = math.atan2(vy, vx)
        chi_p = path.psi[i]
        # signed cross-track error: + means the vehicle is to the LEFT of the path
        e = -math.sin(chi_p) * (x - path.x[i]) + math.cos(chi_p) * (y - path.y[i])
        chi_d = chi_p - self.chi_inf * (2.0 / math.pi) * math.atan(self.k_e * e)
        a = vg * self.k_chi * float(wrap_pi(chi_d - chi))
        if self.feedforward:
            a += vg ** 2 * path.kappa[i]
        return a


class LeadVectorField(VectorField):
    """Vector-field guidance that reads the path's curvature `lead_time` seconds AHEAD.

    Why. The vehicle's sideways acceleration responds to a command with a lag (about 0.3-0.5 s).
    The plain law applies the curvature feed-forward for the point it is ON, so the vehicle
    starts turning late and the error grows during turn entry. When the plan needs all of the
    vehicle's turning ability there is no spare authority to catch up, and the error keeps
    growing while the command saturates. Reading the curvature about one lag ahead removes most
    of that.

    Found by experiment on the tuning routes (see README): a gentler field, a smaller approach
    angle and capping the feed-forward did NOT help; looking ahead in the curvature did.
    Looking ahead in the path HEADING as well did not help either, so only the feed-forward
    is shifted.
    """
    name = "lead_vf"

    def __init__(self, k_e, lead_time=0.6, **kw):
        super().__init__(k_e=k_e, **kw)
        if lead_time < 0:
            raise ValueError("lead_time must be >= 0")
        self.lead_time = lead_time

    def command(self, path, x, y, vx, vy):
        i = self._project(path, x, y)
        vg = math.hypot(vx, vy)
        chi = math.atan2(vy, vx)
        chi_p = path.psi[i]
        e = -math.sin(chi_p) * (x - path.x[i]) + math.cos(chi_p) * (y - path.y[i])
        chi_d = chi_p - self.chi_inf * (2.0 / math.pi) * math.atan(self.k_e * e)
        a = vg * self.k_chi * float(wrap_pi(chi_d - chi))
        if self.feedforward:
            j = min(int(np.searchsorted(path.s, path.s[i] + self.lead_time * vg)), path.n - 1)
            a += vg ** 2 * path.kappa[j]
        return a


class AdaptiveVectorField(VectorField):
    """Vector field whose curvature feed-forward gain and a constant turn-rate bias are ADAPTED online.

    THIS IS OUR OWN LAW "after Fari 2020 / Wang 2022 style adaptive vector fields". It is NOT a
    reproduction of either paper (their formulation was not read). Say so wherever it is reported.

    Let s = wrap(chi_d - chi) be the course error, phi = vg * kappa the path turn rate.
    Control:  a = vg * (k_chi * s + theta_hat * phi + d_hat)
    theta_hat scales the feed-forward (nominal 1: absorbs lag and an a_max mismatch);
    d_hat is a constant turn-rate bias (absorbs a standing disturbance).
    Update (sigma-modification keeps the estimates bounded against the lagged plant; the
    normalisation keeps the step size finite when phi or s are large):
        theta_hat += dt * g_theta * ( phi * s / (1 + phi^2)   - sigma * (theta_hat - 1) )
        d_hat     += dt * g_d     * ( s / (1 + s^2)           - sigma * d_hat )
    Both are clamped (theta in [0.2, 5], |d| <= d_max).
    """
    name = "adaptive_vf"

    def __init__(self, k_e, gamma_theta=0.5, gamma_d=0.2, sigma=0.5, dt=0.05,
                 theta_min=0.2, theta_max=5.0, d_max=1.0, **kw):
        super().__init__(k_e=k_e, **kw)
        self.gamma_theta, self.gamma_d, self.sigma = gamma_theta, gamma_d, sigma
        self.dt = dt
        self.theta_min, self.theta_max, self.d_max = theta_min, theta_max, d_max
        self.theta_hat, self.d_hat = 1.0, 0.0

    def set_dt(self, dt: float):
        self.dt = dt

    def reset(self, path, x, y):
        super().reset(path, x, y)
        self.theta_hat, self.d_hat = 1.0, 0.0

    def command(self, path, x, y, vx, vy):
        i = self._project(path, x, y)
        vg = math.hypot(vx, vy)
        chi = math.atan2(vy, vx)
        chi_p = path.psi[i]
        e = -math.sin(chi_p) * (x - path.x[i]) + math.cos(chi_p) * (y - path.y[i])
        chi_d = chi_p - self.chi_inf * (2.0 / math.pi) * math.atan(self.k_e * e)
        s = float(wrap_pi(chi_d - chi))
        phi = vg * path.kappa[i]
        a = vg * (self.k_chi * s + self.theta_hat * phi + self.d_hat)
        th = self.theta_hat + self.dt * self.gamma_theta * (
            phi * s / (1.0 + phi * phi) - self.sigma * (self.theta_hat - 1.0))
        dh = self.d_hat + self.dt * self.gamma_d * (s / (1.0 + s * s) - self.sigma * self.d_hat)
        self.theta_hat = min(self.theta_max, max(self.theta_min, th))
        self.d_hat = min(self.d_max, max(-self.d_max, dh))
        return a


class CarrotChasing(Guidance):
    """Carrot chasing (after Sujit et al. 2014). Reconstructed from memory; gain form unverified.

    A 'carrot' point is placed delta = delta_time * vg ahead of the vehicle's projection on the path
    and the vehicle steers its course toward it: a = vg * kappa * eta, eta = angle from the velocity
    to the line to the carrot.
      mode='tangent' : carrot on the TANGENT LINE at the projection point (the original definition,
                       which cuts corners on curves)
      mode='arc'     : carrot at path ARC LENGTH delta ahead (our adaptation, so it follows curves)
    """
    name = "carrot_chasing"

    def __init__(self, delta_time: float = 2.0, kappa: float = 1.0, mode: str = "arc", **kw):
        super().__init__(**kw)
        if mode not in ("arc", "tangent"):
            raise ValueError("mode must be 'arc' or 'tangent'")
        self.delta_time, self.kappa, self.mode = delta_time, kappa, mode

    def command(self, path, x, y, vx, vy):
        i = self._project(path, x, y)
        vg = math.hypot(vx, vy)
        chi = math.atan2(vy, vx)
        delta = max(self.delta_time * vg, self.min_lookahead)
        if self.mode == "tangent":
            cx = path.x[i] + delta * math.cos(path.psi[i])
            cy = path.y[i] + delta * math.sin(path.psi[i])
        else:
            j = min(int(np.searchsorted(path.s, path.s[i] + delta)), path.n - 1)
            cx, cy = path.x[j], path.y[j]
        if math.hypot(cx - x, cy - y) < 1e-6:
            return 0.0
        eta = float(wrap_pi(math.atan2(cy - y, cx - x) - chi))
        return vg * self.kappa * eta


DEFAULT_PARAMS = {
    "pure_pursuit": {"lookahead_time": 2.5},
    "l1": {"lookahead_time": 2.5},
    "vector_field": {"k_e_scale": 0.4, "chi_inf_deg": 70.0, "k_chi": 1.0, "feedforward": True},
    "lead_vf": {"k_e_scale": 0.4, "chi_inf_deg": 70.0, "k_chi": 1.0, "feedforward": True,
                "lead_time": 0.6},
    "adaptive_vf": {"k_e_scale": 0.4, "chi_inf_deg": 70.0, "k_chi": 1.0, "gamma_theta": 0.5,
                    "gamma_d": 0.2, "sigma": 0.5},
    "carrot_chasing": {"delta_time": 2.0, "kappa": 1.0},
    "carrot_tangent": {"delta_time": 2.0, "kappa": 1.0},
}


def make_law(name: str, r_nom: float, params: dict | None = None) -> Guidance:
    """Build a law. `params` overrides DEFAULT_PARAMS[name].

    pure_pursuit / l1 : lookahead_time (s)
    vector_field      : k_e_scale (k_e = 1 / (k_e_scale * r_nom)), chi_inf_deg,
                        k_chi (1/s), feedforward (bool)
    lead_vf           : the same, plus lead_time (s): how far ahead the curvature is read
    """
    if name not in DEFAULT_PARAMS:
        raise ValueError(f"unknown law: {name}")
    p = {**DEFAULT_PARAMS[name], **(params or {})}
    unknown = set(p) - set(DEFAULT_PARAMS[name])
    if unknown:
        raise ValueError(f"unknown parameters for {name}: {sorted(unknown)}")
    if name == "pure_pursuit":
        return PurePursuit(lookahead_time=p["lookahead_time"])
    if name == "l1":
        return L1(lookahead_time=p["lookahead_time"])
    if name == "adaptive_vf":
        return AdaptiveVectorField(k_e=1.0 / (p["k_e_scale"] * r_nom), chi_inf=math.radians(p["chi_inf_deg"]),
                                   k_chi=p["k_chi"], gamma_theta=p["gamma_theta"], gamma_d=p["gamma_d"],
                                   sigma=p["sigma"])
    if name in ("carrot_chasing", "carrot_tangent"):
        return CarrotChasing(delta_time=p["delta_time"], kappa=p["kappa"],
                             mode="tangent" if name == "carrot_tangent" else "arc")
    if name == "lead_vf":
        return LeadVectorField(k_e=1.0 / (p["k_e_scale"] * r_nom), lead_time=p["lead_time"],
                               chi_inf=math.radians(p["chi_inf_deg"]), k_chi=p["k_chi"],
                               feedforward=p["feedforward"])
    return VectorField(k_e=1.0 / (p["k_e_scale"] * r_nom),
                       chi_inf=math.radians(p["chi_inf_deg"]),
                       k_chi=p["k_chi"], feedforward=p["feedforward"])


LAW_NAMES = ["pure_pursuit", "l1", "vector_field", "lead_vf"]      # the original four
EXTRA_LAWS = ["adaptive_vf", "carrot_chasing"]                       # added later
ALL_LAW_NAMES = LAW_NAMES + EXTRA_LAWS                              # carrot_tangent: sensitivity check only

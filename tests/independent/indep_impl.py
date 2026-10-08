import numpy as np


def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _get(path, k):
    if isinstance(path, dict):
        return np.asarray(path[k], dtype=float)
    return np.asarray(getattr(path, k), dtype=float)


class IndepLaw:
    def __init__(self, name, **params):
        if name not in ("pure_pursuit", "l1", "vector_field", "lead_vf"):
            raise ValueError(name)
        self.name = name
        self.lookahead_time = params.get("lookahead_time", 2.5)
        self.min_lookahead = params.get("min_lookahead", 3.0)
        self.k_e = params.get("k_e", 0.1)
        self.chi_inf = params.get("chi_inf", np.deg2rad(70.0))
        self.k_chi = params.get("k_chi", 1.0)
        self.feedforward = params.get("feedforward", True)
        self.lead_time = params.get("lead_time", 0.6)
        self.idx = 0

    def reset(self, path, x, y):
        px, py = _get(path, "x"), _get(path, "y")
        self.idx = int(np.argmin((px - x) ** 2 + (py - y) ** 2))

    def _project(self, px, py, x, y):
        n = len(px)
        lo = max(self.idx - 5, 0)
        hi = min(self.idx + 800, n)
        d2 = (px[lo:hi] - x) ** 2 + (py[lo:hi] - y) ** 2
        self.idx = max(self.idx, lo + int(np.argmin(d2)))

    @staticmethod
    def _steer(px, py, j, x, y, vg, chi):
        ax, ay = px[j], py[j]
        d = np.hypot(ax - x, ay - y)
        if d < 1e-6:
            return 0.0
        eta = wrap_pi(np.arctan2(ay - y, ax - x) - chi)
        return 2 * vg ** 2 * np.sin(eta) / d

    def command(self, path, x, y, vx, vy):
        px, py = _get(path, "x"), _get(path, "y")
        n = len(px)
        self._project(px, py, x, y)
        vg = np.hypot(vx, vy)
        chi = np.arctan2(vy, vx)
        ld = max(self.lookahead_time * vg, self.min_lookahead)
        i = self.idx
        if self.name == "pure_pursuit":
            s = _get(path, "s")
            j = min(int(np.searchsorted(s, s[i] + ld)), n - 1)
            return float(self._steer(px, py, j, x, y, vg, chi))
        if self.name == "l1":
            hi = min(i + 800, n)
            d2 = (px[i:hi] - x) ** 2 + (py[i:hi] - y) ** 2
            hit = np.nonzero(d2 >= ld ** 2)[0]
            j = i + int(hit[0]) if len(hit) else n - 1
            return float(self._steer(px, py, j, x, y, vg, chi))
        psi, kappa = _get(path, "psi"), _get(path, "kappa")
        chi_p = psi[i]
        e = -np.sin(chi_p) * (x - px[i]) + np.cos(chi_p) * (y - py[i])
        chi_d = chi_p - self.chi_inf * (2 / np.pi) * np.arctan(self.k_e * e)
        a = vg * self.k_chi * wrap_pi(chi_d - chi)
        if self.feedforward:
            if self.name == "lead_vf":
                s = _get(path, "s")
                j = min(int(np.searchsorted(s, s[i] + self.lead_time * vg)), n - 1)
                a += vg ** 2 * kappa[j]
            else:
                a += vg ** 2 * kappa[i]
        return float(a)


def indep_simulate(path, a_max, V, tau, law, dt):
    px, py = _get(path, "x"), _get(path, "y")
    psi_a, s = _get(path, "psi"), _get(path, "s")
    n = len(px)
    x, y, psi, a_act = px[0], py[0], psi_a[0], 0.0
    law.reset(path, x, y)
    r_min = V ** 2 / a_max
    n_max = int(4 * s[-1] / V / dt)
    T, X, Y, AC, AS = [], [], [], [], []
    for k in range(n_max):
        t = k * dt
        vx, vy = V * np.cos(psi), V * np.sin(psi)
        a_cmd = law.command(path, x, y, vx, vy)
        a_sat = float(np.clip(a_cmd, -a_max, a_max))
        T.append(t); X.append(x); Y.append(y); AC.append(a_cmd); AS.append(a_sat)
        if law.idx >= n - 3 and np.hypot(x - px[-1], y - py[-1]) < 0.4 * r_min:
            break
        a_act += dt * (a_sat - a_act) / tau
        psi += dt * a_act / V
        x += dt * vx
        y += dt * vy
    return dict(t=np.array(T), x=np.array(X), y=np.array(Y),
                a_cmd=np.array(AC), a_sat=np.array(AS))

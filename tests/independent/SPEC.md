# Spec: four path-following laws and a 2-D point-mass simulator

Write `indep_impl.py` from this spec ONLY. Pure numpy, no other dependencies.

## Path
Arrays (all length n): x, y, psi (path heading, rad), kappa (signed curvature 1/m, left = +),
s (cumulative arc length, m, s[0]=0). Angles wrapped with wrap_pi(a) = (a+pi) mod 2pi - pi.

## Common
State: position (x,y), ground velocity (vx,vy). vg = hypot(vx,vy), chi = atan2(vy,vx).
Output of every law: ONE lateral acceleration, left turn = positive (m/s^2).
Every law keeps an integer index `idx` (progress along the path), initialised at reset(path,x,y)
to the argmin over ALL samples of squared distance to (x,y).
Projection (called first in every command): search samples lo..hi-1 with lo = max(idx-5, 0),
hi = min(idx+800, n); new idx = max(idx, lo + argmin of squared distance over that window).
Lookahead distance: ld = max(lookahead_time * vg, min_lookahead). Defaults: lookahead_time 2.5 s,
min_lookahead 3.0 m.

## pure_pursuit
j = min(searchsorted(s, s[idx] + ld), n-1)   (np.searchsorted default side='left')
aim = (x[j], y[j]); d = |aim - pos|; if d < 1e-6 return 0.
eta = wrap_pi(atan2(aim_y - y, aim_x - x) - chi);  a = 2 vg^2 sin(eta) / d.

## l1
Same steering formula, different aim point: among samples idx..min(idx+800,n)-1 take the FIRST whose
squared distance from pos is >= ld^2 (j = that sample; if none, j = n-1). Then as above.

## vector_field  (parameters k_e, chi_inf = 70 deg in rad, k_chi = 1.0 /s, feedforward = True)
chi_p = psi[idx]
e = -sin(chi_p)*(x - xp) + cos(chi_p)*(y - yp), with (xp,yp) = path point idx   (+ = vehicle left of path)
chi_d = chi_p - chi_inf*(2/pi)*atan(k_e*e)
a = vg*k_chi*wrap_pi(chi_d - chi); if feedforward: a += vg^2 * kappa[idx].

## lead_vf  (vector_field plus lead_time T, default 0.6 s)
Identical, except the feed-forward curvature is read at j = min(searchsorted(s, s[idx] + T*vg), n-1):
a += vg^2 * kappa[j].

## Simulator  simulate(path, a_max, V, tau, law, dt, seed-free variant: no wind, no noise)
Start exactly on the path start: x,y = path point 0, psi = psi[0], a_act = 0. law.reset at start.
Loop k = 0,1,...: t = k*dt;  vx = V cos(psi), vy = V sin(psi);
  a_cmd = law.command(path, x, y, vx, vy);  a_sat = clip(a_cmd, -a_max, +a_max)
  record (t, x, y, a_cmd, a_sat)
  stop if law.idx >= n-3 and dist(pos, path end) < 0.4*r_min where r_min = V^2/a_max  (record first)
  then, in this order:
    a_act += dt*(a_sat - a_act)/tau
    psi += dt*a_act/V
    x += dt*vx ; y += dt*vy       (uses the vx,vy computed at the top of the step)
  stop after n_max = int(4*path_length/V/dt) steps.
Return the recorded arrays.

## Deliverable
`indep_impl.py` with: class IndepLaw(name, **params) having reset(path_arrays,x,y) and
command(path_arrays,x,y,vx,vy) and attribute idx; and function indep_simulate(...). Parameters for
the vector fields are passed as k_e, chi_inf (rad), k_chi, feedforward, lead_time.
Do not look at any other code on this machine. If anything above is ambiguous, pick the most literal
reading and list each ambiguity you hit in `AMBIGUITIES.md`.

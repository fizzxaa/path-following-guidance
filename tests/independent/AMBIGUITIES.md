# Ambiguities
1. Path container format unspecified: accept dict or object with attributes x,y,psi,kappa,s.
2. Default k_e not given: chose 0.1 (1/m).
3. Law default params when "feedforward" given for lead_vf: feedforward=False disables the curvature term entirely.
4. indep_simulate signature given loosely: (path, a_max, V, tau, law, dt); returns dict of arrays t,x,y,a_cmd,a_sat.
5. path_length taken as s[-1]; path end = point n-1.
6. Stop check uses idx after command and the pre-update position; the stopping step is recorded (as spec says).
7. Loop runs at most n_max steps (k = 0..n_max-1).
8. Projection window empty (idx >= n) cannot occur since idx <= n-1; not guarded.
9. l1 "squared distance >= ld^2" window starts at idx (after projection).
10. Unknown law name raises ValueError.

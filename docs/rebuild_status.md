# Rebuild status (2026-10-05)

The working copy built during the earlier sessions was lost in a container reset before anything was
pushed. This repo is the original code plus what has been rebuilt and re-checked since. Numbers quoted
in earlier summaries are NOT evidence until they reproduce here.

## Verified now (tests in this repo)
- tests/test_known_answers.py (43): simulator lag and turn radius against closed forms; every law
  gives V^2/R at zero error on a circle and converges to zero error on circles and straights;
  scale, rotation and mirror invariance to 1e-6 m; dt convergence (rms changes <=~3% per halving).
  Seven injected bugs (gain, sign, lag, turn rate, wind, an active min-lookahead) are all caught.
- tests/test_vs_published.py: repo L1 equals ArduPilot AP_L1_Control's waypoint formula on a
  straight segment (zeta = 1/sqrt(2)) to 0.006 m/s^2 over 300 random states (max |a| = 15).
  The formula was transcribed from a fetch summary, not byte-checked.
- tests/test_independent.py (16): a second implementation written from tests/independent/SPEC.md by
  an agent that never saw the code agrees to 1e-9 (commands) and 1e-6 m (trajectories). Limit: the
  spec was written by the same author as the code, so a shared misreading would pass both.
- tests/test_segments.py: labels and per-region RMS recover known planted offsets.

## Not verified
- Vector field vs the Beard & McLain book: the straight-line field formula is the standard one, but
  no source was available to check it. On arcs the repo uses the straight-line field plus curvature
  feed-forward, which is NOT the book's orbit field. Say "straight-line vector field with curvature
  feed-forward".
- ArduCopter SITL results (results/sitl_*): original runs unchanged; no ground truth. See 'SITL re-run 2026-10-08' below.

## Result that did NOT reproduce
Earlier summary: "whole-path winner is distinguishably worse in arcs in 7/18 (quad), 6/18 (fixed-wing)".
Rebuilt, four laws, Dubins-tuned gains, exact metric (results/segments_report.md):
- arcs: 1/18 (quad), 2/18 (fixed-wing), identical at windows 0.5, 1, 2 turning radii;
  straights: 3-8/18 depending on window.
- The 7/18 and 6/18 figures came from the six-law set (adaptive and carrot, lost in the reset).
  Do not cite them.

## What the four-law data does support
- The vector field's error is concentrated in arcs: arc/straight ratio 4.1 (quad), 4.4 (fixed-wing) vs
  2.0-2.3. CORRECTION: its tie with L1 on straights holds in the 18-condition aggregate but fails in 68-91%
  of the perturbed worlds of the sensitivity sweep, so do not claim its failure is confined to arcs.
- Quad, all conditions: arc ties pure_pursuit, l1, lead_vf; whole-path ties l1, lead_vf.
- The lead-time feed-forward closes the vector field's arc deficit (0.80 -> 0.36 m quad).
- Effort: vector_field saturates 0.079 (quad) / 0.105 (fixed-wing) of the time vs 0.026-0.033.
- Zero runs failed to finish in the 1,728 runs.

## Six-law held-out result (results/segments_report_6laws.md)
Adaptive vector field and carrot chasing are rebuilt (adaptive is OUR law, not a reproduction of Fari or
Wang; carrot's gain form is from memory) and retuned on the tuning routes only.
- Carrot chasing (arc-length carrot, tuned) wins whole-path and straights by a wide margin but is
  distinguishably worse than L1 in arcs. Quad: straight 0.094 vs L1 0.162 m, arc 0.454 vs 0.371 m.
  Fixed-wing: straight 0.389 vs 0.804, arc 2.525 vs 1.774.
- The whole-path winner is therefore distinguishably worse in arcs in 9/18 (quad) and 4/18 (fixed-wing)
  conditions. Earlier (pre-reset) figures were 7/18 and 6/18: same sign, different counts, because the
  search is random; treat the count as sensitive to carrot's tuned gains.
- Carrot in its ORIGINAL tangent-line form (same gains) is much worse than the arc form: whole-path
  +0.32 m (quad), +1.62 m (fixed-wing), arcs +0.56 / +2.54 m. Carrot's advantage is therefore a
  property of our arc-length adaptation plus tuning, not of carrot chasing as published.
- Adaptive vector field ties the plain vector field (0.507 vs 0.502 m quad), with sat_fraction 0.094.
- Effort/saturation for the original four laws and carrot match the pre-reset numbers to 3 decimals.
- Tuned carrot has kappa*delta_time/2 ~ 2-2.5, i.e. it commands about twice the on-path curvature at zero
  error; see test_known_answers.py. Do not describe it as an exact-on-circles law.

## Still lost, to rebuild
clothoid route arms (planner vs controller), fig4-8 for six laws, claims doc, paper draft updates.

## Added after the six-law run
- results/sensitivity_report.md: five claims checked in 22 worlds per vehicle (lag, a_max, noise, delay, gusts, wind).
  Robust: carrot beats L1 on straights, vector field worse than L1 in arcs, lead fixes it. Not robust: carrot worse
  than L1 in arcs (27-32%), vector field ties L1 on straights (9-32%).
- results/planner_vs_controller.md: smoothing the route changes every law by about as much as the whole law
  spread; with gains retuned on smooth routes the spread nearly vanishes. See docs/claims.md.

## Source check attempted 2026-10-05 (neither confirmed)
- Fari 2020: Fari, Wang, Roy, Baldi, "Addressing Unmodeled Path-Following Dynamics via Adaptive Vector Field:
  A UAV Test Case", IEEE TAES 56(2):1613-1622, doi 10.1109/TAES.2019.2925487. Abstract seen (adapts to unknown wind
  and unmodeled course dynamics). Open accepted manuscript at research.tudelft.nl/files/72398104/paper6_short_rev6_final.pdf
  returned 403 to the fetch tool; the equations were NOT read. "Wang 2022" appears to be the same group's
  "Adaptive Vector Field Guidance Without a Priori Knowledge of Course Dynamics and Wind" (title only; ResearchGate 429).
  The IFAC paper on reliable vector-field design under uncertain course dynamics is paywalled.
  => keep the wording "ours, after Fari/Wang; not a reproduction".
- Beard & McLain: the book text (dokumen.pub, GitHub mavsim/rosplane, ROSflight docs) was unreachable or 404.
  Indirect support only: an open-access 2025 paper (Robotics 14(1):7) writes the straight-line field as
  chi_des = -chi_inf (2/pi) arctan(k d), the same form as ours (chi_inf, 2/pi, arctan, gain on cross-track error).
  Its orbit field, chi_des = gamma - lambda pi/2 - arctan(k(d-R)), is NOT ours. Ours = straight-line field + curvature feed-forward.
  => the straight-line formula is consistent with a published statement, NOT confirmed against the book.

## SITL re-run 2026-10-08 (run by Fizza on her Mac)
- Setup: ArduCopter V4.8.0-dev (1ea89b0b), SITL QUAD/PLUS, MAVProxy udp:127.0.0.1:14550, wind 0, GUIDED velocity wrapper,
  horizon 0.5 s, 10 Hz. Reference = planned path; position = EKF output (no ground truth).
- One flight: L1, planned radius 25.0 m, path 733 m, finished=True, rms 0.69 m (0.055 r_min), max 1.86 m (0.149 r_min).
  Earlier SITL L1: rms 0.69 m, max 1.87 / 1.88 m. Same numbers on a fresh build -> repeatable at this one setting.
- Screenshots (map + console) show take-off to 10 m, GUIDED flight with a curved track, LAND, DISARMED. They contain no error numbers.
- Still missing: horizon sweep (0.25, 0.75), L1 at 7.5 m, vector_field and lead_vf flights, any repeat flights, ArduPlane.
  Until those exist, SITL is "one setting, one law, repeated once": a sanity check, not a validation of the rankings.

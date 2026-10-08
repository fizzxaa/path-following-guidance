# Claim list (frozen 2026-10-05). Every claim names its evidence; no evidence, no claim.

Evidence files are in `results/`; tests are in `tests/`. Numbers are RMS cross-track error (exact
projection), 12 held-out routes, 18 conditions, route-cluster bootstrap 95% intervals, simulation only
unless stated. Gains were tuned on separate routes (seeds 1000-1005); test routes are 2000-2011.

## Claims that stand
| # | Claim | Evidence | Strength |
|---|---|---|---|
| A | The simulator and the four original laws behave as theory says (closed forms, scale/rotation/mirror invariance, dt convergence); L1 equals ArduPilot's waypoint formula (zeta = 1/sqrt2) to 0.006 m/s^2; an independently written version agrees to 1e-9. | tests/test_known_answers.py, test_vs_published.py, test_independent.py | strong for implementation; says nothing about physics beyond the point-mass model |
| B | On Dubins routes the planner's curvature jumps account for most of the vector field's deficit: moving to a smoothed route cuts its error 0.50 -> 0.18 m (quad) and 2.38 -> 0.84 m (fixed-wing), the largest change of any law. | results/planner_vs_controller.md | strong in simulation; "smoothed Dubins" is not a clothoid |
| C | Planner effect is the same order as controller effect. Quad: law spread 0.229 m [0.175, 0.288] on Dubins vs mean per-law change from smoothing 0.169-0.187 m [0.13, 0.23]. With gains retuned on smooth routes the law spread falls to 0.018 m [0.002, 0.035] (quad), 0.129 m (fixed-wing). | same | strong in simulation |
| D | Rankings of laws depend on the route type: best/worst flips (vector field is 5th of 6 on Dubins, 1st of 6 on smoothed routes with Dubins-tuned gains). | same | strong |
| E | Carrot chasing (OUR arc-length form, tuned) beats L1 on straight segments; the vector field is worse than L1 in arcs; the lead feed-forward closes that arc deficit. | results/sensitivity_report.md: C1a 95%, C2a 95%, C2b 82-95% of 22 perturbed worlds per vehicle | moderate: simulation, Dubins routes |
| F | Carrot in the original tangent-line form is much worse than the arc form (whole-path +0.32 m quad, +1.62 m fixed-wing). Carrot's good numbers belong to our adaptation plus tuning. | results/heldout_segments_tangent.csv | strong |
| G | Adaptive vector field (ours, not Fari/Wang) ties the plain vector field (0.507 vs 0.502 m quad) and saturates more (0.094 vs 0.079). | results/segments_report_6laws.md | moderate |
| H | Effort/saturation: vector-field family saturates 3-4x more than L1/pure pursuit/lead_vf; 0 of 1,728+ held-out runs failed to finish. | results/segments_report*.md | strong |

## Claims that need a caveat in the text
- "Whole-path winner is worse in arcs": 9/18 quad, 4/18 fixed-wing conditions (aggregate). In the two-condition
  sensitivity sweep, "carrot worse than L1 in arcs" held in only 27-32% of worlds, so it is condition-dependent.
- Carrot's tuned gains give kappa*delta_time/2 ~ 2-2.5 (about twice on-path curvature at zero error).
- Reference path is the planned path, not ground truth; SITL has no independent ground truth.

## Dropped (do not write)
- "Whole-path winner is worse in arcs in 7/18 and 6/18": did not reproduce; replaced by the counts above.
- "L1 and pure pursuit carry a standing straight offset": L1 is best or tied on straights in the rebuilt data.
- "The vector field ties L1 on straights / its failure is confined to arcs": holds in only 9-32% of perturbed worlds.
- "Collapse of the law spread on clothoid routes" (earlier, from a broken builder) and any clothoid claim.
- The preview-time design rule T* = tau + 1/k_chi; the demand-ratio redesign; the 2.4 m/s^2 a_max inference.

## Open / not supported by anything run here
- SITL: single flights, ArduCopter only, no wind, version V4.8.0-dev 1ea89b0b (recorded 2026-10-08); ~2x sim/SITL gap unexplained.
- Fari 2020 / Wang 2022 formulation unread; Beard & McLain vector field not checked against the book.
- Section-2 novelty sentence unverified against groups 3-4 of references.md.

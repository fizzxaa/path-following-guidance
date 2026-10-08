# Curvature-limited path tracking for a quadcopter and a fixed-wing plane

**The question.** A vehicle that can't turn tighter than a limit (a plane because of its minimum
speed, a quadcopter because of its maximum sideways acceleration) is given a planned Dubins path.
The path is only a line on a map. Which steering law keeps the vehicle on it when there is wind, and
what breaks when the plan is too tight or the vehicle is weaker than assumed?

This repo has:
- a **Dubins planner** (all six path types) and a waypoint-to-route builder,
- **six guidance laws**: pure pursuit, L1, vector field, `lead_vf` (vector field that reads the curvature ahead), and two author-adapted laws, an adaptive vector field and carrot chasing (not reproductions of published laws),
- a **fast 2-D simulator** (constant airspeed, lateral-acceleration limit and lag, steady wind plus gusts),
- a **benchmark** (2 vehicles x 6 laws x 3 wind levels x 3 planned radii x 2 limit mismatches x seeds),
- an **ArduCopter SITL runner** (tested against fakes, and flown in ArduCopter SITL: ten flights, see "Real autopilot check"),
- **random routes**, a **tuning script** and a **held-out evaluation** with error bars,
- an **animation** script (GIF),
- 194 tests, a segment-resolved analysis (straight / transition / arc), a planner-versus-controller study and a sensitivity check of every claim (`docs/claims.md`).

## Setup and run

```bash
pip install -r requirements.txt
python -m pytest tests -q                    # 194 tests
python scripts/heldout_segments.py --help   # held-out comparison (writes results/heldout_segments*.csv)
python scripts/analyse_segments.py --help    # segment-resolved report
python scripts/planner_vs_controller.py      # route effect vs law effect
python scripts/sensitivity.py                # sensitivity of the claims
python scripts/make_paper_figs.py            # figures for the report
```
Older commands (`ctrack.benchmark`, `ctrack.evaluate`, `ctrack.robustness`, `ctrack.ablations`, `ctrack.plots`,
`ctrack.animate`) still run, but their old outputs were removed from `results/` on 2026-10-08 because they used the
earlier metric and did not all reproduce. `python -m ctrack.tune` re-tunes the laws on the tuning routes.

## Layout

```
ctrack/dubins.py      shortest Dubins path, sampling
ctrack/route.py       chain Dubins paths through waypoints; the benchmark route
ctrack/vehicles.py    fixed-wing and quadcopter parameters (r_min = V^2 / a_max)
ctrack/guidance.py    PurePursuit, L1, VectorField, lead_vf, AdaptiveVectorField, CarrotChasing
ctrack/sim.py         2-D simulator with wind, gusts, lag, limit mismatch
ctrack/metrics.py     cross-track error, saturation, control effort
ctrack/scenarios.py   test conditions, route seeds, and the one function that runs a scenario
ctrack/tune.py        tune each law on tuning routes -> results/tuned_params.json
ctrack/evaluate.py    default vs tuned on unseen routes, paired stats with bootstrap intervals
ctrack/robustness.py  sensor noise / delay / slower response / slower airspeed on the test routes
ctrack/ablations.py   lead-time sweep, feed-forward on/off, L1 lookahead sweep
ctrack/estimate_lag.py  fit the real copter's response (gain, lag) from a SITL log
ctrack/paper_figures.py paper-style figures and tables generated from results/*.csv
paper_fig/            report v2 (PDF and HTML source) and its figures
docs/                 claim list and rebuild status
scripts/              held-out comparison, segment analysis, planner-vs-controller, sensitivity, figures
ctrack/benchmark.py   the exploratory sweep on the fixed route -> results/benchmark.csv
ctrack/plots.py       figures
ctrack/animate.py     animated GIF of the four laws on the same route; optional CSV export
sitl/sitl_quad_runner.py   ArduCopter SITL via pymavlink
ctrack/sitl_control.py    the tested control step and log saving used by the runner
ctrack/plot_sitl.py       plot SITL logs
tests/                Dubins, guidance, known-answer, published-formula and independent-copy tests
```

## What the tests check
- The Dubins path ends on the goal pose (position and heading), over 500 random cases.
- The chosen path is the shortest of the six, is never shorter than a straight line, and never exceeds the curvature limit.
- Two exact cases: a straight line, and a half-circle U-turn of length pi * rho.
- Each law gives zero command on the path, steers back toward it from either side, and never moves its progress marker backwards.
- Runs are repeatable for a given seed, and the physical limit really is applied.

## Watching a run

```bash
python -m ctrack.animate                        # quadcopter, all 4 laws, hard case -> results/run.gif
python -m ctrack.animate --vehicle fixed_wing --wind 0.15 --radius-factor 1.5 --mismatch 1.0
python -m ctrack.animate --route 2003 --seed 4  # a random test route, a different wind
python -m ctrack.animate --tuned                # use the tuned gains
python -m ctrack.animate --csv-dir results/export   # also write the runs as CSV files
```
The panels share the same route, start and wind. A red **LIMIT HIT** label shows when a law asks
for more turn than the vehicle can give. A still image of the last frame is saved next to the GIF.

**A real simulator:** the animation is a plot of the 2-D model, not a physics simulator. For a real
simulator view use ArduPilot SITL (see below); it shows the vehicle on a map.

## How the comparison is set up (so it is fair)
- **Metric**: RMS distance to the planned path in metres (exact projection onto the path polyline), measured after the first 4 turning radii of travel, reported on straights, transitions and arcs.
- **Tuning routes** (random routes, seeds 1000-1005) and **test routes** (random, seeds 2000-2011) never overlap. A fixed "standard" route is kept as an extra check.
- **Conditions** (same on test): wind 0 / 15 / 30% of airspeed x planned radius 1.0 / 1.5 / 2.0 x true vehicle limit 100% / 80% of what the planner assumed.
- Each law is tuned separately for each vehicle. Default gains are always one of the candidates.
- Error bars come from resampling **routes** (scenarios on one route are not independent), and are wide with only 12 routes.

## Results (simulation only; test routes never used for tuning or design)

The earlier results in this README (a score of RMS + 0.25 x worst error, a nearest-sample distance, four laws) were
replaced on 2026-10-08 after a rebuild with an exact distance-to-path metric, six laws and segment-resolved errors. Some
of the old numbers did not reproduce and were withdrawn. The current numbers, with the evidence for each claim, are in:

- `docs/claims.md`: the frozen claim list (what stands, what needs a caveat, what was dropped).
- `docs/rebuild_status.md`: what was verified and what was not.
- `results/segments_report_6laws.md`, `results/planner_vs_controller.md`, `results/sensitivity_report.md`: the tables.
- `paper_fig/path-following-report-v2.pdf`: the report.

Main findings, whole-path RMS error in metres on 12 unseen routes (quadcopter, planned at the limit): L1 0.474,
vector field 1.018, lead vector field 0.522. The route matters about as much as the law: smoothing the route lowers the
vector field's error from 0.50 to 0.18 m. Rankings change with the route type, and no law wins everywhere.

## Real autopilot check (ArduCopter SITL; no wind, default gains, one flight per cell)

RMS error / worst error in metres, quadcopter, cruise speed 5 m/s. Planned radius 25 m needs about 1 m/s^2 of sideways
acceleration, 12.5 m needs about 2, and 7.5 m needs about 3.3.

| law | radius 25 m (`--radius-factor 2.0`) | radius 12.5 m (`1.0`) | radius 7.5 m (`0.6`) |
|---|---|---|---|
| L1 | 0.69 / 1.87 | 0.87 / 2.79 | not flown |
| vector_field | 0.36 / 0.84 | 0.52 / 1.63 | 0.97 / 3.55 |
| lead_vf | 0.34 / 0.84 | 0.49 / 1.67 | 0.59 / 2.26 |

- **The vector-field family tracked about twice as well as L1** at 25 m and at 12.5 m. The simulation predicted this at 25 m, but it predicted L1 and the vector-field law would be close at 12.5 m; on the real autopilot L1 was clearly worse.
- **`lead_vf` and `vector_field` were tied at 25 m and 12.5 m** (within about 6%).
- **At 7.5 m, `lead_vf` was clearly better:** RMS error 39% lower and worst error 36% lower than `vector_field`. This is the direction the simulation predicts for a plan that asks for more than the vehicle can give (the simulation says about half the error).
- The gap between the two grows as the plan gets tighter (6%, 6%, then 39%). That fits the explanation that the benefit appears once the plan needs more turning than the vehicle can give.
- Absolute errors were about 2x the simulation's at the same settings (L1 at 25 m: 0.055 r_min against 0.023).

Caveats: one flight per cell (SITL without wind is nearly repeatable: two L1 flights at the same settings gave worst errors of 1.87 m and 1.88 m). The copter's real sideways-acceleration limit in GUIDED mode is unknown (this ArduCopter build has no `WPNAV_*` parameters). The `r_min` printed by the runner is the nominal 12.5 m for every radius, not the planned radius. L1 was not flown at 7.5 m. All laws used their default (untuned) gains.

## SITL (ArduCopter) step by step

**Status.** The runner has been flown in ArduCopter SITL on macOS (ten flights; results in "Real autopilot check").
The control maths is also tested against a fake copter (north/east conversions included), and the whole runner
against a fake MAVLink connection. The first real runs needed fixes (GUIDED mode retried together with arming,
identifying the vehicle from its heartbeat), which are described below.

Run the simulator and the runner on the same machine (macOS or Ubuntu).

1. Get the repo into the VM (copy the zip in), then:
   ```bash
   cd curvature-tracking
   python3 -m venv .venv && source .venv/bin/activate
   pip install numpy pandas matplotlib pytest pymavlink
   python -m pytest tests -q
   ```
2. **Terminal 1**: start the simulator from your ArduPilot folder and **wait about a minute** for GPS and EKF:
   ```bash
   sim_vehicle.py -v ArduCopter --console --map
   ```
   (No desktop in the VM? Leave out `--console --map`.)
3. **Terminal 2**: fly a first, gentle test (planned radius x2.0 needs about 1 m/s^2 sideways, near the ArduCopter default):
   ```bash
   python sitl/sitl_quad_runner.py --law l1 --radius-factor 2.0 --out results/sitl_l1.csv
   python -m ctrack.plot_sitl results/sitl_l1.csv
   ```
4. Only then compare laws and tighter plans. Repeat each run 3 times (SITL is not perfectly repeatable):
   ```bash
   python sitl/sitl_quad_runner.py --law pure_pursuit --radius-factor 2.0 --out results/sitl_pure_pursuit.csv
   python sitl/sitl_quad_runner.py --law vector_field --radius-factor 2.0 --out results/sitl_vector_field.csv
   python -m ctrack.plot_sitl results/sitl_pure_pursuit.csv results/sitl_l1.csv results/sitl_vector_field.csv
   ```

**On a Mac without MAVProxy** (what worked in practice): install the build helpers into the project environment
(`pip install pexpect "empy==3.3.4" future pyserial setuptools`), start the simulator with
`python Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy` from your ArduPilot folder (first build takes
a while), and connect the runner with `--connect tcp:127.0.0.1:5760`. There is no live map in this mode.
Use `python -m ctrack.animate` and `python -m ctrack.plot_sitl` to look at runs afterwards.

**Seeing the flight.** With `--no-mavproxy` no window or map opens. To watch it:
- **After the flight:** `python -m ctrack.plot_sitl results/sitl_l1.csv --gif results/sitl_replay.gif` then `open results/sitl_replay.gif`
  (list several logs to replay them together).
- **Live:** install QGroundControl for macOS, then add a TCP comm link to `127.0.0.1` port `5762` (SITL's second serial port; the runner
  uses 5760). Do not press Takeoff/Fly in QGroundControl while the runner is flying.

**Lessons from the first real runs.** (1) Setting GUIDED once at the start fails, because GUIDED is refused until GPS
and the EKF have a position; it is now retried together with arming. (2) The connection printed `system 0`: the runner
had not worked out which vehicle it was talking to, and it threw away ArduPilot's status messages while waiting for
heartbeats, so failures had no visible reason. The runner now reads every message, finds the vehicle from its
heartbeat (ignoring ground-station heartbeats), addresses every command to it, reads the flight mode and armed state
directly from the heartbeat, and prints every `[ArduPilot]` message.


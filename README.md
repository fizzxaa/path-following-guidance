# Curvature-limited path tracking for a quadcopter and a fixed-wing plane

**The question.** A vehicle that can't turn tighter than a limit (a plane because of its minimum
speed, a quadcopter because of its maximum sideways acceleration) is given a planned Dubins path.
The path is only a line on a map. Which steering law keeps the vehicle on it when there is wind, and
what breaks when the plan is too tight or the vehicle is weaker than assumed?

This repo has:
- a **Dubins planner** (all six path types) and a waypoint-to-route builder,
- **four guidance laws**: pure pursuit, L1, vector field, and `lead_vf` (a small modification of the vector-field law, see Results),
- a **fast 2-D simulator** (constant airspeed, lateral-acceleration limit and lag, steady wind plus gusts),
- a **benchmark** (2 vehicles x 4 laws x 3 wind levels x 3 planned radii x 2 limit mismatches x seeds),
- an **ArduCopter SITL runner** (tested against fakes, and flown in ArduCopter SITL: nine flights, see "Real autopilot check"),
- **random routes**, a **tuning script** and a **held-out evaluation** with error bars,
- an **animation** script (GIF),
- 83 tests.

## Setup and run

```bash
pip install -r requirements.txt
python -m pytest tests -q                    # 83 tests
python -m ctrack.benchmark --quick           # ~30 s, exploratory run on the fixed route
python -m ctrack.tune                        # ~4 min: tune each law on TUNING routes only
python -m ctrack.evaluate                    # ~4 min: test default vs tuned on unseen routes
python -m ctrack.robustness                  # ~4 min: noise, delay, slower response, slower airspeed
python -m ctrack.ablations                   # ~4 min: which ingredient of each law matters
python -m ctrack.plots                       # writes results/fig1..fig3
```
`evaluate` writes `results/heldout.csv`, `results/heldout_stats.csv` and `results/fig4_heldout_scores.png`.
Add `--reuse` to `evaluate` to redo the statistics without re-running the simulations.

## Layout

```
ctrack/dubins.py      shortest Dubins path, sampling
ctrack/route.py       chain Dubins paths through waypoints; the benchmark route
ctrack/vehicles.py    fixed-wing and quadcopter parameters (r_min = V^2 / a_max)
ctrack/guidance.py    PurePursuit, L1, VectorField
ctrack/sim.py         2-D simulator with wind, gusts, lag, limit mismatch
ctrack/metrics.py     cross-track error, saturation, control effort
ctrack/scenarios.py   test conditions, route seeds, and the one function that runs a scenario
ctrack/tune.py        tune each law on tuning routes -> results/tuned_params.json
ctrack/evaluate.py    default vs tuned on unseen routes, paired stats with bootstrap intervals
ctrack/robustness.py  sensor noise / delay / slower response / slower airspeed on the test routes
ctrack/ablations.py   lead-time sweep, feed-forward on/off, L1 lookahead sweep
ctrack/estimate_lag.py  fit the real copter's response (gain, lag) from a SITL log
ctrack/paper_figures.py paper-style figures and tables generated from results/*.csv
paper/                claim and outline, related-work notes, LaTeX skeleton, figures, tables
ctrack/benchmark.py   the exploratory sweep on the fixed route -> results/benchmark.csv
ctrack/plots.py       figures
ctrack/animate.py     animated GIF of the four laws on the same route; optional CSV export
sitl/sitl_quad_runner.py   ArduCopter SITL via pymavlink
ctrack/sitl_control.py    the tested control step and log saving used by the runner
ctrack/plot_sitl.py       plot SITL logs
tests/                Dubins and guidance tests
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
- **Score** (lower is better) = RMS cross-track error + 0.25 x worst error, both in turning radii, +1 if the run did not finish. Errors are measured after the start transient.
- **Tuning routes** (random routes, seeds 1000-1005) and **test routes** (random, seeds 2000-2011) never overlap. A fixed "standard" route is kept as an extra check.
- **Conditions** (same on test): wind 0 / 15 / 30% of airspeed x planned radius 1.0 / 1.5 / 2.0 x true vehicle limit 100% / 80% of what the planner assumed.
- Each law is tuned separately for each vehicle. Default gains are always one of the candidates.
- Error bars come from resampling **routes** (scenarios on one route are not independent), and are wide with only 12 routes.

## Results (simulation only; test routes never used for tuning or design)

All numbers are the mean **score** (lower is better; see above) over the 12 test routes, tuned gains unless stated.
Figures: `fig4_heldout_scores.png`, `fig5_robustness.png`. Paired statistics: `results/heldout_stats.csv`.

| condition | fixed-wing: L1 | vector field | `lead_vf` | quadcopter: L1 | vector field | `lead_vf` |
|---|---|---|---|---|---|---|
| limit respected (radius >= x1.5, vehicle as rated) | 0.032 | 0.029 | 0.036 | 0.028 | 0.028 | 0.027 |
| vehicle 20% weaker than assumed | 0.057 | 0.084 | 0.061 | 0.051 | 0.070 | 0.050 |
| planned at the limit (radius x1.0) | 0.089 | 0.189 | 0.096 | 0.087 | 0.174 | 0.095 |

**1. Why the vector-field law fails at the limit (and why my first explanation was wrong).**
- I first guessed the correction field was too steep for the vehicle to follow, and built a "limit-aware" law that softened it when the path needed most of the turning. It barely helped (quadcopter at the limit 0.171 to 0.165, against 0.084 for L1), so it was removed.
- Direct tests on the tuning routes: a gentler field, a smaller approach angle and capping the feed-forward did nothing; removing the feed-forward made it much worse.
- A clean trace (no wind, no start offset): the law asks for 2.9-3.5 m/s^2 while the vehicle can give 2.0. It stays saturated for about 3 s and the error grows to 1.7 m. L1 never asks for more than the limit.
- Cause: the vehicle's lateral acceleration responds to a command with a lag (0.3-0.5 s here), but the law applies the path curvature for the point it is on, so it starts turning late. When the plan needs the vehicle's full turning ability there is no spare authority to catch up.

**2. The fix, `lead_vf`.** The same law, but the curvature feed-forward is read about 0.6 s **ahead** along the path. Reading the path heading ahead as well did not help. It was designed on the tuning routes only.
- At the limit it about **halves** the vector-field law's error (0.189 to 0.096 fixed-wing, 0.174 to 0.095 quadcopter). L1 is still slightly better there, and the paired interval excludes zero.
- With default gains (lead 0.6 s) it was the best of the four when the limit is respected (fixed-wing 0.025 against 0.032 for tuned L1) and when the fixed-wing is weaker than assumed.
- Tuning moved it toward the hard case and cost accuracy in easy conditions (fixed-wing, limit respected: 0.025 default, 0.036 tuned). **No single law wins everywhere.** Which one is "best" depends on which conditions you weight, and the tuning objective is a choice.

**3. Robustness** (`ctrack/robustness.py`, one imperfection at a time, fixed-wing typical / at limit):
- Sensor noise (2% of r_min position, 5% of airspeed velocity) barely changes any law. `lead_vf` is slightly worse under velocity noise.
- The plain vector-field law is fragile to timing: a 0.2 s sensor delay takes it from 0.026 to 0.064 (typical), a response lag twice as long to 0.087. `lead_vf` stays flat (0.034 to 0.034 with delay, 0.035 with double lag). L1 rises from 0.029 to 0.043 with double lag.
- At the limit with double lag: `lead_vf` 0.080, L1 0.093, vector field 0.216 (fixed-wing). Quadcopter: `lead_vf` and L1 are tied.
- A slower true airspeed makes every law score better, because the turn it needs is smaller. That is not a fair test of tolerance; ignore that row.

**4. Ablations** (`ctrack/ablations.py`, 6 test routes, other gains at default):
- `lead_time` has a sweet spot. Fixed-wing at the limit: 0 s = 0.189, 0.4 s = 0.127, 0.8 s = 0.096, 1.2 s = 0.103. Quadcopter at the limit is best at 0.6-0.8 s. Too much lead hurts again.
- The best lead is roughly 1.6 to 2.7x each vehicle's response lag (0.3 s quadcopter, 0.5 s fixed-wing). Two vehicles are not enough to call that a rule; it is a hypothesis.
- Removing the vector-field law's feed-forward roughly triples its error when the limit is respected (0.031 to 0.088).
- **L1's lookahead is a trade-off:** 1.0 s gives the best accuracy when the limit is respected (fixed-wing 0.014, better than any other setting of any law) but is poor at the limit (0.134). 2.5 s is best at the limit (0.085). L1's tuned values were picked with a tuning set weighted toward hard cases.

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

**Status.** The runner has been flown in ArduCopter SITL on macOS (nine flights; results in "Real autopilot check").
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


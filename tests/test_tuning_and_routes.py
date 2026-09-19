import math
import numpy as np
import pytest

from ctrack.guidance import DEFAULT_PARAMS, LAW_NAMES, make_law
from ctrack.route import build_route, random_route
from ctrack.scenarios import TEST_ROUTE_SEEDS, TUNE_ROUTE_SEEDS, get_path, run_scenario
from ctrack.vehicles import QUADCOPTER
from ctrack.evaluate import boot_ci


def test_random_route_is_repeatable_and_scales_with_radius():
    a = random_route(1.0, seed=5)
    b = random_route(1.0, seed=5)
    c = random_route(12.5, seed=5)
    assert a == b
    assert np.allclose(np.array(c), 12.5 * np.array(a))


def test_random_routes_differ_between_seeds():
    assert random_route(1.0, 1) != random_route(1.0, 2)


@pytest.mark.parametrize("seed", [1000, 1003, 2000, 2007])
@pytest.mark.parametrize("rf", [1.0, 2.0])
def test_random_route_builds_within_curvature_limit(seed, rf):
    path = get_path(QUADCOPTER, seed, rf)
    assert np.max(np.abs(path.kappa)) <= 1.0 / (rf * QUADCOPTER.r_min) + 1e-12
    assert path.length < 3 * sum(
        math.dist(w0, w1) for w0, w1 in zip(random_route(QUADCOPTER.r_min, seed)[:-1],
                                            random_route(QUADCOPTER.r_min, seed)[1:]))


def test_tuning_and_test_routes_do_not_overlap():
    assert set(TUNE_ROUTE_SEEDS).isdisjoint(TEST_ROUTE_SEEDS)


def test_make_law_uses_defaults_and_overrides():
    law = make_law("l1", 12.5)
    assert law.lookahead_time == DEFAULT_PARAMS["l1"]["lookahead_time"]
    law2 = make_law("l1", 12.5, {"lookahead_time": 4.0})
    assert law2.lookahead_time == 4.0
    vf = make_law("vector_field", 12.5, {"k_chi": 2.0, "feedforward": False})
    assert vf.k_chi == 2.0 and vf.feedforward is False


def test_make_law_rejects_bad_names_and_parameters():
    with pytest.raises(ValueError):
        make_law("nonsense", 12.5)
    with pytest.raises(ValueError):
        make_law("l1", 12.5, {"k_chi": 1.0})


@pytest.mark.parametrize("name", LAW_NAMES)
def test_scenario_score_is_finite_and_lower_for_finished_runs(name):
    s = run_scenario(QUADCOPTER, name, None, TUNE_ROUTE_SEEDS[0], (0.15, 1.5, 1.0), sim_seed=1)
    assert math.isfinite(s["score"]) and s["completed"] and s["score"] < 1.0


def test_stride_changes_cte_only_slightly():
    a = run_scenario(QUADCOPTER, "l1", None, 1000, (0.15, 1.5, 1.0), sim_seed=3, cte_stride=1)
    b = run_scenario(QUADCOPTER, "l1", None, 1000, (0.15, 1.5, 1.0), sim_seed=3, cte_stride=4)
    assert abs(a["rms_cte_m"] - b["rms_cte_m"]) < 0.05 * max(a["rms_cte_m"], 0.1)


def test_bootstrap_interval_contains_mean_and_is_ordered():
    m, lo, hi = boot_ci(np.array([0.1, 0.2, 0.15, 0.3, 0.05, 0.12]))
    assert lo <= m <= hi


def test_bootstrap_interval_of_constant_is_that_constant():
    m, lo, hi = boot_ci(np.full(8, 0.5))
    assert m == lo == hi == 0.5


def test_animation_and_csv_export_run(tmp_path):
    from ctrack.animate import make_animation
    gif, still = make_animation(str(tmp_path / "t.gif"), "quadcopter", ["l1"], wind=0.15,
                                radius_factor=1.5, mismatch=1.0, route_seed=1000, fps=5,
                                speedup=400.0, dpi=30, csv_dir=str(tmp_path / "csv"))
    assert (tmp_path / "t.gif").stat().st_size > 1000
    assert (tmp_path / "t.png").exists()
    assert (tmp_path / "csv" / "quadcopter_l1.csv").exists()
    assert (tmp_path / "csv" / "quadcopter_path.csv").exists()

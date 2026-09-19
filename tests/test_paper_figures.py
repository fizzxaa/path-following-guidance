import numpy as np
import pytest

from ctrack.paper_figures import fine_cte, sitl_summary
from ctrack.route import build_route


def test_fine_cte_is_zero_on_the_path_and_equals_the_offset_beside_it():
    path = build_route([(0, 0), (200, 0)], rho=10.0)
    x = np.array([10.0, 50.0, 120.0])
    assert np.allclose(fine_cte(path, x, np.zeros(3)), 0.0, atol=1e-6)
    assert np.allclose(fine_cte(path, x, np.full(3, 0.7)), 0.7, atol=1e-3)


def test_sitl_summary_skips_the_start_and_reports_rms_and_worst(tmp_path):
    t = np.arange(0, 60, 0.1)
    cte = np.where(t < 10, 5.0, 0.3)                       # a big start transient that must be ignored
    np.savetxt(tmp_path / "a.csv", np.column_stack([t, 0 * t, 0 * t, 0 * t, 0 * t, 0 * t, 0 * t, cte]),
               delimiter=",", comments="", header="t_s,x_east,y_north,v_east,v_north,a_cmd,path_idx,cte_m")
    s = sitl_summary([str(tmp_path / "a.csv")])
    assert s.iloc[0].rms_m == pytest.approx(0.3, abs=1e-6) and s.iloc[0].max_m == pytest.approx(0.3)

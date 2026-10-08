import numpy as np
import pytest

from ctrack.route import build_route
from ctrack.segments import ARC, STRAIGHT, TRANSITION, label_path, segment_rms, turning_share


def route():
    return build_route([[0, 0], [150, 0], [250, 80], [400, 80]], 25.0)


def test_labels_cover_and_arcs_match_curvature():
    p = route()
    lab = label_path(p, 1.0)
    assert set(np.unique(lab)) <= {STRAIGHT, TRANSITION, ARC}
    from ctrack.segments import arc_mask
    assert np.all(lab[arc_mask(p)] == ARC)
    assert (lab == ARC).any() and (lab == STRAIGHT).any() and (lab == TRANSITION).any()


def test_transition_only_near_arcs():
    p = route()
    lab = label_path(p, 1.0)
    s_arc = p.s[lab == ARC]
    for i in np.nonzero(lab == TRANSITION)[0][::20]:
        assert np.min(np.abs(s_arc - p.s[i])) <= p.rho + 1e-6
    for i in np.nonzero(lab == STRAIGHT)[0][::20]:
        assert np.min(np.abs(s_arc - p.s[i])) > p.rho


def test_window_zero_has_no_transition_and_wider_window_grows_it():
    p = route()
    assert not (label_path(p, 0.0) == TRANSITION).any()
    assert (label_path(p, 2.0) == TRANSITION).sum() > (label_path(p, 0.5) == TRANSITION).sum()


def test_segment_rms_recovers_known_offsets():
    # a vehicle that sits exactly 1.0 m left of the path on straights and 3.0 m outside on arcs
    p = route()
    lab = label_path(p, 1.0)
    idx = np.arange(0, p.n, 3)
    off = np.where(lab[idx] == ARC, 3.0, 1.0)
    # push each point sideways by `off` along the path normal
    x = p.x[idx] - off * np.sin(p.psi[idx]); y = p.y[idx] + off * np.cos(p.psi[idx])
    r = segment_rms(p, x, y, window_radii=1.0)
    assert r["straight"][0] == pytest.approx(1.0, abs=0.05)
    assert r["arc"][0] == pytest.approx(3.0, abs=0.1)


def test_turning_share_matches_geometry():
    p = route()
    assert 0.0 < turning_share(p) < 1.0
    straight = build_route([[0, 0], [100, 0]], 25.0)
    assert turning_share(straight) == 0.0


def test_windows_variant_matches_single():
    from ctrack.segments import segment_rms_windows
    p = route()
    idx = np.arange(0, p.n, 3)
    x = p.x[idx] + 0.4; y = p.y[idx] - 0.2
    multi = segment_rms_windows(p, x, y, windows=(0.5, 1.0))
    for w in (0.5, 1.0):
        single = segment_rms(p, x, y, window_radii=w)
        for k in single:
            assert multi[w][k][1] == single[k][1]
            assert multi[w][k][0] == pytest.approx(single[k][0], nan_ok=True)

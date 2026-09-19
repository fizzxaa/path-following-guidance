"""Plot (and optionally animate) SITL logs written by sitl/sitl_quad_runner.py.

    python -m ctrack.plot_sitl results/sitl_pure_pursuit.csv results/sitl_l1.csv results/sitl_vector_field.csv
    python -m ctrack.plot_sitl results/sitl_l1.csv --gif results/sitl_replay.gif     # a video of the flight

Each run's path is shifted so it starts at (0, 0), so runs that started in different
places can be drawn together.
"""
from __future__ import annotations

import argparse
import os

import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon


def load(csv_path):
    d = np.genfromtxt(csv_path, delimiter=",", names=True)
    p = np.genfromtxt(os.path.splitext(csv_path)[0] + "_path.csv", delimiter=",", names=True)
    return d, p


def make_gif(logs, out_gif, fps=20, speedup=6.0, dpi=70):
    """Replay the flown tracks against the planned path, all runs on the same clock."""
    runs = []
    for f in logs:
        d, p = load(f)
        runs.append((os.path.splitext(os.path.basename(f))[0], d, p))
    p0 = runs[0][2]
    x0, y0 = p0["x_east"][0], p0["y_north"][0]
    px, py = p0["x_east"] - x0, p0["y_north"] - y0
    span = max(px.max() - px.min(), py.max() - py.min())
    size = 0.03 * span
    tri = np.array([[1.0, 0.0], [-0.7, 0.5], [-0.7, -0.5]]) * size
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(px, py, "k--", lw=1.2, label="planned path")
    parts = []
    for k, (name, d, p) in enumerate(runs):
        trail, = ax.plot([], [], color=colors[k % len(colors)], lw=1.5, label=name)
        poly = Polygon(tri, closed=True, color=colors[k % len(colors)], zorder=5)
        ax.add_patch(poly)
        parts.append((trail, poly))
    txt = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=8, family="monospace",
                  bbox=dict(fc="white", ec="0.8", alpha=0.85))
    pad = 0.08 * span
    ax.set_xlim(px.min() - pad, px.max() + pad); ax.set_ylim(py.min() - pad, py.max() + pad)
    ax.set_aspect("equal"); ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")
    ax.set_title("SITL replay"); ax.legend(fontsize=8, loc="lower right")
    fig.tight_layout()

    t_end = max(d["t_s"][-1] for _, d, _ in runs)
    step = speedup / fps
    times = list(np.arange(0.0, t_end, step)) + [t_end] * (2 * fps)

    def update(tf):
        lines = [f"t = {tf:6.1f} s"]
        for (name, d, p), (trail, poly) in zip(runs, parts):
            i = min(int(np.searchsorted(d["t_s"], tf)), len(d["t_s"]) - 1)
            xs, ys = d["x_east"] - x0, d["y_north"] - y0
            trail.set_data(xs[:i + 1], ys[:i + 1])
            course = math.atan2(d["v_north"][i], d["v_east"][i])
            c, s_ = math.cos(course), math.sin(course)
            poly.set_xy(tri @ np.array([[c, -s_], [s_, c]]).T + np.array([xs[i], ys[i]]))
            lines.append(f"{name[:14]:14s} error {d['cte_m'][i]:5.2f} m")
        txt.set_text("\n".join(lines))
        return []

    os.makedirs(os.path.dirname(out_gif) or ".", exist_ok=True)
    FuncAnimation(fig, update, frames=times, blit=False).save(out_gif, writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(fig)
    return out_gif


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--out", default="results/sitl_plot.png")
    ap.add_argument("--gif", default=None, help="also write an animated replay to this file")
    ap.add_argument("--speedup", type=float, default=6.0, help="replay speed, flight seconds per video second")
    args = ap.parse_args(argv)

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5.5))
    ref_drawn = False
    print(f"{'run':30s} {'rms (m)':>8s} {'max (m)':>8s}")
    for f in args.logs:
        d, p = load(f)
        x0, y0 = p["x_east"][0], p["y_north"][0]
        if not ref_drawn:
            a1.plot(p["x_east"] - x0, p["y_north"] - y0, "k--", lw=1.3, label="planned path")
            ref_drawn = True
        name = os.path.splitext(os.path.basename(f))[0]
        a1.plot(d["x_east"] - x0, d["y_north"] - y0, lw=1.3, label=name)
        a2.plot(d["t_s"], d["cte_m"], lw=1.2, label=name)
        print(f"{name:30s} {np.sqrt(np.mean(d['cte_m'] ** 2)):8.2f} {d['cte_m'].max():8.2f}   (whole run, including start)")
    a1.set_aspect("equal"); a1.set_xlabel("east (m)"); a1.set_ylabel("north (m)")
    a1.set_title("SITL: planned path and flown tracks"); a1.legend(fontsize=8)
    a2.set_xlabel("time (s)"); a2.set_ylabel("distance from planned path (m)")
    a2.set_title("Cross-track error"); a2.legend(fontsize=8); a2.grid(alpha=0.3)
    fig.tight_layout()
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    fig.savefig(args.out, dpi=140)
    print("wrote", args.out)
    if args.gif:
        make_gif(args.logs, args.gif, speedup=args.speedup)
        print("wrote", args.gif)


if __name__ == "__main__":
    main()

"""Figures and tables for a paper, at IEEE column widths, generated from the result files.

    python -m ctrack.paper_figures                                   # simulation figures + tables
    python -m ctrack.paper_figures --sitl results/sitl_l1_r1.csv results/sitl_vector_field_r1.csv results/sitl_lead_vf_r1.csv

Writes paper/figures/*.pdf and *.png and paper/tables.md. Every number in tables.md comes from
results/*.csv, so nothing has to be typed by hand.
"""
from __future__ import annotations

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .evaluate import boot_ci, per_route_means
from .guidance import LAW_NAMES, make_law
from .metrics import cross_track_errors
from .robustness import CONDITIONS as ROB_CONDITIONS, table as rob_table
from .scenarios import get_path
from .sim import SimConfig, simulate
from .vehicles import VEHICLES

COL, DBL = 3.4, 7.0                       # IEEE column / page width, inches
COLORS = {"pure_pursuit": "#1f77b4", "l1": "#2ca02c", "vector_field": "#d62728", "lead_vf": "#9467bd"}
LABELS = {"pure_pursuit": "pure pursuit", "l1": "L1", "vector_field": "vector field", "lead_vf": "lead vector field"}
GROUPS = {
    "limit respected": lambda d: (d.radius_factor >= 1.5) & (d.mismatch == 1.0),
    "vehicle 20% weaker": lambda d: (d.radius_factor >= 1.5) & (d.mismatch == 0.8),
    "planned at limit": lambda d: d.radius_factor == 1.0,
}


def style():
    plt.rcParams.update({"font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8, "legend.fontsize": 7,
                         "xtick.labelsize": 7, "ytick.labelsize": 7, "axes.linewidth": 0.6,
                         "lines.linewidth": 1.0, "pdf.fonttype": 42, "figure.dpi": 150})


def save(fig, outdir, name):
    os.makedirs(outdir, exist_ok=True)
    fig.savefig(os.path.join(outdir, name + ".pdf"), bbox_inches="tight")
    fig.savefig(os.path.join(outdir, name + ".png"), bbox_inches="tight", dpi=200)
    plt.close(fig)


def fine_cte(path, x, y, spacing=0.05):
    """Distance to the path measured against a finely resampled copy, so the plot is not jittery.

    The main metrics use the 0.5 m path samples (accurate to about 0.25 m); that is fine for
    averages but shows up as a visible band in a time plot.
    """
    class _P:  # only .x and .y are needed by cross_track_errors
        pass
    s_ = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(path.x), np.diff(path.y)))])
    n = int(s_[-1] / spacing) + 1
    fine = _P()
    grid = np.linspace(0.0, s_[-1], n)
    fine.x, fine.y = np.interp(grid, s_, path.x), np.interp(grid, s_, path.y)
    return cross_track_errors(fine, x, y)


def fig_mechanism(outdir, route_seed=1001, t0=8.0, t1=30.0):
    """Why the vector-field law fails at the limit: it asks for more than the vehicle can give."""
    veh = VEHICLES["quadcopter"]
    path = get_path(veh, route_seed, 1.0)
    cfg = SimConfig(wind_speed=0.0, limit_mismatch=1.0, start_offset_frac=0.0, start_heading_deg=0.0)
    runs = {}
    for law in ("l1", "vector_field", "lead_vf"):
        res = simulate(path, veh, make_law(law, veh.r_min), cfg, seed=0)
        runs[law] = (res, fine_cte(path, res.x, res.y))
    fig, (a, b) = plt.subplots(1, 2, figsize=(DBL, 2.5))
    for law, (res, cte) in runs.items():
        m = (res.t >= t0) & (res.t <= t1)
        a.plot(res.t[m], cte[m], color=COLORS[law], label=LABELS[law])
    a.set_xlabel("time (s)"); a.set_ylabel("distance from planned path (m)"); a.legend(frameon=False)
    a.set_title("(a) tracking error, plan at the vehicle's limit")
    for law in ("vector_field", "lead_vf"):
        res, _ = runs[law]
        m = (res.t >= t0) & (res.t <= t1)
        b.plot(res.t[m], np.abs(res.a_cmd[m]), color=COLORS[law], label=LABELS[law])
    b.axhline(veh.a_max, color="k", ls="--", lw=0.8, label="vehicle limit")
    b.set_xlabel("time (s)"); b.set_ylabel("commanded $|a|$ (m/s$^2$)")
    b.legend(frameon=False); b.set_title("(b) what each law asks for")
    fig.tight_layout()
    save(fig, outdir, "fig_mechanism")


def fig_heldout(df, outdir):
    fig, axes = plt.subplots(1, 2, figsize=(DBL, 2.5), sharey=False)
    for ax, vname in zip(axes, ("fixed_wing", "quadcopter")):
        for i, law in enumerate(LAW_NAMES):
            vals, los, his = [], [], []
            for gname, g in GROUPS.items():
                x = per_route_means(df, vname, "tuned", law, g)
                m, lo, hi = boot_ci(x)
                vals.append(m); los.append(m - lo); his.append(hi - m)
            pos = np.arange(len(GROUPS)) + (i - 1.5) * 0.2
            ax.bar(pos, vals, 0.19, yerr=[los, his], color=COLORS[law], label=LABELS[law],
                   error_kw={"lw": 0.6, "capsize": 1.5})
        ax.set_xticks(range(len(GROUPS))); ax.set_xticklabels(list(GROUPS), fontsize=7)
        ax.set_title(vname.replace("_", "-")); ax.set_ylabel("score (lower is better)")
    axes[0].legend(frameon=False, ncol=2)
    fig.tight_layout()
    save(fig, outdir, "fig_heldout")


def fig_robustness(rob, outdir):
    perts = ["none", "sensor delay 0.2 s", "response lag x2", "combined"]
    fig, axes = plt.subplots(1, 2, figsize=(DBL, 2.5))
    for ax, vname in zip(axes, ("fixed_wing", "quadcopter")):
        t = rob_table(rob, vname, "at limit")
        for i, law in enumerate(LAW_NAMES):
            ax.bar(np.arange(len(perts)) + (i - 1.5) * 0.2, [t.loc[p, law] for p in perts], 0.19,
                   color=COLORS[law], label=LABELS[law])
        ax.set_xticks(range(len(perts))); ax.set_xticklabels([p.replace(" ", "\n", 1) for p in perts], fontsize=7)
        ax.set_title(vname.replace("_", "-") + ", plan at the limit"); ax.set_ylabel("mean score")
        ax.set_ylim(0, ax.get_ylim()[1] * 1.25)          # leave room for the legend above the bars
    axes[0].legend(frameon=False, ncol=4, loc="upper left", columnspacing=0.8, handlelength=1.0)
    fig.tight_layout()
    save(fig, outdir, "fig_robustness")


def sitl_summary(paths, skip_s=10.0):
    rows = []
    for p in paths:
        d = np.genfromtxt(p, delimiter=",", names=True)
        keep = d["t_s"] >= skip_s
        c = d["cte_m"][keep]
        rows.append({"log": os.path.splitext(os.path.basename(p))[0],
                     "rms_m": float(np.sqrt(np.mean(c ** 2))), "max_m": float(c.max())})
    return pd.DataFrame(rows)


def fig_sitl(paths, outdir):
    fig, (a, b) = plt.subplots(1, 2, figsize=(DBL, 2.8))
    for k, p in enumerate(paths):
        d = np.genfromtxt(p, delimiter=",", names=True)
        pth = np.genfromtxt(os.path.splitext(p)[0] + "_path.csv", delimiter=",", names=True)
        x0, y0 = pth["x_east"][0], pth["y_north"][0]
        name = os.path.splitext(os.path.basename(p))[0]
        if k == 0:
            a.plot(pth["x_east"] - x0, pth["y_north"] - y0, "k--", lw=0.8, label="planned")
        a.plot(d["x_east"] - x0, d["y_north"] - y0, lw=0.9, label=name)
        b.plot(d["t_s"], d["cte_m"], lw=0.8, label=name)
    a.set_aspect("equal"); a.set_xlabel("east (m)"); a.set_ylabel("north (m)"); a.legend(frameon=False)
    a.set_title("(a) planned path and flown tracks (ArduCopter SITL)")
    b.set_xlabel("time (s)"); b.set_ylabel("distance from planned path (m)"); b.set_title("(b) tracking error")
    fig.tight_layout()
    save(fig, outdir, "fig_sitl")


def md_table(df, index=True, fmt=".4f"):
    """Markdown table without needing the optional 'tabulate' package."""
    cols = ([df.index.name or ""] if index else []) + [str(c) for c in df.columns]
    lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    for idx, row in df.iterrows():
        cells = ([str(idx)] if index else []) + [format(v, fmt) if isinstance(v, float) else str(v) for v in row.tolist()]
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def write_tables(df, stats, rob, sitl, out_md):
    lines = ["# Tables generated from results/*.csv (do not edit by hand)\n"]
    lines.append("## Held-out test routes: mean score, tuned gains (95% interval over the 12 routes)\n")
    for vname in ("fixed_wing", "quadcopter"):
        lines.append(f"\n**{vname}**\n\n| group | " + " | ".join(LABELS[l] for l in LAW_NAMES) + " |\n|---|" + "---|" * len(LAW_NAMES))
        for gname, g in GROUPS.items():
            cells = []
            for law in LAW_NAMES:
                m, lo, hi = boot_ci(per_route_means(df, vname, "tuned", law, g))
                cells.append(f"{m:.3f} [{lo:.3f}, {hi:.3f}]")
            lines.append(f"| {gname} | " + " | ".join(cells) + " |")
    lines.append("\n## Paired differences on the held-out routes (all conditions)\n")
    s = stats[(stats["subset"] == "all conditions") & stats.comparison.str.contains("lead_vf")]
    lines.append("| vehicle | comparison | mean diff | 95% interval | reading |\n|---|---|---|---|---|")
    for _, r in s.iterrows():
        lines.append(f"| {r.vehicle} | {r.comparison} | {r.mean_diff:.4f} | [{r.ci_lo:.4f}, {r.ci_hi:.4f}] | {r.reading} |")
    lines.append("\n## Robustness: mean score, plan at the limit, tuned gains\n")
    for vname in ("fixed_wing", "quadcopter"):
        lines.append(f"\n**{vname}**\n\n" + md_table(rob_table(rob, vname, "at limit")))
    if sitl is not None:
        lines.append("\n## ArduCopter SITL flights (RMS and worst distance from the planned path, m, after the first 10 s)\n")
        lines.append(md_table(sitl, index=False, fmt=".2f"))
    with open(out_md, "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--heldout", default="results/heldout.csv")
    ap.add_argument("--stats", default="results/heldout_stats.csv")
    ap.add_argument("--robustness", default="results/robustness.csv")
    ap.add_argument("--sitl", nargs="*", default=[], help="SITL logs for the real-autopilot figure and table")
    ap.add_argument("--out", default="paper")
    args = ap.parse_args()
    style()
    fdir = os.path.join(args.out, "figures")
    df, stats, rob = pd.read_csv(args.heldout), pd.read_csv(args.stats), pd.read_csv(args.robustness)
    fig_mechanism(fdir); fig_heldout(df, fdir); fig_robustness(rob, fdir)
    sitl = None
    if args.sitl:
        fig_sitl(args.sitl, fdir)
        sitl = sitl_summary(args.sitl)
    write_tables(df, stats, rob, sitl, os.path.join(args.out, "tables.md"))
    print("wrote", fdir, "and", os.path.join(args.out, "tables.md"))


if __name__ == "__main__":
    main()

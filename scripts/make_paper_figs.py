"""Figures for the paper draft, from the rebuilt results (parses the report tables)."""
import re, sys, os
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, ".")
from ctrack import paper_figures as pf

OUT = "paper_fig"
LAWS = ["pure_pursuit", "l1", "vector_field", "lead_vf", "adaptive_vf", "carrot_chasing"]
NICE = {"pure_pursuit": "pure pursuit", "l1": "L1", "vector_field": "vector field", "lead_vf": "lead vector field",
        "adaptive_vf": "adaptive VF (ours)", "carrot_chasing": "carrot chasing (ours)"}
COL = {"pure_pursuit": "#7f7f7f", "l1": "#1f77b4", "vector_field": "#d62728", "lead_vf": "#2ca02c",
       "adaptive_vf": "#9467bd", "carrot_chasing": "#ff7f0e"}
plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False})

# 1. mechanism trace
pf.style(); pf.COLORS.update({'l1':COL['l1'],'vector_field':COL['vector_field'],'lead_vf':COL['lead_vf']}); pf.fig_mechanism(OUT)

# 2. segment RMS (quadcopter + fixed wing), all conditions
txt = open("results/segments_report_6laws.md").read()
def seg_table(vehicle):
    sec = txt.split("## " + vehicle)[1].split("### planned at limit")[0].split("### all conditions")[1]
    rows = {}
    for line in sec.splitlines():
        m = re.match(r"\| (\w+) \|(.*)\|", line)
        if m and m.group(1) in LAWS:
            cells = re.findall(r"([\d.]+) \[([\d.]+), ([\d.]+)\]", m.group(2))
            rows[m.group(1)] = [tuple(map(float, c)) for c in cells]  # all, straight, transition, arc
    return rows
fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.7))
for ax, veh, ttl in zip(axs, ["quadcopter", "fixed_wing"], ["Quadcopter model", "Fixed-wing model"]):
    rows = seg_table(veh); names = ["straight", "transition", "arc"]; w = 0.13
    for k, law in enumerate(LAWS):
        v = [rows[law][i + 1] for i in range(3)]
        y = [a[0] for a in v]; err = [[a[0] - a[1] for a in v], [a[2] - a[0] for a in v]]
        ax.bar(np.arange(3) + (k - 2.5) * w, y, w, yerr=err, color=COL[law], label=NICE[law], capsize=1, error_kw={"lw": .6})
    ax.set_xticks(range(3)); ax.set_xticklabels(names); ax.set_ylabel("RMS cross-track error (m)"); ax.set_title(ttl)
axs[0].legend(frameon=False, fontsize=6.5)
fig.tight_layout(); fig.savefig(f"{OUT}/fig_segments.png", dpi=200); plt.close(fig)

# 3. planner vs controller
pvc = open("results/planner_vs_controller.md").read()
def pvc_table(vehicle):
    sec = pvc.split("## " + vehicle)[1].split("- controller")[0]
    rows = {}
    for line in sec.splitlines():
        m = re.match(r"\| (\w+) \| ([\d.]+) \| ([\d.]+) \| ([\d.]+) \|", line)
        if m and m.group(1) in LAWS: rows[m.group(1)] = [float(m.group(i)) for i in (2, 3, 4)]
    return rows
fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.6))
for ax, veh, ttl in zip(axs, ["quadcopter", "fixed_wing"], ["Quadcopter model", "Fixed-wing model"]):
    r = pvc_table(veh)
    for j, (lab, hatch) in enumerate([("Dubins route", None), ("smoothed route, same gains", "//"), ("smoothed route, retuned", "xx")]):
        ax.bar(np.arange(6) + (j - 1) * 0.27, [r[l][j] for l in LAWS], 0.27, color=[COL[l] for l in LAWS],
               hatch=hatch, edgecolor="white", lw=0.5)
    ax.set_xticks(range(6)); ax.set_xticklabels([NICE[l].replace(" (ours)", "*").replace("lead vector field", "lead VF").replace("vector field", "VF") for l in LAWS], rotation=30, ha="right")
    ax.set_ylabel("whole-path RMS (m)"); ax.set_title(ttl)
from matplotlib.patches import Patch
axs[0].set_ylim(0, 0.68); axs[1].set_ylim(0, 3.2)
axs[0].legend(handles=[Patch(fc="0.6", label="Dubins route"), Patch(fc="0.6", hatch="//", ec="w", label="smoothed, same gains"),
                       Patch(fc="0.6", hatch="xx", ec="w", label="smoothed, retuned")], frameon=False, fontsize=6.5)
fig.tight_layout(); fig.savefig(f"{OUT}/fig_planner.png", dpi=200); plt.close(fig)

# 4. SITL, planned radius 12.5 m
fig, (a, b) = plt.subplots(1, 2, figsize=(7.2, 2.8), gridspec_kw={"width_ratios": [1, 1.2]})
runs = [("l1", "results/sitl_l1_r1.csv"), ("vector_field", "results/sitl_vector_field_r1.csv"), ("lead_vf", "results/sitl_lead_vf_r1.csv")]
for k, (law, f) in enumerate(runs):
    d = np.genfromtxt(f, delimiter=",", names=True); p = np.genfromtxt(f[:-4] + "_path.csv", delimiter=",", names=True)
    if k == 0: a.plot(p["x_east"] - p["x_east"][0], p["y_north"] - p["y_north"][0], "k--", lw=1, label="planned")
    a.plot(d["x_east"] - p["x_east"][0], d["y_north"] - p["y_north"][0], color=COL[law], lw=1, label=NICE[law])
    b.plot(d["t_s"], d["cte_m"], color=COL[law], lw=0.9, label=NICE[law])
a.set_aspect("equal"); a.set_xlabel("east (m)"); a.set_ylabel("north (m)"); a.legend(frameon=False, fontsize=6.5)
b.set_xlabel("time (s)"); b.set_ylabel("distance from planned path (m)")
fig.tight_layout(); fig.savefig(f"{OUT}/fig_sitl.png", dpi=200); plt.close(fig)
print(os.listdir(OUT))

#!/usr/bin/env python3
"""The paper's free-flight trajectory-tracking tables (simulation and experiment), one builder for both.

    /usr/bin/python3 results/utils/make_latex_table.py [--paper <main.tex>]

Simulation: reads results/simulation_results/tables/free_flight_tracking_sim.json (build_free_flight.py),
writes results/simulation_results/tables/free_flight_tracking_sim.tex, and with --paper replaces the
table labelled tab:sim_free_flight_tracking in main.tex. Each cell is the mean over the completed runs
of the per-run RMSE.

Experiment: experiment_tracking.py calls exp_table() / splice() with the selected run of each setting.

Both tables: two decimals; within every (trajectory, speed) block the lowest value of each column is
bold, compared at the printed precision (a tie bolds every tied entry); a setting with no data shows "--".
"""
import argparse
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(os.path.abspath(os.path.join(HERE, "..")), "simulation_results")
SIM_JSON = os.path.join(SIM, "tables", "free_flight_tracking_sim.json")
SIM_TEX = os.path.join(SIM, "tables", "free_flight_tracking_sim.tex")
SIM_LABEL = "tab:sim_free_flight_tracking"
EXP_LABEL = "tab:exp_free_flight_tracking"

COLS = ["platform_x_mm", "platform_y_mm", "platform_z_mm", "platform_heading_deg", "platform_roll_deg",
        "platform_pitch_deg", "ee_x_mm", "ee_y_mm", "ee_z_mm", "ee_heading_deg", "q1_deg", "q2_deg", "q3_deg",
        "q4_deg"]
SHAPES = [("circle", "Circle"), ("figure8", "Figure-8")]
SPEEDS = [0.10, 0.13, 0.20]
PROPOSED = ("whole_body_l1", "Proposed")
GEO = ("geometric_l1", r"Geo-$\mathcal{L}_1$~\cite{cai2025experiment}")
MAC = ("modular_adaptive", r"MAC~\cite{yadav2024modular}")
DEG = r"($^\circ$)"
SYMBOLS = [
    r"$r_{0,x}, r_{0,y}, r_{0,z}$: components of the platform position $\boldsymbol{r}_0$ in $\{I\}$;",
    r"$\psi_0, \phi_0, \theta_0$: platform heading, roll, and pitch, taken as the body-axis components of the rotation vector of $\boldsymbol{R}_{0,d}^T \boldsymbol{R}_0$, where $\boldsymbol{R}_{0,d}$ is the platform attitude of the reference trajectory;",
    r"$r_{e,x}, r_{e,y}, r_{e,z}$: components of the end-effector position $\boldsymbol{r}_e$ in $\{I\}$;",
    r"$\psi_e$: end-effector heading, i.e., the azimuth of $\boldsymbol{b}_{1,e}$;",
    r"$q_1, \dots, q_4$: joint angles of the manipulator.",
]
SPEED_NOTE = r"\item[1] Mean speed of the end-effector along its path."


def build_table(cells, methods, caption, label, notes, sep="5pt"):
    """cells: {(shape, 'v.vv', method): {col: value}}; notes: tablenotes lines after the \\item[] lead-in."""
    L = [r"\begin{table}[htbp]", r"\centering", rf"\caption{{{caption}}}", rf"\label{{{label}}}", r"\footnotesize",
         rf"\setlength{{\tabcolsep}}{{{sep}}}", r"\begin{threeparttable}",
         r"\begin{tabular}{c c l c c c c c c c c c c c c c c}", r"\toprule",
         r"& & & \multicolumn{6}{c}{Platform} & \multicolumn{4}{c}{End-effector} & \multicolumn{4}{c}{Joint} \\",
         r"\cmidrule(lr){4-9} \cmidrule(lr){10-13} \cmidrule(lr){14-17}",
         r"Trajectory & $\bar{v}$\tnote{1} & Method & $r_{0,x}$ & $r_{0,y}$ & $r_{0,z}$ & $\psi_0$ & $\phi_0$ & $\theta_0$"
         r" & $r_{e,x}$ & $r_{e,y}$ & $r_{e,z}$ & $\psi_e$ & $q_1$ & $q_2$ & $q_3$ & $q_4$ \\",
         r"& (m/s) & & (mm) & (mm) & (mm) & " + " & ".join([DEG] * 3) + r" & (mm) & (mm) & (mm) & "
         + " & ".join([DEG] * 5) + r" \\",
         r"\midrule"]
    even = len(methods) % 2 == 0
    mid = len(methods) // 2 - 1 if even else len(methods) // 2
    centre = (lambda t: rf"\smash{{\raisebox{{-0.5\baselineskip}}{{{t}}}}}") if even else (lambda t: t)
    for gi, (shape, sname) in enumerate(SHAPES):
        if gi:
            L.append(r"\addlinespace[0.8em]")
        for vi, v in enumerate(SPEEDS):
            vs = f"{v:.2f}"
            if vi:
                L.append(r"\addlinespace")
            best = {}
            for c in COLS:
                vals = [round(cells[(shape, vs, m)][c], 2) for m, _ in methods if (shape, vs, m) in cells]
                best[c] = min(vals) if len(vals) > 1 else None
            for mi, (m, mname) in enumerate(methods):
                c1 = centre(sname) if (vi == 1 and mi == mid) else ""
                c2 = centre(vs) if mi == mid else ""
                r = cells.get((shape, vs, m))
                if r is None:
                    vals = ["--"] * len(COLS)
                else:
                    vals = []
                    for c in COLS:
                        txt = f"{r[c]:.2f}"
                        vals.append(rf"\textbf{{{txt}}}" if best[c] is not None and round(r[c], 2) == best[c] else txt)
                L.append(f"{c1} & {c2} & {mname} & " + " & ".join(vals) + r" \\")
    L += [r"\bottomrule", r"\end{tabular}", r"\begin{tablenotes}", r"\footnotesize"] + notes + \
         [r"\end{tablenotes}", r"\end{threeparttable}", r"\end{table}"]
    return "\n".join(L) + "\n"


def sim_table(summary, sep="5pt"):
    cells = {(r["shape"], f"{r['mean_speed_mps']:.2f}", r["method"]): r for r in summary}
    notes = ([r"\item[] Each entry is the root-mean-square (RMS) tracking error of the listed quantity with respect to the dynamically compatible whole-body reference trajectory."]
             + SYMBOLS + [r"Geo-$\mathcal{L}_1$: geometric control with $\mathcal{L}_1$ adaptive augmentation;",
                          r"MAC: modular adaptive control.", SPEED_NOTE])
    return build_table(cells, [PROPOSED, GEO, MAC], "RMSE of free-flight trajectory tracking in simulation.",
                       SIM_LABEL, notes, sep)


def exp_table(cells, sep="5pt"):
    notes = [rf"\item[] Symbols as in Table~\ref{{{SIM_LABEL}}}. Each entry is from the completed flight with the lowest end-effector position RMSE among the repeated flights of that setting.",
             SPEED_NOTE]
    return build_table(cells, [PROPOSED, GEO], "RMSE of free-flight trajectory tracking in experiment.",
                       EXP_LABEL, notes, sep)


def splice(paper, tex, label, anchor=None):
    """Replace the table carrying `label` in `paper`; if absent, insert it right after the `anchor` line."""
    s = open(paper).read()
    if s.count(label) > 1:
        raise SystemExit(f"{label} appears more than once in {paper}")
    if label in s:
        i = s.index(label)
        a = s.rindex(r"\begin{table}", 0, i)
        b = s.index(r"\end{table}", i) + len(r"\end{table}")
        s = s[:a] + tex.rstrip("\n") + s[b:]
    else:
        if anchor is None or s.count(anchor) != 1:
            raise SystemExit(f"cannot place {label}: anchor {anchor!r} not found exactly once")
        i = s.index(anchor) + len(anchor)
        s = s[:i] + "\n\n" + tex.rstrip("\n") + "\n" + s[i:]
    open(paper, "w").write(s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--paper", default=None, help="main.tex to update in place")
    ap.add_argument("--sep", default="5pt")
    a = ap.parse_args()
    tex = sim_table(json.load(open(SIM_JSON))["mean_over_runs"], a.sep)
    open(SIM_TEX, "w").write(tex)
    print("wrote", SIM_TEX)
    if a.paper:
        splice(a.paper, tex, SIM_LABEL)
        print("updated", a.paper)


if __name__ == "__main__":
    main()

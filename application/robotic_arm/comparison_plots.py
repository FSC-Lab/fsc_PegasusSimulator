#!/usr/bin/env python3
"""Score and plot one comparison task: whole-body vs geometric+L1.

    /usr/bin/python3 comparison_plots.py --task hover_arm_swing

Reads results/<task>/{wb,l1}.npz (written by comparison_driver.py) and writes,
into the same directory: metrics.json, summary.md, and three figures.

WHAT IS BEING MEASURED
----------------------
Both runs were handed the identical commanded motion, so every error below is
measured against the SAME reference and the two columns are directly
comparable without any alignment step.

  base error   measured base position (mocap odometry) minus the commanded
               base position.
  EE error     measured end-effector position minus the commanded one.  The
               measured EE is FORWARD KINEMATICS of the measured base pose and
               the measured joint angles, on the controller's own chain
               (transition_planner.make_params_t650, EE = the gripper grasp
               point) -- the same model both laws are built on, so neither is
               scored against its own idea of where the EE is.
  EE heading   angle between the measured and commanded EE heading directions.
  joint error  measured joint angles minus the commanded ones.  Worth reading
               even though it is not the headline: the two stacks realise the
               same joint COMMAND through different actuation (position servo
               vs whole-body torque), and this is where that shows.

The "ball" figures answer the user's question directly: the error cloud over
the scored window, with a sphere of radius equal to its RMS norm.  Smaller ball
= tighter hold.
"""

import argparse
import json
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D                          # noqa: F401,E402

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.abspath(os.path.join(
    _HERE, "..", "..", "extensions", "fsc_aerial_manipulation")))
from fsc_aerial_manipulation.robotic_arm.utils_planner import (   # noqa: E402
    transition_planner as TP,
)

# ---- design tokens -------------------------------------------------------
# Categorical slots 1 and 2 of the validated reference palette (blue, orange).
# Slots 1-3 are the documented all-pairs-safe subset, which is what a scatter
# needs; two series stay inside it.  The REFERENCE is drawn in muted ink, not a
# third series colour -- it is the target, not a competitor.
C_WB = "#2a78d6"
C_L1 = "#eb6834"
C_REF = "#8a8984"
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e2e1dc"

SERIES = {"wb": ("Whole-body", C_WB), "l1": ("Geometric + L1", C_L1)}


def _style(ax, xlabel=None, ylabel=None, title=None):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.7, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=8, length=3, color=GRID)
    if xlabel:
        ax.set_xlabel(xlabel, color=INK2, fontsize=9)
    if ylabel:
        ax.set_ylabel(ylabel, color=INK2, fontsize=9)
    if title:
        ax.set_title(title, color=INK, fontsize=10, loc="left", pad=6)


def quat_to_R(qw, qx, qy, qz):
    n = np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


class Run(object):
    """One recorded flight, with the derived error signals."""

    def __init__(self, path, params):
        z = np.load(path, allow_pickle=True)
        self.path = path
        self.controller = str(z["controller"])
        self.label = SERIES[self.controller][0]
        self.color = SERIES[self.controller][1]
        self.aborted = bool(z["aborted"])
        self.abort_reason = str(z["abort_reason"])
        self.events = [str(e) for e in z["events"]]
        cols = [str(c) for c in z["columns"]]
        log = z["log"]
        self.c = {n: log[:, i] for i, n in enumerate(cols)}
        self.n = len(log)
        self.mark_names = [str(s) for s in z["tb_mark_names"]]
        self.mark_t = z["tb_mark_t"]
        self.dt = float(z["tb_dt"])

        c = self.c
        self.t = c["t"]
        self.p = np.stack([c["x"], c["y"], c["z"]], axis=1)
        self.p_ref = np.stack([c["ref_x"], c["ref_y"], c["ref_z"]], axis=1)
        self.q_ref = np.stack([c[f"q_ref{i+1}"] for i in range(4)], axis=1)
        self.q = np.stack([c[f"q_meas{i+1}"] for i in range(4)], axis=1)
        self.tau = np.stack([c[f"tau_meas{i+1}"] for i in range(4)], axis=1)
        self.ee_ref = np.stack([c["r_ed_x"], c["r_ed_y"], c["r_ed_z"]], axis=1)
        self.b1de_ref = np.stack([c["b1de_x"], c["b1de_y"], c["b1de_z"]], axis=1)
        self.direct = c["direct"] > 0.5
        self.k = c["k_task"].astype(int)

        # measured EE by forward kinematics on the MEASURED base pose and the
        # MEASURED joints -- the same chain both controllers model.
        ee = np.full((self.n, 3), np.nan)
        b1e = np.full((self.n, 3), np.nan)
        for i in range(self.n):
            if not np.all(np.isfinite(self.q[i])):
                continue
            R0a = quat_to_R(c["qw"][i], c["qx"][i], c["qy"][i], c["qz"][i])
            R0m = R0a @ TP.R_MODEL
            _, r0e, Re = TP.arm_fk_model(self.q[i], params)
            ee[i] = self.p[i] + R0m @ r0e
            b1e[i] = R0m @ Re[:, 0]
        self.ee = ee
        self.b1e = b1e

        self.e_base = self.p - self.p_ref
        self.e_ee = self.ee - self.ee_ref
        dot = np.clip(np.sum(self.b1e * self.b1de_ref, axis=1), -1.0, 1.0)
        self.e_head_deg = np.degrees(np.arccos(dot))
        self.e_q_deg = np.degrees(self.q - self.q_ref)
        # Base yaw error, wrapped.  A first-class metric, not a diagnostic:
        # the end-effector sits ~0.25 m out on the arm, so 1 deg of base yaw
        # error IS 4.4 mm of EE position error.  In the pinned task that term
        # dominates everything else, and a reader who only sees an EE number
        # would attribute it to the arm.
        dy = self.c["yaw"] - self.c["ref_psi"]
        self.e_yaw_deg = np.degrees(np.arctan2(np.sin(dy), np.cos(dy)))
        self.tilt = c["tilt_deg"]

    # -- scored windows ---------------------------------------------------
    def task_time(self):
        """Seconds since the task started, from the DRIVER's own clock.

        Not k * dt: the table index saturates during the post-task hold, and
        plotting against it stacks that whole hold on one x value.
        """
        m = self.k > 0
        t0 = self.t[m][0] if m.any() else 0.0
        return self.t - t0

    def window(self, name=None):
        """Boolean mask of the scored window.

        Always intersected with DIRECT and with the task actually ADVANCING,
        so neither the mode-switch transient, nor the SAFETY takeoff, nor the
        post-task hold enters a number.  Excluding the hold matters for the
        plots as much as the metrics: it sits at the table's last index, so
        hundreds of its samples would otherwise pile onto one x value.
        `name` optionally narrows to one segment (e.g. test 2's pinned phase).
        """
        m = self.direct & (self.k > 0) & (self.k < self.k.max())
        if name is not None:
            i = self.mark_names.index(name)
            t0, t1 = self.mark_t[i]
            kt = self.k * self.dt
            m = m & (kt >= t0) & (kt <= t1)
        return m


def lag_split(r, m):
    """Decompose a moving-window tracking error into LAG and PATH error.

    A position loop tracking a moving reference sits behind it; that shows up
    as a large error norm which says almost nothing about whether the vehicle
    is on the right path.  Splitting it is what makes the number readable:

      lag_s              the single time shift of the reference that minimises
                         the position RMS -- how far behind the loop runs.
      alongpath_mm       the error component along the commanded velocity
                         (negative mean = behind).  This is the lag.
      crosstrack_mm      the component perpendicular to it -- the part that
                         says the vehicle is off the PATH, not just late.
      lag_removed_mm     the residual RMS once the best shift is applied.

    Only samples where the reference is actually moving are used; a static
    hold has no direction and would dilute both components.  Returns NaNs for
    a task whose base never moves (comparison 1), where the split is
    meaningless rather than merely small.
    """
    v = np.stack([r.c["ref_vx"], r.c["ref_vy"], r.c["ref_vz"]], axis=1)
    sp = norm(v)
    mv = m & (sp > 0.02)
    nan = float("nan")
    if mv.sum() < 100:
        return dict(lag_s=nan, alongpath_rms_mm=nan, alongpath_mean_mm=nan,
                    crosstrack_rms_mm=nan, lag_removed_rms_mm=nan, n=0)
    u = v[mv] / sp[mv, None]
    e = r.e_base[mv]
    al = np.sum(e * u, axis=1)
    ct = e - al[:, None] * u
    t = r.t[mv]
    p = r.p[mv]
    ref = r.p_ref[mv]
    best = (np.inf, 0.0)
    for lag in np.arange(0.0, 2.51, 0.01):
        rs = np.stack([np.interp(t - lag, t, ref[:, i]) for i in range(3)],
                      axis=1)
        v_ = float(np.sqrt(np.mean(np.sum((p - rs) ** 2, axis=1))))
        if v_ < best[0]:
            best = (v_, float(lag))
    return dict(lag_s=best[1],
                alongpath_rms_mm=float(np.sqrt(np.mean(al ** 2)) * 1000.0),
                alongpath_mean_mm=float(al.mean() * 1000.0),
                crosstrack_rms_mm=float(
                    np.sqrt(np.mean(norm(ct) ** 2)) * 1000.0),
                lag_removed_rms_mm=float(best[0] * 1000.0),
                n=int(mv.sum()))


def stats(v, m):
    """rms / mean / p95 / max of a per-sample scalar over a window."""
    v = v[m]
    v = v[np.isfinite(v)]
    if v.size == 0:
        return dict(rms=float("nan"), mean=float("nan"),
                    p95=float("nan"), max=float("nan"), n=0)
    return dict(rms=float(np.sqrt(np.mean(v ** 2))), mean=float(np.mean(v)),
                p95=float(np.percentile(v, 95)), max=float(v.max()),
                n=int(v.size))


def norm(a):
    return np.linalg.norm(a, axis=1)


# ---------------------------------------------------------------------------
# figures
# ---------------------------------------------------------------------------

def fig_tracking(runs, task, scored, out):
    panels = [
        ("base position error", lambda r: norm(r.e_base) * 1000.0, "mm"),
        ("end-effector position error", lambda r: norm(r.e_ee) * 1000.0, "mm"),
        ("base yaw error", lambda r: np.abs(r.e_yaw_deg), "deg"),
        ("end-effector heading error", lambda r: r.e_head_deg, "deg"),
        ("joint tracking error (worst joint)",
         lambda r: np.nanmax(np.abs(r.e_q_deg), axis=1), "deg"),
    ]
    fig, axes = plt.subplots(len(panels), 1, figsize=(9.5, 11.4), sharex=True)
    fig.patch.set_facecolor(SURFACE)
    for ax, (title, fn, unit) in zip(axes, panels):
        _style(ax, ylabel=unit, title=title)
        placed = []
        for r in runs:
            m = r.window(None)
            y = fn(r)
            ax.plot(r.task_time()[m], y[m], color=r.color, linewidth=1.6,
                    label=r.label, solid_joinstyle="round")
            # selective direct label: the RMS over the SCORED window, at the
            # right edge, so the table's headline number is on the trace it
            # came from rather than only in a separate file.
            w = r.window(scored)
            v = y[w][np.isfinite(y[w])]
            if v.size:
                lo, hi = ax.get_ylim()
                frac = (np.nanmean(y[m][-40:]) - lo) / max(hi - lo, 1e-12)
                # de-collide the two labels: equal values would print on top
                # of each other, which is exactly when the reader most needs
                # to see both.
                while any(abs(frac - q) < 0.07 for q in placed):
                    frac += 0.08
                placed.append(frac)
                ax.annotate(f"{np.sqrt(np.mean(v ** 2)):.2f} {unit} rms",
                            xy=(1.005, np.clip(frac, 0.02, 0.96)),
                            xycoords="axes fraction",
                            color=r.color, fontsize=8, va="center")
        # shade the scored window so a reader can see which samples made the
        # numbers in the table
        if scored is not None:
            i = runs[0].mark_names.index(scored)
            t0, t1 = runs[0].mark_t[i]
            ax.axvspan(t0, t1, color=GRID, alpha=0.55, zorder=0, lw=0)
        ax.margins(x=0.01)
    h, la = axes[0].get_legend_handles_labels()
    fig.legend(h, la, frameon=False, fontsize=9, labelcolor=INK2, ncol=2,
               loc="upper right", bbox_to_anchor=(0.99, 0.995))
    axes[-1].set_xlabel("task time [s]", color=INK2, fontsize=9)
    if scored is not None:
        axes[0].text(0.99, 0.95, f"shaded = scored window ({scored})",
                     transform=axes[0].transAxes, ha="right", va="top",
                     color=INK2, fontsize=8)
    fig.suptitle(task["title"], color=INK, fontsize=12, x=0.012, ha="left")
    fig.tight_layout(rect=(0, 0, 0.90, 0.96))
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def _sphere(ax, r, color, alpha=0.10):
    u = np.linspace(0, 2 * np.pi, 48)
    v = np.linspace(0, np.pi, 24)
    x = r * np.outer(np.cos(u), np.sin(v))
    y = r * np.outer(np.sin(u), np.sin(v))
    z = r * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0,
                    shade=False, zorder=1)
    ax.plot_wireframe(x, y, z, color=color, linewidth=0.5, alpha=0.5,
                      rstride=6, cstride=6)


def fig_ball(runs, which, task, scored, out):
    """The error ball: the 3-D error cloud plus a sphere at its RMS radius.

    Both clouds go in ONE axes so 'which is smaller' is a direct visual
    comparison rather than a comparison of two separately-scaled panels; the
    three orthogonal projections beside it give the same answer without
    relying on 3-D depth perception, which is exactly what 3-D scatter plots
    are bad at.
    """
    get = (lambda r: r.e_base) if which == "base" else (lambda r: r.e_ee)
    name = "base position" if which == "base" else "end-effector position"

    data = []
    for r in runs:
        m = r.window(scored)
        e = get(r)[m] * 1000.0
        e = e[np.all(np.isfinite(e), axis=1)]
        data.append((r, e, float(np.sqrt(np.mean(norm(e) ** 2)))
                     if len(e) else float("nan")))

    lim = max(1e-9, max((np.abs(e).max() if len(e) else 0.0)
                        for _, e, _ in data)) * 1.15

    fig = plt.figure(figsize=(11.0, 7.6))
    fig.patch.set_facecolor(SURFACE)
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax.set_facecolor(SURFACE)
    for r, e, rms in data:
        if len(e) == 0:
            continue
        ax.scatter(e[:, 0], e[:, 1], e[:, 2], s=3, c=r.color, alpha=0.30,
                   depthshade=False, linewidths=0, label=f"{r.label}")
        _sphere(ax, rms, r.color)
    for setter in (ax.set_xlim, ax.set_ylim, ax.set_zlim):
        setter(-lim, lim)
    ax.set_xlabel("x [mm]", color=INK2, fontsize=8)
    ax.set_ylabel("y [mm]", color=INK2, fontsize=8)
    ax.set_zlabel("z [mm]", color=INK2, fontsize=8)
    ax.tick_params(colors=INK2, labelsize=7)
    ax.set_box_aspect((1, 1, 1))
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.set_pane_color((1.0, 1.0, 1.0, 0.0))
        pane._axinfo["grid"]["color"] = GRID
    ax.set_title(f"{name} error cloud + RMS sphere",
                 color=INK, fontsize=10, loc="left")
    lg = ax.legend(frameon=False, fontsize=9, labelcolor=INK2,
                   loc="upper left", markerscale=6, scatterpoints=1)
    for h in lg.legendHandles:
        h.set_alpha(1.0)

    for k, (i, j, lab) in enumerate([(0, 1, "x-y"), (0, 2, "x-z"),
                                     (1, 2, "y-z")]):
        a = fig.add_subplot(3, 2, 2 * k + 2)
        _style(a, xlabel=f"{lab[0]} [mm]", ylabel=f"{lab[2]} [mm]")
        th = np.linspace(0, 2 * np.pi, 200)
        for r, e, rms in data:
            if len(e) == 0:
                continue
            a.scatter(e[:, i], e[:, j], s=2, c=r.color, alpha=0.25,
                      linewidths=0)
            a.plot(rms * np.cos(th), rms * np.sin(th), color=r.color,
                   linewidth=1.8)
        a.set_xlim(-lim, lim)
        a.set_ylim(-lim, lim)
        a.set_aspect("equal", adjustable="box")
        if k == 0:
            # direct labels: the number the figure exists to convey
            txt = "  ".join(f"{r.label} RMS {rms:.1f} mm"
                            for r, _, rms in data)
            a.set_title(txt, color=INK, fontsize=9, loc="left")

    fig.suptitle(f"{task['title']} - {name} error ball"
                 + (f" (window: {scored})" if scored else ""),
                 color=INK, fontsize=12, x=0.012, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def fig_paths(runs, task, out):
    fig, axes = plt.subplots(2, 2, figsize=(10.5, 8.4))
    fig.patch.set_facecolor(SURFACE)
    r0 = runs[0]
    m0 = r0.window(None)
    specs = [
        (axes[0][0], "base path, top view", "x [m]", "y [m]",
         lambda r: (r.p[:, 0], r.p[:, 1]),
         lambda r: (r.p_ref[:, 0], r.p_ref[:, 1]), True),
        (axes[0][1], "base path, side view", "x [m]", "z [m]",
         lambda r: (r.p[:, 0], r.p[:, 2]),
         lambda r: (r.p_ref[:, 0], r.p_ref[:, 2]), False),
        (axes[1][0], "end-effector path, top view", "x [m]", "y [m]",
         lambda r: (r.ee[:, 0], r.ee[:, 1]),
         lambda r: (r.ee_ref[:, 0], r.ee_ref[:, 1]), True),
        (axes[1][1], "end-effector path, side view", "x [m]", "z [m]",
         lambda r: (r.ee[:, 0], r.ee[:, 2]),
         lambda r: (r.ee_ref[:, 0], r.ee_ref[:, 2]), False),
    ]
    for ax, title, xl, yl, meas, ref, equal in specs:
        _style(ax, xlabel=xl, ylabel=yl, title=title)
        rx, ry = ref(r0)
        ax.plot(rx[m0], ry[m0], color=C_REF, linewidth=2.4, linestyle=(0, (4, 3)),
                label="commanded", zorder=2)
        for r in runs:
            m = r.window(None)
            mx, my = meas(r)
            ax.plot(mx[m], my[m], color=r.color, linewidth=1.4, label=r.label,
                    zorder=3)
        if equal:
            ax.set_aspect("equal", adjustable="datalim")
    axes[0][0].legend(frameon=False, fontsize=9, labelcolor=INK2)
    fig.suptitle(f"{task['title']} - commanded vs flown",
                 color=INK, fontsize=12, x=0.012, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    plt.close(fig)


# ---------------------------------------------------------------------------

#: which quantity's ball each comparison is about, and which segment is scored
BALLS = {
    "hover_arm_swing": ("base", None),
    "circle_ee_hold": ("ee", "pinned"),
    "figure8_ee_updown": ("ee", None),
}


def write_campaign_summary(repo):
    """Roll the three per-task metrics.json files into results/SUMMARY.md."""
    rows = []
    for t in ("hover_arm_swing", "circle_ee_hold", "figure8_ee_updown"):
        f = os.path.join(repo, "results", t, "metrics.json")
        if os.path.exists(f):
            with open(f) as fh:
                rows.append(json.load(fh))
    if not rows:
        return None
    out = [
        "# Whole-body vs geometric + L1 adaptive - campaign summary", "",
        "Same Isaac plant, same allocator constants, same commanded motion; "
        "the control law and its gains are the only difference. Method and "
        "commands: `docs/docs_aerial_manipulator/Comparison Command.md`.", "",
        "| comparison | headline metric | whole-body | geometric+L1 | ratio |",
        "|---|---|---|---|---|",
    ]
    for m in rows:
        key = "base_err_mm" if m["ball"] == "base" else "ee_err_mm"
        w = m["runs"]["wb"][key]["rms"]
        l = m["runs"]["l1"][key]["rms"]
        nm = "base position RMS [mm]" if m["ball"] == "base" \
            else "EE position RMS [mm]"
        out.append(f"| {m['title']} | {nm} | {w:.2f} | {l:.2f} | "
                   f"{l / w:.2f}x |")
    out += ["", "Full per-task tables, per-axis breakdowns and figures are in "
            "each `results/<task>/` directory.", ""]
    for m in rows:
        out.append(f"- `results/{m['task']}/` - {m['title']} "
                   f"(scored window: {m['scored_window']})")
        for c in ("wb", "l1"):
            r = m["runs"][c]
            if r["aborted"]:
                out.append(f"  - **{r['label']} ABORTED**: {r['abort_reason']}")
    path = os.path.join(repo, "results", "SUMMARY.md")
    with open(path, "w") as fh:
        fh.write("\n".join(out) + "\n")
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--dir", default=None)
    a = ap.parse_args()

    repo = os.path.abspath(os.path.join(_HERE, "..", ".."))
    d = a.dir or os.path.join(repo, "results", a.task)
    params = TP.make_params_t650()

    runs = []
    for c in ("wb", "l1"):
        p = os.path.join(d, f"{c}.npz")
        if not os.path.exists(p):
            print(f"missing {p} -- fly it first")
            return 2
        runs.append(Run(p, params))

    which, scored = BALLS.get(a.task, ("base", None))
    title = {"hover_arm_swing": "1. hover + arm fold sweep",
             "circle_ee_hold": "2. base circle + yaw, EE pinned in the world",
             "figure8_ee_updown": "3. figure-8 base + EE up/down"}.get(
                 a.task, a.task)
    task = {"title": title, "name": a.task}

    metrics = {"task": a.task, "title": title, "scored_window": scored or "task",
               "ball": which, "runs": {}}
    for r in runs:
        m = r.window(scored)
        metrics["runs"][r.controller] = {
            "label": r.label,
            "aborted": r.aborted, "abort_reason": r.abort_reason,
            "samples_scored": int(m.sum()),
            "base_err_mm": stats(norm(r.e_base) * 1000.0, m),
            "ee_err_mm": stats(norm(r.e_ee) * 1000.0, m),
            # Per-axis RMS as well as the norm.  The error cloud of these
            # tasks is NOT isotropic -- the arm's fold drives the CoM almost
            # purely along the body x axis, so the cloud is a thin sheet and a
            # single sphere radius overstates two of its three extents.  The
            # ball figure is still the answer to "which is smaller"; these are
            # the shape behind it.
            "base_err_axis_rms_mm": [
                float(np.sqrt(np.mean((r.e_base[m][:, i] * 1000.0) ** 2)))
                for i in range(3)],
            "ee_err_axis_rms_mm": [
                float(np.sqrt(np.nanmean((r.e_ee[m][:, i] * 1000.0) ** 2)))
                for i in range(3)],
            "base_yaw_err_deg": stats(np.abs(r.e_yaw_deg), m),
            "ee_heading_err_deg": stats(r.e_head_deg, m),
            "joint_err_deg": stats(np.nanmax(np.abs(r.e_q_deg), axis=1), m),
            "tilt_deg": stats(r.tilt, m),
            "joint_torque_Nm": stats(np.nanmax(np.abs(r.tau), axis=1), m),
            "base_lag_split": lag_split(r, m),
        }

    os.makedirs(d, exist_ok=True)
    fig_tracking(runs, task, scored, os.path.join(d, "tracking_error.png"))
    fig_ball(runs, which, task, scored, os.path.join(d, "error_ball.png"))
    fig_paths(runs, task, os.path.join(d, "paths.png"))
    with open(os.path.join(d, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)

    def row(k, unit, fld="rms"):
        w = metrics["runs"]["wb"][k][fld]
        l = metrics["runs"]["l1"][k][fld]
        better = "whole-body" if w < l else ("geometric+L1" if l < w else "-")
        return f"| {k} ({unit}, {fld}) | {w:.3f} | {l:.3f} | {better} |"

    lines = [
        f"# {title}", "",
        f"Scored window: **{scored or 'the whole task, in DIRECT'}** "
        f"({metrics['runs']['wb']['samples_scored']} / "
        f"{metrics['runs']['l1']['samples_scored']} samples).",
        "",
        "Both runs were commanded from the same trajectory table "
        "(`utils_comparison/comparison_tasks.py`) and flew the same Isaac "
        "plant with the same allocator constants; only the control law and "
        "its gains differ.",
        "",
        "| metric | whole-body | geometric+L1 | smaller |",
        "|---|---|---|---|",
        row("base_err_mm", "mm"), row("base_err_mm", "mm", "p95"),
        row("ee_err_mm", "mm"), row("ee_err_mm", "mm", "p95"),
        row("base_yaw_err_deg", "deg"),
        row("ee_heading_err_deg", "deg"),
        row("joint_err_deg", "deg"),
        row("tilt_deg", "deg"),
        "",
        f"Ball figure shows the **{which}** error cloud "
        f"(sphere radius = RMS norm).",
        "",
        "Per-axis RMS [mm] - the cloud is not isotropic, so this is the shape "
        "behind the single radius:",
        "",
        "| axis | whole-body base | L1 base | whole-body EE | L1 EE |",
        "|---|---|---|---|---|",
    ] + [
        "| {} | {:.2f} | {:.2f} | {:.2f} | {:.2f} |".format(
            ax,
            metrics["runs"]["wb"]["base_err_axis_rms_mm"][i],
            metrics["runs"]["l1"]["base_err_axis_rms_mm"][i],
            metrics["runs"]["wb"]["ee_err_axis_rms_mm"][i],
            metrics["runs"]["l1"]["ee_err_axis_rms_mm"][i])
        for i, ax in enumerate("xyz")
    ]
    ls = {c: metrics["runs"][c]["base_lag_split"] for c in ("wb", "l1")}
    if ls["wb"]["n"]:
        lines += [
            "",
            "Where the base is MOVING, most of the error is the loop running "
            "behind the reference rather than being off the path. Splitting "
            "it:",
            "",
            "| | whole-body | geometric+L1 |",
            "|---|---|---|",
            "| fitted lag [s] | {:.2f} | {:.2f} |".format(
                ls["wb"]["lag_s"], ls["l1"]["lag_s"]),
            "| along-path rms [mm] | {:.1f} | {:.1f} |".format(
                ls["wb"]["alongpath_rms_mm"], ls["l1"]["alongpath_rms_mm"]),
            "| cross-track rms [mm] | {:.1f} | {:.1f} |".format(
                ls["wb"]["crosstrack_rms_mm"], ls["l1"]["crosstrack_rms_mm"]),
            "| rms with the lag removed [mm] | {:.1f} | {:.1f} |".format(
                ls["wb"]["lag_removed_rms_mm"],
                ls["l1"]["lag_removed_rms_mm"]),
        ]
    for r in runs:
        if r.aborted:
            lines += ["", f"**{r.label} ABORTED**: {r.abort_reason}"]
    with open(os.path.join(d, "summary.md"), "w") as f:
        f.write("\n".join(lines) + "\n")

    print("\n".join(lines))
    print(f"\nwrote {d}/{{metrics.json,summary.md,tracking_error.png,"
          f"error_ball.png,paths.png}}")
    sp = write_campaign_summary(repo)
    if sp:
        print(f"wrote {sp}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

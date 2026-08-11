#!/usr/bin/env python
"""
plot_results.py

Plot a closed-loop aerial-manipulator run recorded by the in-process demos
(02_aerial_manipulator_free.py -> gmo_log.npz, 02_aerial_manipulator_pick.py ->
pick_log.npz, 01_aerial_manipulator_track.py -> track_log.npz,
03_px4_direct_aerial_manipulator_free.py -> gmo_px4_log.npz).
Python port of the MATLAB plot_result.m layout, adapted to the signals the Isaac
demos log. Produces five PNGs under ./results/:

  trajectories_<tag>.png     ACTUAL vs DESIRED time histories — the reference the
                             controller was asked to follow, overlaid on what it
                             achieved: (a) CoM x_c/x_cd, (b) EE r_e/r_ed,
                             (c) joints q/q_d, (d) horizontal (x-y) top view
  tracking_errors_<tag>.png  (a) drone CoM position error  e_x  = x_c - x_cd
                             (b) drone attitude error      e_R
                             (c) EE position error         e_xE = r_e - r_ed
                             (d) EE yaw error              e_{R_E,3}
  control_<tag>.png          (a) thrust u1, (b) base moment tau(4:6),
                             (c) applied joint torques tau_j, (d) arm task input u3
  states_<tag>.png           base position r_0, joint angles q (+ q_d ref),
                             base linear/angular velocity v_0 / omega_0, joint rates
  trajectory3d_<tag>.png     3D base / CoM / EE paths, actual (solid) vs desired
                             (dashed); the takeoff climb is drawn faded so the
                             tracking phase stands out. Start/end markers.

Usage:
  python plot_results.py [npz_path] [--tag TAG] [--takeoff T] [--show]

  npz_path   log to plot (default: newest .npz in results/log/)
  --tag      filename tag for the saved PNGs (default: derived from the npz name)
  --takeoff  takeoff-time boundary marked on the time plots [s] (default: the
             log's own first phase edge — the takeoff is hover-gated, so its
             length varies per run; falls back to the controllers' nominal
             TAKEOFF_TIME, else 4.0, for logs with no phase table)
  --show     also open the figures interactively (otherwise save-only)

Run under Isaac's python (system python3's matplotlib is broken here by a
numpy 1.x/2.x conflict):

  ~/isaacsim/python_r_fsc.sh plot_results.py log/gmo_log.npz --tag compatible

Runs headless by default (Agg backend) — no display needed.
"""

import os
import sys
import glob
import argparse

import numpy as np
import matplotlib
if "--show" not in sys.argv:
    matplotlib.use("Agg")          # save-only; no display required
import matplotlib.pyplot as plt

# This file lives in robotic_arm/utils_plot/, but the data does NOT: everything
# a run produces belongs to the scenario, not to the plotting code. Anchor one
# level up, at the single output root:
#     robotic_arm/results/log/       .npz written by the demos
#     robotic_arm/results/figures/   .png written by this script
SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(SCRIPT_DIR, "results")
RESULTS_DIR = os.path.join(OUT_DIR, "figures")
LOG_DIR = os.path.join(OUT_DIR, "log")

# consistent colors: x/y/z for 3-vectors, a colormap for the joints
XYZ_COLORS = ("tab:red", "tab:green", "tab:blue")
XYZ_LABELS = ("$x$", "$y$", "$z$")

# Flight-phase background tints (npz "phase_t"/"phase_names", written by the
# demo). Deliberately desaturated and drawn at low alpha under everything else
# so they read as context, not as data — the curve colors above stay dominant.
PHASE_COLORS = {
    "takeoff": "#9e9e9e",   # grey  — climb / not the trajectory yet
    "fly-in":  "#4c9be8",   # blue  — transit out
    "pinned":  "#f5b942",   # amber — the highlight: EE frozen, body moves
    "fly-out": "#6cc24a",   # green — transit to the landing spot
    "land":    "#b07aa1",   # mauve — vertical descent + touchdown
    "landing": "#b07aa1",   # mauve — the same descent, as a SETPOINT phase
                            #         (the demos own the landing; "land" is
                            #          the legacy in-plan descent segment)
    "hold":    "#9e9e9e",   # grey  — after the plan ends
}
PHASE_FALLBACK = ("#4c9be8", "#f5b942", "#6cc24a", "#b07aa1", "#e15759")
PHASE_ALPHA = 0.13


def _default_npz():
    """Newest .npz in results/log/ — plotting the run you just did needs no path."""
    cands = glob.glob(os.path.join(LOG_DIR, "*.npz"))
    if not cands:
        return os.path.join(LOG_DIR, "gmo_log.npz")     # nonexistent -> clean error
    return max(cands, key=os.path.getmtime)


def _takeoff_default():
    """TAKEOFF_TIME, read from the planner — its single source since the
    trajectories moved out of the controllers into utils_planner/. Loaded by
    FILE PATH, not by import, so this script still runs standalone against a
    tree that is not on sys.path. Falls back to the controllers (which still
    define their own) and finally to 4.0 s."""
    import importlib.util
    cands = [("_traj_lib", os.path.join(SCRIPT_DIR, "utils_planner",
                                        "trajectory_library.py"))]
    # controller_track is the only controller that still defines its own
    # TAKEOFF_TIME (the merged controller.py holds no trajectory constants —
    # they live in utils_planner, which is the first candidate above).
    cands += [(n, os.path.join(SCRIPT_DIR, "utils_controller", n + ".py"))
              for n in ("controller_track",)]
    for name, path in cands:
        try:
            spec = importlib.util.spec_from_file_location(name, path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return float(mod.TAKEOFF_TIME)
        except Exception:
            continue
    return 4.0


def _get(d, key):
    """Optional npz field -> array or None (older logs lack some keys)."""
    return d[key] if key in d.files else None


def _mark_takeoff(ax, t_takeoff):
    """Vertical dashed line at the takeoff -> trajectory handover."""
    if t_takeoff is not None and t_takeoff > 0:
        ax.axvline(t_takeoff, color="0.6", ls="--", lw=1.0, zorder=0)


def _phase_color(name, i):
    return PHASE_COLORS.get(str(name).lower(),
                            PHASE_FALLBACK[i % len(PHASE_FALLBACK)])


def _shade_phases(ax, edges, names, label=True):
    """Tint the background of a TIME-axis panel one band per flight phase, and
    write the phase name along the top edge of each band wide enough to hold
    it. Bands are drawn at zorder 0 so every curve stays on top."""
    if edges is None or names is None or len(edges) < 2:
        return
    span = float(edges[-1] - edges[0]) or 1.0
    for i, (a, b) in enumerate(zip(edges[:-1], edges[1:])):
        if i >= len(names):
            break
        ax.axvspan(a, b, color=_phase_color(names[i], i), alpha=PHASE_ALPHA,
                   lw=0, zorder=0)
        # skip the label on slivers — it would collide with its neighbours
        if label and (b - a) / span > 0.07:
            ax.text(0.5 * (a + b), 0.995, str(names[i]),
                    transform=ax.get_xaxis_transform(), ha="center", va="top",
                    fontsize=6.5, color="0.30", zorder=4, clip_on=True)


def _mark_phase_points(ax, edges, names, t, P, z=None):
    """Dot + label where each phase BEGINS along a SPATIAL path (2-D top view or
    3-D) — the path-plot equivalent of the time-axis shading.

    Boundaries that land on nearly the same point are merged into one dot with
    a combined label: a phase can start where the previous one ended without
    the vehicle having moved (the pinned phase holds position, and touchdown →
    hold share a point), which would otherwise stack unreadable labels."""
    if edges is None or names is None:
        return
    span = float(np.linalg.norm(P.max(0) - P.min(0))) or 1.0
    shown = []                       # [(point, [names]), ...]
    for i, tb in enumerate(edges[1:-1], start=1):     # interior boundaries only
        k = int(np.searchsorted(t, tb))
        if not (0 < k < len(t)) or i >= len(names):
            continue
        pt = P[k]
        merged = next((s for s in shown
                       if np.linalg.norm(pt - s[0]) < 0.04 * span), None)
        if merged is not None:
            merged[1].append(str(names[i]))
            continue
        shown.append((pt, [str(names[i])]))
        ax.scatter(*(pt if z else pt[:2]), s=34, color=_phase_color(names[i], i),
                   edgecolors="0.25", linewidths=0.6, zorder=6)
    for pt, labs in shown:
        txt = " / ".join(labs)
        if z:
            ax.text(pt[0], pt[1], pt[2], "  " + txt, fontsize=6.5,
                    color="0.30", zorder=6)
        else:
            ax.annotate(txt, (pt[0], pt[1]), textcoords="offset points",
                        xytext=(7, 5), fontsize=6.5, color="0.30", zorder=6)


def _plot_xyz(ax, t, Y, labels=XYZ_LABELS, colors=XYZ_COLORS, lw=1.4):
    for i in range(Y.shape[1]):
        ax.plot(t, Y[:, i], color=colors[i % len(colors)],
                label=labels[i % len(labels)], lw=lw)


def _plot_xyz_pair(ax, t, Y, Yd, labels=XYZ_LABELS, colors=XYZ_COLORS):
    """Actual (solid) over desired (dashed, same color per component)."""
    for i in range(Y.shape[1]):
        c = colors[i % len(colors)]
        ax.plot(t, Yd[:, i], color=c, lw=2.0, ls="--", alpha=0.55)
        ax.plot(t, Y[:, i], color=c, lw=1.4, label=labels[i % len(labels)])


def _plot_joints(ax, t, Q, label_fmt, ref=None, lw=1.5):
    """Plot each column of Q with a joint color; label_fmt(1-based idx) -> str.
    Optional ref (same shape) is overlaid dashed in the matching color."""
    n = Q.shape[1]
    cmap = plt.get_cmap("tab10")
    for j in range(n):
        c = cmap(j)
        if ref is not None:
            ax.plot(t, ref[:, j], color=c, lw=2.0, ls="--", alpha=0.55)
        ax.plot(t, Q[:, j], color=c, lw=lw, label=label_fmt(j + 1))


def _legend_note(ax, note="solid = actual, dashed = desired"):
    ax.text(0.99, 0.02, note, transform=ax.transAxes, ha="right", va="bottom",
            fontsize=7.5, color="0.35")


def _rmse(e):
    """RMSE of an error signal over time. Returns (overall, per_component):
    overall = sqrt(mean ||e||^2); per_component = per-column RMSE."""
    e = np.asarray(e, float)
    if e.ndim == 1:
        e = e[:, None]
    per = np.sqrt(np.mean(e ** 2, axis=0))
    overall = float(np.sqrt(np.mean(np.sum(e ** 2, axis=1))))
    return overall, per


def _print_rmse(t, tag, crashed, t_takeoff, rows):
    """Print an RMSE table over the tracking phase (t >= takeoff), or the full
    run if nothing lands after takeoff. rows = list of (name, err, unit)."""
    if t_takeoff and np.any(t >= t_takeoff):
        mask = t >= t_takeoff
        phase = f"tracking phase, t >= {t_takeoff:g} s"
    else:
        mask = np.ones_like(t, dtype=bool)
        phase = "full run"
    print(f"\n=== RMSE ({phase}, N={int(mask.sum())})  tag: {tag}"
          + ("  [CRASHED]" if crashed else "") + " ===")
    for name, err, unit in rows:
        ov, per = _rmse(np.asarray(err)[mask])
        line = f"  {name:20s} {ov:8.4f} {unit:5s}"
        if len(per) > 1:
            line += "  [" + "  ".join(f"{v:.4f}" for v in per) + "]"
        print(line)


def make_plots(npz_path, tag, t_takeoff, show):
    d = np.load(npz_path, allow_pickle=True)
    t = d["t"]
    crashed = bool(d["crashed"]) if "crashed" in d.files else False
    suptag = f"{tag}" + ("  [CRASHED]" if crashed else "")
    q_d = _get(d, "q_d")
    # flight-phase bands (newer logs). Older logs fall back to the takeoff line.
    phase_t, phase_names = _get(d, "phase_t"), _get(d, "phase_names")
    # The takeoff -> task handover is HOVER-GATED in the demos, so how long the
    # takeoff took varies from run to run — the log's own first band edge is the
    # authority for where the tracking phase begins. The nominal TAKEOFF_TIME is
    # only the fallback, for logs written before the phase table existed.
    if t_takeoff is None:
        t_takeoff = (float(phase_t[1]) if phase_t is not None and len(phase_t) > 1
                     else _takeoff_default())
    if phase_t is not None:
        print("\nPhases: " + ", ".join(
            f"{n} [{a:.1f}–{b:.1f}s]" for n, a, b
            in zip(phase_names, phase_t[:-1], phase_t[1:])))

    # --- derived tracking errors ---
    e_x = d["x_c"] - d["x_cd"]          # CoM position error
    e_R = d["e_R"]                       # attitude error (logged)
    e_y = d["e_y"]                       # EE task error [e_xE(3), e_RE3(1)]
    e_xE = e_y[:, :3]
    e_RE3 = e_y[:, 3]

    # --- RMSE summary to the terminal (tracking phase) ---
    rows = [("CoM position e_x", e_x, "m"),
            ("Attitude     e_R", e_R, ""),
            ("EE position  e_xE", e_xE, "m"),
            ("EE yaw       e_RE3", e_RE3, "")]
    if q_d is not None:
        rows.append(("Joint  q - q_d", d["q"] - q_d, "rad"))
    _print_rmse(t, tag, crashed, t_takeoff, rows)

    # --- commanded travel, so "did it actually move?" is answerable from the log ---
    trk = t >= t_takeoff if (t_takeoff and np.any(t >= t_takeoff)) else np.ones_like(t, bool)
    span_c = d["x_cd"][trk].max(0) - d["x_cd"][trk].min(0)
    span_e = d["r_ed"][trk].max(0) - d["r_ed"][trk].min(0)
    print(f"\n=== Commanded travel (tracking phase) ===")
    print(f"  CoM x_cd span [m] : {np.round(span_c, 3)}")
    print(f"  EE  r_ed span [m] : {np.round(span_e, 3)}")
    if q_d is not None:
        print(f"  q_d span [deg]    : {np.round(np.degrees(q_d[trk].max(0) - q_d[trk].min(0)), 1)}")

    os.makedirs(RESULTS_DIR, exist_ok=True)
    saved = []

    def _save(fig, name):
        p = os.path.join(RESULTS_DIR, f"{name}_{tag}.png")
        fig.savefig(p, dpi=150)
        saved.append(p)

    def _finish(axes, legend_ncol=1, mark=True, label_phases=True):
        """Grid + legend, plus the phase shading / takeoff marker on TIME-axis
        panels only (`mark=False` for spatial panels — a t=4 s line would land
        at x=4 m). When phase bands are available they replace the single
        takeoff line, which would just duplicate the first band edge."""
        for a in np.atleast_1d(axes).flat:
            if a.has_data():
                a.grid(True, alpha=0.3)
                if mark:
                    if phase_t is not None:
                        _shade_phases(a, phase_t, phase_names, label=label_phases)
                    else:
                        _mark_takeoff(a, t_takeoff)
                if a.get_legend_handles_labels()[1]:
                    a.legend(loc="best", fontsize=8, ncol=legend_ncol)

    # ============ FIGURE 1 — ACTUAL vs DESIRED trajectories =============
    f0, ax = plt.subplots(2, 2, figsize=(11.5, 8))
    f0.suptitle(f"Trajectories: actual vs desired — {suptag}", fontweight="bold")

    _plot_xyz_pair(ax[0, 0], t, d["x_c"], d["x_cd"])
    ax[0, 0].set(title="Drone CoM  $x_c$ vs $x_{cd}$", xlabel="$t$ (s)", ylabel="position (m)")
    _legend_note(ax[0, 0])

    _plot_xyz_pair(ax[0, 1], t, d["r_e"], d["r_ed"])
    ax[0, 1].set(title="End-effector  $r_e$ vs $r_{ed}$", xlabel="$t$ (s)", ylabel="position (m)")
    _legend_note(ax[0, 1])

    _plot_joints(ax[1, 0], t, np.degrees(d["q"]), lambda j: fr"$q_{{{j}}}$",
                 ref=None if q_d is None else np.degrees(q_d))
    ax[1, 0].set(title=r"Joint angles  $q$ vs $q_d$", xlabel="$t$ (s)", ylabel=r"$q$ (deg)")
    _legend_note(ax[1, 0])

    # (d) horizontal top view — the shape of the path, actual vs desired
    a = ax[1, 1]
    a.plot(d["x_cd"][:, 0], d["x_cd"][:, 1], color="tab:green", lw=2.0, ls="--",
           alpha=0.55, label="CoM desired")
    a.plot(d["x_c"][:, 0], d["x_c"][:, 1], color="tab:green", lw=1.4, label="CoM actual")
    a.plot(d["r_ed"][:, 0], d["r_ed"][:, 1], color="tab:orange", lw=2.0, ls="--",
           alpha=0.55, label="EE desired")
    a.plot(d["r_e"][:, 0], d["r_e"][:, 1], color="tab:orange", lw=1.4, label="EE actual")
    a.scatter(*d["x_c"][0, :2], c="tab:blue", s=45, marker="o", edgecolors="k", zorder=5)
    a.scatter(*d["x_c"][-1, :2], c="tab:red", s=45, marker="o", edgecolors="k", zorder=5)
    # phase transitions along the path (no time axis here to shade)
    _mark_phase_points(a, phase_t, phase_names, t, d["x_c"])
    a.set(title="Horizontal path (top view)", xlabel="$x_I$ (m)", ylabel="$y_I$ (m)")
    a.set_aspect("equal", adjustable="datalim")

    _finish(ax[0, :])                 # time-axis panels
    _finish(ax[1, 0])
    _finish(a, mark=False)            # spatial panel: no takeoff time marker
    f0.tight_layout(rect=(0, 0, 1, 0.97))
    _save(f0, "trajectories")

    # ================= FIGURE 2 — tracking errors =======================
    f1, ax = plt.subplots(2, 2, figsize=(11, 7.5))
    f1.suptitle(f"Tracking errors — {suptag}", fontweight="bold")

    _plot_xyz(ax[0, 0], t, e_x)
    ax[0, 0].set(title="Drone CoM position error", xlabel="$t$ (s)", ylabel=r"$e_x$ (m)")

    _plot_xyz(ax[0, 1], t, e_R, labels=("$e_{R,1}$", "$e_{R,2}$", "$e_{R,3}$"))
    ax[0, 1].set(title="Drone attitude error", xlabel="$t$ (s)", ylabel=r"$e_R$")

    _plot_xyz(ax[1, 0], t, e_xE)
    ax[1, 0].set(title="End-effector position error", xlabel="$t$ (s)", ylabel=r"$e_{x_E}$ (m)")

    ax[1, 1].plot(t, e_RE3, color="0.1", lw=1.5)
    ax[1, 1].set(title="End-effector yaw error", xlabel="$t$ (s)", ylabel=r"$e_{R_E,3}$")

    _finish(ax)
    f1.tight_layout(rect=(0, 0, 1, 0.97))
    _save(f1, "tracking_errors")

    # ================= FIGURE 3 — control inputs ========================
    f3, ax = plt.subplots(2, 2, figsize=(11, 8))
    f3.suptitle(f"Control inputs — {suptag}", fontweight="bold")

    ax[0, 0].plot(t, d["thrust"], color="0.1", lw=1.5)
    ax[0, 0].set(title="Thrust $u_1$", xlabel="$t$ (s)", ylabel=r"$u_1$ (N)")

    _plot_xyz(ax[0, 1], t, d["tau_b"])
    ax[0, 1].set(title=r"Base moment $\tau_{4:6}$", xlabel="$t$ (s)", ylabel=r"$\tau_M$ (N·m)")

    _plot_joints(ax[1, 0], t, d["tau_j"], lambda j: fr"$\tau_{{q,{j}}}$")
    ax[1, 0].set(title=r"Applied joint torques $\tau_j$", xlabel="$t$ (s)", ylabel=r"$\tau_q$ (N·m)")

    _plot_joints(ax[1, 1], t, d["u3"], lambda j: fr"$u_{{3,{j}}}$")
    ax[1, 1].set(title="Arm task input $u_3$", xlabel="$t$ (s)", ylabel=r"$u_3$ (N·m)")

    _finish(ax, legend_ncol=2)
    f3.tight_layout(rect=(0, 0, 1, 0.97))
    _save(f3, "control")

    # ================= FIGURE 4 — recorded states =======================
    f2, ax = plt.subplots(3, 2, figsize=(11, 9))
    f2.suptitle(f"Recorded states — {suptag}", fontweight="bold")

    _plot_xyz(ax[0, 0], t, d["p"])
    ax[0, 0].set(title="Base position $r_0$", xlabel="$t$ (s)", ylabel=r"$r_0$ (m)")

    _plot_joints(ax[0, 1], t, d["q"], lambda j: fr"$q_{{{j}}}$", ref=q_d)
    ax[0, 1].set(title=r"Joint angles $q$ (dashed = $q_d$)", xlabel="$t$ (s)", ylabel=r"$q$ (rad)")

    _plot_xyz(ax[1, 0], t, d["v0"])
    ax[1, 0].set(title="Base linear velocity $v_0$ (body)", xlabel="$t$ (s)", ylabel=r"$v_0$ (m/s)")

    _plot_xyz(ax[1, 1], t, d["omega0"],
              labels=(r"$\omega_x$", r"$\omega_y$", r"$\omega_z$"))
    ax[1, 1].set(title=r"Base angular velocity $\omega_0$ (body)", xlabel="$t$ (s)",
                 ylabel=r"$\omega_0$ (rad/s)")

    _plot_joints(ax[2, 0], t, d["qdot"], lambda j: fr"$\dot{{q}}_{{{j}}}$")
    ax[2, 0].set(title=r"Joint rates $\dot{q}$", xlabel="$t$ (s)", ylabel=r"$\dot{q}$ (rad/s)")

    # GMO disturbance estimates when the log carries them (gmo runs), else blank
    d_t = _get(d, "d_t_hat")
    if d_t is not None:
        _plot_xyz(ax[2, 1], t, d_t, labels=(r"$\hat{d}_{t,x}$", r"$\hat{d}_{t,y}$", r"$\hat{d}_{t,z}$"))
        ee_f = _get(d, "ee_force")
        if ee_f is not None and np.abs(ee_f).max() > 1e-9:
            for i in range(3):
                ax[2, 1].plot(t, ee_f[:, i], color=XYZ_COLORS[i], lw=2.0, ls=":", alpha=0.6)
            _legend_note(ax[2, 1], "dotted = applied EE force")
        ax[2, 1].set(title=r"GMO translational estimate $\hat{d}_t$", xlabel="$t$ (s)",
                     ylabel=r"$\hat{d}_t$ (N)")
    else:
        ax[2, 1].axis("off")

    _finish(ax, legend_ncol=2)
    f2.tight_layout(rect=(0, 0, 1, 0.97))
    _save(f2, "states")

    # ================= FIGURE 5 — 3D trajectories =======================
    f4 = plt.figure(figsize=(9.5, 8))
    ax3 = f4.add_subplot(111, projection="3d")
    f4.suptitle(f"3D trajectory — {suptag}", fontweight="bold")

    # split takeoff climb from the tracking phase so the trajectory reads clearly
    k = int(np.searchsorted(t, t_takeoff)) if t_takeoff else 0
    k = min(max(k, 0), len(t) - 1)

    def _p3(P, color, label, ls="-", lw=1.6, alpha=1.0):
        if k > 1:                       # faded takeoff climb, unlabeled
            ax3.plot(P[:k, 0], P[:k, 1], P[:k, 2], color=color, ls=ls, lw=lw, alpha=0.25)
        ax3.plot(P[k:, 0], P[k:, 1], P[k:, 2], color=color, ls=ls, lw=lw,
                 alpha=alpha, label=label)

    _p3(d["p"], "0.2", "base $r_0$")
    _p3(d["x_cd"], "tab:green", "CoM desired $x_{cd}$", ls="--", lw=2.0, alpha=0.55)
    _p3(d["x_c"], "tab:green", "CoM $x_c$")
    _p3(d["r_ed"], "tab:orange", "EE desired $r_{ed}$", ls="--", lw=2.0, alpha=0.55)
    _p3(d["r_e"], "tab:orange", "EE $r_e$")

    for P, mk in ((d["x_c"], "o"), (d["r_e"], "p")):
        ax3.scatter(*P[k], c="tab:blue", s=45, marker=mk, edgecolors="k", zorder=5)
        ax3.scatter(*P[-1], c="tab:red", s=45, marker=mk, edgecolors="k", zorder=5)
    # phase transitions along the CoM path, in the same colors as the bands
    _mark_phase_points(ax3, phase_t, phase_names, t, d["x_c"], z=True)

    ax3.set_xlabel("$x_I$ (m)"); ax3.set_ylabel("$y_I$ (m)"); ax3.set_zlabel("$z_I$ (m)")
    ax3.legend(loc="upper left", fontsize=8)
    ax3.text2D(0.01, 0.01,
               "faded = takeoff climb;  blue = trajectory start, red = end;"
               "  dots = phase transitions",
               transform=ax3.transAxes, fontsize=7.5, color="0.35")
    # equal aspect over the whole scene
    pts = np.vstack([d["p"], d["x_c"], d["r_e"], d["x_cd"], d["r_ed"]])
    c = pts.mean(0); rng = (pts.max(0) - pts.min(0)).max() / 2 or 1.0
    ax3.set_xlim(c[0]-rng, c[0]+rng)
    ax3.set_ylim(c[1]-rng, c[1]+rng)
    ax3.set_zlim(c[2]-rng, c[2]+rng)
    _save(f4, "trajectory3d")

    print(f"\nSaved {len(saved)} figures to {RESULTS_DIR}/  (tag: {tag})")
    for s in saved:
        print("  " + os.path.relpath(s, SCRIPT_DIR))
    if show:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description="Plot an in-process aerial-manipulator run.")
    ap.add_argument("npz", nargs="?", default=None,
                    help="run log (default: newest .npz in results/log/)")
    ap.add_argument("--tag", default=None, help="filename tag for the PNGs")
    ap.add_argument("--takeoff", type=float, default=None, help="takeoff-time boundary [s]")
    ap.add_argument("--show", action="store_true", help="open figures interactively")
    args = ap.parse_args()

    npz = args.npz or _default_npz()
    if not os.path.exists(npz):
        ap.error(f"log not found: {npz}")
    if args.npz is None:
        print(f"[plot_results] newest log: {os.path.relpath(npz, SCRIPT_DIR)}")
    base = os.path.splitext(os.path.basename(npz))[0]
    tag = args.tag or base.replace("_log", "") or "run"
    # None = resolve from the log's phase table (see make_plots); --takeoff
    # overrides it for a log whose bands are wrong or missing.
    make_plots(npz, tag, args.takeoff, args.show)


if __name__ == "__main__":
    main()

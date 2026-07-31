#!/usr/bin/env python
"""
plot_results.py

Plot a closed-loop aerial-manipulator run recorded by the in-process demo
(application/robotic_arm/02_aerial_manipulator_track.py -> track_log.npz).
Python port of the MATLAB plot_result.m layout, adapted to the signals the
Isaac demo logs. Produces four PNGs under ./images/:

  tracking_errors_<tag>.png  (a) drone CoM position error  e_x  = x_c - x_cd
                             (b) drone attitude error      e_R
                             (c) EE position error         e_xE = r_e - r_ed
                             (d) EE yaw error              e_{R_E,3}
  states_<tag>.png           base position r_0, joint angles q (+ q_d ref),
                             base linear/angular velocity v_0 / omega_0, joint rates
  control_<tag>.png          (a) thrust u1, (b) base moment tau(4:6),
                             (c) applied joint torques tau_j, (d) arm task input u3
  trajectory3d_<tag>.png     3D base / CoM / EE paths, actual (solid) vs
                             desired (dashed), start/end markers

Usage:
  python plot_results.py [npz_path] [--tag TAG] [--takeoff T] [--show]

  npz_path   log to plot (default: <robotic_arm>/log/track_log.npz)
  --tag      filename tag for the saved PNGs (default: derived from the npz name)
  --takeoff  takeoff-time boundary marked on the time plots [s]
             (default: controller_track.TAKEOFF_TIME if importable, else 4.0)
  --show     also open the figures interactively (otherwise save-only)

Runs headless by default (Agg backend) — no display needed.
"""

import os
import sys
import argparse

import numpy as np
import matplotlib
if "--show" not in sys.argv:
    matplotlib.use("Agg")          # save-only; no display required
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGES_DIR = os.path.join(SCRIPT_DIR, "images")
DEFAULT_NPZ = os.path.join(SCRIPT_DIR, "log", "track_log.npz")

# consistent colors: x/y/z for 3-vectors, a colormap for the joints
XYZ_COLORS = ("tab:red", "tab:green", "tab:blue")
XYZ_LABELS = ("$x$", "$y$", "$z$")


def _takeoff_default():
    """Read TAKEOFF_TIME from the paired controller if we can; else 4.0 s."""
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "controller_track", os.path.join(SCRIPT_DIR, "controller_track.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return float(mod.TAKEOFF_TIME)
    except Exception:
        return 4.0


def _mark_takeoff(ax, t_takeoff):
    """Vertical dashed line at the takeoff -> trajectory handover."""
    if t_takeoff is not None and t_takeoff > 0:
        ax.axvline(t_takeoff, color="0.6", ls="--", lw=1.0, zorder=0)


def _plot_xyz(ax, t, Y, labels=XYZ_LABELS, colors=XYZ_COLORS, lw=1.4):
    for i in range(Y.shape[1]):
        ax.plot(t, Y[:, i], color=colors[i % len(colors)],
                label=labels[i % len(labels)], lw=lw)


def _plot_joints(ax, t, Q, label_fmt, ref=None, lw=1.5):
    """Plot each column of Q with a joint color; label_fmt(1-based idx) -> str.
    Optional ref (same shape) is overlaid dashed in the matching color."""
    n = Q.shape[1]
    cmap = plt.get_cmap("tab10")
    for j in range(n):
        c = cmap(j)
        ax.plot(t, Q[:, j], color=c, lw=lw, label=label_fmt(j + 1))
        if ref is not None:
            ax.plot(t, ref[:, j], color=c, lw=1.0, ls="--", alpha=0.8)


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
    if "q_d" in d.files:
        rows.append(("Joint  q - q_d", d["q"] - d["q_d"], "rad"))
    _print_rmse(t, tag, crashed, t_takeoff, rows)

    os.makedirs(IMAGES_DIR, exist_ok=True)
    saved = []

    # ================= FIGURE 1 — tracking errors =======================
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

    for a in ax.flat:
        a.grid(True, alpha=0.3)
        _mark_takeoff(a, t_takeoff)
        if a.get_legend_handles_labels()[1]:
            a.legend(loc="best", fontsize=8)
    f1.tight_layout(rect=(0, 0, 1, 0.97))
    p = os.path.join(IMAGES_DIR, f"tracking_errors_{tag}.png")
    f1.savefig(p, dpi=150); saved.append(p)

    # ================= FIGURE 2 — recorded states =======================
    f2, ax = plt.subplots(3, 2, figsize=(11, 9))
    f2.suptitle(f"Recorded states — {suptag}", fontweight="bold")

    _plot_xyz(ax[0, 0], t, d["p"])
    ax[0, 0].set(title="Base position $r_0$", xlabel="$t$ (s)", ylabel=r"$r_0$ (m)")

    _plot_joints(ax[0, 1], t, d["q"], lambda j: fr"$q_{{{j}}}$", ref=d.get("q_d"))
    ax[0, 1].set(title=r"Joint angles $q$ (dashed = $q_d$)", xlabel="$t$ (s)", ylabel=r"$q$ (rad)")

    _plot_xyz(ax[1, 0], t, d["v0"])
    ax[1, 0].set(title="Base linear velocity $v_0$ (body)", xlabel="$t$ (s)", ylabel=r"$v_0$ (m/s)")

    _plot_xyz(ax[1, 1], t, d["omega0"],
              labels=(r"$\omega_x$", r"$\omega_y$", r"$\omega_z$"))
    ax[1, 1].set(title=r"Base angular velocity $\omega_0$ (body)", xlabel="$t$ (s)", ylabel=r"$\omega_0$ (rad/s)")

    _plot_joints(ax[2, 0], t, d["qdot"], lambda j: fr"$\dot{{q}}_{{{j}}}$")
    ax[2, 0].set(title=r"Joint rates $\dot{q}$", xlabel="$t$ (s)", ylabel=r"$\dot{q}$ (rad/s)")

    ax[2, 1].axis("off")               # spare slot (MATLAB fig 2 has 5 panels)

    for a in ax.flat:
        if a.has_data():
            a.grid(True, alpha=0.3)
            _mark_takeoff(a, t_takeoff)
            if a.get_legend_handles_labels()[1]:
                a.legend(loc="best", fontsize=8, ncol=2)
    f2.tight_layout(rect=(0, 0, 1, 0.97))
    p = os.path.join(IMAGES_DIR, f"states_{tag}.png")
    f2.savefig(p, dpi=150); saved.append(p)

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

    for a in ax.flat:
        a.grid(True, alpha=0.3)
        _mark_takeoff(a, t_takeoff)
        if a.get_legend_handles_labels()[1]:
            a.legend(loc="best", fontsize=8, ncol=2)
    f3.tight_layout(rect=(0, 0, 1, 0.97))
    p = os.path.join(IMAGES_DIR, f"control_{tag}.png")
    f3.savefig(p, dpi=150); saved.append(p)

    # ================= FIGURE 4 — 3D trajectories =======================
    f4 = plt.figure(figsize=(9.5, 8))
    ax3 = f4.add_subplot(111, projection="3d")
    f4.suptitle(f"3D trajectory — {suptag}", fontweight="bold")

    def _p3(P, **kw):
        ax3.plot(P[:, 0], P[:, 1], P[:, 2], **kw)

    _p3(d["p"],   color="0.2",        lw=1.6, label="base $r_0$")
    _p3(d["x_c"], color="tab:green",  lw=1.6, label="CoM $x_c$")
    _p3(d["x_cd"], color="tab:green", lw=1.2, ls="--", alpha=0.8, label="CoM desired $x_{cd}$")
    _p3(d["r_e"], color="tab:orange", lw=1.6, label="EE $r_e$")
    _p3(d["r_ed"], color="tab:orange", lw=1.2, ls="--", alpha=0.8, label="EE desired $r_{ed}$")

    for P, mk in ((d["x_c"], "o"), (d["r_e"], "p")):
        ax3.scatter(*P[0],  c="tab:blue", s=45, marker=mk, edgecolors="k", zorder=5)
        ax3.scatter(*P[-1], c="tab:red",  s=45, marker=mk, edgecolors="k", zorder=5)

    ax3.set_xlabel("$x_I$ (m)"); ax3.set_ylabel("$y_I$ (m)"); ax3.set_zlabel("$z_I$ (m)")
    ax3.legend(loc="upper left", fontsize=8)
    # equal-ish aspect
    pts = np.vstack([d["p"], d["x_c"], d["r_e"]])
    c = pts.mean(0); rng = (pts.max(0) - pts.min(0)).max() / 2 or 1.0
    ax3.set_xlim(c[0]-rng, c[0]+rng); ax3.set_ylim(c[1]-rng, c[1]+rng); ax3.set_zlim(c[2]-rng, c[2]+rng)
    p = os.path.join(IMAGES_DIR, f"trajectory3d_{tag}.png")
    f4.savefig(p, dpi=150); saved.append(p)

    print(f"Saved {len(saved)} figures to {IMAGES_DIR}/  (tag: {tag})")
    for s in saved:
        print("  " + os.path.relpath(s, SCRIPT_DIR))
    if show:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description="Plot an in-process aerial-manipulator run.")
    ap.add_argument("npz", nargs="?", default=DEFAULT_NPZ, help="path to track_log.npz")
    ap.add_argument("--tag", default=None, help="filename tag for the PNGs")
    ap.add_argument("--takeoff", type=float, default=None, help="takeoff-time boundary [s]")
    ap.add_argument("--show", action="store_true", help="open figures interactively")
    args = ap.parse_args()

    if not os.path.exists(args.npz):
        ap.error(f"log not found: {args.npz}")
    tag = args.tag or os.path.splitext(os.path.basename(args.npz))[0].replace("track_log", "run") or "run"
    t_takeoff = args.takeoff if args.takeoff is not None else _takeoff_default()
    make_plots(args.npz, tag, t_takeoff, args.show)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Real indoor T650 flight vs IsaacSim, compared on the controller-feedback topic
/uav_0/state_estimator/local_position/odom (ENU position, velocity and orientation).

Figures, following the X650 campaign's templates:
  1  <tag>_per_step.png      one panel per reference step: the axis that stepped, real vs
                             sim, with the reference as a dashed target line.
  2  <tag>_trajectory.png    whole run in x, y, z and yaw, real vs sim vs reference.
  3  <tag>_per_step_vel.png  the same steps, in velocity along the stepping axis.
  4  <tag>_attitude.png      whole run in roll, pitch and yaw.

Time axis: t = 0 at the FIRST reference waypoint on both sides. In simulation the reference
sequence was scheduled on PX4's clock and t = 0 is the driver's own seq_t0, so the two
timelines line up by construction.
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REAL_C, SIM_C, REF_C = "#c0392b", "#2471a3", "#555555"
PRE, POST_MAX = 1.0, 13.0


def quat_to_eul_deg(q):
    """q = [w,x,y,z]. Returns roll, pitch, yaw in degrees (ZYX)."""
    q = np.asarray(q, dtype=np.float64)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees(np.stack([roll, pitch, yaw], axis=1))


def wrap180(a):
    return (np.asarray(a, dtype=np.float64) + 180.0) % 360.0 - 180.0


def load_real(npz):
    d = np.load(npz)
    t0 = float(d["ref_t"][0])
    return dict(
        t=d["odom_t"] - t0, pos=d["odom_pos"], vel=d["odom_vel"],
        eul=quat_to_eul_deg(d["odom_quat"]), omega=d["odom_omega"],
        ref_t=d["ref_t"] - t0, ref_pos=d["ref_pos"],
        ref_yaw=wrap180(d["ref_yaw"]), ref_unit=d["ref_yaw_unit"],
    )


def load_sim(npz):
    d = np.load(npz)
    od = d["odom"]
    t0 = float(d["seq_t0_us"][0])
    # columns: 0 wall_us, 1 px4_us, 2:5 pos ENU, 5:8 vel, 8:12 quat(wxyz), 12:15 omega
    return dict(
        t=(od[:, 1] - t0) * 1e-6, pos=od[:, 2:5], vel=od[:, 5:8],
        eul=quat_to_eul_deg(od[:, 8:12]), omega=od[:, 12:15],
    )


def detect_steps(ref_t, ref_pos, ref_yaw, min_pos=0.05, min_yaw=2.0):
    steps = []
    for i in range(1, len(ref_t)):
        d = ref_pos[i] - ref_pos[i - 1]
        dy = wrap180(ref_yaw[i] - ref_yaw[i - 1])
        hold = (ref_t[i + 1] - ref_t[i]) if i + 1 < len(ref_t) else None
        for k, name in enumerate("xyz"):
            if abs(d[k]) >= min_pos:
                steps.append(dict(t=float(ref_t[i]), axis=name, k=k,
                                  frm=float(ref_pos[i - 1, k]), to=float(ref_pos[i, k]),
                                  size=float(d[k]), hold=hold))
        if abs(dy) >= min_yaw:
            steps.append(dict(t=float(ref_t[i]), axis="yaw", k=3,
                              frm=float(ref_yaw[i - 1]), to=float(ref_yaw[i]),
                              size=float(dy), hold=hold))
    for i, s in enumerate(steps):
        s["idx"] = i
    return steps


def series(side, axis, ref_yaw0=None):
    """(y, label) for a plotted axis. Yaw is unwrapped and, for the simulation, shifted to
    the real reference's yaw datum so both start from the same heading."""
    if axis == "yaw":
        y = np.degrees(np.unwrap(np.radians(side["eul"][:, 2])))
        return y, "yaw [deg]"
    k = "xyz".index(axis)
    return side["pos"][:, k], f"{axis} [m]"


def vel_series(side, axis):
    if axis == "yaw":
        return np.degrees(side["omega"][:, 2]), "yaw rate [deg/s]"
    k = "xyz".index(axis)
    return side["vel"][:, k], f"v{axis} [m/s]"


def win(t, y, t0, lo, hi):
    m = (t >= t0 - lo) & (t <= t0 + hi)
    return t[m] - t0, y[m]


def grid(n, ncol=4):
    nrow = int(np.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(4.6 * ncol, 3.1 * nrow), squeeze=False)
    return fig, axes, nrow, ncol


def fig_per_step(real, sim, steps, title, out, getter=series, show_ref=True):
    fig, axes, nrow, ncol = grid(len(steps))
    for i, st in enumerate(steps):
        ax = axes[i // ncol][i % ncol]
        hold = min(st["hold"] or POST_MAX, POST_MAX)
        for side, colour, name in ((real, REAL_C, "real"), (sim, SIM_C, "sim")):
            if side is None:
                continue
            y, ylab = getter(side, st["axis"])
            tw, yw = win(side["t"], y, st["t"], PRE, hold)
            if len(tw) < 3:
                continue
            ax.plot(tw, yw, color=colour, lw=1.4, label=name)
        if show_ref and getter is series:
            ax.axhline(st["to"], color=REF_C, lw=0.9, ls="--", label="reference")
        ax.axvline(0, color="0.75", lw=0.7, ls=":")
        unit = "deg" if st["axis"] == "yaw" else "m"
        ax.set_title(f"step {st['idx']}  {st['axis']} $\\to$ {st['to']:.2f} {unit}",
                     fontsize=10)
        ax.set_xlabel("t since step [s]", fontsize=8)
        ax.set_ylabel(getter(real, st["axis"])[1], fontsize=8)
        ax.tick_params(labelsize=8)
        ax.grid(alpha=0.25)
        if i == 0:
            ax.legend(fontsize=8, loc="best")
    for j in range(len(steps), nrow * ncol):
        axes[j // ncol][j % ncol].axis("off")
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.975])
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"wrote {out}")


def fig_trajectory(real, sim, steps, title, out):
    fig, axes = plt.subplots(4, 1, figsize=(15, 10), sharex=True)
    for k, axis in enumerate(["x", "y", "z", "yaw"]):
        ax = axes[k]
        for side, colour, name in ((real, REAL_C, "real"), (sim, SIM_C, "sim")):
            if side is None:
                continue
            y, ylab = series(side, axis)
            ax.plot(side["t"], y, color=colour, lw=1.0, label=name)
        # reference as a zero-order-hold staircase, same construction as the controller sees
        rt = np.asarray(real["ref_t"])
        rv = real["ref_yaw"] if axis == "yaw" else real["ref_pos"][:, "xyz".index(axis)]
        ax.step(np.append(rt, real["t"][-1]), np.append(rv, rv[-1]), where="post",
                color=REF_C, lw=1.0, ls="--", label="reference")
        ax.set_ylabel(series(real, axis)[1], fontsize=10)
        ax.grid(alpha=0.25)
        ax.tick_params(labelsize=9)
        if k == 0:
            ax.legend(fontsize=9, ncol=3, loc="upper left")
    axes[-1].set_xlabel("time since first reference waypoint [s]", fontsize=11)
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"wrote {out}")


def fig_attitude(real, sim, title, out):
    fig, axes = plt.subplots(3, 1, figsize=(15, 8), sharex=True)
    for k, name in enumerate(["roll", "pitch", "yaw"]):
        ax = axes[k]
        for side, colour, lbl in ((real, REAL_C, "real"), (sim, SIM_C, "sim")):
            if side is None:
                continue
            y = (np.degrees(np.unwrap(np.radians(side["eul"][:, 2]))) if k == 2
                 else side["eul"][:, k])
            ax.plot(side["t"], y, color=colour, lw=0.9, label=lbl)
        ax.set_ylabel(f"{name} [deg]", fontsize=10)
        ax.grid(alpha=0.25)
        ax.tick_params(labelsize=9)
        if k == 0:
            ax.legend(fontsize=9, ncol=2, loc="upper left")
    axes[-1].set_xlabel("time since first reference waypoint [s]", fontsize=11)
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"wrote {out}")


def main():
    tag = sys.argv[1]
    real = load_real(f"exp_{tag}.npz")
    sim_npz = sys.argv[2] if len(sys.argv) > 2 else None
    sim = load_sim(sim_npz) if sim_npz and os.path.exists(sim_npz) else None
    outdir = sys.argv[3] if len(sys.argv) > 3 else "figures"
    os.makedirs(outdir, exist_ok=True)

    steps = detect_steps(real["ref_t"], real["ref_pos"], real["ref_yaw"])
    print(f"{len(steps)} steps detected")
    for s in steps:
        h = f"{s['hold']:.1f}" if s["hold"] else "-"
        print(f"  {s['idx']:2d} t={s['t']:7.2f}  {s['axis']:>3} {s['frm']:6.2f} -> "
              f"{s['to']:6.2f}  hold {h}")

    what = "real indoor flight vs IsaacSim" if sim else "real indoor flight (sim pending)"
    fig_per_step(real, sim, steps,
                 f"T650 baseline controller, flight {tag}: {what}, same step commands",
                 f"{outdir}/{tag}_per_step.png")
    fig_trajectory(real, sim, steps,
                   f"Full run overlay, flight {tag} (ENU, controller feedback topic)",
                   f"{outdir}/{tag}_trajectory.png")
    fig_per_step(real, sim, steps,
                 f"T650 flight {tag}: velocity along the stepping axis, {what}",
                 f"{outdir}/{tag}_per_step_vel.png", getter=vel_series, show_ref=False)
    fig_attitude(real, sim,
                 f"T650 flight {tag}: attitude (Euler, controller feedback topic)",
                 f"{outdir}/{tag}_attitude.png")


if __name__ == "__main__":
    main()

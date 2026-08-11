#!/usr/bin/env python3
"""Per-step and full-run figures: real indoor T650 flight vs IsaacSim, same step commands.

These bags contain no measured position (only sensor_combined, vehicle_attitude,
vehicle_status and the setpoints), so a position step response cannot be plotted for the
real vehicle. What every panel shows instead is the quantity the position controller
actually emits and the vehicle actually achieves:

  x / y step  -> horizontal specific acceleration projected on the step direction. This is
                 what a horizontal position step commands, it is heading-independent, and it
                 is recoverable on BOTH sides from the attitude setpoint (commanded) and
                 vehicle_attitude (achieved).
  z step      -> normalized thrust command, plus measured -accel_z.
  yaw step    -> yaw angle, commanded and achieved.

Simulated position is drawn only in the full-run figure, clearly marked sim-only.
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from steps import (G, detect_steps, euler_deg, horizontal_accel_ned, ref_onsets,
                   step_direction_ned)

REAL_C, SIM_C = "#c0392b", "#2471a3"
PRE, POST_MAX = 1.5, 13.0


def load_side(npz, kind):
    """Return dict of trace arrays on the replay-time axis.

    kind='real': t=0 at the first recorded attitude setpoint.
    kind='sim' : t=0 at the instant the driver started the reference timeline, which was
                 aligned to the real t=0 by construction.
    """
    d = np.load(npz)
    if kind == "real":
        # On hardware PX4 and ROS share a clock (uXRCE-DDS time sync works because PX4
        # runs in real time), so every stream is already on one axis.
        t0 = float(d["sp_ts"][0])
        s = lambda k: (d[k].astype(np.float64) - t0) * 1e-6
        t_sp = s("sp_ts")
    else:
        # In simulation they are NOT one axis. sensor_combined / vehicle_attitude are
        # stamped by PX4 (simulation time, advancing at ~0.84x wall under lockstep), while
        # the attitude setpoints are stamped by the controller on ROS wall clock. Mixing
        # them silently misplaces every commanded trace by tens of seconds. The odom
        # record carries both clocks per sample, so fit wall -> simulation time from it.
        t0 = float(d["seq_t0_us"][0])
        s = lambda k: (d[k].astype(np.float64) - t0) * 1e-6
        # The ratio is NOT constant -- Isaac's step rate varies through the run -- so an
        # affine fit is the wrong model (59 ms RMS on flight A). Interpolate through the
        # recorded pairs instead, which tracks a varying ratio exactly.
        od = d["odom"]
        w, p = od[:, 0].astype(np.float64), od[:, 1].astype(np.float64)
        keep = np.concatenate([[True], np.diff(w) > 0])
        w, p = w[keep], p[keep]
        wall_to_sim = lambda us: np.interp(us.astype(np.float64), w, p)

        # Validate against an independent set of pairs: the reference log records both
        # clocks at every waypoint switch and was not used to build the map.
        rl = d["reflog"]
        if len(rl) > 1:
            err = (wall_to_sim(rl[:, 0]) - rl[:, 1]) * 1e-6
            rms = float(np.sqrt(np.mean(err**2)))
            if rms > 0.05:
                raise RuntimeError(f"wall->sim clock map is poor ({rms*1e3:.1f} ms RMS)")
        t_sp = (wall_to_sim(d["sp_ts"]) - t0) * 1e-6

    out = {
        "t_sc": s("sc_ts"), "gyro": d["sc_gyro"], "accel": d["sc_accel"],
        "t_att": s("att_ts"), "att_q": d["att_q"],
        "t_sp": t_sp, "sp_q": d["sp_q_d"], "sp_thr": -d["sp_thrust"][:, 2],
    }
    out["att_eul"] = euler_deg(out["att_q"])
    out["sp_eul"] = euler_deg(out["sp_q"])
    out["att_ah"] = horizontal_accel_ned(out["att_q"])
    out["sp_ah"] = horizontal_accel_ned(out["sp_q"])
    if kind == "sim" and "odom" in d and len(d["odom"]):
        od = d["odom"]
        # Column 0 is wall clock and column 1 is PX4/simulation time; the analysis axis is
        # the latter. The estimator publishes nav_msgs/Odometry already in ENU (the
        # NED->ENU conversion happens inside Px4FusedOdomBridge), so columns 2:5 are used
        # as-is -- converting again would scramble the axes.
        out["t_odom"] = (od[:, 1] - t0) * 1e-6
        out["odom_enu"] = od[:, 2:5]
    return out


def win(t, y, t0, lo, hi):
    m = (t >= t0 - lo) & (t <= t0 + hi)
    return t[m] - t0, y[m]


def unwrap_deg(y):
    return np.degrees(np.unwrap(np.radians(y)))


def channel(side, step, want):
    """(t, y, label) for this step's primary channel. want in {'cmd','meas'}.

    One physical quantity per step type, in the same units for commanded and achieved so
    both can share an axis:
      x / y  -> horizontal specific acceleration projected on the step direction
      z      -> vertical specific acceleration (-a_z), i.e. the measured climb command
      yaw    -> yaw angle
    """
    ax = step["axis"]
    if ax in ("x", "y"):
        v = step_direction_ned(step)
        if want == "cmd":
            return side["t_sp"], side["sp_ah"] @ v, r"$\Delta a_h$ along step [m/s$^2$]"
        return side["t_att"], side["att_ah"] @ v, r"$\Delta a_h$ along step [m/s$^2$]"
    if ax == "yaw":
        if want == "cmd":
            return side["t_sp"], unwrap_deg(side["sp_eul"][:, 2]), r"$\Delta$yaw [deg]"
        return side["t_att"], unwrap_deg(side["att_eul"][:, 2]), r"$\Delta$yaw [deg]"
    # Vertical steps: use the thrust command, not measured -a_z. On hardware the vertical
    # accelerometer is dominated by airframe vibration (+-0.15 m/s^2, ~20x the simulated
    # level -- see the report), which buries a 0.3-0.5 m altitude step entirely. The
    # thrust command is the controller's actual vertical output and is clean on both sides.
    if want == "cmd":
        return side["t_sp"], side["sp_thr"], r"$\Delta$thrust cmd [-]"
    return None, None, r"$\Delta$thrust cmd [-]"


def smooth(t, y, fc=6.0):
    """Zero-phase single-pole low-pass; strips airframe vibration so the rigid-body step
    response is visible. Applied identically to both sides."""
    if len(t) < 3:
        return y
    dt = np.gradient(t)
    a = np.clip(1 - np.exp(-2 * np.pi * fc * dt), 0, 1)
    f = np.empty_like(y); f[0] = y[0]
    for i in range(1, len(y)):
        f[i] = f[i - 1] + a[i] * (y[i] - f[i - 1])
    b = np.empty_like(f); b[-1] = f[-1]
    for i in range(len(f) - 2, -1, -1):
        b[i] = b[i + 1] + a[i] * (f[i] - b[i + 1])
    return b


def fig_per_step(real, sim, stepsA, title, out, fc=6.0):
    n = len(stepsA)
    ncol = 4
    nrow = int(np.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(4.6 * ncol, 3.2 * nrow), squeeze=False)
    for i, st in enumerate(stepsA):
        ax = axes[i // ncol][i % ncol]
        hold = min(st["hold"] or POST_MAX, POST_MAX)
        for side, colour, name in ((real, REAL_C, "real"), (sim, SIM_C, "sim")):
            if side is None:
                continue
            # z steps have only the commanded channel, so draw it as the primary line.
            styles = (("cmd", "-", 1.6),) if st["axis"] == "z" else \
                     (("meas", "-", 1.6), ("cmd", "--", 1.0))
            for want, ls, lw in styles:
                t, y, ylab = channel(side, st, want)
                if t is None:
                    continue
                tw, yw = win(t, y, st["t"], PRE, hold)
                if len(tw) < 3:
                    continue
                yw = smooth(tw, yw, fc)
                # Remove the pre-step level. Real and simulated vehicles hover with
                # different persistent trim (opposite sign, ~0.3 m/s^2 each -- see the
                # report), which is a genuine constant offset but not part of the step
                # response. Subtracting it compares response SHAPE, the same choice the
                # earlier X650 campaign made.
                base = yw[tw < -0.2]
                if len(base):
                    yw = yw - base.mean()
                ax.plot(tw, yw, ls, color=colour, lw=lw,
                        alpha=1.0 if (want == "meas" or st["axis"] == "z") else 0.55,
                        label=(name if st["axis"]=="z" else f"{name} {'achieved' if want=='meas' else 'commanded'}"))
        ax.axvline(0, color="0.6", lw=0.8, ls=":")
        ax.axhline(0, color="0.85", lw=0.8)
        unit = "deg" if st["axis"] == "yaw" else "m"
        ax.set_title(f"step {st['idx']}  {st['axis']} {st['frm']:.2f} $\\to$ "
                     f"{st['to']:.2f} {unit}", fontsize=10)
        ax.set_xlabel("t since step [s]", fontsize=8)
        ax.set_ylabel(channel(real, st, "meas")[2], fontsize=8)
        ax.tick_params(labelsize=8)
        ax.grid(alpha=0.25)
        if i == 0:
            ax.legend(fontsize=7, loc="best")
    for j in range(n, nrow * ncol):
        axes[j // ncol][j % ncol].axis("off")
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"wrote {out}")


def fig_full_run(real, sim, steps, t_ref, ref, title, out, fc=6.0):
    have_pos = sim is not None and "odom_enu" in sim
    rows = 5 if have_pos else 4
    fig, axes = plt.subplots(rows, 1, figsize=(15, 2.5 * rows), sharex=True)
    panels = [
        ("roll [deg]", lambda s: (s["t_att"], s["att_eul"][:, 0])),
        ("pitch [deg]", lambda s: (s["t_att"], s["att_eul"][:, 1])),
        ("yaw [deg]", lambda s: (s["t_att"], unwrap_deg(s["att_eul"][:, 2]))),
        ("thrust cmd [-]", lambda s: (s["t_sp"], s["sp_thr"])),
    ]
    for k, (ylab, getter) in enumerate(panels):
        ax = axes[k]
        for side, colour, name in ((real, REAL_C, "real"), (sim, SIM_C, "sim")):
            if side is None:
                continue
            t, y = getter(side)
            ax.plot(t, smooth(t, y, fc), color=colour, lw=1.1, label=name)
        for st in steps:
            ax.axvline(st["t"], color="0.75", lw=0.6, ls=":")
        ax.set_ylabel(ylab, fontsize=9)
        ax.grid(alpha=0.25)
        ax.tick_params(labelsize=8)
        if k == 0:
            ax.legend(fontsize=8, ncol=3, loc="upper right")
    if have_pos:
        ax = axes[4]
        for k, (name, colour) in enumerate((("x", "#1b7837"), ("y", "#762a83"),
                                            ("z", "#b35806"))):
            ax.plot(sim["t_odom"], sim["odom_enu"][:, k], color=colour, lw=1.0,
                    label=f"sim {name}")
            ax.step(t_ref, ref["position"][:, k], where="post", color=colour, lw=0.9,
                    ls="--", alpha=0.6)
        ax.set_ylabel("position [m]\n(sim only)", fontsize=9)
        ax.grid(alpha=0.25)
        ax.tick_params(labelsize=8)
        ax.legend(fontsize=8, ncol=3, loc="upper right")
    axes[-1].set_xlabel("time since first recorded attitude setpoint [s]", fontsize=10)
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"wrote {out}")


def fig_sensor_steps(real, sim, steps, title, out):
    """sensor_combined per step -- the quantity the study was originally asked to compare."""
    n = len(steps)
    ncol = 4
    nrow = int(np.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(4.6 * ncol, 3.2 * nrow), squeeze=False)
    for i, st in enumerate(steps):
        ax = axes[i // ncol][i % ncol]
        hold = min(st["hold"] or POST_MAX, POST_MAX)
        for side, colour, name in ((real, REAL_C, "real"), (sim, SIM_C, "sim")):
            if side is None:
                continue
            g = np.linalg.norm(side["gyro"], axis=1)
            tw, yw = win(side["t_sc"], g, st["t"], PRE, hold)
            if len(tw) < 3:
                continue
            ax.plot(tw, yw, color=colour, lw=0.7, alpha=0.35)
            ax.plot(tw, smooth(tw, yw, 4.0), color=colour, lw=1.6, label=name)
        ax.axvline(0, color="0.6", lw=0.8, ls=":")
        unit = "deg" if st["axis"] == "yaw" else "m"
        ax.set_title(f"step {st['idx']}  {st['axis']} $\\to$ {st['to']:.2f} {unit}",
                     fontsize=10)
        ax.set_xlabel("t since step [s]", fontsize=8)
        ax.set_ylabel(r"$|\omega|$ [rad/s]", fontsize=8)
        ax.tick_params(labelsize=8)
        ax.grid(alpha=0.25)
        if i == 0:
            ax.legend(fontsize=8)
    for j in range(n, nrow * ncol):
        axes[j // ncol][j % ncol].axis("off")
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"wrote {out}")


def main():
    tag = sys.argv[1]
    real_npz = f"exp_{tag}.npz"
    sim_npz = sys.argv[2] if len(sys.argv) > 2 else None
    outdir = sys.argv[3] if len(sys.argv) > 3 else "figures"
    os.makedirs(outdir, exist_ok=True)

    t_ref, ref, _ = ref_onsets(real_npz, f"ref_{tag}.npz")
    steps = detect_steps(t_ref, ref)
    real = load_side(real_npz, "real")
    sim = load_side(sim_npz, "sim") if sim_npz and os.path.exists(sim_npz) else None

    what = "real indoor flight vs IsaacSim" if sim else "real indoor flight (sim pending)"
    base = f"T650 baseline controller, flight {tag}: {what}, same step commands"
    fig_per_step(real, sim, steps, base, f"{outdir}/{tag}_per_step.png")
    fig_full_run(real, sim, steps, t_ref, ref, f"Full run overlay - flight {tag}",
                 f"{outdir}/{tag}_full_run.png")
    fig_sensor_steps(real, sim, steps,
                     f"sensor_combined gyro magnitude per step - flight {tag}",
                     f"{outdir}/{tag}_sensor_steps.png")


if __name__ == "__main__":
    main()

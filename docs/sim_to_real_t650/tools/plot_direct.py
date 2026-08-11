#!/usr/bin/env python3
"""Real indoor T650 DIRECT-actuation flight vs IsaacSim, compared on the controller
feedback topic /uav_0/state_estimator/local_position/odom (ENU).

Figures follow the baseline campaign's templates exactly, so a DIRECT figure can be laid
next to figures/baseline/C_per_step.png and read the same way:

  <tag>_per_step.png      one panel per reference step, real vs sim, reference dashed
  <tag>_trajectory.png    whole run in x, y, z, yaw
  <tag>_per_step_vel.png  the same steps in velocity / body rate along the stepping axis
  <tag>_attitude.png      whole run in roll, pitch, yaw
  <tag>_motors.png        the DIRECT-only inner loop: the four motor commands

Time axis: t = 0 at DIRECT ENGAGEMENT on both sides -- not at the first waypoint, because
both real flights took off and manoeuvred under baseline control first and only that
DIRECT segment is comparable. make_direct_refs.py uses the same origin, and the driver
sets seq_t0 at its own engagement instant, so the two timelines line up by construction.

  plot_direct.py <tag> <real_npz> <ref_npz> [sim_npz] [outdir]
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from plot_odom import (REAL_C, REF_C, SIM_C, detect_steps, fig_per_step,
                       fig_trajectory, quat_to_eul_deg, series, usable_steps,
                       wrap180)


MAX_PHYSICAL_YAW_RATE = 2000.0   # deg/s; the commanded limit is 360, so this is generous


def yaw_glitch_mask(t, quat, verbose=True):
    """True for odometry samples whose reported heading is not physically reachable.

    Flight D2's estimator output contains isolated single samples where the reported yaw
    jumps ~180 deg and returns on the very next sample -- 10,600-11,000 deg/s against a
    360 deg/s command limit. The quaternions are still exactly unit norm, so this is not
    corruption in transport; it is the orientation itself flipping for one sample.

    They matter out of all proportion to their number (6 samples in 13488). They were the
    actual cause of the np.unwrap divergence that sent the D2 yaw trace to +366 deg, and
    left in they would contaminate any yaw RMS, correlation or step metric that happens to
    span one. Rejecting a sample as unphysical is only defensible if it is announced, so
    the count and timestamps are always printed.
    """
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    bad = np.zeros(len(yaw), dtype=bool)
    d_prev = np.abs(wrap180(np.diff(yaw))) / np.maximum(np.diff(t), 1e-9)
    # a spike is an outlier against BOTH neighbours, which a genuine fast turn is not
    bad[1:-1] = (d_prev[:-1] > MAX_PHYSICAL_YAW_RATE) & (d_prev[1:] > MAX_PHYSICAL_YAW_RATE)
    if verbose and bad.any():
        print(f"  rejected {bad.sum()} unphysical yaw sample(s) at t = "
              f"{np.array2string(t[bad], precision=2, separator=', ')} s "
              f"(>{MAX_PHYSICAL_YAW_RATE:.0f} deg/s)")
    return bad


def load_real(real_npz, ref_npz):
    """Real flight, sliced to the DIRECT segment, t = 0 at DIRECT engagement."""
    d = np.load(real_npz, allow_pickle=True)
    r = np.load(ref_npz)
    t_bag0 = float(d["odom_t"][0])
    lo, hi = [float(v) for v in r["direct_window"]]     # relative to bag start
    t = d["odom_t"] - t_bag0
    m = (t >= lo) & (t <= hi) & ~yaw_glitch_mask(t, d["odom_quat"])
    return dict(
        t=t[m] - lo, pos=d["odom_pos"][m], vel=d["odom_vel"][m],
        eul=quat_to_eul_deg(d["odom_quat"][m]), omega=d["odom_omega"][m],
        ref_t=r["t_rel"], ref_pos=r["position"],
        ref_yaw=wrap180(r["yaw"]), ref_unit=r["yaw_unit"],
        mot_t=(d["mot_t"] - t_bag0 - lo) if "mot_t" in d else None,
        mot=d["mot_ctrl"] if "mot_ctrl" in d else None,
    )


def load_sim(npz):
    """Driver recording. odom columns: 0 wall_us, 1 px4_us, 2 hdr_s, 3:6 pos, 6:9 vel,
    9:13 quat(wxyz), 13:16 omega. t = 0 at the driver's seq_t0 (DIRECT engagement)."""
    d = np.load(npz, allow_pickle=True)
    od = d["odom"]
    if len(od) == 0:
        return None
    t0 = float(d["seq_t0_us"][0])
    out = dict(
        t=(od[:, 1] - t0) * 1e-6, pos=od[:, 3:6], vel=od[:, 6:9],
        eul=quat_to_eul_deg(od[:, 9:13]), omega=od[:, 13:16],
    )
    mot = d["mot"]
    out["mot_t"] = (mot[:, 1] - t0) * 1e-6 if len(mot) else None
    out["mot"] = mot[:, 2:6] if len(mot) else None
    return out


def fig_motors(real, sim, title, out):
    """The four normalized motor commands -- the DIRECT loop's actual output, and the one
    signal that simply does not exist in baseline mode (PX4 mixes internally there)."""
    fig, axes = plt.subplots(2, 1, figsize=(15, 7), sharex=True)
    for ax, side, colour, name in ((axes[0], real, REAL_C, "real"),
                                   (axes[1], sim, SIM_C, "sim")):
        if side is None or side.get("mot") is None:
            ax.axis("off")
            continue
        for j in range(4):
            ax.plot(side["mot_t"], side["mot"][:, j], lw=0.6, label=f"motor {j}")
        ax.set_ylabel(f"{name}\nmotor command [-]", fontsize=10)
        ax.grid(alpha=0.25)
        ax.tick_params(labelsize=9)
        ax.legend(fontsize=8, ncol=4, loc="upper right")
    axes[-1].set_xlabel("time since DIRECT engagement [s]", fontsize=11)
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"wrote {out}")


def main():
    tag = sys.argv[1]
    real = load_real(sys.argv[2], sys.argv[3])
    sim_npz = sys.argv[4] if len(sys.argv) > 4 else None
    sim = load_sim(sim_npz) if sim_npz and os.path.exists(sim_npz) else None
    outdir = sys.argv[5] if len(sys.argv) > 5 else "figures/direct"
    os.makedirs(outdir, exist_ok=True)

    steps = usable_steps(detect_steps(real["ref_t"], real["ref_pos"], real["ref_yaw"]))

    what = ("real indoor flight vs IsaacSim" if sim
            else "real indoor flight (sim pending)")
    fig_per_step(real, sim, steps,
                 f"T650 DIRECT actuation, flight {tag}: {what}, same step commands",
                 f"{outdir}/{tag}_per_step.png")
    fig_trajectory(real, sim, steps,
                   f"T650 DIRECT actuation, flight {tag}: full run overlay "
                   f"(ENU, controller feedback topic)",
                   f"{outdir}/{tag}_trajectory.png")


if __name__ == "__main__":
    main()

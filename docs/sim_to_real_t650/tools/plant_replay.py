#!/usr/bin/env python3
"""Same-command plant test: drive the SIMULATOR'S OWN plant model with the motor commands
the real vehicle actually flew, and compare the resulting rigid-body response against what
the real vehicle actually did.

Why this and not a closed-loop trajectory comparison
---------------------------------------------------
In closed loop the controller reacts to the plant, so a plant error and the controller's
compensation for it are entangled -- a trajectory can match well while the inner loop is
wrong (the baseline campaign found exactly that: position shape RMSE 5.5 cm while the
acceleration producing it was off by +22.6% in peak). Feeding the RECORDED commands makes
the input identical by construction, so every difference that remains is plant.

Why not replay the commands through Isaac open-loop
--------------------------------------------------
A multirotor is open-loop unstable in attitude. Replaying 160 s of recorded motor commands
with no feedback diverges within a second or two and tells you nothing. The well-posed
version of "same command, compare state" is the INSTANTANEOUS response -- angular
acceleration and specific thrust -- which is what the plant parameters actually set and
what this compares. It uses the same LaggedQuadraticThrustCurve maths the simulator runs.

What each output isolates
-------------------------
  tau_x, tau_y   ->  k * arm / Ixx,Iyy      roll/pitch authority
  tau_z          ->  c / Izz                yaw authority (the suspected term)
  specific thrust->  k / mass               hover/thrust calibration
Any per-axis gain error reported here is a direct multiplier on the parameter above it.

  plant_replay.py <real_npz> <out_prefix> [--lam L] [--k K] [--c C]
"""
import argparse
import json
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, "/home/fsc-jupiter/Source/fsc_PegasusSimulator/extensions/"
                   "fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft")
import importlib.util
_spec = importlib.util.spec_from_file_location(
    "t650_params", "/home/fsc-jupiter/Source/fsc_PegasusSimulator/extensions/"
    "fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/t650_params.py")
P = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(P)

REAL_C, SIM_C = "#c0392b", "#2471a3"

# Rotor geometry in PX4 FRD (x forward, y right, z down), matching the deployed
# direct-actuation config's alloc_rotor*_p{x,y} exactly.
A = P.ARM_LENGTH / np.sqrt(2.0)
ROTOR_FRD = np.array([[+A, +A], [-A, -A], [+A, -A], [-A, +A]])   # motors 0..3
KM_SIGN = np.array([+1.0, +1.0, -1.0, -1.0])


def lag_omega(u, dt, lam, w_idle, w_max):
    """Exact discrete (zero-order-hold) first-order lag, the same update
    LaggedQuadraticThrustCurve applies: w[k+1] = wc + (w[k]-wc)*exp(-lam*dt)."""
    wc = np.clip(u, 0.0, 1.0) * (w_max - w_idle) + w_idle
    w = np.empty_like(wc)
    w[0] = wc[0]
    for i in range(1, len(wc)):
        decay = np.exp(-lam * dt[i - 1])
        w[i] = wc[i] + (w[i - 1] - wc[i]) * decay
    return w


def predict(u, t, lam, k, c, mass, inertia):
    """Motor commands -> predicted body torques, angular accel and specific thrust."""
    dt = np.diff(t)
    w = np.stack([lag_omega(u[:, j], dt, lam, P.ZERO_POSITION_ARMED, P.MAX_ROTOR_VEL)
                  for j in range(4)], axis=1)
    f = k * w ** 2                                    # N per rotor, along -z_body (FRD)
    # tau = sum r_i x F_i  with F_i = (0,0,-f_i):  tau_x = -sum py*f, tau_y = +sum px*f
    tau = np.stack([
        -(f * ROTOR_FRD[:, 1]).sum(axis=1),
        +(f * ROTOR_FRD[:, 0]).sum(axis=1),
        (c * w ** 2 * KM_SIGN).sum(axis=1),
    ], axis=1)
    alpha = tau / inertia                             # small-rate approx, no gyroscopic term
    a_z = -f.sum(axis=1) / mass                       # accelerometer reads -T/m in FRD hover
    return dict(omega=w, force=f, tau=tau, alpha=alpha, a_z=a_z)


def lp(x, fs, fc):
    """Zero-phase low-pass, windowed-sinc FIR.

    Deliberately not scipy: this repo's analysis toolchain is the system python3 with
    numpy 2.x, and the installed scipy 1.8 is compiled against numpy 1.x and fails to
    import. A symmetric (linear-phase) kernel introduces no group delay at all, which is
    what matters here -- a phase shift between the measured and predicted traces would
    masquerade as a plant timing error, which is precisely what is being measured.
    """
    numtaps = int(4 * fs / fc) | 1                     # odd, so the kernel is centred
    n = np.arange(numtaps) - (numtaps - 1) / 2.0
    h = np.sinc(2.0 * fc / fs * n) * np.hamming(numtaps)
    h /= h.sum()
    pad = numtaps // 2

    def one(v):
        vp = np.concatenate([np.full(pad, v[0]), v, np.full(pad, v[-1])])
        return np.convolve(vp, h, mode="valid")

    if x.ndim == 1:
        return one(x)
    return np.stack([one(x[:, j]) for j in range(x.shape[1])], axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("real_npz")
    ap.add_argument("out_prefix")
    ap.add_argument("--lam", type=float, default=P.ROTOR_LAMBDA)
    ap.add_argument("--k", type=float, default=P.ROTOR_CONSTANT)
    ap.add_argument("--c", type=float, default=P.ROLLING_MOMENT_COEFFICIENT)
    ap.add_argument("--mass", type=float, default=P.MASS)
    ap.add_argument("--fc", type=float, default=12.0, help="analysis low-pass [Hz]")
    ap.add_argument("--fhp", type=float, default=0.2,
                    help="analysis high-pass [Hz]: removes the standing trim so the "
                         "dynamic response is compared about it")
    args = ap.parse_args()

    d = np.load(args.real_npz, allow_pickle=True)
    # PX4 hrt stamps and the controller's ROS stamps share one epoch on hardware, so both
    # streams can use their own timestamp field -- far cleaner than rosbag receive time.
    t_sc = d["sc_ts"].astype(np.float64) * 1e-6
    t_mot = d["mot_ts"].astype(np.float64) * 1e-6
    u_all = d["mot_ctrl"].astype(np.float64)

    good = ~np.isnan(u_all).any(axis=1)               # NaN whenever DIRECT is not driving
    t_mot, u_all = t_mot[good], u_all[good]
    lo, hi = t_mot[0], t_mot[-1]

    m = (t_sc >= lo) & (t_sc <= hi)
    t = t_sc[m]
    gyro = d["sc_gyro"][m].astype(np.float64)
    accel = d["sc_accel"][m].astype(np.float64)

    # Zero-order hold: motor commands are held between updates, which is what both the ESC
    # and the simulator see.
    idx = np.searchsorted(t_mot, t, side="right") - 1
    u = u_all[np.clip(idx, 0, len(u_all) - 1)]

    inertia = P.INERTIA_DIAG
    pred = predict(u, t, args.lam, args.k, args.c, args.mass, inertia)

    fs = 1.0 / np.median(np.diff(t))
    # Measured angular acceleration: differentiate the gyro, then low-pass. The SAME filter
    # is applied to the prediction so the comparison is not a filter artefact.
    alpha_meas_raw = np.gradient(gyro, t, axis=0)
    alpha_meas = lp(alpha_meas_raw, fs, args.fc)
    alpha_pred_dc = lp(pred["alpha"], fs, args.fc)
    az_meas = lp(accel[:, 2], fs, args.fc)
    az_pred = lp(pred["a_z"], fs, args.fc)

    # --- separate the standing trim from the dynamics -------------------------------
    # The real airframe needs markedly asymmetric motor commands to hold attitude (CoM
    # offset / motor-and-prop asymmetry). Driven through a PERFECTLY SYMMETRIC model, those
    # same commands imply a large constant torque that the real vehicle plainly is not
    # experiencing -- its mean gyro is ~0. That difference is real and is reported below as
    # a trim, but it is a static airframe property, not a dynamic-response error, and left
    # in it would swamp the gains completely. So both traces are high-passed at --fhp and
    # the dynamic response is compared about each one's own trim.
    hp = lambda x: x - lp(x, fs, args.fhp)
    alpha_pred = hp(alpha_pred_dc)
    alpha_meas_ac = hp(alpha_meas)
    trim_tau = lp(pred["tau"], fs, args.fc).mean(axis=0)
    thrust_mean = -az_pred.mean() * args.mass

    # Per-axis gain: least-squares slope of measured vs predicted, through the origin.
    # >1 means the real vehicle produced MORE response than the model for the same command.
    res = {"file": args.real_npz, "window_s": float(hi - lo), "fs_hz": float(fs),
           "params": {"lambda": args.lam, "k": args.k, "c": args.c, "mass": args.mass,
                      "inertia": [float(v) for v in inertia]}}
    names = ["roll", "pitch", "yaw"]
    print(f"\nDIRECT window {hi-lo:.1f}s  |  {len(t)} samples @ {fs:.1f} Hz  |  "
          f"band {args.fhp:g}-{args.fc:g} Hz")
    print("\n--- standing trim the real airframe carries and the symmetric model does not "
          "---")
    for j, nm in enumerate(names):
        print(f"  {nm:6} model-implied constant torque {trim_tau[j]:+8.4f} N.m"
              f"   -> {trim_tau[j]/inertia[j]:+7.3f} rad/s^2 the sim would feel")
    print(f"  equivalent CoM offset: x {trim_tau[1]/thrust_mean*1e3:+6.1f} mm, "
          f"y {-trim_tau[0]/thrust_mean*1e3:+6.1f} mm  (thrust {thrust_mean:.2f} N)")
    res["trim_tau_nm"] = [float(v) for v in trim_tau]
    res["com_offset_mm"] = [float(trim_tau[1] / thrust_mean * 1e3),
                            float(-trim_tau[0] / thrust_mean * 1e3)]

    print("\n--- dynamic response about that trim (what the plant parameters set) ---")
    print(f"{'axis':6} {'meas RMS':>10} {'pred RMS':>10} {'gain':>7} {'corr':>7} "
          f"{'implied change':>28}")
    for j, nm in enumerate(names):
        mm, pp = alpha_meas_ac[:, j], alpha_pred[:, j]
        gain = float((pp @ mm) / (pp @ pp))
        corr = float(np.corrcoef(mm, pp)[0, 1])
        target = ("c" if j == 2 else "k*arm") + f" x {gain:.3f}"
        alt = f"or I{'xyz'[j]}{'xyz'[j]} / {gain:.3f}"
        res[nm] = {"gain": gain, "corr": corr,
                   "rms_meas": float(np.sqrt((mm ** 2).mean())),
                   "rms_pred": float(np.sqrt((pp ** 2).mean()))}
        print(f"{nm:6} {np.sqrt((mm**2).mean()):10.3f} {np.sqrt((pp**2).mean()):10.3f} "
              f"{gain:7.3f} {corr:7.3f} {target:>16} {alt:>14}")

    gz = float((az_pred @ az_meas) / (az_pred @ az_pred))
    res["thrust"] = {"gain": gz,
                     "mean_meas": float(az_meas.mean()), "mean_pred": float(az_pred.mean())}
    print(f"{'thrust':6} {az_meas.mean():10.3f} {az_pred.mean():10.3f} {gz:7.3f} "
          f"{'':7} {'k/mass x '+f'{gz:.3f}':>16}")
    print("\n(gain > 1 = the real vehicle produced MORE response than the model for the "
          "same command)")

    with open(f"{args.out_prefix}_plant.json", "w") as fh:
        json.dump(res, fh, indent=2)
    print(f"wrote {args.out_prefix}_plant.json")

    # ---- figure: measured vs model-predicted response to the identical commands ----
    trel = t - t[0]
    fig, axes = plt.subplots(4, 1, figsize=(15, 11), sharex=True)
    for j, nm in enumerate(names):
        ax = axes[j]
        ax.plot(trel, alpha_meas_ac[:, j], color=REAL_C, lw=0.8, label="real (measured)")
        ax.plot(trel, alpha_pred[:, j], color=SIM_C, lw=0.8,
                label="sim plant model (same commands)")
        ax.set_ylabel(f"{nm} ang. accel\n[rad/s$^2$]", fontsize=10)
        ax.grid(alpha=0.25)
        ax.tick_params(labelsize=9)
        ax.set_title(f"{nm}: gain {res[nm]['gain']:.3f}, corr {res[nm]['corr']:.3f}",
                     fontsize=9, loc="right")
        if j == 0:
            ax.legend(fontsize=9, ncol=2, loc="upper left")
    axes[3].plot(trel, az_meas, color=REAL_C, lw=0.8)
    axes[3].plot(trel, az_pred, color=SIM_C, lw=0.8)
    axes[3].set_ylabel("specific thrust\n$a_z$ [m/s$^2$]", fontsize=10)
    axes[3].set_title(f"thrust: gain {gz:.3f}", fontsize=9, loc="right")
    axes[3].grid(alpha=0.25)
    axes[3].set_xlabel("time since DIRECT engagement [s]", fontsize=11)
    fig.suptitle("Same-command plant test: recorded motor commands driven through the "
                 "simulator's own plant model, vs the real vehicle's measured response",
                 fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.965])
    fig.savefig(f"{args.out_prefix}_plant.png", dpi=110)
    plt.close(fig)
    print(f"wrote {args.out_prefix}_plant.png")


if __name__ == "__main__":
    main()

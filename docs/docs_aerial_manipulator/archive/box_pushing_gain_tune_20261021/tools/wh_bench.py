#!/usr/bin/env python3
"""World-held hover bench for the box-pushing gain tune (2026-10-21).

The 10-21 hardware push showed a 0.54 Hz q2/q3 swing (10.5 deg rms) from the moment the claw was
held in the ROOM (the planner's world anchor), before any contact. This bench flies that state
offline: the exact Python law (circle_tune_20260927/tools/circle_bench.py -> wb_entry_sim.Law:
controller.py + l1_observer.py, the C++ port's source of truth) on the mirror plant with
hardware-like feedback, holding the 10-21 flight's OWN reference (the WholeBodyReference streamed
while the claw sat on the fin, jaws open) for t_settle + the run.

circle_bench is used unmodified; three things are patched from outside:
  * the EE reference: circle_bench builds its case with ee_ref="relative" (the claw follows the
    vehicle); EE_REF[0] = "world" is the room hold, "relative" the CoM anchor
  * the L1 observer: the box-pushing yaml's per-block feed-forward filter wb_l1_omega_x_t/r/q
  * the feedback: raw mocap (the box push flies it) instead of EKF2-fused, noise measured on 10-21

Scoring: the swing (q2 - q3)/2, the arm-yaw q1, the wrist roll q4, the CoM error and the tilt,
over the last WIN s of the hold (the room-held hover settled; no reference motion).
"""
import copy
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.abspath(os.path.join(HERE, "..", ".."))
REPO = os.path.abspath(os.path.join(ARCH, "..", "..", ".."))
# the bench's own sys.path entries predate the archive move: put the right ones first
for p in (os.path.join(ARCH, "circle_tune_20260927", "tools"),
          os.path.join(ARCH, "sim2real_tuning_20260926", "tools"),
          os.path.join(REPO, "application", "robotic_arm", "utils"),
          os.path.join(REPO, "extensions", "fsc_aerial_manipulation")):
    sys.path.insert(0, p)
import circle_bench as CB      # noqa: E402
import circle_sweep as CS      # noqa: E402
import wb_entry_sim as ES      # noqa: E402

NPZ = os.path.join(ARCH, "wb_box_pushing_flight_20261021", "npz", "b170123.npz")

# ---- the box-pushing yaml's law + observer (hardware and sim box yamls agree) ----
PUSH = dict(k_x=50.03, k_v=12.58, k_R=1.6, k_w=1.2, mrd_s=0.139992 / 0.130710,
            ky=120.0, dy=20.2, ky_psi=0.2484, dy_psi=0.2903,
            omega_c_t=1.0, omega_c_r=0.7428, omega_c_q=0.8479, omega_x=2.0, omega_x_blk=0.1,
            omega_q=0.2, dls=0.3)

# ---- raw-mocap feedback (the box push flies raw mocap; measured on the 10-21 bags) ----
# odom velocity noise vs a smoothed derivative of the position, hover: x 3.3-5.2, y 4.2-6.1,
# z 2.1-3.1 cm/s (Isaac's emulator injects 2.2). One isotropic AR(1) level here.
FB_RAW = dict(CB.FB, odo_lag_s=0.012, vel_noise=0.045, vel_bw_hz=5.0, pos_noise=0.0005)

EE_REF = ["world"]
_Case = CB.ES.Case


EE_FREE_W = [0.0]


class _PatchedCase(_Case):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.ee_ref = EE_REF[0]
        self.ee_free_w = EE_FREE_W[0]


CB.ES.Case = _PatchedCase


def _make_l1(p):
    b = p.get("omega_x_blk", 0.0)
    return CS.hw_l1(omega_c_t=p["omega_c_t"], omega_c_r=p["omega_c_r"], omega_c_q=p["omega_c_q"],
                    omega_x=p["omega_x"], omega_q=p["omega_q"],
                    omega_x_t=b, omega_x_r=b, omega_x_q=b)


CB.make_l1 = _make_l1


def stream(t_run=(37.0, 39.5)):
    """The 10-21 reference while the claw sat on the fin, jaws open (static), as a bench stream."""
    d = np.load(NPZ, allow_pickle=True)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv") and len(d[k]))
    S = {"t": d["wbref__recv"] - t0}
    for k in ("x_cd", "x_cd_dot", "x_cd_ddot", "x_cd_d3", "x_cd_d4", "b1_d", "b1_d_dot", "b1_d_ddot",
              "r_ed", "r_ed_dot", "r_ed_ddot", "b1_de", "b1_de_dot", "b1_de_ddot"):
        S[k] = np.column_stack([d[f"wbref__{k}.{a}"] for a in "xyz"])
    S["q_d"] = d["wbref__q_d"]
    S["qdot_d"] = d["wbref__qdot_d"]
    S["run"] = t_run
    S["name"] = "1021_world_held"
    return S


def lockin(t, x, f):
    A = np.column_stack([np.cos(2 * np.pi * f * t), np.sin(2 * np.pi * f * t), np.ones_like(t), t])
    c, *_ = np.linalg.lstsq(A, x, rcond=None)
    return float(np.hypot(c[0], c[1]))


def dominant(t, x):
    x = x - np.polyval(np.polyfit(t, x, 1), t)
    P = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    f = np.fft.rfftfreq(len(x), t[1] - t[0])
    m = (f > 0.08) & (f < 3.0)
    return float(f[m][np.argmax(P[m])])


def run(p, ee_ref="world", fb=None, seed=0, t_settle=20.0, win=12.0, delay_ms=16.0, S=None, free_w=None, **kw):
    """One bench case. Returns the swing metrics over the last `win` s of the held hover.
    free_w (or p["free_w"]) = the free-axis anchor weight along the nose (the push direction)."""
    EE_REF[0] = ee_ref
    EE_FREE_W[0] = float(p.get("free_w", 0.0) if free_w is None else free_w)
    S = S or stream()
    r = CB.simulate(p, S, profile="mirror", delay_ms=delay_ms, fb=FB_RAW if fb is None else fb, seed=seed,
                    t_settle=t_settle, keep=True, full=True, **kw)
    out = {"verdict": r["verdict"]}
    if r["verdict"] != "completed":
        out["t_end"] = r["t_end"]
        return out
    H, Hq, F = r["H"], r["Hq"], r["F"]
    t = H["t"]
    t1 = S["run"][0]
    m = (t >= t1 - win) & (t < t1)
    tt = t[m] - t[m][0]
    sw = np.degrees(Hq[m, 1] - Hq[m, 2]) / 2
    f = dominant(tt, sw)
    out.update(f=f, swing_rms=float(np.std(sw - np.polyval(np.polyfit(tt, sw, 1), tt))),
               swing_amp=lockin(tt, sw, f),
               q1_amp=lockin(tt, np.degrees(Hq[m, 0]), f), q4_amp=lockin(tt, np.degrees(Hq[m, 3]), f),
               q2_rng=(float(np.degrees(Hq[m, 1]).min()), float(np.degrees(Hq[m, 1]).max())),
               q3_rng=(float(np.degrees(Hq[m, 2]).min()), float(np.degrees(Hq[m, 2]).max())),
               com_rms=float(np.sqrt(np.mean(H["ex"][m] ** 2))) * 1e3,
               ee_rms=float(np.sqrt(np.mean(H["ee"][m] ** 2))) * 1e3,
               tilt_pk=float(H["tilt"][m].max()), eR=float(np.mean(H["eR"][m])) * 1e3,
               tau_pk=float(H["tau"][m].max()), dw=float(np.sqrt(np.mean(H["dw"][m] ** 2))))
    return out


def fmt(name, o):
    if o["verdict"] != "completed":
        return f"{name:34s} {o['verdict']} at {o.get('t_end', 0):.1f} s"
    return (f"{name:34s} swing {o['swing_rms']:5.2f} deg rms (amp {o['swing_amp']:5.2f} @ {o['f']:.2f} Hz) "
            f"q1 {o['q1_amp']:4.2f} q4 {o['q4_amp']:4.2f} | q2 {o['q2_rng'][0]:5.1f}..{o['q2_rng'][1]:5.1f} "
            f"q3 {o['q3_rng'][0]:5.1f}..{o['q3_rng'][1]:5.1f} | CoM {o['com_rms']:5.1f} mm EE {o['ee_rms']:5.1f} mm "
            f"tilt {o['tilt_pk']:4.1f} |e_R| {o['eR']:5.1f} mrad tau {o['tau_pk']:.2f}")


if __name__ == "__main__":
    import time
    t = time.time()
    print(fmt("push yaml, CoM anchor (hover)", run(PUSH, "relative")))
    print(fmt("push yaml, room-held", run(PUSH, "world")))
    print(fmt("push yaml, room-held, Isaac noise", run(PUSH, "world", fb=dict(FB_RAW, vel_noise=0.022))))
    print(f"({time.time() - t:.0f} s)")

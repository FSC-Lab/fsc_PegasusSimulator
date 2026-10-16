"""Per-leg summary of the 2026-10-15 pick-and-drop flights: duration, attitude vs its feed-forward, the 0.3-2 Hz
pitch oscillation, CoM / claw errors, arm joint error and torque, and the observer's estimates.
usage: legs.py [run ...]   (default e1 d1 d2)"""
import sys
import numpy as np
from pd15_common import *   # noqa: F401,F403

G = 9.80665


def pitch_ref(d, t0):
    """Feed-forward pitch of the planned CoM acceleration in the nose frame (FLU, nose-down +)."""
    t, ref = wbref(d, t0)
    a = ref["x_cd_ddot"]
    yaw = np.arctan2(ref["b1_d"][:, 1], ref["b1_d"][:, 0]) + 0.5 * np.pi      # model heading -> actual nose
    fwd = a[:, 0] * np.cos(yaw) + a[:, 1] * np.sin(yaw)
    lat = -a[:, 0] * np.sin(yaw) + a[:, 1] * np.cos(yaw)
    return t, np.arctan2(fwd, G + a[:, 2]), np.arctan2(-lat, G + a[:, 2])


def summary(run):
    d, t0 = load(run)
    tw, D = wbdebug(d, t0)
    ta, rr, pp, yy = px4_rpy(d, t0)
    tr, pref, rref = pitch_ref(d, t0)
    win = leg_windows(d, t0)
    tq, qm, _ = joints_model(d, t0)
    print(f"\n===== {run} {RUNS[run][0]} {RUNS[run][3]}")
    print(f"{'leg':40s} {'T':>5s} {'pitch':>6s} {'p_ff':>6s} {'p-ff':>6s} {'osc':>5s} {'osc':>5s} {'roll':>5s}"
          f" {'CoM':>5s} {'CoM':>5s} {'EEtask':>6s} {'|q-qd| j1..j4 max (deg)':>26s} {'|tau| max j1..j4':>22s}"
          f" {'d_t,z':>6s} {'d_r,x':>6s} {'d_r,y':>6s}")
    print(f"{'':40s} {'s':>5s} {'mean':>6s} {'mean':>6s} {'rms':>6s} {'rms':>5s} {'p-p':>5s} {'osc':>5s}"
          f" {'rms':>5s} {'max':>5s} {'rms mm':>6s}")
    for lab, (a, b) in win.items():
        if b - a < 1.0:
            continue
        m = (ta >= a) & (ta < b)
        pr = np.interp(ta[m], tr, pref)
        dp = np.degrees(pp[m] - pr)
        tu, osc = band(np.degrees(pp[m]), ta[m], 0.3, 2.0)
        _, osr = band(np.degrees(rr[m]), ta[m], 0.3, 2.0)
        mw = (tw >= a) & (tw < b)
        if mw.sum() < 10:
            continue
        com = np.linalg.norm(D[mw, 45:48] - D[mw, 48:51], axis=1) * 1e3
        eet = np.linalg.norm(D[mw, 24:27], axis=1) * 1e3
        qe = np.degrees(np.abs(D[mw, 5:9] - D[mw, 9:13])).max(axis=0)
        tau = np.abs(D[mw, 13:17]).max(axis=0)
        dh = D[mw, 31:41].mean(axis=0)
        print(f"{lab[:40]:40s} {b - a:5.1f} {np.degrees(pp[m]).mean():6.2f} {np.degrees(pr).mean():6.2f} {rms(dp - dp.mean()):6.2f}"
              f" {rms(osc):5.2f} {osc.max() - osc.min():5.2f} {rms(osr):5.2f} {rms(com):5.1f} {com.max():5.1f} {rms(eet):6.1f}"
              f"  {' '.join(f'{x:5.1f}' for x in qe)}  {' '.join(f'{x:4.2f}' for x in tau)}"
              f" {dh[2]:6.2f} {dh[3]:6.3f} {dh[4]:6.3f}")


if __name__ == "__main__":
    for r in (sys.argv[1:] or ["e1", "d1", "d2"]):
        summary(r)

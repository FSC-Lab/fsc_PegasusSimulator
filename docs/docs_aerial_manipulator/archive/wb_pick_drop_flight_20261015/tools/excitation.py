"""What drives the loaded swing: the PLANNED claw acceleration split along / across the claw (the basket hangs
from the claw and can only swing freely ACROSS it, about the wrist roll q4), the planned arm-yaw q1 and base yaw
motion, and the measured response (q4, basket across the claw, vehicle roll/pitch) per carry leg.
usage: excitation.py [run ...]  (default e1 d1)"""
import sys
import numpy as np
from pd15_common import *   # noqa: F401,F403
from pp_common import quat_R


def claw_axes(d, t0, t):
    te = d["pl_current_ee__recv"] - t0
    Qe = np.column_stack([d[f"pl_current_ee__pose.orientation.{k}"] for k in "wxyz"])
    j = np.clip(np.searchsorted(te, t), 0, len(te) - 1)
    R = np.array([quat_R(*Qe[k]) for k in j])
    return R[:, :, 0], R[:, :, 1]          # claw x (pointing), claw y (lateral) in world


def main(run):
    d, t0 = load(run)
    tr, ref = wbref(d, t0)
    win = leg_windows(d, t0)
    tw, D = wbdebug(d, t0)
    ta, rr, pp, yy = px4_rpy(d, t0)
    cx, cy = claw_axes(d, t0, tr)
    acc = ref["r_ed_ddot"] if "r_ed_ddot" in ref else None
    if acc is None:
        acc = np.column_stack([d[f"wbref__r_ed_ddot.{k}"] for k in "xyz"])
    lat = np.einsum("ij,ij->i", acc, cy); lon = np.einsum("ij,ij->i", acc, cx)
    psi = np.unwrap(np.arctan2(ref["b1_d"][:, 1], ref["b1_d"][:, 0]))
    q1d = ref["q_d"][:, 0]
    print(f"\n===== {run} {RUNS[run][0]} {RUNS[run][3]}")
    print(f"{'leg':36s} {'T':>5s} {'acc_lat':>8s} {'acc_lat':>8s} {'acc_lon':>8s} {'dpsi':>6s} {'psi_dot':>7s} {'dq1':>5s} {'q4':>6s} {'q4':>6s} {'roll':>5s} {'pitch':>5s}")
    print(f"{'':36s} {'s':>5s} {'rms':>8s} {'peak':>8s} {'peak':>8s} {'deg':>6s} {'pk d/s':>7s} {'deg':>5s} {'std':>6s} {'p-p':>6s} {'osc':>5s} {'osc':>5s}")
    for lab, (a, b) in win.items():
        if b - a < 2.0:
            continue
        m = (tr >= a) & (tr < b)
        mw = (tw >= a) & (tw < b); ma = (ta >= a) & (ta < b)
        q4 = np.degrees(D[mw, 8])
        _, osr = band(np.degrees(rr[ma]), ta[ma], 0.3, 2.0)
        _, osp = band(np.degrees(pp[ma]), ta[ma], 0.3, 2.0)
        dpsi = np.degrees(psi[m][-1] - psi[m][0])
        psid = np.degrees(np.abs(np.gradient(psi[m], tr[m]))).max()
        print(f"{lab[:36]:36s} {b - a:5.1f} {rms(lat[m]):8.3f} {np.abs(lat[m]).max():8.3f} {np.abs(lon[m]).max():8.3f}"
              f" {dpsi:6.1f} {psid:7.1f} {np.degrees(q1d[m][-1] - q1d[m][0]):5.1f} {q4.std():6.1f} {q4.max() - q4.min():6.1f}"
              f" {rms(osr):5.2f} {rms(osp):5.2f}")


if __name__ == "__main__":
    for r in (sys.argv[1:] or ["e1", "d1"]):
        main(r)

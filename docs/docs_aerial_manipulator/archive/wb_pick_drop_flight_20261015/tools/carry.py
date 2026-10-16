"""The loaded carry of D-1 (and the same legs of E-1, no payload): dominant frequency of the attitude oscillation,
the arm joints vs their reference, and where the basket hangs relative to the claw.
usage: carry.py [run ...]  (default e1 d1)"""
import sys
import numpy as np
from pd15_common import *   # noqa: F401,F403


def dom_freq(t, x, lo=0.2, hi=3.0):
    tu, xb = band(x, t, lo, hi)
    X = np.abs(np.fft.rfft(xb * np.hanning(len(xb))))
    f = np.fft.rfftfreq(len(xb), 0.01)
    k = np.argmax(X * ((f >= lo) & (f <= hi)))
    return f[k]


def main(run):
    d, t0 = load(run)
    tw, D = wbdebug(d, t0)
    ta, rr, pp, yy = px4_rpy(d, t0)
    win = leg_windows(d, t0)
    to, Po, Qo = obj_pose(d, t0)
    te = d["pl_current_ee__recv"] - t0
    Pe = np.column_stack([d[f"pl_current_ee__pose.position.{k}"] for k in "xyz"])
    Qe = np.column_stack([d[f"pl_current_ee__pose.orientation.{k}"] for k in "wxyz"])
    print(f"\n===== {run} {RUNS[run][0]} {RUNS[run][3]}")
    for lab in ("exit_pick (up the vertical margin)", "go_to_place_start", "execute_place (approach)"):
        if lab not in win:
            continue
        a, b = win[lab]
        m = (ta >= a) & (ta < b)
        fp = dom_freq(ta[m], np.degrees(pp[m])); fr = dom_freq(ta[m], np.degrees(rr[m]))
        mw = (tw >= a) & (tw < b)
        q = np.degrees(D[mw, 5:9]); qd = np.degrees(D[mw, 9:13])
        f4 = dom_freq(tw[mw], q[:, 3])
        print(f"{lab}: {a:.1f}-{b:.1f} s   pitch f {fp:.2f} Hz   roll f {fr:.2f} Hz   q4 f {f4:.2f} Hz")
        print("   q_d  start/end (deg):", np.round(qd[0], 1), np.round(qd[-1], 1))
        print("   q-qd mean (deg):     ", np.round((q - qd).mean(axis=0), 1), " std:", np.round((q - qd).std(axis=0), 1),
              " q4 min/max", np.round(q[:, 3].min(), 1), np.round(q[:, 3].max(), 1))
        # basket relative to the claw, in the claw's own frame (x = claw pointing, y left, z up of the gripper)
        if run != "e1":
            mo = (to >= a) & (to < b)
            rel = []
            for k in np.where(mo)[0]:
                j = min(np.searchsorted(te, to[k]), len(te) - 1)
                R = quat_R(*Qe[j])
                rel.append(R.T @ (Po[k] - Pe[j]))
            rel = np.array(rel) * 1e3
            print("   basket - claw in the claw frame, mm  mean:", np.round(rel.mean(axis=0), 1), " std:",
                  np.round(rel.std(axis=0), 1), " p-p:", np.round(rel.max(axis=0) - rel.min(axis=0), 1))
            fy = dom_freq(to[mo], rel[:, 1] / 1e3)
            print(f"   basket lateral swing f {fy:.2f} Hz;  basket tilt mean {tilt_of(Qo[mo]).mean():.1f} deg")


if __name__ == "__main__":
    from pp_common import quat_R   # noqa
    for r in (sys.argv[1:] or ["e1", "d1"]):
        main(r)

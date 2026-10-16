"""The arm channel per leg: the law's streamed torque, the arm controller's added terms (friction + dither, the
pass-through gravity correction), the command against the servo caps, the joint error, and the observer's
joint-row estimates.  Arm controller law_debug is in HARDWARE joint order j1..j4 and COMMAND units (duty
counts); /KPWM gives N*m.  usage: arm_terms.py [run ...]  (default e1 d1 d2)"""
import sys
import numpy as np
from pd15_common import *   # noqa: F401,F403
from common import KPWM

MAXEFF = np.array([57.6117, 364.8743, 192.0391, 57.6117])      # max_effort, duty counts (hardware yaml)
CAP = MAXEFF / KPWM                                             # N*m


def main(run):
    d, t0 = load(run)
    win = leg_windows(d, t0)
    tl = d["law__recv"] - t0; L = d["law__data"]
    tw, D = wbdebug(d, t0)
    print(f"\n===== {run} {RUNS[run][0]} {RUNS[run][3]}    servo caps N*m {np.round(CAP, 2)}")
    print(f"{'leg':30s} {'law tau rms j1..j4 (N*m)':>27s} {'|law| max':>22s} {'aux rms (N*m)':>22s}"
          f" {'at cap %':>16s} {'q-qd rms (deg)':>22s} {'d_rho mean (N*m)':>25s}")
    for lab, (a, b) in win.items():
        if b - a < 2.0:
            continue
        m = (tl >= a) & (tl < b)
        if m.sum() < 10:
            continue
        ext = L[m, 9:13]                         # N*m, hardware order
        aux = L[m, 18:22] / KPWM
        cmd = L[m, 13:17]
        atcap = (np.abs(cmd) >= MAXEFF * 0.999).mean(axis=0) * 100
        mw = (tw >= a) & (tw < b)
        qe = np.degrees(D[mw, 5:9] - D[mw, 9:13])
        drho = D[mw, 37:41].mean(axis=0)
        f = lambda v, p=2: " ".join(f"{x:5.{p}f}" for x in v)
        print(f"{lab[:30]:30s} {f(rms(ext)):>27s} {f(np.abs(ext).max(axis=0)):>22s} {f(rms(aux)):>22s}"
              f" {f(atcap, 0):>16s} {f(rms(qe), 1):>22s} {f(drho, 3):>25s}")


if __name__ == "__main__":
    for r in (sys.argv[1:] or ["e1", "d1", "d2"]):
        main(r)

#!/usr/bin/env python3
"""atc_score.py -- one-screen score of an arbitrary-time box-pushing flight
(pl_mission.py npz recorded with --dbg-topic atc_control_debug --dbg-len 105).

Per 5 s of DIRECT: the base's position/attitude errors, the yaw torque's clamp
duty, the joint-1 velocity (the 3 Hz j1/yaw mode of atc_01), the pitch torque
(the 1.4 Hz j2/j3/pitch mode), the ACTNDO's estimate, the arm command, the
TSKFE's rule count; then the dominant frequency of the key channels over the
whole DIRECT span, the mission outcome and the box's travel.

    PYTHONNOUSERSITE=1 /usr/bin/python3 atc_score.py runs/<tag>.npz [--win 5]
"""
import argparse
import os

import numpy as np

C = dict(r1p=0, r2p=3, r1a=6, etad=9, thr=12, tau=13, Dh=16, mot=22, eta=26, eta_d=29, Fw=32, Mb=35, th=38,
         tm=42, q=46, q_d=50, qc=54, th1=58, pd=62, vd=65, yawd=68, ad=69, tgo=72, rules=76, x1=77, x2=78,
         contact=79, unal=80, fresh=84, res=85, s1=86, fdes=90, ngen=93, Fm=94, qdot=100, vobs=104)


def spec(x, dt):
    x = x - x.mean()
    if len(x) < 64 or x.std() < 1e-12:
        return float("nan"), 0.0
    P = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    f = np.fft.rfftfreq(len(x), dt)
    i = np.argmax(P[2:]) + 2
    return f[i], P[i] / P.sum()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--win", type=float, default=5.0)
    a = ap.parse_args()
    z = np.load(a.npz, allow_pickle=True)
    d = z["dbg"]
    if d.size <= 1:
        raise SystemExit("no debug array recorded")
    t = d[:, 0] - d[0, 0]
    D = d[:, 2:]
    marks = dict(zip(z["marks_name"], z["marks_t"]))
    t0 = float(marks.get("direct", d[0, 0]))
    rel = {k: round(float(v) - t0, 1) for k, v in marks.items()}
    print(f"== {os.path.basename(a.npz)}: aborted={bool(z['aborted'])} reason='{z['reason']}'")
    print("   marks (s after DIRECT):", rel)
    box = z["box"]
    if box.size > 1:
        print(f"   box: start {np.round(box[0, 1:3], 3)} end {np.round(box[-1, 1:3], 3)} -> travelled "
              f"{np.linalg.norm(box[-1, 1:3] - box[0, 1:3]):.3f} m")
    print(f"   DIRECT span {t[-1]:.1f} s; motors sat (any >= 0.999) {100 * np.mean(D[:, C['mot']:C['mot'] + 4].max(1) >= 0.999):.1f} %; "
          f"arm clamp (|tau_m| >= 2.99) {100 * np.mean(np.abs(D[:, C['tm']:C['tm'] + 4]).max(1) >= 2.99):.2f} %")
    print("\n   t[s]     |e_p| mm  |e_eta| deg  tilt_err x/y deg  tau_z std  clamp%   qd1 std  tau_m1 std  tau_hat std(j1..4)      "
          "tau_y std  qd2/qd3 std   D_hat z   |F_ext|  rules  contact")
    w = a.win
    for s in np.arange(0.0, t[-1], w):
        m = (t >= s) & (t < s + w)
        if m.sum() < 20:
            continue
        ep = np.linalg.norm(D[m, C['r1p']:C['r1p'] + 3], axis=1)
        ea = np.linalg.norm(D[m, C['r1a']:C['r1a'] + 3], axis=1)
        tz = D[m, C['tau'] + 2]
        th = D[m, C['th']:C['th'] + 4].std(0)
        print(f"   {s:5.0f}-{s + w:3.0f}  {1e3 * ep.mean():7.1f}   {np.degrees(ea.mean()):7.2f}     "
              f"{np.degrees(D[m, C['r1a']].std()):4.2f}/{np.degrees(D[m, C['r1a'] + 1].std()):4.2f}        "
              f"{tz.std():5.3f}   {100 * np.mean(np.abs(tz) >= 0.549):5.1f}  {D[m, C['qdot']].std():6.3f}   {D[m, C['tm']].std():6.3f}     "
              f"[{th[0]:.3f} {th[1]:.3f} {th[2]:.3f} {th[3]:.3f}]  {D[m, C['tau'] + 1].std():6.3f}   "
              f"{D[m, C['qdot'] + 1].std():5.3f}/{D[m, C['qdot'] + 2].std():5.3f}   {D[m, C['Dh'] + 5].mean():+6.3f}  "
              f"{np.linalg.norm(D[m, C['Fw']:C['Fw'] + 3], axis=1).mean():5.2f}  {int(D[m, C['rules']].max()):4d}   {int(D[m, C['contact']].max())}")
    dt = np.median(np.diff(t))
    m = t > 2.0
    print("\n   dominant frequency over DIRECT (after 2 s):")
    for name, c in (("qd1", C['qdot']), ("tau_m1", C['tm']), ("tau_hat1", C['th']), ("yaw torque", C['tau'] + 2),
                    ("eta_dot_z", C['etad'] + 2), ("pitch torque", C['tau'] + 1), ("eta_dot_y", C['etad'] + 1),
                    ("qd2", C['qdot'] + 1), ("qd3", C['qdot'] + 2), ("roll torque", C['tau']), ("D_hat_z", C['Dh'] + 5)):
        f, frac = spec(D[m, c], dt)
        print(f"     {name:12s} {f:5.2f} Hz ({100 * frac:4.1f} % of the power)")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""The wrist roll q4 in the Isaac pick-and-drop flights (pnp_mission_v2.py recordings), before / after the j4 fix.
Recorder dbg = [t, step, wb_control_debug...] (debug index i at column i + 2): q [5..8], q_ref [9..12],
tau [13..16], u3 observer share raw [89..92], the law's q_dot [107..110], stuck-hold bits [115] (new builds only).
Per flight, over DIRECT, the CARRY (exit_pick start -> gripper_open start) and the EMPTY wrist after the release
(gripper_open end -> execute_land end): |q4 - q4_d| rms / max, slips (|q4_dot| crossing 3 deg/s upward), the
observer's j4 share max, the law's j4 torque rms, and the fraction of ticks the j4 rows were held.
    PYTHONNOUSERSITE=1 /usr/bin/python3 wrist_score.py <npz> ...     (default: D1..D6 baseline + J* here)"""
import glob
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(HERE, "..", "..", "lift_lurch_gains_20261016", "runs")


def mark(d, name):
    m = [t for t, n in zip(d["marks_t"], d["marks_name"]) if n == name]
    return m[0] if m else np.nan


def win(D, a, b):
    t = D[:, 0]
    return D[(t >= a) & (t < b) & (D[:, 2] > 0.5)]


def stats(W):
    if len(W) < 10:
        return None
    e = np.degrees(W[:, 2 + 8] - W[:, 2 + 12])
    v = np.degrees(W[:, 2 + 110])
    up = np.flatnonzero((np.abs(v[1:]) > 3.0) & (np.abs(v[:-1]) <= 3.0))
    held = (W[:, 2 + 115] if W.shape[1] > 2 + 115 else np.zeros(len(W)))
    held = np.nan_to_num(held)
    return dict(rms=np.sqrt(np.mean(e ** 2)), mx=np.abs(e).max(), slips=len(up),
                est=np.abs(W[:, 2 + 92]).max() * 1e3, tau=np.sqrt(np.mean(W[:, 2 + 16] ** 2)) * 1e3,
                held=np.mean((held.astype(int) & 8) > 0) * 100)


def score(path):
    d = np.load(path, allow_pickle=True)
    D = d["dbg"]
    t0, t1 = D[D[:, 2] > 0.5, 0][[0, -1]]
    rows = {"DIRECT": (t0, t1),
            "carry": (mark(d, "exit_pick:start"), mark(d, "gripper_open:start")),
            "empty after release": (mark(d, "gripper_open:end"), mark(d, "execute_land:end"))}
    out = {}
    for k, (a, b) in rows.items():
        out[k] = stats(win(D, a, b))
    done = not bool(d["aborted"])
    return os.path.basename(path).replace(".npz", ""), done, out


if __name__ == "__main__":
    paths = sys.argv[1:] or (sorted(glob.glob(os.path.join(BASE, "D[1-6].npz")))
                             + sorted(glob.glob(os.path.join(HERE, "..", "runs", "J*.npz"))))
    print(f"{'run':5s} {'done':4s} | {'phase':20s} | {'|q4-q4d| rms':>12s} {'max':>6s} {'slips':>5s} | "
          f"{'obs j4 share max':>16s} {'tau j4 rms':>10s} | {'j4 held':>7s}")
    for p in paths:
        name, done, out = score(p)
        for k, s in out.items():
            if s is None:
                continue
            print(f"{name:5s} {'yes' if done else 'NO':4s} | {k:20s} | {s['rms']:10.2f} deg {s['mx']:6.1f} {s['slips']:5d} | "
                  f"{s['est']:12.1f} mN.m {s['tau']:8.1f} mN.m | {s['held']:5.0f} %")


def split(path, fc=0.2):
    """q4 - q4_d split at fc: the SLOW part (where the wrist parks / creeps -- what the fix targets) vs the swing."""
    from scipy.signal import butter, filtfilt
    d = np.load(path, allow_pickle=True); D = d["dbg"]
    out = {}
    for k, (a, b) in {"carry": (mark(d, "exit_pick:start"), mark(d, "gripper_open:start")),
                      "empty after release": (mark(d, "gripper_open:end"), mark(d, "execute_land:end"))}.items():
        W = win(D, a, b)
        e = np.degrees(W[:, 2 + 8] - W[:, 2 + 12]); t = W[:, 0]
        tu = np.arange(t[0], t[-1], 0.01); eu = np.interp(tu, t, e)
        bb, aa = butter(2, fc / 50.0)
        slow = filtfilt(bb, aa, eu); fast = eu - slow
        out[k] = (np.abs(slow).max(), np.ptp(slow), np.sqrt(np.mean(fast ** 2)))
    return out


if __name__ == "__main__" and os.environ.get("SPLIT", "1") == "1":
    paths = sys.argv[1:] or (sorted(glob.glob(os.path.join(BASE, "D[1-6].npz")))
                             + sorted(glob.glob(os.path.join(HERE, "..", "runs", "J*.npz"))))
    print(f"\n{'run':5s} | {'phase':20s} | {'slow (<0.2 Hz) |e| max':>22s} {'p-p':>6s} | {'swing (>0.2 Hz) rms':>19s}")
    for p in paths:
        for k, (mx, pp, sw) in split(p).items():
            print(f"{os.path.basename(p)[:-4]:5s} | {k:20s} | {mx:18.2f} deg {pp:6.2f} | {sw:15.2f} deg")

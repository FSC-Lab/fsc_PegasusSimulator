#!/usr/bin/env python3
"""Score the pre-grasp room-hold guard runs (runs/rh_*.npz, pl_mission.py's layout: dbg = [t, step, debug...],
joints = [t, q1..q4, eff1..4, step], base_truth = [t, x, y, z, qw, qx, qy, qz, step], box = [t, x, y, z, qw, qx,
qy, qz, step]).
Per run: the outcome; the room hold -> Ready wait (the base gate and the leash at work) and the leash releases; the
law's horizontal CoM error in the hold / on the descent / at Ready; the roll attitude error over the last 3 s of
Go To Start and the observer's roll estimate change in the hold (the 10-31 evening's two signatures: Step 2's A/B);
q3 minima before the grasp and in the turn; the valve's rotation; the peak true tilt.
    PYTHONNOUSERSITE=1 /usr/bin/python3 score_runs.py [tag ...]"""
import glob
import os
import re
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs")


def yaw(qw, qx, qy, qz):
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def tilt(qw, qx, qy, qz):
    return np.degrees(np.arccos(np.clip(1 - 2 * (qx * qx + qy * qy), -1, 1)))


def score(path):
    z = np.load(path, allow_pickle=True)
    tag = os.path.basename(path)[:-4]
    ev = [str(e) for e in z["events"]]
    names, ts = list(z["marks_name"]), list(z["marks_t"])
    mt = lambda n: ts[names.index(n)] if n in names else None   # noqa: E731
    t_hold, t_ready, t_ready_end = mt("go_to_start:end"), mt("ready:start"), mt("ready:end")
    t_push, t_push_end = mt("push:start"), mt("push:end")
    done = any("mission complete" in e for e in ev)
    ab = [e for e in ev if "push_pull: ABORTING" in e]
    reason = re.sub(r"^\[\s*[0-9.]+s\] push_pull: ABORTING ", "", ab[0])[:120] if ab else ""
    t_ab = float(re.match(r"^\[\s*([0-9.]+)s\]", ab[0]).group(1)) if ab else None
    left = any("NOT IN DIRECT" in e for e in ev if mt("direct") and float(re.match(r"^\[\s*([0-9.]+)s\]", e).group(1)) > mt("direct"))
    rel = [e for e in ev if "ROOM HOLD RELEASED" in e]
    n_rel = 0
    prev = False
    for e in ev:
        if "push_pull:" not in e:
            continue
        cur = "ROOM HOLD RELEASED" in e
        n_rel += cur and not prev
        prev = cur
    D = z["dbg"]; tw = D[:, 0]; W = D[:, 2:]
    full = W.shape[1] > 51
    e2 = W[:, 48:50] - W[:, 45:47] if full else np.zeros((len(tw), 2))
    ecom = 1e3 * np.linalg.norm(e2, axis=1)
    ok = np.isfinite(ecom)

    def mx(a, b, x):
        m = ok & (tw >= a) & (tw <= b) if a is not None and b is not None else np.zeros(len(tw), bool)
        return float(np.nanmax(x[m])) if m.any() else np.nan

    def at(t, x):
        if t is None:
            return np.nan
        i = min(np.searchsorted(tw, t), len(tw) - 1)
        return float(x[i])
    end_r = t_ready_end if t_ready_end is not None else (t_ab if t_ab is not None else (t_ready + 3 if t_ready else None))
    roll = np.degrees(W[:, 29]) if full else np.zeros(len(tw))   # e_R model y = roll
    m3 = (tw >= (t_hold or 0) - 3.0) & (tw <= (t_hold or 0)) if t_hold else np.zeros(len(tw), bool)
    roll_std = float(np.std(roll[m3]) * 1e3 / np.degrees(1)) if m3.any() else np.nan   # mrad
    dr = W[:, 35] if full else np.zeros(len(tw))                  # d_hat_r model y = roll
    d_roll = (at(t_ready, dr) - at(t_hold, dr)) if t_hold and t_ready else np.nan
    J = z["joints"]; tj = J[:, 0]; q3 = np.degrees(J[:, 3])

    def q3min(a, b):
        m = (tj >= a) & (tj <= b) if a is not None and b is not None else np.zeros(len(tj), bool)
        return float(q3[m].min()) if m.any() else np.nan
    B = z["box"]
    rot = np.nan
    if t_push is not None and len(B):
        yb = np.unwrap(yaw(B[:, 4], B[:, 5], B[:, 6], B[:, 7]))
        e_ = t_push_end if t_push_end is not None else (t_ab or B[-1, 0])
        rot = float(np.degrees(np.interp(e_, B[:, 0], yb) - np.interp(t_push, B[:, 0], yb)))
    BT = z["base_truth"]
    t0d = mt("direct") or 0.0
    mb = BT[:, 0] >= t0d
    tl = float(tilt(*BT[mb, 4:8].T).max()) if mb.any() else np.nan
    return dict(tag=tag, done=done, reason=reason, left=left, wait=(t_ready - t_hold) if t_hold and t_ready else np.nan,
                n_rel=n_rel, e_hold=mx(t_hold, t_ready, ecom), e_ready=mx(t_ready, end_r, ecom), e_at=at(t_ready, ecom),
                roll_std=roll_std, d_roll=d_roll, q3_pre=q3min(t_hold, end_r), q3_turn=q3min(t_push, t_push_end or t_ab),
                rot=rot, tilt=tl)


tags = sys.argv[1:]
paths = [t if t.endswith(".npz") else os.path.join(RUNS, t + ".npz") for t in tags] if tags \
    else sorted(glob.glob(os.path.join(RUNS, "rh_*.npz")))
f = lambda x, p=0: "  -  " if x is None or (isinstance(x, float) and not np.isfinite(x)) else f"{x:.{p}f}"   # noqa: E731
print(f"{'run':12s} {'outcome':9s} {'wait s':>6s} {'rel':>3s} | {'e hold':>6s} {'e Rdy':>6s} {'e@Rdy':>6s} mm | "
      f"{'roll3s':>6s} mrad {'d_roll':>7s} N.m | {'q3 pre':>6s} {'q3 turn':>7s} | {'turn':>6s} {'tilt':>5s} | reason")
for p in paths:
    if not os.path.exists(p):
        print(f"{os.path.basename(p)[:-4]:12s} (no npz)")
        continue
    s = score(p)
    oc = "complete" if s["done"] else ("ABORT" if s["reason"] else "?")
    if s["left"]:
        oc += "/SAFE"
    print(f"{s['tag']:12s} {oc:9s} {f(s['wait'], 2):>6s} {s['n_rel']:3d} | {f(s['e_hold']):>6s} {f(s['e_ready']):>6s} "
          f"{f(s['e_at']):>6s}    | {f(s['roll_std']):>6s}      {f(s['d_roll'], 3):>7s}     | {f(s['q3_pre'], 1):>6s} "
          f"{f(s['q3_turn'], 1):>7s} | {f(s['rot'], 1):>6s} {f(s['tilt'], 1):>5s} | {s['reason']}")

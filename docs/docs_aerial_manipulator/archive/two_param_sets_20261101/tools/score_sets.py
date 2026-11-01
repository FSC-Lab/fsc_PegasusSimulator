#!/usr/bin/env python3
"""Score the two-parameter-set runs (runs/*.npz, pl_mission.py's layout: dbg = [t, step, wb_control_debug...],
joints = [t, q1..q4, eff1..4, step], base_truth = [t, x, y, z, qw, qx, qy, qz, step], box = [t, x, y, z, qw..qz, step],
events, marks_name / marks_t).
Per run: the outcome; WHEN the sets switched (debug [126] the set in force, [121] the blend) against the grasp /
release marks; the gains applied ([127] K_y x, [128] omega_c_t) in free flight, in contact and in the exit; the
anchor blend in force at the hand-backs ([130], [129] the CoM-anchor weight's fastest change); the law's
horizontal CoM error in the free-flight legs (Go To Start, the way out) vs the contact phase; q3 minima before the
grasp / in the contact phase / in the release window; the valve's or box's motion; the peak true tilt.
    PYTHONNOUSERSITE=1 /usr/bin/python3 score_sets.py [tag ...]      (default: every ts*.npz)"""
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
    t_gts0, t_hold = mt("go_to_start:start"), mt("go_to_start:end")
    t_ready, t_ready_end = mt("ready:start"), mt("ready:end")
    t_close = mt("gripper_close:end") or mt("close:start")
    t_push, t_push_end = mt("push:start"), mt("push:end")
    t_rel = mt("gripper_release:end") or mt("release:start")
    t_exit, t_exit_end = mt("exit:start"), mt("exit:end")
    done = any("mission complete" in e for e in ev)
    ab = [e for e in ev if "push_pull: ABORTING" in e]
    reason = re.sub(r"^\[\s*[0-9.]+s\] push_pull: ABORTING ", "", ab[0])[:100] if ab else ""
    t_ab = float(re.match(r"^\[\s*([0-9.]+)s\]", ab[0]).group(1)) if ab else None
    td = mt("direct") or 0.0
    left = any("NOT IN DIRECT" in e for e in ev if float(re.match(r"^\[\s*([0-9.]+)s\]", e).group(1)) > td + 1.0)
    D = z["dbg"]; tw = D[:, 0]; W = D[:, 2:]
    wide = W.shape[1] > 130
    e2 = W[:, 48:50] - W[:, 45:47]
    ecom = 1e3 * np.linalg.norm(e2, axis=1)

    def win(a, b):
        if a is None or b is None:
            return np.zeros(len(tw), bool)
        return (tw >= a) & (tw <= b) & np.isfinite(ecom)

    def rms(m):
        return float(np.sqrt(np.nanmean(ecom[m] ** 2))) if m.any() else np.nan

    def mx(m, x=None):
        x = ecom if x is None else x
        return float(np.nanmax(x[m])) if m.any() else np.nan
    # the sets
    if wide:
        inter = W[:, 126]; ky = W[:, 127]; wct = W[:, 128]; aw = W[:, 129]; ab_s = W[:, 130]
        edges = np.where(np.diff(inter) != 0)[0]
        sw = [(float(tw[k + 1]), int(inter[k + 1])) for k in edges]
        on = next((t for t, s in sw if s == 1), None)
        off = next((t for t, s in sw if s == 0 and on is not None and t > on), None)
        at = lambda t: int(min(np.searchsorted(tw, t), len(tw) - 1)) if t is not None else None   # noqa: E731
        gains = {}
        for nm, t in (("hold", t_hold), ("push", (t_push or 0) + 2.0 if t_push else None), ("exit", (t_exit or 0) + 0.5 if t_exit else None)):
            k = at(t)
            gains[nm] = (float(ky[k]), float(wct[k])) if k is not None else (np.nan, np.nan)
        # the fastest CoM-anchor hand-back seen (weight 0 -> 1): its 10-90 % rise time
        rise = np.nan
        k0 = np.where((np.diff(aw) > 0) & (aw[:-1] < 0.05))[0]
        for k in k0:
            j = k
            while j < len(aw) - 1 and aw[j] < 0.9:
                j += 1
            if aw[j] >= 0.9:
                rise = np.nanmin([rise, tw[j] - tw[k]]) if np.isfinite(rise) else tw[j] - tw[k]
        fast_s = float(np.nanmin(ab_s)) if np.isfinite(ab_s).any() else np.nan
    else:
        on = off = None; gains = {}; rise = np.nan; fast_s = np.nan
    # joints
    J = z["joints"]; tj = J[:, 0]; q3 = np.degrees(J[:, 3]); q2 = np.degrees(J[:, 2])

    def q3min(a, b):
        m = (tj >= a) & (tj <= b) if a is not None and b is not None else np.zeros(len(tj), bool)
        return (float(q3[m].min()), float(q2[m].max())) if m.any() else (np.nan, np.nan)
    end_pre = t_close or t_ready_end or t_ab
    rel_end = (t_exit_end or t_ab or tw[-1])
    q3_pre, q2_pre = q3min(t_hold, end_pre)
    q3_c, q2_c = q3min(t_push, t_push_end or t_ab)
    q3_r, q2_r = q3min(t_rel if t_rel else t_push_end, rel_end)
    # the handle's motion (valve: yaw; box: along y)
    B = z["box"]; motion = np.nan
    if t_push is not None and len(B):
        e_ = t_push_end if t_push_end is not None else (t_ab or B[-1, 0])
        yb = np.unwrap(yaw(B[:, 4], B[:, 5], B[:, 6], B[:, 7]))
        dyaw = np.degrees(np.interp(e_, B[:, 0], yb) - np.interp(t_push, B[:, 0], yb))
        dpos = 1e3 * (np.interp(e_, B[:, 0], B[:, 2]) - np.interp(t_push, B[:, 0], B[:, 2]))
        motion = dyaw if abs(dyaw) > 15 else dpos
    # the peak true tilt in FLIGHT: from DIRECT entry to the land step's end (after it the sim's touchdown /
    # disarm on the 0.30 m land hover rolls the vehicle on the ground, 13 s after NOT IN DIRECT -- not flight)
    BT = z["base_truth"]; t_land_end = mt("land:end") or (t_ab + 15.0 if t_ab else np.inf)
    mb = (BT[:, 0] >= td) & (BT[:, 0] <= t_land_end)
    tl = float(tilt(*BT[mb, 4:8].T).max()) if mb.any() else np.nan
    return dict(tag=tag, done=done, reason=reason, left=left, on=on, off=off, t_close=t_close, t_rel=t_rel, t_exit_end=t_exit_end,
                gains=gains, rise=rise, fast_s=fast_s,
                e_free=rms(win(t_gts0, t_hold)), e_free_max=mx(win(t_gts0, t_hold)), e_hold=mx(win(t_hold, t_ready)),
                e_contact=rms(win(t_push, t_push_end or t_ab)), e_out=rms(win(t_exit, t_exit_end)),
                q3_pre=q3_pre, q2_pre=q2_pre, q3_c=q3_c, q2_c=q2_c, q3_r=q3_r, q2_r=q2_r, motion=motion, tilt=tl)


tags = sys.argv[1:]
paths = [t if t.endswith(".npz") else os.path.join(RUNS, t + ".npz") for t in tags] if tags \
    else sorted(glob.glob(os.path.join(RUNS, "ts*.npz")))
f = lambda x, p=0: "  -  " if x is None or (isinstance(x, float) and not np.isfinite(x)) else f"{x:.{p}f}"   # noqa: E731
print(f"{'run':10s} {'outcome':9s} | {'set on':>7s} {'(close)':>7s} {'set off':>7s} {'(rel/exit)':>11s} | "
      f"{'Ky/wct hold':>12s} {'push':>10s} {'exit':>10s} | {'hand-back':>9s} | {'CoM rms free':>12s} {'hold max':>8s} {'contact':>7s} {'out':>5s} mm | "
      f"{'q3/q2 pre':>10s} {'contact':>10s} {'release':>10s} | {'motion':>6s} {'tilt':>5s} | reason")
for p in paths:
    if not os.path.exists(p):
        print(f"{os.path.basename(p)[:-4]:10s} (no npz)")
        continue
    s = score(p)
    oc = "complete" if s["done"] else ("ABORT" if s["reason"] else "?")
    if s["left"]:
        oc += "/SAFE"
    g = s["gains"]
    gs = lambda k: f"{g[k][0]:.0f}/{g[k][1]:.2f}" if k in g and np.isfinite(g[k][0]) else "  -  "   # noqa: E731
    print(f"{s['tag']:10s} {oc:9s} | {f(s['on'], 2):>7s} {f(s['t_close'], 2):>7s} {f(s['off'], 2):>7s} "
          f"{f(s['t_rel'], 2):>5s}/{f(s['t_exit_end'], 1):>5s} | {gs('hold'):>12s} {gs('push'):>10s} {gs('exit'):>10s} | "
          f"{f(s['rise'], 2):>5s}s{f(s['fast_s'], 2):>4s} | {f(s['e_free'], 1):>12s} {f(s['e_hold']):>8s} {f(s['e_contact'], 1):>7s} {f(s['e_out'], 1):>5s}    | "
          f"{f(s['q3_pre'], 1):>4s}/{f(s['q2_pre'], 0):>3s} {f(s['q3_c'], 1):>6s}/{f(s['q2_c'], 0):>3s} {f(s['q3_r'], 1):>6s}/{f(s['q2_r'], 0):>3s} | "
          f"{f(s['motion'], 0):>6s} {f(s['tilt'], 1):>5s} | {s['reason']}")

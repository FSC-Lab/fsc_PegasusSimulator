#!/usr/bin/env python3
"""(2026-10-31) The roll fin vs the fixed fin in Isaac, one row per run.

    PYTHONNOUSERSITE=1 /usr/bin/python3 grip_score.py [tag ...]     (no tags = this campaign's runs + the v2 control)

Runs: this campaign's runs/ (v3: the roll fin FREE or LOCKED, the tag's .roll file says which) and the v2
control, archive/valve_safety_fixes_20261030/sim/runs (vs_nom_1..4, vs_hw_s2..s4: the same harness on
the 2026-10-31 build, the fixed 5 mm fin).

Line 1: the 10-22 campaign scorer (outcome, beam turn, in-turn offset, arm, tilt, contact force).
Line 2 (the turn = push:start .. push:end, or the abort):
  couple at G   the PhysX contact wrench of the drone on the valve (both bodies), moved to the grip point G
                (the rotor's G, from obj_0's pose) and resolved on the rotor's fin axes: X radial (the jaws'
                closing axis), N the roll axis (the fin's plane, 30 deg up toward the turn), C (the fin's
                plane, square to N, down). rms of each, N.m. The roll fin can pass about N only its O-ring.
  sway          the CoM error x_c - x_cd (the law's own, [48..50] - [45..47]) in the BODY frame: sideways
                and along-nose rms [mm], and the sideways error's RATE p99 [mm/s] (the 10-30 sway guard's
                signal, its 0.15 m/s limit)
  e_R roll      the law's attitude error about the nose (model y, [29]) rms [mrad]; d_hat roll [35] rms [N.m]
  q1 / q4       std in the turn [deg], and their correlation (the 10-29 hardware sway: -0.99, antiphase)
  roll fin      (v3) its angle about N in the turn: min..max [deg], rms of its rate [deg/s]
"""
import glob
import importlib.util
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs") + "/"
V2 = os.path.join(HERE, "..", "..", "valve_safety_fixes_20261030", "sim", "runs") + "/"
V2_TAGS = ["vs_nom_1", "vs_nom_2", "vs_nom_3", "vs_nom_4", "vs_hw_s2", "vs_hw_s3", "vs_hw_s4"]
_spec = importlib.util.spec_from_file_location(
    "campaign_score", os.path.join(HERE, "..", "..", "valve_turning_20261015", "tools", "campaign_score.py"))
cs = importlib.util.module_from_spec(_spec)
_argv, sys.argv = sys.argv, sys.argv[:1]
_spec.loader.exec_module(cs)
sys.argv = _argv

G = np.array([0.160, 0.0006, 0.179])                    # the grip point, asset (rotor) frame
X = np.array([1.0, 0.0, 0.0])
N = np.array([0.0, np.cos(np.radians(30)), np.sin(np.radians(30))])
C = np.array([0.0, 0.5, -np.sqrt(3) / 2])
PIVOT = np.array([0.09349, 0.00136, 0.08989])           # obj_0 = the markers' centroid, rotor frame (v2 = v3)
D = 2                                                   # dbg column = debug index + 2


def q2R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def interp_rows(t, tt, A):
    return np.column_stack([np.interp(t, tt, A[:, k]) for k in range(A.shape[1])])


def window(d):
    marks = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    t0 = marks.get("push:start", np.nan)
    t1 = marks.get("push:end", np.nan)
    if not np.isfinite(t1):
        ab = [str(e) for e in d["events"] if "ABORT" in str(e) or "left DIRECT" in str(e)]
        t1 = float(ab[0].split("s]")[0].strip("[ ")) if ab else d["dbg"][-1, 0]
    return t0, t1


def mode_of(path, tag):
    f = path + tag + ".roll"
    if not os.path.exists(f):
        return "v2 fixed"
    kv = dict(l.strip().split("=", 1) for l in open(f) if "=" in l)
    if kv.get("ROLL_MODE") == "v2":                       # chain2.sh's same-day v2 control
        return "v2 fixed"
    base = "v3 LOCKED" if kv.get("ROLL_LOCK") == "1" else f"v3 free {float(kv.get('ROLL_FRICTION', 0.002)) * 1e3:.0f}mNm"
    return base + (f" +shim{float(kv['SHIM_UP_MM']):.0f}" if "SHIM_UP_MM" in kv else "")       # queue3.sh's diagnostic


def metrics(path, tag):
    d = np.load(path + tag + ".npz", allow_pickle=True)
    t0, t1 = window(d)
    out = {"mode": mode_of(path, tag)}
    if not np.isfinite(t0):
        return out
    c, box, dbg, J, od = d["contact"], d["box"], d["dbg"], d["joints"], d["odom"]
    # the couple at G, on the rotor's fin axes
    mc = (c[:, 0] >= t0) & (c[:, 0] <= t1)
    F = c[mc, 1:4] + c[mc, 4:7]
    MO = c[mc, 13:16] + c[mc, 16:19]
    B = interp_rows(c[mc, 0], box[:, 0], box[:, 1:8])
    M = np.zeros((mc.sum(), 3))
    for i in range(mc.sum()):
        R = q2R(B[i, 3:7])
        p_rot = B[i, 0:3] - R @ PIVOT
        g = p_rot + R @ G
        mg = MO[i] - np.cross(g, F[i])
        M[i] = [mg @ (R @ X), mg @ (R @ N), mg @ (R @ C)]
    on = np.linalg.norm(F, axis=1) > 0.05                # samples with the drone touching the valve
    out["M_rms"] = np.sqrt((M[on] ** 2).mean(0)) if on.any() else np.full(3, np.nan)
    # HOW the valve is turned, over the moving part of the turn (t0 + 3 .. t0 + 13 s, or to the abort):
    # the tangential force on the fin [N] (x 0.160 m = its torque) against the couple about the
    # vertical at G [N.m] (the grip twisting the handle); and the CoM's along-nose offset [mm]
    mm = (c[:, 0] >= t0 + 3.0) & (c[:, 0] <= min(t0 + 13.0, t1))
    if mm.sum() > 20:
        B3 = interp_rows(c[mm, 0], box[:, 0], box[:, 1:8])
        Ft, Mz = [], []
        for i, k in enumerate(np.flatnonzero(mm)):
            R = q2R(B3[i, 3:7])
            f = c[k, 1:4] + c[k, 4:7]
            g = B3[i, 0:3] - R @ PIVOT + R @ G
            Ft.append(f @ (R @ np.array([0.0, 1.0, 0.0])))
            Mz.append((c[k, 13:16] + c[k, 16:19] - np.cross(g, f))[2])
        out["F_tan"], out["M_z"] = float(np.mean(Ft)), float(np.mean(Mz))
        md3 = (dbg[:, 0] >= t0 + 3.0) & (dbg[:, 0] <= min(t0 + 13.0, t1))
        e3 = dbg[md3, 48 + D:51 + D] - dbg[md3, 45 + D:48 + D]
        Q3 = interp_rows(dbg[md3, 0], od[:, 0], od[:, 7:11])
        out["lon_mean"] = 1e3 * float(np.mean([(q2R(Q3[i]).T @ e3[i])[0] for i in range(len(e3))]))
        out["eRp_mean"] = 1e3 * float(dbg[md3, 28 + D].mean())
    # THE EXIT: the jaws opening to the end of the record (completed runs): the peak contact force [N]
    # (the open fingers dragging on the handle while the claw climbs)
    marks0 = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    if "gripper_release:start" in marks0 and not bool(d["aborted"]):
        me = c[:, 0] >= marks0["gripper_release:start"] + 0.3
        out["F_exit_max"] = float(np.linalg.norm(c[me, 1:4] + c[me, 4:7], axis=1).max()) if me.any() else np.nan
    # FREE-FLIGHT reference: the hover from DIRECT entry to Go To Start (the same drift, no contact):
    # the sideways CoM error's rms [mm]
    marks = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    mh = (dbg[:, 0] >= marks.get("direct", 0) + 3.0) & (dbg[:, 0] <= marks.get("go_to_start:start", 0))
    if mh.sum() > 100:
        eh = dbg[mh, 48 + D:51 + D] - dbg[mh, 45 + D:48 + D]
        Qh = interp_rows(dbg[mh, 0], od[:, 0], od[:, 7:11])
        sh = np.array([(q2R(Qh[i]).T @ eh[i])[1] for i in range(len(eh))])
        out["side_hover"] = 1e3 * np.sqrt(np.mean((sh - sh.mean()) ** 2))
    out["F_p95"] = np.percentile(np.linalg.norm(F, axis=1), 95) if mc.any() else np.nan
    # sway: the CoM error in the body frame
    md = (dbg[:, 0] >= t0) & (dbg[:, 0] <= t1)
    tt = dbg[md, 0]
    ex = dbg[md, 48 + D:51 + D] - dbg[md, 45 + D:48 + D]
    Q = interp_rows(tt, od[:, 0], od[:, 7:11])
    eb = np.array([q2R(Q[i]).T @ ex[i] for i in range(len(ex))])
    out["side_rms"] = 1e3 * np.sqrt(np.mean((eb[:, 1] - eb[:, 1].mean()) ** 2))
    out["lon_rms"] = 1e3 * np.sqrt(np.mean((eb[:, 0] - eb[:, 0].mean()) ** 2))
    # the sway guard's signal: |d/dt (horizontal CoM error)|, low-passed 0.25 s
    if len(tt) > 20:
        tu = np.arange(tt[0], tt[-1], 0.004)
        e = interp_rows(tu, tt, ex[:, :2])
        a = 1 - np.exp(-0.004 / 0.25)
        v = np.gradient(e, 0.004, axis=0)
        lp = np.zeros_like(v)
        for k in range(1, len(v)):
            lp[k] = lp[k - 1] + a * (v[k] - lp[k - 1])
        out["sway_rate_p99"] = 1e3 * np.percentile(np.linalg.norm(lp, axis=1), 99)
    out["eR_roll"] = 1e3 * np.sqrt(np.mean(dbg[md, 29 + D] ** 2))
    out["eR_pitch"] = 1e3 * np.sqrt(np.mean(dbg[md, 28 + D] ** 2))
    out["dr_roll"] = np.sqrt(np.mean((dbg[md, 35 + D] - dbg[md, 35 + D].mean()) ** 2))
    mj = (J[:, 0] >= t0) & (J[:, 0] <= t1)
    q = np.degrees(J[mj, 1:5])
    out["q1_std"], out["q4_std"] = q[:, 0].std(), q[:, 3].std()
    out["q14_corr"] = np.corrcoef(q[:, 0], q[:, 3])[0, 1] if len(q) > 10 else np.nan
    out["q3_min"] = q[:, 2].min()
    if c.shape[1] >= 24:                                  # v3: [20..22] in the message -> columns 21..23
        r = np.degrees(c[mc, 21])
        out["roll"] = (np.nanmin(r), np.nanmax(r))
        out["roll_rate_rms"] = np.degrees(np.sqrt(np.nanmean(c[mc, 22] ** 2)))
        out["roll_sep_max"] = 1e3 * np.nanmax(c[mc, 23])
        # the roll fin's angle before the grasp (Ready) and after the release
        marks = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
        pre = (c[:, 0] < marks.get("gripper_close:start", t0)) & (c[:, 0] > marks.get("go_to_start:start", 0))
        out["roll_pre"] = (np.nanmin(np.degrees(c[pre, 21])), np.nanmax(np.degrees(c[pre, 21]))) if pre.any() else None
    return out


def fmt(m):
    s = (f"   {m['mode']:13s} couple at G X/N/C {m['M_rms'][0] * 1e3:5.1f}/{m['M_rms'][1] * 1e3:5.1f}/"
         f"{m['M_rms'][2] * 1e3:5.1f} mN.m | sway side {m['side_rms']:5.1f} lon {m['lon_rms']:5.1f} mm, "
         f"rate p99 {m.get('sway_rate_p99', np.nan):5.0f} mm/s | e_R roll {m['eR_roll']:5.1f} pitch {m['eR_pitch']:5.1f} mrad"
         f", d_hat roll {m['dr_roll'] * 1e3:4.0f} mN.m | q1 {m['q1_std']:4.1f} q4 {m['q4_std']:4.1f} deg corr {m['q14_corr']:+.2f}")
    if "roll" in m:
        s += (f" | ROLL {m['roll'][0]:+5.1f}..{m['roll'][1]:+5.1f} deg, rate rms {m['roll_rate_rms']:4.1f} deg/s, "
              f"joint err max {m['roll_sep_max']:.2f} mm"
              + (f", before the grasp {m['roll_pre'][0]:+.1f}..{m['roll_pre'][1]:+.1f}" if m.get("roll_pre") else ""))
    return s


if __name__ == "__main__":
    tags = sys.argv[1:] or (sorted(os.path.basename(p)[:-4] for p in glob.glob(RUNS + "*.npz")) + V2_TAGS)
    for t in tags:
        path = RUNS if os.path.exists(RUNS + t + ".npz") else V2
        cs.R = path
        try:
            print(cs.row(t))
            m = metrics(path, t)
            if "M_rms" in m:
                print(fmt(m))
        except Exception as exc:
            print(f"{t:14s} ERROR {exc!r}")

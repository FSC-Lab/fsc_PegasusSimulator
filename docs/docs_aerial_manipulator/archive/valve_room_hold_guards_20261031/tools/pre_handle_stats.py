#!/usr/bin/env python3
"""The signals the new pre-grasp guards would read, over every recorded valve run: the room hold above the handle
(Go To Start done -> Ready) and the Ready descent (Ready -> Ready done / abort). Per phase: the horizontal CoM error
||x_c - x_cd||_xy (wb_control_debug [48,49] - [45,46]) max, its rate low-passed 0.25 s (the sway guard's filter) max,
q3 min, and the q3 rate guard's prediction min (look-ahead 0.1 s, filter 0.02 s, q2 wall 45 deg); plus, at Ready's
start, the CoM error and rate the Ready gate would have seen.
    PYTHONNOUSERSITE=1 /usr/bin/python3 pre_handle_stats.py [--isaac-only | --hw-only]
Isaac runs: the pl_mission.py npz (dbg = [t, step, debug...], joints = [t, q1..q4 (arm convention), eff.., step]).
Hardware: the extracted bags of 10-28, 10-29 and 10-31 (bp_common Bag)."""
import glob
import importlib.util
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.join(HERE, "..", "..")
TAU_SWAY, TAU_Q3, LOOK, Q2W = 0.25, 0.02, 0.1, 45.0


def lp_rate(t, e2):
    """the sway guard's rate filter on the 2-D error e2 (N x 2), as the planner runs it"""
    v = np.zeros(2); out = np.zeros(len(t))
    for k in range(1, len(t)):
        ds = t[k] - t[k - 1]
        if 0 < ds < 0.1:
            v += (1 - np.exp(-ds / TAU_SWAY)) * ((e2[k] - e2[k - 1]) / ds - v)
        elif ds >= 0.1:
            v[:] = 0
        out[k] = np.linalg.norm(v)
    return out


def q3_pred(t, q2, q3):
    """the q3 rate guard's prediction (deg) and the filtered q3 rate (deg/s)"""
    v3 = v2 = 0.0; pred = np.zeros(len(t)); rate = np.zeros(len(t))
    for k in range(len(t)):
        if k and 0 < t[k] - t[k - 1] < 0.1:
            dt = t[k] - t[k - 1]; a = 1 - np.exp(-dt / TAU_Q3)
            v3 += a * ((q3[k] - q3[k - 1]) / dt - v3); v2 += a * ((q2[k] - q2[k - 1]) / dt - v2)
        elif k:
            v3 = v2 = 0.0
        credit = max(0.0, q2[k] + LOOK * v2 - Q2W) if q2[k] <= Q2W + 1 else 0.0
        pred[k] = q3[k] + LOOK * v3 + credit; rate[k] = v3
    return pred, rate


def phase_stats(tw, ecom, vcom, tj, q2, q3, a, z):
    mw = (tw >= a) & (tw <= z); mj = (tj >= a) & (tj <= z)
    if mw.sum() < 5 or mj.sum() < 5:
        return None
    pred, rate = q3_pred(tj[mj], q2[mj], q3[mj])
    # the rate guard's own condition: prediction below 3 deg while falling faster than 30 deg/s
    fall = rate < -30.0
    return dict(e_max=1e3 * ecom[mw].max(), v_max=1e3 * vcom[mw].max(), q3_min=q3[mj].min(),
                pred_min=(pred[fall].min() if fall.any() else np.nan), dur=z - a)


def at(tw, x, t):
    i = min(np.searchsorted(tw, t), len(tw) - 1)
    return x[i]


rows = []


def add(label, tw, D, tj, q2, q3, t_hold, t_ready, t_end, end_kind):
    xcd = D[:, 45:47]; xc = D[:, 48:50]
    ok = np.isfinite(xcd).all(1) & np.isfinite(xc).all(1)
    tw, e2 = tw[ok], (xc - xcd)[ok]
    ecom = np.linalg.norm(e2, axis=1); vcom = lp_rate(tw, e2)
    h = phase_stats(tw, ecom, vcom, tj, q2, q3, t_hold, t_ready)
    r = phase_stats(tw, ecom, vcom, tj, q2, q3, t_ready, t_end)
    rows.append((label, h, r, 1e3 * at(tw, ecom, t_ready), 1e3 * at(tw, vcom, t_ready), end_kind))


def isaac_runs():
    for f in sorted(glob.glob(os.path.join(ARCH, "valve_*", "runs", "*.npz")) +
                    glob.glob(os.path.join(ARCH, "wb_valve_turning_flight_20261028", "sim", "runs", "*.npz"))):
        try:
            z = np.load(f, allow_pickle=True)
            names = list(z["marks_name"]); ts = list(z["marks_t"])
            mt = lambda n: ts[names.index(n)] if n in names else None
            th, tr = mt("go_to_start:end"), mt("ready:start")
            if th is None or tr is None or "dbg" not in z.files or z["dbg"].shape[1] < 52:
                continue
            te = mt("ready:end")
            ab = [t for t, n in zip(ts, names) if n.startswith("abort") and t > tr]
            end_kind = "done" if te is not None else "abort"
            te = te if te is not None else (ab[0] if ab else tr + 3.0)
            D = z["dbg"]; tw = D[:, 0]; D = D[:, 2:]
            J = z["joints"]; tj = J[:, 0]; q2 = np.degrees(J[:, 2]); q3 = np.degrees(J[:, 3])
            run = os.path.relpath(f, ARCH).replace("/runs/", "/").replace(".npz", "")
            add("isaac " + run, tw, D, tj, q2, q3, th, tr, te, end_kind)
        except Exception as ex:  # noqa: BLE001 -- an old npz layout: skip it, say so
            print("skip", os.path.basename(f), ex, file=sys.stderr)


def hw_runs():
    spec = importlib.util.spec_from_file_location(
        "bp", os.path.join(ARCH, "wb_box_pushing_flight_20261021", "tools", "bp_common.py"))
    bp = importlib.util.module_from_spec(spec); spec.loader.exec_module(bp)
    for folder, tags in (("wb_valve_turning_flight_20261028", ("180652", "181833")),
                         ("wb_valve_turning_flight_20261029", ("145526",)),
                         ("wb_valve_turning_flight_20261031", ("174943", "175725", "180423", "211808"))):
        bp.NPZ = os.path.join(ARCH, folder, "npz")
        for tag in tags:
            b = bp.Bag(tag); ed = b.edges("pl_pstatus")
            th = [t for t, s in ed if s.startswith("DONE go_to_start")]
            tr = [t for t, s in ed if s.startswith("FLYING ready")]
            if not th or not tr:
                continue
            th, tr = th[0], tr[0]
            te = [(t, s) for t, s in ed if t > tr and (s.startswith("DONE ready") or s.startswith("ABORTING"))]
            te, kind = (te[0][0], "done" if te[0][1].startswith("DONE") else "abort") if te else (tr + 3.0, "?")
            W = b.f("wb", "data"); ok = np.array([len(r) >= 51 for r in W])
            D = np.array([r[:51] for r in W[ok]]); tw = b.t("wb")[ok]
            names = [s.strip(" '[]") for s in str(b.f("js", "name")[0]).split(",")]
            P = np.degrees(b.f("js", "position")); tj = b.t("js")
            add(f"HW {folder[-8:]} {tag}", tw, D, tj, P[:, names.index("joint2")], P[:, names.index("joint3")],
                th, tr, te, kind)


if "--hw-only" not in sys.argv:
    isaac_runs()
if "--isaac-only" not in sys.argv:
    hw_runs()
f = lambda x: "  -  " if x is None or not np.isfinite(x) else f"{x:5.0f}"   # noqa: E731
print(f"{'run':58s} | {'HOLD':^27s} | {'READY':^35s} | at Ready  ")
print(f"{'':58s} | {'dur':>4s} {'e mm':>5s} {'v mm/s':>6s} {'q3min':>5s} | {'dur':>4s} {'e mm':>5s} {'v mm/s':>6s} "
      f"{'q3min':>5s} {'pred':>5s} {'end':>5s} | {'e mm':>5s} {'v':>4s}")
for lab, h, r, e0, v0, kind in rows:
    hs = f"{h['dur']:4.1f} {f(h['e_max'])} {f(h['v_max']):>6s} {f(h['q3_min'])}" if h else " " * 23
    rs = f"{r['dur']:4.1f} {f(r['e_max'])} {f(r['v_max']):>6s} {f(r['q3_min'])} {f(r['pred_min'])} {kind:>5s}" \
        if r else " " * 31
    print(f"{lab[:58]:58s} | {hs} | {rs} | {f(e0)} {f(v0)[1:]}")

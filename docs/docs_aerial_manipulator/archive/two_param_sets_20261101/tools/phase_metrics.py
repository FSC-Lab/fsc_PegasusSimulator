#!/usr/bin/env python3
"""Per-sub-phase stability metrics of a box-pushing flight (pl_mission.py npz), the
acceptance table of ../PLAN.md section 1. One row per run, then one pooled row per
configuration (the tag without its trailing _<rep>): mean [min..max] (std).

    PYTHONNOUSERSITE=1 /usr/bin/python3 phase_metrics.py ../runs/*.npz [--ref DIR ...] [--csv out.csv]

Windows (the driver's marks; with --gs-auto the arm GS runs Ready / close / Push / Exit):
  READY    ready:end (the claw arrives on the fin and is held in the ROOM, jaws open)
           -> push:start (the jaws gripped; the contact flag goes on)
  SETTLE   push:start -> + push_pull_push_settle_s (the clamped pause; the free-axis
           anchor blends in here unless it was latched at the world hold)
  SLIDE    the end of the settle -> push:end
  RELEASE  push:end -> gripper_release:end + 1 s (the jaws still closed, then open)
  EXIT     push:end -> exit:end
Columns:
  held     READY's length [s] (the open-jaw room hold before the grasp)
  rdy_sw / rdy_q3 / rdy_q2   READY: swing (q2-q3)/2 peak-to-peak [deg], q3 min, q2 max
  clw      the claw's distance to the grasp point at the close [mm] (arrival_error [2])
  set_sw / sld_sw            SETTLE / SLIDE swing std [deg]
  psh_q3 / psh_q2            SETTLE+SLIDE q3 min / q2 max [deg]
  back     the box moved AGAINST the push in the settle [mm]
  stuck / pk                 SLIDE: share with the box below 3 mm/s while the plan moves
                             [%] / the box's peak speed [mm/s]
  trav     box travel along the push at push:end [mm]
  lag      the planner's box-lag guard reading, max |.| over the push [mm] (limit 40)
  frc      the force guard's reading (wb_control_debug [97..99] low-passed 3 rad/s, as the
           planner does), max from arrival to the exit [N] (limit 5)
  eimp     impedance residual, SLIDE xyz rms [N] (pl_metrics.py)
  ptilt    vehicle tilt max over SETTLE+SLIDE [deg]
  rel_q3 / rel_q2            RELEASE q3 min / q2 max [deg]
  rel_t    push:end -> the jaws fully open [s]
  bmove    the box's displacement from push:end to exit:end + 0.5 s [mm]
  etilt    vehicle tilt max over EXIT [deg]
  clamp    share of DIRECT samples from arrival to exit:end with a joint torque clamped [%]
"""
import argparse
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PL = os.path.abspath(os.path.join(HERE, "..", "..", "push_pull_20261003", "tools"))
sys.path.insert(0, PL)
try:
    import pl_metrics as PM   # noqa: E402
except Exception as e:        # the residual is optional (scipy under PYTHONNOUSERSITE)
    PM = None
    print(f"(pl_metrics unavailable: {e!r} -- no e_imp)", file=sys.stderr)

D = 2   # recorder columns before wb_control_debug[0]
D_EY, D_XCD, D_XC, D_NCLAMP, D_F = 24, 45, 48, 88, 97


def win(a, t0, t1):
    return a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]


def zax(q):
    w, x, y, z = q.T
    return np.column_stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def args_of(path):
    kv = {}
    p = path[:-4] + ".args"
    if os.path.isfile(p):
        for ln in open(p).read().split("\n"):
            if "=" in ln:
                k, v = ln.split("=", 1)
                kv[k.strip()] = v.strip()
    return kv


def lpf(t, x, w):
    """first-order low-pass at w [rad/s], causal, on irregular samples (the planner's guard filter)"""
    y = np.zeros_like(x)
    y[0] = x[0]
    for k in range(1, len(t)):
        a = 1.0 - np.exp(-w * max(t[k] - t[k - 1], 0.0))
        y[k] = y[k - 1] + a * (x[k] - y[k - 1])
    return y


def one(path):
    z = np.load(path, allow_pickle=True)
    tag = os.path.basename(path)[:-4]
    kv = args_of(path)
    m = {str(n): float(t) for n, t in zip(z["marks_name"], z["marks_t"])}
    r = {"tag": tag, "ok": not bool(z["aborted"]), "reason": str(z["reason"]),
         "noise": kv.get("NOISE", "") or "iso", "bw": kv.get("NOISE_BW", "") or "-"}
    need = ("ready:end", "push:start", "push:end")
    if any(k not in m for k in need):
        r["stage"] = "before the push" if "push:start" not in m else "in the push"
        return r
    settle = float(kv.get("push_pull_push_settle_s", 1.5))
    ar, ps, pe = m["ready:end"], m["push:start"], m["push:end"]
    sl0 = ps + settle
    xe = m.get("exit:end", pe + 7.0)
    re_ = m.get("gripper_release:end", pe + 1.0) + 1.0
    J = z["joints"]
    deg = np.degrees

    def sw_of(a, b):
        j = win(J, a, b)
        return j, deg(j[:, 2] - j[:, 3]) / 2
    r["held"] = ps - ar
    j, s = sw_of(ar, ps)
    if len(j):
        r["rdy_sw"], r["rdy_q3"], r["rdy_q2"] = np.ptp(s), deg(j[:, 3]).min(), deg(j[:, 2]).max()
    A = z["arrival"]
    if len(A) and "gripper_close:start" in m:
        k = np.searchsorted(A[:, 0], m["gripper_close:start"])
        k = min(max(k - 1, 0), len(A) - 1)
        r["clw"] = A[k, 3] * 1e3
    _, s = sw_of(ps, sl0)
    r["set_sw"] = s.std() if len(s) else np.nan
    _, s = sw_of(sl0, pe)
    r["sld_sw"] = s.std() if len(s) else np.nan
    j, _ = sw_of(ps, pe)
    r["psh_q3"], r["psh_q2"] = deg(j[:, 3]).min(), deg(j[:, 2]).max()
    B = z["box"]
    b0 = win(B, ps - 0.3, ps)[-1]
    sgn = lambda bb: -(bb[:, 2] - b0[2]) * 1e3   # noqa: E731  the push is -y
    r["back"] = -min(0.0, sgn(win(B, ps, sl0)).min())
    r["trav"] = sgn(win(B, pe - 0.1, pe))[-1]
    bs = win(B, sl0, pe)
    t = bs[:, 0]
    ss = sgn(bs) * 1e-3
    kk = np.searchsorted(t, t - 0.2)
    okk = (t - t[kk]) > 0.15
    vb = np.full_like(t, np.nan)
    vb[okk] = (ss[okk] - ss[kk[okk]]) / (t[okk] - t[kk[okk]])
    r["pk"] = np.nanmax(vb) * 1e3 if np.isfinite(vb).any() else np.nan
    rd = win(z["refd"], sl0, pe)
    if len(rd) > 3:
        vp = -rd[:, 2]
        vpi = np.interp(t, rd[:, 0], vp)
        moving = vpi > 0.3 * vp.max()
        r["stuck"] = 100.0 * np.sum(moving & (vb < 3e-3)) / max(moving.sum(), 1)
    a = win(A, ps, pe)
    r["lag"] = np.abs(a[:, 12]).max() * 1e3 if len(a) else np.nan
    dbg = z["dbg"].astype(float)
    dd = win(dbg, ar - 2.0, m.get("exit:start", pe + 1.0))
    if len(dd) > 5:
        F = np.column_stack([lpf(dd[:, 0], dd[:, D + D_F + i], 3.0) for i in range(3)])
        mm = dd[:, 0] >= ar
        r["frc"] = np.linalg.norm(F[mm], axis=1).max()
    dc = win(dbg, ar, xe)
    if len(dc):
        r["clamp"] = 100.0 * np.mean(dc[:, D + D_NCLAMP] > 0)
    dp = win(dbg, ps, pe)
    ex = np.linalg.norm(dp[:, D + D_XC:D + D_XC + 3] - dp[:, D + D_XCD:D + D_XCD + 3], axis=1) * 1e3
    r["com"] = np.sqrt(np.mean(ex ** 2))
    BT = z["base_truth"]
    tilt = lambda a_, b_: deg(np.arccos(np.clip(zax(win(BT, a_, b_)[:, 4:8])[:, 2], -1, 1))).max()   # noqa: E731
    r["ptilt"] = tilt(ps, pe)
    j, _ = sw_of(pe, re_)
    if len(j):
        r["rel_q3"], r["rel_q2"] = deg(j[:, 3]).min(), deg(j[:, 2]).max()
    if "gripper_release:end" in m:
        r["rel_t"] = m["gripper_release:end"] - pe
    if "exit:end" in m:
        b1 = win(B, pe - 0.05, pe)[-1]
        b2 = win(B, xe, xe + 0.6)
        if len(b2):
            r["bmove"] = np.linalg.norm(b2[-1, 1:4] - b1[1:4]) * 1e3
        r["etilt"] = tilt(pe, xe)
    if PM is not None:
        try:
            y = path[:-4] + ".yaml"
            o = PM.analyse(path, PM.yaml_gains(y if os.path.isfile(y) else PM.YAML), 5.0)
            if "SLIDE" in o["imp"]:
                r["eimp"] = o["imp"]["SLIDE"]["e_xyz"][0]
        except BaseException as e:   # noqa: BLE001  (analyse raises SystemExit on old runs)
            r["eimp_err"] = repr(e)[:60]
    return r


COLS = [("held", "{:5.2f}"), ("rdy_sw", "{:6.1f}"), ("rdy_q3", "{:6.1f}"), ("rdy_q2", "{:6.1f}"),
        ("clw", "{:5.1f}"), ("set_sw", "{:6.2f}"), ("sld_sw", "{:6.2f}"), ("psh_q3", "{:6.1f}"),
        ("psh_q2", "{:6.1f}"), ("back", "{:5.1f}"), ("stuck", "{:5.0f}"), ("pk", "{:4.0f}"),
        ("trav", "{:6.1f}"), ("lag", "{:5.1f}"), ("frc", "{:5.2f}"), ("eimp", "{:5.2f}"),
        ("com", "{:5.1f}"), ("ptilt", "{:5.1f}"), ("rel_q3", "{:6.1f}"), ("rel_q2", "{:6.1f}"),
        ("rel_t", "{:5.2f}"), ("bmove", "{:5.1f}"), ("etilt", "{:5.1f}"), ("clamp", "{:5.1f}")]


def group_of(tag):
    head, _, tail = tag.rpartition("_")
    return head if tail.isdigit() and head else tag


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--csv")
    a = ap.parse_args()
    paths = []
    for p in a.runs:
        paths += sorted(glob.glob(p)) if any(c in p for c in "*?[") else [p]
    rows = [one(p) for p in paths]
    w = max(len(r["tag"]) for r in rows) + 1
    hdr = f"{'run':{w}s} ok noise  bw " + " ".join(f"{k:>{len(f.format(0.0))}s}" for k, f in COLS)
    print(hdr)
    for r in rows:
        line = f"{r['tag']:{w}s} {'y ' if r['ok'] else 'N '} {r['noise']:>5s} {r['bw']:>3s} "
        line += " ".join((f.format(r[k]) if k in r and np.isfinite(r[k]) else " " * len(f.format(0.0))) for k, f in COLS)
        if not r["ok"]:
            line += "  " + r["reason"][:80]
        print(line)
    groups = {}
    for r in rows:
        groups.setdefault(group_of(r["tag"]), []).append(r)
    print()
    for g, lst in groups.items():
        okg = [r for r in lst if r["ok"]]
        line = f"{g}: {len(okg)}/{len(lst)} complete"
        for k, _ in COLS:
            x = np.array([r.get(k, np.nan) for r in okg], float)
            x = x[np.isfinite(x)]
            if len(x):
                line += f" | {k} {x.mean():.2f} [{x.min():.2f}..{x.max():.2f}] ({x.std():.2f})"
        print(line + "\n")
    if a.csv:
        import csv
        with open(a.csv, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["tag", "group", "ok", "reason", "noise", "bw"] + [k for k, _ in COLS])
            for r in rows:
                wr.writerow([r["tag"], group_of(r["tag"]), r["ok"], r["reason"], r["noise"], r["bw"]]
                            + [r.get(k, "") for k, _ in COLS])


if __name__ == "__main__":
    main()

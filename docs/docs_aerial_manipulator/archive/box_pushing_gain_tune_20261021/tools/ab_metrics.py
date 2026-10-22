#!/usr/bin/env python3
"""The 2026-10-21 box-pushing A/B (station automation on; free-axis anchor 0.75 vs 0), per run,
plus the release-delay campaign's d02_* runs (driver-run Ready / Push, Exit by the station 0.2 s)
as a reference. Isaac runs only (pl_mission.py npz).

    /usr/bin/python3 ab_metrics.py

Columns:
  held     the claw held in the room with the jaws OPEN before the grasp [s]: ready:end -> the push
  swing    (q2 - q3)/2 std over the push step (the settle + the slide) [deg]
  q3min / q2max   from the claw's arrival on the handle to the end of the exit [deg]
           (the planner's q3 guard 10, the real arm's q2 limit 45)
  q1/q4    std over the push step [deg]
  trav     the box's travel along the push at the push end [mm of 100]; end = after the exit
  back     the box's largest move AGAINST the push while clamped, before the slide [mm]
  stuck    the share of the slide with the box below 3 mm/s while the plan moves [%]
  pk       the box's peak speed in the slide [mm/s]
  btilt    the box's largest tilt change, arrival -> exit end [deg]
  CoM / EE the CoM error and the law's claw error e_y, rms over the push step [mm]
  vtilt    the vehicle's largest tilt over the push step [deg]
"""
import glob
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs")
REF = os.path.join(HERE, "..", "..", "box_pushing_release_delay_20261021", "runs")
SETTLE = 1.5


def win(a, t0, t1):
    return a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]


def zax(q):
    w, x, y, z = q.T
    return np.column_stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def args_of(path):
    kv = {}
    p = path[:-4] + ".args"
    if os.path.isfile(p):
        for ln in open(p).read().split():
            if "=" in ln:
                k, v = ln.split("=", 1)
                kv[k] = v
    return kv


def one(path):
    z = np.load(path, allow_pickle=True)
    tag = os.path.basename(path)[:-4]
    kv = args_of(path)
    m = {str(n): float(t) for n, t in zip(z["marks_name"], z["marks_t"])}
    r = {"tag": tag, "ok": not bool(z["aborted"]), "reason": str(z["reason"]),
         "w": float(kv.get("FREE_W", "0") or 0), "auto": kv.get("ARM_GS_AUTO_STEPS", "false") == "true"}
    need = ("ready:end", "push:start", "push:end")
    if any(k not in m for k in need):
        return r
    re_, ps, pe = m["ready:end"], m["push:start"], m["push:end"]
    xe = m.get("exit:end", pe + 7.0)
    r["held"] = ps - re_
    J = z["joints"]
    jp = win(J, ps, pe)
    sw = np.degrees(jp[:, 2] - jp[:, 3]) / 2
    r["swing"] = sw.std()
    r["q1"], r["q4"] = np.degrees(jp[:, 1]).std(), np.degrees(jp[:, 4]).std()
    ja = win(J, re_, xe)
    r["q3min"], r["q2max"] = np.degrees(ja[:, 3]).min(), np.degrees(ja[:, 2]).max()
    B = z["box"]
    b0 = win(B, ps - 0.2, ps)[-1]
    s = lambda bb: -(bb[:, 2] - b0[2]) * 1e3   # noqa: E731  the push is -y
    r["trav"] = s(win(B, pe - 0.05, pe))[-1]
    r["end"] = s(win(B, xe - 0.1, xe + 2.0))[-1]
    r["back"] = -min(0.0, s(win(B, ps, ps + SETTLE)).min())
    bs = win(B, ps + SETTLE, pe)
    t = bs[:, 0]
    ss = s(bs) * 1e-3
    k = np.searchsorted(t, t - 0.2)
    okk = (t - t[k]) > 0.15
    vb = np.full_like(t, np.nan)
    vb[okk] = (ss[okk] - ss[k[okk]]) / (t[okk] - t[k[okk]])
    r["pk"] = np.nanmax(vb) * 1e3
    rd = win(z["refd"], ps + SETTLE, pe)
    vp = -rd[:, 2]
    vpi = np.interp(t, rd[:, 0], vp)
    moving = vpi > 0.3 * vp.max()
    r["stuck"] = 100.0 * np.sum(moving & (vb < 3e-3)) / max(moving.sum(), 1)
    ba = win(B, re_, xe)
    zz = zax(ba[:, 4:8])
    r["btilt"] = np.degrees(np.arccos(np.clip(zz @ zz[0], -1, 1))).max()
    d = win(z["dbg"].astype(float), ps, pe)
    ex = np.linalg.norm(d[:, 2 + 48:2 + 51] - d[:, 2 + 45:2 + 48], axis=1) * 1e3
    ey = np.linalg.norm(d[:, 2 + 24:2 + 27], axis=1) * 1e3
    r["com"], r["ee"] = np.sqrt(np.mean(ex ** 2)), np.sqrt(np.mean(ey ** 2))
    bt = win(z["base_truth"], ps, pe)
    r["vtilt"] = np.degrees(np.arccos(np.clip(zax(bt[:, 4:8])[:, 2], -1, 1))).max()
    return r


COLS = [("held", "{:5.2f}"), ("swing", "{:5.2f}"), ("q3min", "{:6.1f}"), ("q2max", "{:6.1f}"),
        ("q1", "{:5.2f}"), ("q4", "{:5.2f}"), ("trav", "{:6.1f}"), ("end", "{:6.1f}"), ("back", "{:5.1f}"),
        ("stuck", "{:5.0f}"), ("pk", "{:5.0f}"), ("btilt", "{:5.2f}"), ("com", "{:5.1f}"), ("ee", "{:5.1f}"),
        ("vtilt", "{:5.1f}")]


def main():
    paths = sorted(glob.glob(os.path.join(RUNS, "a*.npz"))) + sorted(glob.glob(os.path.join(REF, "d02_*.npz")))
    rows = [one(p) for p in paths]
    print("run       w    auto ok " + " ".join(f"{k:>{len(f.format(0.0))}s}" for k, f in COLS))
    for r in rows:
        line = f"{r['tag']:9s} {r['w']:4.2f} {'yes ' if r['auto'] else 'no  '} {'y ' if r['ok'] else 'N '} "
        line += " ".join((f.format(r[k]) if k in r and np.isfinite(r[k]) else " " * len(f.format(0.0))) for k, f in COLS)
        if not r["ok"]:
            line += "  " + r["reason"][:70]
        print(line)
    groups = {}
    for r in rows:
        groups.setdefault((r["w"], r["auto"]), []).append(r)
    for (w, auto), g in sorted(groups.items()):
        okg = [r for r in g if r["ok"]]
        line = f"\nfree-axis {w:.2f}, station automation {'on' if auto else 'off'}: {len(okg)}/{len(g)} complete"
        for k, _ in COLS:
            x = np.array([r.get(k, np.nan) for r in okg], float)
            x = x[np.isfinite(x)]
            if len(x):
                line += f" | {k} {x.mean():.1f} [{x.min():.1f}..{x.max():.1f}]"
        print(line)


if __name__ == "__main__":
    main()

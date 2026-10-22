#!/usr/bin/env python3
"""The release-delay A/B (2026-10-21): what the arm and the box do between the end of the push and
the end of the exit, per run, with the 10-21 hardware flight as the reference row.

    /usr/bin/python3 release_metrics.py [tag ...]

Windows, from the driver's marks (hardware: the 170123 bag's planner / gripper events):
  clamped  push end -> the jaws start opening (contact off, jaws closed, claw still world-held)
  release  the jaws start opening -> the exit starts flying
  exit     the exit flying
Columns:
  delay    the arm GS's push_pull_auto_exit_delay_s [s]
  open     push end -> the jaws start opening [s]   (the GS's delay + the release round trip)
  q3min / q2max per window [deg] (the planner's q3 guard 10, the real q2 limit 45)
  swing    (q2 - q3)/2 peak-to-peak over push end -> exit end [deg]
  box      the box's travel along the push from push end to exit end: min / final [mm]
           (negative = pulled back toward the vehicle)
  btilt    the box's largest tilt change in the same window [deg]
  vtilt    the vehicle's largest tilt in the same window [deg]
"""
import os
import sys
import glob
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs")
HW = os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "npz", "b170123.npz")


def tilt(q):
    w, x, y, z = q.T
    return np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))


def zaxis(q):
    w, x, y, z = q.T
    return np.column_stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def win(a, t0, t1):
    return a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]


def args_of(tag):
    kv = {}
    p = os.path.join(RUNS, tag + ".args")
    if os.path.isfile(p):
        for ln in open(p).read().split():
            if "=" in ln:
                k, v = ln.split("=", 1)
                kv[k] = v
    return kv


def score(name, delay, joints, box, base, m, ok, reason=""):
    """joints: t, q1..q4 [rad]; box/base: t, xyz, wxyz; m: marks dict"""
    r = {"run": name, "delay": delay, "ok": ok, "reason": reason}
    pe, ro = m.get("push:end"), m.get("release:start")
    xs, xe = m.get("exit:start"), m.get("exit:end")
    if pe is None:
        return r
    r["open"] = ro - pe if ro is not None else np.nan
    end = xe if xe is not None else (xs + 6.0 if xs is not None else pe + 8.0)
    for k, (a, b) in {"cl": (pe, ro), "re": (ro, xs), "ex": (xs, xe)}.items():
        if a is None or b is None or b <= a:
            continue
        j = win(joints, a, b)
        if len(j):
            r[f"q3_{k}"] = np.degrees(j[:, 3].min())
            r[f"q2_{k}"] = np.degrees(j[:, 2].max())
    j = win(joints, pe, end)
    sw = np.degrees(j[:, 2] - j[:, 3]) / 2
    r["swing"] = sw.max() - sw.min()
    bx = win(box, pe - 0.02, end)
    s = -(bx[:, 2] - bx[0, 2]) * 1e3           # the push is -y in both worlds
    r["box_min"], r["box_end"] = s.min(), s[-1]
    z = zaxis(bx[:, 4:8])
    r["btilt"] = np.degrees(np.arccos(np.clip(z @ z[0], -1, 1))).max()
    bs = win(base, pe, end)
    r["vtilt"] = tilt(bs[:, 4:8]).max()
    return r


def sim_row(tag):
    z = np.load(os.path.join(RUNS, tag + ".npz"), allow_pickle=True)
    m = {str(n): float(t) for n, t in zip(z["marks_name"], z["marks_t"])}
    kv = args_of(tag)
    return score(tag, float(kv.get("ARM_GS_AUTO_EXIT_DELAY", "nan")), z["joints"][:, :5], z["box"][:, :8],
                 z["base_truth"][:, :8], m, not bool(z["aborted"]), str(z["reason"]))


def hw_row():
    if not os.path.isfile(HW):
        return None
    z = np.load(HW, allow_pickle=True)
    t0 = min(z[k][0] for k in z.files if k.endswith("__recv") and len(z[k]))
    t = lambda k: z[k + "__recv"] - t0   # noqa: E731
    js = z["js__position"]
    joints = np.column_stack([t("js"), js[:, 2], js[:, 0], js[:, 1], js[:, 3]])
    box = np.column_stack([t("obj")] + [z[f"obj__pose.position.{c}"] for c in "xyz"]
                          + [z[f"obj__pose.orientation.{c}"] for c in "wxyz"])
    base = np.column_stack([t("mocap")] + [z[f"mocap__pose.position.{c}"] for c in "xyz"]
                           + [z[f"mocap__pose.orientation.{c}"] for c in "wxyz"])
    # 10-21: the q3 guard aborted at 46.62 s before the GS's 1.0 s exit; the abort's open
    # started at 46.65 s and its climb is the "exit" here (ends 58.93 s)
    m = {"push:end": 45.84, "release:start": 46.65, "exit:start": 48.15, "exit:end": 58.93}
    return score("HW 170123", 1.0, joints, box, base, m, False, "q3 guard abort at +0.78 s")


COLS = [("delay", "{:5.1f}"), ("open", "{:5.2f}"), ("q3_cl", "{:6.1f}"), ("q2_cl", "{:6.1f}"),
        ("q3_re", "{:6.1f}"), ("q2_re", "{:6.1f}"), ("q3_ex", "{:6.1f}"), ("q2_ex", "{:6.1f}"),
        ("swing", "{:6.1f}"), ("box_min", "{:7.1f}"), ("box_end", "{:7.1f}"), ("btilt", "{:5.2f}"),
        ("vtilt", "{:5.1f}")]


def main():
    tags = sys.argv[1:] or sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(RUNS, "d*.npz")))
    rows = [r for r in [hw_row()] if r] + [sim_row(t) for t in tags]
    hdr = "run          ok  " + " ".join(f"{k:>{len(f.format(0.0))}s}" for k, f in COLS)
    print(hdr)
    for r in rows:
        line = f"{r['run']:12s} {'y' if r['ok'] else 'N'}   "
        line += " ".join((f.format(r[k]) if k in r and np.isfinite(r[k]) else " " * len(f.format(0.0))) for k, f in COLS)
        if not r["ok"]:
            line += "   " + r["reason"][:80]
        print(line)
    for d in sorted({r["delay"] for r in rows[1:]}):
        g = [r for r in rows[1:] if r["delay"] == d]
        line = f"\nIsaac, delay {d:.1f} s: {sum(r['ok'] for r in g)}/{len(g)} complete"
        for k in ("open", "q3_cl", "q3_re", "q3_ex", "q2_cl", "q2_re", "swing", "box_min", "box_end", "btilt", "vtilt"):
            x = np.array([r.get(k, np.nan) for r in g], float)
            x = x[np.isfinite(x)]
            if len(x):
                line += f" | {k} {x.mean():.1f} [{x.min():.1f}..{x.max():.1f}]"
        print(line)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""One line per CAD-box run: outcome, box travel, the slide's impedance residual
and rho_UAV (pl_metrics, true contact wrench), the contact force's angle above
the table, the joints against the real arm's limits (q2 <= 45, q3 > 0, q3 <= 50),
the airframe drift along the push, and the airframe's lean while it drags the box
(mean body pitch, FLU + = nose down, and the peak tilt, from 1.5 s into the push),
the release (q3's peak from the push's end to 3 s into the exit, the box's
movement during the exit, the guard-filtered contact reading's peak), and the
Ready descent (q2 p-p, the claw height error p-p in mm, the largest touch in N),
and the grip (claw to the grasp point / across the fin when the jaws close, mm).

    PYTHONNOUSERSITE=1 /usr/bin/python3 summarize.py [tag ...]
"""
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs")
sys.path.insert(0, os.path.join(HERE, "..", "..", "push_pull_20261003", "tools"))
import pl_metrics as PM                     # noqa: E402
from pl_metrics import mark, interp, Rz     # noqa: E402
from pl_score import tilt_deg               # noqa: E402


def one(tag):
    z = np.load(os.path.join(RUNS, tag + ".npz"), allow_pickle=True)
    y = os.path.join(RUNS, tag + ".yaml")
    ytxt = open(y).read()
    pose = ytxt.split("push_pull_push_pose_deg:")[1].split("\n")[0].split("#")[0].strip()
    dist = float(ytxt.split("push_pull_push_distance:")[1].split("\n")[0].split("#")[0])
    r = dict(tag=tag, task="PULL" if dist < 0 else "push", pose=pose, aborted=bool(z["aborted"]),
             reason=str(z["reason"])[:90])
    ps, pe = mark(z, "push:start"), mark(z, "push:end")
    o, b, j = z["odom"], z["box"], z["joints"]
    t_end = pe
    if ps is not None and pe is None:          # failed in the push: stop before it broke away
        oo = o[o[:, 0] > ps]
        big = oo[tilt_deg(oo[:, 7], oo[:, 8], oo[:, 9], oo[:, 10]) > 8.0]
        ends = [float(big[0, 0])] if len(big) else []
        for e in z["events"]:
            e = str(e)
            if "push_pull: ABORTING" in e:
                ends.append(float(e[1:e.index("s]")]))
                break
        ends = [e for e in ends if e > ps]
        if ends:
            r["failed_after_s"] = min(ends) - ps
            t_end = min(ends) - 0.25
    if ps is not None and t_end is not None:
        bb = b[(b[:, 0] >= ps) & (b[:, 0] <= t_end)]
        r["box_mm"] = float(np.linalg.norm(bb[-1, 1:3] - bb[0, 1:3]) * 1e3) if len(bb) > 1 else float("nan")
        m = (j[:, 0] > ps) & (j[:, 0] < t_end)
        q = np.degrees(j[m, 1:5])
        r["q2"] = (q[:, 1].min(), q[:, 1].max())
        r["q3"] = (q[:, 2].min(), q[:, 2].max())
        ref, bt = z["ref"], z["base_truth"]
        xb = np.array([rw[4:7] - Rz(np.arctan2(rw[12], rw[11])) @ PM.TP.arm_fk_model(rw[7:11], PM.P)[0] for rw in ref])
        tu = np.arange(ps, min(t_end, bt[-1, 0], ref[-1, 0]), 0.01)
        eb = interp(tu, bt, (1, 2, 3)) - np.column_stack([np.interp(tu, ref[:, 0], xb[:, i]) for i in range(3)])
        r["drift_std_mm"] = float(eb[:, 1].std() * 1e3)
        r["drift_pk_mm"] = float(np.abs(eb[:, 1]).max() * 1e3)
        # the airframe lean while dragging the box: body pitch (FLU, + = nose down) and tilt
        w, x, yq, zq = bt[:, 4], bt[:, 5], bt[:, 6], bt[:, 7]
        pitch = np.degrees(np.arcsin(np.clip(2 * (w * yq - zq * x), -1, 1)))
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (x * x + yq * yq), -1, 1)))
        ms = (bt[:, 0] > ps + 1.5) & (bt[:, 0] < t_end)
        r["pitch_mean"] = float(pitch[ms].mean())
        r["tilt_max"] = float(tilt[ms].max())
    # THE RELEASE: from the push's end through the first 3 s of the exit -- q3's peak
    # (its +50 stop is where the stored load threw it before the unload), the box's
    # movement during the exit (the claw catching the fin), and the law's contact
    # reading low-passed as the planner's guard does it (vector, 3 rad/s)
    ex = mark(z, "exit:start")
    if pe is not None and ex is not None:
        m = (j[:, 0] > pe) & (j[:, 0] < ex + 3.0)
        r["rel_q3"] = float(np.degrees(j[m, 3]).max())
        kb = lambda tt: b[min(np.searchsorted(b[:, 0], tt), len(b) - 1), 1:3]
        r["exit_box_mm"] = float(np.linalg.norm(kb(ex + 6.0) - kb(ex)) * 1e3)
        d = z["dbg"].astype(float)
        G = np.zeros((len(d), 3))
        for i in range(1, len(d)):
            al = np.exp(-3.0 * min(max(d[i, 0] - d[i - 1, 0], 0.0), 0.1))
            G[i] = al * G[i - 1] + (1 - al) * d[i, 99:102]
        mr = (d[:, 0] > pe) & (d[:, 0] < ex + 1.0)
        r["rel_force"] = float(np.linalg.norm(G[mr], axis=1).max()) if mr.any() else float("nan")
    # THE READY DESCENT (the planner's "FLYING ready T=..." event to T later): the
    # arm's q2 swing, the law's claw height error e_y[z] (p-p) and the largest
    # claw-on-box contact force (the open jaws touching the handle)
    ev = [str(e) for e in z["events"] if "FLYING ready T=" in str(e)]
    if ev:
        t0 = float(ev[0][1:ev[0].index("s]")])
        T = float(ev[0].split("T=")[1].split("s")[0])
        m = (j[:, 0] > t0) & (j[:, 0] < t0 + T)
        if m.sum() > 1:
            r["rdy_q2pp"] = float(np.ptp(np.degrees(j[m, 2])))
        d = z["dbg"].astype(float)
        md = (d[:, 0] > t0) & (d[:, 0] < t0 + T)
        if md.sum() > 1:
            r["rdy_ezpp"] = float(np.ptp(d[md, 2 + 26]) * 1e3)
        ct = z["contact"]
        mc = (ct[:, 0] > t0) & (ct[:, 0] < t0 + T)
        r["rdy_touch"] = float(np.linalg.norm(ct[mc, 1:4] + ct[mc, 4:7], axis=1).max()) if mc.any() else 0.0
    # the grip: how far the claw was from the grasp point / across the fin when
    # the jaws were told to close (pl_mission's "close: claw ..." event)
    cl = [str(e) for e in z["events"] if "close: claw " in str(e)]
    if cl:
        w = cl[0].split("close: claw ")[1].split()
        r["grip_mm"], r["grip_across_mm"] = float(w[0]), float(w[6])
    try:
        a = PM.analyse(os.path.join(RUNS, tag + ".npz"), PM.yaml_gains(y), 5.0)
        clean = (not r["aborted"]) or "while exit" in r["reason"] or "exit response" in r["reason"]
        if clean and "SLIDE" in a["imp"]:
            r["imp"] = a["imp"]["SLIDE"]["e_xyz"][0]
            r["rho"] = a["eps"]["SLIDE"][0] / PM.L
            ser = a["series"]; tt = ser["t"]
            mm = (tt > ps + 2.5) & (tt < pe - 1.0)
            Fm = ser["F"][mm, :3].mean(0)
            r["angle"] = float(np.degrees(np.arctan2(Fm[2], np.hypot(Fm[0], Fm[1]))))
    except Exception as exc:                   # noqa: BLE001
        r["err"] = str(exc)[:60]
    return r


def main():
    tags = sys.argv[1:] or sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(RUNS, "cad_*.npz")))
    print(f"{'run':7s} {'task':4s} {'pose':22s} {'outcome':34s} {'box':>6s} {'e_imp':>6s} {'rho':>6s} "
          f"{'F ang':>6s} {'q2 range':>12s} {'q3 range':>12s} {'drift std/pk':>13s} {'pitch':>6s} {'tilt':>5s} "
          f"{'relq3':>6s} {'exitbox':>7s} {'relF':>5s} {'rdy q2':>6s} {'rdy ez':>6s} {'touch':>5s} {'grip':>5s} {'across':>6s}")
    for t in tags:
        r = one(t)
        out = "complete" if not r["aborted"] else (
            f"failed {r['failed_after_s']:.1f} s into the push" if "failed_after_s" in r else "failed: " + r["reason"][:26])
        f = lambda k, n=2: f"{r[k]:.{n}f}" if k in r else "-"
        q2 = f"{r['q2'][0]:5.1f}..{r['q2'][1]:4.1f}" if "q2" in r else "-"
        q3 = f"{r['q3'][0]:5.1f}..{r['q3'][1]:4.1f}" if "q3" in r else "-"
        dr = f"{r['drift_std_mm']:4.0f} / {r['drift_pk_mm']:3.0f}" if "drift_std_mm" in r else "-"
        print(f"{t:7s} {r['task']:4s} {r['pose']:22s} {out:34s} {f('box_mm', 0):>6s} {f('imp'):>6s} {f('rho', 3):>6s} "
              f"{f('angle', 0):>6s} {q2:>12s} {q3:>12s} {dr:>13s} {f('pitch_mean', 1):>6s} {f('tilt_max', 1):>5s} "
              f"{f('rel_q3', 0):>6s} {f('exit_box_mm', 0):>7s} {f('rel_force', 1):>5s} "
              f"{f('rdy_q2pp', 0):>6s} {f('rdy_ezpp', 0):>6s} {f('rdy_touch', 1):>5s} "
              f"{f('grip_mm', 1):>5s} {f('grip_across_mm', 1):>6s}")


if __name__ == "__main__":
    main()

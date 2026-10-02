#!/usr/bin/env python3
"""Generate the reference-trajectory grid (2026-09-27): radius x lap time x arm
motion (+ two clockwise variants), each from the real planner via
gen_circle_stream.py, N at a time on isolated DDS domains.

    /usr/bin/python3 gen_grid.py [--par 6]
Streams land in ../data/stream_<name>.npz; existing ones are skipped.
Arm motions (fold 60 deg, q2 centre 30 deg, all end where they start):
  static  q2 amp 0          half10  +-10 deg, period 2 x lap (the flown design)
  sync10  +-10, period = lap   sync15  +-15, period = lap   dbl10  +-10, period = lap/2
"""
import argparse
import os
import subprocess
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")
LOGD = os.path.join(HERE, "..", "logs")
UDP = os.path.expanduser("~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/q2_sine_sim_20260924/tools/fastdds_udp_only.xml")


def arm(kind, lap):
    return {"static": (0.0, 2 * lap), "half10": (10.0, 2 * lap), "sync10": (10.0, lap),
            "sync15": (15.0, lap), "dbl10": (10.0, lap / 2)}[kind]


def grid():
    out = []
    for r in (0.50, 0.75):
        for lap in (24, 32, 40):
            for kind in ("static", "half10", "sync10", "sync15", "dbl10"):
                out.append((f"r{int(r*100):03d}_L{lap}_{kind}", r, lap, kind, True))
    out.append(("r075_L24_half10_cw", 0.75, 24, "half10", False))
    out.append(("r075_L24_sync10_cw", 0.75, 24, "sync10", False))
    # round 2 (2026-09-27): clockwise across lap times, and a 28 s lap at 0.75 m
    for r in (0.50, 0.75):
        for lap in (24, 32, 40):
            for kind in ("static", "sync15"):
                out.append((f"r{int(r*100):03d}_L{lap}_{kind}_cw", r, lap, kind, False))
    for kind, ccw in (("sync15", True), ("sync15", False), ("static", True), ("static", False)):
        out.append((f"r075_L28_{kind}{'' if ccw else '_cw'}", 0.75, 28, kind, ccw))
    return out


def valid(path):
    import numpy as np
    try:
        d = np.load(path, allow_pickle=True)
    except Exception:
        return False
    st = [str(x) for x in d["pl_status__data"]]
    if len(st) != 2 or not st[0].startswith("EXECUTING") or st[1] != "HOLD":
        return False
    T = float(st[0].split("T=")[1].rstrip("s"))
    return len(d["wbref__recv"]) >= 0.97 * 100 * T


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--par", type=int, default=6); a = ap.parse_args()
    todo = []
    for g in grid():
        f = os.path.join(DATA, f"stream_{g[0]}.npz")
        if os.path.exists(f) and not valid(f):
            os.remove(f); print(f"[grid] {g[0]}: corrupt, regenerating", flush=True)
        if not os.path.exists(f):
            todo.append(g)
    print(f"[grid] {len(todo)} streams to generate", flush=True)
    running = []
    while todo or running:
        while todo and len(running) < a.par:
            name, r, lap, kind, ccw = todo.pop(0)
            amp, per = arm(kind, lap)
            # a FREE domain only: reusing one whose job is still running makes two
            # planners talk over each other (the first grid attempt's corrupted runs)
            busy = {it[3] for it in running}
            d = next(x for x in range(80, 80 + a.par + 1) if x not in busy)
            env = dict(os.environ, ROS_DOMAIN_ID=str(d), FASTRTPS_DEFAULT_PROFILES_FILE=UDP)
            cmd = ["/usr/bin/python3", os.path.join(HERE, "gen_circle_stream.py"), "--radius", str(r),
                   "--lap-time", str(lap), "--set", f"ee_traj_q2_amp_deg={amp}", "--set", f"ee_traj_q2_period_s={per}",
                   "--set", f"ee_traj_ccw={'true' if ccw else 'false'}", "--ns", f"uav_g{d}",
                   "--out", os.path.join(DATA, f"stream_{name}.npz")]
            lf = open(os.path.join(LOGD, f"gen_{name}.log"), "w")
            running.append((name, subprocess.Popen(cmd, stdout=lf, stderr=subprocess.STDOUT, env=env), lf, d))
        time.sleep(2)
        for item in list(running):
            name, pr, lf, d = item
            if pr.poll() is not None:
                lf.close(); running.remove(item)
                txt = open(os.path.join(LOGD, f"gen_{name}.log")).read().strip().splitlines()
                last = [l for l in txt if "[gen]" in l or "Error" in l or "refused" in l]
                f = os.path.join(DATA, f"stream_{name}.npz")
                ok = os.path.exists(f) and valid(f)
                print(f"[grid] {name} {'OK' if ok else 'FAILED'} {last[-1][:150] if last else txt[-1] if txt else ''}", flush=True)
                if os.path.exists(f) and not ok:
                    os.remove(f)


if __name__ == "__main__":
    main()

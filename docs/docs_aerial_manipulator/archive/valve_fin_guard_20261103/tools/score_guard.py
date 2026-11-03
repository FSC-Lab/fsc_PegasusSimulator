#!/usr/bin/env python3
"""Score the fin-loss safety runs (2026-11-03): per run, how it ended (completed / which guard aborted it), when
relative to the fault (the valve passing 30 deg into the turn), what the new guards read (push_pull/arrival_error
[34..38]: the claw's depth under the fin top, the fin across the claw, the fin ahead of the bracket, the clearance to
the valve's boxes and to its marker balls), the abort's climb, the vehicle's tilt, q3, DIRECT kept, and every
vehicle-valve contact by link (Isaac's contact report; the gripper parts -- hub, left/right grip and link -- push the
fin by design, anything else, /body above all, is a collision).
Run: PYTHONNOUSERSITE=1 /usr/bin/python3 score_guard.py [runs/*.npz]"""
import sys, os, glob, re, collections, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
GRIPPER = ("/hub", "/left_grip", "/right_grip", "/left_link", "/right_link")
FAULT_DEG = 30.0


def yaw(q):
    return np.degrees(np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                                           1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))))


def tilt(q):
    return np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))


def ev_time(z, pat):
    for e in z["events"]:
        if re.search(pat, e):
            return float(re.match(r"\[\s*([\d.]+)s\]", e).group(1)), e
    return None, None


def contacts(path):
    c = collections.Counter()
    if os.path.exists(path):
        for line in open(path):
            v = line.split(",")
            if v[0] == "C":
                c[v[2]] += 1
    return c


files = sys.argv[1:] or sorted(glob.glob(os.path.join(HERE, "..", "sim", "runs", "*.npz")))
for f in files:
    z = np.load(f, allow_pickle=True)
    name = os.path.basename(f)[:-4]
    mt = dict(zip(z["marks_name"], z["marks_t"]))
    t_push = mt.get("push:start")
    bx = z["box"]
    yv = yaw(bx[:, 4:8])
    a = z["arrival"]
    od = z["odom"]
    t_ab, e_ab = ev_time(z, r"push_pull: ABORTING")
    reason = re.sub(r".*SAFETY GUARD: ", "", e_ab).split(")) --")[0].split(") --")[0] if e_ab and "GUARD" in e_ab else (
        "operator / driver" if e_ab else "")
    turned = None
    t_fault = None
    if t_push is not None:
        i0 = np.searchsorted(bx[:, 0], t_push)
        y0 = yv[i0]
        after = (bx[:, 0] >= t_push) & (yv - y0 >= FAULT_DEG)
        t_fault = bx[after, 0][0] if after.any() else None
        tend = t_ab if t_ab else mt.get("push:end", bx[-1, 0])
        turned = yv[min(np.searchsorted(bx[:, 0], tend + 3.0), len(yv) - 1)] - y0
    out = f"{name:18s} "
    out += (f"ABORT +{t_ab - t_fault:5.2f} s after 30 deg" if (t_ab and t_fault) else
            f"ABORT (valve < 30 deg) " if t_ab else "completed") + f" | {reason[:95]}"
    print(out)
    # the guards' readings: [34..38] at columns 36..40 (the driver copy's layout), during the turn
    if t_push is not None and a.shape[1] > 40:
        tend = t_ab if t_ab else mt.get("push:end", a[-1, 0])
        w = (a[:, 0] >= t_push) & (a[:, 0] <= tend)
        if w.any():
            col = lambda k: a[w, 36 + k]
            fin = np.isfinite(col(0))
            def rng(x):
                x = x[np.isfinite(x)]
                return f"{x.min()*1e3:6.1f}..{x.max()*1e3:6.1f}" if len(x) else "   --"
            print(f"{'':18s} turn: depth {rng(col(0))} | across {rng(col(1))} | gap {rng(col(2))} | "
                  f"clear boxes {rng(col(3))} | balls {rng(col(4))} mm ({fin.sum()} samples)")
    # the abort's climb, the vehicle
    q = od[:, 7:11]
    tl = tilt(q)
    if t_ab:
        z0 = np.interp(t_ab, od[:, 0], od[:, 3])
        dz = lambda dt: np.interp(t_ab + dt, od[:, 0], od[:, 3]) - z0
        win = (od[:, 0] >= t_ab) & (od[:, 0] <= t_ab + 6)
        print(f"{'':18s} abort: base +{dz(0.5)*1e3:4.0f} / +{dz(1.0)*1e3:4.0f} / +{dz(1.5)*1e3:4.0f} / +{dz(3.0)*1e3:4.0f} mm "
              f"at 0.5/1/1.5/3 s; tilt max after {tl[win].max() if win.any() else float('nan'):.1f} deg")
    direct = od[:, 11]
    lost = bool((direct[od[:, 0] > mt.get("direct", 0) + 1] < 0.5).any())
    js = z["joints"]
    q3 = np.degrees(js[:, 3])
    w = js[:, 0] >= (mt.get("ready:start") or 0)
    print(f"{'':18s} valve turned {turned if turned is not None else float('nan'):5.1f} deg | tilt max {tl[od[:,0] > mt.get('direct', 0)].max():4.1f} deg | "
          f"q3 min {q3[w].min():5.1f} deg | DIRECT {'LOST' if lost else 'kept'} | aborted flag {bool(z['aborted'])}")
    c = contacts(f[:-4] + ".contacts.csv")
    bad = {k: n for k, n in c.items() if k not in GRIPPER}
    print(f"{'':18s} contacts: gripper {sum(n for k, n in c.items() if k in GRIPPER)}, "
          f"OTHER {dict(bad) if bad else 'none'}")

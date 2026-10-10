"""Hover error with and without the payload: every window where the planner's reference stands still.

window     reference CoM and claw both slower than 5 mm/s for at least 1.5 s, inside DIRECT
settled    the same window without its first 3 s (arrival transient)
error      airframe = measured CoM - reference CoM; claw = measured claw - reference claw (planner's FK on the
           fused odometry, the frame the planner and the law work in); horizontal = (x, y), vertical = z
payload    PP-2 only: hooked from the lift-off (47.0 s) until the basket was taken off the fingers (118.3 s);
           89.5-96.5 s (basket resting on the hat rim, then picked up again by the open fingers) is left out
Writes ../analysis/hover.json.
"""
import json
import tracking as T
from pp_common import *

LOADED = {"p2": (47.0, 118.3)}
EXCLUDE = {"p2": [(89.5, 96.5)]}
SETTLE = 3.0


def stats(E):
    """E: n x 3 [m] -> mm statistics."""
    h = E[:, :2]; m = h.mean(0); r = np.linalg.norm(h, axis=1); dev = np.linalg.norm(h - m, axis=1)
    return dict(n=int(len(E)), mean_xy=(1e3 * m).round(1).tolist(), bias=float(1e3 * np.linalg.norm(m)),
                scatter=float(1e3 * np.sqrt((dev ** 2).mean())), rms=float(1e3 * np.sqrt((r ** 2).mean())),
                p95=float(1e3 * np.percentile(r, 95)), max=float(1e3 * r.max()),
                z_mean=float(1e3 * E[:, 2].mean()), z_std=float(1e3 * E[:, 2].std()), z_max=float(1e3 * np.abs(E[:, 2]).max()),
                within20=float((r < 0.020).mean()), within50=float((r < 0.050).mean()), within70=float((r < 0.070).mean()))


def windows_of(nm):
    d, t0, S = T.build(nm); t = S["t"]; dt = t[1] - t[0]
    sp = np.maximum(np.linalg.norm(np.gradient(S["x_cd"], t, axis=0), axis=1), np.linalg.norm(np.gradient(S["r_ed"], t, axis=0), axis=1))
    still = sp < 0.005
    ph = pp_phases(d, t0)
    out = []; i = 0
    while i < len(t):
        if not still[i]:
            i += 1; continue
        j = i
        while j < len(t) and still[j]:
            j += 1
        if (j - i) * dt >= 1.5:
            out.append((t[i], t[j - 1]))
        i = j
    def lab(ts):
        for a, b, l in ph:
            if a <= ts < b:
                return l
        return ""
    return d, t0, S, [(a, b, lab(0.5 * (a + b))) for a, b in out]


def load_state(nm, a, b):
    if nm in LOADED:
        la, lb = LOADED[nm]
        if a >= la and b <= lb:
            return "payload"
        if b <= la or a >= lb:
            return "none"
        return "mixed"
    return "none"


if __name__ == "__main__":
    res = {}; pool = {"none": {"com": [], "ee": []}, "payload": {"com": [], "ee": []}}; pts = {"none": [], "payload": []}
    for nm in RUNS:
        d, t0, S, W = windows_of(nm); t = S["t"]; rows = []
        print(f"== {RUNS[nm][0]}")
        print("   window [s]      len  state    airframe: bias scatter  p95   max | claw: bias scatter  p95   max   z mean/std | phase")
        for a, b, l in W:
            # cut the window at the payload edges and at the excluded spans
            cuts = [a, b]
            if nm in LOADED:
                cuts += [x for x in LOADED[nm] if a < x < b]
            for x0, x1 in EXCLUDE.get(nm, []):
                cuts += [x for x in (x0, x1) if a < x < b]
            cuts = sorted(cuts)
            for k in range(len(cuts) - 1):
                ca, cb = cuts[k], cuts[k + 1]
                if any(x0 <= 0.5 * (ca + cb) <= x1 for x0, x1 in EXCLUDE.get(nm, [])):
                    continue
                st = load_state(nm, ca, cb)
                ms = (t >= (ca + SETTLE if ca == a else ca + SETTLE)) & (t <= cb)
                mall = (t >= ca) & (t <= cb)
                if mall.sum() < 50:
                    continue
                r = dict(t0=float(ca), t1=float(cb), state=st, phase=l, all=dict(com=stats(S["e_com"][mall]), ee=stats(S["e_ee"][mall]),
                                                                                  ee_mocap=stats(S["e_ee_mocap"][mall])))
                if ms.sum() >= 50:
                    r["settled"] = dict(com=stats(S["e_com"][ms]), ee=stats(S["e_ee"][ms]), ee_mocap=stats(S["e_ee_mocap"][ms]))
                    if st in pool:
                        pool[st]["com"].append(S["e_com"][ms]); pool[st]["ee"].append(S["e_ee"][ms])
                        e = S["e_ee"][ms][::5]; tt = t[ms][::5]          # 10 Hz for the scatter figure
                        pts[st].append(dict(run=RUNS[nm][0], phase=l, t=np.round(tt, 1).tolist(), xy=(1e3 * e[:, :2]).round(1).tolist(),
                                            z=(1e3 * e[:, 2]).round(1).tolist()))
                rows.append(r)
                c = r.get("settled", r["all"]); tag = "settled" if "settled" in r else "all    "
                print(f"   {ca:6.1f}-{cb:6.1f} {cb - ca:5.1f}  {st:8s} {c['com']['bias']:5.0f} {c['com']['scatter']:6.0f} {c['com']['p95']:5.0f} {c['com']['max']:5.0f} |"
                      f" {c['ee']['bias']:5.0f} {c['ee']['scatter']:6.0f} {c['ee']['p95']:5.0f} {c['ee']['max']:5.0f}  {c['ee']['z_mean']:+5.0f}/{c['ee']['z_std']:3.0f} | {tag} {l[:44]}")
        res[nm] = rows
    print("\npooled, settled part of every hover window [mm]")
    summ = {}
    for st in ("none", "payload"):
        summ[st] = {}
        for k in ("com", "ee"):
            E = np.vstack(pool[st][k]); s = stats(E)
            # scatter about each window's own mean (wander), and the spread of the window means (offset)
            dev = np.vstack([e[:, :2] - e[:, :2].mean(0) for e in pool[st][k]])
            means = np.array([e[:, :2].mean(0) for e in pool[st][k]])
            s["wander"] = float(1e3 * np.sqrt((dev ** 2).sum(1).mean()))
            s["offset_rms"] = float(1e3 * np.sqrt((means ** 2).sum(1).mean())); s["offset_max"] = float(1e3 * np.linalg.norm(means, axis=1).max())
            s["seconds"] = float(len(E) * 0.02); s["windows"] = len(pool[st][k])
            summ[st][k] = s
            print(f"  {st:8s} {k:4s} within 20/50/70 mm: {100 * s['within20']:.0f} / {100 * s['within50']:.0f} / {100 * s['within70']:.0f} %")
            print(f"  {st:8s} {k:4s} {s['seconds']:6.1f} s in {s['windows']:2d} windows: offset rms {s['offset_rms']:5.1f} (max {s['offset_max']:5.1f}), "
                  f"wander {s['wander']:5.1f}, total rms {s['rms']:5.1f}, p95 {s['p95']:5.1f}, max {s['max']:5.1f}; z {s['z_mean']:+5.1f} +- {s['z_std']:4.1f} (max {s['z_max']:.0f})")
    json.dump(dict(windows=res, pooled=summ, settle_s=SETTLE), open(os.path.join(OUT, "hover.json"), "w"), indent=1)
    json.dump(dict(pts=pts, pooled={st: summ[st]["ee"] for st in summ}), open(os.path.join(OUT, "fig_hover.json"), "w"))

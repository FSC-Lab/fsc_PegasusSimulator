#!/usr/bin/env python3
"""Hover wander across every whole-body flight 10-20 .. 11-03, split at the 10-29 evening sidewall.
Per flight:
  MID  -- free hovers with the arm at HOME and a still reference (start mark, Land Start, post-abort holds; >= 3 s, the
          first 1.5 s dropped): the law's CoM error x_c - x_cd about its mean (rms per world axis), the EKF2 horizontal
          speed, and the slow (0.05-1 Hz) variation of the observer's translational disturbance estimate (a wind proxy).
  WORK -- the hold above the handle (Go To Start done + 0.5 s -> Ready / abort): the base speed over the planner's 0.3 s
          window, the CoM error, and the geometry: in the planner's table frame, the base's height above the table top,
          the rotor plane's (push_pull_guard_rotor z 0.1822) height, the body's distance from the table's near edge, and
          the share of rotor-disc area over the table footprint.
Usage: history.py   (reads the npz of every flight archive; edit FLIGHTS)"""
import os, re, glob, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.join(HERE, "..", "..")
import importlib.util
_spec = importlib.util.spec_from_file_location("bpc", os.path.join(ARCH, "wb_box_pushing_flight_20261021", "tools", "bp_common.py"))
_c = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_c)
ROTOR_XY, ROTOR_Z, ROTOR_R, GEAR = 0.2299, 0.1822, 0.1905, 0.248

def find_npz(ts):
    for d in sorted(glob.glob(os.path.join(ARCH, "*", "npz"))):
        p = os.path.join(d, f"b{ts}.npz")
        if os.path.exists(p): return p
    return None

def R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

def wb(b):
    t = b.t("wb"); W = b.f("wb", "data")
    ok = np.array([len(r) >= 57 and r[0] > 0.5 for r in W])
    return t[ok], np.array([np.asarray(r[:57], float) for r, k in zip(W, ok) if k])

def static_segments(t, D, min_len=3.0):
    xcd = D[:, 45:48]; dt = np.median(np.diff(t)); k = max(1, int(0.5 / dt))
    still = np.zeros(len(t), bool); still[k:] = np.linalg.norm(xcd[k:] - xcd[:-k], axis=1) < 0.001
    segs, i = [], 0
    while i < len(t):
        if still[i]:
            j = i
            while j + 1 < len(t) and still[j + 1] and t[j + 1] - t[j] < 0.2: j += 1
            if t[j] - t[i] >= min_len: segs.append((t[i] + 1.5, t[j]))
            i = j + 1
        else: i += 1
    return segs

def band(t, x, lo=0.1, hi=1.0):
    if t[-1] - t[0] < 2.5: return np.nan
    tt = np.arange(t[0], t[-1], 0.02); y = np.interp(tt, t, x); y = y - y.mean()
    F = np.fft.rfft(y); f = np.fft.rfftfreq(len(y), 0.02); F[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(F, len(y)).std()

def phases(b):
    ed = b.edges("pl_pstatus") if b.has("pl_pstatus") else []
    def first(pat, after=0.0):
        for t, s in ed:
            if t >= after and re.search(pat, s): return t
        return None
    g = first(r"^DONE go_to_start")
    nxt = None
    if g is not None:
        cand = [x for x in (first(r"^FLYING ready", g), first(r"^ABORTING", g), first(r"^FLYING go_to_land", g), first(r"^NOT IN DIRECT", g)) if x]
        nxt = min(cand) if cand else None
    return g, nxt

def analyse(ts, task):
    p = find_npz(ts)
    if p is None: return None
    _c.NPZ = os.path.dirname(p); b = _c.Bag(ts)
    t, D = wb(b)
    if len(D) < 100: return None
    e = D[:, 48:51] - D[:, 45:48]; q = np.degrees(D[:, 5:9]); dhat = D[:, 31:34]
    tp = b.t("vodom_px"); vp = np.stack(b.f("vodom_px", "velocity")).astype(float)
    out = {"ts": ts, "task": task}
    # MID: home-pose static holds
    E, S, Dh, dur = [], [], [], 0.0
    for a, c in static_segments(t, D):
        w = (t >= a) & (t <= c)
        if w.sum() < 50: continue
        qm = q[w].mean(0)
        if not (abs(qm[1] - 40) < 5 and abs(qm[2] - 40) < 5): continue
        z = D[w, 47].mean()
        if z < 0.6: continue          # not the landing hover just above the floor
        E.append(e[w, :2] - e[w, :2].mean(0)); dur += c - a
        wp = (tp >= a) & (tp <= c); S.append(np.linalg.norm(vp[wp, :2], axis=1))
        Dh.append([band(t[w], dhat[w, 0]), band(t[w], dhat[w, 1])])
    if E:
        E = np.vstack(E); S = np.concatenate(S); Dh = np.array(Dh)
        out.update(mid_s=dur, mid_rms=E.std(0) * 1e3, mid_v50=np.median(S) * 1e3, mid_v90=np.percentile(S, 90) * 1e3,
                   mid_dh=np.nanmean(Dh, 0))
    # WORK: the hold above the handle
    g, nxt = phases(b)
    if g is not None and b.has("pl_current_base"):
        a, c = g + 0.5, (nxt or g + 10)
        tb = b.t("pl_current_base")
        pb = np.stack([b.f("pl_current_base", f"pose.position.{k}") for k in "xyz"], 1)
        qb = np.stack([b.f("pl_current_base", f"pose.orientation.{k}") for k in "wxyz"], 1)
        j = np.searchsorted(tb, tb - 0.3); okv = (tb - tb[np.clip(j, 0, len(tb) - 1)]) >= 0.25
        vb = np.where(okv, np.linalg.norm(pb - pb[j], axis=1) / np.maximum(tb - tb[j], 1e-3), np.nan)
        w = (tb >= a) & (tb <= c) & okv
        if w.sum() >= 3:
            out.update(work_s=c - a, work_v50=np.nanmedian(vb[w]) * 1e3, work_v90=np.nanpercentile(vb[w], 90) * 1e3)
            w1 = (tb >= g + 0.3) & (tb <= g + 1.3) & okv          # the same 1 s for every flight
            if w1.sum() >= 3: out.update(w1_v50=np.nanmedian(vb[w1]) * 1e3, w1_v90=np.nanpercentile(vb[w1], 90) * 1e3)
            ww = (t >= a) & (t <= c)
            if ww.sum() > 20:
                ee = e[ww, :2]; out["work_rms"] = (ee - ee.mean(0)).std(0) * 1e3
            # geometry vs the planner's table
            if b.has("pl_info"):
                I = [np.asarray(r, float) for r in b.f("pl_info", "data")]
                I = [r for r in I if len(r) > 86 and r[77] > 0.5 and np.isfinite(r[86])]
                if I:
                    r = I[-1]; ox, oy, yaw = r[79], r[80], np.radians(r[81]); x0, x1, y0, y1, ztop = r[82:87]
                    c_, s_ = np.cos(yaw), np.sin(yaw)
                    pm = pb[w].mean(0); qm = qb[w][len(qb[w]) // 2]
                    Rb = R(*qm)
                    loc = lambda P: np.array([c_ * (P[0] - ox) + s_ * (P[1] - oy), -s_ * (P[0] - ox) + c_ * (P[1] - oy)])
                    L = loc(pm)
                    # distance of the body centre from the footprint (0 = above it)
                    dx = max(x0 - L[0], 0, L[0] - x1); dy = max(y0 - L[1], 0, L[1] - y1)
                    # rotor discs over the footprint (Monte-Carlo on a disc grid)
                    g_ = np.linspace(-ROTOR_R, ROTOR_R, 25); GX, GY = np.meshgrid(g_, g_); disc = GX ** 2 + GY ** 2 <= ROTOR_R ** 2
                    over = 0; tot = 0
                    for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
                        cen = pm + Rb @ np.array([sx * ROTOR_XY, sy * ROTOR_XY, ROTOR_Z])
                        P = np.stack([cen[0] + GX[disc], cen[1] + GY[disc]], 1)
                        Lp = np.array([loc(pp) for pp in P])
                        over += np.sum((Lp[:, 0] >= x0) & (Lp[:, 0] <= x1) & (Lp[:, 1] >= y0) & (Lp[:, 1] <= y1)); tot += len(P)
                    out.update(z_top=ztop, base_above=pm[2] - ztop, rotor_above=pm[2] + ROTOR_Z - ztop,
                               gear_above=pm[2] - GEAR - ztop, body_out=np.hypot(dx, dy), rotor_over=over / tot * 100,
                               table_len=max(x1 - x0, y1 - y0), table_dep=min(x1 - x0, y1 - y0), base_xy=pm[:2])
    return out

FLIGHTS = [  # (timestamp, task)  -- the sidewall went up on 10-29 evening (after 15:30)
    ("182234", "PnD 10-20"),
    ("161745", "box 10-21"), ("164338", "box 10-21"), ("165311", "box 10-21"), ("170123", "box 10-21"),
    ("172853", "box 10-23"),
    ("150423", "box 10-24"), ("151159", "box 10-24"), ("151634", "box 10-24"), ("162850", "box 10-24"), ("163413", "box 10-24"),
    ("164419", "box 10-24"), ("194310", "box 10-24e"), ("195657", "box 10-24e"),
    ("171114", "box 10-27"), ("172227", "box 10-27"),
    ("175015", "trk 10-27"), ("175849", "trk 10-27"), ("180659", "trk 10-27"),
    ("144602", "trk 10-28"), ("150821", "trk 10-28"), ("152202", "trk 10-28"),
    ("161640", "PnD 10-28"), ("162711", "PnD 10-28"), ("164816", "PnD 10-28"), ("165603", "PnD 10-28"),
    ("180652", "valve 10-28"), ("181833", "valve 10-28"),
    ("145128", "valve 10-29"), ("145526", "valve 10-29"), ("150625", "trk 10-29"),
    ("--- sidewall ---", ""),
    ("153032", "mail 10-31"), ("161915", "mail 10-31"), ("163410", "mail 10-31"), ("170629", "mail 10-31"),
    ("174943", "valve 10-31"), ("175725", "valve 10-31"), ("180423", "valve 10-31"), ("211808", "valve 10-31"),
    ("173835", "valve 11-01"), ("174531", "valve 11-01"), ("175259", "valve 11-01"), ("175932", "valve 11-01"),
    ("164233", "valve 11-03"), ("164722", "valve 11-03"), ("165100", "valve 11-03"), ("165700", "valve 11-03"),
]
if __name__ == "__main__":
    f = lambda v, n=0: "   --" if v is None or (isinstance(v, float) and not np.isfinite(v)) else f"{v:5.{n}f}"
    print(f"{'flight':18s} | MID home-pose hover: s   CoM rms x/y mm  EKF2 v p50/p90  dhat 0.05-1Hz x/y N | WORK above handle: s  base v p50/p90  CoM rms x/y | base/rotor/gear above top, body from table, rotor over %")
    rows = []
    for ts, task in FLIGHTS:
        if ts.startswith("---"): print("-" * 60, "sidewall added 10-29 evening", "-" * 60); continue
        try: r = analyse(ts, task)
        except Exception as ex: print(f"{task:11s} {ts}  ERROR {ex}"); continue
        if r is None: print(f"{task:11s} {ts}  (no npz / no DIRECT)"); continue
        rows.append(r)
        s = f"{task:11s} {ts} | "
        if "mid_s" in r:
            s += f"{r['mid_s']:5.1f}  {r['mid_rms'][0]:5.1f}/{r['mid_rms'][1]:5.1f}  {r['mid_v50']:4.0f}/{r['mid_v90']:4.0f}  {r['mid_dh'][0]:5.2f}/{r['mid_dh'][1]:5.2f}"
        else: s += " " * 45
        s += " | "
        if "work_s" in r:
            wr = r.get("work_rms", [np.nan, np.nan])
            s += f"{r['work_s']:5.1f} {r['work_v50']:4.0f}/{r['work_v90']:4.0f} [1s {r.get('w1_v50', np.nan):4.0f}/{r.get('w1_v90', np.nan):4.0f}]  {wr[0]:5.1f}/{wr[1]:5.1f}"
            if "base_above" in r:
                s += f" | {r['base_above']:5.3f}/{r['rotor_above']:5.3f}/{r['gear_above']:5.3f}  {r['body_out']:5.3f}  {r['rotor_over']:4.0f}%  (table {r['table_len']:.2f}x{r['table_dep']:.2f}, top {r['z_top']:.3f}, base {r['base_xy'][0]:+.2f},{r['base_xy'][1]:+.2f})"
        print(s)

#!/usr/bin/env python3
"""Claw hold precision while the planner's reference stands still -- T650-AM hardware flights 2026-10-09..21.

window      the whole-body planner in HOLD (no leg in flight; the leg end = the HOLD edge) with a static streamed
            reference (|d r_ed/dt|, |d x_cd/dt| < 5 mm/s, heading rates < 0.5 deg/s), inside DIRECT; ended at the
            next FLYING / ABORTING status edge (the next reference change) and cut at every gripper motion (a window
            ends 0.1 s before the motion, the next one starts 1.0 s after it).
settle      'settled' = from 3 s after the reference became static (the spec); 's1' = from 1 s after; 'full' = from the
            static start. The gripper holds of these missions last 0.4-3 s, so they are mostly 'full' only.
types       approach_wait (claw held at the point behind / above the pick (or the place, set-down mode), before the
            Pick / Place press), at_pick (claw at the pick point, jaws open, until the close), at_drop (claw at the
            drop / release point, until the open), above_handle (box pushing: above the fin, CoM-anchored), at_handle
            (box pushing: at the fin, WORLD-held, jaws open, until the close), other (any other static hover >= 1.5 s).
            Kept apart: direct_entry (hover from the SAFETY->DIRECT switch; the observer restarts at zero), post_abort,
            bin (1014 PD-1 over the full bin: aero-disturbed), near_ground (body below 0.5 m), excluded (claw in contact).
error       claw (grasp point, the planner's FK on the encoders) minus the streamed reference r_ed, in the nose-heading
            frame: along-nose (+ = claw ahead of the target), lateral (+ = left), vertical (+ = above).
estimates   M = /uav_0/mocap pose (physical body pose) + encoders; C = odometry position + PX4 attitude + encoders
            (exactly the planner's arrival_error and the law's view).
load        loaded = basket in the jaws (close at the pick -> open at the drop / place) on the basket flights.
sets        main = the flights asked for (10-14..10-21); supp = the 10-09/10-10 set-down pick-and-place flights (long holds).
Writes ../stats.json, ../windows.csv, ../events.csv and prints the per-window table.
"""
import json, csv
import numpy as np
from hover_common import FLIGHTS, OUT, build, interp

SETTLE, EV_BEFORE, EV_AFTER, MIN_SPAN, MIN_SETTLED = 3.0, 0.1, 1.0, 1.5, 0.5
GRIP_TYPES = ("approach_wait", "at_pick", "at_drop", "above_handle", "at_handle")
APART = ("bin", "direct_entry", "post_abort", "near_ground", "post_lift")
LOADED_FLIGHTS = {k for k, v in FLIGHTS.items() if v[3]}
SETTLES = (("full", 0.0), ("s1", 1.0), ("settled", SETTLE))
DT = 0.02


def spans(mask, t, min_len=0.2):
    out, i, n = [], 0, len(mask)
    while i < n:
        if not mask[i]:
            i += 1; continue
        j = i
        while j < n and mask[j]:
            j += 1
        if t[j - 1] - t[i] >= min_len:
            out.append((t[i], t[j - 1]))
        i = j
    return out


def dominant_phase(phs, a, b):
    best, lab = -1.0, ""
    for pa, pb, l in phs:
        ov = min(b, pb) - max(a, pa)
        if ov > best:
            best, lab = ov, l
    return lab


def in_any(x, ivs):
    return any(a <= x <= b for a, b in ivs)


def classify(fid, S, a, b, after_event):
    ph = dominant_phase(S["phases"], a, b)
    grip = S["grip"]; tm = 0.5 * (a + b)
    ends_close = any(abs(te - EV_BEFORE - b) < 0.05 and k == "close" for te, k in grip)
    ends_open = any(abs(te - EV_BEFORE - b) < 0.05 and k == "open" for te, k in grip)
    if in_any(tm, S["excl"]):
        return "excluded", ph
    if fid == "pd14a" and (ph.startswith("DONE execute_place") or ph.startswith("ABORT")):
        return "bin", ph
    if ph.startswith(("COMPLETE", "NOT IN DIRECT")) or np.median(S["z_body"][(S["t"] >= a) & (S["t"] <= b)]) < 0.5:
        return "near_ground", ph
    if ph.startswith("ABORT"):
        return "post_abort", ph
    if a < S["a"] + 1.0:
        return "direct_entry", ph          # the hover from the SAFETY->DIRECT switch (observer restarts at zero)
    if ph.startswith(("WAITING execute_pick", "WAITING execute_place")):
        return "approach_wait", ph
    if ph.startswith("DONE execute_pick"):
        if not after_event and ends_close:
            return "at_pick", ph
        if after_event and fid in LOADED_FLIGHTS:
            return "excluded", ph          # basket grasped but still standing on its pillar
        return "other", ph
    if ph.startswith("DONE exit_pick") and load_state(fid, S, tm) == "loaded":
        return "post_lift", ph             # the hover right after the load step (lift lurch), kept apart
    if ph.startswith("DONE execute_place") and not after_event:
        return ("at_drop" if ends_open else "other"), ph
    if ph.startswith("DONE go_to_start") and S["status_key"] == "pl_pstatus":
        return "above_handle", ph
    if ph.startswith("DONE ready"):
        return ("at_handle" if (not after_event and ends_close) else "excluded"), ph
    return "other", ph


def load_state(fid, S, tm):
    if "loaded" not in S:
        return "unloaded"
    a, b = S["loaded"]
    return "loaded" if a <= tm <= b else "unloaded"


def comp_stats(x):
    x = np.asarray(x, float); ax = np.abs(x)
    if len(x) == 0:
        return None
    return dict(n=int(len(x)), mean=float(x.mean()), std=float(x.std()), rms=float(np.sqrt((x ** 2).mean())),
                p01=float(np.percentile(x, 1)), p05=float(np.percentile(x, 5)), p95=float(np.percentile(x, 95)),
                p99=float(np.percentile(x, 99)), p95_abs=float(np.percentile(ax, 95)), p99_abs=float(np.percentile(ax, 99)),
                max_abs=float(ax.max()))


def freq_of(x, dt):
    """Detrended series -> (periodogram peak in 0.05-2 Hz, zero-crossing mean frequency, freqs, psd)."""
    n = len(x); tt = np.arange(n) * dt
    y = x - np.polyval(np.polyfit(tt, x, 1), tt)
    w = np.hanning(n); nf = 1 << 14
    P = np.abs(np.fft.rfft(y * w, nf)) ** 2 / (w ** 2).sum() * dt
    f = np.fft.rfftfreq(nf, dt)
    band = (f >= 0.05) & (f <= 2.0)
    zc = np.sum(np.diff(np.sign(y)) != 0) / 2.0 / (n * dt)
    return float(f[band][np.argmax(P[band])]), float(zc), f, P


def block_stats(blocks):
    """blocks: list of dicts with eh (n x 3, m), head (deg), tilt (deg), dur. Pooled sample + window decomposition."""
    if not blocks:
        return None
    E = np.vstack([b["eh"] for b in blocks]) * 1e3
    r = dict(n_windows=len(blocks), seconds=float(len(E) * DT), flights=sorted({b["fid"] for b in blocks}))
    for i, k in enumerate(("along", "lateral", "vertical")):
        r[k] = comp_stats(E[:, i])
        means = np.array([b["eh"][:, i].mean() * 1e3 for b in blocks])
        within = np.concatenate([b["eh"][:, i] * 1e3 - b["eh"][:, i].mean() * 1e3 for b in blocks])
        r[k]["window_mean_rms"] = float(np.sqrt((means ** 2).mean()))
        r[k]["window_mean_min"] = float(means.min()); r[k]["window_mean_max"] = float(means.max())
        r[k]["within_window_std"] = float(np.sqrt((within ** 2).mean()))
    r["horizontal"] = comp_stats(np.linalg.norm(E[:, :2], axis=1))
    r["norm3d"] = comp_stats(np.linalg.norm(E, axis=1))
    r["heading_deg"] = comp_stats(np.concatenate([b["head"] for b in blocks]))
    r["tilt_deg"] = comp_stats(np.concatenate([b["tilt"] for b in blocks]))
    for thr in (10, 20, 30):
        r[f"frac_within_{thr}mm"] = float((np.linalg.norm(E, axis=1) <= thr).mean())
    long = [b for b in blocks if b["dur"] >= 6.0]
    fr = dict(n_windows=len(long), seconds=float(sum(b["dur"] for b in long)))
    if long:
        fgrid = np.linspace(0.02, 2.0, 397)
        for i, k in enumerate(("along", "lateral", "vertical")):
            pk, zc, acc, wsum = [], [], np.zeros_like(fgrid), 0.0
            for b in long:
                p, z, f, P = freq_of(b["eh"][:, i] * 1e3, DT)
                pk.append(p); zc.append(z)
                acc += np.interp(fgrid, f, P) * b["dur"]; wsum += b["dur"]
            acc /= wsum
            df = fgrid[1] - fgrid[0]; tot = acc.sum() * df
            bands = {"<0.2Hz": (0, 0.2), "0.2-0.5Hz": (0.2, 0.5), "0.5-1Hz": (0.5, 1.0), ">1Hz": (1.0, 2.01)}
            fr[k] = dict(peak_of_mean_psd_hz=float(fgrid[np.argmax(acc)]), median_window_peak_hz=float(np.median(pk)),
                         window_peaks_hz=[round(x, 3) for x in pk], median_zero_crossing_hz=float(np.median(zc)),
                         band_fraction={n_: float(acc[(fgrid >= lo) & (fgrid < hi)].sum() * df / tot) for n_, (lo, hi) in bands.items()})
    r["frequency"] = fr
    return r


CURVE_T = (0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0)


def arrival(t, e3, a, b):
    """Time from the static start a until |e| <= thr (3-D, mm): first entry followed by a 1 s dwell inside, and the
    time after which it stays inside to the end of the window (None = not reached before the window ended)."""
    m = (t >= a) & (t <= b); tt = t[m] - a; ee = e3[m]
    out = dict(window_s=float(tt[-1]) if len(tt) else 0.0, e0_mm=float(ee[0]) if len(ee) else None,
               e_end_mm=float(ee[-1]) if len(ee) else None, max_mm=float(ee.max()) if len(ee) else None)
    for thr in (10, 20, 30):
        ok = ee <= thr
        dwell = None
        for i in range(len(tt)):
            if not ok[i]:
                continue
            if tt[-1] - tt[i] < 1.0 - 1e-6:
                break
            j = np.searchsorted(tt, tt[i] + 1.0, side="right")
            if ok[i:j].all():
                dwell = float(tt[i]); break
        bad = np.where(~ok)[0]
        stay = 0.0 if len(bad) == 0 else (float(tt[bad[-1] + 1]) if bad[-1] + 1 < len(tt) else None)
        out[f"dwell1s_{thr}"] = dwell; out[f"stay_{thr}"] = stay
    out["curve"] = {str(c): (float(np.interp(c, tt, ee)) if len(tt) and tt[-1] >= c else None) for c in CURVE_T}
    return out


def analyse(fid, blocks, rows, events, arrivals):
    d, t0, S = build(fid)
    t = S["t"]; fset = FLIGHTS[fid][5]
    S["status_key"] = "pp_status" if "pp_status__recv" in d.files else "pl_pstatus"
    S["excl"] = list(FLIGHTS[fid][6])
    grip = S["grip"]
    if fid in LOADED_FLIGHTS:
        tp = next(a for a, b, l in S["phases"] if l.startswith("DONE execute_pick"))
        tc = next(te for te, k in grip if k == "close" and te > tp - 0.5)
        td = next(a for a, b, l in S["phases"] if l.startswith("DONE execute_place"))
        to_ = next(te for te, k in grip if k == "open" and te > td - 0.5)
        S["loaded"] = (tc, to_)
    if fid == "bp21":
        tr = next(a for a, b, l in S["phases"] if l.startswith("DONE ready"))
        tc = next(te for te, k in grip if k == "close" and te > tr - 0.5)
        to_ = next(te for te, k in grip if k == "open" and te > tc)
        S["excl"].append((tc, to_ + EV_AFTER))
    static = S["hold"] & S["ref_ok"] & S["q_ok"] & (S["v_r_ed"] < 0.005) & (S["v_x_cd"] < 0.005) & (S["w_ref"] < 0.5)
    print(f"== {fid} {FLIGHTS[fid][0]} [{fset}]  DIRECT {S['a']:.1f}-{S['b']:.1f}  mocap {S['mocap_stats']}  grip {[(round(x, 2), k) for x, k in grip]}"
          + (f"  loaded {S['loaded'][0]:.2f}-{S['loaded'][1]:.2f}" if 'loaded' in S else "") + f"  excluded {[(round(a, 2), round(b, 2)) for a, b in S['excl']]}")
    moves = [a_ for a_, b_, l_ in S["phases"] if l_.startswith(("FLYING", "ABORTING"))]
    for sa, sb in spans(static, t):
        nxt = [m_ for m_ in moves if sa + 0.2 < m_ < sb]
        if nxt:
            sb = nxt[0]
        cuts = [(sa, False)] + [(te, True) for te, k in grip if sa < te < sb]
        segs = []
        for i, (ca, ev) in enumerate(cuts):
            cb = cuts[i + 1][0] - EV_BEFORE if i + 1 < len(cuts) else sb
            ca2 = ca + EV_AFTER if ev else ca
            if cb - ca2 >= 0.15:
                segs.append((ca2, cb, ev))
        for ca, cb, ev in segs:
            typ, ph = classify(fid, S, ca, cb, ev)
            if typ not in GRIP_TYPES and typ != "excluded" and sb - sa < MIN_SPAN:
                continue                       # an "other settled hover" = a static reference for >= 1.5 s
            load = "contact" if typ == "excluded" else load_state(fid, S, 0.5 * (ca + cb))
            row = dict(fid=fid, flight=FLIGHTS[fid][0], set=fset, stack=FLIGHTS[fid][2], type=typ, phase=ph, load=load,
                       static_start=round(float(sa), 2), start=round(float(ca), 2), end=round(float(cb), 2),
                       after_gripper=bool(ev), dur=round(float(cb - ca), 2))
            for est in ("M", "C"):
                okm = S[f"{est}_ok"]
                for kind, skip in SETTLES:
                    lo = max(ca, sa + skip)
                    m = (t >= lo) & (t <= cb) & okm
                    if (cb - lo) < (0.15 if kind == "full" else MIN_SETTLED) or m.sum() < 5:
                        continue
                    blk = dict(eh=S[f"eh_{est}"][m], head=S[f"head_{est}"][m], tilt=S[f"tilt_{est}"][m],
                               dur=float(cb - lo), fid=fid)
                    blocks.setdefault((fset, est, kind, typ, load), []).append(blk)
                    blocks.setdefault((fset, est, kind, typ, load, fid), []).append(blk)
                    if est == "M":
                        e = blk["eh"] * 1e3
                        row[f"{kind}_s"] = round(float(cb - lo), 2)
                        row[f"{kind}_mean_along_lat_vert_mm"] = e.mean(0).round(1).tolist()
                        row[f"{kind}_std_along_lat_vert_mm"] = e.std(0).round(1).tolist()
                        row[f"{kind}_max3d_mm"] = round(float(np.linalg.norm(e, axis=1).max()), 1)
            m = (t >= ca) & (t <= cb)
            row["law_task_err_mean_mm"] = (S["ey_law"][m].mean(0) * 1e3).round(1).tolist()
            row["law_task_err_std_mm"] = (S["ey_law"][m].std(0) * 1e3).round(1).tolist()
            row["com_err_C_mean_mm"] = (S["ecom_C"][m].mean(0) * 1e3).round(1).tolist()
            row["com_err_C_std_mm"] = (S["ecom_C"][m].std(0) * 1e3).round(1).tolist()
            # arrival: windows that start at the leg end (a HOLD edge reached by motion)
            if not ev:
                i0 = np.searchsorted(t, sa)
                moved = i0 > 5 and (S["v_r_ed"][max(0, i0 - 100):i0].max() > 0.02 or S["v_x_cd"][max(0, i0 - 100):i0].max() > 0.02)
                if moved:
                    for est in ("M", "C"):
                        e3 = np.linalg.norm(S[f"eh_{est}"], axis=1) * 1e3
                        A = arrival(t, e3, sa, cb)
                        A.update(est=est, fid=fid, set=fset, type=typ, load=load, phase=ph, t_leg_end=round(float(sa), 2))
                        arrivals.append(A)
            rows.append(row)
            f_ = row.get("full_mean_along_lat_vert_mm"); s_ = row.get("full_std_along_lat_vert_mm")
            print(f"   {ca:7.2f}-{cb:7.2f} ({cb - ca:5.2f} s, from {sa:7.2f}) {typ:13s} {load:8s} {ph[:26]:26s} full mean {f_} std {s_}"
                  + (f" | settled {row['settled_s']:.1f}s mean {row['settled_mean_along_lat_vert_mm']} std {row['settled_std_along_lat_vert_mm']}" if 'settled_s' in row else ""))
    # the error at each gripper command (motion start - 0.25..0.05 s), wherever the claw was holding a target
    for te, k in grip:
        if not (S["a"] + 0.5 < te < S["b"]):
            continue
        ph = dominant_phase(S["phases"], te - 0.3, te)
        if in_any(te, S["excl"][:len(FLIGHTS[fid][6])]):
            continue
        if not ph.startswith(("DONE execute_pick", "DONE execute_place", "DONE ready", "DONE push", "ABORTING")):
            continue
        typ = {"DONE execute_pick": "close_at_pick", "DONE execute_place": "open_at_drop", "DONE ready": "close_at_handle",
               "DONE push": "open_after_push", "ABORTING": "open_at_abort"}[next(p for p in ("DONE execute_pick", "DONE execute_place", "DONE ready", "DONE push", "ABORTING") if ph.startswith(p))]
        if fid == "pd14a" and typ in ("open_at_abort", "open_at_drop"):
            typ += "_bin"
        hold = te - max([a_ for a_, b_, l_ in S["phases"] if l_ == ph and a_ <= te] or [te])
        for est in ("M", "C"):
            m = (t >= te - 0.25) & (t <= te - 0.05) & S[f"{est}_ok"]
            if m.sum():
                e = S[f"eh_{est}"][m].mean(0) * 1e3
                events.append(dict(fid=fid, flight=FLIGHTS[fid][0], set=fset, event=typ, gripper=k, est=est,
                                   load=load_state(fid, S, te - 0.2), t=round(float(te), 2), hold_s=round(float(hold), 2),
                                   along=round(float(e[0]), 1), lateral=round(float(e[1]), 1), vertical=round(float(e[2]), 1),
                                   norm3d=round(float(np.linalg.norm(e)), 1), head_deg=round(float(S[f"head_{est}"][m].mean()), 2),
                                   tilt_deg=round(float(S[f"tilt_{est}"][m].mean()), 2)))
    return d, t0, S, static


def cross_checks(fid, d, t0, S, static):
    t = S["t"]; ck = {}
    key = "pp_arr" if "pp_arr__recv" in d.files else ("pl_arr" if "pl_arr__recv" in d.files else None)
    if key:
        ta = d[f"{key}__recv"] - t0; Ar = d[f"{key}__data"]
        m = (Ar[:, 1] == 1) & (ta > S["a"] + 0.2) & (ta < S["b"] - 0.2)
        eC = interp(ta, t, S["e_C"]); rr = interp(ta, t, S["r_ed"]); want = eC + rr + Ar[:, 3:6]
        same = m & (np.linalg.norm(want - rr, axis=1) < 0.002)
        if same.sum():
            diff = (-Ar[same][:, 3:6] - eC[same]) * 1e3
            ck["arrival_vs_C"] = dict(n=int(same.sum()), mean_mm=diff.mean(0).round(2).tolist(),
                                     rms_mm=np.sqrt((diff ** 2).mean(0)).round(2).tolist(), max_mm=round(float(np.abs(diff).max()), 2))
    st = static & S["M_ok"]
    dm = (S["eh_M"][st] - S["eh_C"][st]) * 1e3
    ck["M_minus_C_in_holds"] = dict(mean_mm=dm.mean(0).round(1).tolist(), std_mm=dm.std(0).round(1).tolist(),
                                    tilt_M_minus_C_deg=round(float((S["tilt_M"][st] - S["tilt_C"][st]).mean()), 2),
                                    tilt_M_deg=round(float(S["tilt_M"][st].mean()), 2), tilt_C_deg=round(float(S["tilt_C"][st].mean()), 2))
    de = (S["ey_law"] - (S["eh_C"] - S["ecom_C"]))[st] * 1e3
    ck["law_task_minus_C_claw_minus_com"] = dict(mean_mm=de.mean(0).round(2).tolist(), std_mm=de.std(0).round(2).tolist())
    return ck


def summarise_arrivals(arrivals, fset, est, load, types):
    rows = [a for a in arrivals if a["set"] == fset and a["est"] == est and a["load"] == load and a["type"] in types]
    if not rows:
        return None
    s = dict(n=len(rows), window_s_median=float(np.median([r["window_s"] for r in rows])),
             e_at_leg_end_median_mm=float(np.median([r["e0_mm"] for r in rows])))
    curve = {}
    for c in CURVE_T:
        v = [r["curve"][str(c)] for r in rows if r["curve"][str(c)] is not None]
        if v:
            curve[str(c)] = dict(n=len(v), median=float(np.median(v)), p90=float(np.percentile(v, 90)), max=float(max(v)),
                                 frac_le_10=float(np.mean(np.array(v) <= 10)), frac_le_20=float(np.mean(np.array(v) <= 20)),
                                 frac_le_30=float(np.mean(np.array(v) <= 30)))
    s["error_vs_time_after_leg_end"] = curve
    for thr in (10, 20, 30):
        for kind in ("dwell1s", "stay"):
            v = [r[f"{kind}_{thr}"] for r in rows]
            got = [x for x in v if x is not None]
            s[f"{kind}_{thr}"] = dict(reached=len(got), not_reached=len(v) - len(got),
                                      median=float(np.median(got)) if got else None,
                                      p90=float(np.percentile(got, 90)) if got else None, max=float(max(got)) if got else None)
    return s


def main():
    blocks, rows, events, arrivals, checks = {}, [], [], [], {}
    for fid in FLIGHTS:
        d, t0, S, static = analyse(fid, blocks, rows, events, arrivals)
        checks[fid] = cross_checks(fid, d, t0, S, static)
        print(f"   checks {checks[fid]}")
    pooled, per_flight, arr_sum = {}, {}, {}
    for fset in ("main", "supp", "all"):
        sets = ("main", "supp") if fset == "all" else (fset,)
        for est in ("M", "C"):
            for kind, _ in SETTLES:
                for load in ("unloaded", "loaded"):
                    g = {t_: sum((blocks.get((s_, est, kind, t_, load), []) for s_ in sets), []) for t_ in GRIP_TYPES + ("other",) + APART}
                    g["gripper_holds"] = sum((g[t_] for t_ in GRIP_TYPES), [])
                    g["all_free"] = g["gripper_holds"] + g["other"]
                    for name, bl in g.items():
                        s = block_stats(bl)
                        if s:
                            pooled.setdefault(fset, {}).setdefault(est, {}).setdefault(kind, {}).setdefault(name, {})[load] = s
                for load in ("unloaded", "loaded"):
                    for grp, types in (("gripper_holds", GRIP_TYPES), ("other", ("other",)), ("all_free", GRIP_TYPES + ("other",))):
                        s = None
                        if fset != "all":
                            s = summarise_arrivals(arrivals, fset, est, load, types)
                        else:
                            rr = [dict(a, set="all") for a in arrivals]
                            s = summarise_arrivals(rr, "all", est, load, types)
                        if s:
                            arr_sum.setdefault(fset, {}).setdefault(est, {}).setdefault(load, {})[grp] = s
    for est in ("M", "C"):
        for fid in FLIGHTS:
            fset = FLIGHTS[fid][5]
            for kind, _ in SETTLES:
                for load in ("unloaded", "loaded"):
                    bl = sum((blocks.get((fset, est, kind, t_, load, fid), []) for t_ in GRIP_TYPES + ("other",)), [])
                    s = block_stats(bl)
                    if s:
                        per_flight.setdefault(est, {}).setdefault(fid, {}).setdefault(kind, {})[load] = s
    meta = dict(flights={k: dict(label=v[0], npz=v[1], stack=v[2], basket=v[3], note=v[4], set=v[5], excluded=v[6])
                         for k, v in FLIGHTS.items()},
                definitions=__doc__, settle_s=SETTLE, grid_dt=DT,
                units="mm for positions, deg for heading/tilt, s for times; percentiles p95_abs/p99_abs/max_abs are of |component|",
                keys="pooled[set][estimate][settle][window type][load] ; per_flight[estimate][flight][settle][load] (gripper holds + other); "
                     "arrival_summary[set][estimate][load][group]")
    json.dump(dict(meta=meta, pooled=pooled, per_flight=per_flight, arrival_summary=arr_sum, arrivals=arrivals,
                   events=events, windows=rows, cross_checks=checks), open(f"{OUT}/stats.json", "w"), indent=1)
    keys = ["fid", "flight", "set", "stack", "type", "load", "phase", "static_start", "start", "end", "dur", "after_gripper",
            "full_s", "full_mean_along_lat_vert_mm", "full_std_along_lat_vert_mm", "full_max3d_mm",
            "s1_s", "s1_mean_along_lat_vert_mm", "s1_std_along_lat_vert_mm", "s1_max3d_mm",
            "settled_s", "settled_mean_along_lat_vert_mm", "settled_std_along_lat_vert_mm", "settled_max3d_mm",
            "com_err_C_mean_mm", "com_err_C_std_mm", "law_task_err_mean_mm", "law_task_err_std_mm"]
    with open(f"{OUT}/windows.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore"); w.writeheader()
        for r in rows:
            w.writerow(r)
    with open(f"{OUT}/events.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(events[0].keys())); w.writeheader()
        for r in events:
            w.writerow(r)
    print("wrote", f"{OUT}/stats.json", f"{OUT}/windows.csv", f"{OUT}/events.csv")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Score fused_feedback_recorder.py output: the controller's feedback vs Isaac truth.

    /usr/bin/python3 fused_feedback_score.py fb_run.npz [fb_run2.npz ...]

Per run, per phase (GROUND = before lift-off, SAFETY = airborne in SAFETY,
DIRECT = whole-body DIRECT):

  timing   real-time factor (ground-truth messages / 250 Hz / wall time),
           feedback rate, fraction of feedback samples that repeat the previous
           one exactly, receive-minus-header latency, and the timesync offset
           the fused bridge converts PX4 stamps with
  EKF2     fraction of samples with each fusion flag true, and when all came up
  error    feedback minus ground truth, position [mm] and velocity [mm/s]:
           mean (bias), rms and max of the 3-D norm, per-axis mean. Aligned on
           the RECORDER'S receive clock, so it is the error the loop actually
           sees, delays included.
  lag      the delay tau that minimises rms(feedback(t) - truth(t - tau)) --
           the effective end-to-end delay of the feedback path
  jumps    feedback steps that exceed the truth's own step by > 20 mm in one
           sample (EKF2 resets surface here)

The same numbers are computed for the /mocap stream (the raw-mocap stack's
feedback, and the fused estimator's input), so a fused run carries its own
raw-path reference.

Ground truth is Pegasus's state/pose + state/twist_inertial, published every
physics step (4 ms of simulated time).
"""
import sys

import numpy as np

PHYS_DT = 1.0 / 250.0
AIRBORNE_Z = 0.45        # the vehicle rests at 0.305 m (body origin)
REST_Z = 0.35
JUMP_M = 0.020


def _interp(t_src, y_src, t):
    return np.column_stack([np.interp(t, t_src, y_src[:, k]) for k in range(y_src.shape[1])])


def _phase_masks(t, mode, z_truth_at_t):
    code = np.full(t.shape, 0.0)
    for tm, _, c in mode:
        code[t >= tm] = c
    # The vehicle SPAWNS above the floor (0.61 m) and drops to its 0.305 m rest,
    # so "airborne" only counts after it has first been seen resting.
    rest = z_truth_at_t < REST_Z
    t_rest = t[np.argmax(rest)] if rest.any() else t[-1] + 1.0
    air = (z_truth_at_t > AIRBORNE_Z) & (t >= t_rest)
    first_air = np.argmax(air) if air.any() else len(t)
    ground = np.zeros_like(air)
    ground[:first_air] = True
    return {"GROUND": ground,
            "SAFETY": air & (code == 0.0),
            "DIRECT": code == 1.0}


def _err_stats(e):
    n = np.linalg.norm(e, axis=1)
    return {"mean_axes": e.mean(axis=0), "rms": float(np.sqrt(np.mean(n ** 2))),
            "p95": float(np.percentile(n, 95)), "max": float(n.max())}


def _best_lag(t, y, t_tr, y_tr, mask, lags):
    best = (np.inf, 0.0)
    for tau in lags:
        ref = _interp(t_tr, y_tr, t[mask] - tau)
        r = float(np.sqrt(np.mean(np.sum((y[mask] - ref) ** 2, axis=1))))
        if r < best[0]:
            best = (r, tau)
    return best[1], best[0]


def _stream(name, t, pos, vel, t_tr, p_tr, t_tv, v_tr, masks, lines):
    p_ref = _interp(t_tr, p_tr, t)
    v_ref = _interp(t_tv, v_tr, t)
    ep, ev = pos - p_ref, vel - v_ref
    same = np.r_[False, np.all(np.diff(pos, axis=0) == 0.0, axis=1)]
    dstep = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    tstep = np.linalg.norm(np.diff(p_ref, axis=0), axis=1)
    jumps = np.where(dstep - tstep > JUMP_M)[0]
    lines.append(f"  [{name}] {len(t)} samples, {len(t) / (t[-1] - t[0]):.1f} Hz, "
                 f"exact repeats {100 * same.mean():.1f}%, jumps>{1000 * JUMP_M:.0f}mm: "
                 f"{len(jumps)}" + (f" at t={', '.join(f'{t[j + 1]:.2f}' for j in jumps[:6])}"
                                    if len(jumps) else ""))
    lags = np.arange(-0.02, 0.301, 0.002)
    out = {}
    for ph, m in masks.items():
        if m.sum() < 50:
            continue
        sp, sv = _err_stats(ep[m] * 1000), _err_stats(ev[m] * 1000)
        line = (f"    {ph:6s} n={m.sum():6d}  pos mm: rms {sp['rms']:6.2f} p95 {sp['p95']:6.2f} "
                f"max {sp['max']:6.2f} bias [{sp['mean_axes'][0]:+5.1f} {sp['mean_axes'][1]:+5.1f} "
                f"{sp['mean_axes'][2]:+5.1f}] | vel mm/s: rms {sv['rms']:6.1f} max {sv['max']:6.1f}")
        if ph != "GROUND":
            lag_p, r_p = _best_lag(t, pos, t_tr, p_tr, m, lags)
            lag_v, r_v = _best_lag(t, vel, t_tv, v_tr, m, lags)
            line += (f" | lag pos {1000 * lag_p:5.0f} ms (rms {1000 * r_p:5.2f}), "
                     f"vel {1000 * lag_v:5.0f} ms (rms {1000 * r_v:5.1f})")
            out[ph] = dict(pos=sp, vel=sv, lag_p=lag_p, lag_v=lag_v)
        lines.append(line)
    return out


def score(path):
    d = np.load(path)
    tp, tv, mc, od = d["truth_pose"], d["truth_vel"], d["mocap"], d["odom"]
    fl, ts, mode = d["flags"], d["timesync"], d["mode"]
    lines = [f"== {path}"]
    if len(tp) < 100 or len(od) < 100:
        lines.append("  not enough data")
        return "\n".join(lines), {}

    # --- timing ---------------------------------------------------------------
    span = tp[-1, 0] - tp[0, 0]
    rtf = len(tp) * PHYS_DT / span
    win = 10.0
    edges = np.arange(tp[0, 0], tp[-1, 0], win)
    rtf_w = np.array([np.sum((tp[:, 0] >= a) & (tp[:, 0] < a + win)) * PHYS_DT / win
                      for a in edges[:-1]]) if len(edges) > 2 else np.array([rtf])
    lines.append(f"  RTF {rtf:.3f} overall (10 s windows min {rtf_w.min():.3f} / "
                 f"median {np.median(rtf_w):.3f}); truth {len(tp)} msgs over {span:.1f} s")
    lat_od = (od[:, 1] - od[:, 2]) * 1000
    lat_mc = (mc[:, 1] - mc[:, 2]) * 1000
    lines.append(f"  receive - header stamp: odom median {np.median(lat_od):.1f} ms "
                 f"(p95 {np.percentile(lat_od, 95):.1f}), mocap median {np.median(lat_mc):.1f} ms "
                 f"(p95 {np.percentile(lat_mc, 95):.1f})")
    if len(ts):
        lines.append(f"  timesync: {len(ts)} msgs, estimated_offset median "
                     f"{np.median(ts[:, 3]) / 1e6:.3f} s (spread "
                     f"{(np.percentile(ts[:, 3], 95) - np.percentile(ts[:, 3], 5)) / 1e3:.2f} ms)")
    else:
        lines.append("  timesync: NO fmu/out/timesync_status received "
                     "(fused odom falls back to arrival-time stamps)")

    z_at = np.interp(od[:, 0], tp[:, 0], tp[:, 5])
    masks = _phase_masks(od[:, 0], mode, z_at)
    air_all = masks["SAFETY"] | masks["DIRECT"]
    t_lift = od[np.argmax(air_all), 0] if air_all.any() else np.nan
    t_dir = [m[0] for m in mode if m[2] == 1.0]
    lines.append(f"  lift-off t={t_lift:.1f} s, DIRECT entries at "
                 f"{', '.join(f'{x:.1f}' for x in t_dir) or 'none'}, "
                 f"DIRECT total {masks['DIRECT'].sum() / max(1, len(od)) * (od[-1, 0] - od[0, 0]):.1f} s")

    # --- EKF2 flags -------------------------------------------------------------
    if len(fl):
        names = ["yaw_align", "ev_pos", "ev_hgt", "ev_vel", "ev_yaw", "ev_yaw_FAULT"]
        z_fl = np.interp(fl[:, 0], tp[:, 0], tp[:, 5])
        air = (z_fl > AIRBORNE_Z) & (fl[:, 0] >= t_lift)
        allok = np.all(fl[:, 2:7] > 0.5, axis=1)
        t_ok = fl[np.argmax(allok), 0] if allok.any() else np.nan
        frac = ", ".join(f"{n} {100 * fl[air, 2 + k].mean():.1f}%"
                         for k, n in enumerate(names)) if air.any() else "never airborne"
        lines.append(f"  EKF2 flags airborne: {frac}; all five fusion flags first true "
                     f"at t={t_ok:.1f} s")

    # --- errors -------------------------------------------------------------------
    p_tr, v_tr = tp[:, 3:6], tv[:, 3:6]
    res = {"rtf": rtf}
    res["odom"] = _stream("odom = controller FEEDBACK", od[:, 0], od[:, 3:6], od[:, 6:9],
                          tp[:, 0], p_tr, tv[:, 0], v_tr, masks, lines)
    zm = np.interp(mc[:, 0], tp[:, 0], tp[:, 5])
    mm = _phase_masks(mc[:, 0], mode, zm)
    res["mocap"] = _stream("mocap = raw path / estimator input", mc[:, 0], mc[:, 3:6],
                           mc[:, 6:9], tp[:, 0], p_tr, tv[:, 0], v_tr, mm, lines)

    # --- header-stamp alignment (estimator accuracy with its latency removed) ---
    for ph in ("SAFETY", "DIRECT"):
        m = masks[ph]
        if m.sum() < 50:
            continue
        pr = _interp(tp[:, 2], p_tr, od[m, 2])
        e = np.linalg.norm(od[m, 3:6] - pr, axis=1) * 1000
        lines.append(f"  odom vs truth at the odom's OWN header stamp, {ph}: "
                     f"rms {np.sqrt(np.mean(e ** 2)):.2f} mm, max {e.max():.2f} mm")
    return "\n".join(lines), res


def main():
    for p in sys.argv[1:]:
        txt, _ = score(p)
        print(txt)
        print()


if __name__ == "__main__":
    main()

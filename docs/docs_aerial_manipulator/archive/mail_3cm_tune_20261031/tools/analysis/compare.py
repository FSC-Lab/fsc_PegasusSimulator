#!/usr/bin/env python3
"""The insert, hardware vs sim: one metric table + one overlay figure, aligned at the slide start.

    cd runs/work && /usr/bin/python3 ../../tools/analysis/compare.py hw3cm base_1 base_2 ... [--fig out.png]

Each tag needs {events,run,dbg}_<tag>.* in the working directory (process_bag.sh). The first tag is
the reference (the hardware 3cm flight). Every metric is the same code for every tag.
"""
import argparse
import json

import numpy as np

from analyze import analyze, at, interp
from fsmooth import smooth


def insert_metrics(r):
    s, ph, u = r['_series'], r['phases'], r['_series']['u']
    t0, t1, to, tx = ph['place:slide'], ph['place:hold'], ph['gripper_open'], ph['exit_place:fly']
    tw, wr, tm, mc = s['tw'], s['wr'], s['tm'], s['mc']
    eu = 1e3 * (interp(tm, tw, mc[:, :3]) - wr[:, :3]) @ u          # CoM - reference along the line [mm]
    tg, Fs = smooth(s['tg'], s['F'], 0.5)
    Fu = Fs @ u
    te = s['te']
    al, lat, up = (1e3 * x for x in s['claw_truth'])
    sl, mo = (tw >= t0) & (tw <= t1), (tw >= t1) & (tw <= to)
    gsl, gmo = (tg >= t0) & (tg <= t1), (tg >= t1) & (tg <= to)
    esl = (te >= t0) & (te <= t1)
    eall = (te >= t0) & (te <= to)
    # motor commands are a noisy 250 Hz signal: judge saturation by how long a motor STAYS at the top
    # (single-sample touches of 1.0 are the noise, 8 ms at most on the 10-31 3cm flight), not by the max
    tgm, mot = s['tg'], s['mot']
    mi = ~np.isnan(mot).any(1) & (tgm >= ph['place:wait']) & (tgm <= ph['exit_place:done'])
    hi = (mot[mi] >= 0.98).any(1)
    longest, cur = 0, 0
    for h in hi:
        cur = cur + 1 if h else 0
        longest = max(longest, cur)
    return {
        'release': r['release_mode'].split()[0],
        'slide_end_to_open_s': to - t1,
        'lag_in_slide_mm': float(eu[sl].min()),
        'ahead_after_slide_mm': float(eu[mo].max()) if mo.any() else float('nan'),
        'claw_max_past_mm': float(al[eall].max()),
        'claw_along_at_open_mm': float(at(to, te, al)),
        'claw_up_slide_mm': [float(up[esl].min()), float(up[esl].max())],
        'claw_lat_slide_mm': [float(lat[esl].min()), float(lat[esl].max())],
        'claw_up_at_open_mm': float(at(to, te, up)),
        'claw_lat_at_open_mm': float(at(to, te, lat)),
        'claw_err3d_at_open_mm': float(np.sqrt(at(to, te, al) ** 2 + at(to, te, lat) ** 2 + at(to, te, up) ** 2)),
        'Fu_hold_N': float(Fu[(tg >= ph['place:wait']) & (tg <= t0)].mean()),
        'Fu_slide_min_N': float(Fu[gsl].min()),
        'Fu_mouth_N': float(Fu[gmo].mean()) if gmo.any() else float('nan'),
        'kick_open_to_exit_mm': float(at(tx, tw, eu) - at(to, tw, eu)),
        'tilt_insert_deg': r['phase_vehicle']['slide + release']['peak_tilt_deg'],
        'track_rms_carry_mm': r['phase_vehicle']['carry']['track_rms_mm'],
        'track_rms_insert_mm': r['phase_vehicle']['slide + release']['track_rms_mm'],
        'motor_p99_insert': float(np.percentile(mot[mi], 99, axis=0).max()),
        'motor_sat_longest_ms': longest * 1e3 * float(np.median(np.diff(tgm[mi]))),
        'motor_mean_insert': r['sat']['insert_mean_motor'],
        'pick_err_mm': r['pick_err_at_close_mm'][0],
    }


def fmt(v):
    if isinstance(v, str):
        return v[:10]
    if isinstance(v, (list, tuple)):
        return '/'.join(f'{x:.0f}' if abs(x) >= 1 else f'{x:.2f}' for x in v)
    return f'{v:.2f}' if abs(v) < 10 else f'{v:.0f}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('tags', nargs='+')
    ap.add_argument('--fig', default='')
    ap.add_argument('--json', default='')
    a = ap.parse_args()
    runs = {t: analyze(t) for t in a.tags}
    M = {t: insert_metrics(r) for t, r in runs.items()}
    keys = list(next(iter(M.values())))
    print(f"{'':26s}" + ''.join(f'{t:>14s}' for t in a.tags))
    for k in keys:
        print(f'{k:26s}' + ''.join(f'{fmt(M[t][k]):>14s}' for t in a.tags))
    if a.json:
        json.dump(M, open(a.json, 'w'), indent=1)
    if a.fig:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
        for i, (t, r) in enumerate(runs.items()):
            s, ph, u = r['_series'], r['phases'], r['_series']['u']
            t0 = ph['place:slide']
            hw = i == 0
            kw = dict(color='k', lw=2.0, label=f'{t} (hardware)') if hw else dict(lw=1.0, alpha=0.85, label=t)
            te = s['te']; m = (te >= t0 - 3) & (te <= ph['exit_place:fly'] + 2)
            al, lat, up = s['claw_truth']
            ax[0].plot(te[m] - t0, 1e3 * al[m], **kw)
            ax[1].plot(te[m] - t0, 1e3 * up[m], **kw)
            tw = s['tw']; mw = (tw >= t0 - 3) & (tw <= ph['exit_place:fly'] + 2)
            e = interp(s['tm'], tw[mw], s['mc'][:, :3]) - s['wr'][mw, :3]
            ax[2].plot(tw[mw] - t0, 1e3 * (e @ u), **kw)
            tg, Fs = smooth(s['tg'], s['F'], 0.5); mg = (tg >= t0 - 3) & (tg <= ph['exit_place:fly'] + 2)
            ax[3].plot(tg[mg] - t0, Fs[mg] @ u, **kw)
            for x in ax:
                x.axvline(ph['gripper_open'] - t0, color='k' if hw else x.lines[-1].get_color(), lw=0.8, ls='--')
        ax[0].axhspan(-15, 20, color='g', alpha=0.08)
        ax[0].set_ylim(-170, 100)
        ax[0].set_ylabel('claw along line vs\nrelease pt [mm] (+ past)')
        ax[1].set_ylabel('claw up/down off\nthe line [mm]')
        ax[2].set_ylabel('CoM - ref along\nline [mm] (+ deeper)')
        ax[3].set_ylabel('L1 F_hat along line\n[N, 0.5 s mean]')
        ax[3].set_xlabel('t - slide start [s]   (dashed = gripper open)')
        for x in ax:
            x.axhline(0, color='k', lw=0.5); x.axvline(0, color='0.6', lw=0.8); x.axvline(3.0, color='0.6', lw=0.8)
            x.grid(alpha=0.3); x.legend(fontsize=7, ncol=3)
        fig.suptitle('3cm mail insert: hardware (black) vs Isaac', fontsize=11)
        fig.tight_layout()
        fig.savefig(a.fig, dpi=110)
        print('figure:', a.fig)


if __name__ == '__main__':
    main()

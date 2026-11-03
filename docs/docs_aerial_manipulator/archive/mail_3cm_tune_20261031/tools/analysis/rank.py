#!/usr/bin/env python3
"""Rank the gain candidates on the mail insert, repeats grouped (<layout>_<candidate>_<n>, e.g. c4_p_kv19_2).

    cd runs/work && /usr/bin/python3 ../../tools/analysis/rank.py [--ref hw3cm] [--json out.json]

Every run with {events,run,dbg}_<tag>.* here is used. Flights must be REAL TIME (fsc_lab_machine since
2026-11-02, the pacer at RTF 1.000); runs below that are kept in work/excluded/. A sim ranking, to be
confirmed in flight. Lower score = a more precise insert:

  PRECISION score (2026-11-02, mm): where the claw is when the gripper opens, and how far it strays off
  the insertion line while it slides through the mouth (that eats the mouth's clearance):
  score = 3-D claw error from the release point at the open
        + worst claw excursion off the line in the slide, up/down + worst lateral
        + 0.5 |CoM lag behind the reference in the slide| + 0.5 max(0, CoM ahead after the slide)
        + max(0, furthest past the release point - 20)
        + 50 if a motor STAYED at >= 0.98 for 20 ms or more in the insert
          (single-sample touches are command noise, not saturation)
  an aborted run scores 500 (it did not deliver).
"""
import argparse
import glob
import json
import re
from collections import defaultdict

import numpy as np

from analyze import analyze
from compare import insert_metrics

ABORT_SCORE = 500.0


def score(m):
    up = max(abs(m['claw_up_slide_mm'][0]), abs(m['claw_up_slide_mm'][1]))
    lat = max(abs(m['claw_lat_slide_mm'][0]), abs(m['claw_lat_slide_mm'][1]))
    return (m['claw_err3d_at_open_mm'] + up + lat
            + 0.5 * abs(m['lag_in_slide_mm']) + 0.5 * max(0.0, m['ahead_after_slide_mm'])
            + max(0.0, m['claw_max_past_mm'] - 20.0)
            + (50.0 if m['motor_sat_longest_ms'] >= 20.0 else 0.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ref', default='hw3cm')
    ap.add_argument('--json', default='')
    a = ap.parse_args()
    tags = sorted(re.sub(r'^events_|\.txt$', '', f) for f in glob.glob('events_*.txt'))
    rows = {}
    for t in tags:
        try:
            r = analyze(t)
        except Exception as e:                       # an incomplete recording (flight died early)
            rows[t] = {'tag': t, 'error': f'{type(e).__name__}: {e}'[:120]}
            continue
        m = insert_metrics(r)
        m['aborted'] = bool(r.get('aborted'))
        m['abort_reason'] = r.get('abort_reason', '')
        m['score'] = ABORT_SCORE if m['aborted'] else score(m)
        rows[t] = m
    groups = defaultdict(list)
    for t, m in rows.items():
        g = t if t == a.ref else re.sub(r'_\d+$', '', t)
        groups[g].append((t, m))
    keys = ['claw_err3d_at_open_mm', 'claw_along_at_open_mm', 'claw_max_past_mm', 'lag_in_slide_mm',
            'ahead_after_slide_mm', 'tilt_insert_deg', 'track_rms_carry_mm', 'track_rms_insert_mm',
            'motor_p99_insert', 'motor_sat_longest_ms']
    short = ['err3d', 'along', 'past', 'lag', 'ahead', 'tilt', 'rms_c', 'rms_i', 'mot_p99', 'sat_ms']
    print(f"{'candidate':12s} {'n':>2s} {'ok':>2s} {'auto':>4s} {'score':>7s} {'sd':>5s}  "
          + ''.join(f'{s:>8s}' for s in short) + '   up_slide  lat_slide')
    summary = {}
    for g, runs in sorted(groups.items(), key=lambda kv: np.mean([m.get('score', ABORT_SCORE) for _, m in kv[1]])):
        good = [m for _, m in runs if 'error' not in m and not m['aborted']]
        n_auto = sum(m['release'] == 'auto' for m in good)
        sc = np.mean([m.get('score', ABORT_SCORE) for _, m in runs])
        means = [np.mean([m[k] for m in good]) if good else float('nan') for k in keys]
        summary[g] = {'runs': [t for t, _ in runs], 'n_ok': len(good), 'n_auto': n_auto, 'score': sc,
                      **{k: v for k, v in zip(keys, means)},
                      'aborts': [m.get('abort_reason') or m.get('error') for _, m in runs
                                 if 'error' in m or m['aborted']]}
        sd = np.std([m.get('score', ABORT_SCORE) for _, m in runs])
        upr = '/'.join(f"{np.mean([m['claw_up_slide_mm'][i] for m in good]):.0f}" for i in (0, 1)) if good else '-'
        latr = '/'.join(f"{np.mean([m['claw_lat_slide_mm'][i] for m in good]):.0f}" for i in (0, 1)) if good else '-'
        print(f"{g:12s} {len(runs):2d} {len(good):2d} {n_auto:4d} {sc:7.1f} {sd:5.1f}  " +
              ''.join(f'{v:8.1f}' if abs(v) < 100 else f'{v:8.0f}' for v in means) + f'   {upr:>8s}  {latr:>9s}')
    for g, s in summary.items():
        for why in s['aborts']:
            print(f'  {g}: {why}')
    if a.json:
        json.dump(summary, open(a.json, 'w'), indent=1, default=float)


if __name__ == '__main__':
    main()

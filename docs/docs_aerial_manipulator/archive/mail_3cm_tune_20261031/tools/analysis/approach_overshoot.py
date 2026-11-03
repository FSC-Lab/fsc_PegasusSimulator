#!/usr/bin/env python3
"""How far the CoM runs PAST its reference toward the box in the place approach and after the
slide, per candidate:

    cd runs/work && /usr/bin/python3 ../../tools/analysis/approach_overshoot.py [--over 50]

Overshoot = max over the place approach (execute_place start .. the slide, or the abort) of the
CoM's forward error x_c - x_cd along world +x (the insertion direction in both layouts; debug
[48] - [45], from dbg_<tag>.npz). Positive = ahead of the reference, toward the mailbox face,
which is what eats the envelope's margin to the face (envelope_truth.py's pre-slide column).
Post-slide = the same from the slide's start to the gripper's open (or the abort): what runs the
claw past the release window and the envelope's tip into the box's back wall.
"""
import argparse
import glob
import re
from collections import defaultdict

import numpy as np


def first(ev, pat, after=-1.0):
    for m in re.finditer(r'^\s*([\d.]+) .*' + pat, ev, re.M):
        if float(m.group(1)) > after:
            return float(m.group(1))
    return None


def overshoot(tag):
    ev = open(f'events_{tag}.txt').read()
    t0 = int(re.search(r't0_ns (\d+)', ev).group(1))
    a = first(ev, r'execute_place: executing')
    if a is None:
        return None, None
    sl = first(ev, r'FLYING execute_place \(descent\)', a)
    end = min(x for x in (first(ev, r'ABORTING', a), first(ev, r'mode: SAFETY', a), 1e9) if x is not None)
    op = first(ev, r'isaac_gripper: goal: position 0\.0190', a)
    g = np.load(f'dbg_{tag}.npz')
    idx = list(g['idx'])
    t = (g['t'] - t0) / 1e9
    fwd = 1e3 * (g['x'][:, idx.index(48)] - g['x'][:, idx.index(45)])
    m = (t >= a) & (t <= min(sl or end, end))
    post = None
    if sl is not None and sl < end:
        mp = (t >= sl) & (t <= min(op or end, end))
        post = float(fwd[mp].max()) if mp.any() else None
    return (float(fwd[m].max()) if m.any() else None), post


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--over', type=float, default=50.0)
    a = ap.parse_args()
    groups = defaultdict(list)
    for f in sorted(glob.glob('events_*.txt')):
        tag = re.sub(r'^events_|\.txt$', '', f)
        if tag.startswith('hw'):
            continue
        try:
            v, p = overshoot(tag)
        except (OSError, KeyError, ValueError):
            v = p = None
        if v is not None:
            g = re.sub(r'_\d+$', '', re.sub(r'^c[34]_', '', tag))
            g = 'base' if g in ('base', 'diag') else g
            groups[g].append((tag, v, p))
    o = str(int(a.over))
    print(f"{'candidate':10s} {'n':>3s} | approach: {'>' + o + 'mm':>6s} {'median':>7s} {'max':>6s} |"
          f" post-slide: {'n':>3s} {'>' + o + 'mm':>6s} {'median':>7s} {'max':>6s}")
    for g, rows in sorted(groups.items(), key=lambda kv: np.median([r[1] for r in kv[1]])):
        v = np.array([r[1] for r in rows])
        p = np.array([r[2] for r in rows if r[2] is not None])
        ps = (f"{len(p):3d} {int((p > a.over).sum()):6d} {np.median(p):+7.0f} {p.max():+6.0f}" if len(p)
              else f"{0:3d}")
        print(f"{g:10s} {len(v):3d} |           {int((v > a.over).sum()):6d} {np.median(v):+7.0f} {v.max():+6.0f} |"
              f"             {ps}")


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Where the ENVELOPE really is during the place, from Isaac's ground truth (diag_topics.txt bags):

    cd runs && /usr/bin/python3 ../tools/analysis/envelope_truth.py c4_diag_1 [c3_diag_1 ...] [--trace]

For every sample from the place approach to the exit (or the abort), the envelope's slab and its
invisible grip pad (07_px4_t650_aerial_manipulator_mail_delivery.py: 150 x 90 x 4 mm, pad 45 x 90 x
20 mm at the near end, payload_0 = the slab centre, +x toward the far end) are put in the world and
measured against the RED mailbox the layout builds (front face at TABLE_FRONT_X, 5 mm walls, the
mouth cut in the front): clearance to the mouth's upper and lower lip and to the jambs where the
envelope crosses the wall, how deep its far end is against the back wall, and whether it has
slipped or turned in the jaws (the claw_0 point in the envelope's frame, against its value at the
start of the place). Negative clearance = the envelope is IN the wall: contact.
"""
import argparse
import re

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message

TABLE_H, WALL = 0.800, 0.005
ENV_L, ENV_W, ENV_T, GRIP_D, GRIP_T = 0.150, 0.090, 0.004, 0.045, 0.020
LAYOUT = {   # front x, red y, box (w, d, h), mouth (w, h, centre above the box bottom) -- run_mail_3cm.sh
    '3cm': dict(fx=0.601, cy=0.286, box=(0.235, 0.125, 0.3185), mouth=(0.190, 0.060, 0.2485)),
    '4cm': dict(fx=0.621, cy=0.283, box=(0.235, 0.125, 0.2985), mouth=(0.190, 0.050, 0.2285)),
}
TOPICS = ['/payload_0/state/pose', '/claw_0/state/pose', '/uav_0/state/pose']


def quat_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def read_poses(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    out = {k: ([], []) for k in TOPICS}
    while r.has_next():
        k, raw, st = r.read_next()
        p = deserialize_message(raw, PoseStamped).pose
        out[k][0].append(st)
        out[k][1].append([p.position.x, p.position.y, p.position.z,
                          p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z])
    return {k: (np.array(t, dtype=np.int64), np.array(v)) for k, (t, v) in out.items()}


def edge_points(n=31):
    """Sample points on the slab's and the pad's outline, envelope frame."""
    xs = np.linspace(-0.5 * ENV_L, 0.5 * ENV_L, n)
    pts = []
    for y in (-0.5 * ENV_W, 0.0, 0.5 * ENV_W):
        for z in (-0.5 * ENV_T, 0.5 * ENV_T):
            pts += [(x, y, z) for x in xs]
    xp = np.linspace(-0.5 * ENV_L, -0.5 * ENV_L + GRIP_D, 7)
    for y in (-0.5 * ENV_W, 0.0, 0.5 * ENV_W):
        for z in (-0.5 * GRIP_T, 0.5 * GRIP_T):
            pts += [(x, y, z) for x in xp]
    return np.array(pts)


PTS = edge_points()


def events(tag):
    ev = open(f'work/events_{tag}.txt').read()
    t0 = int(re.search(r't0_ns (\d+)', ev).group(1))
    def first(pat, after=-1.0):
        for m in re.finditer(r'^\s*([\d.]+) .*' + pat, ev, re.M):
            if float(m.group(1)) > after:
                return float(m.group(1))
        return None
    a = first(r'execute_place: executing')
    return t0, {'approach': a, 'slide': first(r'AUTO INSERT', a),
                'open': first(r'isaac_gripper: goal: position 0\.0190', a), 'abort': first(r'ABORTING', a),
                'exit': first(r'exit_place: executing', a), 'safety': first(r'mode: SAFETY', a)}


def analyze(tag, trace=False):
    L = LAYOUT['4cm' if tag.startswith('c4') else '3cm']
    fx, cy = L['fx'], L['cy']
    bw, bd, bh = L['box']
    mw, mh, mz = L['mouth']
    m_lo, m_hi = TABLE_H + mz - 0.5 * mh, TABLE_H + mz + 0.5 * mh
    back_in = fx + bd - WALL
    t0, ph = events(tag)
    P = read_poses(f'{tag}.bag')
    te, E = P['/payload_0/state/pose']
    tc, C = P['/claw_0/state/pose']
    if not len(te):
        raise SystemExit(f'{tag}: no /payload_0/state/pose in the bag (record with tools/diag_topics.txt)')
    te = (te - t0) / 1e9
    tc = (tc - t0) / 1e9
    a = ph['approach']
    b = min(x for x in (ph['exit'], ph['safety'], ph['abort'] and ph['abort'] + 2.0, te[-1]) if x is not None)
    rows = []
    for k in np.nonzero((te >= a) & (te <= b + 1.0))[0]:
        R = quat_R(E[k, 3:7])
        W = E[k, :3] + PTS @ R.T
        # the wall slab x in [fx, fx + WALL]: which outline points are in it, and are they in the opening?
        inw = (W[:, 0] >= fx - 0.001) & (W[:, 0] <= fx + WALL + 0.001)
        top_c = bot_c = lat_c = np.nan
        if inw.any():
            zw, yw = W[inw, 2], W[inw, 1]
            top_c = 1e3 * (m_hi - zw.max())
            bot_c = 1e3 * (zw.min() - m_lo)
            lat_c = 1e3 * (0.5 * mw - np.abs(yw - cy).max())
        tip_back = 1e3 * (back_in - W[:, 0].max())          # far end to the back wall's inner face
        face = 1e3 * (fx - W[:, 0].max())                   # > 0: entirely in front of the face
        j = min(np.searchsorted(tc, te[k]), len(tc) - 1)
        claw_env = R.T @ (C[j, :3] - E[k, :3])              # the claw point in the envelope's frame
        pitch = np.degrees(np.arcsin(np.clip(-R[2, 0], -1, 1)))   # envelope nose-down positive
        rows.append((te[k], top_c, bot_c, lat_c, tip_back, face, *claw_env, pitch))
    A = np.array(rows)
    t = A[:, 0]
    grip0 = A[0, 6:9]
    slip = 1e3 * np.linalg.norm(A[:, 6:9] - grip0, axis=1)
    # the summary covers the envelope while it is still HELD: up to the gripper's open or the abort
    # (after either it falls into the box or is backed out, and its clearances mean nothing)
    held = t <= min(x for x in (ph['open'], ph['abort'], t[-1]) if x is not None)
    def mn(col):
        v = np.where(held & ~np.isnan(A[:, col]), A[:, col], np.inf)
        k = int(v.argmin())
        return (float(v[k]), float(t[k])) if np.isfinite(v[k]) else (np.nan, np.nan)
    sl = np.where(held, slip, -1.0)
    # the approach alone (before the slide, or the abort): how close the envelope came to the face
    pre = t < min(x for x in (ph['slide'], ph['abort'], t[-1] + 1) if x is not None)
    face_pre = (float(A[pre, 5].min()), float(t[pre][A[pre, 5].argmin()])) if pre.any() else (np.nan, np.nan)
    res = {'tag': tag, 'phases': ph, 'top_min': mn(1), 'bot_min': mn(2), 'lat_min': mn(3), 'back_min': mn(4),
           'face_min': mn(5), 'face_pre': face_pre, 'slip_max_mm': float(sl.max()), 'slip_at': float(t[sl.argmax()]),
           'pitch_range': (float(A[held, 9].min()), float(A[held, 9].max()))}
    if trace:
        print(f'   t      top_lip  bot_lip  jamb   back  face | claw in env (mm)     slip | env pitch')
        for k in range(0, len(A), max(1, len(A) // 60)):
            r = A[k]
            print(f'  {r[0]:6.2f}  ' + '  '.join(f'{v:+6.1f}' if not np.isnan(v) else '     -' for v in r[1:6])
                  + f' | {1e3*r[6]:+5.1f} {1e3*r[7]:+5.1f} {1e3*r[8]:+5.1f}  {slip[k]:5.1f} | {r[9]:+5.1f}')
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('tags', nargs='+')
    ap.add_argument('--trace', action='store_true')
    a = ap.parse_args()
    print(f"{'flight':12s} {'result':8s} | while held, min clearance mm (at t):  upper lip    lower lip     jamb"
          f"      back wall   face (held)    face (pre-slide) | slip in jaws  | env pitch deg")
    for tag in a.tags:
        if a.trace:
            print(f'== {tag}')
        r = analyze(tag, a.trace)
        ph = r['phases']
        res = 'abort' if ph['abort'] else 'ok'
        f = lambda m: f'{m[0]:+6.1f} ({m[1]:5.1f})' if not np.isnan(m[0]) else '     -        '
        print(f"{tag:12s} {res:8s} | {f(r['top_min'])} {f(r['bot_min'])} {f(r['lat_min'])} {f(r['back_min'])} "
              f"{f(r['face_min'])} {f(r['face_pre'])} | "
              f"{r['slip_max_mm']:5.1f} ({r['slip_at']:5.1f}) | {r['pitch_range'][0]:+5.1f}..{r['pitch_range'][1]:+5.1f}"
              + (f"   [abort {ph['abort']:.1f}]" if ph['abort'] else f"   [slide {ph['slide']}, open {ph['open']}]"))


if __name__ == '__main__':
    main()

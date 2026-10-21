#!/usr/bin/env python3
"""Replay the guard reference over every recorded hardware flight.

The drone's /uav_0/mocap as RECORDED ON THE LAPTOP (the processor's output,
before the WiFi hop to the Orin), so this tests the processor-side faults --
freezes, flips, Motive pauses -- not WiFi drops. Scored over each flight's
airborne span (mocap z > 0.45 m; the vehicle rests at 0.25-0.31 m).

Run: PYTHONNOUSERSITE=1 /usr/bin/python3 -I replay_flights.py
"""

import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import guard_reference as G  # noqa: E402

ARCH = os.path.abspath(os.path.join(HERE, '..', '..'))
SOURCES = sorted(glob.glob(os.path.join(ARCH, 'wb_pick_drop_flight_20261014', 'npz', 'hist', '*.npz'))) + [
    os.path.join(ARCH, 'wb_pick_drop_flight_20261015', 'npz', f'{n}.npz') for n in ('e1', 'd1', 'd2')] + [
    os.path.join(ARCH, 'wb_pick_drop_flight_20261016', 'npz', f'{n}.npz') for n in ('e1', 'd1', 'd2')]
AIR_Z = 0.45
SKIP_START = 1.0   # [s] of each recording not scored


def label(path):
    b = os.path.basename(path)[:-4]
    if b.startswith('flight_'):
        b = b[len('flight_'):]
    else:
        b = os.path.basename(os.path.dirname(os.path.dirname(path)))[-8:] + '_' + b
    return b.replace('wb_l1_4d_', 'wb_').replace('decoupled_l1_', 'dec_')


def load(path):
    z = np.load(path, allow_pickle=True)
    t = z['mocap__recv'].astype(float)
    p = np.c_[z['mocap__pose.position.x'], z['mocap__pose.position.y'], z['mocap__pose.position.z']]
    v = np.c_[z['mocap__twist.linear.x'], z['mocap__twist.linear.y'], z['mocap__twist.linear.z']]
    q = np.c_[z['mocap__pose.orientation.w'], z['mocap__pose.orientation.x'],
              z['mocap__pose.orientation.y'], z['mocap__pose.orientation.z']]
    o = np.argsort(t, kind='stable')
    return t[o], p[o], v[o], q[o]


def main():
    rows = []
    episodes = []
    total_air = 0.0
    for path in SOURCES:
        if not os.path.exists(path):
            continue
        t, p, v, q = load(path)
        air = np.where(p[:, 2] > AIR_Z)[0]
        if len(air) < 200:
            continue
        # A bag that starts mid-flight opens with the recorder's own catch-up
        # (a 345 ms gap and then samples 0.1 ms apart on 09-18): not the feed.
        t0, t1 = max(t[air[0]], t[0] + SKIP_START), t[air[-1]]
        ev, chk, max_gap = G.run_ladder(t, p, v, q, t_start=t0, t_end=t1)
        br = [e for e in ev if e.kind == 'bridging']
        cf = [e for e in ev if e.kind == 'confirmed']
        lo = [e for e in ev if e.kind == 'lost']
        reasons = {}
        for e in br:
            reasons[e.reason] = reasons.get(e.reason, 0) + 1
        name = label(path)
        total_air += t1 - t0
        rows.append((name, t1 - t0, max_gap * 1e3, len(br), sum(e.t1 - e.t0 for e in br),
                     ' '.join(f'{k}:{n}' for k, n in sorted(reasons.items())), len(cf), len(lo),
                     chk.n_jump, chk.n_frozen))
        for e in cf + lo:
            episodes.append((name, e.kind, e.t0 - t0, e.t1 - e.t0, e.reason))
        if chk.jumps:
            big = max(chk.jumps, key=lambda j: j[1])
            episodes.append((name, 'largest jump', big[0] - t0, big[1], f'{big[2]:.0f} deg'))
    print(f'{"flight":46s} {"air s":>6s} {"gap ms":>7s} {"bridge":>6s} {"bridge s":>8s} '
          f'{"reasons":14s} {"CONF":>4s} {"LOST":>4s} {"jumps":>5s} {"frozen":>6s}')
    for r in rows:
        print(f'{r[0]:46s} {r[1]:6.1f} {r[2]:7.1f} {r[3]:6d} {r[4]:8.2f} {r[5]:14s} {r[6]:4d} {r[7]:4d} '
              f'{r[8]:5d} {r[9]:6d}')
    print(f'\n{len(rows)} flights, {total_air:.0f} s airborne, '
          f'{sum(r[6] for r in rows)} confirmed episodes, {sum(r[7] for r in rows)} lost episodes')
    print('\nconfirmed / lost episodes and the largest jump per flight (t from lift-off):')
    for e in episodes:
        if e[1] == 'largest jump':
            print(f'  {e[0]:46s} largest jump at {e[2]:7.2f} s: {e[3] * 1e3:7.1f} mm, {e[4]}')
        else:
            print(f'  {e[0]:46s} {e[1]:9s} at {e[2]:7.2f} s, {e[3]:5.2f} s after the episode began ({e[4]})')


if __name__ == '__main__':
    main()

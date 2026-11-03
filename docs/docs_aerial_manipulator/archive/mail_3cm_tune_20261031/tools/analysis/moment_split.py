#!/usr/bin/env python3
"""Where the base moment goes during the place: the whole-body law's tau_body split into its three
shares (wb_control_debug [144..149], flown since 2026-11-02 -- older bags lack them):

    cd runs && /usr/bin/python3 ../tools/analysis/moment_split.py c4_m25_1 [...] [--plot out.png]

tau_body [18..20] = u1 (r_0c x e3) + u2 + N1^T u3, MODEL frame (model x = pitch):
  u2      [144..146]  the attitude law's moment (k_R e_R + k_w e_w, inertia shaped, + Coriolis)
  arm     [147..149]  N1^T u3, the arm reaction the rotors pre-compensate
  thrust  the rest    u1 (r_0c x e3), thrust about the CoM offset; moves with the arm's fold

Question it answers: when the law asks for pitch (e_R [28] grows) but the physical moment stays
put, WHICH share cancels u2? Printed per flight and window: the spread (std) of each share, how
much of u2's swing each other share takes back (slope of share on u2: -1 = cancels it all), and
the CoM's forward error (x_c - x_cd, debug [48] - [45]).
"""
import argparse
import re

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray

TOP = '/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug'


def read_dbg(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[TOP]))
    ts, rows = [], []
    while r.has_next():
        _, raw, st = r.read_next()
        d = np.asarray(deserialize_message(raw, Float32MultiArray).data, dtype=float)
        if len(d) < 150:
            continue                       # SAFETY's short array, or a bag from before the split
        rows.append(d[:150])
        ts.append(st)
    return np.array(ts, dtype=np.int64), np.array(rows)


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


def split(D):
    tau, u2, arm = D[:, 18:21], D[:, 144:147], D[:, 147:150]
    return {'e_R': np.degrees(D[:, 28:31]), 'tau': tau, 'u2': u2, 'arm': arm, 'thrust': tau - u2 - arm,
            'com_fwd': 1e3 * (D[:, 48] - D[:, 45])}


def stats(S, m, ax=0):
    u = S['u2'][m, ax] - S['u2'][m, ax].mean()
    out = {'n': int(m.sum())}
    for k in ('e_R', 'u2', 'arm', 'thrust', 'tau'):
        out[f'sd_{k}'] = float(S[k][m, ax].std())
    for k in ('arm', 'thrust', 'tau'):
        v = S[k][m, ax] - S[k][m, ax].mean()
        out[f'slope_{k}'] = float(u @ v / (u @ u)) if u @ u > 0 else np.nan
    out['com_fwd'] = (float(S['com_fwd'][m].min()), float(S['com_fwd'][m].max()))
    out['e_R_rng'] = (float(S['e_R'][m, ax].min()), float(S['e_R'][m, ax].max()))
    return out


def analyze(tag):
    t0, ph = events(tag)
    ts, D = read_dbg(f'{tag}.bag')
    if not len(ts):
        raise SystemExit(f'{tag}: no 150-slot wb_control_debug (flown before the 2026-11-02 split?)')
    t = (ts - t0) / 1e9
    a = ph['approach']
    end = min(x for x in (ph['exit'], ph['safety'], ph['abort'] and ph['abort'] + 2.0, t[-1]) if x is not None)
    s = ph['slide'] or end
    S = split(D)
    wins = {'approach': (t >= a) & (t < s), 'slide+': (t >= s) & (t <= end), 'place': (t >= a) & (t <= end)}
    return t, S, ph, {k: stats(S, m) for k, m in wins.items() if m.sum() > 50}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('tags', nargs='+')
    ap.add_argument('--plot', default='')
    a = ap.parse_args()
    print('PITCH axis (model x), N.m; slope = share moved per N.m of u2 (-1: cancels all of it)')
    print(f"{'flight':10s} {'window':9s} {'res':5s} | e_R deg range  | sd: u2    arm  thrust  tau  |"
          f" slope: arm thrust  tau | CoM fwd err mm")
    runs = {}
    for tag in a.tags:
        t, S, ph, W = analyze(tag)
        runs[tag] = (t, S, ph)
        res = 'abort' if ph['abort'] else 'ok'
        for w, r in W.items():
            print(f"{tag:10s} {w:9s} {res:5s} | {r['e_R_rng'][0]:+5.1f}..{r['e_R_rng'][1]:+5.1f} |"
                  f" {r['sd_u2']:.3f} {r['sd_arm']:.3f} {r['sd_thrust']:.3f} {r['sd_tau']:.3f} |"
                  f" {r['slope_arm']:+5.2f} {r['slope_thrust']:+5.2f} {r['slope_tau']:+5.2f} |"
                  f" {r['com_fwd'][0]:+5.0f}..{r['com_fwd'][1]:+5.0f}")
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(len(runs), 1, figsize=(11, 3.4 * len(runs)), squeeze=False)
        for ax, (tag, (t, S, ph)) in zip(axs[:, 0], runs.items()):
            m = (t >= ph['approach'] - 2) & (t <= (ph['exit'] or ph['abort'] or t[-1]) + 3)
            for k, c in (('u2', 'C0'), ('arm', 'C3'), ('thrust', 'C2'), ('tau', 'k')):
                ax.plot(t[m], S[k][m, 0], c, lw=1.6 if k == 'tau' else 1.0, label=k + (' (sum)' if k == 'tau' else ''))
            ax.set_ylabel('pitch moment N.m')
            ax2 = ax.twinx()
            ax2.plot(t[m], S['e_R'][m, 0], 'C1--', lw=0.9, label='e_R pitch deg')
            ax2.plot(t[m], S['com_fwd'][m] / 10, 'C4:', lw=0.9, label='CoM fwd err cm')
            ax2.set_ylabel('deg / cm')
            for k, v in ph.items():
                if v is not None and k != 'safety':
                    ax.axvline(v, color='0.6', lw=0.6)
                    ax.text(v, ax.get_ylim()[1], k, fontsize=7, rotation=90, va='top')
            ax.set_title(f"{tag} ({'abort' if ph['abort'] else 'delivered'})", fontsize=10)
            h1, l1 = ax.get_legend_handles_labels()
            h2, l2 = ax2.get_legend_handles_labels()
            ax.legend(h1 + h2, l1 + l2, fontsize=7, ncol=6, loc='lower left')
        axs[-1, 0].set_xlabel('t [s]')
        fig.tight_layout()
        fig.savefig(a.plot, dpi=110)
        print('wrote', a.plot)


if __name__ == '__main__':
    main()

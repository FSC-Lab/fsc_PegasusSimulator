"""Per-run metrics for the 10-31 hardware mail-delivery flights (whole-body L1 4-D, fused).

Inputs per run: run_<tag>.npz (extract.py), dbg_<tag>.npz (extract_dbg.py) and
events_<tag>.txt (events.py).
Claw = the planner's own estimate (current_ee: fused odometry + measured joints);
claw_truth = that estimate shifted by the fused-odometry position error against the
processed mocap (/uav_0/mocap), which removes the estimator's position error but not
its attitude error.
"""
import json
import re
import sys

import numpy as np

LEGS = ['go_to_start', 'execute_pick', 'go_to_place_start', 'execute_place', 'go_to_land_start', 'execute_land']


def parse_events(path):
    txt = open(path).read()
    t0 = int(re.search(r't0_ns (\d+)', txt).group(1))
    ev = []
    for ln in txt.splitlines():
        m = re.match(r'\s*([0-9.]+) (.*)$', ln)
        if m:
            ev.append((float(m.group(1)), m.group(2)))
    return t0, ev


def first(ev, pat, after=-1.0):
    for t, s in ev:
        if t > after and re.search(pat, s):
            return t
    return None


def gripper_goals(ev):
    """Start time of each gripper action goal (a new entry in the status list)."""
    out, n = [], 0
    for t, s in ev:
        if 'isaac_gripper: goal: position' in s:   # sim: the Isaac gripper action server logs each goal
            out.append(t)
            continue
        if s.startswith('GRIPPER_ACTION'):
            k = len(s.split(' ', 1)[1].split(','))
            if k > n:
                out.append(t)
                n = k
    return out


def interp(t, tq, x):
    return np.column_stack([np.interp(tq, t, x[:, i]) for i in range(x.shape[1])])


def tilt_deg(q):
    x, y = q[:, 1], q[:, 2]
    return np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))


def at(t, ts, x):
    i = np.searchsorted(ts, t)
    return x[min(i, len(x) - 1)]


def analyze(tag):
    d = np.load(f'run_{tag}.npz')
    g = np.load(f'dbg_{tag}.npz')
    t0, ev = parse_events(f'events_{tag}.txt')
    T = lambda k: (d[k + '_t'] - t0) / 1e9
    info = [r for r in d['pp_info'] if r[4] == 1 and not np.isnan(r[14])][-1]
    goal = {LEGS[k]: info[14 + 7 * k:14 + 7 * k + 7] for k in range(6)}
    rel_claw = goal['execute_place'][4:7]
    rel_base = goal['execute_place'][:3]

    r = {'tag': tag}
    # ---------------- timeline ----------------
    t_direct = first(ev, r'mode: DIRECT')
    t_safety = first(ev, r'mode: SAFETY', after=t_direct)
    gg = gripper_goals(ev)
    r['t_direct'], r['t_safety'], r['direct_s'] = t_direct, t_safety, t_safety - t_direct
    ph = {}
    for leg in LEGS:
        ph[leg + ':fly'] = first(ev, rf'pp/status: FLYING {leg}')
        ph[leg + ':done'] = first(ev, rf'pp/status: DONE {leg}')
    ph['pick:wait'] = first(ev, r'WAITING execute_pick')
    ph['pick:descent'] = first(ev, r'FLYING execute_pick \(descent\)')
    ph['place:wait'] = first(ev, r'WAITING execute_place')
    ph['place:slide'] = first(ev, r'FLYING execute_place \(descent\)')
    ph['place:hold'] = ph['execute_place:done']
    ph['exit_pick:fly'] = first(ev, r'FLYING exit_pick')
    ph['exit_place:fly'] = first(ev, r'FLYING exit_place')
    ph['exit_place:done'] = first(ev, r'DONE exit_place')
    t_close = [x for x in gg if ph['execute_pick:done'] - 0.5 < x < ph['exit_pick:fly'] + 0.5]
    t_ex = ph['exit_place:fly'] if ph['exit_place:fly'] is not None else float('inf')   # none when aborted
    t_open = [x for x in gg if ph['place:hold'] - 0.5 < x < t_ex + 0.5]
    ph['gripper_close'] = t_close[0] if t_close else None
    ph['gripper_open'] = t_open[0] if t_open else None
    ph['release_signalled'] = first(ev, r'AUTO RELEASE', after=ph['place:hold'] - 0.1)
    # an ABORTED flight (the planner's guard, a watchdog): the insert window ends at the abort
    ph['abort'] = first(ev, r'pp/status: ABORTING', after=ph['place:wait'] or 0.0)
    r['aborted'] = ph['abort'] is not None and (ph['gripper_open'] is None or ph['abort'] <= ph['gripper_open'] + 0.05)
    # (2026-11-02) the driver and the running arm GS BOTH open on the planner's auto release; the
    # gripper server serves the newer goal and aborts the older, and when the driver's was the
    # older the old driver read it as a failed open and aborted AFTER a clean release (jaws fully
    # open, envelope let go at the release point). A delivery: scored to the open, the SAFETY
    # revert standing in for the exit.
    dup = ph['gripper_open'] is not None and ph['exit_place:fly'] is None and any(
        ph['gripper_open'] <= t <= ph['gripper_open'] + 0.1 and 'goal preempted by a newer one' in s for t, s in ev)
    r['open_dup_abort'] = bool(dup)
    if dup:
        r['aborted'] = False
        ph['exit_place:fly'] = first(ev, r'mode: SAFETY', after=ph['gripper_open'])
        ph['exit_place:done'] = ph['exit_place:fly'] + 3.0
    if ph['exit_place:fly'] is not None and ph['exit_place:done'] is None:
        # the arm GS started the exit and the old driver, refused its own start, aborted it
        ph['exit_place:done'] = first(ev, r'mode: SAFETY', after=ph['exit_place:fly']) or ph['exit_place:fly'] + 3.0
    if r['aborted']:
        ph['gripper_open'] = ph['abort']
        ph['exit_place:fly'] = ph['abort']
        ph['exit_place:done'] = ph['abort'] + 3.0
        r['abort_reason'] = next(s for t, s in ev if t >= ph['abort'] and 'ABORTING' in s)[:160]
    r['phases'] = ph
    by_hand = first(ev, r'gripper OPEN by hand', after=ph['place:hold'] - 0.1)
    if r['aborted']:
        r['release_mode'] = 'aborted'
    elif by_hand is not None and abs(by_hand - ph['gripper_open']) < 0.1:
        r['release_mode'] = 'by hand (Insert)'
    else:
        r['release_mode'] = 'auto (planner)'
    r['wait_pick_s'] = ph['pick:descent'] - ph['pick:wait']
    r['wait_insert_gate_s'] = ph['place:slide'] - ph['place:wait']
    r['hold_to_open_s'] = ph['gripper_open'] - ph['place:hold']
    r['mission_s'] = (ph['execute_land:fly'] + 5.8 - ph['go_to_start:fly']) if ph['execute_land:fly'] else float('nan')
    reasons = [s for t, s in ev if ph['place:hold'] <= t <= ph['gripper_open'] and 'auto release:' in s]
    r['release_gate_nan_ticks'] = sum('nan m/s' in s for s in reasons)
    r['release_gate_ticks'] = len(reasons)
    win = ' '.join(s for t, s in ev if ph['place:hold'] <= t <= ph['exit_place:fly'])
    m_past = [float(x) for x in re.findall(r'claw (\d+) mm past the release point', win)]
    m_short = [float(x) for x in re.findall(r'claw (\d+) mm short of the release point', win)]
    r['status_max_past_mm'] = max(m_past) if m_past else 0.0
    r['status_max_short_mm'] = max(m_short) if m_short else 0.0

    # insertion line: the hold (pending_base at WAITING execute_place) -> the release base
    hold = [s for t, s in ev if s.startswith('pending_base') and abs(t - ph['place:wait']) < 0.05]
    hold_base = np.array([float(v) for v in re.findall(r'-?\d+\.\d+', hold[0])])
    u = rel_base - hold_base
    r['insert_margin_m'] = float(np.linalg.norm(u))
    u = u / np.linalg.norm(u)
    r['insert_pitch_deg'] = float(np.degrees(np.arcsin(-u[2])))
    r['release_claw_target'] = rel_claw.tolist()
    lat_dir = np.cross(u, [0, 0, 1.0])
    lat_dir /= np.linalg.norm(lat_dir)
    up_dir = np.cross(lat_dir, u)

    # ---------------- signals ----------------
    te, ee = T('current_ee'), d['current_ee'][:, :3]
    to, od = T('odom'), d['odom']
    tm, mc = T('mocap'), d['mocap']
    if 'vrpn_uav_0' in d:                       # hardware: Motive's rigid body
        tv, qv = T('vrpn_uav_0'), d['vrpn_uav_0'][:, 3:7]
    else:                                      # sim: no VRPN, PX4's attitude (tilt is frame-independent)
        tv, qv = T('att'), d['att']
    est_err = interp(to, te, od[:, :3]) - interp(tm, te, mc[:, :3])
    ee_truth = ee - est_err
    tilt = tilt_deg(qv)

    def claw_frame(p):
        dm = p - rel_claw
        along = dm @ u
        offv = dm - np.outer(along, u)
        return along, offv @ lat_dir, offv @ up_dir

    s0, s1, o = ph['place:slide'], ph['place:hold'], ph['gripper_open']
    for name, P in (('claw', ee), ('claw_truth', ee_truth)):
        a, lat, up = claw_frame(P)
        sl = (te >= s0) & (te <= o)
        sl2 = (te >= s0) & (te <= s1)
        r[name] = {
            'at_slide_end_mm': [1e3 * at(s1, te, a), 1e3 * at(s1, te, lat), 1e3 * at(s1, te, up)],
            'at_open_mm': [1e3 * at(o, te, a), 1e3 * at(o, te, lat), 1e3 * at(o, te, up)],
            'max_past_mm': 1e3 * float(a[sl].max()),
            'up_range_mm': [1e3 * float(up[sl].min()), 1e3 * float(up[sl].max())],
            'lat_range_mm': [1e3 * float(lat[sl].min()), 1e3 * float(lat[sl].max())],
            'slide_up_range_mm': [1e3 * float(up[sl2].min()), 1e3 * float(up[sl2].max())],
            'slide_lat_range_mm': [1e3 * float(lat[sl2].min()), 1e3 * float(lat[sl2].max())],
        }

    ins = (to >= ph['place:wait']) & (to <= ph['exit_place:fly'])
    pe = od[ins, :3] - interp(tm, to[ins], mc[:, :3])
    ve = od[ins, 7:10] - interp(tm, to[ins], mc[:, 3:6])
    r['est_pos_err_insert_mm'] = {'mean': (1e3 * pe.mean(0)).tolist(), 'max_norm': float(1e3 * np.linalg.norm(pe, axis=1).max())}
    r['est_vel_err_insert_mps'] = {'mean': ve.mean(0).tolist()}

    ta, ae = T('pp_arrival_error'), d['pp_arrival_error']
    k = np.searchsorted(ta, ph['gripper_close']) - 1
    r['pick_err_at_close_mm'] = (1e3 * ae[k, 2:6]).tolist()

    # vehicle per phase
    tw, wr = T('wb_ref'), d['wb_ref']

    def phase_stats(a, b):
        s = (tv >= a) & (tv <= b)
        sw = (tw >= a) & (tw <= b)
        e = interp(tm, tw[sw], mc[:, :3]) - wr[sw, :3]
        n = np.linalg.norm(e, axis=1)
        if not s.any() or not len(n):          # a phase not flown (the open_dup_abort exit)
            return {'peak_tilt_deg': float('nan'), 'track_rms_mm': float('nan'), 'track_max_mm': float('nan')}
        return {'peak_tilt_deg': float(tilt[s].max()), 'track_rms_mm': float(1e3 * np.sqrt((n ** 2).mean())),
                'track_max_mm': float(1e3 * n.max())}
    r['phase_vehicle'] = {
        'pick': phase_stats(ph['execute_pick:fly'], ph['gripper_close']),
        'carry': phase_stats(ph['exit_pick:fly'], ph['place:wait']),
        'hold before slide': phase_stats(ph['place:wait'], ph['place:slide']),
        'slide + release': phase_stats(ph['place:slide'], ph['gripper_open'] + 0.5),
        'exit + return + land': phase_stats(ph['exit_place:fly'], t_safety),
    }
    s = (tm >= o) & (tm <= o + 1.0)
    disp = mc[s, :3] - at(o, tm, mc[:, :3])
    r['post_open_1s'] = {'max_disp_mm': float(1e3 * np.linalg.norm(disp, axis=1).max()),
                         'disp_along_u_mm': [float(1e3 * (disp @ u).min()), float(1e3 * (disp @ u).max())],
                         'peak_tilt_deg': float(tilt[(tv >= o) & (tv <= o + 1.0)].max())}

    # whole-body debug: F_hat (the 4-D L1 attribution), rotor saturation
    tg, gx = (g['t'] - t0) / 1e9, g['x']
    col = {int(i): j for j, i in enumerate(g['idx'])}
    F = gx[:, [col[97], col[98], col[99]]]
    nsat = gx[:, col[51]]
    una = gx[:, col[55]]
    mot = gx[:, [col[41], col[42], col[43], col[44]]]

    def fwin(a, b):
        s = (tg >= a) & (tg <= b)
        Fu = F[s] @ u          # along the insertion line (+ = pushed deeper)
        return {'Fu_min': float(Fu.min()), 'Fu_max': float(Fu.max()), 'F_norm_max': float(np.linalg.norm(F[s], axis=1).max()),
                'Fu_mean': float(Fu.mean())}
    r['F_hat'] = {
        'carry': fwin(ph['exit_pick:fly'], ph['place:wait']),
        'hold before slide': fwin(ph['place:wait'], ph['place:slide']),
        'slide': fwin(ph['place:slide'], ph['place:hold']),
        'held in mouth': fwin(ph['place:hold'], ph['gripper_open']),
        '1 s after open': fwin(ph['gripper_open'], ph['gripper_open'] + 1.0),
    }
    sd = (tg >= t_direct) & (tg <= t_safety)
    si = (tg >= ph['place:wait']) & (tg <= ph['exit_place:done'])
    r['sat'] = {'direct_frac_nsat>0': float((nsat[sd] > 0).mean()), 'insert_frac_nsat>0': float((nsat[si] > 0).mean()),
                'insert_max_motor': float(np.nanmax(mot[si])), 'direct_max_motor': float(np.nanmax(mot[sd])),
                'insert_min_unalloc_thrust_N': float(np.nanmin(una[si])), 'direct_min_unalloc_thrust_N': float(np.nanmin(una[sd])),
                'insert_mean_motor': np.nanmean(mot[si], 0).round(3).tolist()}
    names = list(d['joint_names'])
    js, tj = d['joint_states'], T('joint_states')
    q4 = np.degrees(js[:, names.index('joint4')])
    r['q4_abs_deg_insert'] = float(np.abs(q4[(tj >= s0) & (tj <= o)]).mean())
    tb, bt = T('battery_status'), d['battery_status']
    r['battery_v'] = [float(at(t_direct, tb, bt[:, 0])), float(bt[(tb >= t_direct) & (tb <= t_safety), 0].min())]
    r['land_err_mm'] = [float('nan')] * 3 if r['aborted'] or dup else (1e3 * (at(t_safety, tm, mc[:, :3]) - goal['execute_land'][:3])).tolist()
    r['_series'] = dict(te=te, claw=claw_frame(ee), claw_truth=claw_frame(ee_truth), tg=tg, F=F, u=u, tv=tv, tilt=tilt,
                        tm=tm, mc=mc, tw=tw, wr=wr, mot=mot, nsat=nsat)
    return r


if __name__ == '__main__':
    tags = sys.argv[1:] or ['6cm', '5cm', '4cm', '3cm']
    out = {}
    for tag in tags:
        r = analyze(tag)
        out[tag] = {k: v for k, v in r.items() if not k.startswith('_')}
    json.dump(out, open('metrics.json', 'w'), indent=1, default=float)
    print('wrote metrics.json')

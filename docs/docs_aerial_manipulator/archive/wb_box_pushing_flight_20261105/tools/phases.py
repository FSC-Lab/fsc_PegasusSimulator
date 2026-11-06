"""Per-phase vehicle tracking: horizontal CoM error (law: x_c - x_cd, debug [48..50]-[45..47]), its rate,
the claw error (planner arrival [2]), true tilt (mocap uav), battery, drone mocap jitter."""
import sys
import numpy as np


def R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def edges(z):
    out, last = {}, None
    for t, s in zip(z['status_t'], z['status']):
        k = ' '.join(str(s).split()[:2])
        if k != last:
            out.setdefault(k, t)
            last = k
    return out


rows = []
for f in sys.argv[1:]:
    z = np.load(f)
    e = edges(z)
    d = z['dbg']
    t = d[:, 0]
    err = d[:, 49:52] - d[:, 46:49]           # columns shifted by the time column
    eh = np.linalg.norm(err[:, :2], axis=1)
    entry = e.get('NOT PLANNED:', np.nan)
    gs, gsd = e['FLYING go_to_start'], e['DONE go_to_start']
    rd = e.get('FLYING ready', np.nan)
    rdd = e.get('DONE ready', np.nan)
    end = min([v for k, v in e.items() if k.startswith(('FLYING push', 'ABORTING'))] + [rdd + 3])
    win = {'hover (entry+2 s -> GTS)': (entry + 2, gs), 'Go To Start': (gs, gsd), 'hold above handle': (gsd, rd),
           'Ready descent': (rd, rdd), 'hand-over (-> push/abort)': (rdd, end)}
    a = z['arr']
    vb = z['vrpn_uav']
    tilt = np.degrees(np.arccos(np.clip([R(q)[2, 2] for q in vb[:, 4:8]], -1, 1)))
    bat = z['bat']
    print(f'== {f.split("/")[-1]}  battery at Go To Start {np.interp(gs, bat[:,0], bat[:,1]):.2f} V')
    for name, (a0, a1) in win.items():
        m = (t >= a0) & (t < a1)
        if m.sum() < 10:
            continue
        sp = np.linalg.norm(np.diff(err[m, :2], axis=0), axis=1) / np.maximum(np.diff(t[m]), 1e-3)
        sp = np.convolve(sp, np.ones(25) / 25, mode='same')     # ~0.1 s
        ma = (a[:, 0] >= a0) & (a[:, 0] < a1)
        mt = (vb[:, 0] >= a0) & (vb[:, 0] < a1)
        claw = f'{1e3*np.nanmedian(a[ma,3]):4.0f}/{1e3*np.nanmax(a[ma,3]):4.0f}' if ma.sum() else '   -/   -'
        print(f'   {name:27s} {a1-a0:6.1f} s  CoM err rms {1e3*np.sqrt(np.mean(eh[m]**2)):5.1f}  max {1e3*eh[m].max():5.1f} mm'
              f'  rate p95 {1e3*np.percentile(sp,95):4.0f} mm/s  | claw err med/max {claw} mm'
              f'  | tilt mean {tilt[mt].mean():4.1f} max {tilt[mt].max():4.1f} deg')

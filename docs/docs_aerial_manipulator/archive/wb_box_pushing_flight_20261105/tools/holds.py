"""Free holds with the 11-03 site_control definitions, on bx_extract npz files.

A hold = the law's CoM reference x_cd still (< 5 mm/s over 0.3 s) for >= 1.5 s, first 0.5 s dropped, z > 0.6 m, outside
the hand-over / contact (planner status FLYING ready/push, DONE ready/push left out). Per hold: site, base speed over the
planner's 0.3 s window (median / p90), CoM wander about the hold's mean (rms), the law's moment estimate [34..35]
(detrended rms). Site: table = base within x 0.25..1.4, y -0.8..0.65 and z > 1.25 (above the handle or after an abort);
mid-field otherwise.
"""
import sys
import numpy as np


def status_at(z, t):
    i = np.searchsorted(z['status_t'], t) - 1
    return str(z['status'][i]) if i >= 0 else ''


out = {}
for f in sys.argv[1:]:
    z = np.load(f)
    d = z['dbg']
    t, D = d[:, 0], d[:, 1:]
    xcd = D[:, 45:48]
    # reference speed over 0.3 s
    j = np.clip(np.searchsorted(t, t - 0.3), 0, len(t) - 1)
    vr = np.linalg.norm(xcd[:, :2] - xcd[j, :2], axis=1) / np.maximum(t - t[j], 1e-3)
    still = (vr < 0.005) & (t - t[j] > 0.25)
    b = z['base']
    tb, pb = b[:, 0], b[:, 1:4]
    jb = np.clip(np.searchsorted(tb, tb - 0.3), 0, len(tb) - 1)
    okv = (tb - tb[jb]) >= 0.25
    vb = np.where(okv, np.linalg.norm(pb[:, :2] - pb[jb, :2], axis=1) / np.maximum(tb - tb[jb], 1e-3), np.nan)
    # segments of `still`
    edges = np.flatnonzero(np.diff(np.r_[0, still.astype(int), 0]))
    for s, e in zip(edges[::2], edges[1::2]):
        a, c = t[s] + 0.5, t[e - 1]
        if c - a < 1.0:
            continue
        st = status_at(z, a)
        if st.startswith(('FLYING ready', 'FLYING push', 'DONE ready', 'DONE push')):
            continue
        ww = (t >= a) & (t <= c)
        w = (tb >= a) & (tb <= c) & okv
        if w.sum() < 5:
            continue
        x, y, zz = pb[w].mean(0)
        if zz < 0.6:
            continue
        site = 'table' if (0.25 < x < 1.4 and -0.8 < y < 0.65 and zz > 1.25) else 'mid-field'
        e3 = D[ww, 48:50] - D[ww, 45:47]
        e3 = e3 - e3.mean(0)
        M = D[ww, 34:36]
        M = M - M.mean(0)
        row = (f.split('/')[-1][:7], c - a, x, y, zz, np.nanmedian(vb[w]) * 1e3, np.nanpercentile(vb[w], 90) * 1e3,
               np.sqrt((e3 ** 2).sum(1).mean()) * 1e3, np.sqrt((M ** 2).sum(1).mean()) * 1e3, st.split(' --')[0][:40])
        out.setdefault((f.split('/')[-1][0], site), []).append(row)
for k in sorted(out):
    v = out[k]
    print(f"== {'10-27' if k[0]=='o' else '11-05'} {k[1]}: {len(v)} holds, {sum(r[1] for r in v):.0f} s | base speed median "
          f"{np.median([r[5] for r in v]):.0f} mm/s, p90 {np.median([r[6] for r in v]):.0f} | CoM wander median "
          f"{np.median([r[7] for r in v]):.1f} mm | moment estimate median {np.median([r[8] for r in v]):.0f} mN.m")
    for r in v:
        print(f"   {r[0]} {r[1]:5.1f}s ({r[2]:+.2f},{r[3]:+.2f},{r[4]:.2f})  v {r[5]:3.0f}/{r[6]:3.0f}  wander {r[7]:5.1f}  M {r[8]:4.0f}  [{r[9]}]")

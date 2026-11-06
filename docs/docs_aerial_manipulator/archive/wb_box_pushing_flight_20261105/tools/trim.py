"""Standing motor trim in still-reference holds (motors_debug, PX4 Quad-X order: 0 FR ccw, 1 RL ccw, 2 FL cw, 3 RR cw):
left-right, front-back, cw-ccw pair differences and the mean; plus mid-field CoM wander per flight (holds as holds.py)."""
import sys
import numpy as np

print(f"{'flight':9s} {'holds s':>7s} {'mean':>6s} {'L-R':>7s} {'F-B':>7s} {'CW-CCW':>7s} | {'mid-field wander (median of holds)':s}")
for f in sys.argv[1:]:
    z = np.load(f)
    d = z['dbg']
    t, D = d[:, 0], d[:, 1:]
    xcd = D[:, 45:48]
    j = np.clip(np.searchsorted(t, t - 0.3), 0, len(t) - 1)
    vr = np.linalg.norm(xcd[:, :2] - xcd[j, :2], axis=1) / np.maximum(t - t[j], 1e-3)
    still = (vr < 0.005) & (t - t[j] > 0.25)
    m = z['mot']
    im = np.clip(np.searchsorted(m[:, 0], t), 0, len(m) - 1)
    U = m[im, 1:5]
    edges = np.flatnonzero(np.diff(np.r_[0, still.astype(int), 0]))
    sel = np.zeros(len(t), bool)
    wander = []
    b = z['base']
    for s, e in zip(edges[::2], edges[1::2]):
        a, c = t[s] + 0.5, t[e - 1]
        if c - a < 1.0:
            continue
        ww = (t >= a) & (t <= c)
        ib = np.clip(np.searchsorted(b[:, 0], (a + c) / 2), 0, len(b) - 1)
        x, y, zz = b[ib, 1:4]
        if zz < 0.6:
            continue
        sel |= ww
        if not (0.25 < x < 1.4 and -0.8 < y < 0.65 and zz > 1.25):
            e3 = D[ww, 48:50] - D[ww, 45:47]
            e3 = e3 - e3.mean(0)
            wander.append((c - a, np.sqrt((e3 ** 2).sum(1).mean()) * 1e3))
    u = U[sel]
    lr = (u[:, 1] + u[:, 2] - u[:, 0] - u[:, 3]) / 2
    fb = (u[:, 0] + u[:, 2] - u[:, 1] - u[:, 3]) / 2
    yw = (u[:, 2] + u[:, 3] - u[:, 0] - u[:, 1]) / 2
    wtxt = ', '.join(f'{w:.0f} mm/{d_:.0f}s' for d_, w in wander)
    print(f"{f.split('/')[-1][:7]:9s} {sel.sum()/250:7.0f} {u.mean():6.3f} {lr.mean():+7.3f} {fb.mean():+7.3f} {yw.mean():+7.3f} | "
          f"{np.median([w for _, w in wander]) if wander else float('nan'):5.1f}  ({wtxt})")

import sys, numpy as np
A = '/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/'
runs = {'1015 e1': 'wb_pick_drop_flight_20261015/npz/e1.npz', '1015 d1': 'wb_pick_drop_flight_20261015/npz/d1.npz',
        '1015 d2': 'wb_pick_drop_flight_20261015/npz/d2.npz', '1016 e1': 'wb_pick_drop_flight_20261016/npz/e1.npz',
        '1016 d1': 'wb_pick_drop_flight_20261016/npz/d1.npz', '1016 d2': 'wb_pick_drop_flight_20261016/npz/d2.npz'}
def lp(x, t, tau):
    y = np.empty_like(x); y[0] = x[0]
    for k in range(1, len(x)):
        a = 1 - np.exp(-(t[k]-t[k-1])/tau); y[k] = y[k-1] + a*(x[k]-y[k-1])
    return y
rows = []
for name, f in runs.items():
    z = np.load(A+f, allow_pickle=True)
    tm = z['mocap__recv']; hm = z['mocap__hdr']
    pm = np.c_[z['mocap__pose.position.x'], z['mocap__pose.position.y'], z['mocap__pose.position.z']]
    qm = np.c_[z['mocap__pose.orientation.w'], z['mocap__pose.orientation.x'], z['mocap__pose.orientation.y'], z['mocap__pose.orientation.z']]
    vm = np.c_[z['mocap__twist.linear.x'], z['mocap__twist.linear.y'], z['mocap__twist.linear.z']]
    air = pm[:, 2] > 0.5
    if air.sum() < 100: continue
    i0, i1 = np.where(air)[0][[0, -1]]
    sl = slice(i0, i1+1)
    tm, hm, pm, qm, vm = tm[sl], hm[sl], pm[sl], qm[sl], vm[sl]
    gap = np.diff(tm)*1e3; hgap = np.diff(hm)*1e3
    # frozen: runs of bit-identical positions
    same = np.all(pm[1:] == pm[:-1], axis=1)
    run = mx = 0; dur = 0.0; mxd = 0.0; st = None
    for k, s in enumerate(same):
        if s:
            if run == 0: st = hm[k]
            run += 1; mx = max(mx, run+1); mxd = max(mxd, hm[k+1]-st)
        else: run = 0
    # jump: residual vs constant-velocity prediction (header time)
    dt = np.diff(hm); ok = dt > 1e-4
    pred = pm[:-1] + vm[:-1]*dt[:, None]
    res = np.linalg.norm(pm[1:]-pred, axis=1)[ok]*1e3
    step = np.linalg.norm(np.diff(pm, axis=0), axis=1)[ok]*1e3
    dq = np.abs(np.sum(qm[1:]*qm[:-1], axis=1)).clip(0, 1)
    ang = np.degrees(2*np.arccos(dq))[ok]
    # EKF2 (fused odom, Orin) vs raw mocap, on receive time, best shift
    to = z['odom__recv']; po = np.c_[z['odom__pose.pose.position.x'], z['odom__pose.pose.position.y'], z['odom__pose.pose.position.z']]
    m = (to > tm[0]+1) & (to < tm[-1]-1); to, po = to[m], po[m]
    best = None
    for tau in np.arange(-0.10, 0.101, 0.005):
        pi = np.c_[[np.interp(to - tau, tm, pm[:, j]) for j in range(3)]].T
        d = po - pi; r = np.sqrt(np.mean(np.sum(d**2, 1)))
        if best is None or r < best[0]: best = (r, tau, d)
    r, tau, d = best
    dn = np.linalg.norm(d, axis=1)*1e3
    dfast = d - np.c_[[lp(d[:, j], to, 2.0) for j in range(3)]].T
    dfn = np.linalg.norm(dfast, axis=1)*1e3
    rows.append((name, (tm[-1]-tm[0]), np.percentile(gap, 99.9), gap.max(), hgap.max(), mx, mxd*1e3,
                 np.percentile(res, 99.9), res.max(), np.percentile(step, 99.9), step.max(), ang.max(),
                 tau*1e3, np.percentile(dn, 50), np.percentile(dn, 99), dn.max(), np.percentile(dfn, 99), dfn.max()))
hdr = ('run', 'air s', 'gap p99.9', 'gap max', 'hdr gap max', 'identical max n', 'identical max ms',
       'res p99.9 mm', 'res max mm', 'step p99.9 mm', 'step max mm', 'rot step max deg',
       'EKF lag ms', '|d| p50 mm', '|d| p99 mm', '|d| max mm', 'fast|d| p99', 'fast|d| max')
print(' | '.join(hdr))
for r in rows:
    print(' | '.join([r[0]] + [f'{v:.1f}' if isinstance(v, float) else str(v) for v in r[1:]]))

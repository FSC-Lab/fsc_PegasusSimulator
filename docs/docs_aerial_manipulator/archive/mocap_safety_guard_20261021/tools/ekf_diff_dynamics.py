import numpy as np
A = '/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/'
runs = {'1015 e1': 'wb_pick_drop_flight_20261015/npz/e1.npz', '1015 d1': 'wb_pick_drop_flight_20261015/npz/d1.npz',
        '1015 d2': 'wb_pick_drop_flight_20261015/npz/d2.npz', '1016 e1': 'wb_pick_drop_flight_20261016/npz/e1.npz',
        '1016 d1': 'wb_pick_drop_flight_20261016/npz/d1.npz', '1016 d2': 'wb_pick_drop_flight_20261016/npz/d2.npz'}
lags = [0.1, 0.25, 0.5, 1.0, 2.0]
print('run | mocap hdr-recv ms (med) | odom hdr-recv ms (med, p1, p99) | hdr-aligned lag ms | |d| p50/p99/max mm | p99 |d(t+D)-d(t)| mm for D =', lags)
for name, f in runs.items():
    z = np.load(A+f, allow_pickle=True)
    tm, hm = z['mocap__recv'], z['mocap__hdr']
    pm = np.c_[z['mocap__pose.position.x'], z['mocap__pose.position.y'], z['mocap__pose.position.z']]
    to, ho = z['odom__recv'], z['odom__hdr']
    po = np.c_[z['odom__pose.pose.position.x'], z['odom__pose.pose.position.y'], z['odom__pose.pose.position.z']]
    air = pm[:, 2] > 0.5
    i0, i1 = np.where(air)[0][[0, -1]]
    tm, hm, pm = tm[i0:i1+1], hm[i0:i1+1], pm[i0:i1+1]
    mo = (to > tm[0]+1) & (to < tm[-1]-1); to, ho, po = to[mo], ho[mo], po[mo]
    a = (hm - tm)*1e3; b = (ho - to)*1e3
    best = None
    for tau in np.arange(-0.06, 0.0601, 0.002):
        pi = np.c_[[np.interp(ho - tau, hm, pm[:, j]) for j in range(3)]].T
        d = po - pi; r = np.sqrt(np.mean(np.sum(d**2, 1)))
        if best is None or r < best[0]: best = (r, tau, d)
    r, tau, d = best
    dn = np.linalg.norm(d, axis=1)*1e3
    inc = []
    for D in lags:
        j = np.searchsorted(ho, ho + D); ok = j < len(ho)
        inc.append(np.percentile(np.linalg.norm(d[j[ok]] - d[ok], axis=1)*1e3, 99))
    print(f"{name} | {np.median(a):.1f} | {np.median(b):.1f}, {np.percentile(b,1):.1f}, {np.percentile(b,99):.1f} | {tau*1e3:.0f} | "
          f"{np.percentile(dn,50):.1f}/{np.percentile(dn,99):.1f}/{dn.max():.1f} | " + ' '.join(f'{v:.1f}' for v in inc))

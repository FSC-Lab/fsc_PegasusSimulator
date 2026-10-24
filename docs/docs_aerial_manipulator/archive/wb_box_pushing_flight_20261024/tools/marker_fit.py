import numpy as np, struct
p = "/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Box_Assembly_Push.stl"
d = open(p, 'rb').read(); n = struct.unpack('<I', d[80:84])[0]
a = np.frombuffer(d, dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]), count=n, offset=84)
V = a['v'].reshape(-1, 3).astype(float)
key = np.round(V, 6); uniq, inv = np.unique(key, axis=0, return_inverse=True); inv = inv.ravel()
parent = np.arange(len(uniq))
def find(i):
    while parent[i] != i: parent[i] = parent[parent[i]]; i = parent[i]
    return i
for t in range(len(V)//3):
    a_, b_, c_ = inv[3*t:3*t+3]
    for x in (b_, c_):
        ra, rx = find(a_), find(x)
        if ra != rx: parent[rx] = ra
roots = np.array([find(i) for i in range(len(uniq))]); comp = roots[inv[0::3]]
box_c = np.array([-0.0044, 0.0101, 0.0169])
def sphere_fit(P):
    A = np.c_[2*P, np.ones(len(P))]; b = (P**2).sum(1)
    s = np.linalg.lstsq(A, b, rcond=None)[0]; c = s[:3]; r = np.sqrt(s[3] + c @ c)
    return c, r, np.abs(np.linalg.norm(P - c, axis=1) - r).max()
balls = []
for r in np.unique(comp):
    idx = np.where(comp == r)[0]
    if len(idx) != 6268: continue
    P = np.unique(np.round(V[np.concatenate([3*idx, 3*idx+1, 3*idx+2])], 7), axis=0)
    lo, hi = P.min(0), P.max(0); ax = np.argmax(hi - lo)
    # outward direction = away from the box centre along the long axis
    sgn = 1 if (lo[ax] + hi[ax]) / 2 > box_c[ax] else -1
    far = hi[ax] if sgn > 0 else lo[ax]
    best = None
    for frac in (0.25, 0.35, 0.45):
        L = (hi[ax] - lo[ax]) * frac
        sel = P[np.abs(P[:, ax] - far) <= L]
        c, rad, res = sphere_fit(sel)
        if best is None or res < best[2]: best = (c, rad, res, frac)
    balls.append(best[0])
    print(f"ball centre {np.round(best[0]*1e3,1)} mm  radius {best[1]*1e3:.1f} mm  fit residual {best[2]*1e3:.2f} mm")
B = np.array(balls); cen = B.mean(0)
print("marker centroid (CAD, mm):", np.round(cen*1e3, 1))
print("box centre      (CAD, mm):", np.round(box_c*1e3, 1))
print("centroid - box centre (CAD mm):", np.round((cen-box_c)*1e3, 1))

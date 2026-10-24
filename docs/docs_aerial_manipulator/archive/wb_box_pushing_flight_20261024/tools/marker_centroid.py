import numpy as np, struct
exec(open(__import__("os").path.join(__import__("os").path.dirname(__import__("os").path.abspath(__file__)), "marker_fit.py")).read().split("box_c =")[0])
box_c = np.array([-0.0044, 0.0101, 0.0169])
def sphere_fit(P):
    A = np.c_[2*P, np.ones(len(P))]; b = (P**2).sum(1)
    s = np.linalg.lstsq(A, b, rcond=None)[0]; c = s[:3]; r = np.sqrt(s[3] + c @ c)
    return c, r, np.abs(np.linalg.norm(P - c, axis=1) - r).max()
B = []
for r in np.unique(comp):
    idx = np.where(comp == r)[0]
    if len(idx) != 6268: continue
    P = np.unique(np.round(V[np.concatenate([3*idx, 3*idx+1, 3*idx+2])], 7), axis=0)
    lo, hi = P.min(0), P.max(0); ax = np.argmax(hi - lo)
    sgn = 1 if (lo[ax] + hi[ax]) / 2 > box_c[ax] else -1
    s = (P[:, ax] - (lo[ax] if sgn > 0 else hi[ax])) * sgn
    c, rad, res = sphere_fit(P[s >= 0.022])
    B.append(c); print(f"ball {np.round(c*1e3,1)} mm  r {rad*1e3:.1f}  resid {res*1e3:.3f}")
B = np.array(B); cen = B.mean(0)
fin_cad_y = 0.5 * (7.562 + 12.562) / 1e3          # fin mid-plane, CAD y (old converter's CAD frame)
print("centroid CAD mm", np.round(cen*1e3, 1), " box centre CAD mm", np.round(box_c*1e3, 1))
d = cen - box_c
# asset/obj_0 frame: x = -CAD y (across the fin), y = CAD x (along the push), z = z
print("centroid - box centre, box frame (x across, y along, z up) mm:", np.round(np.array([-d[1], d[0], d[2]])*1e3, 1))
print("fin mid-plane - box centre, across (box x) mm:", round(-(fin_cad_y - box_c[1])*1e3, 1))
print("fin mid-plane - centroid, across (box x) mm:", round(-(fin_cad_y - cen[1])*1e3, 1))

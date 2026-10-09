import numpy as np, glob, os
R = np.degrees
W = 3.0   # [s] window for "the lowest reference nearby"
rows = {"wb": [], "geo": []}
print(f"{'run':22s} rig  pose-region      worst dip below lowest nearby ref [deg]  where")
for f in sorted(glob.glob("../../pick_place_controllers_20261003/runs/wb_*.npz") + glob.glob("../../pick_place_controllers_20261003/runs/geo_*.npz") + glob.glob("../../pick_place_controllers_20261003/runs/bench_*.npz")):
    z = np.load(f, allow_pickle=True)
    if bool(z["aborted"]) or len(z["ref"]) < 100: continue
    J, Rf = z["joints"], z["ref"]; t = J[:, 0]; q2 = R(J[:, 2])
    tr = Rf[:, 0]; qd2 = R(Rf[:, 8])
    m = (t >= tr[0] + W) & (t <= tr[-1] - W)
    # rolling min of the reference over +-W s
    lo = np.array([qd2[(tr >= tt - W) & (tr <= tt + W)].min() for tt in t[m][::5]])
    tt = t[m][::5]; qq = q2[m][::5]
    dip = lo - qq                       # > 0: below the lowest commanded q2 within +-W
    names, mt = z["marks_name"], z["marks_t"]
    for region, sel in (("pick/place (-20)", np.abs(lo + 20) < 1.0), ("hold (-30)", np.abs(lo + 30) < 1.0)):
        if not sel.any(): continue
        k = np.argmax(np.where(sel, dip, -1e9))
        i = np.searchsorted(mt, tt[k]) - 1
        ph = str(names[i]) if i >= 0 else "-"
        rows[str(z["rig"])].append((region, dip[k]))
        print(f"{os.path.basename(f)[:-4]:22s} {str(z['rig']):4s} {region:16s} {dip[k]:6.2f}   ({tt[k]:5.1f} s, {ph})")
for rig, v in rows.items():
    for reg in ("pick/place (-20)", "hold (-30)"):
        d = [x[1] for x in v if x[0] == reg]
        if d: print(f"{rig:4s} {reg:16s} n={len(d):2d}  worst {max(d):5.2f}  median {np.median(d):5.2f} deg")

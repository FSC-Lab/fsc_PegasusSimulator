"""Score an Isaac interaction flight (interaction_force_driver.py npz).
Per force segment: chi latch/release, the controller's consumed task force
F_hat_y (debug [58..60], world) vs the force 07 applied, the EE task error
along the force (debug [24..26], CoM-anchored), CoM error, tilt, arm torque."""
import sys, json, numpy as np
d = np.load(sys.argv[1], allow_pickle=True)
wb = d["wb"]; wt = d["wb_t"]; fs = d["fs"]; od = d["od"]
te = float(d["t_entry"]); yaw0 = float(d["yaw0"])
ts = wt[:, 1] - te                         # sim s after DIRECT entry
ok = np.isfinite(ts) & (wb[:, 0] == 1)
fts = fs[:, 1] - te; Fap = fs[:, 2:5]
def F_at(t):
    return np.column_stack([np.interp(t, fts, Fap[:, i]) for i in range(3)])
c, s = np.cos(yaw0), np.sin(yaw0)
dirs = {"radial": (c, s, 0), "-radial": (-c, -s, 0), "lateral": (-s, c, 0), "-lateral": (s, -c, 0), "up": (0, 0, 1), "down": (0, 0, -1)}
# tilt from odometry
oq = od[:, 5:9]; ots = od[:, 1] - te
R22 = 1 - 2 * (oq[:, 1] ** 2 + oq[:, 2] ** 2); tilt = np.degrees(np.arccos(np.clip(R22, -1, 1)))
Fy = wb[:, 58:61]; ey = wb[:, 24:27]; chi = wb[:, 105]; tau = wb[:, 13:17]; nsat = wb[:, 51]
exc = wb[:, 48:51] - wb[:, 45:48]
out = {"t_direct_end": float(ts[ok].max()) if ok.any() else None}
print(f"DIRECT span {ts[ok].min():.1f}..{ts[ok].max():.1f} s sim, heading {np.degrees(yaw0):.1f} deg")
pre = ok & (ts > 8) & (ts < 14.5)
print(f"pre-contact hover: |e_x| rms {np.sqrt(np.mean(np.sum(exc[pre]**2,1)))*1e3:.1f} mm, chi free {np.mean(chi[pre]==1)*100:.0f}%, "
      f"|F_hat_y| max {np.abs(Fy[pre]).max():.2f}")
res = []
for sg in d["segs"]:
    t0, hold, dn, Fm = str(sg).split(":"); t0 = float(t0); hold = float(hold); Fm = float(Fm)
    u = np.asarray(dirs[dn], float)
    seg = ok & (ts >= t0 - 0.5) & (ts <= t0 + hold + 2 + 6)
    if not seg.any():
        print(f"{sg}: no DIRECT data (aborted before?)"); continue
    w = ok & (ts >= t0 + 1 + hold - 3) & (ts < t0 + 1 + hold)
    lat = np.where(seg & (chi == 0) & (ts >= t0))[0]
    t_on = ts[lat[0]] - t0 if len(lat) else None
    rel = np.where(seg & (chi == 1) & (ts > (ts[lat[0]] if len(lat) else 1e9)))[0]
    t_off = ts[rel[0]] - (t0 + 2 + hold) if len(rel) else None
    Fa = F_at(ts[w]) @ u
    r = dict(seg=str(sg), latch_after_s=t_on, release_after_end_s=t_off,
             F_applied=float(Fa.mean()) if w.any() else None,
             F_hat_consumed=float((Fy[w] @ u).mean()) if w.any() else None,
             ee_def_mm=float((ey[w] @ u).mean() * 1e3) if w.any() else None,
             pred_def_mm=float(Fa.mean() / 211.9 * 1e3) if w.any() else None,
             com_err_pk_mm=float(np.linalg.norm(exc[seg], axis=1).max() * 1e3),
             tilt_pk_deg=float(tilt[(ots >= t0 - 0.5) & (ots <= t0 + hold + 8)].max()) if len(ots) else None,
             tau_pk=np.abs(tau[seg]).max(axis=0).round(3).tolist(), sat_pct=float(np.mean(nsat[seg] > 0) * 100))
    post = ok & (ts >= t0 + 1 + hold) & (ts <= t0 + hold + 2 + 9)
    q = np.degrees(wb[:, 5:9])
    r["q_range_deg"] = [[round(float(q[seg, j].min()), 1), round(float(q[seg, j].max()), 1)] for j in range(4)]
    r["j2_pk_post"] = float(np.abs(tau[post, 1]).max()) if post.any() else None
    r["ey_pk_post_mm"] = float(np.linalg.norm(ey[post], axis=1).max() * 1e3) if post.any() else None
    r["ey_rms_post_mm"] = float(np.sqrt(np.mean(np.sum(ey[post] ** 2, 1))) * 1e3) if post.any() else None
    res.append(r)
    print(json.dumps(r))
try:
    modes = [tuple(x) for x in d["mode"]]
except Exception:
    modes = []
print("modes:", [(round(float(a[1]) - te, 1) if float(a[1]) > 0 else None, a[2]) for a in modes])
json.dump(res, open(sys.argv[1].replace(".npz", "_score.json"), "w"), indent=1)

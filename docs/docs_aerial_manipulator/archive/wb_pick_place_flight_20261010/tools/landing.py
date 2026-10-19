"""The end of PP-4: pilot takeover (STAB), descent, ground contact, tip-over; the arm's law and torques on the ground."""
from pp10_common import *
from scipy.spatial.transform import Rotation as Rot
d, t0 = load("p4")
tm, Pm, Qm, Vm = mocap_pose(d, t0, "mocap")
eul = Rot.from_quat(Qm[:, [1, 2, 3, 0]]).as_euler("ZYX", degrees=True); tilt = tilt_of(Qm)
vz = np.gradient(Pm[:, 2], tm); vh = np.linalg.norm(np.gradient(Pm[:, :2], tm, axis=0), axis=1)
k = 12; ker = np.ones(k) / k
vz = np.convolve(vz, ker, "same"); vh = np.convolve(vh, ker, "same")
tv = d["vstatus__recv"] - t0
print("nav_state:", [(round(float(a), 2), int(b)) for a, b in zip(tv, d["vstatus__nav_state"]) if a > 72 and a < 80])
print("arming   :", [(round(float(a), 2), int(b)) for a, b in zip(tv, d["vstatus__arming_state"]) if a > 72 and a < 80])
tj, qj, _ = joints_model(d, t0)
te = d["js__recv"] - t0; eff = d["js__effort"][:, :4][:, JS_IDX] * HW_SIGN / KT
tl = d["law__recv"] - t0; L = d["law__data"]
tb = d["batt__recv"] - t0
print(" t      x      y      z    vz    vh   yaw  pitch  roll  tilt | q1..q4 deg | applied tau j1..j4 N.m | law(0 hold/1 pass) | battV")
for ts in np.arange(74.0, 89.5, 0.2 if True else 0.1):
    i = np.searchsorted(tm, ts); j = np.searchsorted(tj, ts); e = np.searchsorted(te, ts); l = min(np.searchsorted(tl, ts), len(tl) - 1); b = min(np.searchsorted(tb, ts), len(tb) - 1)
    if ts > 79 and (round(ts * 10) % 10):
        continue
    print(f"{tm[i]:6.2f} {Pm[i,0]:6.3f} {Pm[i,1]:6.3f} {Pm[i,2]:6.3f} {vz[i]:5.2f} {vh[i]:5.2f} {eul[i,0]:6.1f} {eul[i,1]:6.1f} {eul[i,2]:6.1f} {tilt[i]:5.1f} | "
          f"{np.round(np.degrees(qj[j]),1)} | {np.round(eff[e],2)} | {int(L[l,0])} | {d['batt__voltage_v'][b]:.2f}")
m = (tm > 74.8) & (tm < 78)
print("max z after takeover", Pm[m, 2].max(), "at", tm[m][np.argmax(Pm[m, 2])], "| min vz", vz[m].min(), "at", tm[m][np.argmin(vz[m])], "| max tilt", tilt[m].max(), "at", tm[m][np.argmax(tilt[m])])
g = (tm > 78.0) & (tm < 82.0)
print("rest pose 78-82 s:", Pm[g].mean(0).round(3), "tilt", tilt[g].mean().round(1), "pitch/roll", eul[g, 1].mean().round(1), eul[g, 2].mean().round(1))
me = (te > 77.2) & (te < 87.0)
print("applied arm torque on the ground 77.2-87 s: max |.|", np.abs(eff[me]).max(0).round(2), "N.m; mean |.|", np.abs(eff[me]).mean(0).round(2), "; seconds with |tau2| > 2.0:", float((np.abs(eff[me][:, 1]) > 2.0).sum() * 0.004), "|tau3| > 1.2:", float((np.abs(eff[me][:, 2]) > 1.2).sum() * 0.004))

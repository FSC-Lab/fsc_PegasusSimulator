"""The arm controller's position guard on joint 2 (max_position 45 deg, hardware yaml since 2026-10-09) in PP-4:
when q2 > 45 deg and the streamed torque is positive, external_torque_controller writes its own kp/kd pull-back instead
of the whole-body law's torque, and the law is not told.  Compares the streamed torque (law_debug[9..12], joint order of
the controller) with the written command (law_debug[13..16], duty counts) to find those ticks.
usage: q2_guard.py [run]   Writes ../analysis/q2_guard_<run>.json"""
import json
from pp10_common import *
nm = sys.argv[1] if len(sys.argv) > 1 else "p4"
d, t0 = load(nm)
tl = d["law__recv"] - t0; L = d["law__data"]
tj, qj, qdj = joints_model(d, t0)
print("joint_states names:", d["js__name"][0] if "js__name" in d.files else None)
q2 = np.degrees(interp(tl, tj, qj[:, 1])); q3 = np.degrees(interp(tl, tj, qj[:, 2])); q1 = np.degrees(interp(tl, tj, qj[:, 0])); q4 = np.degrees(interp(tl, tj, qj[:, 3]))
pt = L[:, 0] > 0.5
# controller joint order: find which of the 4 slots is j2 by correlating tau_ext with the law's tau (wb debug, model order j1..j4)
tw, D = wbdebug(d, t0)
tau_law = interp(tl, tw, D[:, WB["tau"]])
ext = L[:, 9:13]; cmd = L[:, 13:17]
m = pt & (tl > tw[0] + 1) & (tl < tw[-1] - 1)
C = np.array([[np.corrcoef(ext[m, i], tau_law[m, j])[0, 1] for j in range(4)] for i in range(4)])
print("corr(tau_ext slot i, law tau j):\n", C.round(2))
slot = [int(np.argmax(np.abs(C[:, j]))) for j in range(4)]; sign = [float(np.sign(C[slot[j], j])) for j in range(4)]
print("slot of j1..j4:", slot, "sign:", sign)
s2 = slot[1]
kpwm = KPWM[1]
# expected written command without a guard ~ kappa_pwm * (tau_ext + gravity corr + aux) ; compare via regression on the quiet part
exp = ext[:, s2] * kpwm * (1.0) + L[:, 22 + s2] + L[:, 18 + s2]
res = cmd[:, s2] - exp
quiet = m & (q2 < 43.0)
print(f"j2: written - expected, q2 < 43 deg: mean {res[quiet].mean():.1f} std {res[quiet].std():.1f} counts (n {quiet.sum()})")
over = m & (q2 > 45.0)
near = m & (q2 > 44.0)
print(f"q2 > 44 deg: {near.sum() / m.sum() * 100:.2f} % of pass-through ticks ({near.sum() * np.median(np.diff(tl)):.2f} s); q2 > 45: {over.sum() * np.median(np.diff(tl)):.2f} s; max q2 {q2[m].max():.2f}")
# episodes with q2 > 44.5
ep = []; i = 0; thr = 44.5
idx = np.where(m)[0]
on = (q2 > thr) & m
k = 0
while k < len(tl):
    if on[k]:
        j = k
        while j < len(tl) and on[j]:
            j += 1
        ep.append((float(tl[k]), float(tl[j - 1]), float(q2[k:j].max()), float(ext[k:j, s2].mean()), float(cmd[k:j, s2].mean() / kpwm), float(res[k:j].mean()), float(np.abs(res[k:j]).max())))
        k = j
    else:
        k += 1
print(f"episodes with q2 > {thr} deg (t0, t1, max q2, streamed tau2 mean [N.m], written/kappa mean [N.m], written-expected mean / max |.| [counts]):")
for e in ep:
    print("   %.2f-%.2f  q2max %.2f  tau_ext %.2f  written %.2f  res %.0f / %.0f" % e)
json.dump(dict(max_q2=float(q2[m].max()), s_over_44=float(near.sum() * np.median(np.diff(tl))), s_over_45=float(over.sum() * np.median(np.diff(tl))),
               episodes=ep), open(os.path.join(OUT10, f"q2_guard_{nm}.json"), "w"), indent=1)
if len(sys.argv) > 3:
    a, b = float(sys.argv[2]), float(sys.argv[3])
    print(" t      q1    q2    q3    q4 | tau_ext j2  written/k j2 | law tau j1..j4")
    for ts in np.arange(a, b, 0.1):
        i = np.searchsorted(tl, ts)
        print(f"{tl[i]:6.2f} {q1[i]:5.1f} {q2[i]:5.1f} {q3[i]:5.1f} {q4[i]:5.1f} | {ext[i, s2]:6.2f} {cmd[i, s2] / kpwm:6.2f} | {np.round(tau_law[i], 2)}")

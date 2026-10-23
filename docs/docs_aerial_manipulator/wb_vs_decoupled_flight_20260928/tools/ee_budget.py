"""Exact split of the EE position error into its three sources, every flight, circle run:
    r_e - r_ed = (x_b - x_b,ref)                              airframe position
               + (R0 - R0,ref) r_0e(q_d)                      airframe attitude (mostly yaw on the decoupled rig)
               + R0 (r_0e(q) - r_0e(q_d))                     arm joints
(the planner stream satisfies r_ed = x_b,ref + R0,ref r_0e(q_d) to 0.001 mm, bridge log). rms of the norm of each
term and of the sum; the terms are not orthogonal, so their rms values do not add. Writes ../analysis/ee_budget.json."""
import json
from common import *
import metrics as M
res = {}
for nm in ("w1", "w2", "d1", "d2", "a1", "a2", "a3", "a4"):
    o, S = M.analyse(nm)
    d, t0 = load(nm)
    n = len(S["t"])
    tr, ref = wbref(d, t0); R = {k: interp(S["t"], tr, v) for k, v in ref.items()}
    xb, R0r, _ = ref_base(R)
    to, P, Q, V, W = odom(d, t0); Qu = quat_cont(interp(S["t"], to, Q)); Qu /= np.linalg.norm(Qu, axis=1)[:, None]
    _, re, _, R0m = fk_world(S["Pu"], Qu, S["q"])
    ea = np.zeros((n, 3)); ej = np.zeros((n, 3))
    for k in range(n):
        _, r0e_d, _ = TP.arm_fk_model(R["q_d"][k], PARAMS); _, r0e_m, _ = TP.arm_fk_model(S["q"][k], PARAMS)
        ea[k] = (R0m[k] - R0r[k]) @ r0e_d; ej[k] = R0m[k] @ (r0e_m - r0e_d)
    eb = S["Pu"] - xb; tot = re - R["r_ed"]
    chk = np.abs(tot - (eb + ea + ej)).max()
    nr = lambda X: float(1e3 * rms(np.linalg.norm(X, axis=1)))
    res[nm] = dict(total=nr(tot), base=nr(eb), attitude=nr(ea), joints=nr(ej), closure_mm=float(1e3 * chk))
    print(f"{FLIGHTS[nm]:14s} EE {res[nm]['total']:6.1f} mm rms = airframe {res[nm]['base']:6.1f} + attitude {res[nm]['attitude']:5.1f} + joints {res[nm]['joints']:5.1f}   (closure {res[nm]['closure_mm']:.2e} mm)")
json.dump(res, open("../analysis/ee_budget.json", "w"), indent=1)

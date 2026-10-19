"""PP-4: what the vehicle carried (the 10-09 payload_budget gauges on p4) and how close the arm ran to its torque caps.
Writes ../analysis/load_p4.json."""
import json
from pp10_common import *
import payload_budget as PBm

X = PBm.build("p4"); t = X["t"]; d, t0 = X["d"], X["t0"]
W = dict(pre=(26.0, 30.4), after_lift=(33.9, 34.3), place_start=(50.5, 52.5), above_place=(62.6, 65.6), trim_hold=(67.6, 71.2))
win = {}
for k, (a, b) in W.items():
    m = (t >= a) & (t < b); f = lambda x: float(np.nanmean(x[m]))
    win[k] = dict(t=[a, b], u1=f(X["u1"]), T_fit=f(X["T_fit"]), P=f(X["Pw"]), mot=f(X["mot"].mean(1)), V=f(X["V"]), I=f(X["I"]), dz=f(X["dh"][:, 2]),
                  dr=np.nanmean(X["dh"][m, 3:6], axis=0).round(3).tolist(), dt_=np.nanmean(X["dh"][m, 0:3], axis=0).round(3).tolist(),
                  app=X["app"][m].mean(0).round(3).tolist(), res=X["res"][m].mean(0).round(3).tolist(), lev=X["lev"][m].mean(0).round(3).tolist())
    win[k]["ratio"] = win[k]["u1"] / win[k]["T_fit"]
    print(k, json.dumps(win[k]))
out = dict(windows=win, gauges={}, moment={})
W0 = 3.74617 * 9.80665
for k in ("above_place", "trim_hold"):
    dT = win[k]["T_fit"] - win["pre"]["T_fit"]; pr = win[k]["P"] / win["pre"]["P"]
    out["gauges"][k] = dict(fit_N=dT, power_N=[(pr ** (2 / 3) - 1) * win["pre"]["T_fit"], (pr ** (2 / 3) - 1) * W0], d_u1=win[k]["u1"] - win["pre"]["u1"], power_ratio=pr)
    m = (t >= W[k][0]) & (t < W[k][1]); q = X["q"][m].mean(0)
    r0c, r0e, _ = TP.arm_fk_model(q, C.PARAMS); lever = float((r0e - r0c)[1])
    dM = win[k]["dr"][0] - win["pre"]["dr"][0]
    out["moment"][k] = dict(d_hat_r_x=win[k]["dr"][0], baseline=win["pre"]["dr"][0], dM_alloc=dM, dM_delivered=dM / win[k]["ratio"], lever=lever, F=-dM / win[k]["ratio"] / lever)
print(json.dumps(out["gauges"], indent=0)); print(json.dumps(out["moment"], indent=0))

# ---- the arm controller's caps: commands written (duty counts) against max_effort
CAPS = np.array([57.6117, 364.8743, 192.0391, 57.6117])           # external_torque_controller_hardware_aerial_pwm.yaml, joint1..4
tl = d["law__recv"] - t0; L = d["law__data"]
pt = L[:, 0] > 0.5
cmd = L[:, 13:17]                                                  # written command, controller joint order (j1..j4, see q2_guard.py)
ext = L[:, 9:13]; aux = L[:, 18:22]; grav = L[:, 22:26]
nominal = np.abs(ext) * KPWM + 0.0
torque_term = ext * np.array([-1, 1, 1, -1]) * 0 + (cmd)          # the written duty also carries the back-EMF feed-forward, added after the clamp
tj, qj, _ = joints_model(d, t0)
tot = ext * KPWM + grav + aux                                      # the torque term the clamp acts on (streamed + gravity correction + friction), duty counts
caps = {}
for a, b, lab in ((6.0, 30.5, "before the lift"), (31.5, 72.6, "with the basket")):
    m = pt & (tl >= a) & (tl <= b)
    share = np.abs(tot[m]) / CAPS
    caps[lab] = dict(max_share=share.max(0).round(3).tolist(), ticks=int(m.sum()), clipped_ticks=[int((share[:, j] >= 0.999).sum()) for j in range(4)],
                     s_clipped=[float((share[:, j] >= 0.999).sum() * 0.004) for j in range(4)], s_above_90=[float((share[:, j] >= 0.9).sum() * 0.004) for j in range(4)],
                     streamed_max_nm=np.abs(ext[m]).max(0).round(2).tolist())
    print(lab, json.dumps(caps[lab]))
# when: the clipped ticks of j2 / j3 by phase
ph = pp_phases(d, t0)
m = pt & (tl >= 31.5) & (tl <= 72.6); sh = np.abs(tot) / CAPS
for a, b, l in ph:
    mm_ = m & (tl >= a) & (tl < b)
    if mm_.sum() > 50:
        print(f"   {a:6.1f}-{b:6.1f} {l[:42]:42s} j2 >=cap {(sh[mm_, 1] >= 0.999).sum() * 0.004:5.2f} s, j3 >=cap {(sh[mm_, 2] >= 0.999).sum() * 0.004:5.2f} s, max share j2 {sh[mm_, 1].max():.2f} j3 {sh[mm_, 2].max():.2f}")
out["caps"] = caps
te = d["js__recv"] - t0; cur = np.abs(d["js__effort"][:, :4][:, JS_IDX])
mc = (te > 31.5) & (te < 72.6)
out["current_counts_loaded"] = dict(mean=cur[mc].mean(0).round(0).tolist(), peak=cur[mc].max(0).round(0).tolist(), seconds=float(mc.sum() * 0.004))
print("servo current, loaded, counts mean/peak j1..j4:", out["current_counts_loaded"])
json.dump(out, open(os.path.join(OUT10, "load_p4.json"), "w"), indent=1)

"""Section 1.3 numbers: the body-fixed lateral force both controllers see, what each law does with it, and the
resulting airframe offset; plus the heading lag. Circle run (trimmed 2 s each end) and the start hold.
  whole-body  d_hat_t = wb_control_debug[31..33] (world, N) rotated into the body frame (FLU)
  decoupled   gamma_um = l1_control_debug[24..25] (body, N) -- the unmatched estimate the law does not cancel; its
              position loop balances it: Kp e_p + Kv e_v with Kp 4 N/m, Kv 6 N s/m (params_..._geometric_l1_..._t650.yaml)
Writes ../analysis/mechanism.json."""
import json
from common import *
from scipy.spatial.transform import Rotation as Rot
from arm_stats import hold_window
KP = np.array([4.0, 4.0, 8.0]); KV = np.array([6.0, 6.0, 10.0])
res = {}
for nm in ("w1", "w2", "d1", "d2"):
    d, t0 = load(nm); a, b, c0, c1 = windows(nm); to, P, Q, V, W = odom(d, t0)
    def body(tq, X):
        Qi = quat_cont(interp(tq, to, Q)); Qi /= np.linalg.norm(Qi, axis=1)[:, None]
        R = Rot.from_quat(Qi[:, [1, 2, 3, 0]]).as_matrix(); return np.einsum("nji,nj->ni", R, X)
    r = {}
    if nm[0] == "w":
        tw = d["wb__recv"] - t0; D = d["wb__data"]; m = (tw > c0 + 2) & (tw < c1 - 2)
        r["force_body_N"] = body(tw[m], D[m, 31:34])[:, :2].mean(0).tolist()
    else:
        tl = d["l1__recv"] - t0; L = d["l1__data"]; m = (tl > c0 + 2) & (tl < c1 - 2)
        r["force_body_N"] = L[m, 24:26].mean(0).tolist()
        epb = body(tl[m], L[m, 0:3]); evb = body(tl[m], L[m, 3:6])
        r["kp_ep_kv_ev_body_N"] = (KP * epb + KV * evb)[:, :2].mean(0).tolist()
    # airframe error in the body frame, circle run
    tr, ref = wbref(d, t0); tu = np.arange(c0 + 2, c1 - 2, 0.01); R = {k: interp(tu, tr, v) for k, v in ref.items()}
    xb, R0, yaw_ref = ref_base(R); Pu = interp(tu, to, P)
    eb = body(tu, Pu - xb); r["airframe_err_body_mm"] = (1e3 * eb[:, :2].mean(0)).tolist()
    Qu = quat_cont(interp(tu, to, Q)); Ra = Rot.from_quat((Qu / np.linalg.norm(Qu, axis=1)[:, None])[:, [1, 2, 3, 0]]).as_matrix()
    r["yaw_err_mean_deg"] = float(np.degrees(wrap(np.arctan2(Ra[:, 1, 0], Ra[:, 0, 0]) - np.unwrap(yaw_ref))).mean())
    r["yaw_rate_ref_deg_s"] = float(np.degrees(np.gradient(np.unwrap(yaw_ref), 0.01)).mean())
    h0, h1 = hold_window(nm, c0); th = np.arange(h0 + 0.3, h1 - 0.1, 0.01); Rh = {k: interp(th, tr, v) for k, v in ref.items()}
    xbh, _, _ = ref_base(Rh); r["hold_offset_mm"] = float(1e3 * np.linalg.norm((interp(th, to, P) - xbh).mean(0)))
    res[nm] = r
    print(FLIGHTS[nm], {k: (np.round(v, 3).tolist() if isinstance(v, list) else round(v, 2)) for k, v in r.items()})
json.dump(res, open("../analysis/mechanism.json", "w"), indent=1)

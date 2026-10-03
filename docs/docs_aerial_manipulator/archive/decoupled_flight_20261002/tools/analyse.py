"""Tables of the 1002 section: full-state RMSE (metrics.json), the exact EE error split (ee_budget.json) and the
lateral-force / heading-lag check against the new gains (mechanism.json), for the two circle runs (r1, r2) and the
PS4 teleoperation flight (tp). Definitions are the 0928 report's (metrics.analyse, ee_budget.py, mechanism.py).

    AM_NPZ=<npz dir> PYTHONNOUSERSITE=1 /usr/bin/python3 analyse.py
"""
import json
from c1002 import *
import metrics as M
from scipy.spatial.transform import Rotation as Rot

OUT = "../analysis"
KR_Z, KW_Z = 1.737, 0.4105            # 2026-10-01 tune, l1geo_kr_z / l1geo_komega_z


def budget(nm, S):
    d, t0 = load(nm); n = len(S["t"])
    tr, ref = wbref(d, t0); R = {k: interp(S["t"], tr, v) for k, v in ref.items()}
    xb, R0r, _ = ref_base(R)
    to, P, Q, V, W = odom(d, t0); Qu = quat_cont(interp(S["t"], to, Q)); Qu /= np.linalg.norm(Qu, axis=1)[:, None]
    _, re, _, R0m = fk_world(S["Pu"], Qu, S["q"])
    ea = np.zeros((n, 3)); ej = np.zeros((n, 3))
    for k in range(n):
        _, r0e_d, _ = TP.arm_fk_model(R["q_d"][k], PARAMS); _, r0e_m, _ = TP.arm_fk_model(S["q"][k], PARAMS)
        ea[k] = (R0m[k] - R0r[k]) @ r0e_d; ej[k] = R0m[k] @ (r0e_m - r0e_d)
    eb = S["Pu"] - xb; tot = re - R["r_ed"]
    nr = lambda X: float(1e3 * rms(np.linalg.norm(X, axis=1)))
    return dict(total=nr(tot), base=nr(eb), attitude=nr(ea), joints=nr(ej), closure_mm=float(1e3 * np.abs(tot - (eb + ea + ej)).max()))


def mechanism(nm, trim=2.0):
    d, t0 = load(nm); a, b, c0, c1 = windows(nm); to, P, Q, V, W = odom(d, t0)

    def body(tq, X):
        Qi = quat_cont(interp(tq, to, Q)); Qi /= np.linalg.norm(Qi, axis=1)[:, None]
        return np.einsum("nji,nj->ni", Rot.from_quat(Qi[:, [1, 2, 3, 0]]).as_matrix(), X)
    tl = d["l1__recv"] - t0; L = d["l1__data"]; m = (tl > c0 + trim) & (tl < c1 - trim)
    r = dict(force_body_N=L[m, 24:26].mean(0).tolist())
    r["kp_ep_kv_ev_body_N"] = (KP_NEW * body(tl[m], L[m, 0:3]) + KV_NEW * body(tl[m], L[m, 3:6]))[:, :2].mean(0).tolist()
    tr, ref = wbref(d, t0); tu = np.arange(c0 + trim, c1 - trim, 0.01); R = {k: interp(tu, tr, v) for k, v in ref.items()}
    xb, R0, yaw_ref = ref_base(R)
    r["airframe_err_body_mm"] = (1e3 * body(tu, interp(tu, to, P) - xb)[:, :2].mean(0)).tolist()
    Qu = quat_cont(interp(tu, to, Q)); Ra = Rot.from_quat((Qu / np.linalg.norm(Qu, axis=1)[:, None])[:, [1, 2, 3, 0]]).as_matrix()
    r["yaw_err_mean_deg"] = float(np.degrees(wrap(np.arctan2(Ra[:, 1, 0], Ra[:, 0, 0]) - np.unwrap(yaw_ref))).mean())
    r["yaw_rate_ref_deg_s"] = float(np.degrees(np.gradient(np.unwrap(yaw_ref), 0.01)).mean())
    r["yaw_lag_pred_deg"] = KW_Z / KR_Z * r["yaw_rate_ref_deg_s"]
    r["offset_pred_mm"] = float(1e3 * np.linalg.norm(r["force_body_N"]) / KP_NEW[0])
    st = [e for e in edges(d, t0, "pl_status") if e[0] < c0]
    h0 = [t for t, v in st if v.startswith("HOLD")][-1]
    th = np.arange(h0 + 0.3, c0 - 0.1, 0.01); Rh = {k: interp(th, tr, v) for k, v in ref.items()}
    xbh, _, _ = ref_base(Rh); r["hold_offset_mm"] = float(1e3 * np.linalg.norm((interp(th, to, P) - xbh).mean(0)))
    r["hold_window"] = [h0, c0]
    return r


if __name__ == "__main__":
    met, bud, mech = {}, {}, {}
    for nm in ("r1", "r2", "tp"):
        o, S = M.analyse(nm); met[nm] = o
        d, t0 = load(nm); tb = d["batt__recv"] - t0; w = o["window"]
        o["metrics"]["pack_v"] = float(d["batt__voltage_v"][(tb > w[0]) & (tb < w[1])].mean())
        bud[nm] = budget(nm, S)
        if nm != "tp":
            mech[nm] = mechanism(nm)
        m = o["metrics"]
        print(f"{FLIGHTS[nm]:11s} {w[0]:.2f}-{w[1]:.2f} V {m['pack_v']:.2f} | EE {m['ee_pos']['rms_norm']:.1f} mm head {m['ee_head']['rms']:.2f} | "
              f"airframe {m['base_pos']['rms_norm']:.1f} CoM {m['com_pos']['rms_norm']:.1f} | budget {bud[nm]}")
        if nm in mech:
            print("   mechanism", {k: (np.round(v, 3).tolist() if isinstance(v, list) else round(v, 3)) for k, v in mech[nm].items()})
    json.dump(met, open(f"{OUT}/metrics.json", "w"), indent=1)
    json.dump(bud, open(f"{OUT}/ee_budget.json", "w"), indent=1)
    json.dump(mech, open(f"{OUT}/mechanism.json", "w"), indent=1)

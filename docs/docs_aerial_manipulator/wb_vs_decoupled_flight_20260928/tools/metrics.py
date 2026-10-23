"""Full-state tracking metrics over the EE-circle run (the planner's EXECUTING T=28 s window), every flight.

One definition for both rigs, computed from signals both record (not from either law's debug array):
  measured   odometry (EKF2-fused position, world velocity, attitude, body rates) + joint_states (model convention)
             -> CoM x_c, EE r_e and EE heading b1_e by the planner's own model FK (transition_planner.arm_fk_model)
  reference  the planner's WholeBodyReference stream (the same plan feeds both rigs):
             CoM x_cd; airframe x_b = x_cd - R0 r_0c(q_d) and yaw = atan2(b1_d) + 90 deg (the decoupled bridge's
             conversion, reproduced to 0.4 mm); v_b = d/dt x_b; attitude R0 = the plan's compatible attitude
             (thrust along x_cd_ddot + g, heading b1_d); omega_ref from R0; joints q_d, qdot_d; EE r_ed, b1_de
  grid       100 Hz uniform, circle window trimmed 0.1 s at each end; joint velocity = 0.1 s central difference of
             the measured joint angle (joint_states' Present Velocity lags ~50 ms, so it is not used)
  attitude   error rotation R_ref^T R (body axes, log map) -> roll / pitch / yaw [deg]; EE heading = azimuth of the
             FK gripper heading minus that of b1_de [deg]
Writes ../analysis/metrics.json and prints the table.
"""
import json, math
from common import *
from scipy.spatial.transform import Rotation as Rot

RZ90 = TP._Rz(0.5 * math.pi)


def cdiff(X, dt, h):
    Y = (np.roll(X, -h, 0) - np.roll(X, h, 0)) / (2 * h * dt)
    Y[:h] = Y[h]; Y[-h:] = Y[-h - 1]
    return Y


def analyse(nm, trim=0.1):
    d, t0 = load(nm); a, b, c0, c1 = windows(nm)
    dt = 0.01; tu = np.arange(c0 + trim, c1 - trim, dt)
    to, P, Q, V, W = odom(d, t0)
    tj, qj, _ = joints_model(d, t0)
    tr, ref = wbref(d, t0)
    Pu = interp(tu, to, P); Qu = quat_cont(interp(tu, to, Q)); Qu /= np.linalg.norm(Qu, axis=1)[:, None]
    Vu = interp(tu, to, V); Wu = interp(tu, to, W); qu = interp(tu, tj, qj)
    qdu = cdiff(interp(tu, tj, qj), dt, 5)
    R = {k: interp(tu, tr, v) for k, v in ref.items()}
    xb, R0, yaw_ref = ref_base(R)
    vb = cdiff(xb, dt, 2)
    xc, re, b1e, R0m = fk_world(Pu, Qu, qu)
    Ra = Rot.from_quat(Qu[:, [1, 2, 3, 0]]).as_matrix()
    Rref = np.einsum("nij,jk->nik", R0, RZ90)                     # the plan's attitude, actual (x-forward) body frame
    E = np.einsum("nji,njk->nik", Rref, Ra)                         # R_ref^T R
    eatt = np.degrees(Rot.from_matrix(E).as_rotvec())              # body-axis roll / pitch / yaw error
    dR = cdiff(Rref.reshape(len(tu), 9), dt, 2).reshape(-1, 3, 3)
    Om = np.einsum("nji,njk->nik", Rref, dR); wref = np.column_stack([Om[:, 2, 1], Om[:, 0, 2], Om[:, 1, 0]])
    yaw = np.arctan2(Ra[:, 1, 0], Ra[:, 0, 0])
    ez = lambda x: x
    out = dict(name=nm, label=FLIGHTS[nm], window=[c0, c1], direct=[a, b], n=len(tu))
    e = {}
    e["com_pos"] = xc - R["x_cd"]
    e["base_pos"] = Pu - xb
    e["base_vel"] = Vu - vb
    e["att"] = eatt
    e["rate"] = np.degrees(Wu - np.einsum("nji,nj->ni", np.einsum("nij,njk->nik", np.transpose(Ra, (0, 2, 1)), Rref), wref) * 0 - wref) if False else np.degrees(Wu - wref)
    e["yaw_world"] = np.degrees(wrap(yaw - np.unwrap(yaw_ref)))
    e["joint"] = np.degrees(qu - R["q_d"])
    e["joint_vel"] = np.degrees(qdu - R["qdot_d"])
    e["ee_pos"] = re - R["r_ed"]
    az_m = np.arctan2(b1e[:, 1], b1e[:, 0]); az_r = np.arctan2(R["b1_de"][:, 1], R["b1_de"][:, 0])
    e["ee_head"] = np.degrees(wrap(az_m - az_r))
    cosang = np.clip(np.sum(b1e * R["b1_de"], 1) / np.linalg.norm(b1e, axis=1) / np.linalg.norm(R["b1_de"], axis=1), -1, 1)
    e["ee_head3d"] = np.degrees(np.arccos(cosang))
    r = {}
    for k in ("com_pos", "base_pos", "base_vel", "ee_pos"):
        s = 1e3
        r[k] = dict(rms_xyz=(s * rms(e[k])).tolist(), rms_norm=float(s * rms(np.linalg.norm(e[k], axis=1))),
                    mean_xyz=(s * e[k].mean(0)).tolist(), max_norm=float(s * np.linalg.norm(e[k], axis=1).max()))
    for k in ("att", "rate", "joint", "joint_vel"):
        r[k] = dict(rms=rms(e[k]).tolist(), mean=e[k].mean(0).tolist(), maxabs=np.abs(e[k]).max(0).tolist())
    for k in ("yaw_world", "ee_head", "ee_head3d"):
        r[k] = dict(rms=float(rms(e[k])), mean=float(e[k].mean()), maxabs=float(np.abs(e[k]).max()))
    # tilt, EE lag fit (pure time shift of the reference that minimises the mean |e|) and the residual after it
    tilt = np.degrees(np.arccos(np.clip(Ra[:, 2, 2], -1, 1))); r["tilt_max_deg"] = float(tilt.max())
    best = (0.0, np.linalg.norm(e["ee_pos"], axis=1).mean())
    for lag in np.arange(0.0, 3.0, 0.02):
        red = interp(tu - lag, tr, ref["r_ed"]); m = np.linalg.norm(re - red, axis=1).mean()
        if m < best[1]: best = (float(lag), float(m))
    r["ee_lag_s"] = best[0]; r["ee_resid_after_lag_mm"] = 1e3 * best[1]
    r["ee_mean_mm"] = float(1e3 * np.linalg.norm(e["ee_pos"], axis=1).mean())
    # realised EE circle: radius and height of the measured EE path
    rad = np.linalg.norm(re[:, :2], axis=1); r["ee_radius_meas_mean_m"] = float(rad.mean()); r["ee_z_std_mm"] = float(1e3 * re[:, 2].std())
    # law-internal attitude error |e_R| (each law vs its OWN desired attitude)
    if "wb__recv" in d.files:
        tw = d["wb__recv"] - t0; D = d["wb__data"]; mw = (tw > c0 + trim) & (tw < c1 - trim)
        r["law_eR_rms"] = float(rms(np.linalg.norm(D[mw, 28:31], axis=1))); r["law_eR_max"] = float(np.linalg.norm(D[mw, 28:31], axis=1).max())
        r["sat"] = int((D[mw, 51] > 0).sum()); r["u1_mean"] = float(D[mw, 17].mean())
        r["tau_arm_max"] = np.abs(D[mw, 13:17]).max(0).tolist()
        r["obs_flag_frac"] = float(np.nanmean(D[mw, 106])) if D.shape[1] > 106 else None
    else:
        tl = d["l1__recv"] - t0; L = d["l1__data"]; ml = (tl > c0 + trim) & (tl < c1 - trim)
        r["law_eR_rms"] = float(rms(np.linalg.norm(L[ml, 6:9], axis=1))); r["law_eR_max"] = float(np.linalg.norm(L[ml, 6:9], axis=1).max())
        r["u1_mean"] = float(L[ml, 12].mean()); r["uL1_thrust_mean"] = float(L[ml, 16].mean())
        mot = L[ml, 32:36]; r["sat"] = int(((mot >= 0.999) | (mot <= 0.001)).any(1).sum())
    # odometry health inside the window
    og = np.diff(to[(to > c0) & (to < c1)]); r["odom_max_gap_ms"] = float(1e3 * og.max())
    out["metrics"] = r
    return out, dict(t=tu, e=e, Pu=Pu, xb=xb, xc=xc, xcd=R["x_cd"], re=re, red=R["r_ed"], b1e=b1e, b1de=R["b1_de"], q=qu, qd=R["q_d"], yaw=yaw, yaw_ref=yaw_ref)


if __name__ == "__main__":
    import sys
    names = sys.argv[1:] or ["w1", "w2", "d1", "d2", "a1", "a2", "a3", "a4"]
    res = {}
    for nm in names:
        o, _ = analyse(nm); res[nm] = o; m = o["metrics"]
        f = lambda v: " ".join(f"{x:6.1f}" for x in v)
        print(f"\n{o['label']:14s} circle {o['window'][0]:.2f}-{o['window'][1]:.2f}")
        print(f"  CoM pos   mm  xyz {f(m['com_pos']['rms_xyz'])} | norm {m['com_pos']['rms_norm']:6.1f}  mean xyz {f(m['com_pos']['mean_xyz'])}")
        print(f"  base pos  mm  xyz {f(m['base_pos']['rms_xyz'])} | norm {m['base_pos']['rms_norm']:6.1f}  mean xyz {f(m['base_pos']['mean_xyz'])}")
        print(f"  base vel mm/s xyz {f(m['base_vel']['rms_xyz'])} | norm {m['base_vel']['rms_norm']:6.1f}")
        print(f"  attitude deg rpy {f(m['att']['rms'])}  mean {f(m['att']['mean'])} | rate deg/s {f(m['rate']['rms'])} | yaw(world) {m['yaw_world']['rms']:.2f} mean {m['yaw_world']['mean']:+.2f}")
        print(f"  joints deg  {f(m['joint']['rms'])}  mean {f(m['joint']['mean'])} | joint vel deg/s {f(m['joint_vel']['rms'])}")
        print(f"  EE pos    mm  xyz {f(m['ee_pos']['rms_xyz'])} | norm {m['ee_pos']['rms_norm']:6.1f} mean|e| {m['ee_mean_mm']:.1f} max {m['ee_pos']['max_norm']:.1f} | lag {m['ee_lag_s']:.2f}s resid {m['ee_resid_after_lag_mm']:.1f}")
        print(f"  EE heading deg rms {m['ee_head']['rms']:.2f} mean {m['ee_head']['mean']:+.2f} max {m['ee_head']['maxabs']:.2f} (3d {m['ee_head3d']['rms']:.2f}) | radius {m['ee_radius_meas_mean_m']:.3f} | tilt max {m['tilt_max_deg']:.2f}")
        print(f"  law |e_R| rms {m['law_eR_rms']:.4f} max {m['law_eR_max']:.4f} | sat {m['sat']} | u1 {m['u1_mean']:.2f} | odom max gap {m['odom_max_gap_ms']:.0f} ms | obs {m.get('obs_flag_frac')}")
    json.dump(res, open("../analysis/metrics.json", "w"), indent=1)

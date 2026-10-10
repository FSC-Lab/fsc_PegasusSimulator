"""Tracking of the whole-body controller through each flight's DIRECT span, per mission phase.

measured   fused odometry (what the law flies on) + joint_states -> CoM x_c, claw r_e, claw heading by the planner's FK
reference  the planner's WholeBodyReference stream (x_cd, r_ed, b1_d, b1_de, q_d)
grid       50 Hz, DIRECT entry -> the law's last tick
Writes ../analysis/tracking.json (per-phase metrics) and ../analysis/series_<run>.json (20 Hz series for the figures).
"""
import json
from pp_common import *
from scipy.spatial.transform import Rotation as Rot


def build(nm):
    d, t0 = load(nm); a, b = direct_span(d, t0)
    dt = 0.02; t = np.arange(a + 0.1, b - 0.05, dt)
    to, P, Q, V, W = odom(d, t0); tm, Pm, Qm, Vm = mocap_pose(d, t0, "mocap")
    tj, qj, _ = joints_model(d, t0); tr, ref = wbref(d, t0)
    Pu = interp(t, to, P); Qu = quat_cont(interp(t, to, Q)); Qu /= np.linalg.norm(Qu, axis=1)[:, None]
    Pmu = interp(t, tm, Pm); Qmu = quat_cont(interp(t, tm, Qm)); Qmu /= np.linalg.norm(Qmu, axis=1)[:, None]
    q = interp(t, tj, qj)
    R = {k: interp(t, tr, v) for k, v in ref.items()}
    xc, re, b1e, _ = fk_world(Pu, Qu, q)
    xc_m, re_m, b1e_m, _ = fk_world(Pmu, Qmu, q)
    yaw = yaw_of(Qu); yaw_ref = np.arctan2(R["b1_d"][:, 1], R["b1_d"][:, 0]) + 0.5 * np.pi
    az = np.arctan2(b1e[:, 1], b1e[:, 0]); az_r = np.arctan2(R["b1_de"][:, 1], R["b1_de"][:, 0])
    tw, D = wbdebug(d, t0)
    Dn = lambda sl: interp(t, tw, D[:, sl])
    S = dict(t=t, a=a, b=b, e_com=xc - R["x_cd"], e_ee=re - R["r_ed"], e_ee_mocap=re_m - R["r_ed"],
             e_task=Dn(WB["e_y"])[:, :3], e_yaw=np.degrees(wrap(yaw - yaw_ref)), e_head=np.degrees(wrap(az - az_r)),
             e_q=np.degrees(q - R["q_d"]), q=np.degrees(q), q_d=np.degrees(R["q_d"]), tilt=tilt_of(Qu),
             tilt_mocap=tilt_of(Qmu), eR=np.linalg.norm(Dn(WB["e_R"]), axis=1), motors=Dn(WB["motors"]),
             tau=Dn(WB["tau"]), u1=Dn(WB["u1"]), dhat=Dn(WB["d_hat"]), Fraw=Dn(WB["F_raw"])[:, :3],
             x_c=xc, x_cd=R["x_cd"], r_e=re, r_ed=R["r_ed"], r_e_mocap=re_m, base=Pu, base_mocap=Pmu,
             yaw=np.degrees(yaw), yaw_ref=np.degrees(yaw_ref))
    S["n_sat"] = D[(tw >= a) & (tw <= b), WB["n_sat"]]; S["n_clamped"] = D[(tw >= a) & (tw <= b), WB["n_clamped"]]
    S["fresh"] = D[(tw >= a) & (tw <= b), WB["stream_fresh"]]
    return d, t0, S


def metrics(S, m):
    n = lambda x: np.linalg.norm(x, axis=1)
    r = {}
    for k, key in (("com", "e_com"), ("ee", "e_ee"), ("ee_mocap", "e_ee_mocap"), ("task", "e_task")):
        e = S[key][m]
        r[k] = dict(rms=float(1e3 * rms(n(e))), max=float(1e3 * n(e).max()), mean_xyz=(1e3 * e.mean(0)).round(1).tolist(),
                    rms_xyz=(1e3 * rms(e)).round(1).tolist())
    for k in ("e_yaw", "e_head"):
        r[k] = dict(rms=float(rms(S[k][m])), max=float(np.abs(S[k][m]).max()), mean=float(S[k][m].mean()))
    r["e_q"] = dict(rms=rms(S["e_q"][m]).round(2).tolist(), max=np.abs(S["e_q"][m]).max(0).round(2).tolist(),
                    mean=S["e_q"][m].mean(0).round(2).tolist())
    r["tilt_max"] = float(S["tilt"][m].max()); r["eR_rms"] = float(rms(S["eR"][m])); r["eR_max"] = float(S["eR"][m].max())
    r["motor_min"] = float(S["motors"][m].min()); r["motor_max"] = float(S["motors"][m].max())
    r["tau_max"] = np.abs(S["tau"][m]).max(0).round(3).tolist()
    r["dhat_z"] = float(S["dhat"][m][:, 2].mean())
    return r


if __name__ == "__main__":
    out = {}
    for nm in RUNS:
        d, t0, S = build(nm); t = S["t"]
        ph = [(x, y, lab) for x, y, lab in pp_phases(d, t0) if y > S["a"] and x < S["b"]]
        rows = []
        for x, y, lab in ph:
            m = (t >= x) & (t < y)
            if m.sum() < 10:
                continue
            r = metrics(S, m); r.update(label=lab, t0=x, t1=y); rows.append(r)
        allm = np.ones(len(t), bool)
        tot = metrics(S, allm)
        tot.update(direct=[S["a"], S["b"]], n_sat_ticks=int((S["n_sat"] > 0).sum()), n_clamp_ticks=int((S["n_clamped"] > 0).sum()),
                   ticks=int(len(S["n_sat"])), stream_stale_ticks=int((S["fresh"] < 0.5).sum()))
        out[nm] = dict(tag=RUNS[nm][0], time=RUNS[nm][1], total=tot, phases=rows)
        print(f"===== {nm}  DIRECT {S['a']:.1f}-{S['b']:.1f} s  sat ticks {tot['n_sat_ticks']} clamp ticks {tot['n_clamp_ticks']} of {tot['ticks']}, stale-stream ticks {tot['stream_stale_ticks']}")
        print(f"   whole DIRECT: CoM rms {tot['com']['rms']:.1f} max {tot['com']['max']:.1f} | EE rms {tot['ee']['rms']:.1f} max {tot['ee']['max']:.1f} | task rms {tot['task']['rms']:.1f} | yaw rms {tot['e_yaw']['rms']:.2f} max {tot['e_yaw']['max']:.2f} | head rms {tot['e_head']['rms']:.2f} | tilt max {tot['tilt_max']:.1f} | eR rms {tot['eR_rms']:.3f} | motors {tot['motor_min']:.2f}-{tot['motor_max']:.2f} | tau max {tot['tau_max']}")
        for r in rows:
            print(f"   {r['t0']:7.2f}-{r['t1']:7.2f} {r['label'][:34]:34s} CoM {r['com']['rms']:5.1f}/{r['com']['max']:5.1f}  EE {r['ee']['rms']:5.1f}/{r['ee']['max']:5.1f}  task {r['task']['rms']:4.1f}/{r['task']['max']:5.1f}  yaw {r['e_yaw']['rms']:4.2f}/{r['e_yaw']['max']:4.2f} head {r['e_head']['rms']:4.2f}/{r['e_head']['max']:5.2f}  q-rms {r['e_q']['rms']}  tilt {r['tilt_max']:4.1f}  dz {r['dhat_z']:6.2f}")
        k = slice(None, None, 3)
        ser = {key: np.round(S[key][k], 4).tolist() for key in ("t", "e_com", "e_ee", "e_task", "e_yaw", "e_head", "e_q", "q", "q_d", "tilt", "eR",
                                                                 "motors", "tau", "u1", "x_c", "x_cd", "r_e", "r_ed", "r_e_mocap", "base", "base_mocap", "yaw", "yaw_ref")}
        ser["dhat"] = np.round(S["dhat"][k], 3).tolist(); ser["Fraw"] = np.round(S["Fraw"][k], 3).tolist()
        ser["phases"] = [[x, y, lab] for x, y, lab in ph]
        json.dump(ser, open(os.path.join(OUT, f"series_{nm}.json"), "w"))
    json.dump(out, open(os.path.join(OUT, "tracking.json"), "w"), indent=1)

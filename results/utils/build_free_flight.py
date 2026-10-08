#!/usr/bin/env python3
"""Free-flight trajectory tracking, simulation: raw Isaac runs -> MATLAB files + the paper table.

    /usr/bin/python3 results/utils/build_free_flight.py

Reads every completed run on disk, free_flight_tracking/<shape>/v<speed>/<method>_<shape>_v<speed>_run<k>.npz
(raw npz written by application/robotic_arm/utils/am_ee_compare_driver.py, flown by run_tracking_campaign.py),
and for each one writes, next to the raw npz,
    <name>.mat    struct `run`: meta, raw streams (column-named), the 100 Hz tracking signals and the RMSEs
and over all runs
    tables/free_flight_tracking_sim.csv / .json   per run and per (shape, speed, method) mean

Two interpreters are needed on this machine: the raw npz carry a numpy-2 pickled object array
(the controller debug stream), which only the user-site numpy 2 can read, while scipy (rotations,
savemat) is the apt build for numpy 1.21. Stage 1 runs under numpy 2 and rewrites each run into a
plain-array npz in a temp directory; stage 2 re-executes this file with PYTHONNOUSERSITE=1.

METRICS (same definitions as the hardware report, docs/.../wb_vs_decoupled_flight_20260928/tools/metrics.py)
  window     the planner's trajectory run (driver marks run_start..run_end, ramps included),
             trimmed 0.1 s at each end, 100 Hz uniform grid
  reference  the planner's WholeBodyReference stream, identical for every method:
             platform position x_b = x_cd - R0 r_0c(q_d), R0 = the plan's compatible attitude
             (thrust along x_cd_ddot + g e3, heading b1_d), EE position r_ed, EE heading b1_de, joints q_d
  measured   odometry (position, attitude) + joint_states; EE position and heading by the planner's
             own forward kinematics (transition_planner.arm_fk_model)
  platform position   RMSE of |p - x_b|                                   [mm]
  platform attitude   rotation vector of R_ref^T R in body axes -> roll / pitch / heading, RMSE each [deg]
  EE position         RMSE of |r_e - r_ed|                                [mm]
  EE heading          RMSE of the azimuth of b1_e minus that of b1_de     [deg]
  joints              RMSE of q_i - q_d,i, i = 1..4                       [deg]
"""
import glob
import json
import math
import os
import re
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.abspath(os.path.join(HERE, ".."))                  # results/
ROOT = os.path.join(RESULTS, "simulation_results")
REPO = os.path.abspath(os.path.join(RESULTS, ".."))
FF = os.path.join(ROOT, "free_flight_tracking")
TABLES = os.path.join(ROOT, "tables")

METHODS = {
    "whole_body_l1": "Whole-body L1 impedance (proposed)",
    "geometric_l1": "Geometric control with L1 adaptive augmentation (Cai et al., CEP 2025)",
    "modular_adaptive": "Modular adaptive control (Yadav et al., TMECH 2025)",
}

NAME = re.compile(r"^(whole_body_l1|geometric_l1|modular_adaptive)_(circle|figure8)_v(\d)p(\d\d)_run(\d+)$")


def discover():
    """Every completed run of the campaign on disk: (shape, speed, method, run, npz path, campaign record)."""
    recs = {}
    log = os.path.join(FF, "campaign.jsonl")
    if os.path.isfile(log):
        for line in open(log):
            r = json.loads(line)
            if r["status"] == "completed":
                recs[os.path.normpath(os.path.join(ROOT, r["file"]))] = r
    out = []
    for path in sorted(glob.glob(os.path.join(FF, "*", "v*", "*.npz"))):
        m = NAME.match(os.path.basename(path)[:-4])
        if m:
            method, shape = m.group(1), m.group(2)
            v = int(m.group(3)) + int(m.group(4)) / 100.0
            out.append((shape, v, method, int(m.group(5)), path, recs.get(os.path.normpath(path), {})))
    return out


TRIM = 0.1
DT = 0.01
G = 9.80665


def vtag(v):
    return "v" + f"{v:.2f}".replace(".", "p")


def run_name(shape, v, method, k):
    return f"{method}_{shape}_{vtag(v)}_run{k}"


def run_path(shape, v, method, k, ext):
    return os.path.join(FF, shape, vtag(v), run_name(shape, v, method, k) + ext)


# ----------------------------------------------------------------------------------------------------------
# stage 1 (numpy 2): raw npz -> plain arrays
# ----------------------------------------------------------------------------------------------------------
def stage1(tmp):
    for shape, v, method, k, src, rec in discover():
        d = np.load(src, allow_pickle=True)
        out = {}
        for key in d.files:
            a = d[key]
            if key == "dbg":
                rows = [np.asarray(r, float) for r in a]
                n = max(len(r) for r in rows) if rows else 1
                m = np.full((len(rows), n), np.nan)
                for i, r in enumerate(rows):
                    m[i, :len(r)] = r
                a = m
            elif a.dtype == object:
                a = np.array([str(x) for x in np.atleast_1d(a)], dtype="U")
            out[key] = a
        np.savez(os.path.join(tmp, run_name(shape, v, method, k) + ".npz"), **out)


# ----------------------------------------------------------------------------------------------------------
# stage 2 (numpy 1 + scipy): metrics, MATLAB files, table
# ----------------------------------------------------------------------------------------------------------
def stage2(tmp):
    import scipy.io
    from scipy.spatial.transform import Rotation as Rot
    sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
    from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP

    RZ90 = TP._Rz(0.5 * math.pi)        # model (y-forward) -> actual (x-forward) body frame
    RZM90 = TP._Rz(-0.5 * math.pi)

    def interp(tq, t, X):
        X = np.asarray(X)
        if X.ndim == 1:
            return np.interp(tq, t, X)
        return np.column_stack([np.interp(tq, t, X[:, i]) for i in range(X.shape[1])])

    def wrap(a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    def quat_cont(Q):
        Q = Q.copy()
        for i in range(1, len(Q)):
            if Q[i] @ Q[i - 1] < 0:
                Q[i] = -Q[i]
        return Q

    def build_r0(b3, b1d):
        b3 = b3 / np.linalg.norm(b3)
        b2 = np.cross(b3, b1d); b2 /= np.linalg.norm(b2)
        return np.column_stack([np.cross(b2, b3), b2, b3])

    def rms(x, axis=0):
        return np.sqrt(np.mean(np.asarray(x) ** 2, axis=axis))

    rows = []
    for shape, v, method, k, src, rec in discover():
        name = run_name(shape, v, method, k)
        d = np.load(os.path.join(tmp, name + ".npz"))
        marks = {}
        for m in d["marks"]:
            key, _, val = str(m).partition("=")
            marks[key] = float(val)
        t0, t1 = marks["run_start"], marks["run_end"]
        log, js, wb = d["log"], d["js"], d["wbref"]
        js = js[np.all(np.isfinite(js), axis=1)]
        params = TP.make_params_t650(base_com=d["base_com"])

        tu = np.arange(t0 + TRIM, t1 - TRIM, DT)
        n = len(tu)
        P = interp(tu, log[:, 0], log[:, 1:4])
        Qxyzw = quat_cont(log[:, 7:11])
        Qu = interp(tu, log[:, 0], Qxyzw); Qu /= np.linalg.norm(Qu, axis=1)[:, None]
        Ra = Rot.from_quat(Qu).as_matrix()                               # actual body -> world
        q = interp(tu, js[:, 0], js[:, 1:5])
        W = interp(tu, wb[:, 0], wb[:, 1:])
        x_cd, x_cd_dd, b1d = W[:, 0:3], W[:, 6:9], W[:, 9:12]
        r_ed, b1de, q_d = W[:, 12:15], W[:, 18:21], W[:, 21:25]

        xb = np.zeros((n, 3)); Rref = np.zeros((n, 3, 3)); xc = np.zeros((n, 3))
        re = np.zeros((n, 3)); b1e = np.zeros((n, 3))
        for i in range(n):
            R0m_ref = build_r0(x_cd_dd[i] + np.array([0.0, 0.0, G]), b1d[i])
            r0c_d, _, _ = TP.arm_fk_model(q_d[i], params)
            xb[i] = x_cd[i] - R0m_ref @ r0c_d
            Rref[i] = R0m_ref @ RZ90
            Rm = Ra[i] @ RZM90
            r0c, r0e, Re = TP.arm_fk_model(q[i], params)
            xc[i] = P[i] + Rm @ r0c; re[i] = P[i] + Rm @ r0e; b1e[i] = Rm @ Re[:, 0]

        E = np.einsum("nji,njk->nik", Rref, Ra)                           # R_ref^T R
        att_err = np.degrees(Rot.from_matrix(E).as_rotvec())             # body axes: roll, pitch, heading
        yaw = np.degrees(np.unwrap(np.arctan2(Ra[:, 1, 0], Ra[:, 0, 0])))
        yaw_ref = np.degrees(np.unwrap(np.arctan2(Rref[:, 1, 0], Rref[:, 0, 0])))
        az = np.degrees(np.unwrap(np.arctan2(b1e[:, 1], b1e[:, 0])))
        az_ref = np.degrees(np.unwrap(np.arctan2(b1de[:, 1], b1de[:, 0])))
        ee_head_err = np.degrees(wrap(np.radians(az - az_ref)))
        e_pos = P - xb; e_ee = re - r_ed; e_com = xc - x_cd; e_q = np.degrees(q - q_d)
        tilt = np.degrees(np.arccos(np.clip(Ra[:, 2, 2], -1, 1)))

        r = dict(
            platform_pos_mm=float(1e3 * rms(np.linalg.norm(e_pos, axis=1))),
            platform_pos_xyz_mm=(1e3 * rms(e_pos)).tolist(),
            platform_roll_deg=float(rms(att_err[:, 0])), platform_pitch_deg=float(rms(att_err[:, 1])),
            platform_heading_deg=float(rms(att_err[:, 2])),
            ee_pos_mm=float(1e3 * rms(np.linalg.norm(e_ee, axis=1))),
            ee_pos_xyz_mm=(1e3 * rms(e_ee)).tolist(),
            ee_heading_deg=float(rms(ee_head_err)),
            joints_deg=rms(e_q).tolist(),
            com_pos_mm=float(1e3 * rms(np.linalg.norm(e_com, axis=1))),
            ee_pos_max_mm=float(1e3 * np.linalg.norm(e_ee, axis=1).max()),
            tilt_max_deg=float(tilt.max()),
        )
        # cross-check: the planner's own current_ee (FK on its side) against the same reference
        ee = d["ee"]; win = (ee[:, 0] >= t0 + TRIM) & (ee[:, 0] <= t1 - TRIM)
        r["check_planner_ee_pos_mm"] = float(1e3 * rms(np.linalg.norm(ee[win, 1:4] - ee[win, 4:7], axis=1)))

        info = d["info"]
        meta = dict(
            name=name, method=method, method_long=METHODS[method], shape=shape, mean_speed_nominal_mps=v,
            run_index=k, flown=rec.get("time", ""), campaign="results/utils/run_tracking_campaign.py",
            controller_yaml=rec.get("yaml", ""), driver_args=" ".join(rec.get("args", [])),
            rtf_min=rec.get("rtf_min") if rec.get("rtf_min") is not None else float("nan"),
            plant="Isaac Sim, sim-to-real mirror plant (config A), headless at real time (RTF 1), raw mocap feedback",
            lap_time_s=float(d["lap_time"]), laps=int(d["laps"]), time_scale=float(d["time_scale"]),
            radius_m=float(d["radius"]) if shape == "circle" else float("nan"),
            fig8_a_m=float(d["fig8_a"]) if shape == "figure8" else float("nan"),
            fig8_b_m=float(d["fig8_b"]) if shape == "figure8" else float("nan"),
            q2_center_deg=float(d["q2_center_deg"]), q2_amp_deg=float(d["q2_amp_deg"]),
            q2_period_s=float(d["q2_period"]), fold_deg=float(d["fold_deg"]),
            base_com_model_m=np.asarray(d["base_com"], float),
            planner_info=np.asarray(info, float),
            run_start_s=t0, run_end_s=t1, direct_enter_s=marks.get("direct_enter", float("nan")),
            aborted=bool(d["aborted"]), clock="driver clock [s] (raw streams); tracking.t is s since run_start",
        )
        raw = dict(
            odom=dict(t=log[:, 0], pos_m=log[:, 1:4], vel_mps=log[:, 4:7], quat_xyzw=log[:, 7:11],
                      direct_mode=log[:, 11]),
            joints=dict(t=d["js"][:, 0], q_rad=d["js"][:, 1:5]),
            reference=dict(t=wb[:, 0], x_cd=wb[:, 1:4], x_cd_dot=wb[:, 4:7], x_cd_ddot=wb[:, 7:10],
                           b1_d_model=wb[:, 10:13], r_ed=wb[:, 13:16], r_ed_dot=wb[:, 16:19], b1_de=wb[:, 19:22],
                           q_d_rad=wb[:, 22:26], qdot_d_rad_s=wb[:, 26:30], x_b=wb[:, 30:33], v_b=wb[:, 33:36],
                           a_b=wb[:, 36:39], yaw_ref_rad=wb[:, 39]),
            planner_ee=dict(t=ee[:, 0], current_m=ee[:, 1:4], reference_m=ee[:, 4:7]),
            arm_reference=dict(t=d["armref"][:, 0], q_rad=d["armref"][:, 1:5], qdot_rad_s=d["armref"][:, 5:9]),
            controller_debug=dict(t=d["dbg"][:, 0], data=d["dbg"][:, 1:]),
            events=np.array([str(x) for x in d["events"]], dtype=object),
        )
        tracking = dict(
            t=tu - t0,
            platform_pos_m=P, platform_pos_ref_m=xb, platform_pos_err_m=e_pos,
            platform_quat_xyzw=Qu, platform_att_err_deg=att_err,
            platform_heading_deg=yaw, platform_heading_ref_deg=yaw_ref, platform_tilt_deg=tilt,
            com_pos_m=xc, com_pos_ref_m=x_cd, com_pos_err_m=e_com,
            ee_pos_m=re, ee_pos_ref_m=r_ed, ee_pos_err_m=e_ee,
            ee_heading_deg=az, ee_heading_ref_deg=az_ref, ee_heading_err_deg=ee_head_err,
            q_deg=np.degrees(q), q_ref_deg=np.degrees(q_d), q_err_deg=e_q,
            columns="platform_att_err_deg = [roll pitch heading] (body axes); xyz columns are world ENU",
        )
        mat = dict(meta=meta, rmse=r, tracking=tracking, raw=raw)
        scipy.io.savemat(src[:-4] + ".mat", {"run": mat}, do_compression=True,
                         long_field_names=True, oned_as="column")
        rows.append(dict(name=name, shape=shape, mean_speed_mps=v, method=method, run=k, flown=rec.get("time", ""), **r))
        print(f"{name:40s} plat {r['platform_pos_mm']:6.1f} mm  rpy {r['platform_roll_deg']:.2f}/"
              f"{r['platform_pitch_deg']:.2f}/{r['platform_heading_deg']:.2f} deg  EE {r['ee_pos_mm']:6.1f} mm "
              f"(planner {r['check_planner_ee_pos_mm']:.1f})  head {r['ee_heading_deg']:.2f}  q "
              + " ".join(f"{x:.2f}" for x in r["joints_deg"]))

    # per (shape, speed, method) mean over runs
    keys = ["platform_pos_mm", "platform_heading_deg", "platform_roll_deg", "platform_pitch_deg",
            "ee_pos_mm", "ee_heading_deg"]
    groups = {}
    for r in rows:
        groups.setdefault((r["shape"], r["mean_speed_mps"], r["method"]), []).append(r)
    fails = {}
    log = os.path.join(FF, "campaign.jsonl")
    if os.path.isfile(log):
        for line in open(log):
            c = json.loads(line)
            if c["status"] == "in_run":
                key = (c["shape"], float(f"{c['speed']:.2f}"), c["method"])
                fails[key] = fails.get(key, 0) + 1
    summary = []
    for (shape, v, method), rr in sorted(groups.items()):
        s = dict(shape=shape, mean_speed_mps=v, method=method, runs=len(rr),
                 failed_after_start=fails.get((shape, float(f"{v:.2f}"), method), 0))
        for kk in keys:
            s[kk] = float(np.mean([r[kk] for r in rr]))
        for j, ax in enumerate("xyz"):
            s[f"platform_{ax}_mm"] = float(np.mean([r["platform_pos_xyz_mm"][j] for r in rr]))
            s[f"ee_{ax}_mm"] = float(np.mean([r["ee_pos_xyz_mm"][j] for r in rr]))
        for j in range(4):
            s[f"q{j + 1}_deg"] = float(np.mean([r["joints_deg"][j] for r in rr]))
        summary.append(s)
    os.makedirs(TABLES, exist_ok=True)
    with open(os.path.join(TABLES, "free_flight_tracking_sim.json"), "w") as f:
        json.dump(dict(runs=rows, mean_over_runs=summary), f, indent=1)
    cols = (["shape", "mean_speed_mps", "method", "runs", "failed_after_start"]
            + [f"platform_{a}_mm" for a in "xyz"] + ["platform_heading_deg", "platform_roll_deg", "platform_pitch_deg"]
            + [f"ee_{a}_mm" for a in "xyz"] + ["ee_heading_deg"] + [f"q{j}_deg" for j in range(1, 5)]
            + ["platform_pos_mm", "ee_pos_mm"])
    with open(os.path.join(TABLES, "free_flight_tracking_sim.csv"), "w") as f:
        f.write(",".join(cols) + "\n")
        for s in summary:
            f.write(",".join(str(s[c]) if not isinstance(s[c], float) else f"{s[c]:.4f}" for c in cols) + "\n")
    for s in summary:
        print(f"MEAN {s['shape']} {s['mean_speed_mps']:.2f} {s['method']} (n={s['runs']}): "
              + "  ".join(f"{c} {s[c]:.2f}" for c in cols[5:]))


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--stage2":
        stage2(sys.argv[2])
    else:
        with tempfile.TemporaryDirectory() as tmp:
            stage1(tmp)
            env = dict(os.environ, PYTHONNOUSERSITE="1")
            subprocess.run(["/usr/bin/python3", os.path.abspath(__file__), "--stage2", tmp], env=env, check=True)

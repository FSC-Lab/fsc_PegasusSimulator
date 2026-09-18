#!/usr/bin/env python3
"""
wb_step_sim.py — a planned base STEP of the whole-body law, offline (2026-09-18).

wb_entry_sim.py closes the DIRECT-entry loop on a static hold. This is the
same plant, law and observer (imported, not copied) flown through the
mission's step legs instead: the observer is first allowed to settle on the
hold, then a COMPATIBLE transition planned by the same flat B-spline planner
the C++ node runs (utils_planner.flat_bspline_planner) moves the base 0.5 m
ALONG the arm or LATERAL to it, and the vehicle holds the goal for 16 s.

WHY. The flown 4-D campaign (l1_4d_20260916, traj_errors.txt) settles the
along-arm (world-x at yaw 0) steps at 15-33 mm against 3-7 mm lateral, with
a ~0.15 Hz, lightly damped oscillation of +-50..100 mm that outlives the
16 s hold. This tool exists to (a) reproduce that ordering offline and (b)
sweep every gain group of the law, the observer and the feedforward filter
in seconds per case, so that the flights confirm a candidate rather than
search for one.

Same caveat as wb_entry_sim: the plant is the model with injections, not
PhysX (no arm friction, no current noise, no PX4). Trust the ORDERING.

Usage:
    /usr/bin/python3 wb_step_sim.py --validate         # shipped 4-D config, both step directions
    /usr/bin/python3 wb_step_sim.py --sweep            # one gain group at a time
    /usr/bin/python3 wb_step_sim.py --case g.k_v=28,l1.omega_x=0.1 [--dir along|lateral] [--trace]
"""
import argparse
import sys
import os

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wb_entry_sim as ES  # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C  # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import (  # noqa: E402
    transition_planner as TP,
    flat_bspline_planner as FB,
)

# the planner section of the 4-D sim yaml
PLAN_OPTS = dict(v_max=0.30, a_max=0.15, w_max=0.30, dw_max=0.60,
                 T_min=3.0, T_max=40.0, tau_joint_max=3.0, Ncheck=0)


def shipped_case(**over):
    """The 4-D sim yaml of 2026-09-17 (law + observer + feedforward) on the
    config-A plant, minus what the offline plant cannot carry (friction,
    current noise)."""
    kw = dict(
        posture=0.0, ee_ref="relative", int_ff=True, int_ff_source="four_d",
        int_ff_blocks="all",
        mass=1.10, inertia=1.10, com=(0.010, 0.010, 0.005),
        kf=1.0, kf_alloc=4.7544506e-05 / ES.KF_TRUE,     # allocator-side +17.6 %
        arm_mass=1.05,
        gains=dict(k_x=32.0, k_v=20.0, k_R=2.0, k_w=1.5,
                   mrd=(0.116522, 0.136107, 0.125102),
                   ky=(20.0, 20.0, 20.0, 0.3), dy=(12.0, 12.0, 12.0, 0.3),
                   my=(1.0, 1.0, 1.0, 0.05), dls_lambda=0.3, tau_max=3.0),
        l1=dict(a_t=2.0, a_r=2.0, a_q=2.0, omega_c_t=2.0, omega_c_r=0.5,
                omega_c_q=0.5, omega_i=2.0, omega_x=0.25, four_d=True,
                omega_q=0.2, lc_var_f=100.0, lc_var_m=0.25, lc_var_q=0.0025,
                max_force_n=20.0, max_torque_nm=2.0, max_joint_nm=1.5,
                max_wrench_force_n=15.0, max_wrench_torque_nm=3.0),
        t_end=0.0, z0=1.0)
    for k, v in over.items():
        if k in ("gains", "l1"):
            kw[k] = dict(kw[k], **v)
        else:
            kw[k] = v
    return ES.Case(**kw)


def simulate_step(case, direction="along", step=0.5, t_settle=15.0, t_hold=16.0,
                  trace=False):
    model = TP.make_params_t650()
    plant = ES.plant_params(model, case)
    n = model["n"]
    cfg = ES.make_gains(**case.gains)
    law = ES.Law(model, cfg, case)
    dt = case.dt
    kf_plant = ES.KF_TRUE * case.kf
    kf_alloc = ES.KF_TRUE * case.kf_alloc

    X = np.zeros(12 + 3 * n + 6)
    X[0:3] = [0.0, 0.0, case.z0]
    X[3:12] = np.eye(3).flatten(order="F")
    X[12:12 + n] = ES.HOME

    # hold reference (model heading 0: b1_d = +x, the arm along model +y)
    rest0 = {"x_b": X[0:3].copy(), "phi": 0.0, "q": ES.HOME.copy()}
    d = np.array([0.0, step, 0.0]) if direction == "along" else np.array([step, 0.0, 0.0])
    rest1 = {"x_b": X[0:3] + d, "phi": 0.0, "q": ES.HOME.copy()}
    hold0 = TP.rest_ref(model, rest0["x_b"], 0.0, rest0["q"])
    hold1 = TP.rest_ref(model, rest1["x_b"], 0.0, rest1["q"])
    plan = FB.plan_flat_transition(model, rest0, rest1, PLAN_OPTS)
    T = plan["T"]

    def reference(t):
        if t < t_settle:
            return hold0
        if t < t_settle + T:
            return plan["ref"](t - t_settle)
        return hold1

    dynp = C.dynamics(X, plant)
    gp = dynp["g"]
    thrust_true = gp[2]; tau_true = gp[3:6]
    omega_rot = ES.allocate(thrust_true, tau_true, kf_plant)
    n_del = int(round(case.delay_ms * 1e-3 / dt))
    cmd_fifo = [ES.allocate(thrust_true, tau_true, kf_alloc) for _ in range(max(n_del, 0))]
    t_end = t_settle + T + t_hold
    steps = int(t_end / dt)
    hist = {k: [] for k in ("t", "e", "ey", "tilt", "nsat", "tau", "eR", "q")}
    verdict = "completed"; t_fail = None

    for k in range(steps):
        t = k * dt
        ref = reference(t)
        dyn = C.dynamics(X, model)
        out = law(X, dyn, ref, dt)
        u1 = float(out["u1"])
        tau_body = np.asarray(out["tau_body"], float)
        tau_joint = np.clip(np.asarray(out["tau_joint"], float), -cfg.tau_max, cfg.tau_max)
        w_cmd = ES.allocate(u1, tau_body, kf_alloc)
        if n_del > 0:
            cmd_fifo.append(w_cmd); w_cmd = cmd_fifo.pop(0)
        omega_rot = w_cmd + (omega_rot - w_cmd) * np.exp(-ES.LAMBDA_ROTOR * dt)
        thrust_a, tau_a = ES.wrench_from_rotors(omega_rot, kf_plant)
        q = X[12:12 + n]; qd = X[18 + n:18 + 2 * n]
        tau_stop = np.zeros(n)
        over = q - ES.Q_MAX; under = ES.Q_MIN - q
        tau_stop -= np.where(over > 0, ES.K_STOP * over + ES.C_STOP * np.maximum(qd, 0), 0.0)
        tau_stop += np.where(under > 0, ES.K_STOP * under + ES.C_STOP * np.maximum(-qd, 0), 0.0)
        dynp = C.dynamics(X, plant)
        Q = np.concatenate([[0.0, 0.0, thrust_a], tau_a, tau_joint + tau_stop])
        xi = np.concatenate([X[12 + n:15 + n], X[15 + n:18 + n], X[18 + n:18 + 2 * n]])
        xi_dot = np.linalg.solve(dynp["M"], Q - dynp["C"] @ xi - dynp["g"])
        xi = xi + xi_dot * dt
        v0, w0, qd = xi[0:3], xi[3:6], xi[6:6 + n]
        R = X[3:12].reshape(3, 3, order="F")
        X[0:3] = X[0:3] + R @ v0 * dt
        nw = np.linalg.norm(w0)
        R = R @ C.joint_rotation(w0 / (nw + 1e-12), nw * dt)
        uu, _, vt = np.linalg.svd(R); R = uu @ vt
        X[3:12] = R.flatten(order="F")
        X[12:12 + n] = X[12:12 + n] + qd * dt
        X[12 + n:18 + n] = np.concatenate([v0, w0])
        X[18 + n:18 + 2 * n] = qd

        tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
        hist["t"].append(t); hist["e"].append(np.asarray(out["e_x"], float).copy())
        hist["ey"].append(float(np.linalg.norm(out["e_y"][:3])))
        hist["tilt"].append(tilt); hist["nsat"].append(out["n_sat"])
        hist["tau"].append(tau_joint.copy()); hist["eR"].append(float(np.linalg.norm(out["e_R"])))
        hist["q"].append(np.degrees(X[12:12 + n]).copy())
        if trace and k % 125 == 0:
            e = out["e_x"] * 1e3
            print(f"  t={t:5.1f} e=({e[0]:+6.0f},{e[1]:+6.0f},{e[2]:+5.0f}) mm |e_y|={hist['ey'][-1]*1e3:5.1f} "
                  f"tilt={tilt:4.1f} |eR|={hist['eR'][-1]:.3f} tau={np.round(tau_joint,2)} nsat={out['n_sat']}")
        ex = float(np.linalg.norm(out["e_x"]))
        if not np.isfinite(ex) or tilt > 35.0 or ex > 1.5 or abs(X[2] - case.z0) > 0.8:
            verdict = "ABORT"; t_fail = t; break

    for k in hist:
        hist[k] = np.asarray(hist[k])
    return score(hist, verdict, t_fail, t_settle, T, direction, dt)


def score(hist, verdict, t_fail, t_settle, T, direction, dt):
    ax = 1 if direction == "along" else 0
    t = hist["t"]; e = hist["e"]
    res = dict(verdict=verdict, t_fail=t_fail, T=T, hist=hist, direction=direction)
    if verdict != "completed":
        return res
    trans = (t >= t_settle) & (t < t_settle + T)
    hold = t >= t_settle + T
    th = t[hold] - (t_settle + T)
    eh = e[hold]
    res["peak_trans"] = float(np.abs(e[trans][:, ax]).max())
    res["peak_hold"] = float(np.abs(eh[:, ax]).max())
    res["settled_rms"] = float(np.sqrt(np.mean(eh[th > th[-1] - 2.0][:, ax] ** 2)))
    # last time the along-step axis error exceeds 20 mm, from the end of the move
    exc = np.where(np.abs(eh[:, ax]) > 0.020)[0]
    res["t_settle20"] = float(th[exc[-1]]) if exc.size else 0.0
    exc = np.where(np.abs(eh[:, ax]) > 0.050)[0]
    res["t_settle50"] = float(th[exc[-1]]) if exc.size else 0.0
    # envelope decay: peak |e| in [0,4) s vs [8,12) s of the hold
    a = np.abs(eh[(th >= 0) & (th < 4)][:, ax]).max()
    b = np.abs(eh[(th >= 8) & (th < 12)][:, ax]).max()
    res["decay_8_12_over_0_4"] = float(b / a) if a > 0 else 0.0
    # lateral-axis leakage (the other horizontal axis) and z during the whole step
    oth = 0 if ax == 1 else 1
    res["peak_other"] = float(np.abs(e[trans | hold][:, oth]).max())
    res["peak_z"] = float(np.abs(e[trans | hold][:, 2]).max())
    res["ey_peak"] = float(hist["ey"][trans | hold].max())
    res["ey_settled"] = float(hist["ey"][hold][th > th[-1] - 2.0].mean())
    res["tilt_max"] = float(hist["tilt"][trans | hold].max())
    res["eR_max"] = float(hist["eR"][trans | hold].max())
    res["tau_max"] = float(np.abs(hist["tau"][trans | hold]).max())
    res["clamp_pct"] = float(100.0 * np.mean(hist["nsat"][trans | hold] > 0))
    return res


def fmt(r):
    if r["verdict"] != "completed":
        return f"ABORT t={r['t_fail']:.1f}s"
    return (f"T={r['T']:4.1f}s peak {r['peak_trans']*1e3:5.0f} | hold peak {r['peak_hold']*1e3:5.0f} "
            f"t20 {r['t_settle20']:5.1f}s t50 {r['t_settle50']:5.1f}s decay {r['decay_8_12_over_0_4']:4.2f} "
            f"settled {r['settled_rms']*1e3:5.1f} mm | oth {r['peak_other']*1e3:4.0f} z {r['peak_z']*1e3:3.0f} "
            f"| ey {r['ey_peak']*1e3:4.1f}/{r['ey_settled']*1e3:4.1f} tilt {r['tilt_max']:4.1f} "
            f"eR {r['eR_max']:.3f} tau {r['tau_max']:4.2f} clamp {r['clamp_pct']:4.1f}%")


def run(case, tag, dirs=("along", "lateral"), trace=False):
    out = {}
    for d in dirs:
        r = simulate_step(case, d, trace=trace)
        print(f"{tag:34s} {d:7s} {fmt(r)}", flush=True)
        out[d] = r
    return out


def parse_case(spec):
    kw = {}
    for kv in spec.split(","):
        if not kv.strip():
            continue
        k, v = kv.split("=")
        k = k.strip(); v = v.strip()
        if k in ("ee_ref", "observer", "int_ff_source", "int_ff_blocks"):
            kw[k] = v
        elif k == "com":
            kw[k] = tuple(float(x) for x in v.split("/"))
        elif k.startswith("g."):
            name = k[2:]
            if name in ("ky", "dy", "my", "mrd"):
                kw.setdefault("gains", {})[name] = tuple(float(x) for x in v.split("/"))
            else:
                kw.setdefault("gains", {})[name] = float(v)
        elif k.startswith("l1."):
            name = k[3:]
            if name in ("four_d", "decompose", "contact"):
                kw.setdefault("l1", {})[name] = v.lower() in ("1", "true", "yes")
            else:
                kw.setdefault("l1", {})[name] = float(v)
        elif k in ("int_ff",):
            kw[k] = v.lower() in ("1", "true", "yes")
        else:
            kw[k] = float(v)
    return shipped_case(**kw)


def sweep(dirs):
    print("=== SWEEP around the shipped 4-D config, one group at a time ===")
    run(shipped_case(), "shipped", dirs)
    for kx, kv in ((32, 28), (32, 36), (40, 28), (48, 32), (24, 20), (32, 14)):
        run(shipped_case(gains=dict(k_x=kx, k_v=kv)), f"k_x={kx} k_v={kv}", dirs)
    for kR, kw in ((2.0, 2.0), (2.0, 2.5), (3.0, 2.0), (1.5, 1.5), (2.5, 1.5)):
        run(shipped_case(gains=dict(k_R=kR, k_w=kw)), f"k_R={kR} k_w={kw}", dirs)
    for wt in (0.5, 1.0, 4.0):
        run(shipped_case(l1=dict(omega_c_t=wt)), f"omega_c_t={wt}", dirs)
    for wr in (0.25, 1.0):
        run(shipped_case(l1=dict(omega_c_r=wr)), f"omega_c_r={wr}", dirs)
    for wq in (0.25, 1.0):
        run(shipped_case(l1=dict(omega_c_q=wq)), f"omega_c_q={wq}", dirs)
    for wx in (0.05, 0.1, 0.5, 1.0):
        run(shipped_case(l1=dict(omega_x=wx)), f"omega_x={wx}", dirs)
    run(shipped_case(int_ff=False), "u3 internal ff OFF", dirs)
    for wqq in (0.05, 0.5, 1.0):
        run(shipped_case(l1=dict(omega_q=wqq)), f"omega_q={wqq}", dirs)
    for ky, dy in ((8.0, 8.0), (50.0, 20.0), (20.0, 20.0), (20.0, 6.0)):
        run(shipped_case(gains=dict(ky=(ky, ky, ky, 0.3), dy=(dy, dy, dy, 0.3))), f"K_y={ky} D_y={dy}", dirs)
    for mrd in ((0.132, 0.112, 0.116), (0.155, 0.181, 0.167), (0.0777, 0.0907, 0.0834)):
        run(shipped_case(gains=dict(mrd=mrd)), f"M_r_d={mrd}", dirs)
    run(shipped_case(ee_ref="world"), "EE ref world", dirs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--case", default=None)
    ap.add_argument("--dir", default="both", choices=("along", "lateral", "both"))
    ap.add_argument("--trace", action="store_true")
    a = ap.parse_args()
    np.set_printoptions(precision=3, suppress=True, linewidth=160)
    dirs = ("along", "lateral") if a.dir == "both" else (a.dir,)
    if a.validate:
        run(shipped_case(), "shipped 4-D", dirs, trace=a.trace)
    if a.sweep:
        sweep(dirs)
    if a.case is not None:
        run(parse_case(a.case), "case", dirs, trace=a.trace)
    if not (a.validate or a.sweep or a.case is not None):
        run(shipped_case(), "shipped 4-D", dirs, trace=a.trace)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Gate for the SO(3) super-twisting extended state observer / disturbance estimator
(fsc_autopilot_ros2: px4_vtol_direct_actuator/so3_steso.py).

    ~/envs/isaac_env/bin/python3 tests/test_so3_steso.py [--quick]

What is checked, in order, and why each check exists:
  1. Log/Exp on SO(3): round trips, the small-angle branch, the near-pi branch (the paper's
     Euler observer has no such thing; on the group the innovation must be right everywhere).
  2. THE SIGN, EMPIRICALLY: a known +1 N.m pitch torque the model does not contain must read
     as +1/Iyy rad/s^2 (and +1 N.m of `torque`), other axes ~0, both modes. The paper's eq. 12
     has the correction sign flipped relative to its own error dynamics (eq. 13); this project
     has inverted the pitch sign six times; the plant decides.
  3. Finite-time EXACT convergence on a rigid body with the assembled inertia: from a 20 deg
     attitude error the innovation is < 1e-5 rad within 1.5 s and the disturbance estimate is
     exact up to the discretisation chatter (1.1 L dt) -- a ramping sinusoid inside L, a step
     re-acquired in < 0.5 s.
  4. Attitude invariance: the same disturbance seen from a level body at rest and from an
     inverted body tumbling at 1.1 rad/s gives the same estimate; a 120 deg initial error converges.
  5. L too small: a disturbance steeper than L cannot be followed (the trade L makes, measured).
  6. Noise table at the rig's 120 Hz: attitude mode vs mocap jitter, rate mode vs gyro noise,
     across L -- the cube-root / square-root laws in numbers. Printed; loose bounds asserted.
  7. Against a LINEAR extended-state observer whose bandwidth is tuned to the SAME noise floor
     (real-mocap noise, 120 Hz): MEASURED PARITY. The super-twisting estimator tracks a 1 Hz
     moment within +-20% of the linear one and re-acquires a step SLOWER (its third channel can
     only slew at 1.1 L per second; the linear one's correction grows with the error). The
     finite-time exactness of check 3 is a noiseless, 1 kHz property; at this rig's rate and
     noise the two designs are the same filter with different knobs. The check pins parity so a
     change to the discretisation (an implicit scheme, say) shows up here as a number.
  8. The case this project needs, on the plant double (the simulator's own aero + assembled mass
     properties): a hover with a 5 cm CG error the controller is NOT told about. A P-D attitude
     loop parks 12 deg off (or, under a position hold, 1.1 m off -- the position loop tilts the
     reference to fight the drift and the two standing errors cancel in pitch). The estimate
     must read the CG moment (38.8 N x 0.05 m) to 3%, and feeding it forward must take both to
     ~0 -- also with the 30 ms transport delay and real-mocap noise in the loop.
  9. The plumbing: `delay` delivers the command from `delay` ago; `hold` freezes the estimate.
"""
import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
from controller_path import inner_dir                       # noqa: E402
import plant_double as pd                                   # noqa: E402

_spec = importlib.util.spec_from_file_location("so3_steso", os.path.join(inner_dir(), "so3_steso.py"))
st = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(st)

J = np.array([0.265, 0.303, 0.490])          # the assembled inertia (so3.IXX/IYY/IZZ)
Jm = np.diag(J)
QUICK = "--quick" in sys.argv


# ---------------------------------------------------------------------------
#  A rigid body with a KNOWN applied torque and an UNKNOWN disturbance torque.
#  M_model (what the estimator is told) is the applied torque plus the gyroscopic term,
#  i.e. the plant is exactly J w_dot = M_model + d.  sigma_true = d / J.
# ---------------------------------------------------------------------------
def disturbance(t, kind="mixed", amp=0.8):
    if kind == "mixed":           # constant roll, 1 Hz + step pitch, 0.7 Hz yaw
        return np.array([0.3, amp * math.sin(2 * math.pi * 1.0 * t) + (1.0 if t > 2.0 else 0.0),
                         -0.2 * math.cos(2 * math.pi * 0.7 * t)])
    if kind == "gentle":          # for the noise table: slopes well inside L = 5
        return np.array([0.3, 0.3 * math.sin(2 * math.pi * 0.5 * t), -0.1 * math.cos(2 * math.pi * 0.3 * t)])
    if kind == "const":
        return np.array([0.0, 1.0, 0.0])
    if kind == "sine":
        return np.array([0.0, amp * math.sin(2 * math.pi * 1.0 * t), 0.0])
    if kind == "step":
        return np.array([0.0, 1.0 if t > 4.5 else 0.0, 0.0])
    raise ValueError(kind)


def rigid_body_run(est, dt, T, kind="mixed", noise_att=0.0, noise_rate=0.0, seed=0,
                   R0=None, w0=None, R_hat0=None, amp=0.8, both_meas=False):
    """Integrate truth at dt, feed the estimator noisy measurements taken at the START of each
    step (as a mocap sample is), return per-tick (t, sigma_err, e_att, w_err, sigma_hat, sigma_true)."""
    rng = np.random.default_rng(seed)
    R = np.eye(3) if R0 is None else np.array(R0, float)
    w = np.zeros(3) if w0 is None else np.array(w0, float)
    if R_hat0 is not None:
        est.reset(R=R_hat0, omega=np.zeros(3))
    n = int(round(T / dt))
    out = np.zeros((n, 6, 3)); tt = np.arange(n) * dt
    for k in range(n):
        t = tt[k]
        M_cmd = np.array([0.2 * math.sin(math.pi * t), 0.1 * math.cos(2.0 * t), 0.05 * math.sin(t)])
        M_model = M_cmd - np.cross(w, Jm @ w)
        d = disturbance(t, kind, amp)
        s_true = d / J
        w_dot = np.linalg.solve(Jm, M_model + d)
        Rm = R @ st.so3_exp(rng.normal(0.0, math.radians(noise_att), 3)) if noise_att > 0 else R
        wm = w + rng.normal(0.0, math.radians(noise_rate), 3) if noise_rate > 0 else w
        est.step(Rm, M_model, dt, omega=(wm if (est.mode == "rate" or both_meas) else None))
        # the estimate after the step is for t + dt: advance the truth, then compare
        R = R @ st.so3_exp(w * dt)
        w = w + w_dot * dt
        s_true = disturbance(t + dt, kind, amp) / J
        e_att = st.so3_log(est.R_hat.T @ R) if est.mode == "attitude" else np.zeros(3)
        out[k, 0] = est.sigma - s_true
        out[k, 1] = e_att
        out[k, 2] = est.omega_hat - w
        out[k, 3] = est.sigma
        out[k, 4] = s_true
        out[k, 5] = getattr(est, "sigma_filt", est.sigma)
    return tt, out


def rms(x, axis=0):
    return np.sqrt(np.mean(np.asarray(x) ** 2, axis=axis))


# ---------------------------------------------------------------------------
#  Plant-double hover: SE(3) position + attitude hold that believes the NOMINAL CG,
#  while the plant's true CG sits cg_dx forward. The estimator is told the controller's
#  own moment model (nominal geometry + gyroscopics), nothing about the CG.
# ---------------------------------------------------------------------------
CG_NOM = pd.CG.copy()


def alloc_matrix(cg):
    """[Fz, Mx, My, Mz] = B @ T for the four upright rotors about `cg` (body FLU)."""
    B = np.zeros((4, 4))
    for i in range(4):
        r = pd.ROTOR_HUB[i] - cg
        B[0, i] = 1.0
        B[1, i] = r[1]                                          # (r x T z)_x =  y T
        B[2, i] = -r[0]                                         # (r x T z)_y = -x T
        B[3, i] = -pd.ROTOR_SPIN[i] * pd.ROTOR_MOMENT_RATIO     # reaction torque
    return B


def plant_cg_moment(cg_dx):
    """Open-loop: the moment the plant makes with symmetric hover thrust and the CG error,
    minus what it makes without it. The SIGN reference for the estimate."""
    thr = np.sqrt(np.full(4, pd.MASS * pd.G / 4.0 / pd.MAX_THRUST))
    pd.CG = CG_NOM + np.array([cg_dx, 0.0, 0.0])
    try:
        _, M1 = pd.VTOLPlant().forces(thr, 0.0, [0.0, 0.0, 0.0])
    finally:
        pd.CG = CG_NOM
    _, M0 = pd.VTOLPlant().forces(thr, 0.0, [0.0, 0.0, 0.0])
    return M1 - M0


def hover_run(mode, cg_dx=0.05, feedforward=False, delay=0.0, L=20.0, T=12.0, dt=1.0 / 200.0,
              tau_out=0.0, noise_att=0.0, noise_rate=0.0, seed=0, position_hold=True):
    rng = np.random.default_rng(seed)
    pd.CG = CG_NOM + np.array([cg_dx, 0.0, 0.0])            # in force for the whole run
    try:
        plant = pd.VTOLPlant().reset(pos=(0.0, 0.0, 5.0))
        B = alloc_matrix(CG_NOM); Binv = np.linalg.inv(B)
        est = st.SO3STESO(J, L=L, mode=mode, eps="auto", delay=delay, tau_out=tau_out)
        kR, kW, kp, kv = 9.0, 1.8, 2.0, 3.0
        p_ref = np.array([0.0, 0.0, 5.0])
        nd = int(round(delay / dt)); q = [np.sqrt(np.full(4, pd.MASS * pd.G / 4.0 / pd.MAX_THRUST))] * nd
        n = int(round(T / dt)); tt = np.arange(n) * dt
        pitch = np.zeros(n); tq = np.zeros((n, 3)); pos_err = np.zeros(n)
        for k in range(n):
            R, w = plant.R, plant.omega
            Rm = R @ st.so3_exp(rng.normal(0.0, math.radians(noise_att), 3)) if noise_att > 0 else R
            wm = w + rng.normal(0.0, math.radians(noise_rate), 3) if noise_rate > 0 else w
            # --- SE(3) hold, nominal geometry ---
            if position_hold:
                a_des = kp * (p_ref - plant.pos) - kv * plant.vel
            else:
                a_des = np.array([0.0, 0.0, kp * (p_ref[2] - plant.pos[2]) - kv * plant.vel[2]])
            F_w = pd.MASS * (a_des + np.array([0.0, 0.0, pd.G]))
            b3 = F_w / np.linalg.norm(F_w)
            b2 = np.cross(b3, np.array([1.0, 0.0, 0.0])); b2 /= np.linalg.norm(b2)
            b1 = np.cross(b2, b3)
            Rd = np.column_stack([b1, b2, b3])
            eR = 0.5 * st.vee(Rd.T @ R - R.T @ Rd)
            M_des = -kR * eR - kW * w
            if feedforward:
                M_des = M_des - (est.torque_filt if tau_out > 0 else est.torque)
            Fz = float(F_w @ (R @ np.array([0.0, 0.0, 1.0])))
            Tr = np.clip(Binv @ np.array([Fz, M_des[0], M_des[1], M_des[2]]), 0.5, pd.MAX_THRUST)
            thr = np.sqrt(Tr / pd.MAX_THRUST)                  # momentum model: T = thr^2 * max at rest
            # --- transport delay on the command path ---
            q.append(thr); thr_applied = q.pop(0)
            # --- the estimator is handed the FRESH command and its own `delay` ---
            M_fresh = (B @ (thr ** 2 * pd.MAX_THRUST))[1:4] - np.cross(w, Jm @ w)
            est.step(Rm, M_fresh, dt, omega=wm)
            plant.step(thr_applied, 0.0, [0.0, 0.0, 0.0], dt)
            pitch[k] = plant.rpy[1]; tq[k] = est.torque; pos_err[k] = np.linalg.norm(plant.pos - p_ref)
    finally:
        pd.CG = CG_NOM
    w1 = tt > T - 3.0
    return dict(pitch_steady=float(pitch[w1].mean()), pitch_max=float(np.abs(pitch).max()),
                torque=tq[w1].mean(axis=0), torque_sd=tq[w1].std(axis=0),
                pos_err=float(pos_err[w1].mean()), z=float(plant.pos[2]))


# ---------------------------------------------------------------------------
def main():
    bad = []

    def check(cond, msg):
        if not cond:
            bad.append(msg)
        return cond

    # ---- 1. Log / Exp ------------------------------------------------------
    rng = np.random.default_rng(3)
    worst = 0.0
    for _ in range(200):
        v = rng.normal(size=3); v *= rng.uniform(1e-8, math.pi - 1e-3) / np.linalg.norm(v)
        worst = max(worst, float(np.linalg.norm(st.so3_log(st.so3_exp(v)) - v)))
    v = np.array([0.0, math.pi, 0.0]); Rpi = st.so3_exp(v); lp = st.so3_log(Rpi)
    near = st.so3_log(st.so3_exp(np.array([0.3, -0.2, 0.1]) / np.linalg.norm([0.3, -0.2, 0.1]) * (math.pi - 1e-6)))
    print(f"1. Log/Exp round trip worst {worst:.2e} rad over 200 draws; Log(exp(pi y)) = {np.array2string(lp, precision=4)};"
          f" |Log| at pi-1e-6 = {np.linalg.norm(near):.6f}")
    check(worst < 1e-9, f"Log/Exp round trip {worst:.2e}")
    check(abs(abs(lp[1]) - math.pi) < 1e-6 and abs(lp[0]) < 1e-6 and abs(lp[2]) < 1e-6, "Log at pi wrong")
    check(abs(np.linalg.norm(near) - (math.pi - 1e-6)) < 1e-5, "Log near pi wrong")
    Rt = st.so3_exp(np.array([0.1, 0.2, -0.3]))
    check(np.allclose(Rt @ Rt.T, np.eye(3), atol=1e-12) and abs(np.linalg.det(Rt) - 1) < 1e-12, "Exp not in SO(3)")

    # ---- 2. the sign, empirically ------------------------------------------
    print("2. sign: +1 N.m pitch torque the model does not contain")
    for mode in ("rate", "attitude"):
        est = st.SO3STESO(J, L=20.0, mode=mode)
        tt, out = rigid_body_run(est, 1e-3, 4.0, kind="const")
        w1 = tt > 2.0
        sig = out[w1, 3].mean(axis=0); tq = Jm @ sig
        print(f"   {mode:8s}: sigma_hat = {np.array2string(sig, precision=3)} rad/s^2 (expect +{1/J[1]:.3f} on y),"
              f" torque = {np.array2string(tq, precision=3)} N.m")
        check(abs(tq[1] - 1.0) < 0.03 and abs(tq[0]) < 0.03 and abs(tq[2]) < 0.03, f"{mode}: sign/magnitude wrong: {tq}")

    # ---- 3. finite-time exact convergence -----------------------------------
    print("3. finite-time exact convergence, rigid body, dt 1 ms, L 30 (disturbance slope 16.6 rad/s^3)")
    L, dt = 30.0, 1e-3
    est = st.SO3STESO(J, L=L, mode="attitude")
    tt, out = rigid_body_run(est, dt, 6.0, R_hat0=st.so3_exp(np.radians([0.0, 20.0, 0.0])))
    e_at = lambda t0: float(np.linalg.norm(out[int(t0 / dt) - 1, 1]))
    chatter = 1.1 * L * dt
    s_err = rms(out[(tt > 3.0), 0])
    # step at t=2: time to get back under 3x chatter
    k2 = int(2.0 / dt); back = next((k for k in range(k2, len(tt)) if np.all(np.abs(out[k:k + 50, 0, 1]) < 3 * chatter)), None)
    t_back = (tt[back] - 2.0) if back is not None else float("inf")
    print(f"   attitude: |e| {e_at(0.5):.1e} @0.5s  {e_at(1.0):.1e} @1.0s  {e_at(1.5):.1e} @1.5s;"
          f" sigma err rms t>3 {np.array2string(s_err, precision=3)} vs chatter 1.1 L dt = {chatter:.3f};"
          f" step re-acquired in {t_back:.2f} s")
    check(e_at(1.5) < 1e-5 and e_at(3.0) < 1e-5, "attitude mode: innovation not converged by 1.5 s")
    check(np.all(s_err < 1.5 * chatter), f"attitude mode: sigma error {s_err} above chatter {chatter}")
    check(t_back < 0.5, f"attitude mode: step re-acquired in {t_back:.2f} s")
    est = st.SO3STESO(J, L=L, mode="rate")
    tt, out = rigid_body_run(est, dt, 6.0)
    s_err_r = rms(out[(tt > 3.0), 0]); w_err = float(np.abs(out[(tt > 3.0), 2]).max())
    back = next((k for k in range(k2, len(tt)) if np.all(np.abs(out[k:k + 50, 0, 1]) < 3 * chatter)), None)
    t_back_r = (tt[back] - 2.0) if back is not None else float("inf")
    print(f"   rate    : sigma err rms t>3 {np.array2string(s_err_r, precision=3)}; |w_hat - w| max {w_err:.1e};"
          f" step re-acquired in {t_back_r:.2f} s")
    check(np.all(s_err_r < 1.5 * chatter) and w_err < 1e-3, f"rate mode not exact: {s_err_r}, {w_err}")
    check(t_back_r < 0.5, f"rate mode: step re-acquired in {t_back_r:.2f} s")

    # ---- 4. attitude invariance --------------------------------------------
    print("4. attitude invariance: level at rest vs inverted and tumbling (0.5, 0, 1.0) rad/s")
    est_a = st.SO3STESO(J, L=L, mode="attitude"); tta, oa = rigid_body_run(est_a, dt, 6.0)
    est_b = st.SO3STESO(J, L=L, mode="attitude")
    R0 = st.so3_exp(np.array([math.pi, 0.0, 0.0])) @ st.so3_exp(np.array([0.3, 0.2, 0.1]))
    ttb, ob = rigid_body_run(est_b, dt, 6.0, R0=R0, w0=(0.5, 0.0, 1.0),
                             R_hat0=R0 @ st.so3_exp(np.radians([0.0, 120.0, 0.0])))
    w1 = tta > 3.0
    diff = rms(oa[w1, 3] - ob[w1, 3])
    t_conv = next((tta[k] for k in range(len(tta)) if np.all(np.linalg.norm(ob[k:, 1], axis=1) < 1e-5)), float("inf"))
    print(f"   estimate difference rms t>3 {np.array2string(diff, precision=3)} (chatter {chatter:.3f});"
          f" 120 deg initial error on the tumbling body converged at t = {t_conv:.2f} s")
    check(np.all(diff < 2.0 * chatter), f"estimate depends on attitude: {diff}")
    check(t_conv < 3.0, f"120 deg initial error did not converge ({t_conv})")

    # ---- 5. L too small ------------------------------------------------------
    est = st.SO3STESO(J, L=5.0, mode="rate"); tt, out = rigid_body_run(est, dt, 6.0)
    lag = rms(out[(tt > 3.0), 0])
    print(f"5. L = 5 against a 16.6 rad/s^3 pitch slope: sigma err rms {np.array2string(lag, precision=3)}"
          f" -- pitch cannot be followed (roll/yaw, inside L, still exact)")
    check(lag[1] > 1.0 and lag[0] < 1.5 * 1.1 * 5.0 * dt and lag[2] < 1.5 * 1.1 * 5.0 * dt, f"L trade-off not as expected {lag}")

    # ---- 6. noise table at 120 Hz ------------------------------------------
    dt120 = 1.0 / 120.0
    print("6. noise floor at 120 Hz (eps auto), sigma error rms [rad/s^2] on PITCH over t>4 of a gentle disturbance")
    Ls = (5.0, 20.0, 80.0)
    print("   attitude mode (mocap jitter, deg)      " + "".join(f"L={L_:<5g}   " for L_ in Ls) + "   1.1 L dt")
    tab_a = {}
    for na in (0.0, 0.0004, 0.02, 0.1):
        row = []
        for L_ in Ls:
            est = st.SO3STESO(J, L=L_, mode="attitude", eps="auto")
            tt, out = rigid_body_run(est, dt120, 8.0, kind="gentle", noise_att=na, seed=1)
            row.append(float(rms(out[(tt > 4.0), 0, 1])))
        tab_a[na] = row
        print(f"   {na:<8g}                              " + "".join(f"{r:<9.3f}" for r in row)
              + "   " + " ".join(f"{1.1 * L_ * dt120:.3f}" for L_ in Ls))
    print("   rate mode (gyro noise, deg/s)          " + "".join(f"L={L_:<5g}   " for L_ in Ls))
    tab_r = {}
    for nr in (0.0, 0.1, 1.0):
        row = []
        for L_ in Ls:
            est = st.SO3STESO(J, L=L_, mode="rate", eps="auto")
            tt, out = rigid_body_run(est, dt120, 8.0, kind="gentle", noise_rate=nr, seed=1)
            row.append(float(rms(out[(tt > 4.0), 0, 1])))
        tab_r[nr] = row
        print(f"   {nr:<8g}                              " + "".join(f"{r:<9.3f}" for r in row))
    check(all(tab_a[0.0][i] < 1.5 * 1.1 * Ls[i] * dt120 for i in range(3)), f"attitude zero-noise floor above chatter {tab_a[0.0]}")
    check(all(tab_r[0.0][i] < 1.5 * 1.1 * Ls[i] * dt120 for i in range(3)), f"rate zero-noise floor above chatter {tab_r[0.0]}")
    check(tab_a[0.1][1] < 2.0, f"attitude mode 0.1 deg / L 20: {tab_a[0.1][1]} rad/s^2")
    check(tab_r[1.0][1] < 1.0, f"rate mode 1 deg/s / L 20: {tab_r[1.0][1]} rad/s^2")
    check(tab_a[0.1][1] > tab_a[0.0004][1] and tab_a[0.1][2] > tab_a[0.1][0], "attitude noise floor not monotone")
    check(tab_r[1.0][1] > tab_r[0.1][1], "rate noise floor not monotone")

    # ---- 7. vs a linear ESO at the same noise floor --------------------------
    print("7. super-twisting vs LINEAR ESO at 120 Hz, real-mocap noise (0.05 deg / 0.5 deg/s), pitch [rad/s^2]:")
    print("   the linear ESO's bandwidth wo is tuned so its error on a CONSTANT 1 N.m (the noise floor) matches the STESO's")
    for mode, nz in (("rate", dict(noise_rate=0.5)), ("attitude", dict(noise_att=0.05))):
        def err(est, kind, amp=0.8):
            tt, o = rigid_body_run(est, dt120, 8.0, kind=kind, seed=2, amp=amp, **nz)
            return float(rms(o[tt > 4.0, 0, 1]))
        s_const = err(st.SO3STESO(J, L=20.0, mode=mode, eps="auto"), "const")
        wos = np.geomspace(4.0, 160.0, 24) if not QUICK else np.geomspace(4.0, 160.0, 12)
        lc = [err(st.LinearESO(J, wo=w, mode=mode), "const") for w in wos]
        i = min(max(int(np.searchsorted(lc, s_const)), 1), len(wos) - 1)
        wo_m = float(np.interp(s_const, [lc[i - 1], lc[i]], [wos[i - 1], wos[i]]))
        l_const = err(st.LinearESO(J, wo=wo_m, mode=mode), "const")
        ratios = []
        for amp in (0.3, 0.8):
            s_sine = err(st.SO3STESO(J, L=20.0, mode=mode, eps="auto"), "sine", amp)
            l_sine = err(st.LinearESO(J, wo=wo_m, mode=mode), "sine", amp)
            ratios.append(l_sine / max(s_sine, 1e-9))
            print(f"   {mode:8s} L 20 (floor {s_const:.3f}) vs linear wo {wo_m:.1f} (floor {l_const:.3f}):"
                  f" 1 Hz sine {amp:.1f} N.m  STESO {s_sine:.3f}  linear {l_sine:.3f}  ratio {ratios[-1]:.2f}x")
        # step re-acquisition after a 1 N.m step: time back under 3x the noise floor for 30 ticks
        times = []
        for name, est in (("STESO", st.SO3STESO(J, L=20.0, mode=mode, eps="auto")), ("linear", st.LinearESO(J, wo=wo_m, mode=mode))):
            tt, o = rigid_body_run(est, dt120, 8.0, kind="step", seed=2, **nz)
            k0 = int(4.5 / dt120); thr = 3.0 * s_const
            k = next((k for k in range(k0, len(tt) - 30) if np.all(np.abs(o[k:k + 30, 0, 1]) < thr)), None)
            times.append((tt[k] - 4.5) if k is not None else float("inf"))
        print(f"   {mode:8s} 1 N.m step at t=4.5, back under {thr:.2f}: STESO {times[0]:.3f} s, linear {times[1]:.3f} s"
              f" (STESO slews at most 1.1 L = {1.1 * 20:.0f} rad/s^3: {1.0 / J[1] / 22:.3f} s for this step)")
        check(all(0.6 < r < 1.6 for r in ratios), f"{mode}: STESO/linear parity broken: ratios {ratios}")
        check(times[0] < 0.5 and times[1] < 0.5, f"{mode}: step re-acquisition {times}")

    # ---- 8. the plant double, CG error --------------------------------------
    Mcg = plant_cg_moment(0.05)
    print(f"8. plant double: CG 5 cm forward of what the controller believes. Open-loop, symmetric hover thrust:"
          f" plant moment {np.array2string(Mcg, precision=3)} N.m (T dx = {pd.MASS * pd.G * 0.05:+.3f} on y)")
    check(abs(Mcg[1] - pd.MASS * pd.G * 0.05) < 1e-3, "plant CG moment sanity")
    rows = [("rate", dict(feedforward=False, position_hold=False)),
            ("rate", dict(feedforward=False)),
            ("rate", dict(feedforward=True)),
            ("attitude", dict(feedforward=True)),
            ("rate", dict(feedforward=True, delay=0.03, noise_att=0.05, noise_rate=0.5, tau_out=0.1)),
            ("attitude", dict(feedforward=True, delay=0.03, noise_att=0.05, noise_rate=0.5, tau_out=0.1))]
    if QUICK:
        rows = rows[:4]
    res = []
    for mode, kw in rows:
        r = hover_run(mode, **kw)
        res.append((mode, kw, r))
        tag = ("ff " if kw.get("feedforward") else "P-D") + ("" if kw.get("position_hold", True) else ", attitude+altitude only") \
            + (f", delay {kw['delay']*1e3:.0f} ms" if kw.get("delay") else "") \
            + (f", noise {kw['noise_att']} deg / {kw['noise_rate']} deg/s, tau_out {kw['tau_out']}" if kw.get("noise_att") else "")
        print(f"   {mode:8s} {tag:56s}: pitch steady {r['pitch_steady']:+6.2f} deg (max |{r['pitch_max']:.1f}|),"
              f" torque_y {r['torque'][1]:+.3f} +- {r['torque_sd'][1]:.3f} N.m, pos err {r['pos_err']:.2f} m")
    att_only, pd_hold = res[0][2], res[1][2]
    check(abs(att_only["pitch_steady"]) > 3.0, f"P-D attitude loop should park at a standing pitch error: {att_only['pitch_steady']}")
    check(pd_hold["pos_err"] > 0.5, f"P-D position hold should park off its setpoint: {pd_hold['pos_err']} m")
    check(abs(pd_hold["torque"][1] - Mcg[1]) < 0.03 * abs(Mcg[1]), f"estimate without feed-forward {pd_hold['torque'][1]} vs {Mcg[1]}")
    for mode, kw, r in res[2:]:
        check(abs(r["torque"][1] - Mcg[1]) < 0.03 * abs(Mcg[1]), f"{mode} {kw}: torque_y {r['torque'][1]} vs {Mcg[1]}")
        check(abs(r["torque"][0]) < 0.1 and abs(r["torque"][2]) < 0.1, f"{mode} {kw}: roll/yaw torque not ~0: {r['torque']}")
        check(abs(r["pitch_steady"]) < 0.3, f"{mode} {kw}: standing pitch with feed-forward {r['pitch_steady']}")
        check(r["pos_err"] < 0.05, f"{mode} {kw}: position error with feed-forward {r['pos_err']}")

    # ---- 9. plumbing ---------------------------------------------------------
    est = st.SO3STESO(J, L=20.0, mode="rate", delay=0.03)
    seen = []
    for k in range(8):
        seen.append(float(est._applied(np.array([float(k), 0.0, 0.0]))[0]))
        est.t += 0.01
    est2 = st.SO3STESO(J, L=20.0, mode="rate"); rigid_body_run(est2, 1e-3, 1.0, kind="const")
    s_before = est2.sigma.copy(); est2.hold = True
    for _ in range(200):
        est2.step(np.eye(3), np.zeros(3), 1e-3, omega=np.array([0.0, 0.5, 0.0]))
    print(f"9. delay 30 ms at 10 ms ticks: applied command index {seen} (expect the command from 3 ticks ago);"
          f" hold: sigma {np.array2string(s_before, precision=3)} -> {np.array2string(est2.sigma, precision=3)} under a 0.5 rad/s rate error")
    check(seen[3:] == [0.0, 1.0, 2.0, 3.0, 4.0], f"delay buffer wrong: {seen}")
    check(np.allclose(est2.sigma, s_before), "hold did not freeze sigma")

    print()
    if bad:
        print(f"FAIL ({len(bad)}):")
        for b in bad:
            print("  -", b)
        return 1
    print("PASS -- SO(3) STESO: right sign, exact when noiseless, attitude-invariant, at parity with a matched linear ESO"
          " under noise, reads a 5 cm CG error to 3% and cancels it on the plant double")
    return 0


if __name__ == "__main__":
    sys.exit(main())

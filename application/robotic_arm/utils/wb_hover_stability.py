#!/usr/bin/env python3
"""
wb_hover_stability.py — closed-loop hover stability of the whole-body law,
offline (system python3, no Isaac, no PX4, no ROS).

WHY THIS EXISTS (2026-08-23). Flying the AM-T650 whole-body rig showed the
DIRECT hold is UNSTABLE at the calibrated thrust coefficient and only bounded
because the shipped yaml carries a +15% allocator-kf mismatch that quietly cuts
loop gain ~13% (Command.md 7.14.4, runs A-D — including run D, which removes
the reference governor entirely and diverges just the same, so the fault is the
TUNE, not the reference path). Re-tuning by flying is ~3 minutes per candidate
and every unstable one crashes the sim; this closes the same loop in ~5 s so a
sweep is practical, and the winners then go to the full stack for confirmation.

WHAT IS MODELLED — deliberately the loop the flight showed to be at fault:
  * the EXACT law (utils_controller.MatlabController) and the EXACT model
    (transition_planner.make_params_t650, i.e. the T650 body override in the
    model frame with r_e at the gripper), so gains map 1:1 onto the yaml;
  * rigid-body dynamics M(q) xi_dot + C xi + g = [u1*e3; tau_body; tau_joint],
    the contract verified against dynamics(): at hover g = [0,0,36.75 N] with
    the known +0.72 N.m arm moment in g[3:6];
  * the ALLOCATOR with a believable-vs-true kf split (the whole point) and the
    first-order ROTOR LAG, exact zero-order-hold form, lambda = 10.0265 1/s;
  * the joint-torque clamp (tau_max) the law and plant both apply;
  * the ARM SERVO (2026-09-03): PWM-mode back-EMF droop, tau_app = clip(tau) -
    Kt^2/R * qd on joints 2 and 3, identified from the 0902/0903 flights. Before
    this the simulated arm delivered its command exactly, which is the same
    idealisation that kept Isaac from reproducing the hardware. `--servo ideal`
    restores it for an A/B.

WHAT IS NOT: ground contact, PX4's own loops (DIRECT bypasses them), DDS
latency, aerodynamics, sensor noise. So this predicts STABILITY and the
oscillation mode, not tracking numbers. It is a screening tool — the flight is
still the gate.

Usage:
    python3 wb_hover_stability.py --validate       # reproduce flight runs A/B
    python3 wb_hover_stability.py --sweep          # gain search at matched kf
    python3 wb_hover_stability.py --gains k_R=4,k_w=3
"""

import argparse
import itertools
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "extensions", "fsc_aerial_manipulation"))

from fsc_aerial_manipulation.robotic_arm.utils_controller import (  # noqa: E402
    control_params as CP,
    controller as C,
)
from fsc_aerial_manipulation.robotic_arm.utils_planner import (  # noqa: E402
    transition_planner as TP,
)
from fsc_aerial_manipulation.robotic_arm import servo_model as SM  # noqa: E402

# ---------------------------------------------------------------- plant knobs
# STALE, FOUND 2026-09-03, NOT CHANGED HERE (changing them re-bases every sweep
# result recorded against this tool -- decide deliberately, then re-validate):
#   KF_TRUE    should be t650_params.ROTOR_CONSTANT = 4.041283e-05 since the
#              2026-08-25 thrust re-anchor; the value below is 15.8 % HIGH, i.e.
#              this tool still flies the pre-re-anchor, too-strong plant.
#   KF_SHIPPED should be the sim yaml's alloc_thrust_coeff = 4.6474755e-05.
#              The RATIO (+15 %) is right, so the injection is modelled; the
#              absolute loop gain is not.
#   make_gains()'s mrd is the PRE-retune M_r_d; the shipped yaml has carried
#              wb_mrd = (0.116522, 0.136107, 0.125102) since 2026-08-23.
# Until those three are refreshed, validate() disagrees with both flight
# campaigns (it calls the +15 % run divergent in every combination tried, with
# and without the servo model, at both M_r_d values) -- trust the ORDERING only.
KF_TRUE = 4.679931e-05        # the calibrated plant (t650_params, tuned)
KF_SHIPPED = 5.38192065e-05   # what the yaml's allocator BELIEVES (+15%)
KM_TRUE = 2.474152e-06        # rolling-moment coefficient (t650_params tuned)
LAMBDA_ROTOR = 10.0265        # 1/s, MN4010 spin-up bandwidth (tau 99.7 ms)
OMEGA_IDLE, OMEGA_MAX = 64.0603, 730.0507
HOME = np.array([0.0, np.deg2rad(40.0), np.deg2rad(40.0), 0.0])

# Rotor geometry, from the law's own RotorMixer (AM_xfwd channel order).
ROTOR_POS = np.array([[0.229907, 0.229907, 0.182191],
                      [-0.229907, -0.229907, 0.182191],
                      [0.229907, -0.229907, 0.182191],
                      [-0.229907, 0.229907, 0.182191]])
ROT_DIR = np.array([-1.0, -1.0, 1.0, 1.0])


def make_gains(**over):
    """The whole-body yaml's gain block as the law's own ControlParams.

    Uses ControlParams rather than a look-alike so the attribute names, the
    diagonal shapes and the shaped/natural switch are the law's, not a copy
    that can drift. Defaults are the AM-T650 whole-body yaml as the node prints
    it at startup (k_x=16 k_v=12 k_R=2 k_w=1.5, K_y=[2 2 2 1], D_y=[4 4 4 1],
    GMO 0.5/0.1/0.1, shaped M_Y, dls 0.3, tau_max 3).
    """
    # M_r_d is the yaml's PER-AXIS wb_mrd_x/y/z — the T650 I0 diagonal in the
    # model frame, NOT a scalar 0.065. Accepts a 3-vector or a scalar.
    v = dict(k_x=16.0, k_v=12.0, k_R=2.0, k_w=1.5,
             mrd=(0.077681, 0.090738, 0.083401),
             ky=(2.0, 2.0, 2.0, 1.0), dy=(4.0, 4.0, 4.0, 1.0),
             my=(1.0, 1.0, 1.0, 1.0), ko=(0.5, 0.1, 0.1),
             use_gmo=True, dls_lambda=0.3, tau_max=3.0)
    for k, val in over.items():
        if k not in v:
            raise KeyError(f"unknown gain '{k}' (have {sorted(v)})")
        v[k] = val
    ky = np.atleast_1d(v["ky"]).astype(float)
    dy = np.atleast_1d(v["dy"]).astype(float)
    if ky.size == 1:
        ky = np.array([ky[0]] * 3 + [1.0])
    if dy.size == 1:
        dy = np.array([dy[0]] * 3 + [1.0])
    kt, kr, kq = v["ko"]
    return CP.ControlParams(
        k_x=v["k_x"], k_v=v["k_v"], k_R=v["k_R"], k_w=v["k_w"],
        M_r_d=np.diag(np.broadcast_to(np.atleast_1d(
            np.asarray(v["mrd"], float)), (3,)).copy()),
        K_y=np.diag(ky), D_y=np.diag(dy),
        use_gmo=v["use_gmo"],
        K_o=np.diag([kt] * 3 + [kr] * 3 + [kq] * 4),
        impedance_mode="shaped", M_Y=np.diag(np.atleast_1d(v["my"])),
        dls_lambda=v["dls_lambda"], tau_max=v["tau_max"],
        source="wb_hover_stability", scenario="am_t650_whole_body")


def gains_str(g):
    return (f"k_R={g.k_R:g} k_w={g.k_w:g} K_y={g.K_y[0,0]:g} "
            f"D_y={g.D_y[0,0]:g} k_x={g.k_x:g} k_v={g.k_v:g}")


def _alloc_matrix():
    B = np.zeros((4, 4))
    for i, r in enumerate(ROTOR_POS):
        B[0, i] = 1.0
        B[1, i] = r[1]
        B[2, i] = -r[0]
        B[3, i] = ROT_DIR[i] * KM_TRUE / KF_TRUE
    return B


_B = _alloc_matrix()
_B_PINV = np.linalg.pinv(_B)


def allocate(thrust, tau_body, kf_believed):
    """(u1, tau_body) -> commanded rotor speeds, through the believed kf.

    The mismatch lives HERE and nowhere else: a believed kf HIGHER than truth
    makes this ask for a smaller omega than the wrench really needs, so the
    plant under-delivers and the loop runs at reduced gain.
    """
    f = np.maximum(_B_PINV @ np.array([thrust, *tau_body]), 0.0)
    w = np.sqrt(f / kf_believed)
    return np.clip(w, 0.0, OMEGA_MAX)


def wrench_from_rotors(omega):
    """True thrust/torque produced by actual rotor speeds (calibrated kf)."""
    f = KF_TRUE * omega ** 2
    thrust = f.sum()
    tau = np.zeros(3)
    for i, r in enumerate(ROTOR_POS):
        tau[0] += r[1] * f[i]
        tau[1] += -r[0] * f[i]
        tau[2] += ROT_DIR[i] * KM_TRUE * omega[i] ** 2
    return thrust, tau


def simulate(gains, kf_believed=KF_SHIPPED, t_end=25.0, dt=1.0 / 250.0,
             tilt0_deg=0.5, params=None, verbose=False, delay_ms=0.0,
             servo="pwm"):
    """Closed-loop hover from a small attitude perturbation.

    Returns dict with the pitch history and a growth verdict. The perturbation
    is what a real engagement always has (a settled hover is never exactly the
    reference), and a stable tune must shrink it.
    """
    p = params if params is not None else TP.make_params_t650()
    n = p["n"]
    ctrl = C.MatlabController(p, gains)
    srv = servo if not isinstance(servo, str) else (
        None if servo == "ideal" else SM.DynamixelPwmServo(
            tau_cap=np.full(4, gains.tau_max)))

    # ---- initial state: hover at 1.2 m, arm at home, small pitch offset ----
    R0 = C.joint_rotation(np.array([0.0, 1.0, 0.0]), np.deg2rad(tilt0_deg))
    X = np.zeros(12 + 3 * n + 6)
    X[0:3] = [0.0, 0.0, 1.2]
    X[3:12] = R0.flatten(order="F")
    X[12:12 + n] = HOME

    # ---- reference: the static hold the governor streams --------------------
    dyn0 = C.dynamics(X, p)
    x_cd = X[0:3] + np.eye(3) @ dyn0["r_0c_0"]
    r_ed = X[0:3] + np.eye(3) @ dyn0["r_0e_0"]
    b1_de = (np.eye(3) @ dyn0["R_e_0"])[:, 0]
    z3 = np.zeros(3)
    ref = {"x_cd": x_cd, "x_cd_dot": z3, "x_cd_ddot": z3,
           "x_cd_d3": z3, "x_cd_d4": z3,
           "b1_d": np.array([1.0, 0.0, 0.0]), "b1_d_dot": z3, "b1_d_ddot": z3,
           "r_ed": r_ed, "r_ed_dot": z3, "r_ed_ddot": z3,
           "b1_de": b1_de, "b1_de_dot": z3, "b1_de_ddot": z3,
           "q_d": HOME.copy(), "qdot_d": np.zeros(n)}

    omega_rot = np.full(4, np.sqrt(sum(p["m_i"]) * p["g"] / 4.0 / KF_TRUE))
    # TRANSPORT DELAY. The law does not talk to the plant directly on the rig:
    # it publishes ActuatorMotors over DDS, PX4 relays them, and Isaac applies
    # them over the HIL link — tens of ms of pure delay that the law's model
    # knows nothing about. Modelled as a FIFO on the commanded wrench.
    n_del = int(round(delay_ms * 1e-3 / dt))
    hover_w = np.sqrt(sum(p["m_i"]) * p["g"] / 4.0 / KF_TRUE)
    cmd_fifo = [allocate(sum(p["m_i"]) * p["g"], np.zeros(3), kf_believed)
                for _ in range(max(n_del, 0))]
    steps = int(t_end / dt)
    pitch = np.zeros(steps)
    tsr = np.arange(steps) * dt
    diverged = False

    for k in range(steps):
        dyn = C.dynamics(X, p)
        out = ctrl(X, dyn, ref, dt)
        u1 = float(out["u1"])
        tau_body = np.asarray(out["tau_body"], float)
        tau_joint = np.clip(np.asarray(out["tau_joint"], float),
                            -gains.tau_max, gains.tau_max)
        if srv is not None:
            # The servo, not the law: the delivered torque is the commanded one
            # minus Kt^2/R per rad/s of joint speed (servo_model.py).
            tau_joint = srv.applied(tau_joint, X[18 + n:18 + 2 * n])

        # allocator (believed kf) -> rotor lag -> true wrench
        w_cmd = allocate(u1, tau_body, kf_believed)
        if n_del > 0:
            cmd_fifo.append(w_cmd)
            w_cmd = cmd_fifo.pop(0)
        omega_rot = w_cmd + (omega_rot - w_cmd) * np.exp(-LAMBDA_ROTOR * dt)
        thrust_a, tau_a = wrench_from_rotors(omega_rot)

        Q = np.concatenate([[0.0, 0.0, thrust_a], tau_a, tau_joint])
        xi = np.concatenate([X[12 + n:15 + n], X[15 + n:18 + n],
                             X[18 + n:18 + 2 * n]])
        xi_dot = np.linalg.solve(dyn["M"], Q - dyn["C"] @ xi - dyn["g"])

        # integrate (semi-implicit Euler; the 4 ms tick is the control step)
        xi = xi + xi_dot * dt
        v0, w0, qd = xi[0:3], xi[3:6], xi[6:6 + n]
        R = X[3:12].reshape(3, 3, order="F")
        X[0:3] = X[0:3] + R @ v0 * dt
        R = R @ C.joint_rotation(w0 / (np.linalg.norm(w0) + 1e-12),
                                 np.linalg.norm(w0) * dt)
        u, _, vt = np.linalg.svd(R)          # re-orthonormalise
        R = u @ vt
        X[3:12] = R.flatten(order="F")
        X[12:12 + n] = X[12:12 + n] + qd * dt
        X[12 + n:18 + n] = np.concatenate([v0, w0])
        X[18 + n:18 + 2 * n] = qd

        pitch[k] = np.degrees(np.arcsin(np.clip(-R[2, 0], -1.0, 1.0)))
        if not np.isfinite(pitch[k]) or abs(pitch[k]) > 60.0:
            diverged = True
            pitch[k:] = np.nan
            break

    # growth verdict: compare the last third's amplitude with the first third
    m = np.isfinite(pitch)
    tv, pv = tsr[m], pitch[m]
    if len(pv) < 100:
        return dict(stable=False, diverged=True, growth=np.inf, amp_end=np.inf,
                    t=tv, pitch=pv, t_lost=tv[-1] if len(tv) else 0.0)
    third = len(pv) // 3
    a0 = np.abs(pv[:third] - pv[:third].mean()).max()
    a1 = np.abs(pv[-third:] - pv[-third:].mean()).max()
    growth = a1 / max(a0, 1e-9)
    stable = (not diverged) and growth < 1.0 and a1 < 5.0
    return dict(stable=stable, diverged=diverged, growth=growth, amp_end=a1,
                amp_start=a0, t=tv, pitch=pv,
                t_lost=tv[-1] if diverged else t_end)


def _fmt(r):
    if r["diverged"]:
        return f"DIVERGED at t={r['t_lost']:.1f}s"
    return (f"{'STABLE ' if r['stable'] else 'growing'} "
            f"amp {r['amp_start']:.2f}->{r['amp_end']:.2f} deg "
            f"(x{r['growth']:.2f})")


def validate(servo="pwm"):
    """Reproduce the flight A/B: shipped +15% kf bounded, matched kf diverges."""
    print(f"=== validation against the flown runs (Command.md 7.14.4), "
          f"servo={servo} ===")
    p = TP.make_params_t650()
    g = make_gains()
    for kf, lab, expect in ((KF_SHIPPED, "A) shipped +15% kf", "bounded"),
                            (KF_TRUE, "B) matched kf     ", "diverges")):
        r = simulate(g, kf_believed=kf, params=p, t_end=25.0, servo=servo)
        print(f"  {lab}: {_fmt(r)}   [flight: {expect}]")
    return


def sweep(servo="pwm"):
    """Search for a tune that is stable at the CALIBRATED kf."""
    p = TP.make_params_t650()
    print(f"=== baseline at the calibrated kf, servo={servo} ===")
    base = make_gains()
    print(f"  {gains_str(base)}: "
          f"{_fmt(simulate(base, KF_TRUE, params=p, servo=servo))}")

    print("\n=== attitude loop (k_R, k_w) at the calibrated kf ===")
    best = []
    for k_r, k_w in itertools.product((0.5, 1.0, 2.0, 4.0), (1.5, 3.0, 6.0)):
        gg = make_gains(k_R=k_r, k_w=k_w)
        r = simulate(gg, KF_TRUE, params=p, servo=servo)
        print(f"  k_R={k_r:<4} k_w={k_w:<4}: {_fmt(r)}")
        if r["stable"]:
            best.append((r["amp_end"], k_r, k_w))
    if best:
        best.sort()
        print(f"\n  best attitude pair: k_R={best[0][1]} k_w={best[0][2]}")
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--gains", default=None,
                    help="comma list, e.g. k_R=4,k_w=3")
    ap.add_argument("--kf", default="true", choices=("true", "shipped"))
    ap.add_argument("--t-end", type=float, default=25.0)
    ap.add_argument("--servo", default="pwm", choices=("pwm", "ideal"),
                    help="arm actuator: 'pwm' is the real PWM-mode servo "
                         "(back-EMF droop, servo_model.py); 'ideal' is the "
                         "pre-2026-09-03 exact torque source")
    a = ap.parse_args()
    if a.validate:
        validate(a.servo)
    if a.sweep:
        sweep(a.servo)
    if a.gains:
        over = {}
        for kv in a.gains.split(","):
            k, v = kv.split("=")
            over[k.strip()] = float(v)
        g = make_gains(**over)
        kf = KF_TRUE if a.kf == "true" else KF_SHIPPED
        print(f"{gains_str(g)} @ kf={a.kf}, servo={a.servo}: "
              f"{_fmt(simulate(g, kf, t_end=a.t_end, servo=a.servo))}")
    if not (a.validate or a.sweep or a.gains):
        validate(a.servo)


if __name__ == "__main__":
    main()

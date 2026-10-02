#!/usr/bin/env python3
"""geo_bench.py -- the geometric + L1 adaptive law (Cai et al., CEP 2025) on the SAME
offline circle bench the whole-body law (H1b, 2026-09-27) and the modular adaptive law
(2026-09-30) were tuned on (2026-10-01).

circle_tune_20260927/tools/circle_bench.py is reused UNCHANGED: its mirror plant
(rotor lag, allocator kf/km mismatch, body-fixed force/torque bias, arm friction,
joint stops, transport delay), its feedback realism and its scoring on the TRUE
state. What this file swaps in for the decoupled rig, each a line-by-line port:

  * the LAW -- fsc_autopilot_ros2 single_aerial_manipulator_geometric_l1_direct_actuation:
      l1_geometric_controller.cpp   u_b  (eqs 17/18, omega_d = 0, saturations, tilt clamp)
      l1_adaptive_augmentation.cpp  u_L1 (PWC adaptation eq 20, exact-ZOH predictor eq 19,
                                    LPF eq 23, matched clamps, achieved-wrench anti-windup)
      arm_state_feedforward.cpp     r_os from the arm encoders (+ armff_com_trim)
      client innerLoop()            u = u_b + u_L1, thrust clamp, L1 advanced once per
                                    feedback sample (l1_every ticks; Isaac's mocap is 60 Hz)
  * the REFERENCE BRIDGE -- scripts/tools/decoupled_reference_bridge.py: the planner's
    WholeBodyReference -> base position (x_cd - R0 r_0c(q_d)), velocity/acceleration with
    the bridge's own 8 Hz / 4 Hz smoothing of the arm term's derivatives, actual yaw;
  * the ARM -- 06's position-mode servo emulation (PD 3.0 / 0.25 + gravity + integral 2.0,
    clamp 0.35 N.m, reference slewed at 0.5 rad/s) tracking the planner's q_d. The
    whole-body arm-side friction feed-forward is switched OFF (ff_scale 0): the
    position-mode stack has none; the plant's gearbox friction still acts.

The law's gains are the only free parameters (GEO_HW = the 2026-09-28 hardware set).
Everything is a screen: fly the result.
"""
import contextlib
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "docs", "docs_aerial_manipulator", "circle_tune_20260927", "tools"))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
import circle_bench as CB                                                         # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

CS, ES = CB.CS, CB.ES
R_MODEL = TP.R_MODEL            # actual (FLU) = R_MODEL @ model
N = 4
G = 9.80665                     # fsc::numbers::std_gravity -- law, predictor and bridge share it
MIRROR_YAML = os.path.join(REPO, "docs", "docs_aerial_manipulator", "rtf_profile_20261001",
                           "variants", "geometric_l1_mirror_sim.yaml")

# ---- the 2026-09-28 hardware set (= the report's v7 geometric flights) ------------------
GEO_HW = dict(kp_xy=4.0, kp_z=8.0, kv_xy=6.0, kv_z=10.0,
              kr_xy=1.0, kr_z=0.49, kw_xy=0.55, kw_z=0.35,
              as_v=2.0, as_w=2.0, omega_c=6.0)
GAIN_KEYS = list(GEO_HW)
# yaml key for each bench gain (both axes of a pair get the same value)
YAML_KEYS = dict(kp_xy=("l1geo_kp_x", "l1geo_kp_y"), kp_z=("l1geo_kp_z",),
                 kv_xy=("l1geo_kv_x", "l1geo_kv_y"), kv_z=("l1geo_kv_z",),
                 kr_xy=("l1geo_kr_x", "l1geo_kr_y"), kr_z=("l1geo_kr_z",),
                 kw_xy=("l1geo_komega_x", "l1geo_komega_y"), kw_z=("l1geo_komega_z",),
                 as_v=("l1adapt_as_v",), as_w=("l1adapt_as_omega",), omega_c=("l1adapt_omega_c",))

# 06's position-mode servo emulation (application/robotic_arm/06_px4_t650_aerial_manipulator_free_flight.py)
ARM_HOLD_KP, ARM_HOLD_KD, ARM_HOLD_RATE = 3.0, 0.25, 0.5
ARM_POS_KI, ARM_POS_I_MAX = 2.0, 0.35
PLANNER_BASE_COM = np.array([0.0, -0.017854, 0.0])      # wb_base_com_* (model frame)


def _yaml_scalars(path):
    out = {}
    for line in open(path):
        m = re.match(r"^\s+([a-z][a-z0-9_]*):\s*([^#\n]*?)\s*(#.*)?$", line)
        if m and m.group(1) not in out:
            out[m.group(1)] = m.group(2).strip().strip('"')
    return out


def _f(y, k):
    return float(y[k])


class GeoConfig:
    """Everything the C++ node reads from the yaml, except the gains under tune."""

    def __init__(self, yaml_path=MIRROR_YAML):
        y = _yaml_scalars(yaml_path)
        self.mass = _f(y, "vehicle_mass")
        I = np.array([[_f(y, "l1geo_inertia_xx"), _f(y, "l1geo_inertia_xy"), _f(y, "l1geo_inertia_xz")],
                      [_f(y, "l1geo_inertia_xy"), _f(y, "l1geo_inertia_yy"), _f(y, "l1geo_inertia_yz")],
                      [_f(y, "l1geo_inertia_xz"), _f(y, "l1geo_inertia_yz"), _f(y, "l1geo_inertia_zz")]])
        self.I = I
        self.I_inv = np.linalg.inv(I)
        self.max_pos_err = _f(y, "l1geo_max_pos_err_m")
        self.max_vel_err = _f(y, "l1geo_max_vel_err_mps")
        self.max_tilt_deg = _f(y, "l1geo_max_tilt_deg")
        self.max_tq_xy = _f(y, "l1geo_max_torque_xy")
        self.max_tq_z = _f(y, "l1geo_max_torque_z")
        self.l1_thrust = _f(y, "l1adapt_max_thrust_n")
        self.l1_txy = _f(y, "l1adapt_max_torque_xy")
        self.l1_tz = _f(y, "l1adapt_max_torque_z")
        self.l1_um = _f(y, "l1adapt_max_unmatched_n")
        # arm model for r_os (arm_state_feedforward.cpp)
        self.base_mass = _f(y, "armff_base_mass")
        self.base_com = np.array([_f(y, f"armff_base_com_{a}") for a in "xyz"])
        self.link_m = np.array([_f(y, f"armff_link{i}_mass") for i in range(1, 5)])
        self.link_l = [np.array([_f(y, f"armff_link{i}_l_{a}") for a in "xyz"]) for i in range(1, 5)]
        self.link_c = [np.array([_f(y, f"armff_link{i}_com_{a}") for a in "xyz"]) for i in range(1, 5)]
        self.trim = np.array([_f(y, f"armff_com_trim_{a}") for a in "xyz"])
        self.mismatch = np.array([_f(y, f"armff_mismatch_{a}") for a in "xyz"])
        self.gains_in_yaml = {k: _f(y, YAML_KEYS[k][0]) for k in GAIN_KEYS}


_AXES = [np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]),
         np.array([0.0, 0.0, 1.0])]


def _rot(h, q):
    a = h / np.linalg.norm(h)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(q) * k + (1.0 - math.cos(q)) * (k @ k)


def r_os_flu(cfg, q):
    """ArmStateFeedforward::comOffsetFlu, verbatim."""
    rot = np.eye(3); origin = np.zeros(3); w = np.zeros(3)
    for i in range(4):
        rot = rot @ _rot(_AXES[i], q[i])
        w = w + cfg.link_m[i] * (origin + rot @ cfg.link_c[i])
        origin = origin + rot @ cfg.link_l[i]
    w = w - cfg.link_m.sum() * cfg.base_com
    total = cfg.base_mass + cfg.link_m.sum()
    return R_MODEL @ (w / total) + cfg.trim + cfg.mismatch


def bridge_r0c(model, q):
    """decoupled_reference_bridge.arm_fk's r_0c with the planner's base_com (model frame) --
    the bench sets model["base_com"] per profile (mirror: PLANNER_BASE_COM; robustness: 0),
    exactly what the bridge's --base-com reads off each profile's planner yaml."""
    m = model["m_i"]
    R = np.eye(3); o = np.zeros(3); acc = m[0] * np.asarray(model["base_com"], float)
    for i in range(4):
        R = R @ _rot(model["h_i_im1"][i], q[i])
        acc = acc + m[i + 1] * (o + R @ model["com_i"][i])
        o = o + R @ model["l_i"][i]
    return acc / sum(m)


def build_r0(b3, b1d):
    b3 = b3 / np.linalg.norm(b3)
    s = b1d - b3 * float(b3 @ b1d)
    b1 = s / np.linalg.norm(s)
    return np.column_stack([b1, np.cross(b3, b1), b3])


def _vee(M):
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


class GeoL1Adapter:
    """The decoupled rig's DIRECT tick, presented to circle_bench as a `Law`."""

    def __init__(self, g, cfg, model, dt, profile="mirror", l1_every=4, l1_enable=True):
        self.g = dict(GEO_HW); self.g.update(g)
        self.cfg = cfg; self.model = model; self.dt = dt
        self.kp = np.array([self.g["kp_xy"], self.g["kp_xy"], self.g["kp_z"]])
        self.kv = np.array([self.g["kv_xy"], self.g["kv_xy"], self.g["kv_z"]])
        self.kr = np.array([self.g["kr_xy"], self.g["kr_xy"], self.g["kr_z"]])
        self.kw = np.array([self.g["kw_xy"], self.g["kw_xy"], self.g["kw_z"]])
        self.As = -np.array([self.g["as_v"]] * 3 + [self.g["as_w"]] * 3)
        self.l1_every = max(1, int(l1_every)); self.l1_enable = l1_enable
        self.tick = 0
        # L1 state
        self.seeded = False
        self.zh = np.zeros(6); self.zt = np.zeros(6)
        self.gm = np.zeros(4); self.gum = np.zeros(2); self.ul1 = np.zeros(4)
        # bridge state
        self.last_arm = None; self.v_arm = np.zeros(3); self.a_arm = np.zeros(3)
        # arm servo state
        self.hold_ref = None; self.i_pos = np.zeros(N)
        # allocator the bench applies (profile constants), for the L1's achieved wrench
        if profile == "robustness":
            self.kf_alloc, km_ratio = CS.ROB_KF_ALLOC, CS.ROB_KM_ALLOC_RATIO
        else:
            self.kf_alloc, km_ratio = CS.KF_ALLOC, CS.KM_ALLOC_RATIO
        B = np.zeros((4, 4))
        for i, r in enumerate(ES.ROTOR_POS):
            B[0, i] = 1.0; B[1, i] = r[1]; B[2, i] = -r[0]; B[3, i] = ES.ROT_DIR[i] * km_ratio
        self.B = B; self.Bp = np.linalg.pinv(B)
        self.max_coll = 4.0 * self.kf_alloc * ES.OMEGA_MAX ** 2
        self.trace = None

    # ------------------------------------------------------------------ bridge
    def _bridge(self, ref):
        R0 = build_r0(ref["x_cd_ddot"] + np.array([0.0, 0.0, G]), ref["b1_d"])
        arm_term = R0 @ bridge_r0c(self.model, ref["q_d"])
        x_b = ref["x_cd"] - arm_term
        dt = self.dt
        if self.last_arm is not None:
            v_raw = (arm_term - self.last_arm) / dt
            alpha = min(1.0, dt * 2.0 * math.pi * 8.0)
            a_raw = (v_raw - self.v_arm) / dt
            self.v_arm = self.v_arm + alpha * (v_raw - self.v_arm)
            self.a_arm = self.a_arm + min(1.0, dt * 2.0 * math.pi * 4.0) * (a_raw - self.a_arm)
        self.last_arm = arm_term
        yaw = math.atan2(ref["b1_d"][1], ref["b1_d"][0]) + 0.5 * math.pi
        return x_b, ref["x_cd_dot"] - self.v_arm, ref["x_cd_ddot"] - self.a_arm, yaw

    # --------------------------------------------------------------- u_b
    def _geometric(self, p, v, R, w, ref_p, ref_v, ref_a, yaw, r_os):
        c = self.cfg; m = c.mass; e3 = np.array([0.0, 0.0, 1.0])
        e_p = np.clip(p - ref_p, -c.max_pos_err, c.max_pos_err) if c.max_pos_err > 0 else p - ref_p
        e_v = np.clip(v - ref_v, -c.max_vel_err, c.max_vel_err) if c.max_vel_err > 0 else v - ref_v
        cen = np.cross(w, np.cross(w, r_os))
        f_d = -self.kp * e_p - self.kv * e_v + m * G * e3 + m * ref_a + m * (R @ cen)
        f_d[2] = max(f_d[2], 0.3 * m * G)
        if c.max_tilt_deg > 0:
            max_lat = math.tan(math.radians(c.max_tilt_deg)) * f_d[2]
            lat = np.linalg.norm(f_d[:2])
            if lat > max_lat:
                f_d[:2] *= max_lat / lat
        thrust = max(float(f_d @ (R @ e3)), 0.0)
        b3 = f_d / np.linalg.norm(f_d)
        b2 = np.cross(b3, np.array([math.cos(yaw), math.sin(yaw), 0.0])); b2 /= np.linalg.norm(b2)
        Rd = np.column_stack([np.cross(b2, b3), b2, b3])
        e_r = _vee(0.5 * (Rd.T @ R - R.T @ Rd))
        tau_com = np.cross(r_os, thrust * e3 - m * cen)
        tq = -self.kr * e_r - self.kw * w + np.cross(w, c.I @ w) + tau_com
        if c.max_tq_xy > 0:
            tq[:2] = np.clip(tq[:2], -c.max_tq_xy, c.max_tq_xy)
        if c.max_tq_z > 0:
            tq[2] = np.clip(tq[2], -c.max_tq_z, c.max_tq_z)
        return thrust, tq, Rd, e_r, e_p

    # --------------------------------------------------------------- L1
    def _model(self, w, R, r_os):
        c = self.cfg; e3 = np.array([0.0, 0.0, 1.0])
        cen = np.cross(w, np.cross(w, r_os))
        f = np.concatenate([-G * e3 - R @ cen,
                            c.I_inv @ (np.cross(r_os, c.mass * cen) - np.cross(w, c.I @ w))])
        Gb = np.zeros((6, 6))
        Gb[0:3, 0] = R[:, 2] / c.mass
        Gb[3:6, 0] = -c.I_inv @ np.cross(r_os, e3)
        Gb[3:6, 1:4] = c.I_inv
        Gb[0:3, 4] = R[:, 0] / c.mass
        Gb[0:3, 5] = R[:, 1] / c.mass
        return f, Gb

    def _adapt(self, v, w, R, r_os, dt):
        c = self.cfg
        z = np.concatenate([v, w])
        if not self.seeded or not np.all(np.isfinite(self.zh)):
            self.zh = z.copy(); self.seeded = True
        self.zt = self.zh - z
        ead = np.exp(self.As * dt)
        rhs = self.As * ead / (ead - 1.0) * self.zt
        _, Gb = self._model(w, R, r_os)
        gam = -np.linalg.solve(Gb, rhs)
        if np.all(np.isfinite(gam)):
            gm = gam[:4].copy()
            gm[0] = np.clip(gm[0], -2 * c.l1_thrust, 2 * c.l1_thrust)
            gm[1:3] = np.clip(gm[1:3], -2 * c.l1_txy, 2 * c.l1_txy)
            gm[3] = np.clip(gm[3], -2 * c.l1_tz, 2 * c.l1_tz)
            self.gm = gm
            self.gum = np.clip(gam[4:], -c.l1_um, c.l1_um)
        a = math.exp(-self.g["omega_c"] * dt)
        u = a * self.ul1 - (1.0 - a) * self.gm
        u[0] = np.clip(u[0], -c.l1_thrust, c.l1_thrust)
        u[1:3] = np.clip(u[1:3], -c.l1_txy, c.l1_txy)
        u[3] = np.clip(u[3], -c.l1_tz, c.l1_tz)
        self.ul1 = u
        return u

    def _propagate(self, v, w, R, r_os, u_app, dt):
        if not self.seeded:
            return
        f, Gb = self._model(w, R, r_os)
        inp = np.concatenate([u_app + self.gm, self.gum])
        z = np.concatenate([v, w])
        b = f + Gb @ inp - self.As * z
        ead = np.exp(self.As * dt)
        self.zh = ead * self.zh + (ead - 1.0) / self.As * b
        if not np.all(np.isfinite(self.zh)):
            self.seeded = False

    # --------------------------------------------------------------- allocation (bench's)
    def _achieved(self, thrust, tau_flu):
        tau_m = R_MODEL.T @ tau_flu
        f = np.maximum(self.Bp @ np.array([thrust, *tau_m]), 0.0)
        om = np.clip(np.sqrt(f / self.kf_alloc), 0.0, ES.OMEGA_MAX)
        wr = self.B @ (self.kf_alloc * om ** 2)
        sat = bool(np.any(om >= ES.OMEGA_MAX - 1e-9) or np.any(self.Bp @ np.array([thrust, *tau_m]) < 0.0))
        return np.concatenate([[wr[0]], R_MODEL @ wr[1:4]]), sat

    # --------------------------------------------------------------- the tick
    def __call__(self, Xm, dyn, ref, dt):
        n = N
        Rm = Xm[3:12].reshape(3, 3, order="F")
        R = Rm @ R_MODEL.T
        p = Xm[0:3]
        v = Rm @ Xm[12 + n:15 + n]
        w = R_MODEL @ Xm[15 + n:18 + n]
        q = Xm[12:12 + n]; qd = Xm[18 + n:18 + 2 * n]
        r_os = r_os_flu(self.cfg, q)
        x_b, v_b, a_b, yaw = self._bridge(ref)
        thrust_b, tq_b, Rd, e_r, e_p = self._geometric(p, v, R, w, x_b, v_b, a_b, yaw, r_os)
        adv = self.l1_enable and (self.tick % self.l1_every == 0)
        dt_l1 = self.l1_every * self.dt
        if adv:
            u_l1 = self._adapt(v, w, R, r_os, dt_l1)
        else:
            u_l1 = self.ul1 if self.l1_enable else np.zeros(4)
        thrust = float(np.clip(thrust_b + u_l1[0], 0.0, self.max_coll))
        tq = tq_b + u_l1[1:4]
        u_app, sat = self._achieved(thrust, tq)
        if adv:
            self._propagate(v, w, R, r_os, u_app, dt_l1)
        self.tick += 1
        # ---- 06's position-mode arm servo on the planner's joint reference ----
        if self.hold_ref is None:
            self.hold_ref = q.copy()
        self.hold_ref = self.hold_ref + np.clip(ref["q_d"] - self.hold_ref, -ARM_HOLD_RATE * dt, ARM_HOLD_RATE * dt)
        g_arm = np.asarray(dyn["g"], float)[6:6 + n]
        tau_j = -ARM_HOLD_KP * (q - self.hold_ref) - ARM_HOLD_KD * qd + g_arm
        self.i_pos = np.clip(self.i_pos - ARM_POS_KI * (q - self.hold_ref) * dt, -ARM_POS_I_MAX, ARM_POS_I_MAX)
        tau_j = tau_j + self.i_pos
        if self.trace is not None:
            self.trace.append(np.concatenate([e_p, e_r, u_l1, self.gum, [thrust], tq]))
        return dict(u1=thrust, tau_body=R_MODEL.T @ tq, tau_joint=tau_j, e_R=e_r, n_sat=int(sat),
                    R0c=Rd @ R_MODEL, omega_0c=np.zeros(3))


@contextlib.contextmanager
def _patched(g, profile, l1_every, l1_enable, cfg, hook=None):
    orig = CB.ES.Law

    def make(model, wcfg, case):
        a = GeoL1Adapter(g, cfg, model, case.dt, profile=profile, l1_every=l1_every, l1_enable=l1_enable)
        if hook is not None:
            hook(a)
        return a
    CB.ES.Law = make
    try:
        yield
    finally:
        CB.ES.Law = orig


_CFG = {}


def config(yaml_path=MIRROR_YAML):
    if yaml_path not in _CFG:
        _CFG[yaml_path] = GeoConfig(yaml_path)
    return _CFG[yaml_path]


STREAM = "r050_L24_f55_a15_p6"     # the comparison circle (r 0.5 m, 24 s lap, q2 25 +- 15 @ 6 s, fold 55)

# Isaac's feedback on this rig: raw mocap emulator at 60 Hz (sim_feedback_*), no EKF lag.
FB_ISAAC = dict(CB.FB, odo_lag_s=0.010, vel_noise=0.022, vel_bw_hz=30.0, pos_noise=0.0005)


def run(g=None, stream=STREAM, l1_every=4, l1_enable=True, yaml_path=MIRROR_YAML, hook=None, **kw):
    """One bench flight of the geometric+L1 law (g = gain overrides of GEO_HW).
    kw -> circle_bench.simulate (profile, delay_ms, fb, seed, t_settle, gust, rtf, keep, full,
    arm_delay_ms ...). The arm-side friction feed-forward is off (position-mode arm)."""
    kw.setdefault("ff_scale", np.zeros(N))
    prof = kw.get("profile", "mirror")
    with _patched(g or {}, prof, l1_every, l1_enable, config(yaml_path), hook=hook):
        return CB.simulate(dict(CB.BASE), CB.stream(stream), **kw)


if __name__ == "__main__":
    import multiprocessing as mp
    import time
    cfg = config()
    print("hardware gains in the mirror yaml:", cfg.gains_in_yaml)
    assert all(abs(cfg.gains_in_yaml[k] - GEO_HW[k]) < 1e-12 for k in GAIN_KEYS), "GEO_HW != yaml"
    jobs = [("hardware set, hardware-like feedback", dict(), dict()),
            ("hardware set, Isaac feedback (60 Hz mocap)", dict(), dict(fb=FB_ISAAC)),
            ("hardware set, ideal feedback", dict(), dict(fb=CB.FB_IDEAL)),
            ("hardware set, L1 OFF (paper's A/B)", dict(), dict(l1_enable=False))]

    def job(j):
        t0 = time.time(); nm, g, kw = j
        kw = dict(kw); l1e = kw.pop("l1_enable", True)
        r = run(g, l1_enable=l1e, **kw)
        return nm, r, time.time() - t0
    with mp.get_context("fork").Pool(len(jobs)) as pool:
        for nm, r, dtw in pool.imap(job, jobs):
            print(CB.fmt(nm, r) + f"  [{dtw:.0f} s]", flush=True)

"""
utils_planner/arm_sweep.py — prescribed ARM-SWEEP references for the
`…_whole` free-flight trajectories (hover_whole / circle_whole / figure8_whole).

The `…_whole` modes move the WHOLE body: the base flies its shape while the
arm runs a smooth prescribed joint sweep on top. This module owns that sweep —
the joint profile and the FK that turns it into an EE reference:

  build_sweep(cfg, T)          catalogue knobs -> a sweep spec (radians)
  sweep_joint_profile(t, sp)   q(t), qdot, qddot  (4-vectors, rest-to-rest)
  sweep_start_pose(cfg)        q(0) alone — the demos' pre-Isaac Q_SPAWN
  ee_com_offset(q, params)     u(q) = r_0e^0 - r_0c^0, the EE offset from the
                               SYSTEM CoM in the base frame — the same FK the
                               offline planner uses (exact link CoMs, not the
                               MATLAB l_i/2 approximation)
  ee_offset_derivs(...)        u, udot, uddot along the sweep (FD Jacobians)

WHY prescribing both x_cd and a swept r_ed is dynamically SOUND at hover: the
system CoM responds ONLY to the total EXTERNAL force (thrust + gravity) — arm
motion is INTERNAL and cannot move the CoM at all. Holding x_cd fixed while
the arm sweeps is therefore exactly compatible: the BASE counter-translates
under the sweeping arm (visibly "gimballing" about the held CoM) and the
attitude channel absorbs the arm's reaction torques (the GMO's job). For the
moving-base sweeps (circle/figure8_whole) the EE offset is mapped through a
LEVEL-attitude yaw frame; the ~1 deg thrust tilt of the flight-validated
circle bends that mapping by <2 mm at this arm's ~0.24 m reach — absorbed by
the EE impedance loop, exactly like the validated fixed-arm carry.

JOINT PROFILE (q4 = 0 throughout — the same wrist rule as the poly_whole
showcase: with the wrist at zero, b1_de is INDEPENDENT of the fold, so the EE
heading reference stays exact while beta moves):

    beta(t) = beta0 + beta_amp * sin(2*pi*n_cycles*s(t/T) + phase)   fold
    q1(t)   =         q1_amp   * sin(2*pi*n_cycles*s(t/T))           arm yaw
    q2 = split*beta,  q3 = (1-split)*beta

s is the rest-to-rest min-snap (septic) phase shared with the showcase's
_scal_sincycle, so q, qdot AND qddot start/end at rest for ANY phase offset
(s' = s'' = 0 at both ends) and q(0) is the exported takeoff-hold pose. With
phase = 90 deg the fold runs in QUADRATURE with the arm yaw, so the EE traces
a closed (yaw x height) loop under the hover — the same "combo" recipe as the
poly_whole pinned phase, with the roles of base and EE swapped.

Pure numpy — no Isaac, no ROS2. Importable and testable from system python.
"""

import numpy as np

from .compatible_trajectory import _arm_kin, _minsnap3

# Joint limits of the AM_realign asset [deg] — every sweep is scanned against
# these at build time (the same numbers TRAJ_CONFIG["poly_whole"]["q_lim_deg"]
# hands the offline planner). The singular branches are at NEGATIVE q2/q3 on
# this asset, so the positive-fold sweeps here live on the well-conditioned
# side by construction.
Q_LIM_DEG = [[-35.0, 35.0], [-90.0, 50.0], [-90.0, 50.0], [-180.0, 180.0]]

# FD steps: 1e-6 rad for the central FIRST differences (truncation ~1e-12 m),
# 1e-4 rad for the directional SECOND difference — second differences lose
# half the mantissa, a 1e-6 step there would be roundoff-dominated.
_EPS_J = 1e-6
_EPS_H = 1e-4


def build_sweep(cfg, T):
    """Catalogue knobs -> sweep spec (radians, seconds)."""
    rad = np.deg2rad
    return {"T": float(T),
            "beta0": float(rad(float(cfg["beta0_deg"]))),
            "beta_amp": float(rad(float(cfg["beta_amp_deg"]))),
            "q1_amp": float(rad(float(cfg["q1_amp_deg"]))),
            "n": int(cfg["n_cycles"]),
            "phase": float(rad(float(cfg["sweep_phase_deg"]))),
            "split": float(cfg.get("split", 0.5))}


def sweep_joint_profile(t, sp):
    """(q, qdot, qddot) at time t — 4-vectors, rest-to-rest by construction.
    Past T the min-snap phase clamps at 1, so the profile holds q(T) = q(0)
    at rest (the same terminal-hold convention as every other trajectory)."""
    s, ds, d2s = _minsnap3(t / sp["T"])
    s1, s2 = ds / sp["T"], d2s / sp["T"] ** 2
    w = 2.0 * np.pi * sp["n"]
    thb = w * s + sp["phase"]
    be = sp["beta0"] + sp["beta_amp"] * np.sin(thb)
    be1 = sp["beta_amp"] * w * np.cos(thb) * s1
    be2 = sp["beta_amp"] * (-w ** 2 * np.sin(thb) * s1 ** 2
                            + w * np.cos(thb) * s2)
    thy = w * s
    y = sp["q1_amp"] * np.sin(thy)
    y1 = sp["q1_amp"] * w * np.cos(thy) * s1
    y2 = sp["q1_amp"] * (-w ** 2 * np.sin(thy) * s1 ** 2
                         + w * np.cos(thy) * s2)
    a, b = sp["split"], 1.0 - sp["split"]
    q = np.array([y, a * be, b * be, 0.0])
    qd = np.array([y1, a * be1, b * be1, 0.0])
    qdd = np.array([y2, a * be2, b * be2, 0.0])
    return q, qd, qdd


def sweep_start_pose(cfg):
    """q(0) WITHOUT anchoring — the pose the takeoff hold pre-positions the
    arm at (and the demos spawn at). T is irrelevant at t = 0."""
    return sweep_joint_profile(0.0, build_sweep(cfg, 1.0))[0]


def ee_com_offset(q, params):
    """u(q) = r_0e^0 - r_0c^0 — EE position relative to the SYSTEM CoM, in the
    base frame. `params` = controller.make_params()."""
    r0c, r0e = _arm_kin(q, params)
    return r0e - r0c


def ee_offset_derivs(q, qd, qdd, params):
    """u(q), udot, uddot along the sweep — analytic chain over FD kinematics:

        udot  = J*qdot                      J = du/dq, central difference
        uddot = J*qddot + qdot'*H*qdot      directional second difference

    FD on the EXACT FK, not a model approximation: first-difference error
    ~1e-11 m, second ~1e-8 m — both far below the millimetre tracking floor.
    Cost is 11 _arm_kin calls (~0.1 ms), fine at the 250 Hz control rate."""
    u = ee_com_offset(q, params)
    J = np.zeros((3, q.size))
    for j in range(q.size):
        dq = np.zeros(q.size)
        dq[j] = _EPS_J
        J[:, j] = (ee_com_offset(q + dq, params)
                   - ee_com_offset(q - dq, params)) / (2.0 * _EPS_J)
    u1 = J @ qd
    nqd = float(np.linalg.norm(qd))
    if nqd > 1e-12:
        eps = _EPS_H / nqd                   # step so that |eps*qdot| = 1e-4
        curv = (ee_com_offset(q + eps * qd, params) - 2.0 * u
                + ee_com_offset(q - eps * qd, params)) / eps ** 2
    else:
        curv = np.zeros(3)
    u2 = curv + J @ qdd
    return u, u1, u2


def check_sweep_limits(sp, name, n_check=201):
    """Joint-limit scan of the whole sweep — mirrors the offline planner's
    q_lim_deg warning. A violating sweep still evaluates (the WARNING is the
    contract, same as the planner): the demo's startup log is where this is
    meant to be read."""
    lim = np.asarray(Q_LIM_DEG, float)
    qs = np.array([sweep_joint_profile(tq, sp)[0]
                   for tq in np.linspace(0.0, sp["T"], n_check)])
    qmin = np.degrees(qs.min(axis=0))
    qmax = np.degrees(qs.max(axis=0))
    if (qmin < lim[:, 0]).any() or (qmax > lim[:, 1]).any():
        print(f"WARNING: '{name}' arm sweep exceeds joint limits "
              f"{lim.tolist()} deg — range min {np.round(qmin, 1)} "
              f"max {np.round(qmax, 1)}. Reduce beta0/beta_amp/q1_amp.")
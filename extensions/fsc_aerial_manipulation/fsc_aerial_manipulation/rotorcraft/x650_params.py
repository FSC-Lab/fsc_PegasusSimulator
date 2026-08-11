#!/usr/bin/env python
"""
| File: x650_params.py
| License: BSD-3-Clause
| Description: Pure-Python (no Isaac Sim / pegasus.simulator imports) X650
|   physical/calibration constants, factored out of x650_bare_frame_utils.py
|   so they can be imported by code that does NOT have an Isaac Sim
|   environment available (e.g. a standalone deterministic Python validation
|   model) -- x650_bare_frame_utils.py imports `pegasus.simulator....` and
|   `omni....` at module scope, which fails outside Isaac Sim's own Python,
|   so its constants were not importable standalone before this split.
|   x650_bare_frame_utils.py now imports its calibration constants from here
|   instead of defining them inline; values are unchanged (verify with
|   `git diff` if in doubt) -- this is a pure refactor, not a recalibration.

All values here are load-bearing, sourced, and cross-referenced in CLAUDE.md
under "X650 airframe reused standalone" -- do not hand-edit without updating
both this file's docstring and CLAUDE.md.
"""
import numpy as np

# ── MN4014 + 15x5" prop bench-test calibration ──────────────────────────────
# Source: docs/propeller_testing/MN_4014_15x5_report.pdf, Step 1 polynomial fits
# (throttle x in 0-100%): rpm(x) = 70.2593x + 781.49, thrust(x) [N] =
# 0.0020455x^2 + 0.0960918x - 0.361797, torque(x) [N.m] = -2.94144e-07x^3 +
# 7.56245e-05x^2 + 0.00046387x + 0.0104334, projected onto the sim's
# force=k*omega^2/torque=c*omega^2 form via an unweighted zero-intercept
# least-squares fit (see x650_bare_frame_utils.py's original derivation
# comment / CLAUDE.md for the fit-quality numbers: thrust RMSE 0.31N, torque
# RMSE 0.0136 N.m).
ZERO_POSITION_ARMED = 81.8374   # rad/s, == rpm(throttle=0%) in rad/s (armed-idle only)
MIN_ROTOR_VEL = 0.0             # rad/s -- unconditional floor; must allow true zero (disarmed)
MAX_ROTOR_VEL = 817.5911        # rad/s, == rpm(throttle=100%) in rad/s
ROTOR_CONSTANT = 4.536223e-05               # N / (rad/s)^2
ROLLING_MOMENT_COEFFICIENT = 8.366000e-07   # N.m / (rad/s)^2

# Step 3 lambda (rotor spin-up bandwidth, 1/s) -- bench-measured value, mean of
# the report's lag_all/lag_filtered variants (10.519/10.501). See
# lagged_thrust_curve.py's docstring for why this pair (not direct_all/
# direct_filtered) was chosen, and CLAUDE.md's "X650 PX4 gain tuning" section
# for why *PX4* needed this raised to 15.51 in practice -- that was a
# workaround for PX4's untuned generic rate/attitude gains' phase margin, not
# a correction to the physical value. The bench value (10.51) is used here
# unmodified; a from-scratch geometric controller gets its own gain tuning
# instead of inheriting PX4's workaround.
ROTOR_LAMBDA = 10.51   # 1/s

# ── MN4010 + 15x5" prop bench-test calibration (added 2026-08-05) ────────────
# Source: docs/propeller_testing/MN_4010_15x5_report.pdf, Step 1 polynomial fits
# (throttle x in 0-100%):
#     rpm(x)    = 63.5974 x + 611.731                       (deg 1, R^2 0.99699)
#     thrust(x) = 0.00178203 x^2 + 0.0636277 x - 0.293853    (deg 2, R^2 0.99866) [N]
#     torque(x) = -3.446e-07 x^3 + 7.7154e-05 x^2
#                 - 0.000375849 x + 0.0125214               (deg 3, R^2 0.99915) [N.m]
#
# Projected onto the sim's fixed force = k*omega^2 / torque = c*omega^2 form by
# the SAME unweighted zero-intercept least-squares fit, sampled uniformly across
# throttle 0-100%, that produced the MN4014 constants above. That procedure was
# re-run against the MN4014 polynomials first and reproduced the committed
# MN4014 k/c to 0.000%, so the two motor sets are directly comparable and not
# fitted by different methods.
#
# Fit quality: thrust RMSE 0.18 N (range -0.29..23.89 N), torque RMSE 0.0119 N.m
# (range 0.012..0.402 N.m) -- slightly better than MN4014's 0.31 N / 0.0136 N.m,
# because this motor's curve is less steep.
MN4010_ZERO_POSITION_ARMED = 64.0603     # rad/s, == rpm(throttle=0%)   =  611.731 rpm
MN4010_MIN_ROTOR_VEL = 0.0               # rad/s -- unconditional floor; must allow true zero
MN4010_MAX_ROTOR_VEL = 730.0507          # rad/s, == rpm(throttle=100%) = 6971.471 rpm
MN4010_ROTOR_CONSTANT = 4.540431e-05              # N / (rad/s)^2
MN4010_ROLLING_MOMENT_COEFFICIENT = 8.247173e-07  # N.m / (rad/s)^2

# Step 3 lambda, chosen by the same rule as MN4014's: the mean of the report's
# lag_all/lag_filtered variants (10.025/10.028), the system-ID estimates derived
# from measured step-response timing (lambda = 1/tau). NOT direct_all/
# direct_filtered, which here are 3.454/3.238 with 18-19% unphysical negative
# values -- see lagged_thrust_curve.py's header for the full rationale.
#
# THIS MOTOR IS SLOWER THAN THE MN4014: tau = 1/lambda = 99.7 ms against the
# MN4014's 95.1 ms, i.e. ~4.8% more rotor lag. Both sit inside the report's own
# physically-expected 70-200 ms band. The difference is modest, but it moves in
# the direction that costs phase margin, so the softened X650 PX4 gains
# (MC_*RATE_K=0.3, MC_ROLL_P/MC_PITCH_P=3.25, MC_YAW_P=1.4 -- see CLAUDE.md's
# "X650 PX4 gain tuning") are if anything more necessary here than for MN4014.
# Do not raise lambda to buy stability: that was tried on the MN4014 (15.51) and
# is a workaround for untuned gains, not a physical value.
MN4010_ROTOR_LAMBDA = 10.0265   # 1/s

# ── Body mass/inertia (2026-07-13/14 CAD correction, see CLAUDE.md's "X650 CAD
# inertia correction") ───────────────────────────────────────────────────────
# Total vehicle mass (body 3.416kg authored + 4 rotors' own ~0.021kg each).
MASS = 3.5   # kg
# Diagonal body inertia (Ixx, Iyy, Izz), kg*m^2, in x650_new.usd's body axes
# labeling -- exact CAD (OnShape) "Mass and section properties" values, NOT
# the earlier (wrong) mass-ratio-scaled ones. CAD's own Lxx/Lyy/Lzz axis
# labels are swapped Y<->Z relative to the USD asset's; the values below are
# already in the USD/body convention (Ixx unaffected, Iyy/Izz swapped from
# CAD's Lzz/Lyy) -- see CLAUDE.md for the cross-check that confirmed this.
INERTIA_DIAG = np.array([0.05777498, 0.06408172, 0.065004565])   # kg*m^2

# ── Rotor geometry measured from x650_new.usd's joint anchors ────────────────
# Verified against x650_new.usd's rotor prim positions on 2026-07-25. Uses a
# right-handed body frame (X-forward, Y-left,
# Z-up); torque_i = r_i x F_i with F_i=[0,0,f_i] is frame-handedness
# independent, so this doesn't affect the physics, only which axis label
# means "roll" vs "pitch" in this standalone model.
ARM_LENGTH = 0.3251370742254931  # m; hypot(0.22990663, 0.22990663)
ROT_DIR = np.array([-1, -1, 1, 1])
# x650_new.usd/PX4 Quad-X order in the X-forward, Y-left body frame:
# front-right, rear-left, front-left, rear-right.
_rotor_angles_deg = np.array([-45.0, 135.0, 45.0, -135.0])
ROTOR_POSITIONS = np.array([
    [ARM_LENGTH * np.cos(np.radians(a)), ARM_LENGTH * np.sin(np.radians(a)), 0.0]
    for a in _rotor_angles_deg
])   # (4,3), body-frame xyz relative to CoM

NUM_ROTORS = 4

# ── T650: the airframe these MN4010 motors are actually fitted to ───────────
# Tarot 650 build, total mass 2.95 kg (user-stated 2026-08-05) -- NOT the X650's
# 3.5 kg. It reuses x650_new.usd's geometry, rotor ordering and arm length,
# because no separate T650 asset exists; only mass and the motor set differ.
T650_MASS = 2.95   # kg, total
# Body mass = total - the asset's four authored rotor masses (0.0839 kg), the
# same bookkeeping x650_bare_frame_utils.py does for the X650.
T650_BODY_MASS = 2.866079044342041   # kg
# Inertia is left at the X650's exact CAD-derived diagonal. Rationale is the
# same one recorded for the X650's own 1.467 -> 3.416 kg body-mass increase
# (CLAUDE.md, "X650 CAD inertia correction"): the mass difference is assumed to
# be centrally concentrated (batteries near the CG) and therefore not to move
# the rotational inertia much. That assumption was accepted going UP by 1.95 kg;
# applying it going DOWN by 0.55 kg is the consistent choice, but it IS an
# assumption -- if the T650 ever gets its own CAD numbers, put them here.
T650_INERTIA_DIAG = INERTIA_DIAG

# Derived hover point, for reference when writing a matching controller config
# (T_hover = 2.95*9.81/4 = 7.2349 N per rotor):
#     omega_hover = sqrt(T_hover / k)                             = 399.18 rad/s
#     hover_cmd   = (omega_hover - omega_idle)/(omega_max - omega_idle) = 0.50319
# Tangent-line linearization at that point, matching the derivation in
# fsc_autopilot_ros2's config headers:
#     vehicle_thrust_scaling = 1/(2*k*omega_hover*(omega_max-omega_idle)) = 0.041423
#     vehicle_idle_thrust    = hover_cmd - thrust_scaling*T_hover         = 0.203500
# Static thrust-to-weight is 3.35 (24.20 N per rotor at 100% throttle), so climb
# authority is ample -- better than the X650/MN4014 combination's 3.53 only
# because the T650 is lighter, not because this motor is stronger.
T650_HOVER_COMMAND = 0.503188
T650_THRUST_SCALING = 0.041423
T650_IDLE_THRUST = 0.203500

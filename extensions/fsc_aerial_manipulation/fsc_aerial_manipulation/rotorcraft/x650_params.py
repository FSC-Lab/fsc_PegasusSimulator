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

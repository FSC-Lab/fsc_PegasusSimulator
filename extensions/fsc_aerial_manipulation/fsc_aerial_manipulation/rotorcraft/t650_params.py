#!/usr/bin/env python
"""
| File: t650_params.py
| License: BSD-3-Clause
| Description: Pure-Python (no Isaac Sim / pegasus.simulator imports) T650 physical and
|   calibration constants -- the Tarot 650 build: MN4010 + 15x5" propellers on the same
|   x650_new.usd airframe geometry.
|
|   Parallel to x650_params.py rather than derived from it: the two vehicles are
|   independent airframes that happen to share a USD asset, and keeping their constants in
|   separate modules means a change to one can never silently move the other. The values
|   that genuinely are shared (rotor geometry, body inertia) are COPIED here with a note
|   saying so, not imported.
|
|   No Isaac Sim dependency, for the same reason x650_params.py has none: these constants
|   must be importable by standalone validation code that has no Isaac environment.

All values here are load-bearing and sourced. Do not hand-edit without updating both this
docstring and CLAUDE.md.
"""
import numpy as np

# ── MN4010 + 15x5" prop bench-test calibration ──────────────────────────────
# Source: docs/propeller_testing/MN_4010_15x5_report.pdf, Step 1 polynomial fits
# (throttle x in 0-100%):
#     rpm(x)    = 63.5974 x + 611.731                       (deg 1, R^2 0.99699)
#     thrust(x) = 0.00178203 x^2 + 0.0636277 x - 0.293853    (deg 2, R^2 0.99866) [N]
#     torque(x) = -3.446e-07 x^3 + 7.7154e-05 x^2
#                 - 0.000375849 x + 0.0125214               (deg 3, R^2 0.99915) [N.m]
#
# rpm(x) is linear in x, and PX4MavlinkBackend's controls(0-1)->omega[rad/s] map
# (omega = (controls + input_offset)*input_scaling + zero_position_armed) is also linear,
# so the two are matched EXACTLY (not merely at the endpoints) by evaluating rpm at
# throttle 0% and 100%.
#
# thrust/torque are quadratic/cubic in throttle, but the simulator's thrust curve supports
# only the pure k*omega^2 / c*omega^2 form through the origin, so k and c come from an
# unweighted zero-intercept least-squares fit sampled uniformly across throttle 0-100%.
#
# PROCEDURE VALIDATED BEFORE USE, per CLAUDE.md's rule for adding a motor: re-running this
# exact fit against the MN4014 polynomials reproduces that motor's already-committed and
# flight-validated constants to +0.004% (k) and +0.015% (c), and reproduces the MN4010
# values below to +0.003% / +0.018%. The two motor sets are therefore directly comparable
# and not fitted by different methods.
#
# Fit quality: thrust RMSE 0.18 N (range -0.29..23.89 N), torque RMSE 0.0119 N.m
# (range 0.012..0.402 N.m) -- slightly better than the MN4014's 0.31 N / 0.0136 N.m,
# because this motor's curve is less steep.
ZERO_POSITION_ARMED = 64.0603    # rad/s, == rpm(throttle=0%)   =  611.731 rpm
MIN_ROTOR_VEL = 0.0              # rad/s -- unconditional floor; MUST allow true zero
MAX_ROTOR_VEL = 730.0507         # rad/s, == rpm(throttle=100%) = 6971.471 rpm

# The bench values, kept named so the tuning below is transparent and reversible.
BENCH_ROTOR_CONSTANT = 4.540431e-05              # N / (rad/s)^2
BENCH_ROLLING_MOMENT_COEFFICIENT = 8.247173e-07  # N.m / (rad/s)^2

# ── Empirical tuning against flight C (2026-08-06) ──────────────────────────
# Fitted to docs/experimental_data_ros2_bag/debug_recording_20260806_164742 replayed
# through the same fsc_autopilot_ros2 stack in simulation. Full method and before/after
# metrics: docs/sim_to_real_t650/REPORT_flightC.md and TUNING_t650.md.
#
# THRUST_FIT_FACTOR -- hover-command match at FIXED mass.
#   The real vehicle's hover command early in flight C (t=5-15 s, before battery sag
#   accumulates) is 0.5025 +- 0.0009. With MASS pinned at 3.033921 kg, the bench k puts the
#   simulated hover at 0.5117, i.e. +1.8% too high. Since mass is not available as a
#   degree of freedom, the thrust constant absorbs the difference:
#       k = MASS*g/4 / omega_hover^2   with omega_hover set by the measured hover command.
#   Note this is a mass/thrust trade, not evidence the bench thrust curve is wrong: at the
#   previous 2.95 kg TOTAL the bench k reproduced the measured hover to +0.14%.
THRUST_FIT_FACTOR = 1.030724     # -> hover command 0.5025, matching hardware
ROTOR_CONSTANT = BENCH_ROTOR_CONSTANT * THRUST_FIT_FACTOR          # 4.679933e-05

# YAW_TORQUE_FIT_FACTOR -- effective yaw-torque coefficient, not the bench drag coefficient.
#   The simulated yaw axis is markedly under-powered: on flight C's two yaw steps the
#   simulation rises in 0.840 s against the real 0.555 s and overshoots 44.9% against 9.2%,
#   and PX4's own autotune could not identify the axis at all (it returned 5.6x less
#   response per unit excitation than roll/pitch and timed out at 20 s, while roll and pitch
#   converged in ~5 s each).
#   The cause is a missing TERM, not a wrong number: the thrust curve applies only the
#   steady aerodynamic drag torque c*omega^2 and never the reaction to the rotors' own
#   angular acceleration, I_rotor*omega_dot, which during a yaw command adds across all four
#   rotors and is 2-4x larger than the modelled torque. Modelling is out of scope here, so c
#   is inflated to stand in for it. That makes c an EFFECTIVE yaw-torque coefficient valid
#   near the tested step sizes -- it is deliberately NOT the bench drag coefficient, and it
#   will over-predict steady-state yaw drag torque.
#   Only the ratio c/Izz sets yaw dynamics, so this could equally have been done by lowering
#   Izz; c was chosen because Izz has independent CAD support and matching the response by
#   inertia alone would need Izz ~ 0.029 kg.m^2, less than half any physical estimate.
#
#   Factor chosen by measurement, not assumption -- flight C replayed at 2.5 and 3.0:
#       factor   yaw overshoot      yaw rise 10-90      yaw shape RMSE
#       (real)      9.16 %             0.555 s               --
#       1.0        44.88 %             0.840 s            4.109 deg
#       2.5        13.51 %             0.628 s            1.292 deg
#       3.0         7.80 %             0.588 s            1.268 deg   <-- shipped
#   3.0 is kept because it centres the overshoot error (-1.35 pts, against +4.35 at 2.5).
#   The two are otherwise equivalent within run-to-run noise, so do not read the 2%
#   difference in RMSE as meaningful. Settling is the one metric that degrades with more
#   gain (2.07 s against the real 2.98 s), and n=2 steps is a weak sample -- treat the yaw
#   fit as good but not finely resolved.
YAW_TORQUE_FIT_FACTOR = 3.0
ROLLING_MOMENT_COEFFICIENT = BENCH_ROLLING_MOMENT_COEFFICIENT * YAW_TORQUE_FIT_FACTOR

# ZERO_POSITION_ARMED is the idle-at-0%-throttle speed *once armed*; it matches real ESC
# behaviour and is only ever applied by PX4MavlinkBackend.update_input_reference(), which
# handle_control() calls only when PX4 reports armed. MIN_ROTOR_VEL, by contrast, is the
# thrust curve's own floor and is NOT armed-gated, so it must stay 0.0 -- setting it to the
# idle value makes the props spin while disarmed (confirmed live on the X650, 2026-07-13).

# Step 3 lambda (rotor spin-up bandwidth, 1/s): mean of the report's lag_all / lag_filtered
# variants (10.025 / 10.028), the system-ID estimates derived from measured step-response
# timing (lambda = 1/tau). NOT direct_all / direct_filtered, which are 3.454 / 3.238 here
# with 18-19% unphysical negative values and NRMSE > 1.2 -- see lagged_thrust_curve.py's
# header for the full rationale.
#
# THIS MOTOR IS SLOWER THAN THE MN4014: tau = 1/lambda = 99.7 ms against 95.1 ms, i.e. ~4.8%
# more rotor lag. Both sit inside the report's physically-expected 70-200 ms band. The
# difference is modest but costs phase margin, so the softened X650 PX4 gains
# (MC_*RATE_K=0.3, MC_ROLL_P/MC_PITCH_P=3.25, MC_YAW_P=1.4) are if anything more necessary
# here. Do NOT raise lambda to buy stability: that was tried on the MN4014 (15.51) and is a
# workaround for untuned gains, not a physical value.
ROTOR_LAMBDA = 10.0265   # 1/s

# ── Mass model ──────────────────────────────────────────────────────────────
# BODY_MASS is the mass written onto x650_new.usd's body mesh at spawn time. The asset also
# authors four rotor rigid bodies of ~0.020980 kg each, which PhysX adds on top, so the
# vehicle the solver sees is BODY_MASS + ROTOR_MASS_TOTAL.
#
# !! CHANGED 2026-08-06, ON INSTRUCTION: BODY mass is now 2.95 kg. !!
# Previously 2.95 kg was the vehicle's TOTAL mass (body 2.866079 + rotors 0.083921), which
# is where the flight-validated hover command of 0.503188 came from. Treating 2.95 kg as the
# BODY mass instead raises the total to 3.033921 kg and moves the hover command to 0.511645
# (+0.85 percentage points). That is a real change to the plant, not bookkeeping:
# 0.5032 was confirmed four separate times -- twice in simulation (QGC/PX4 mixer takeoff
# 0.503341, fsc_autopilot_ros2 baseline hover 0.50320) and twice against hardware (flight B
# averages 0.5033 over its first 5 s, flight A 0.5018, before battery sag accumulates).
# If the intent was 2.95 kg TOTAL, set BODY_MASS = 2.95 - ROTOR_MASS_TOTAL and everything
# below follows automatically.
ROTOR_MASS_TOTAL = 0.083920955657959   # kg, the four rotor bodies authored in x650_new.usd
BODY_MASS = 2.95 - ROTOR_MASS_TOTAL                      # kg, written onto /body/body at spawn
MASS = BODY_MASS + ROTOR_MASS_TOTAL    # kg, total vehicle mass the physics solver sees

# ── Body inertia ────────────────────────────────────────────────────────────
# FULL inertia tensor about the body CoM (kg*m^2), in x650_new.usd's body-axis labelling,
# INCLUDING the Ixy product of inertia. Derived by regression against flight data
# (2026-08-13) -- record the bag/script here, per this module's docstring rule.
#
# This is the shipped value as written: it is NOT scaled by THRUST_FIT_FACTOR. Earlier
# versions scaled Ixx/Iyy by that factor to hold k/I (and therefore the flight-C roll/pitch
# fit) invariant against the thrust refit; a tensor identified from flight data already
# reflects the real plant, so that bookkeeping no longer applies.
#
# Ixy is load-bearing and must not be dropped: USD/PhysX stores inertia as principal
# moments plus a principal-axes rotation, so anything writing this into a stage goes
# through fsc_aerial_manipulation.utils.author_inertia_tensor(), which converts exactly.
INERTIA_TENSOR = np.array([[0.077,   0.0089,     0.0],
                           [0.0089,  0.06408172, 0.0],
                           [0.0,     0.0,        0.065004565]])   # kg*m^2

# Diagonal view (Ixx, Iyy, Izz) for the offline tools in docs/sim_to_real_t650, which model
# a diagonal plant. It DISCARDS Ixy -- never use it to author a stage.
INERTIA_DIAG = np.diag(INERTIA_TENSOR).copy()

# ── Rotor geometry ──────────────────────────────────────────────────────────
# Shared with the X650 because the airframe asset is literally the same file; measured from
# x650_new.usd's joint anchors. Right-handed body frame (X-forward, Y-left, Z-up);
# torque_i = r_i x F_i is frame-handedness independent, so this only fixes which axis label
# means "roll" vs "pitch" in standalone models, not the physics.
ARM_LENGTH = 0.3251370742254931  # m; hypot(0.22990663, 0.22990663)
ROT_DIR = np.array([-1, -1, 1, 1])
# x650_new.usd / PX4 Quad-X order: front-right, rear-left, front-left, rear-right.
_rotor_angles_deg = np.array([-45.0, 135.0, 45.0, -135.0])
ROTOR_POSITIONS = np.array([
    [ARM_LENGTH * np.cos(np.radians(a)), ARM_LENGTH * np.sin(np.radians(a)), 0.0]
    for a in _rotor_angles_deg
])   # (4,3), body-frame xyz relative to CoM
NUM_ROTORS = 4

# ── Derived hover point ─────────────────────────────────────────────────────
# Recomputed from the constants above rather than hard-coded, so a change to BODY_MASS or to
# the motor calibration cannot leave these stale:
#     T_hover     = MASS*g/4                                         = 7.4407 N
#     omega_hover = sqrt(T_hover / ROTOR_CONSTANT)                   = 404.811 rad/s
#     HOVER_COMMAND = (omega_hover - omega_idle)/(omega_max - omega_idle)
# THRUST_SCALING / IDLE_THRUST are the tangent-line linearisation at that point, matching the
# derivation in fsc_autopilot_ros2's config headers.
_G = 9.81
_T_HOVER = MASS * _G / NUM_ROTORS
_OMEGA_HOVER = float(np.sqrt(_T_HOVER / ROTOR_CONSTANT))
_OMEGA_SPAN = MAX_ROTOR_VEL - ZERO_POSITION_ARMED

HOVER_COMMAND = (_OMEGA_HOVER - ZERO_POSITION_ARMED) / _OMEGA_SPAN     # 0.511653
THRUST_SCALING = 1.0 / (2.0 * ROTOR_CONSTANT * _OMEGA_HOVER * _OMEGA_SPAN)  # 0.040846
IDLE_THRUST = HOVER_COMMAND - THRUST_SCALING * _T_HOVER                # 0.207733

# Static thrust-to-weight, for reference: 4*k*omega_max^2 / (MASS*g) = 3.25.
THRUST_TO_WEIGHT = (NUM_ROTORS * ROTOR_CONSTANT * MAX_ROTOR_VEL ** 2) / (MASS * _G)

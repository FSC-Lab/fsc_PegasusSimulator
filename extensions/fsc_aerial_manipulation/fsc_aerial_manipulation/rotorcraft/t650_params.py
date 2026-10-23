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
# THRUST_FIT_FACTOR -- RE-ANCHORED 2026-08-24 ON THE 0820 STEPPED-PAYLOAD FLIGHTS.
#
# !! PREVIOUS VALUE, KEPT FOR REVERSION !!
#     THRUST_FIT_FACTOR_PREV = 1.030724  -> ROTOR_CONSTANT 4.679931e-05
#   Derived values it produced (recompute automatically if you restore it):
#     bare T650 3.033921 kg : hover 0.502525  scaling 0.040232  idle 0.203169  T/W 3.352
#     AM-T650   3.746170 kg : hover 0.569101  scaling 0.036206  idle 0.236457  T/W 2.715
#     bare+769g 3.802921 kg : hover 0.574122  scaling 0.035935  idle 0.238967  T/W 2.674
#   That factor was a HOVER-COMMAND MATCH to flight C (2026-08-06), and flight C was flown
#   on a nearly fresh pack. It therefore encoded a ~25.3 V battery as if it were a property
#   of the propeller, and made the simulated vehicle 15.8% stronger than the real one at the
#   operating point the aerial manipulator actually flies. That surplus is exactly the
#   standing thrust compensation the L1/UDE augmentation was always seen carrying in
#   simulation, and it is why a sim-tuned allocator constant under-delivered on hardware.
#
# THE SHIPPED VALUE IS MEASURED, NOT FITTED TO A HOVER COMMAND. Back-solved from
#   docs/experimental_data_ros2_bag/"0820 - T650 baseline - stepped payload hover"/
#   hover_px4_ulog/log_252_UnknownDate.ulg  -- the +719 g step, chosen because 719 g is the
#   ARM-EQUIVALENT payload (AM total 3.746 kg vs that flight's 3.753 kg) and it hovers at
#   the same collective the AM does (measured 0.6159).
#       kf_eff = m*(g - a_z) / (cos(theta) * sum_i omega_i^2),  omega = 64.0603 + 665.9904*u
#   over 112 steady samples: kf_eff = 4.041283e-05 N/(rad/s)^2, sample sd 2.4%.
#
# ALL SEVEN 0820 HOVERS, for context (bare 3.034 kg -> +719 g, arm dismounted):
#     log  payload      u       V       kf_eff       /bench
#     246      0 g   0.5225  24.34 V  4.331429e-05   0.9540
#     247     69 g   0.5381  24.07 V  4.212756e-05   0.9278
#     248    119 g   0.5496  23.86 V  4.128784e-05   0.9093
#     249    219 g   0.5683  23.59 V  4.010399e-05   0.8833
#     250    419 g   0.5649  24.42 V  4.323084e-05   0.9521   <- pack swap vs 249
#     251    519 g   0.5822  24.15 V  4.207497e-05   0.9267
#     252    719 g   0.6159  23.85 V  4.041283e-05   0.8901   <- SHIPPED
#   Every flight is BELOW the bench constant, and the spread is 8.0% -- but it is not
#   payload-driven: logs 249->250 hover at the same collective (0.5683 vs 0.5649) yet differ
#   7.8% in kf because a battery pack was swapped between them. Regressing all seven gives
#       ln kf = -16.767 + 2.149*ln V - 0.264*u,   R^2 = 0.9972
#   i.e. kf_eff ~ V^2 with the collective term worth under 2% across the flown range. THE
#   PROPELLER CONSTANT IS NOT WRONG: BENCH_ROTOR_CONSTANT was fitted by pairing thrust(x)
#   and rpm(x) at the same throttle, so it is a thrust-vs-omega relation and is immune to
#   supply voltage. What sags with the battery is the throttle->omega map
#   (ZERO_POSITION_ARMED / MAX_ROTOR_VEL).
#
# SO THIS FACTOR IS A STAND-IN, EXACTLY LIKE YAW_TORQUE_FIT_FACTOR BELOW. The physically
# faithful refactor is to keep ROTOR_CONSTANT = BENCH_ROTOR_CONSTANT and scale BOTH ends of
# the omega map by sqrt(0.890066) = 0.943433, i.e. model a 23.50 V pack against the bench's
# 24.91 V. It is deliberately NOT done here, for one reason: the DIRECT allocator carries
# its own copy of that same map (alloc_omega_idle / alloc_omega_max in every
# fsc_autopilot_ros2 T650 yaml). Scaling the plant's map without scaling the controller's
# would inject a second, undeclared mismatch on top of the deliberate alloc_thrust_coeff
# one, and a robustness campaign needs exactly one knob. Folding the voltage deficit into k
# keeps plant and allocator on ONE omega map. The cost, stated so it is not discovered
# later: torque is c*omega^2 and is NOT reduced by this factor, so the simulated
# yaw-torque-to-thrust ratio is ~12% higher than a voltage-faithful model would give.
# YAW_TORQUE_FIT_FACTOR is left alone on purpose -- its 3.0 was fitted against flight C's
# yaw SHAPE (c/Izz), which this change does not touch.
#
# NOTE the assumed mass is load-bearing: kf_eff is linear in it. At log 252 the bare
# airframe is taken as MASS = 3.033921 kg. If the bare airframe is really 3.109 kg (what the
# bare-T650 L1 HARDWARE yaml infers from its own hover balance, 75 g more), kf_eff rises
# 2.0% to 4.122046e-05. WEIGH THE AIRFRAME to close that out.
THRUST_FIT_FACTOR = 0.890066     # -> ROTOR_CONSTANT 4.041283e-05, log_252 measured
ROTOR_CONSTANT = BENCH_ROTOR_CONSTANT * THRUST_FIT_FACTOR          # 4.041283e-05

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
# The body-mass interpretation is intentional.  Do not subtract the authored
# rotor masses here: every T650 controller/config downstream uses the resulting
# 3.033921 kg bare-airframe total (and 3.746170 kg with the manipulator).
ROTOR_MASS_TOTAL = 0.083920955657959   # kg, the four rotor bodies authored in x650_new.usd
BODY_MASS = 2.95                                         # kg, written onto /body/body at spawn
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
#     omega_hover = sqrt(T_hover / ROTOR_CONSTANT)                   = 429.088 rad/s
#     HOVER_COMMAND = (omega_hover - omega_idle)/(omega_max - omega_idle)
# THRUST_SCALING / IDLE_THRUST are the tangent-line linearisation at that point, matching the
# derivation in fsc_autopilot_ros2's config headers.
_G = 9.81
_T_HOVER = MASS * _G / NUM_ROTORS
_OMEGA_HOVER = float(np.sqrt(_T_HOVER / ROTOR_CONSTANT))
_OMEGA_SPAN = MAX_ROTOR_VEL - ZERO_POSITION_ARMED

HOVER_COMMAND = (_OMEGA_HOVER - ZERO_POSITION_ARMED) / _OMEGA_SPAN     # 0.548098
THRUST_SCALING = 1.0 / (2.0 * ROTOR_CONSTANT * _OMEGA_HOVER * _OMEGA_SPAN)  # 0.043295
IDLE_THRUST = HOVER_COMMAND - THRUST_SCALING * _T_HOVER                # 0.225955

# Static thrust-to-weight, for reference: 4*k*omega_max^2 / (MASS*g) = 2.895.
THRUST_TO_WEIGHT = (NUM_ROTORS * ROTOR_CONSTANT * MAX_ROTOR_VEL ** 2) / (MASS * _G)

#!/usr/bin/env python3
"""CASCADE: 6-state longitudinal NMPC (outer, force+moment) + geometric stabilisation (inner).

WHY THIS NODE EXISTS
    The 19-state `nmpc_full_smc_node` puts position, attitude, tilt and body rates in ONE solve.
    That is the right model and the wrong budget. Measured in flight it takes 27 ms median and
    58-93 ms p95 against a 100 ms tick, and it is COMPUTE-SATURATED: four independent attempts to
    stiffen it (w_aoa 0.5->2.0; an AoA inequality constraint; w_u 0.02->0.5; roll damping 12->40)
    EACH improved the physics and EACH saturated IPOPT at MAX_ITER=25, and raising MAX_ITER to 40
    put p95 at 93 ms with the loop slipping. There was no tuning left to buy.

    Splitting the problem buys the budget back. Measured on this machine:

        19-state, in flight   27 ms median   58-93 p95      MAX_ITER 25, saturated
         6-state, bench      9-12 ms         12-14 p95      whole corridor, hover to 90 deg

    The 6-state longitudinal model is the one that CRACKED the corridor (nmpc6_cruise_node): it
    predicts and counters the PITCH MOMENT, which is what every point-mass controller before it
    could not do. What it lacked was lateral robustness -- and lateral is precisely the axis that
    does NOT need prediction. It needs bandwidth. So:

        OUTER (NMPC6, background thread, ~20 Hz)   the longitudinal wrench + the trim package:
                                                   Tf, Tr (collective + pitch trim), beta, de,
                                                   and the reference pitch theta.
        INNER (geometric SE(3), every mocap ~120 Hz)  attitude stabilisation and allocation of the
                                                   residual moments to rotors + surfaces.

    The outer loop predicts; the inner loop stabilises. Neither does the other's job.

WHAT IS DIFFERENT FROM nmpc6's `lateral_control`, AND WHY IT MATTERS
    nmpc6's inner loop faded differential thrust out with `lowspd = clip((6-V)/6, 0, 1)` -- gone by
    6 m/s -- while the aero surfaces fade in on dynamic pressure. Measured roll authority through
    the corridor with that fade:

        V     0     4     6     8    16   m/s
        auth 38.6  15.8   8.7  15.5  62.0 rad/s^2

    There is a NOTCH at 6 m/s, and it is self-inflicted: at that speed the airframe still physically
    has 8.50 N.m of differential-thrust roll (cos 33.8 deg = 0.83 of hover authority) and the fade
    throws all of it away. Retaining it gives 40.8 rad/s^2 instead of 8.7 -- 4.7x -- at exactly the
    speed the aircraft has been departing. The 6 m/s crossover is hand-tuned; the airframe's
    measured roll hand-over is 8.9 m/s.

    The fade existed because at tilt beta a FRONT differential makes roll*cos(beta) AND yaw*sin(beta)
    -- coupled -- and `lateral_control` treated roll and yaw as independent scalar channels, applying
    one `droll*cos(beta)` to BOTH pairs. That is what loses authority at high tilt, not the airframe.
    Solve the coupling instead and the axes DECOUPLE naturally at 90 deg, because the REAR pair is
    still vertical (pure roll) while the front pair is horizontal (pure yaw). Conditioning of the
    4-rotor allocation, measured:

        tilt      0    45    85    90   deg
        cond(B)  8.3   4.0  13.3  33.8
        roll    0.568 0.434 0.321 0.304  N.m per N of differential   <- 54% of hover at 90 deg,
                                                                        not zero

    So the inner loop here allocates through B_rot(beta)^-1 with NO speed gate, and splits each
    moment between rotors and surfaces by their ACTUAL authority (rotor headroom vs qbar), which is
    smooth in both V and beta and has no magic crossover speed to tune.

    The one real limit is thrust headroom: the rear pair is unloaded in cruise, so it cannot give
    much differential before a rotor hits zero. That is the known fix already documented for nmpc6
    ("rotor FLOOR -- rotors never fully unload so they RETAIN differential-thrust authority") and is
    `rear_floor` here.

INVARIANTS INHERITED FROM THE CAMPAIGN (do not remove; each cost a flight)
    * auto_arm defaults FALSE. The operator controls arming and offboard. (User directive.)
    * A frozen actuator stream is the worst failure mode -- the loop is exception-wrapped and falls
      back to hover rather than letting PX4 hold the last command forever (bag vt1).
    * The failed-solve fallback must be SAFE, not the cruise trim: applying the trim while upset
      commands full forward tilt and deepens the upset (bag vt2). Diverged -> safe_hover().
    * A trim is a package -- the outer's (Tf, Tr, beta, de) ship together; the inner adds only
      DIFFERENTIALS that preserve net vertical force.
"""
# ★ Run-as-a-script bootstrap. The imports below are absolute
# (`from px4_vtol_direct_actuator...`), which is correct for an installed
# package but needs the package's PARENT on sys.path. That is true under
# `ros2 run` and `python3 -m`, and false under `python3 this_file.py` -- which
# is how every launcher and every offline gate in this project invokes it.
# Adding the parent here keeps both modes working without a global path shim.
if __package__ in (None, ""):
    import os as _os, sys as _sys
    _sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from px4_vtol_direct_actuator import nmpc6_cruise_node as n6                    # noqa: E402  NMPC6, Schedule, build_ode6
from px4_vtol_direct_actuator import nmpc_so3_transition_node as so3            # noqa: E402  geometry, aero, trim_split

MAX_THRUST = so3.MAX_THRUST
RPOS, SPIN, CQ = so3.RPOS, so3.SPIN, so3.CQ
DA_MAX = so3.SO3TransitionNMPC.DA_MAX
DR_MAX = so3.SO3TransitionNMPC.DR_MAX
DE_MAX = n6.DE_MAX
_cl = n6._cl

# How far the FRONT pair may lean AFT [rad]. With beta >= 0 the ONLY decelerating
# force is nose-up pitch, and that converts speed into HEIGHT (434 J at 16 m/s is
# EXACTLY 12.8 m of climb -- it cannot be tuned away). Leaning the front pair aft puts
# the braking force along the flight path instead. Validated at 20 deg (bag full_tr32:
# altitude excursion 18.9 m -> 2.2 m against that 12.8 m floor).
BETA_AFT_MAX = math.radians(25.0)

# Smallest fraction of its NOMINAL thrust any rotor may be scaled back to. A rotor
# commanded to zero stops, and the spin-up lag then leaves the airframe with a dead
# corner for ~0.12 s -- long enough to roll it over (bag vtol_20260911_133910: one
# tick of FR=0.00 against FL=0.71 put the aircraft at -21 deg of roll).
ROTOR_FLOOR_FRAC = 0.15


# ============================ propeller: thrust <-> throttle ============================
# The controllers reason in NEWTONS; the plant takes a THROTTLE. With a flat thrust model those
# were the same thing up to a constant (T = throttle * MAX_THRUST). With the sim's
# thrust_model="momentum" they are not, in TWO ways:
#   * thrust is QUADRATIC in throttle (via omega), not linear -- so even in HOVER, T/MAX_THRUST
#     asks for roughly twice the throttle actually needed;
#   * thrust falls with the airspeed along the rotor's OWN axis, reaching ZERO at the advance
#     ratio J0 -- at 16 m/s there is no positive thrust below ~74% throttle.
# Publishing T/MAX_THRUST against that plant makes the controller silently UNDER-DELIVER: it
# plans a trim it cannot buy and settles wherever the error balances (offline it parked at
# 13.5 m/s and 67 deg tilt instead of reaching 90, and that was insensitive to the prop choice).
#
# `trim_split` and the ODEs are NOT wrong -- they solve force balance in newtons, which inflow
# does not change. What has to change is THIS MAPPING and the solver's thrust BOUNDS.
PROP_D = 0.3048          # m, matches RotorConfig.prop_diameter_m for this airframe
PROP_J0 = 0.6            # advance ratio at which thrust reaches zero
OMEGA_MAX = 749.0        # rad/s at throttle 1
_N_MAX = OMEGA_MAX / (2.0 * math.pi)
_CT0 = MAX_THRUST / (so3.RHO * _N_MAX ** 2 * PROP_D ** 4)
INFLOW = True            # False -> the legacy flat mapping, for A/B


def thrust_available(v_axial):
    """Most thrust ONE rotor can make at full throttle, given the airspeed along its own axis."""
    if not INFLOW:
        return MAX_THRUST
    x = _N_MAX * PROP_D
    return max(0.0, so3.RHO * PROP_D ** 2 * (_CT0 * x * x - (_CT0 / PROP_J0) * x * float(v_axial)))


def thrust_to_throttle(T, v_axial):
    """Throttle delivering thrust `T` at axial airspeed `v_axial`. Exact inverse of the sim's
    momentum model: with x = n*D, T = rho*D^2*( CT0*x^2 - (CT0/J0)*x*V ), so x solves a quadratic
    and only the positive root is physical."""
    T = max(0.0, float(T))
    if not INFLOW:
        return _cl(T / MAX_THRUST, 0.0, 1.0)
    if T <= 0.0:
        return 0.0
    k = so3.RHO * PROP_D ** 2 * _CT0
    b = float(v_axial) / PROP_J0
    x = 0.5 * (b + math.sqrt(b * b + 4.0 * T / k))
    return _cl(x / (PROP_D * _N_MAX), 0.0, 1.0)


# ============================== the inner loop ==============================
def B_rot(beta):
    """dT(4) -> [dFz, Mx, My, Mz] in the FLU body frame, at tilt `beta`.

    Each rotor contributes F = T*d and M = r x (T*d) - spin*CQ*T*d (the reaction torque acts about
    the rotor's OWN axis, so it tilts with the front pair -- that is the yaw->roll rotation that
    made the scalar approximation fail). Front pair (0,1) tilts; rear pair (2,3) stays vertical.
    Row 0 is the vertical force so the caller can demand dFz = 0 and get a pure moment differential
    that leaves the outer loop's trim intact.
    """
    B = np.zeros((4, 4))
    for i in range(4):
        d = (np.array([math.sin(beta), 0.0, math.cos(beta)]) if i < 2
             else np.array([0.0, 0.0, 1.0]))
        B[0, i] = d[2]
        B[1:4, i] = np.cross(RPOS[i], d) - SPIN[i] * CQ * d
    return B




def predict6(F6, x0, ubuf, step):
    """Roll the longitudinal state forward by the loop latency using the commands ALREADY IN FLIGHT.

    ★ THE OUTER LOOP IS USELESS WITHOUT THIS. Measured on the outer's OWN model -- perfect model
    match, no lateral axis, no inner loop, nothing else to blame:

        latency 0.00 s    0/1201 solves failed   smooth 0->90 deg, altitude flat, cruise held
        latency 0.05 s   87/1201 failed          oscillating
        latency 0.10 s  154/706  failed          ground at t=35.3 s

    Delay compensation is one of the five documented keys for the 6-state node and omitting it here
    made the cascade look like an architecture problem when it was a missing prediction. The project
    rule from earlier campaigns: compensation >= true latency; UNDER-compensating by 30 ms is fatal,
    over-compensating is merely a safe partial-tilt stall.
    """
    if not ubuf:
        return np.asarray(x0, float)
    Xp = np.asarray(x0, float).copy()
    for uu in ubuf:
        for _ in range(3):
            Xp = Xp + (step / 3.0) * np.array(F6(Xp, uu)).ravel()
    return Xp


def planned_theta(nmpc, fallback, stage=1):
    """The pitch the outer NMPC actually PLANS at `stage` of its horizon, read out of the solution.

    The inner loop must track THIS, not the trim schedule's pitch. The solver optimises a pitch
    trajectory jointly with tilt and thrust; handing the inner loop a different pitch target makes
    the two loops disagree, and the tilt command bang-bangs (measured -- see inner_step).
    ★ MEASURED DEAD END, KEPT AS A WARNING. Stage 0 is pinned to the measured state by the
    X[:,0] == x0 constraint, and stage 1 is only DT = 0.06 s beyond it, so this returns very nearly
    the pitch the aircraft ALREADY HAS. Feeding it to the inner loop gives e_pitch ~ 0 and therefore
    NO restoring force -- a reference that re-anchors to the achieved state, which is the pathology
    that has cost this project four separate flights. It crashed at t=7.0 s. The inner loop tracks
    the outer's REFERENCE pitch instead. Verified correct as an extraction (stage 0 reproduces x0
    exactly): the layout is right, the IDEA is wrong.

    Layout: z = [X(6 x (N+1)) ; U(4 x N)] flattened column-major, theta is state index 4.
    """
    try:
        z = nmpc.zsol
        if z is None:
            return fallback
        return float(z[nmpc.nx * stage + 4])
    except (AttributeError, IndexError, TypeError):
        return fallback


def tilt_clamp(beta, Tf, Tr, de, theta, V, sched, margin_rad):
    """★ THE AIRSPEED TILT CLAMP. Applied tilt <= trim_tilt(V_now) + margin, self-relaxing to
    90 deg as speed builds.

    Over-tilting at low airspeed is the documented killer: at high tilt the front rotors are
    horizontal (almost no pitch moment) AND the elevator has no dynamic pressure, so pitch
    authority goes to ZERO and the nose simply falls (bag full2: tilt 82-90 deg at V~1 m/s ->
    pitch -2 to -80 deg in 6 s -> inverted). The SO(3) node needed this clamp (bag so3i) and it
    has now been omitted from a new node three times -- it is an invariant, not a tuning knob.

    Clamping the tilt ALONE would break the reference: a trim is a package, and shipping a tilt
    without the thrusts that balance it is how bags full_tr7 and full_tr25/27 were lost. So the
    split is RE-SOLVED at the clamped tilt.
    """
    b_lim = float(sched.at(max(V, 0.0))["beta"]) + margin_rad
    if beta <= b_lim:
        return beta, Tf, Tr
    beta = min(b_lim, math.pi / 2.0)
    try:
        Tf, Tr = so3.trim_split(beta, theta, max(V, 1.0), de, so3.M * so3.G, 0.0)
    except (np.linalg.LinAlgError, ValueError, ZeroDivisionError):
        pass                      # keep the solver's thrusts rather than shipping nothing
    return beta, float(Tf), float(Tr)


class RotorAlloc:
    """B_rot(beta)^-1, cached on a 1-degree tilt grid (the inverse is ~30 us, the loop runs at
    mocap rate, and beta moves slowly -- there is no reason to pay for it every sample)."""

    def __init__(self):
        self._g = {}

    def inv(self, beta):
        # NEGATIVE tilt is real: the front pair leans AFT by up to BETA_AFT_MAX in the
        # back-transition, and clamping it to 0 here would allocate for a tilt the aircraft is
        # not at.
        k = int(round(math.degrees(_cl(beta, -BETA_AFT_MAX, math.pi / 2))))
        m = self._g.get(k)
        if m is None:
            m = np.linalg.inv(B_rot(math.radians(k)))
            self._g[k] = m
        return m


def surface_authority(V):
    """Moment each surface can deliver at full deflection [N.m]: M = qbar*S*b*C_delta*delta_max.
    Returns (roll, pitch, yaw), all >= 0 -- magnitudes, the sign lives in the coefficient."""
    qbarS = 0.5 * so3.RHO * V * V * so3.S_WING
    return (abs(qbarS * so3.B_WING * so3.CLDA * DA_MAX),
            abs(qbarS * so3.C_WING * so3.CMDE * DE_MAX),
            abs(qbarS * so3.B_WING * so3.CNDR * DR_MAX))


def inner_step(R, w, theta_ref, heading, beta, V, Tf, Tr, de_trim, g, alloc, vb=None,
               ei=None, dt=0.008):
    """Geometric stabilisation of ROLL and YAW ONLY, plus authority-weighted allocation.

    ★ THE INNER LOOP TRACKS THE OUTER'S OWN PLANNED PITCH. `theta_ref` is read out of the NMPC's
    SOLUTION (stage 1 of the horizon), not from the trim schedule. Three variants were measured:

      * inner tracks the SCHEDULE pitch -> the two loops disagree, because the outer is optimising
        a pitch trajectory of its own. The inner perturbs pitch at 200 Hz, the outer sees a pitch
        it never planned and re-plans a completely different tilt: the tilt command bang-banged
        10->40->11->0->39->43->15->2->46->45->0 deg at ~1 Hz and it departed at t=19 s.
      * inner does NO pitch at all -> hover itself is unstable. At 20 Hz with 100 ms of latency the
        pitch dynamics are far too fast for the outer alone: ground at t=4.6 s, 63/93 solves failed.
      * inner does pitch RATE DAMPING only -> no steady-state pitch feedback, so the aircraft parked
        nose-up at +12 deg and CLIMBED at ~1 m/s without accelerating, with every solve reporting
        ok=1. The reference then pinned itself (v_cmd clamped to V + v_lead) and the corridor was
        never entered.

    Tracking the outer's PLAN removes the disagreement at its source: the outer decides the pitch
    trajectory as part of the same optimisation that decides tilt and thrust (a trim is a package),
    and the inner is simply the fast servo that realises it.

    Roll and yaw are not in the outer's longitudinal model at all, so the inner owns them outright.
    Their attitude errors are extracted DIRECTLY (roll from R[2,1]/R[2,2], yaw from R[1,0]/R[0,0])
    rather than through the SO(3) error, which mixes a pitch discrepancy into both axes.

    Returns (mot[4], aileron, elevator, rudder). `Tf`/`Tr` are the outer's PAIR totals and are
    preserved exactly in both net vertical force and net pitch moment.
    """
    # ---- SE(3) attitude error against Rz(heading) . Ry(theta_ref), roll = 0 ----
    # theta is NOSE-UP positive (2026-09-06 convention): theta = asin(R[2,0]).
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.asin(_cl(R[2, 0], -1.0, 1.0))
    yaw = math.atan2(R[1, 0], R[0, 0])
    # ★ BANK-TO-TURN ONCE THE WING IS FLYING. Holding HEADING with the rudder is a multicopter
    # reflex and it does not survive wing-borne flight: a banked aeroplane TURNS, and that turn is
    # not a disturbance to be cancelled. Measured -- inner loop alone at the 90 deg cruise trim, a
    # 5 deg roll upset drove yaw to -163 deg with the RUDDER SATURATED at -1.000, while roll itself
    # recovered fine. Disabling the rotor differential changed nothing, so it was the yaw law.
    #
    # So heading error commands ROLL, and the rudder is left to damp. The blend is physical rather
    # than a tuned crossover speed: `wing` is the fraction of the weight the wing could carry at the
    # CL peak, (V / V_stall)^2 against the measured 14.13 m/s stall -- 0 in hover (pure multicopter
    # yaw, the rotors have the authority) and 1 once wing-borne (pure bank-to-turn). Banking only
    # steers when the wing is making the lift you are banking, which is exactly what this ratio is.
    #
    # Sign, from the geometry: roll = atan2(R[2,1], R[2,2]) is positive for a rotation about body +x
    # (forward), which lifts the LEFT wing = banks RIGHT. Body +z then has a -y (rightward, in ENU)
    # component, so the aircraft accelerates right and yaw DECREASES. A positive yaw error therefore
    # calls for POSITIVE roll.
    #
    # ★ THE BANK COMMAND NEEDS A RATE TERM. Heading responds to roll through a DOUBLE integration
    # (roll -> turn rate -> heading), and a P-only command on a double integrator is marginally
    # stable at best. Written P-only it was: a disturbance recovered cleanly, held for a minute or
    # two, and then wound up into a slow lateral oscillation -- with ttr=20, yaw ran -9.9 -> +8.3
    # -> -18.5 deg while roll grew to +19 against the 25 deg bank limit, onset around t=120 s. The
    # onset came SOONER the faster the transition (t~60 s at ttr=8) because a quicker ramp leaves
    # more residual for the loop to amplify, which is what made it look like a `ttr` problem.
    # Damping the yaw RATE closes it: opposing the turn as it develops stops the overshoot that
    # feeds the next half-cycle. Undisturbed flight was always clean, so this never showed up in a
    # no-disturbance soak -- a long run with NO upset does not exercise this mode at all.
    wing = _cl((V / 14.13) ** 2, 0.0, 1.0)
    e_yaw_pre = math.atan2(math.sin(yaw - heading), math.cos(yaw - heading))
    # ★ HEADING -> BANK. Currently the simple proportional form. It is KNOWN to be marginally
    # stable -- heading responds to roll through a double integration, and in flight (bag
    # vtol_20260911_102706) a 13 deg yaw drift at 90 deg tilt demanded a bank near the 25 deg
    # limit and the roll loop rode straight through it: +7 -> +31 -> +62 -> +118 deg in one
    # second. It is kept because it is the BASELINE THAT SURVIVES: it limit-cycles at ~24 deg
    # roll under noise but does not crash, and it recovers every injected upset.
    #
    # ★★ A TURN-RATE REDESIGN WAS TRIED TWICE AND IS WORSE BOTH TIMES. DO NOT REPEAT IT AS-IS.
    #   attempt 1  psi_dot_cmd = psi_dot_des - k_d*(psi_dot - psi_dot_des). Mixes two sign
    #              conventions (psi_dot_des is "how much right bank do we want", psi_dot is the
    #              true heading rate) so the damping became POSITIVE feedback. Crashed the noise
    #              and upset cases outright.
    #   attempt 2  signs corrected and verified empirically (a +17 deg/s yaw rate produced +0.058
    #              aileron, opposing it, as it should). STILL WORSE -- it then crashed the CLEAN
    #              case, which the proportional law flies perfectly.
    # The likely reason, and the thing to fix before a third attempt: the true heading rate
    # psi_dot = (q*sin(phi) + r*cos(phi))/cos(theta) couples PITCH RATE into the signal at any
    # nonzero bank. At 25 deg of bank a 40 deg/s pitch rate -- ordinary during a transition --
    # looks like 17 deg/s of heading rate on its own, saturating a 12 deg/s turn-rate command
    # from pitch motion that has nothing to do with heading. Damping that signal feeds pitch
    # activity straight into the roll command. A usable version needs the pitch term removed or
    # heavily filtered, and should be validated on the CLEAN case first, not the upset cases.
    wing = _cl((V / 14.13) ** 2, 0.0, 1.0)
    e_yaw_pre = math.atan2(math.sin(yaw - heading), math.cos(yaw - heading))
    roll_ref = _cl(wing * g["k_bank"] * e_yaw_pre, -g["bank_max"], g["bank_max"])
    e_roll = roll - roll_ref
    e_pitch = pitch - theta_ref                          # theta_ref = the OUTER's planned pitch
    # direct yaw hold fades out as the wing takes over; the rudder keeps damping throughout
    e_yaw = (1.0 - wing) * e_yaw_pre
    eW = np.asarray(w, float)                            # trim is steady -> desired body rate is 0
    # ★ PITCH SIGN. Verified against the plant, not reasoned about: a front-heavy split gives
    # omega_dot_y = -12.9 and drives theta = asin(R[2,0]) to +58 deg, i.e. NOSE-UP corresponds to
    # NEGATIVE My and negative omega_y, while theta is nose-up POSITIVE. So correcting a nose-up
    # error (e_pitch > 0) needs a POSITIVE (nose-down) My -- the attitude term is +kR, not -kR.
    # The RATE term keeps its minus sign: pitching nose-up is omega_y < 0, and -kW*eW[1] is then
    # positive = nose-down = opposing. Getting this backwards is positive feedback on pitch, and it
    # diverged in hover before the transition even started (0 -> -13 -> +17 -> +57 deg in 2.2 s).
    # This inversion has now been found SIX times in this project; check it empirically every time.
    # ★ ATTITUDE INTEGRAL, band-limited. A pure PD leaves a standing attitude error against any
    # steady disturbance (CG offset, a trim the outer cannot express, an asymmetric airframe).
    # The safeguards are the ones this project already paid for on the safe controller:
    #   * BAND -- integrate only when the error is already small. A naive integral on this laggy
    #     plant winds up during fast tracking and diverges (it once flipped the aircraft inverted).
    #   * CLAMP to i_max, so a long saturation cannot store an unrecoverable amount.
    #   * FREEZE while the allocation is saturated (applied below, after the mixer knows).
    # Pitch is DELIBERATELY EXCLUDED: the outer NMPC owns pitch attitude and its trim already
    # integrates the longitudinal state; a second integrator there fights it for the same axis.
    e_att = np.array([e_roll, 0.0, e_yaw])
    if ei is None:
        ei = np.zeros(3)
    ei = np.asarray(ei, float).copy()
    ei_in = ei.copy()
    band = g.get("i_band", math.radians(12.0))
    imax = g.get("i_max", 1.5)
    for k in (0, 2):
        if abs(e_att[k]) < band:
            ei[k] += e_att[k] * dt
            ei[k] = _cl(ei[k], -imax, imax)
        else:
            ei[k] = 0.0                  # outside the band the error is a transient, not a bias
    # ★ SCHEDULE THE PITCH GAINS WITH DYNAMIC PRESSURE.
    # The airframe's pitch stiffness is qbar*S*c*Cm_alpha, so its SHORT PERIOD rises with speed:
    # 0.29 Hz at 4 m/s, 1.17 Hz at 16, 1.32 at 18 (measured against the true plant). A FIXED
    # kR_pitch puts the control loop at sqrt(kR/Iyy) = 0.50 Hz no matter what, so it is faster
    # than the mode below ~8 m/s and 2.3x SLOWER at 16. A loop slower than the mode it is meant
    # to damp cannot damp it, and the loop's own phase lag then drives it -- which is the heavy
    # pitch oscillation at high speed (bag vtol_20260911_124102: +-25 deg at ~1 Hz, elevator
    # slamming rail to rail). The mode is only zeta = 0.29, so it is easy to excite.
    # Holding the loop a fixed RATIO above the mode means kR ~ stiffness ~ qbar -- no arbitrary
    # crossover speed, and it falls back to the floor in hover where stiffness is zero.
    stiff = qbarS_p = 0.5 * so3.RHO * V * V * so3.S_WING * so3.C_WING * abs(so3.CMA)
    kR_p = max(g["kR_pitch"], (g.get("pitch_ratio", 2.0) ** 2) * stiff)
    kW_p = 2.0 * g.get("pitch_zeta", 0.7) * math.sqrt(max(kR_p, 1e-9) * so3.IYY)
    kW_p = max(g["kW_pitch"], kW_p)
    M_des = np.array([-g["kR_roll"] * e_roll - g["kW_roll"] * eW[0] - g.get("kI_roll", 0.0) * ei[0],
                      +kR_p * e_pitch - kW_p * eW[1],
                      -g["kR_yaw"] * e_yaw - g["kW_yaw"] * eW[2] - g.get("kI_yaw", 0.0) * ei[2]])
    M_des = np.clip(M_des, [-6.0, -6.0, -3.0], [6.0, 6.0, 3.0])

    # ---- split each axis between surfaces and rotors BY ACTUAL AUTHORITY ----
    # No crossover speed. A_surf ~ qbar (zero at hover), A_rot ~ available thrust headroom scaled
    # by how well the geometry converts it at this tilt. Each effector does work in proportion to
    # what it can actually deliver, so the split is smooth in V and in beta.
    A_surf = np.array(surface_authority(V))
    Binv = alloc.inv(beta)
    # ★ HEADROOM IS SPEED-DEPENDENT NOW. A rotor's ceiling is thrust_available(v_axial), not
    # MAX_THRUST: at 16 m/s the front pair can make 4.79 N each, not 18. Sizing the allocation
    # against 18 would promise the inner loop differential authority that does not exist.
    vb_ = np.zeros(3) if vb is None else np.asarray(vb, float)
    v_ax_f = float(vb_[0] * math.sin(beta) + vb_[2] * math.cos(beta))   # front pair tilts
    v_ax_r = float(vb_[2])                                              # rear pair stays vertical
    cap_f = thrust_available(v_ax_f)
    cap_r = thrust_available(v_ax_r)
    head_f = max(0.0, cap_f - Tf / 2.0)                     # per-rotor headroom, front pair
    head_r = max(0.0, min(Tr / 2.0, cap_r - Tr / 2.0))      # rear: limited by how little it holds
    head = max(0.05, min(head_f, head_r))
    A_rot = np.array([head / (np.linalg.norm(Binv @ np.array([0.0, 1.0, 0.0, 0.0])) + 1e-9),
                      head / (np.linalg.norm(Binv @ np.array([0.0, 0.0, 1.0, 0.0])) + 1e-9),
                      head / (np.linalg.norm(Binv @ np.array([0.0, 0.0, 0.0, 1.0])) + 1e-9)])
    # ★ CHARGE THE ROTORS FOR THE FORWARD FORCE THEY LEAK.
    # The differential is solved against dFz = 0, but at high tilt that row only constrains the
    # REAR pair -- the front pair is horizontal, so anything done to it is FORWARD FORCE. And the
    # front pair's pitch arm at 90 deg is only |z| = 0.033 m, so 1 N.m of pitch takes a ~15.6 N
    # change on EACH front rotor: 31 N of thrust, against a cruise drag of 3.7 N. Measured:
    #     tilt   0 deg   1 N.m pitch ->   0.00 N forward
    #     tilt  45 deg                 ->  -1.66 N
    #     tilt  90 deg                 -> -31.25 N
    # Authority alone already sends ~99% of pitch to the elevator, but the surviving ~1% is still
    # a couple of newtons on a 3.7 N drag -- and it is ALWAYS THERE, because the pitch rate term
    # never stops working. That is the speed running away from its commanded value.
    # So cap each axis' ROTOR authority by the forward force it would cost, and let the
    # authority-weighted split do the rest. At hover the leak is zero and nothing changes; at
    # 90 deg the pitch channel is handed almost entirely to the elevator, which has the authority
    # for it anyway (10.5 N.m at 16 m/s).
    sb = math.sin(beta)
    for i, unit in enumerate((np.array([0.0, 1.0, 0.0, 0.0]),
                              np.array([0.0, 0.0, 1.0, 0.0]),
                              np.array([0.0, 0.0, 0.0, 1.0]))):
        d = Binv @ unit
        leak = abs((d[0] + d[1]) * sb)              # N of forward force per N.m demanded
        if leak > 1e-6:
            A_rot[i] = min(A_rot[i], g.get("fx_budget", 0.10) / leak)
    wsurf = A_surf / (A_surf + A_rot + 1e-9)
    M_surf = wsurf * M_des
    M_rot = (1.0 - wsurf) * M_des

    # ---- rotors: pure moment differential, zero net vertical force ----
    dT = Binv @ np.array([0.0, M_rot[0], M_rot[1], M_rot[2]])
    nom = np.array([Tf / 2.0, Tf / 2.0, Tr / 2.0, Tr / 2.0])
    T = nom + dT
    # Scale the differential back rather than clipping it: clipping one rotor changes the moment
    # AND the net force, which is how a saturating mixer turns a roll command into a vertical
    # transient. Attitude-priority desaturation, same rule as the safe controller uses.
    caps = np.array([cap_f, cap_f, cap_r, cap_r])
    # ★ PER-ROTOR desaturation. The previous version compared nom.min() against min(nom + dT),
    # which are NOT NECESSARILY THE SAME ROTOR -- mixing the two indices makes the scale factor
    # too large, leaves a NEGATIVE thrust on one rotor, and thrust_to_throttle() clips that to
    # ZERO. Flown, that put a single-sample FR=0.00 against FL=0.71 at the instant the tilt
    # reached 90 deg (bag vtol_20260911_133910, t=22.3): one tick of full asymmetry, a +0.70
    # diagonal impulse, and the aircraft rolled to -21 deg and never recovered.
    # Scale the differential by the tightest per-rotor limit instead, so the moment is preserved
    # in DIRECTION and only reduced in magnitude -- attitude priority, and no rotor ever stops.
    saturated = bool((T < 0.0).any() or (T > caps).any())
    if saturated:
        # ★ NEVER COMMAND A ROTOR TO ZERO. Scaling so the most-limited rotor lands exactly AT
        # zero satisfies "keep it >= 0" but STOPS that rotor -- and with the spin-up lag it takes
        # ~0.12 s to come back, during which the airframe has one dead corner. Keep a floor of
        # ROTOR_FLOOR_FRAC of nominal so every rotor stays spinning and the differential is
        # merely reduced.
        sc = 1.0
        for k in range(4):
            if dT[k] < -1e-9:                      # this rotor is being pushed DOWN
                room = max(0.0, nom[k] * (1.0 - ROTOR_FLOOR_FRAC))
                sc = min(sc, room / (-dT[k]))
            elif dT[k] > 1e-9:                     # ... or UP
                sc = min(sc, max(0.0, caps[k] - nom[k]) / dT[k])
        T = nom + max(0.0, min(1.0, sc)) * dT
        T = np.clip(T, nom * ROTOR_FLOOR_FRAC, caps)
    # ★ NEWTONS -> THROTTLE through the propeller model, per rotor, at ITS OWN axial inflow.
    # `t / MAX_THRUST` is the flat-model mapping and is wrong twice over here (quadratic, and
    # speed-dependent); it is what made the controller under-deliver and stall at 13.5 m/s.
    mot = [thrust_to_throttle(T[0], v_ax_f), thrust_to_throttle(T[1], v_ax_f),
           thrust_to_throttle(T[2], v_ax_r), thrust_to_throttle(T[3], v_ax_r)]

    # ---- surfaces: invert each surface's own moment equation (dynamic-pressure normalised) ----
    # delta = M / (qbar*S*ref*C_delta). The delivered moment then matches the demand at EVERY
    # speed and the sign comes from the coefficient, not from a hand-tuned gain. (CNDR is
    # NEGATIVE: +dr yaws left. A +k_rud*Mz rudder once amplified yaw to -100 deg, HIL bag myrun.)
    qbarS = 0.5 * so3.RHO * V * V * so3.S_WING
    ail = _cl(M_surf[0] / (qbarS * so3.B_WING * so3.CLDA * DA_MAX + 1e-9), -1.0, 1.0)
    # ★ RUDDER SIGN. Uses -CNDR, NOT CNDR. The coefficients are published in the STANDARD (FRD)
    # frame and the plant converts FLU->FRD and back; that flip negates Mz (and My) while leaving
    # Mx alone. So the aileron can use CLDA directly -- verified, it opposes a roll rate correctly
    # -- but the rudder cannot use CNDR directly, and doing so makes the yaw damping POSITIVE
    # FEEDBACK. Measured before the fix: omega_z = +0.30 rad/s produced rud +0.141, which gave
    # +0.481 rad/s^2 of yaw acceleration -- reinforcing the very rate it was meant to damp, and
    # overcoming the airframe's own weathercock stability to do it. That is why yaw drifted to
    # 13 deg on its own in a NOISE-FREE sim with nothing disturbing it (bag vtol_20260911_102706),
    # which then drove the bank-to-turn law and rolled the aircraft over.
    # CLAUDE.md records nmpc6 hitting this same trap ("+k_rud*Mz AMPLIFIED the yaw -> -100 deg").
    # VERIFY THIS EMPIRICALLY after any change: a positive yaw rate must produce a NEGATIVE
    # yaw acceleration.
    rud = _cl(M_surf[2] / (qbarS * so3.B_WING * (-so3.CNDR) * DR_MAX + 1e-9), -1.0, 1.0)
    # elevator = the outer's pitch TRIM plus the inner's pitch-rate DAMPING share
    de_damp = M_surf[1] / (qbarS * so3.C_WING * so3.CMDE * DE_MAX - 1e-9)
    elev = _cl(de_trim / DE_MAX + de_damp, -1.0, 1.0)
    if saturated:
        ei = ei_in                      # conditional integration: do not charge while saturated
    return mot, ail, elev, rud, ei


def safe_hover():
    """The only command recoverable from an upset: rotors UP, equal thrust, neutral surfaces."""
    t = (so3.M * so3.G / 4.0) / MAX_THRUST
    return [t] * 4, 0.0, 0.0, 0.0



def track_heading(p_xy, target_xy, track0, heading_latched, k_ct, max_corr, capture=3.0):
    """Heading command that flies the aircraft back onto the straight track to `target_xy`.

    ★ WHY THIS EXISTS: the cascade closes ALTITUDE and SPEED but had no horizontal position loop
    at all -- it flew the heading latched at engage and never saw a target, so cross-track error
    integrated freely and the steady-state position error grew without bound. The outer NMPC
    cannot fix this: `s` is a state of its model but appears in NO cost term, and its reference
    carries no position.

    Rather than add a position term to the solver (which would mean a new NLP and a JIT rebuild),
    the correction goes where the authority already is: heading error already commands BANK
    through the bank-to-turn law, so steering the heading steers the aircraft.

        heading_cmd = track bearing + clip(k_ct * cross_track_error, +-max_corr)

    The correction is capped so a large cross-track error asks for a sane intercept angle instead
    of a 90 deg turn, and `capture` softens it near the track so it does not chatter about zero.
    Returns (heading_cmd, cross_track_error).
    """
    if target_xy is None or track0 is None:
        return heading_latched, 0.0
    d = np.asarray(target_xy, float) - np.asarray(track0, float)
    n = float(np.linalg.norm(d))
    if n < 1e-3:
        return heading_latched, 0.0
    t_hat = d / n                                   # unit vector along the commanded track
    rel = np.asarray(p_xy, float) - np.asarray(track0, float)
    # cross-track error: positive means LEFT of the track (t_hat rotated +90 deg)
    e_ct = float(-t_hat[1] * rel[0] + t_hat[0] * rel[1])
    bearing = math.atan2(t_hat[1], t_hat[0])
    corr = k_ct * e_ct
    if abs(e_ct) < capture:                         # soften near the track: no chatter about zero
        corr *= abs(e_ct) / capture
    corr = _cl(corr, -max_corr, max_corr)
    # left of track (e_ct > 0) -> steer RIGHT -> heading DECREASES
    return math.atan2(math.sin(bearing - corr), math.cos(bearing - corr)), e_ct


# ============================== the outer loop ==============================

class OuterNMPC(n6.NMPC6):
    """The 6-state longitudinal NMPC with AUDIT FINDING C9 BACK-PORTED.

    nmpc6 ships `w_beta = 12`, a SOFT tilt guide. C9: "a soft tilt guide leaves a nearly flat
    tilt-vs-pitch optimum and the solver hunts along it (applied tilt 17->5->24->13 deg against a
    steady 21-24 reference). vtol_nmpc raised it for exactly this and it was never back-ported."
    It was never back-ported to nmpc6 either, and the hunting is reproducible here: driving the
    TRUE plant, the commanded tilt bang-banged 10->40->11->0->39->43->15->2->46->45->0 deg at ~1 Hz
    while the solver reported ok=1 throughout -- it was not failing, it was hunting along a flat
    optimum. nmpc6's OWN offline test shows the same signature (10/16/9/32/47/14/85/90/11/90/54/0).

    The airspeed tilt clamp (`tilt_clamp`) is what owns corridor safety here, so the tilt guide does
    not also need to be soft to keep the solver from over-tilting -- which is the same reasoning
    that let vtol_nmpc raise it.

    The JIT cache key hashes these weights, so this subclass compiles and caches separately; the
    stock nmpc6 solver is untouched.
    """
    w_beta = 60.0
    # ★ PITCH-RATE PENALTY. The inner loop CANNOT damp the short period -- it is slower than the
    # mode and the ~65 ms actuator lag forbids speeding it up (see pitch_ratio). The outer NMPC
    # can, because it PREDICTS the mode over its horizon instead of reacting to it, so it does not
    # pay the same phase penalty. nmpc6 ships w_q = 3.0. Baked into the NLP, so changing it costs
    # a JIT rebuild and gets its own cache entry.
    w_q = float(os.environ.get("CASC_WQ", "3.0"))

    def __init__(self):
        super().__init__()
        # ALLOW AFT TILT. nmpc6 bounds beta to [0, pi/2], so the back-transition's negative tilt
        # would be clipped to 0 and the reference would stop being reachable. The bounds are
        # RUNTIME arrays (passed as lbx/ubx at solve time) and the JIT cache key does not include
        # them, so widening this costs no rebuild and cannot load a stale solver.
        for k in range(self.N):
            self.lbx[self.iU + self.nu * k + 2] = -BETA_AFT_MAX


def build_refs(sched, N, DT, period, v_cmd, Vc, ttr, z_ref,
               decel=False, decel_rate=1.5, flare=0.0, beta_aft=0.0, b_slew=35.0,
               b_prev=None, th_slew=8.0, th_prev=None):
    """Horizon reference, shared by the ROS node and the offline gate.

    Forward: run the trim schedule UP on the speed clock. Decelerating: run the SAME schedule
    BACKWARDS, plus a nose-up flare and an aft tilt -- so the rotors come upright IN STEP with the
    wing unloading instead of snapping. Returns (refs, b_ref_prev).

    ONE implementation on purpose: the offline test used to rebuild this inline, and a gate that
    exercises different code from the flight path is not a gate.
    """
    out = []
    for k in range(N):
        if decel:
            V = max(v_cmd - k * DT * decel_rate, 0.0)
        else:
            V = min(v_cmd + k * DT * (Vc / max(ttr, 1e-3)), Vc)
        s = sched.at(V)
        th = float(s["theta"]); beta = float(s["beta"])
        Tf, Tr = float(s["Tf"]), float(s["Tr"])
        if decel and Vc > 1e-3:
            # brake hardest while there is still speed to kill, fading to nothing at hover
            frac = min(1.0, V / Vc)
            th = th + flare * frac
            beta = -beta_aft * frac
        # ★ RATE-LIMIT THE COMMANDED TILT. The applied tilt is slew-limited to 80 deg/s, so a
        # reference that STEPS is one the aircraft cannot reach -- and the thrust split, solved
        # for the reference tilt, is then unbalanced for the tilt it is actually at. Forward
        # this costs a pitch excursion; BACKWARD it is fatal, because the aft-tilt brake is
        # FULL at the instant decel starts: the reference jumps +90 -> -20 deg in one tick,
        # every solve fails from that moment and the aircraft just cruises on.
        if b_prev is not None:
            lim = math.radians(b_slew) * period * (k + 1)
            beta = max(b_prev - lim, min(b_prev + lim, beta))
        # ★ AND RATE-LIMIT THE COMMANDED PITCH, for exactly the same reason. The nose-up flare is
        # `flare * min(1, V/Vc)`, which is FULL at the instant decel begins: the reference pitch
        # STEPPED +5.7 -> +20.7 deg in one tick, the inner loop chased a 15 deg step with the
        # elevator hard over, and the aircraft was on the ground 5 s later. Rate-limiting the tilt
        # alone is not enough -- the flare and the aft tilt are two halves of one manoeuvre and
        # both have to be eased in.
        if th_prev is not None:
            lim = math.radians(th_slew) * period * (k + 1)
            th = max(th_prev - lim, min(th_prev + lim, th))
        # a trim is a PACKAGE -- re-solve the split for whatever tilt/pitch we just commanded
        if abs(beta - float(s["beta"])) > 1e-9 or abs(th - float(s["theta"])) > 1e-9:
            try:
                Tf, Tr = so3.trim_split(beta, th, max(V, 1.0), float(s["de"]),
                                        so3.M * so3.G, 0.0)
                Tf, Tr = float(Tf), float(Tr)
            except (np.linalg.LinAlgError, ValueError, ZeroDivisionError):
                pass                     # keep the schedule's thrusts rather than ship nothing
        out.append(dict(z=z_ref, vs=V, vz=0.0, theta=th, beta=beta,
                        Tf=Tf, Tr=Tr, de=float(s["de"])))
    return out, out[0]["beta"], out[0]["theta"]


class Outer:
    """NMPC6 in a background thread. CasADi releases the GIL during the solve, so the inner loop
    keeps running at mocap rate while a solve is in flight (this is the decouple that made the
    earlier SO(3) cascade work: the fast loop held ~99 Hz through 0.24-0.94 s solves).

    Publishes ONE package under a lock: (Tf, Tr, beta, de, theta). A trim is a package -- shipping
    a tilt without the thrusts that balance it is how bags full_tr7 and full_tr25/27 were lost.
    """

    def __init__(self, nmpc, sched, log, hz=20.0):
        self.nmpc, self.sched, self.log = nmpc, sched, log
        self.period = 1.0 / max(hz, 1.0)
        self.margin = math.radians(15.0)
        self.latency = 0.10
        self.F6 = n6.build_ode6()
        self.ubuf = []
        self.decel = False
        self.decel_rate = 1.5
        self.flare = math.radians(15.0)
        self.beta_aft = math.radians(20.0)
        self.b_slew = 35.0
        self.th_slew = 8.0
        self._b_ref_prev = None
        self._th_ref_prev = None
        self.lock = threading.Lock()
        self.pkg = None                  # (Tf, Tr, beta, de, theta)
        self.solve_ms = 0.0
        self.fails = 0
        self.solves = 0
        self._req = None                 # (x0, v_cmd, Vc, ttr, z_ref) posted by the fast loop
        self._stop = False
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def post(self, x0, v_cmd, Vc, ttr, z_ref):
        with self.lock:
            self._req = (np.array(x0, float), float(v_cmd), float(Vc), float(ttr), float(z_ref))

    def get(self):
        with self.lock:
            return self.pkg, self.solve_ms, self.fails, self.solves

    def reset(self):
        with self.lock:
            self.pkg = None; self.fails = 0; self.solves = 0
            self.ubuf = []
            self._b_ref_prev = None; self._th_ref_prev = None; self.decel = False

    def stop(self):
        self._stop = True

    def _refs(self, v_cmd, Vc, ttr, z_ref):
        out, self._b_ref_prev, self._th_ref_prev = build_refs(
            self.sched, self.nmpc.N, self.nmpc.DT, self.period, v_cmd, Vc, ttr, z_ref,
            self.decel, self.decel_rate, self.flare, self.beta_aft, self.b_slew,
            self._b_ref_prev, self.th_slew, self._th_ref_prev)
        return out


    def _run(self):
        while not self._stop:
            with self.lock:
                req = self._req
            if req is None:
                time.sleep(0.005); continue
            x0, v_cmd, Vc, ttr, z_ref = req
            refs = self._refs(v_cmd, Vc, ttr, z_ref)
            n_lat = int(round(self.latency / self.period))
            x0 = predict6(self.F6, x0, self.ubuf[-n_lat:] if n_lat > 0 else [], self.period)
            t0 = time.perf_counter()
            try:
                u, ok = self.nmpc.solve(x0, refs)     # NB: returns (control, converged-flag)
            except Exception as e:                        # noqa: BLE001 - never kill the thread
                self.log(f"outer solve raised: {type(e).__name__}: {e}")
                u, ok = None, False
            ms = (time.perf_counter() - t0) * 1000.0
            with self.lock:
                self.solve_ms = ms
                self.solves += 1
                if ok and u is not None:
                    Tf, Tr, beta, de = (float(u[0]), float(u[1]), float(u[2]), float(u[3]))
                    th = float(refs[0]["theta"])
                    V_now = math.hypot(float(x0[2]), float(x0[3]))
                    beta, Tf, Tr = tilt_clamp(beta, Tf, Tr, de, th, V_now,
                                              self.sched, self.margin)
                    self.pkg = (Tf, Tr, beta, de, th)
                    self.ubuf.append(np.array([Tf, Tr, beta, de]))
                    self.ubuf = self.ubuf[-40:]
                else:
                    self.fails += 1
                    # Leave the LAST GOOD package in place. Do NOT substitute the schedule trim:
                    # at speed that trim is tilt=90, and applying it while upset is what turned an
                    # upset into a death spiral in bag vt2.
            slack = self.period - (time.perf_counter() - t0)
            if slack > 0.0:
                time.sleep(slack)


# ============================== ROS node ==============================
def run_ros():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                           QoSHistoryPolicy, qos_profile_sensor_data)
    from px4_msgs.msg import (ActuatorMotors, ActuatorServos, OffboardControlMode,
                              VehicleCommand, VehicleStatus)
    from std_msgs.msg import String
    from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference

    class CascadeNode(Node):
        def __init__(self):
            super().__init__("nmpc_cascade_control")
            P = self.declare_parameter
            P("px4_ns", "uav_0"); P("controller_id", "cascade"); P("active_at_start", False)
            # ★ auto_arm is FALSE. The operator controls arming and offboard -- user directive.
            P("auto_arm", False); P("arm_after", 150)
            P("rate_hz", 50.0)            # SLOW timer: OCM + reference clock + status
            P("outer_hz", 20.0)           # NMPC solve rate (bench: 9-12 ms -> 20 Hz is 4x margin)
            # ★ 15.0, not 16. With thrust_model="momentum" the prop loses thrust to its own inflow, so
            # the cruise speed is now a real cost: 4.79 N/rotor available at 16 m/s vs 5.62 at 15.
            # 14 m/s was requested but does NOT close -- level flight there needs CL = 0.805 against
            # a CL PEAK of 0.791, i.e. the wing cannot carry the weight and the AoA required is past
            # the peak where lift FALLS. 15 needs CL = 0.702 at AoA +6.6 deg, with margin.
            P("vcruise", 16.0); P("cap_deg", 90.0); P("alpha_max_deg", 8.0)
            # ★ 20 s, not nmpc_full's 40. Swept offline against the true plant at the measured
            # 30 ms latency: EVERY ramp from 40 s down to 4 s holds altitude at 30.0 m with 0/2201
            # solve failures, and peak AoA IMPROVES as the ramp quickens (11.3 deg at ttr=40 -> 6.7
            # at ttr=6) -- a quicker crossing spends less time below stall speed where the wing
            # cannot carry the weight. That inverts the nmpc_full lore ("18 s rang the phugoid"),
            # which was measured on a different controller and the pre-2026-09-06 plant.
            # 20 s is HALF the flown transition at the same peak AoA; the offline limit is far
            # lower. `tilt_margin_deg` is NOT the lever -- 15 through 90 deg give identical
            # results, so the airspeed clamp never binds; the tilt follows the SPEED ramp.
            P("ttr", 20.0)
            P("v_lead", 2.5)              # bounded lead; the reference must NEVER re-anchor to V_now
            P("settle_s", 2.0); P("cruise_alt", 30.0); P("cruise_alt_band", 15.0)
            # inner-loop gains
            # ★ kR = 3.0, NOT the safe controller's 9.0. Measured across the latency range, and
            # softer wins everywhere -- it is not a robustness-for-performance trade:
            #   lat 0.05/0.10   kR 3.0 -> 0/2401 solve failures, AoA settles on the EXACT +5.7 trim
            #   lat 0.10        kR 9.0 -> 5/4001, AoA wanders to +10.8 (it excites the phugoid)
            #   lat 0.15        kR 9.0 -> ground at 7.0 s;  kR 3.0 -> full 120 s cruise
            # The latency ceiling is this loop's, not the solver's: CLAUDE.md measured the safe
            # controller stable to 125 ms and diverging at 150 with kR=9 at 200 Hz, "the ATTITUDE
            # loop goes first". kR=3.0 moves that to ~150 ms. If HIL measures more, soften further.
            # ★ kW IS BOUNDED BY THE ROTOR SPIN LAG, NOT BY THE MODE IT DAMPS.
            # The sim's rotors carry rotor_tau_spin=0.12 s of first-order lag (VTOLConfig
            # default, forced ON by thrust_model="momentum"), and the controller does not model
            # it. Rate feedback through that lag is ~46 deg of phase at the 1.4 Hz short period,
            # so past a threshold the RATE term stops damping and starts driving. Measured
            # offline against the true plant at 60 ms loop latency, with the sim's actuator lags:
            #     kW 1.0 / 1.2 / 1.4 -> flies, z=10.0 V=16.02 tilt=90.0 roll=0.0
            #     kW 1.6 / 1.8       -> INVERTED, on the ground
            # and kR is almost irrelevant (1.2, 2.0 and 3.0 all crash at kW=1.8, all fly at 1.2).
            # 1.8 was validated BEFORE the rotor/servo lags existed. Flown at 1.8: bag
            # vtol_20260911_145534, a 1.4 Hz pitch limit cycle growing 12 -> 17 -> 58 deg until
            # it departed, roll/yaw flat at 0.4 deg throughout -- a pure pitch failure.
            # DO NOT RAISE THIS to chase tracking; it is a stability boundary. If more pitch
            # damping is genuinely needed, take the LAG out (send the fast rate term to the
            # surfaces, which lag 0.03 s, not the rotors) rather than raising the gain.
            P("kR_roll", 3.0); P("kW_roll", 1.2)
            P("kR_pitch", 3.0); P("kW_pitch", 1.2)   # FLOORS; scheduled above by qbar
            # ★ DEFAULT 0 = OFF. The observation behind this knob is correct -- the airframe's
            # short period rises with qbar (0.29 Hz at 4 m/s, 1.17 at 16, zeta only 0.29) while a
            # fixed kR_pitch parks the loop at 0.50 Hz, so above ~8 m/s the controller is SLOWER
            # than the mode it is meant to damp. But the prescription does NOT work: the loop
            # cannot be sped up, because the actuator chain is ~65 ms of lag (30 ms transport plus
            # the 0.03 s surface servo and its 400 deg/s rate limit), which at the 2.35 Hz a 2x
            # schedule asks for is ~55 deg of phase. Measured, max |pitch| under noise:
            #     ratio 0.1 (floor) 6.2 deg   0.4 -> 6.2   0.7 -> 6.5   1.0 -> 15.5   >=1.5 CRASH
            # So raising pitch bandwidth is monotonically WORSE and the floor is already best.
            # Damping the short period needs either LESS LAG or a predictive loop -- the OUTER
            # NMPC is the natural place, since it models the mode instead of reacting to it, and
            # its w_q term is the lever. Do not raise this expecting a fix.
            P("pitch_ratio", 0.0)        # >0: hold the loop this many x above the short period
            P("pitch_zeta", 0.7)         # damping the scheduled loop is designed for
            P("kR_yaw", 5.0); P("kW_yaw", 2.0)
            # cross-track guidance: closes the HORIZONTAL POSITION loop the cascade
            # otherwise lacks entirely (it flew a latched heading and never saw a target).
            # ★ MOCAP NOISE INJECTION, default OFF. The whole simulation chain is noise-free:
            # the emulator republishes EXACT state (measured frame-to-frame attitude jitter
            # 0.00042 deg sd, vs 0.02-0.1 for a real OptiTrack). So neither the offline gate nor
            # HIL exercises noise rejection, and a clean lateral result from either proves nothing
            # about it -- 0.1 deg of attitude noise takes roll from 0.0 to ~20 deg offline.
            # These corrupt only the CONTROLLER's view; the plant keeps its true state.
            P("debug_yaw", False)        # log the yaw-loop internals once a second
            P("noise_att_deg", 0.0)      # attitude 1-sigma; 0.1 is representative of real mocap
            P("noise_rate_dps", 0.0)     # body-rate 1-sigma [deg/s]; ~1.0 is representative
            P("noise_vel", 0.0)          # velocity 1-sigma [m/s]; ~0.02 is representative
            P("k_ct", 0.05)              # rad of heading correction per metre off track
            P("ct_max_deg", 35.0)        # largest intercept angle it will ask for
            # heading loop: bounded TURN RATE, converted to a coordinated bank.
            P("k_head", 0.5)             # 1/s: heading error -> desired turn rate
            P("turn_rate_max_dps", 12.0) # largest turn rate it will ask for
            P("k_head_d", 0.8)           # damping on the TRUE heading rate
            P("k_bank", 1.2)             # (legacy, unused by the turn-rate law)
            # Yaw-RATE damping on the bank command. DEFAULT 0.0 = OFF: it was added to try to
            # damp the slow lateral oscillation described above and IT DID NOT WORK -- swept
            # 0.0/0.5/0.8/1.5 against a roll15+yaw-25 upset over 300 s, none converged (1.5 parks
            # the yaw at +34 deg with roll drifting instead). Kept as a knob because the structure
            # is right (a P-only command on a double integrator needs a rate term) but the loop
            # needs proper design, not gain search. Do not enable it expecting a fix.
            # attitude integral: removes the standing error a pure PD leaves against a
            # steady disturbance. Band-limited + clamped + frozen while saturated.
            P("kI_roll", 0.8); P("kI_yaw", 0.4)
            P("i_band_deg", 12.0)         # only integrate inside this error band
            P("i_max", 1.5)               # N.m of stored correction, hard clamp
            P("k_bank_d", 0.0)
            P("bank_max_deg", 25.0)      # bank limit while wing-borne
            P("tilt_margin_deg", 15.0)   # airspeed tilt clamp headroom
            # ★ MEASURED, not guessed: 30 ms. Cross-correlating the elevator command against
            # the resulting pitch ACCELERATION over 20 s of steady HIL cruise (bag casc1) peaks
            # at lag 30 ms, corr +0.66 -- the elevator is a direct moment actuator, so that lag
            # IS the command->effect delay of mocap -> node -> DDS -> PX4 -> Isaac -> physics.
            # It was set to 0.12 from the 19-state node, which OVER-compensates by 4x: the outer
            # then plans for a state the aircraft will never be in. Offline, over-compensation is
            # not benign for this cascade (true .15 / comp .20 still crashed).
            # Re-measure with scripts/casc_timing.py after any change to the stack's rates.
            P("latency_s", 0.04)
            # ---- PROPELLER MODEL (must match the sim's RotorConfig / thrust_model) ----
            # The controller inverts this to turn desired NEWTONS into a throttle. If it drifts
            # from the airframe's actual prop the controller silently under- or over-delivers,
            # which is exactly what a flat mapping did: it planned a trim it could not buy and
            # parked at 13.5 m/s / 67 deg tilt instead of reaching 90.
            # ★ DOUBLE, not bool. The ground station types every mission parameter it sends as
            # PARAMETER_DOUBLE unless the value is literally a bool or the string "true"/"false",
            # and CASC_OVERRIDES carries numbers. Declaring this as a bool made the node REJECT
            # it, and the auto mission aborts on any rejected parameter -- "NMPC node did not
            # accept inflow". `decel_now` in this same node is already a float flag for exactly
            # this reason; follow that convention for anything the GS sets.
            P("inflow", 1.0)             # 0.0 -> legacy flat T = throttle * MAX_THRUST
            P("prop_diameter_m", 0.3048)
            P("prop_zero_thrust_J", 0.6)
            P("rotor_omega_max", 749.0)
            # how much FORWARD force the attitude differential may disturb [N].
            # At 90 deg tilt a rotor pitch correction is almost pure thrust, so this
            # is what stops the pitch loop pushing the aircraft off its trim speed.
            P("fx_budget", 0.10)
            P("rear_floor", 4.0)          # keep the rear pair loaded -> it retains ROLL authority
            # ---- BACK-TRANSITION (the GS auto mission drives these) ----
            # Readiness handshake for the GS auto mission: reports the cruise speed the trim
            # schedule is ACTUALLY built for, so the mission can refuse to launch against a
            # schedule built for a different speed instead of silently flying the wrong one.
            P("sched_ready_v", 0.0)
            P("decel_now", 0.0)           # 1.0 = run the trim schedule BACKWARDS to a stop
            P("decel_rate", 1.5)          # m/s^2 the commanded speed bleeds down at
            P("flare_deg", 15.0)          # nose-up flare. NEVER below ~15 with aft tilt enabled:
            #                               at flare 5 / aft 20 the aircraft DEPARTS (bag full_tr35,
            #                               AoA ran to 172 deg flying backwards, -40 m).
            P("beta_aft_deg", 20.0)       # how far the FRONT pair leans AFT to brake
            P("tilt_slew_ref_deg_s", 35.0)  # rate limit on the TILT reference
            P("pitch_slew_deg_s", 8.0)    # rate limit on the PITCH reference -- omitting this put
            #                               the aircraft on the ground 5 s after decel began
            P("tilt_slew_deg_s", 80.0)    # the sim's servo rate limit; model what the plant does

            ns = self.get_parameter("px4_ns").value
            pre = f"/{ns}" if ns else ""
            self.cid = self.get_parameter("controller_id").value
            self._active = bool(self.get_parameter("active_at_start").value)
            self.auto_arm = bool(self.get_parameter("auto_arm").value)
            self.arm_after = int(self.get_parameter("arm_after").value)

            self.get_logger().info("building 6-state outer NMPC...")
            self.nmpc = OuterNMPC()
            self.sched = n6.Schedule(V_cruise=self.gp("vcruise"),
                                     tilt_cap_deg=self.gp("cap_deg"),
                                     alpha_max_deg=self.gp("alpha_max_deg"))
            self.get_logger().info("outer NMPC built.")
            self._sched_v = self.gp("vcruise")
            # push the prop model into the module-level constants the mapping uses
            self._apply_prop_params()
            self.alloc = RotorAlloc()
            self.outer = Outer(self.nmpc, self.sched, self.get_logger().warn,
                               hz=self.gp("outer_hz"))
            self.outer.margin = math.radians(self.gp("tilt_margin_deg"))
            self.outer.latency = self.gp("latency_s")

            self.state = None; self.p0 = None; self.heading = 0.0; self.z0 = 0.0
            self.target = None; self.track0 = None
            self.heading_cmd = 0.0; self._e_ct = 0.0
            self._noise_rng = np.random.default_rng(1)
            self._dbg_yaw = bool(self.get_parameter('debug_yaw').value)
            self.armed = self.offb = False; self.counter = 0; self._eng = 0
            self._v_cmd = 0.0; self._settled = False; self._beta = 0.0
            self._t_prev = None; self._logged_exc = False
            self._ei = np.zeros(3)
            self._logged_slow_exc = False; self._warned_vc = False

            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            self.motp = self.create_publisher(ActuatorMotors, f"{pre}/fmu/in/actuator_motors", qos)
            self.srvp = self.create_publisher(ActuatorServos, f"{pre}/fmu/in/actuator_servos", qos)
            self.ocm = self.create_publisher(OffboardControlMode,
                                             f"{pre}/fmu/in/offboard_control_mode", qos)
            self.cmdp = self.create_publisher(VehicleCommand, f"{pre}/fmu/in/vehicle_command", qos)
            self.create_subscription(VehicleStatus, f"{pre}/fmu/out/vehicle_status_v1",
                                     self.on_status, qos)
            self.create_subscription(String, f"{pre}/ground_station/active_controller",
                                     self.on_active, 10)
            self.create_subscription(Mocap, f"{pre}/mocap", self.on_mocap, qos_profile_sensor_data)
            # ★ POSITION REFERENCE. Without this the cascade flies a HEADING latched at engage and
            # never sees a target, so cross-track error integrates freely -- altitude and speed are
            # closed loops but horizontal POSITION is not controlled at all. The launcher remaps
            # this to `reference_smc`.
            self.create_subscription(
                PositionControllerReference,
                f"{pre}/fsc_autopilot_ros2/position_controller/reference",
                self.on_ref, 10)
            self.create_timer(1.0 / self.gp("rate_hz"), self.slow)
            self.get_logger().info(
                f"cascade up: id={self.cid} outer={self.gp('outer_hz'):.0f}Hz "
                f"inner=mocap-rate auto_arm={self.auto_arm}")

        def gp(self, n): return float(self.get_parameter(n).value)

        def _apply_prop_params(self):
            """Propagate the propeller parameters into the module constants the thrust<->throttle
            mapping uses, and report what the airframe can actually do at the commanded cruise --
            the single number that decides whether a mission is flyable at all."""
            # ★ WRITE INTO *THIS* MODULE'S GLOBALS. `import nmpc_cascade_node` would bind a
            # SECOND module object when the file runs as __main__ (which is how the node is
            # launched), so the constants would be set on a copy while the live
            # thrust_to_throttle() kept reading the originals -- a param change that silently
            # did nothing. sys.modules[__name__] is the module actually executing.
            import sys as _sys
            g = vars(_sys.modules[__name__])
            g["PROP_D"] = self.gp("prop_diameter_m")
            g["PROP_J0"] = max(self.gp("prop_zero_thrust_J"), 1e-3)
            g["OMEGA_MAX"] = self.gp("rotor_omega_max")
            g["_N_MAX"] = g["OMEGA_MAX"] / (2.0 * math.pi)
            g["_CT0"] = MAX_THRUST / (so3.RHO * g["_N_MAX"] ** 2 * g["PROP_D"] ** 4)
            g["INFLOW"] = float(self.get_parameter("inflow").value) > 0.5
            vc = self.gp("vcruise")
            avail = thrust_available(vc)
            # ★ LOG ONLY ON CHANGE. This is called from slow(), which runs at rate_hz (50 Hz),
            # so an unconditional info() here emitted the banner 50x/second: it pushed the CASC
            # status lines out of the pane inside a second and made the flight monitor report
            # "STATUS FROZEN -- node has stopped logging" on a node that was perfectly healthy
            # and mid-mission. The APPLY must stay every tick (that is how a live `ros2 param
            # set` takes effect); only the report is conditional.
            stamp = (round(g["PROP_D"], 4), round(g["PROP_J0"], 4),
                     round(g["OMEGA_MAX"], 1), bool(g["INFLOW"]), round(vc, 2))
            if stamp != getattr(self, "_prop_stamp", None):
                self._prop_stamp = stamp
                self.get_logger().info(
                    f"prop: D={g['PROP_D']:.3f} m J0={g['PROP_J0']:.2f} "
                    f"inflow={g['INFLOW']} -> {avail:.2f} N/rotor at {vc:.1f} m/s "
                    f"(static {MAX_THRUST:.1f})")

        def _publish_ready(self):
            """Report the speed the trim schedule is built for, so the ground station's mission
            build can verify it rather than assume it."""
            try:
                if abs(self.gp("sched_ready_v") - self._sched_v) > 1e-6:
                    self.set_parameters([rclpy.Parameter(
                        "sched_ready_v", rclpy.Parameter.Type.DOUBLE, float(self._sched_v))])
            except Exception as e:                      # noqa: BLE001
                self.get_logger().warn(f"could not publish sched_ready_v: {e}")
        def now_us(self): return int(self.get_clock().now().nanoseconds / 1000)

        def on_status(self, m):
            self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            self.offb = (m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        def on_ref(self, m):
            """Latest commanded position. Only the HORIZONTAL part is used: altitude and speed
            are already tracked by the outer NMPC, and letting the reference drive them too would
            put two controllers on the same axis."""
            try:
                self.target = np.array([float(m.position.x), float(m.position.y)])
            except Exception:                             # noqa: BLE001
                pass

        def on_active(self, m):
            was = self._active
            self._active = (m.data == self.cid)
            if self._active and not was:
                self._engage()

        def _engage(self):
            """Reset EVERY latched piece of engage state. Stale state here produced a reference
            that opened at tilt=32 deg on a level hover (the user's 'the reference looks horrible')."""
            self._eng = 0; self._v_cmd = 0.0; self._settled = False; self._beta = 0.0
            # ★ a stale integral IS a stored disturbance: carried across an engage it commands a
            # standing moment the aircraft never asked for, from the first tick.
            self._ei = np.zeros(3)
            self.outer.reset()
            if self.state is not None:
                p, R, _, _ = self.state
                self.p0 = p.copy(); self.z0 = float(p[2])
                self.heading = math.atan2(R[1, 0], R[0, 0])
            self.get_logger().info(f"ENGAGED at z={self.z0:.1f} heading={math.degrees(self.heading):.0f}")

        def on_mocap(self, m):
            q = m.pose.orientation
            R = n6._q2R(q.x, q.y, q.z, q.w)
            p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            v = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])
            w = np.array([m.twist.angular.x, m.twist.angular.y, m.twist.angular.z])
            # ★ inject measurement noise if asked. Corrupts only what the CONTROLLER sees --
            # the plant's own state is untouched, exactly as a real sensor error behaves.
            na = self.gp("noise_att_deg"); nr = self.gp("noise_rate_dps"); nv = self.gp("noise_vel")
            if na > 0.0 or nr > 0.0 or nv > 0.0:
                rng = self._noise_rng
                if na > 0.0:
                    d = rng.normal(0.0, math.radians(na), 3)
                    K = np.array([[0.0, -d[2], d[1]], [d[2], 0.0, -d[0]], [-d[1], d[0], 0.0]])
                    R = R @ (np.eye(3) + K)               # small-angle attitude perturbation
                    U_, _, Vt_ = np.linalg.svd(R)
                    R = U_ @ Vt_                          # keep it a rotation
                if nr > 0.0:
                    w = w + rng.normal(0.0, math.radians(nr), 3)
                if nv > 0.0:
                    v = v + rng.normal(0.0, nv, 3)
            self.state = (p, R, v, w)
            if self._active:
                self.inner(p, R, v, w)

        # ---------------- the fast inner loop, one per mocap sample ----------------
        def inner(self, p, R, v, w):
            # ★ EXCEPTION-WRAPPED. An uncaught throw here stops the actuator stream and PX4 HOLDS
            # THE LAST COMMAND FOREVER -- bag vt1 fell for 17 s on a byte-identical frozen frame.
            try:
                now = time.perf_counter()
                dt = 0.01 if self._t_prev is None else min(0.05, max(1e-3, now - self._t_prev))
                self._t_prev = now
                V = float(np.linalg.norm(v))
                slew_up = math.radians(self.gp("tilt_slew_deg_s")) * dt
                pkg, _, _, _ = self.outer.get()

                # DIVERGED -> the only recoverable command. Applying the cruise trim while upset
                # commands full forward tilt and deepens it (bag vt2).
                roll = math.atan2(R[2, 1], R[2, 2])
                pitch = math.asin(_cl(R[2, 0], -1.0, 1.0))
                if abs(math.degrees(roll)) > 50.0 or abs(math.degrees(pitch)) > 45.0 or R[2, 2] < 0.4:
                    # ★ SAFE HOVER MEANS ROTORS UP. Two bugs lived on this line and together they
                    # made an upset unrecoverable in HIL (bag casc1):
                    #   1. it published `self._beta` -- RADIANS -- on a channel that takes a
                    #      NORMALISED command. 80.5 deg = 1.405 rad went out as 1.405, which the
                    #      sim clipped to 1.0 = 90 deg. The bag's "tilt 126.4 deg" (= 1.405 x 90)
                    #      is impossible for a servo clamped to 90 and was the giveaway.
                    #   2. even correctly scaled it HELD the cruise tilt, so the recovery command
                    #      kept the rotors pointing forward while the aircraft fell.
                    # Drive the tilt to zero on the same slew the servo enforces, so the rotors
                    # come upright as fast as the hardware allows.
                    self._beta += _cl(0.0 - self._beta, -slew_up, slew_up)
                    t0 = _cl(self._beta / (math.pi / 2.0), -1.0, 1.0)
                    mot, ail, elev, rud = safe_hover()
                    self.pub(mot, [t0, t0, ail, elev, rud])
                    return

                if pkg is None:                      # outer has not produced a package yet
                    mot, ail, elev, rud = safe_hover()
                    self.pub(mot, [0.0, 0.0, ail, elev, rud])
                    return

                Tf, Tr, beta_cmd, de, theta_ref = pkg
                Tr = max(Tr, self.gp("rear_floor"))  # keep the rear pair loaded -> ROLL authority
                # applied tilt tracks the command through the SAME 80 deg/s rate limit the sim's
                # servo enforces, so the inner loop allocates for the tilt the plant actually has.
                self._beta += _cl(beta_cmd - self._beta, -slew_up, slew_up)

                g = dict(kR_roll=self.gp("kR_roll"), kW_roll=self.gp("kW_roll"),
                         kR_pitch=self.gp("kR_pitch"), kW_pitch=self.gp("kW_pitch"),
                         kR_yaw=self.gp("kR_yaw"), kW_yaw=self.gp("kW_yaw"),
                         k_bank=self.gp("k_bank"), k_bank_d=self.gp("k_bank_d"),
                         k_head=self.gp("k_head"), k_head_d=self.gp("k_head_d"),
                         turn_rate_max=math.radians(self.gp("turn_rate_max_dps")),
                         kI_roll=self.gp("kI_roll"), kI_yaw=self.gp("kI_yaw"),
                         i_band=math.radians(self.gp("i_band_deg")),
                         i_max=self.gp("i_max"), fx_budget=self.gp("fx_budget"),
                         pitch_ratio=self.gp("pitch_ratio"),
                         pitch_zeta=self.gp("pitch_zeta"),
                         bank_max=math.radians(self.gp("bank_max_deg")))
                vb = R.T @ np.asarray(v, float)      # world -> body, for each rotor's inflow
                mot, ail, elev, rud, self._ei = inner_step(
                    R, w, theta_ref, getattr(self, 'heading_cmd', self.heading),
                    self._beta, V, Tf, Tr, de, g, self.alloc,
                    vb=vb, ei=self._ei, dt=dt)
                tsrv = self._beta / (math.pi / 2.0)   # cmd=0 is 0 deg (re-anchored 2026-09-07)
                self.pub(mot, [tsrv, tsrv, ail, elev, rud])
                # ★ YAW-LOOP DIAGNOSTIC. Three HIL flights oscillated in yaw from the first
                # second of hover -- motors alternating on the DIAGONAL (FL/RR vs FR/RL, the
                # spin-reaction pair) -- while EVERY offline model of the same situation holds
                # rock steady. So the driver is something the models do not contain, and the only
                # way to see it is from inside the live loop. Logs what the yaw channel is
                # actually being told, once a second, while active.
                if self._dbg_yaw and self.counter % 25 == 0:
                    yaw_m = math.atan2(R[1, 0], R[0, 0])
                    hd = getattr(self, "heading_cmd", self.heading)
                    self.get_logger().info(
                        "YAWDBG yaw=%+7.2f hdg_cmd=%+7.2f e_yaw=%+7.2f wz=%+7.2f "
                        "diag=%+6.3f rud=%+6.3f V=%5.2f wing=%4.2f" % (
                            math.degrees(yaw_m), math.degrees(hd),
                            math.degrees(math.atan2(math.sin(yaw_m - hd),
                                                    math.cos(yaw_m - hd))),
                            math.degrees(float(w[2])),
                            (mot[0] + mot[3]) - (mot[1] + mot[2]), rud, V,
                            _cl((V / 14.13) ** 2, 0.0, 1.0)))
            except Exception as e:                    # noqa: BLE001
                if not self._logged_exc:
                    import traceback
                    self.get_logger().error(f"INNER LOOP EXCEPTION: {e}\n{traceback.format_exc()}")
                    self._logged_exc = True
                mot, ail, elev, rud = safe_hover()
                self.pub(mot, [0.0, 0.0, ail, elev, rud])

        # ---------------- slow timer: OCM, reference clock, status ----------------
        def slow(self):
            # ★ EXCEPTION-WRAPPED, for the same reason inner() is. slow() is a TIMER callback, so
            # an uncaught throw kills the rclpy executor and the WHOLE NODE -- and PX4 then holds
            # the last actuator frame forever. That is exactly what happened in HIL (bag casc2):
            # an UnboundLocalError on the first decel_now transition killed the node mid-cruise,
            # the tilt froze at 90 deg, and the aircraft mushed from 30 m to the ground while the
            # status lines simply stopped. A frozen actuator stream is the worst failure mode this
            # vehicle has, and wrapping ONE of the two loops is not wrapping it.
            try:
                self._slow()
            except Exception as e:                        # noqa: BLE001
                if not self._logged_slow_exc:
                    import traceback
                    self.get_logger().error(
                        f"SLOW LOOP EXCEPTION: {e}\n{traceback.format_exc()}")
                    self._logged_slow_exc = True

        def _slow(self):
            self.counter += 1
            o = OffboardControlMode(); o.direct_actuator = True; o.timestamp = self.now_us()
            if self._active:
                self.ocm.publish(o)
            # ★ The readiness handshake and the schedule rebuild must run while STANDBY -- that is
            # the ONLY time the schedule may be rebuilt, and it is exactly when the ground station
            # polls for it. Returning early here made the mission time out waiting for a schedule
            # that could never be published.
            if not self._active:
                vc_now = self.gp("vcruise")
                if abs(vc_now - self._sched_v) > 1e-6:
                    self.sched = n6.Schedule(V_cruise=vc_now,
                                             tilt_cap_deg=self.gp("cap_deg"),
                                             alpha_max_deg=self.gp("alpha_max_deg"))
                    self.outer.sched = self.sched
                    self._sched_v = vc_now
                    self.get_logger().info(f"trim schedule rebuilt for {vc_now:.1f} m/s")
                self._publish_ready()
                return
            if self.state is None:
                return
            if self.auto_arm and self.counter == self.arm_after:
                self.vcmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

            # ★ READ THE STATE FIRST. The live-param block below logs the airspeed, and having
            # that line ABOVE this unpack raised UnboundLocalError on the first decel_now
            # transition, which killed the node mid-cruise in HIL (bag casc2).
            p, R, v, w = self.state
            V = float(np.linalg.norm(v))

            # The schedule is a function of Vc, so flying at a speed it was not built for is
            # flying the wrong trim. It may only be rebuilt while STANDBY (done above); in flight,
            # say so rather than silently using the stale one.
            vc_now = self.gp("vcruise")
            if abs(vc_now - self._sched_v) > 1e-6 and not self._warned_vc:
                self.get_logger().warn(
                    f"vcruise {self._sched_v:.1f} -> {vc_now:.1f} IGNORED while active; "
                    f"go to standby to rebuild the trim schedule")
                self._warned_vc = True
            self._publish_ready()

            # live-tunable: read EVERY tick, so `ros2 param set` takes effect without a restart.
            o = self.outer
            o.latency = self.gp("latency_s")
            o.decel_rate = self.gp("decel_rate")
            o.flare = math.radians(self.gp("flare_deg"))
            o.beta_aft = math.radians(self.gp("beta_aft_deg"))
            o.b_slew = self.gp("tilt_slew_ref_deg_s")
            o.th_slew = self.gp("pitch_slew_deg_s")
            o.margin = math.radians(self.gp("tilt_margin_deg"))
            self._apply_prop_params()
            decel = self.gp("decel_now") > 0.5
            if decel != o.decel:
                self.get_logger().info(
                    f"BACK-TRANSITION {'ARMED' if decel else 'cleared'} at V={V:.1f}")
            o.decel = decel

            self._eng += 1
            dt = 1.0 / self.gp("rate_hz")

            # SETTLE GATE -- LATCHES. Re-evaluating it every tick turns a 1 m altitude dip in
            # cruise into a command to untilt at speed; that ended every wing-borne episode of
            # bag full_tr15 while the flight was holding altitude to within 1 m.
            if not self._settled:
                at_alt = p[2] >= self.gp("cruise_alt") - 1.0
                if self._eng > self.gp("settle_s") * self.gp("rate_hz") and at_alt:
                    self._settled = True
                    self.get_logger().info(f"SETTLED at z={p[2]:.1f} -- starting transition")

            # SPEED REFERENCE ON ITS OWN CLOCK with a BOUNDED LEAD. It must never re-anchor to the
            # achieved speed: that pathology (the reference following the vehicle down) has cost
            # this project four separate flights.
            Vc = self.gp("vcruise")
            if decel:
                # BACKWARDS on the same clock. Bounded BELOW by V - v_lead for the same reason the
                # forward ramp is bounded above: a reference that runs away from the achieved state
                # in either direction stops being a trim the aircraft can track.
                self._v_cmd = max(self._v_cmd - dt * self.gp("decel_rate"), 0.0)
                self._v_cmd = max(self._v_cmd, V - self.gp("v_lead"))
            elif self._settled:
                self._v_cmd = min(self._v_cmd + dt * Vc / max(self.gp("ttr"), 1e-3), Vc)
                self._v_cmd = min(self._v_cmd, V + self.gp("v_lead"))
            else:
                self._v_cmd = 0.0

            # ★ CROSS-TRACK: steer back onto the line from the engage point to the target.
            # The correction is applied to the HEADING, which the bank-to-turn law already turns
            # into a bank -- so the position loop reuses authority that is already there.
            if self.track0 is None and self.p0 is not None:
                self.track0 = self.p0[:2].copy()
            self.heading_cmd, e_ct = track_heading(
                p[:2], self.target, self.track0, self.heading,
                self.gp("k_ct"), math.radians(self.gp("ct_max_deg")))
            self._e_ct = e_ct

            z_ref = max(self.gp("cruise_alt"), self.z0)
            s_now = float(np.dot(p - self.p0, [math.cos(self.heading), math.sin(self.heading), 0.0]))
            vs = float(np.dot(v, [math.cos(self.heading), math.sin(self.heading), 0.0]))
            theta = math.asin(_cl(R[2, 0], -1.0, 1.0))
            # q is the NOSE-UP pitch rate, which is -omega_y (the plant's omega[1] is nose-down
            # positive). nmpc6's own node does the same negation; feeding +w[1] inverts the pitch
            # rate the solver plans against.
            x0 = [s_now, float(p[2]), vs, float(v[2]), theta, -float(w[1])]
            self.outer.post(x0, self._v_cmd, Vc, self.gp("ttr"), z_ref)

            if self.counter % int(max(1, self.gp("rate_hz"))) == 0:
                pkg, ms, fails, solves = self.outer.get()
                b = math.degrees(self._beta)
                self.get_logger().info(
                    f"CASC z={p[2]:5.1f} V={V:5.2f} vcmd={self._v_cmd:5.2f} tilt={b:5.1f} "
                    f"th={math.degrees(theta):+6.1f} roll={math.degrees(math.atan2(R[2,1],R[2,2])):+6.1f} "
                    f"ct={self._e_ct:+5.1f}m "
                    f"outer={ms:4.1f}ms fails={fails}/{solves} "
                    f"{'settled' if self._settled else 'climbing'}")

        def pub(self, mot, srv):
            if not self._active:
                return
            mm = ActuatorMotors(); c = [math.nan] * 12
            for i in range(4):
                c[i] = mot[i]
            mm.control = c; mm.timestamp = mm.timestamp_sample = self.now_us()
            self.motp.publish(mm)
            ss = ActuatorServos(); sc = [math.nan] * 8
            for i in range(5):
                sc[i] = srv[i]
            ss.control = sc; ss.timestamp = ss.timestamp_sample = self.now_us()
            self.srvp.publish(ss)

        def vcmd(self, cmd, p1=0.0, p2=0.0):
            m = VehicleCommand(); m.command = cmd; m.param1 = p1; m.param2 = p2
            m.target_system = m.target_component = m.source_system = m.source_component = 1
            m.from_external = True; m.timestamp = self.now_us()
            self.cmdp.publish(m)

    rclpy.init()
    node = CascadeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.outer.stop()


# ============================== offline test ==============================
def offline_test():
    """Closed loop against the TRUE 19-state plant (so3.build_ode) with latency.

    Deliberately NOT the outer model. Validating a controller against its own build_ode is finding
    C5 of the controller audit -- model error cancels and the test passes with the bug present. The
    plant here is the same one the sim implements.

    Usage: --test [--roll0 D] [--yaw0 D] [--kick_q DPS] [--kick_at S] [--lat S] [--dur S]
                  [--vc V] [--alt Z]
    """
    def argf(n, d):
        return float(sys.argv[sys.argv.index(n) + 1]) if n in sys.argv else d

    Vc = argf("--vc", 16.0); dur = argf("--dur", 90.0); LAT = argf("--lat", 0.10)
    z0 = argf("--alt", 30.0); ttr = argf("--ttr", 40.0)
    roll0 = math.radians(argf("--roll0", 0.0)); yaw0 = math.radians(argf("--yaw0", 0.0))
    # ★ A PITCH KICK **AT CRUISE**, which is the only way this gate can see the short period.
    # roll0/yaw0 are applied at t=0 -- in HOVER -- so they excite a 0.3 Hz mode and say nothing
    # about the 1.4 Hz short period that actually departed in flight. Worse, an UNDISTURBED run
    # never exercises the pitch damping path at all: measured, the inner's de_damp contribution
    # has rms 0.000 at cruise here (ratio 1466:1 against the outer's trim) while in flight it is
    # ~1:6 -- so the gate has never once tested the term it is supposed to be validating.
    # A body pitch-RATE impulse is the clean excitation: it injects energy into the mode without
    # displacing the trim, so what follows is the mode's own decay (or growth).
    kick_q = math.radians(argf("--kick_q", 0.0))     # deg/s, one-shot body pitch-rate impulse
    kick_at = argf("--kick_at", 30.0)                # s, when to apply it (default: in cruise)
    rear_floor = argf("--rear_floor", 4.0)
    margin_deg = argf("--tilt_margin", 15.0)
    # ★ DISTURBANCES. Without these the offline plant is PERFECTLY SYMMETRIC with EXACT state
    # feedback, so roll and yaw sit at 0.0 because nothing ever displaces them -- not because the
    # lateral loop is good. A clean nominal run is therefore no evidence at all about lateral
    # stability, and it is how a marginally-unstable heading mode stayed hidden. Sensor noise also
    # matters disproportionately here because the inner loop differentiates nothing but DOES use
    # the measured body RATES directly in its damping terms.
    n_att = math.radians(argf("--noise_att", 0.0))     # attitude noise, deg 1-sigma
    n_rate = math.radians(argf("--noise_rate", 0.0))   # body-rate noise, deg/s 1-sigma
    n_vel = argf("--noise_vel", 0.0)                   # velocity noise, m/s 1-sigma
    gust = argf("--gust", 0.0)                         # lateral gust acceleration, m/s^2 peak
    gust_T = argf("--gust_T", 7.0)                     # gust period [s]
    rng = np.random.default_rng(int(argf("--seed", 1)))
    # ★ PROP INFLOW IN THE OFFLINE PLANT. `so3.build_ode` takes rotor thrust in NEWTONS, so the
    # sim's momentum model can be applied HERE without touching the controller's own model --
    # which is the point: it creates exactly the plant/model mismatch that switching Isaac to
    # thrust_model="momentum" creates, and lets it be measured before a flight instead of after.
    y0 = argf("--y0", 0.0)          # start this far LEFT of the commanded track
    k_ct = argf("--k_ct", 0.05); ct_max = math.radians(argf("--ct_max", 35.0))
    momentum = "--momentum" in sys.argv
    # keep the CONTROLLER's mapping and the PLANT's model in step: running the plant
    # with inflow while the controller inverts a flat model is the mismatch under test,
    # so make it explicit rather than accidental.
    globals()["INFLOW"] = momentum or ("--inflow" in sys.argv)
    PROP_D = argf("--prop_D", 0.3048); PROP_J0 = argf("--prop_J0", 0.6)
    OMEGA_MAX = argf("--omega_max", 749.0)
    _n_max = OMEGA_MAX / (2.0 * math.pi)
    _ct0 = MAX_THRUST / (so3.RHO * _n_max ** 2 * PROP_D ** 4)

    # ★ CONTROL-SURFACE SERVO. The sim's FSC airframe now runs surface_tau=0.03 s with a
    # 400 deg/s slew limit, and the aero reads the ACTUAL deflection. That lag sits in the FAST
    # PITCH loop -- the elevator is the pitch actuator at cruise -- and neither the controller nor
    # this gate modelled it, so an offline run could not see the pitch/tilt oscillation it causes.
    # ★ DEFAULTS ARE THE SIM'S ACTUAL VALUES, and that is the whole point. These defaulted to
    # ZERO until 2026-09-11, so every offline gate this campaign validated a plant with PERFECT
    # actuators -- no rotor spin lag, no servo lag -- while the flown airframe had both. The gate
    # passed a config that was unflyable: offline showed a dead-flat 90 deg cruise at the exact
    # settings that produced a divergent 1.4 Hz pitch limit cycle in HIL. A harness whose defaults
    # are more forgiving than the plant does not gate anything. Keep in step with vtol.py.
    surf_tau = argf("--surf_tau", 0.03)
    surf_rate = argf("--surf_rate", 400.0)
    _surf_state = [0.0, 0.0, 0.0]        # aileron, elevator, rudder, ACTUAL normalised deflection

    def surf_apply(cmd, i, dt):
        """Normalised command -> actual deflection, through the servo. Mirrors vtol.py exactly:
        first-order lag, then a slew clamp."""
        if surf_tau <= 1e-6 and surf_rate <= 0.0:
            return cmd
        cur = _surf_state[i]
        step = (cmd - cur) * (min(dt / surf_tau, 1.0) if surf_tau > 1e-6 else 1.0)
        if surf_rate > 0.0:
            lim = (surf_rate / 25.0) * dt          # deg/s -> normalised/s (full scale = 25 deg)
            step = max(-lim, min(lim, step))
        cur += step
        _surf_state[i] = cur
        return cur

    tau_spin = argf("--tau_spin", 0.12)      # rotor spin-up lag [s]; the sim uses 0.12 with
    #                                          thrust_model="momentum" (dynamics is forced on)
    _omega_state = [0.0, 0.0, 0.0, 0.0]

    def rotor_thrust(throttle, v_axial, idx=None, dt=None):
        """Newtons from a normalised throttle, with inflow if enabled. Identical formula to
        vtol.py's thrust_model='momentum'; CT0 derived so STATIC thrust is exactly MAX_THRUST."""
        if not momentum:
            return throttle * MAX_THRUST
        # ★ SPIN-UP LAG. The sim integrates each rotor's omega toward throttle*omega_max with a
        # first-order lag (rotor_tau_spin) whenever the thrust model has dynamics -- which
        # "momentum" forces on. The controller does NOT model it, and neither did this gate, so
        # a 0.12 s lag sat in the PITCH loop (pitch is the front/rear thrust SPLIT) entirely
        # unmodelled. That is what the offline run could not see.
        if tau_spin > 1e-6 and idx is not None and dt is not None:
            a = min(dt / tau_spin, 1.0)
            _omega_state[idx] += (throttle - _omega_state[idx]) * a
            throttle = _omega_state[idx]
        n = throttle * _n_max
        return max(0.0, so3.RHO * PROP_D * PROP_D *
                   (_ct0 * (n * PROP_D) ** 2 - (_ct0 / PROP_J0) * (n * PROP_D) * v_axial))
    every = argf("--every", 2.0)
    # ★ HOLD IN HOVER and never transition. Every other mode drives straight into the
    # corridor, so the LOW-SPEED end -- where the aero surfaces have no authority and
    # 100% of the roll/yaw moment goes through the rotor differential -- is barely
    # exercised. Three HIL flights oscillated in yaw from the first second of hover.
    hover_only = "--hover_only" in sys.argv
    decel_at = argf("--decel_at", 0.0)     # start the back-transition at this time [s]
    decel_rate_v = argf("--decel_rate", 1.5)
    flare_deg = argf("--flare", 15.0)
    beta_aft_deg = argf("--beta_aft", 20.0)
    decel_on = False; b_ref_prev = None; th_ref_prev = None
    COMP = argf("--comp", -1.0)          # delay COMPENSATION; <0 means "match --lat"
    if COMP < 0.0: COMP = LAT

    F = so3.build_ode()
    nmpc = OuterNMPC()
    sched = n6.Schedule(V_cruise=Vc, tilt_cap_deg=90.0, alpha_max_deg=8.0)
    alloc = RotorAlloc()

    DT = 0.005                       # 200 Hz plant + inner loop
    OUTER_EVERY = int(round((1.0 / 20.0) / DT))
    lat_n = max(0, int(round(LAT / DT)))

    cr, sr = math.cos(roll0), math.sin(roll0)
    cy, sy = math.cos(yaw0), math.sin(yaw0)
    R = (np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
         @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]))
    x = np.zeros(19)
    x[0:3] = [0.0, y0, z0]; x[3:12] = R.reshape(-1); x[12] = 0.0
    track0 = np.array([0.0, 0.0]); target = np.array([2000.0, 0.0])

    kR = argf("--kR", 3.0); kW = argf("--kW", 1.2)
    ei_state = np.zeros(3)
    _kicked = [False]
    g = dict(kR_roll=kR, kW_roll=kW, kR_pitch=kR, kW_pitch=kW,
             kR_yaw=argf("--kR_yaw", 5.0), kW_yaw=argf("--kW_yaw", 2.0),
             k_bank=argf('--k_bank', 1.2), k_bank_d=argf('--k_bank_d', 0.0),
             bank_max=math.radians(argf('--bank_max', 25.0)),
             k_head=argf('--k_head', 0.5), k_head_d=argf('--k_head_d', 0.8),
             turn_rate_max=math.radians(argf('--turn_rate_max', 12.0)),
             fx_budget=argf('--fx_budget', 0.10),
             pitch_ratio=argf('--pitch_ratio', 0.0), pitch_zeta=argf('--pitch_zeta', 0.7),
             kI_roll=argf('--kI_roll', 0.8), kI_yaw=argf('--kI_yaw', 0.4),
             i_band=math.radians(argf('--i_band', 12.0)), i_max=argf('--i_max', 1.5))
    heading = 0.0
    heading_cmd = 0.0; e_ct = 0.0
    v_cmd = 0.0; beta_app = 0.0; pkg = None
    ubuf = []; obuf = []; F6o = n6.build_ode6()
    settled = False
    print(f"  thrust model: {'MOMENTUM (inflow)' if momentum else 'flat'}")
    print(f"  actuator lag: rotor tau={tau_spin:.3f}s  surface tau={surf_tau:.3f}s "
          f"rate={surf_rate:.0f}deg/s")
    if not momentum:
        print("  !! WARNING: the flown FSC airframe runs thrust_model='momentum'. This run uses a "
              "FLAT\n     thrust map and is NOT a flight gate -- pass --momentum.")
    print(f"  disturbances: att={math.degrees(n_att):.2f}deg rate={math.degrees(n_rate):.2f}deg/s "
          f"vel={n_vel:.2f}m/s gust={gust:.2f}m/s^2")
    print(f"  cascade offline: Vc={Vc} alt={z0} lat={LAT*1000:.0f}ms comp={COMP*1000:.0f}ms roll0={math.degrees(roll0):.0f} "
          f"yaw0={math.degrees(yaw0):.0f} rear_floor={rear_floor}")
    print("     t     z      V  vcmd  tilt bcmd   theta   AoA   roll    yaw   wing%  rotN  ms")
    ok6 = False
    t = 0.0; k = 0; worst_roll = 0.0; worst_yaw = 0.0; solve_ms = 0.0; fails = 0; solves = 0
    while t < dur:
        R = x[3:12].reshape(3, 3); p = x[0:3]; v = x[13:16]; w = x[16:19]
        # what the CONTROLLER sees -- the plant keeps integrating the true state
        if n_att > 0.0 or n_rate > 0.0 or n_vel > 0.0:
            d = rng.normal(0.0, n_att, 3)
            K = np.array([[0.0, -d[2], d[1]], [d[2], 0.0, -d[0]], [-d[1], d[0], 0.0]])
            R = R @ (np.eye(3) + K)                       # small-angle attitude perturbation
            U_, _, Vt_ = np.linalg.svd(R); R = U_ @ Vt_
            w = w + rng.normal(0.0, n_rate, 3)
            v = v + rng.normal(0.0, n_vel, 3)
        V = float(np.linalg.norm(v))
        theta = math.asin(max(-1.0, min(1.0, R[2, 0])))

        if k % OUTER_EVERY == 0:
            if not settled and t > 2.0 and not hover_only:
                settled = True
            decel_on = decel_at > 0.0 and t >= decel_at
            if decel_on:
                # the reference speed must come DOWN on its own clock, mirroring the node
                v_cmd = max(v_cmd - OUTER_EVERY * DT * decel_rate_v, 0.0)
            elif settled:
                v_cmd = min(v_cmd + OUTER_EVERY * DT * Vc / ttr, Vc)
                v_cmd = min(v_cmd, V + 2.5)
            refs, b_ref_prev, th_ref_prev = build_refs(
                sched, nmpc.N, nmpc.DT, OUTER_EVERY * DT, v_cmd, Vc, ttr, z0,
                decel=decel_on, decel_rate=decel_rate_v,
                flare=math.radians(flare_deg), beta_aft=math.radians(beta_aft_deg),
                b_slew=35.0, b_prev=b_ref_prev,
                th_slew=argf("--th_slew", 8.0), th_prev=th_ref_prev)
            hd = np.array([math.cos(heading), math.sin(heading), 0.0])
            x0 = [float(np.dot(p, hd)), float(p[2]), float(np.dot(v, hd)), float(v[2]),
                  theta, -float(w[1])]
            n_lat = int(round(COMP / (OUTER_EVERY * DT)))
            x0 = predict6(F6o, np.array(x0), obuf[-n_lat:] if n_lat > 0 else [], OUTER_EVERY * DT)
            t0 = time.perf_counter()
            try:
                u6, ok6 = nmpc.solve(np.array(x0), refs)
                if ok6:
                    Tf_, Tr_, b_, de_ = (float(u6[0]), float(u6[1]), float(u6[2]), float(u6[3]))
                    th_ = float(refs[0]["theta"])
                    b_, Tf_, Tr_ = tilt_clamp(b_, Tf_, Tr_, de_, th_, V,
                                              sched, math.radians(margin_deg))
                    pkg = (Tf_, Tr_, b_, de_, th_)
                    obuf.append(np.array([Tf_, Tr_, b_, de_])); obuf = obuf[-40:]
                else:
                    fails += 1
            except Exception as e:
                # Do NOT swallow this. A try/except around a numerical path once hid an
                # AttributeError for a whole iteration and the fix "appeared to do nothing".
                if fails == 0:
                    import traceback; traceback.print_exc()
                fails += 1
            solve_ms = (time.perf_counter() - t0) * 1000.0
            solves += 1

        # cross-track guidance, exactly as the node runs it: steer back onto the commanded track
        heading_cmd, e_ct = track_heading(x[0:2], target, track0, heading, k_ct, ct_max)
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
        worst_roll = max(worst_roll, abs(math.degrees(roll)))
        worst_yaw = max(worst_yaw, abs(math.degrees(yaw - heading)))

        if pkg is None or abs(math.degrees(roll)) > 50.0 or R[2, 2] < 0.4:
            mot, ail, elev, rud = safe_hover()
        else:
            Tf, Tr, beta_cmd, de, th_ref = pkg
            Tr = max(Tr, rear_floor)
            slew = math.radians(80.0) * DT
            beta_app += max(-slew, min(slew, beta_cmd - beta_app))
            vb_i = R.T @ x[13:16]
            mot, ail, elev, rud, ei_state = inner_step(
                R, w, th_ref, heading_cmd, beta_app, V, Tf, Tr, de, g, alloc,
                vb=vb_i, ei=ei_state, dt=DT)

        # each rotor sees the airspeed along ITS OWN thrust axis: the front pair tilts, the rear
        # pair stays vertical. At 90 deg tilt the front axis is FORWARD, so v_ax is the cruise
        # airspeed -- the worst case for a flat thrust model.
        vb_now = x[3:12].reshape(3, 3).T @ x[13:16]
        v_ax_f = float(vb_now[0] * math.sin(beta_app) + vb_now[2] * math.cos(beta_app))
        v_ax_r = float(vb_now[2])
        u = np.array([rotor_thrust(mot[0], v_ax_f, 0, DT), rotor_thrust(mot[1], v_ax_f, 1, DT),
                      rotor_thrust(mot[2], v_ax_r, 2, DT), rotor_thrust(mot[3], v_ax_r, 3, DT),
                      beta_app,
                      surf_apply(elev, 1, DT) * DE_MAX,
                      surf_apply(ail, 0, DT) * DA_MAX,
                      surf_apply(rud, 2, DT) * DR_MAX])
        # NOTE `mot` is now a THROTTLE produced by thrust_to_throttle(), and rotor_thrust()
        # turns it back into newtons with the same propeller model -- so the controller's
        # inversion and the plant's forward model are exercised against each other here.
        ubuf.append(u)
        u_app = ubuf[0] if len(ubuf) > lat_n else np.array(
            [so3.M * so3.G / 4.0] * 4 + [0.0, 0.0, 0.0, 0.0])
        if len(ubuf) > lat_n:
            ubuf.pop(0)

        xdot = np.array(F(x, u_app)).ravel()
        if gust != 0.0:
            xdot[14] += gust * math.sin(2.0 * math.pi * t / max(gust_T, 1e-3))
        x = x + DT * xdot
        Rn = x[3:12].reshape(3, 3)
        U, _, Vt = np.linalg.svd(Rn)                # re-orthonormalise (Euler integration drifts)
        x[3:12] = (U @ Vt).reshape(-1)

        if k % max(1, int(round(every / DT))) == 0:
            vb = x[3:12].reshape(3, 3).T @ x[13:16]
            aoa = math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-6))
            Fa = np.array(so3._AERO_NUM(vb, x[16:19], elev * DE_MAX,
                                        ail * DA_MAX, rud * DR_MAX)[0]).ravel()
            wing = 100.0 * float(Fa[2]) / (so3.M * so3.G)
            # ★ ACTUAL newtons, not sum(mot)*MAX_THRUST. `mot` is a THROTTLE now, and with
            # inflow the flat conversion over-reads badly: [0.86,0.86,0.30,0.30] reads as 41.9 N
            # when the rotors are really making ~9. A diagnostic that lies is worse than none.
            rot = (rotor_thrust(mot[0], v_ax_f) + rotor_thrust(mot[1], v_ax_f)
                   + rotor_thrust(mot[2], v_ax_r) + rotor_thrust(mot[3], v_ax_r))
            bc = math.degrees(pkg[2]) if pkg else 0.0
            _ect = e_ct
            sb = math.degrees(sched.at(v_cmd)["beta"])
            th_r = math.degrees(pkg[4]) if pkg else 0.0
            th_p = math.degrees(planned_theta(nmpc, 0.0, 4))
            print(f"  {t:6.1f} {x[2]:6.1f} {float(np.linalg.norm(x[13:16])):6.2f} "
                  f"{v_cmd:5.1f} {math.degrees(beta_app):5.1f} {bc:4.0f} "
                  f"{math.degrees(theta):+7.1f} {aoa:+6.1f} "
                  f"{math.degrees(roll):+6.1f} {math.degrees(yaw):+6.1f} {wing:7.1f} {rot:6.1f} "
                  f"{solve_ms:5.1f} ct={_ect:+6.1f}m ok={int(ok6)} thR={th_r:+5.1f} "
                  f"ail={ail:+5.2f} elv={elev:+5.2f} rud={rud:+5.2f} "
                  f"mot=[{','.join(f'{m:.2f}' for m in mot)}]")
        if kick_q != 0.0 and not _kicked[0] and t >= kick_at:
            x[17] += kick_q          # body-y rate; the node reads nose-up as -w[1]
            _kicked[0] = True
            print(f"  >> PITCH KICK {math.degrees(kick_q):+.0f} deg/s at t={t:.1f} "
                  f"(V={float(np.linalg.norm(x[13:16])):.1f})")
        if x[2] < 1.0:
            print(f"  !! GROUND at t={t:.1f}")
            break
        t += DT; k += 1

    R = x[3:12].reshape(3, 3)
    print(f"\n  final: z={x[2]:.1f} V={float(np.linalg.norm(x[13:16])):.2f} "
          f"tilt={math.degrees(beta_app):.1f} roll={math.degrees(math.atan2(R[2,1],R[2,2])):+.1f} "
          f"yaw={math.degrees(math.atan2(R[1,0],R[0,0])):+.1f}")
    print(f"  worst |roll| {worst_roll:.1f} deg   worst |yaw err| {worst_yaw:.1f} deg   "
          f"solve fails {fails}/{solves}")
    return 0


def main(args=None):
    """console_scripts entry point.

    Delegates to the SAME dispatch as `python3 nmpc_cascade_node.py`, so `ros2 run` and a
    direct invocation cannot drift apart -- including the offline gate flags,
    which are how this controller is validated before it is ever flown.
    """
    if "--test" in sys.argv:
        offline_test()
    else:
        run_ros()



if __name__ == "__main__":
    main()

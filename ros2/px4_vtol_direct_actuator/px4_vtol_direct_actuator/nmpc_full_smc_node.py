#!/usr/bin/env python3
"""FULL-STATE NMPC (19-state, 6-DOF) + OUTER POSITIONAL SMC  ->  90 deg wing-borne cruise.

WHY: the 6-state longitudinal NMPC (nmpc6_cruise_node) cracked the rotor->wing corridor by
co-optimizing the PITCH MOMENT, but its lateral axes were a hand-rolled PD outside the solver --
which is fragile exactly at 90 deg (front rotors horizontal -> differential thrust makes YAW not
roll; rear unloaded -> no roll authority) and a combined roll+yaw disturbance still diverged.
This node keeps the pitch-moment insight but puts the FULL attitude (roll, pitch, yaw), the tilt
and the body rates INSIDE one NMPC over the sim-exact 6-DOF plant, so the solver allocates the
four rotors + three surfaces against ALL the coupled moments simultaneously.

ARCHITECTURE
  outer:  positional SMC (smc_outer.SMCOuter, or an external smc_outer_node feeding
          .../reference_smc) -> a converging position set-point + velocity feed-forward
  inner:  this NMPC, state x(19) = [pos, R(row-major), beta, vel(inertial), omega(body)],
          control u(8) = [T_FL, T_FR, T_RL, T_RR, betacmd, de, da, dr], plant = so3.build_ode()
          (byte-identical to the sim).
  ref:    nmpc6.Schedule.at(V) = FSCTransitionProfile trim + PITCH-BALANCED front/rear split
          (My_thrust + My_aero = 0) + SPEED-GATED pitch, wrapped into a full attitude
          Rr = Rz(heading) @ Ry(theta) (roll = 0, yaw = heading -> lateral is TRACKED, not patched).

The 5 keys carried over from nmpc6 (all necessary there, kept here):
  (1) full moment dynamics in the model      (2) STATE bounds on the stiff rates
  (3) reference-feedforward fallback on solve failure (a frozen control diverges)
  (4) SOFT tilt guide (the NMPC self-limits through the corridor, then commits)
  (5) delay compensation (predict the state forward by the loop latency)
"""
import inspect
import math
import os
import sys

import numpy as np
import casadi as ca

import nmpc_so3_transition_node as so3          # sim-exact 19-state plant + constants
from nmpc6_cruise_node import Schedule, DE_MAX   # validated wing-borne trim schedule

M = so3.M; G = so3.G; MAX_THRUST = so3.MAX_THRUST
NX = so3.NX; NU = so3.NU                         # 19, 8
DA_MAX = so3.SO3TransitionNMPC.DA_MAX; DR_MAX = so3.SO3TransitionNMPC.DR_MAX


def _cl(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# Wing reference area and the EFFECTIVE peak lift coefficient of the sim's blended
# (linear -> flat-plate) curve.  CL peaks at alpha = 10.0 deg at 0.790; see aoa_lim.
# How far the FRONT pair may lean AFT (rad).  Gives a decelerating force with the body LEVEL,
# so the back-transition does not have to buy braking with a zoom-climb.
BETA_MIN = math.radians(20.0)
# Above this airspeed the wing carries the weight (stall is 14.13 m/s at the CL peak).
WINGBORNE_V = 14.5
# ★ How much of the altitude-holding cost survives THROUGH THE CORRIDOR (0..1).
# Below stall speed the wing cannot carry the aircraft, so forward force must come from tilt
# (which costs vertical thrust) or from pitching down (which costs altitude) -- neither is free.
# Holding altitude hard through that region is spending thrust on the one thing the wing will
# do for free a few seconds later, and it is what kept the tilt trailing the speed schedule.
# Fade it here, restore it for cruise (the validated cruise held +-0.3 m and we keep that).
ALT_AUTH_TRANS = 0.25
# ★ THE COMMITTED TRANSITION TILT.  Ramp to it, HOLD it while the speed builds, then go to 90.
TILT_HOLD_DEG = 45.0     # at 45 deg the front pair makes 25.5 N forward = 7.4 m/s^2
TILT_HOLD_V = 4.0        # ramp the hold in over 0..this airspeed (never full tilt at V=0)
TILT_COMMIT_V = 11.0     # above this, blend the held tilt to 90 deg
TILT_COMMIT_BAND = 5.0   # width [m/s] of that blend -- must be slower than the servo
# The APPLIED-tilt path (apply_tilt_limits).  ONE source of truth, so the offline test flies the
# same limiting the aircraft does -- see apply_tilt_limits for why that matters.
TILT_MARGIN_DEG = 15.0
TILT_LPF_TAU = 0.6
# ★ EXTERNAL RATE LIMITS ARE OFF BY DEFAULT -- THEY FIGHT THE SOLVER'S OWN MODEL.
# so3.build_ode models the tilt servo as an 80 deg/s rate limit (finding C10: modelling it as a
# lag WITHOUT the rate limit let the solver believe it could swing the nacelles arbitrarily fast).
# Imposing 15 deg/s plus a 0.6 s low-pass OUTSIDE that model recreates exactly the mismatch C10
# fixed, in the opposite direction: the solver plans a tilt it will not get, the plant lags, the
# solver corrects, and the aircraft bounces. The same argument applies to the surfaces, which have
# no servo model at all. Each limiter was added to damp a specific oscillation and together they
# ADD phase lag to a loop that already penalises control rate through w_du.
# Set these > 0 only with a matching change to the model the solver predicts with.
TILT_SLEW_DEG_S = 15.0
# Commanded nose-DOWN pitch through the corridor, and the sink that goes with it.
# Holding altitude below stall speed forces the solver to buy lift with AoA, and past
# the 10 deg CL peak that costs both lift and drag.
# ★ OFF BY DEFAULT -- the commanded dive is NOT validated and made things worse on every
# metric the offline test can measure: identical time to stall speed (37.5 s either way),
# max pitch 27 -> 60 deg, and more altitude lost. The DIAGNOSIS behind it is sound (in HIL
# the altitude is pinned at 29.0-29.5 m while pitch runs +15 and AoA +11..+15, past the
# 10 deg CL peak, and it departs at 9.7 m/s) -- but the offline plant shows AoA 11 where
# HIL shows 15, so it cannot validate a fix for this, and tuning against it is guesswork.
# Set dive_deg > 0 to experiment in HIL, where the defect actually exists.
DIVE_DEG = 0.0
# trans_sink stays at the VALIDATED 0.5. Raising it to 2.0 alongside the reduced altitude
# authority made the aircraft descend 19 m offline (20 -> 1.2 m), straight through the 8 m
# budget floor -- with alt_auth at 0.25 the solver does not defend that floor. The dive
# lowers AoA by pointing the nose down, which is the actual fix; commanding a big sink as
# well just gives the altitude away twice.
TRANS_SINK = 0.5
DESCENT_BUDGET = 8.0
PITCH_GATE_V = 12.0      # scale commanded pitch by (V/this)^2 -> level while the tilt accelerates
PITCH_SLEW_DEG_S = 3.0
WING_S = 0.35
CL_PEAK = 0.790


def _alt_authority(V, v_wb=WINGBORNE_V, a_trans=ALT_AUTH_TRANS, settled=True):
    """Vertical-cost scale (0..1) as a function of airspeed.

    1.0 in hover (we must hold the hover), fading to `a_trans` across the corridor, and back to
    1.0 once wing-borne.  Shaped on SPEED because that is what decides whether the wing can carry
    the weight.  Ramped, never stepped -- a step in a cost weight shows up as a jump in the
    commanded thrust.
    """
    if not settled or a_trans >= 1.0:
        return 1.0
    lo = min(1.0, max(0.0, (V - 2.0) / 3.0))              # 0 below 2 m/s -> 1 by 5 m/s
    hi = min(1.0, max(0.0, (V - (v_wb - 2.0)) / 2.0))     # 0 until wb-2  -> 1 at wing-borne
    f = lo * (1.0 - hi)                                   # 1 only in the middle (the corridor)
    return 1.0 + (a_trans - 1.0) * f


def _aoa_gated(xk):
    """Angle of attack, faded out at low airspeed -- as a CONSTRAINT quantity.

    ★ WHY A CONSTRAINT AND NOT A COST.  The AoA guard has always been a soft penalty, and in
    flight it does not hold: bag vtol_20260910_093744 ran AoA +11..+15 through the whole corridor
    against a 10 deg limit, past the CL peak where lift FALLS and drag climbs -- which is why the
    aircraft needed 30 s to reach 9.7 m/s and was still in the lateral gap when it departed.
    Raising the weight is not available: w_aoa=2.0 made every solve fail from 6 m/s (bag
    full_tr13), because the penalty's gradient explodes in the tail.
    The pitch limit had exactly this history and was solved by moving from a cost wall to a BOUND,
    which an interior-point method handles natively at no cost in conditioning. AoA is not a state,
    so it becomes a nonlinear inequality rather than a box -- same idea.
    The speed gate is folded in so the constraint relaxes automatically below a few m/s, where the
    angle of attack is meaningless noise and a hard bound would be nonsense.
    """
    R = ca.reshape(xk[3:12], 3, 3).T
    vb = ca.mtimes(R.T, xk[13:16])
    alpha = ca.atan2(-vb[2], ca.sqrt(vb[0] * vb[0] + 1.0))
    # ★ A SHARP GATE.  The cost's gate is u^2/(u^2+4), which is 0.69 at 3 m/s -- as a CONSTRAINT
    # that silently relaxes the limit to 12/0.69 = 17.4 deg through the whole corridor, exactly
    # where it has to bite. Measured: with the soft gate the aircraft still reached AoA 16.5.
    # u^2/(u^2+0.25) is 0.94 by 2 m/s, so the bound is honest above walking pace and still fades
    # to nothing at hover, where the angle of attack is meaningless noise.
    gate = (vb[0] * vb[0]) / (vb[0] * vb[0] + 0.25)
    return gate * alpha


def _aoa_excess(xk, lim):
    """How far this stage's angle of attack lies beyond `lim` (zero inside it).

    STANDARD AoA, nose-up positive: alpha = atan2(-w_flu, |u_flu|) on the body velocity
    v_body = R^T v_world. The sqrt smooths |u| so IPOPT sees a differentiable function and
    keeps it finite at hover, and the forward-speed gate fades the guard out below a couple
    of m/s where the angle of attack is meaningless noise.
    """
    R = ca.reshape(xk[3:12], 3, 3).T
    vb = ca.mtimes(R.T, xk[13:16])
    alpha = ca.atan2(-vb[2], ca.sqrt(vb[0] * vb[0] + 1.0))
    gate = (vb[0] * vb[0]) / (vb[0] * vb[0] + 4.0)
    # SMOOTH, no kinks. The obvious `fmax(0, fabs(alpha) - lim)` has a corner exactly at the
    # limit -- which is where corridor trajectories ride -- and `fabs` has a second corner at
    # alpha = 0, which the vehicle crosses constantly. This project has already paid for that
    # once: a piecewise post-stall break caused IPOPT convergence failures at precisely the
    # tilt angles the transition passes through. An even power is smooth everywhere.
    #
    # EXPONENT 4, NOT 6 (bag full_tr13).  The ^6 form blew up in the FAR TAIL: at alpha=25 deg
    # it reaches 244, and raising w_aoa to make it hold at the limit made the gradient so
    # large that IPOPT could not converge inside MAX_ITER=25 -- every solve failed from 6 m/s
    # onward.  ^4 reaches only 39 at 25 deg, so with w_aoa=1.5 the guard is 3x STRONGER at the
    # limit (1.5 vs 0.5) where it has to hold the CL peak, while the gradient out at 25 deg is
    # ~3x SMALLER than the ^6/2.0 form that broke the solver.  Stronger where it matters,
    # cheaper where it was killing convergence.
    return gate * (alpha / lim) ** 4


def _energy_split(xk, r):
    """Specific TOTAL ENERGY and ENERGY DISTRIBUTION errors vs the reference, both normalised.

    A phugoid is an energy-CONSERVING exchange: potential <-> kinetic at roughly constant total
    energy.  Tracking altitude and speed SEPARATELY cannot see it, because that cost is diagonal --
    it penalises dz and dV independently and never their product, so the exchange is free.  Bag
    full_tr20 porpoised exactly this way: altitude 63-90 m, speed 7-20 m/s, vz +-7.5 m/s, only 13%
    of samples level, while the vehicle happily held 90 deg tilt through all of it.

    Rotating the same two errors into
        E = g*dz + 0.5*d(V^2)     (total energy   -- what THRUST controls)
        B = g*dz - 0.5*d(V^2)     (distribution   -- what PITCH/tilt controls)
    introduces the missing cross term.  Through a zoom (dz>0, dV<0) and a dive (dz<0, dV>0) E stays
    near zero while B swings hard, so penalising B^2 is a DIRECT phugoid penalty -- this is the
    TECS decomposition.  Both are smooth quadratics, so unlike the ^6 AoA term (bag full_tr13) they
    cost the solver nothing in conditioning.
    """
    g_ = so3.G
    dz = xk[2] - r[2]
    V2 = xk[13] ** 2 + xk[14] ** 2 + xk[15] ** 2
    V2r = r[13] ** 2 + r[14] ** 2 + r[15] ** 2
    dK = 0.5 * (V2 - V2r)
    scale = g_ * 10.0                      # normalise on 10 m of height -> both terms are O(1)
    E = (g_ * dz + dK) / scale
    B = (g_ * dz - dK) / scale
    return E * E, B * B


def R_ref(heading, theta):
    """Reference attitude: yaw=heading, pitch=theta NOSE-UP positive, roll=0.

    The docstring said nose-up and the matrix built Ry(+theta), which is nose-DOWN -- the
    fifth copy of the inversion fixed elsewhere on 2026-09-06, and the one that was missed.
    It cancelled against the old inverted aero model, so it looked right. HIL bag `full_tr1`:
    the vehicle accelerated normally to 4.2 m/s, then pitched nose-UP 10 -> 25 -> 77 deg in
    1.5 s and departed. The controller's own log printed `pitch=-77` where the truth was
    +87: it was measuring pitch in the OLD convention while the trim schedule fed it a
    reference in the NEW one, so past a few degrees the attitude error changed sign and the
    loop drove the divergence instead of arresting it.
    """
    cy, sy = math.cos(heading), math.sin(heading)
    ct, st = math.cos(theta), math.sin(theta)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[ct, 0.0, -st], [0.0, 1.0, 0.0], [st, 0.0, ct]])
    return Rz @ Ry


class FullNMPC:
    """19-state 6-DOF NMPC. Tracks pos/vel/attitude/tilt from the wing-borne schedule."""
    # SIZED FOR REAL-TIME UNDER HIL LOAD (bag full2): N=12/iter60 solved in 22 ms on an IDLE
    # machine but 48-96 ms with Isaac+PX4+GS competing for CPU -- over the 67 ms budget at 15 Hz,
    # with ok=0 solve failures -> the loop falls behind -> tumble. N 12->8, DT .08->.10 (same ~0.8 s
    # horizon), MAX_ITER 60->25. Real-time margin beats horizon length on this plant.
    # HORIZON 0.8 s -> 2.0 s (2026-09-07). At ~10 m/s the phugoid period is ~4 s, so a 0.8 s
    # horizon covered a FIFTH of the mode the controller was exciting: it optimised through a
    # fraction of a cycle and was surprised by the rest, which is the pitch/speed limit cycle
    # seen in every bag. N is unchanged so the problem SIZE is identical -- this isolates the
    # horizon length as the variable. NRK4 1 -> 2 because a single RK4 step across 0.25 s is
    # too coarse for the stiff attitude dynamics (0.125 s substeps instead).
    # 0.8 s -> 1.6 s. At ~10 m/s the phugoid period is ~4 s, so the old horizon covered a
    # FIFTH of the mode the controller was exciting -- it optimised through a fraction of a
    # cycle and was surprised by the rest, which is the pitch/speed limit cycle in every bag.
    # N and NRK4 are unchanged, so the symbolic expression is the SAME SIZE: this isolates
    # horizon length as the variable AND keeps the JIT build at its ~5 min baseline.
    # (DT 0.25 with NRK4 2 was tried first for integration accuracy and did not finish
    #  compiling in 15 min -- doubling the unrolled RK4 more than doubles the build.)
    # HORIZON: the phugoid is ~4 s and a 1.6 s horizon cannot see a full period of it, so the
    # solver cannot anticipate the mode it is exciting (bag full_tr20 porpoised 63-90 m).
    # DT 0.20 -> 0.30 buys 2.4 s at the SAME symbolic size (~4 min JIT build); DT 0.25 with
    # NRK4=2 does NOT build inside 15 min.  RK1 at 0.30 s is a coarser integration -- if the
    # offline test degrades, this is the first thing to put back.
    # ★ DT MUST STAY 0.20 AT NRK4=1 (offline gate, 2026-09-07).  DT=0.30 buys a 2.4 s horizon
    # against the ~4 s phugoid, but RK1 at 0.30 s is too coarse for the vertical dynamics: the
    # offline test drifted from z=20 to z=50 m (it had been dead flat at 20.0) and the final solve
    # returned ok=0.  A horizon the solver cannot integrate accurately is worse than a short one.
    # The correct version is DT=0.25 with NRK4=2, which does NOT build inside 15 minutes here.
    N = 8; DT = 0.20; NRK4 = 1
    # w_beta 12 -> 60 (2026-09-07, HIL bags full_tr1/full_tr2). A SOFT tilt guide leaves a
    # nearly flat tilt-vs-pitch optimum, and the solver hunts along it: the applied tilt went
    # 17 -> 5 -> 24 -> 13 deg in successive steps while the reference sat at 21-24, and the
    # pitch departed with it. vtol_nmpc raised w_beta for exactly this reason and the change
    # was never back-ported here (audit finding C9, the back-port gap). The AIRSPEED TILT
    # CLAMP owns corridor safety, so the guide does not need to be soft.
    w_pos = 25.0; w_vel = 30.0; w_att = 150.0; w_beta = 60.0
    # Dedicated yaw-error weight -- see the cost.  Sized so a 14 deg yaw error costs about the
    # same as a 1 m position error (sin(14)^2 * 200 = 11.7 vs w_pos * 1 = 25), rather than 4.5.
    # SWEPT offline against an injected 1 N.m corridor yaw disturbance (--dist_yaw):
    #     w_yaw =   0  ->  47 deg peak yaw
    #     w_yaw = 200  ->  33 deg
    #     w_yaw = 600  ->  26 deg      (only 20% better for 3x the weight)
    # 200 takes most of the benefit.  A constant disturbance cannot be zeroed by a proportional
    # cost at all -- the residual is where the yaw gradient balances the moment -- so chasing it
    # with weight has diminishing returns, and this project has twice been bitten by a large
    # weight that solved fine offline and then failed every solve in HIL (w_aoa=2.0, full_tr13).
    w_yaw = 200.0
    # ★ PER-AXIS RATE DAMPING. w_om was ONE weight over all three body rates, so roll could not
    # be damped without equally damping pitch -- and pitch damping is what the corridor trim needs
    # to be free. The flown failure is a ROLL oscillation under a CONVERGING solver (bag
    # vtol_20260910_141622: roll +13 -> -24 -> +36 -> -41 with every solve ok=1), which is the
    # signature of a marginally-damped loop pushed unstable by loop latency -- and it does NOT
    # reproduce offline, where the same yaw disturbance gives roll of only 7-8 deg. Damping the
    # roll rate specifically is the targeted lever; this was the documented "next step" left over
    # from the SO(3) campaign.
    # ★ MEASURED CLIFF -- and the central constraint on this controller.
    #     roll damping 12 -> AoA 15.4, pitch 23,   7/200 solve failures
    #     roll damping 20 -> AoA 15.1, pitch 23,   8/200   (no benefit, no cost)
    #     roll damping 40 -> AoA 10.0, pitch 14, 110/200   (real benefit, solver breaks)
    # There is no useful value in between. This is the FOURTH cost-stiffness change to behave
    # exactly this way (w_aoa=2.0, the AoA inequality constraint, w_u=0.5, and this): each
    # IMPROVES the physics and saturates IPOPT at MAX_ITER=25. More iterations are not available
    # -- 40 pushed solve p95 to 93 ms against a 100 ms budget and the loop began slipping.
    # So the binding constraint is COMPUTE, not tuning: buying better lateral behaviour requires
    # a cheaper solve first (shorter horizon is the obvious lever, though N=8/DT=0.20 was itself
    # what broke the 10.5 m/s plateau, so it is not free either).
    w_om_p = 12.0                     # ROLL rate

    w_om_q = 12.0                     # pitch rate -- unchanged
    w_om_r = 12.0                     # yaw rate   -- unchanged
    w_om = 12.0                       # kept for the cache key / back-compat
    w_bcmd = 25.0                     # tilt COMMAND vs its reference -- see the cost below
    # 12, NOT 60.  At 60 the wall is too stiff for IPOPT inside MAX_ITER=25: every solve
    # returned ok=0 and the offline run departed (roll 25 deg, yaw 164, altitude 20 -> 32.5 m).
    # Same failure as w_aoa=2.0 in bag full_tr13 -- an over-weighted even power is unsolvable,
    # not merely firm.  12 still charges 12x at the limit versus the cruise trim and rises 3x
    # again by 20 deg, which is a wall the solver can actually respect.
    w_pitch = 0.0                     # OFF: the pitch limit is a box bound, not a penalty
    PITCH_MAX_DEG = 15.0              # the transition may not pitch beyond this
    # ANGLE-OF-ATTACK GUARD (2026-09-07). The measured reason this controller plateaus at
    # ~8.5 m/s across four different tunings: the AoA swings to +-46 deg against a 13 deg
    # stall break. Every excursion collapses CL, dumps the wing's share back onto the rear
    # rotors, breaks the pitch trim and bleeds the speed it just built -- so it never reaches
    # the ~14 m/s the wing needs to carry the aircraft. Nothing in the cost knew what AoA was.
    # This is the aerodynamic twin of the AIRSPEED TILT CLAMP: a corridor-safety term.
    # SOFT rather than a hard constraint -- an infeasible NLP on a transient would drop into
    # the solve-failure path, which is the dangerous one -- but steep enough to bite: at
    # 30 deg it contributes ~365, comparable to a 3 m/s velocity error.
    # The wing's EFFECTIVE CL peaks at alpha ~= 10.3 deg (CL 0.79) and has
    # collapsed to ~0.33 by 25 deg -- stall_alpha_rad is 13 deg and the sigmoid
    # blend to the flat plate starts biting well before it.  So aoa_lim sits ON
    # the CL peak deliberately.  In the corridor the rotors alone cannot hold
    # the weight (72*cos(65 deg) = 30 N < 34 N), so altitude depends on the wing
    # sitting exactly on that peak -- a knife edge.  w_aoa=0.5 was too soft to
    # hold it: bag full_tr12 departed to alpha=25 deg, lift halved, it sank, and
    # the sink raised alpha further (descending at speed tilts the flight path
    # down).  2.0 makes the limit a wall rather than a suggestion.
    # Paired with the ^4 exponent (see _aoa_excess).  w_aoa=2.0 on the OLD ^6 form
    # was a regression (bag full_tr13): every solve returned ok=0 from V=6 m/s
    # onward, 50-80 ms against a 67 ms budget, node fell back to safe_hover, the
    # vehicle stalled and tumbled.  The fix is the exponent, not the weight --
    # ^4 at 1.5 holds 3x harder at the limit with ~3x less gradient in the tail.
    w_aoa = 1.5          # x (alpha/aoa_lim)^4 -- see _aoa_excess
    aoa_lim = math.radians(10.0)
    # w_u stays LOW. Raising it 0.02 -> 0.5 to make lateral asymmetry expensive was tried and
    # measured WORSE: roll unchanged (8.0 vs 7.0), yaw slightly worse, and solve failures went
    # 7/200 -> 114/200. Penalising deviation from the trim that hard over-constrains the solver.
    w_u = 0.02; w_de = 1.0; w_du = 4.0; w_surf = 3.0   # w_surf: da/dr WERE unpenalized
    # TECS phugoid damping (see _energy_split).  w_bal is the one that damps the porpoise;
    # w_ene merely holds total energy on the reference. Set both to 0 to restore the old cost.
    # ★ OFF (bag full_tr22).  The TECS decomposition is the RIGHT tool for a phugoid and the
    # WRONG one for this vehicle, because the ROLLING TRANSITION IS ITSELF a potential->kinetic
    # exchange -- exactly what B = g*dz - 0.5*d(V^2) charges for.  In principle a perfectly
    # tracked reference gives B=0 (the reference descends and accelerates too), but the vehicle
    # LAGS its reference through the dive-out, so B is large and negative during the one
    # manoeuvre that crosses the corridor.  Result: it never crossed -- peak tilt 73 deg (vs 90),
    # wing 61% (vs 238%), speed 12.4 (vs 22.8).  Damping the phugoid this way also damps the
    # transition; they are the same physics.  Set > 0 only with a reference the vehicle can track
    # tightly enough that B stays near zero through the corridor.
    # (Earlier note kept: a "30% failure at hover" scare was a bad TEST -- activating this node
    #  while the SMC destination is the CURRENT position asks it to hold station AND accelerate.
    #  Against a consistent far target the same solver failed 1/24. Measure solver health ONLY
    #  against a reference the vehicle could actually fly.)
    w_ene = 0.0; w_bal = 0.0
    # ★ VERTICAL-RATE DAMPING -- the targeted alternative to TECS.  The phugoid shows up as vz
    # swinging +-7.5 m/s (bag full_tr20) about a reference that commands vz = -trans_sink during
    # the corridor and 0 afterwards.  Penalising (vz - vz_ref)^2 therefore charges ONLY for the
    # oscillation, never for the commanded descent -- which is exactly where the TECS balance term
    # went wrong (full_tr22: it charged for the dive-out itself and the vehicle never crossed).
    # vz is already inside w_vel; this adds weight to that ONE component. Plain quadratic, so it
    # costs nothing in conditioning.
    # BRACKETED by two flights, not guessed:
    #   w_vz=0 (full_tr20): 41.7 s episodes but a 27 m phugoid (alt 63-90 m)
    #   w_vz=6 (full_tr23): phugoid damped 3.5x to 7.6 m (alt 103.0-110.6) but the episode
    #                       collapses after 14.7 s -- holding altitude that rigidly stops the
    #                       vehicle trading height for speed, so V decays below the 14.13 m/s
    #                       stall and it drops out into an 8 m/s mush.
    # 2.5 aims between them: enough to damp the porpoise, loose enough to keep flying speed.
    w_vz = 2.5
    w_posT = 40.0; w_attT = 200.0
    # ★ OFF (0). The AoA CONSTRAINT WORKS BUT DOES NOT FIT THE COMPUTE BUDGET.
    # It does what it was designed to do -- offline it held AoA at 12.1 deg where the soft cost
    # let it reach 15.3, and cut peak pitch from 23 to 15 -- which is exactly the defect seen in
    # flight (AoA +11..+15 past the 10 deg CL peak). But it makes the NLP much harder:
    #     MAX_ITER=25  ->  110/200 solves fail (loose acceptance does not help: also 110/200)
    #     MAX_ITER=50  ->  offline fine, but 91-149 ms per solve in HIL
    # and restart_hil.sh runs the node at 15 Hz, so the budget is 67 ms. That is a 2x overrun,
    # and an overrunning loop is a documented tumble cause here -- rep1 departed with yaw
    # running -27 -> -67 -> -79 while motors saturated.
    # Set aoa_max_deg > 0 (and raise MAX_ITER) only if the solve is first made cheaper -- a
    # shorter horizon is the obvious lever, though N=8/DT=0.20 was itself hard-won.
    AOA_MAX_DEG = 0.0
    # 50, not 25: the AoA CONSTRAINT makes the NLP genuinely harder, and at 25 every
    # failure was Maximum_Iterations_Exceeded (110/200). At 50 it is 5/200.
    # 25. Raising it to 40 was tried and REVERTED: it pushed solve p95 to 93 ms against a
    # 100 ms budget (LOOP IS SLIPPING, 7% late) and did not prevent the departure. The failure
    # split proves why -- before the aircraft lost control only 2 of 48 ticks failed (4%), after
    # it 13 of 29 (44%). The solver was CONVERGING while the aircraft departed, so
    # Maximum_Iterations_Exceeded is a CONSEQUENCE of the upset, not its cause, and buying more
    # iterations buys nothing.
    MAX_ITER = 25
    _HOLD_TICKS = 4      # consecutive failed solves to hold the last converged control
    USE_JIT = True

    def __init__(self):
        F = so3.build_ode()
        self._sin_pitch_max = math.sin(math.radians(self.PITCH_MAX_DEG))
        self.pitch_max_rad = math.radians(self.PITCH_MAX_DEG)   # live-tunable, see the node params
        N = self.N
        X = ca.SX.sym("X", NX, N + 1); U = ca.SX.sym("U", NU, N)
        # params: x0(19) + xref(19)*(N+1) + uref(8)*N + uprev(8) + alt_auth(1)
        P = ca.SX.sym("P", NX + NX * (N + 1) + NU * N + NU + 1)
        x0 = P[0:NX]
        xr = lambda k: P[NX + NX * k: NX + NX * (k + 1)]
        ubase = NX + NX * (N + 1)
        ur = lambda k: P[ubase + NU * k: ubase + NU * (k + 1)]
        uprev = P[ubase + NU * N: ubase + NU * N + NU]
        # ★ ALTITUDE AUTHORITY, as a live solver PARAMETER (0..1).
        # Below the 14.13 m/s stall speed the wing cannot carry the weight, so every newton the
        # solver spends holding altitude is a newton not spent crossing the corridor.  w_pos and
        # w_vel lumped x, y and z together, so altitude could only be de-prioritised by giving up
        # along-track tracking too.  Splitting them lets the VERTICAL terms alone be faded through
        # the corridor and restored for cruise -- where the validated run held +-0.3 m and we want
        # to keep that.  A parameter, not a constant, so it is tunable in flight with no rebuild.
        alt_auth = P[ubase + NU * N + NU]
        J = 0; g = [X[:, 0] - x0]; g_aoa = []
        for k in range(N):
            xk = X[:, k]; uk = U[:, k]; r = xr(k); ru = ur(k)
            J += self.w_pos * (ca.sumsqr(xk[0:2] - r[0:2])
                               + alt_auth * (xk[2] - r[2]) ** 2)
            J += self.w_vel * (ca.sumsqr(xk[13:15] - r[13:15])
                               + alt_auth * (xk[15] - r[15]) ** 2)
            # SO(3) attitude error 0.5*trace(I - Rr^T R): smooth, no singularity, covers roll+yaw+pitch
            Rk = ca.reshape(xk[3:12], 3, 3).T; Rrk = ca.reshape(r[3:12], 3, 3).T
            J += self.w_att * 0.5 * (3.0 - ca.trace(ca.mtimes(Rrk.T, Rk)))
            # ★ YAW NEEDS ITS OWN TERM.  The SO(3) trace cost is w_att*(1-cos psi): at a 14 deg
            # yaw error that is 0.03*150 = 4.5, while a 1 m position error costs w_pos = 25.  So
            # yaw was worth less than a metre of position and the solver correctly spent its
            # effort elsewhere.  MEASURED in flight (bag vtol_20260909_215829): yawing at
            # -14.4 deg/s the commanded rudder was -0.04 of a +-0.52 range -- 8% of the authority
            # available -- and the front differential was 0.04.  The aircraft was not short of
            # yaw authority; nothing was asking for it.  1-cos is quadratically FLAT at small
            # angles, which is exactly where the corridor needs the correction to start.
            # The z component of the SO(3) error vector 0.5*vee(Rr'R - R'Rr) is sin(psi) --
            # LINEAR at small angles -- so this term has gradient where the trace term has none.
            Ek = ca.mtimes(Rrk.T, Rk)
            J += self.w_yaw * (0.5 * (Ek[1, 0] - Ek[0, 1])) ** 2
            J += self.w_beta * (xk[12] - r[12]) ** 2
            J += (self.w_om_p * xk[16] ** 2 + self.w_om_q * xk[17] ** 2
                  + self.w_om_r * xk[18] ** 2)
            J += self.w_aoa * _aoa_excess(xk, self.aoa_lim)
            e2, b2 = _energy_split(xk, r)
            J += self.w_ene * e2 + self.w_bal * b2
            J += self.w_vz * alt_auth * (xk[15] - r[15]) ** 2
            J += self.w_u * ca.sumsqr(uk[0:4] - ru[0:4]) + self.w_de * (uk[5] - ru[5]) ** 2
            # ★ COST THE TILT *COMMAND*, NOT JUST THE TILT STATE.
            # w_beta constrains x[12] (the tilt STATE) against the schedule, but u[4] -- the tilt
            # COMMAND -- had no cost against its reference at all: only w_du (a rate penalty
            # between horizon steps) touched it.  Any command that lands the state near the
            # schedule was equally good, so with MAX_ITER=25 (not converged to the same answer
            # twice) the command jittered between solves.  That is the roughness in the applied
            # tilt; the slew limit only caps how fast the jitter propagates, it does not remove
            # the jitter.  Anchoring the command to the reference makes consecutive solves agree.
            J += self.w_bcmd * (uk[4] - ru[4]) ** 2
            # ★ HARD PITCH LIMIT.  The state carries R row-major in x[3:12], so R[2][0] = x[9]
            # and sin(theta) = x[9] with theta NOSE-UP positive.  Nothing previously bounded the
            # pitch at all: w_att pulls the whole attitude toward the reference, but the solver
            # was free to pitch as far as it liked in between.  An even power gives a smooth wall
            # -- 1.0 at the limit, 3x at 20 deg, 14x at 30 -- with no corner for IPOPT to trip on
            # (the ^6 form was too stiff to solve, bag full_tr13; ^4 is the one that works).
            # (pitch is limited by an adaptive BOX BOUND at solve time -- see solve().  A cost
            #  wall here was tried at w_pitch=60 and made every solve fail; the bound is free.)
            J += self.w_pitch * (xk[9] / self._sin_pitch_max) ** 4   # w_pitch defaults to 0
            # ★ THE SURFACE PENALTY MUST FADE WITH TILT (bag full_tr36).  A CONSTANT penalty on
            # aileron/rudder is right in hover -- the rotors own roll and yaw there, and without
            # it the rudder pinned at +1.0 (full_tr13).  But at 90 deg tilt the FRONT pair points
            # forward, so differential thrust makes YAW not roll, and the REAR pair is unloaded to
            # ~5 N: the surfaces are the ONLY lateral authority left.  Penalising them there
            # suppresses exactly the control the aircraft needs, which is how a small yaw grew
            # into a 21 deg bank, an 85 m excursion and finally a circle.
            # cos^2(beta) is 1 in hover and 0 at 90 deg -- the penalty lifts as the rotors lose
            # their roll authority.  A floor keeps a little damping so the surfaces cannot chatter.
            surf_fade = self.w_surf * (0.15 + 0.85 * ca.cos(xk[12]) ** 2)
            J += surf_fade * (uk[6] ** 2 + uk[7] ** 2)
            up = uprev if k == 0 else U[:, k - 1]
            J += self.w_du * ca.sumsqr(uk - up)
            g.append(X[:, k + 1] - so3.rk4(F, xk, uk, self.DT, self.NRK4))
            if k > 0 and self.AOA_MAX_DEG > 0:   # stage 0 is pinned to x0 by the equality
                g_aoa.append(_aoa_gated(xk))
        rN = xr(N)
        RN = ca.reshape(X[3:12, N], 3, 3).T; RrN = ca.reshape(rN[3:12], 3, 3).T
        J += self.w_posT * (ca.sumsqr(X[0:2, N] - rN[0:2])
                            + alt_auth * (X[2, N] - rN[2]) ** 2)
        J += self.w_attT * 0.5 * (3.0 - ca.trace(ca.mtimes(RrN.T, RN)))
        n_eq = int(ca.vertcat(*g).shape[0])
        self.n_eq = n_eq; self.n_aoa = len(g_aoa)
        gv = ca.vertcat(*g, *g_aoa) if g_aoa else ca.vertcat(*g)
        z = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        nlp = {"x": z, "f": J, "g": gv, "p": P}
        opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.max_iter": self.MAX_ITER,
                # ★ DO NOT LOOSEN ACCEPTANCE. Tried acceptable_tol 1e-4 -> 1e-2 with acceptable_iter
                # 6 -> 3 to cut iterations for the AoA constraint. It did NOT reduce the iterations
                # needed (110/200 failures at MAX_ITER=25 either way) and it quietly destroys the
                # SOLUTION: in flight the solves all "succeeded" (1/120 failures) while the aircraft
                # never transitioned at all -- peak speed 3.71 m/s, peak tilt 9.4 deg, wing share
                # 3.9%. A solver that stops as soon as an iterate is barely acceptable returns
                # control too weak to fly the manoeuvre, and reports success while doing it.
                "ipopt.sb": "yes", "ipopt.acceptable_iter": 6, "ipopt.acceptable_tol": 1e-4,
                "ipopt.mu_strategy": "adaptive", "ipopt.warm_start_init_point": "yes"}
        if self.USE_JIT:
            opts.update({"jit": True, "compiler": "shell",
                         "jit_options": {"flags": ["-O2"], "verbose": False},
                         "jit_serialize": "embed"})
        self.iX = 0; self.iU = NX * (N + 1)
        import hashlib as _hl
        key = _hl.md5(repr([self.N, self.DT, self.NRK4, self.w_pos, self.w_vel, self.w_att,
                            self.w_beta, self.w_om, self.w_om_p, self.w_om_q, self.w_om_r, self.w_u, self.w_de, self.w_du,
                            self.w_posT, self.w_attT, self.w_surf, self.w_bcmd, self.w_yaw,
                            self.w_pitch, self.PITCH_MAX_DEG, self.MAX_ITER,
                            self.AOA_MAX_DEG, inspect.getsource(_aoa_gated),
                            # hash the COST BUILDER: changing the shape of J (not just a weight)
                            # must invalidate the cache, or the old solver is silently reused.
                            inspect.getsource(type(self).__init__),
                            self.w_aoa, float(self.aoa_lim),
                            inspect.getsource(_aoa_excess),
                            self.w_ene, self.w_bal, inspect.getsource(_energy_split), self.w_vz,
                            # The key MUST cover everything baked into the serialized solver,
                            # not just the weights: a cached solver is reloaded wholesale, so
                            # a model change with an unchanged key silently no-ops (audit C6).
                            inspect.getsource(so3.aero_body), inspect.getsource(so3.build_ode),
                            so3.M, so3.G, so3.IXX, so3.IYY, so3.IZZ, so3.MAX_THRUST, so3.CQ,
                            so3.TAU_BETA, so3.RPOS.tolist()]).encode()).hexdigest()[:10]
        cache = os.path.expanduser(f"~/.cache/nmpc_full_N{self.N}_DT{int(self.DT*1000)}_{key}.casadi")
        self.solver = None
        if self.USE_JIT and os.path.exists(cache):
            try:
                self.solver = ca.Function.load(cache); print(f"[nmpc-full] loaded cached solver: {cache}")
            except Exception:
                self.solver = None
        if self.solver is None:
            print("[nmpc-full] building solver (JIT ~2-4 min the first time)...")
            self.solver = ca.nlpsol("nmpcfull", "ipopt", nlp, opts)
            if self.USE_JIT:
                try:
                    os.makedirs(os.path.dirname(cache), exist_ok=True); self.solver.save(cache)
                    print(f"[nmpc-full] cached: {cache}")
                except Exception as e:
                    print(f"[nmpc-full] cache save failed ({e})")
        # bounds
        lbx = -1e9 * np.ones(z.shape[0]); ubx = 1e9 * np.ones(z.shape[0])
        for k in range(N):
            b = self.iU + NU * k
            for i in range(4):
                lbx[b + i] = 0.0; ubx[b + i] = MAX_THRUST
            lbx[b + 4] = -BETA_MIN; ubx[b + 4] = math.pi / 2   # negative = rotors lean AFT
            lbx[b + 5] = -DE_MAX; ubx[b + 5] = DE_MAX
            lbx[b + 6] = -DA_MAX; ubx[b + 6] = DA_MAX
            lbx[b + 7] = -DR_MAX; ubx[b + 7] = DR_MAX
        # STATE bounds on the stiff body rates (key 2): unbounded, IPOPT explores huge omega_dot
        # (an unbalanced split gives qdot ~ -200) -> ill-conditioned RK4 -> solve failures.
        for k in range(N + 1):
            b = self.iX + NX * k
            for i in range(3):
                lbx[b + 16 + i] = -3.0; ubx[b + 16 + i] = 3.0
            lbx[b + 15] = -12.0; ubx[b + 15] = 12.0        # vertical speed
        # index of x[9] (= R[2][0] = sin(pitch), nose-up positive) for every horizon stage, so
        # the pitch box bound can be adapted at solve time -- see solve().
        self.i_sinpitch = [self.iX + NX * k + 9 for k in range(N + 1)]
        self.lbx, self.ubx = lbx, ubx
        self.lbg = np.zeros(gv.shape[0]); self.ubg = np.zeros(gv.shape[0])
        if self.n_aoa:
            a = math.radians(self.AOA_MAX_DEG)
            self.lbg[self.n_eq:] = -a
            self.ubg[self.n_eq:] = a
        self.zsol = None; self._zgood = None
        self._lam_g = None; self._lam_x = None   # duals carried between solves (see solve())
        self._u_good = np.array([M * G / 4] * 4 + [0.0, 0.0, 0.0, 0.0])
        self._uprev = self._u_good.copy()

    def solve(self, x0, xrefs, urefs, alt_auth=1.0):
        """xrefs: N+1 arrays(19); urefs: N arrays(8). Returns (u0(8), ok).

        alt_auth (0..1) scales ONLY the vertical position/velocity cost -- see __init__."""
        x0v = np.asarray(x0, float).ravel()
        self._x_last = x0v
        P = [x0v]
        P += [np.asarray(a, float).ravel() for a in xrefs]
        P += [np.asarray(a, float).ravel() for a in urefs]
        P.append(self._uprev)
        P.append(np.array([float(alt_auth)]))
        P = np.concatenate(P)
        if self.zsol is None:
            self.zsol = np.zeros(self.iU + NU * self.N)
            for k in range(self.N + 1):
                self.zsol[self.iX + NX * k: self.iX + NX * (k + 1)] = xrefs[min(k, len(xrefs) - 1)]
            for k in range(self.N):
                self.zsol[self.iU + NU * k: self.iU + NU * (k + 1)] = urefs[min(k, len(urefs) - 1)]
        try:
            # ★ PITCH LIMIT AS A BOX BOUND, NOT A PENALTY.
            # A stiff cost wall is the wrong tool: at w_pitch=60 IPOPT could not converge inside
            # MAX_ITER=25, every solve returned ok=0 and the aircraft departed (roll 25, yaw 164).
            # A box bound on sin(pitch) is handled natively by the interior-point method and costs
            # nothing in conditioning.  The bound is WIDENED to always admit the CURRENT pitch, so
            # it can never make the problem infeasible if the vehicle is already past the limit --
            # it then simply forbids going further, and pulls back as the state recovers.
            if self.pitch_max_rad > 0.0:
                s_lim = math.sin(self.pitch_max_rad)
                s_now = float(np.clip(x0v[9], -0.999, 0.999))
                hi = max(s_lim, abs(s_now) + 0.02)
                for i in self.i_sinpitch[1:]:            # stage 0 is pinned to x0 by the equality
                    self.lbx[i] = -hi; self.ubx[i] = hi
            # ★ NEVER LET THE BOUND MAKE THE PROBLEM INFEASIBLE.
            # If the aircraft is ALREADY past the limit the constraint must still admit where it
            # is, or every solve fails and the fallback flies -- which is how a guard becomes the
            # cause of a crash. Widen to the current AoA, so the bound forbids going FURTHER and
            # pulls back as the state recovers.
            if getattr(self, "n_aoa", 0):
                # NB: `self.aoa_max_deg` is an attribute the NODE sets from its parameter.
                # Calling self.gp() here would reach for a method that lives on the NODE class,
                # not on this one -- it raises, the except swallows it, and every solve reports
                # failure (120/120, seen live). Same wrong-class mistake as _publish_sched_ready.
                lim_deg = getattr(self, "aoa_max_deg", self.AOA_MAX_DEG)
                a = math.radians(lim_deg) if lim_deg > 0 else math.radians(self.AOA_MAX_DEG)
                Rn = np.asarray(x0v[3:12], float).reshape(3, 3)
                vbn = Rn.T @ np.asarray(x0v[13:16], float)
                a_now = abs(math.atan2(-vbn[2], math.sqrt(vbn[0] ** 2 + 1.0)))
                # ★ WIDEN ONLY THE NEAR STAGES, OR THE BOUND RATCHETS.
                # Widening every stage to (current AoA + margin) means that once the aircraft
                # drifts past the limit the bound follows it up and never pulls back: with
                # MAX_ITER=50 the solves converged but AoA reached 16.6 deg, WORSE than the 15.3
                # of no constraint at all. The aircraft cannot change its angle of attack
                # instantly, so the first stages must admit where it is -- but the rest of the
                # horizon has to come back inside the limit, which is what forces recovery.
                hi_near = max(a, a_now + math.radians(2.0))
                # ★ DO NOT WIDEN. The applied control is stage 0's, and the constraint starts
                # at stage 1 -- so exempting the near stages exempts precisely the part that
                # matters. Measured over the same disturbed mission:
                #     no constraint          AoA 15.3  pitch 23  fails 7/200
                #     widen 2 near stages    AoA 16.9  pitch 15  fails 7/200   (no effect)
                #     widen 1                AoA 13.2
                #     widen NONE             AoA 12.1  pitch 15  fails 5/200   <- holds the limit
                # The widening was a precaution against infeasibility; it cost the entire effect
                # and bought nothing -- the strict bound actually solves BETTER than the baseline.
                n_near = min(int(getattr(self, 'aoa_near', 0)), self.n_aoa)
                self.lbg[self.n_eq:] = -a
                self.ubg[self.n_eq:] = a
                if n_near:
                    self.lbg[self.n_eq:self.n_eq + n_near] = -hi_near
                    self.ubg[self.n_eq:self.n_eq + n_near] = hi_near
            sol = self.solver(x0=self.zsol, lbx=self.lbx, ubx=self.ubx,
                              lbg=self.lbg, ubg=self.ubg, p=P)
            st = self.solver.stats()["return_status"]
            ok = ("Solve_Succeeded" in st) or ("Acceptable" in st)
            if not ok:
                self._last_status = st
                if os.environ.get("NMPC_DEBUG_STATUS"):
                    it = self.solver.stats().get("iter_count", -1)
                    print("    [solve fail] %s iters=%s" % (st, it), flush=True)
            zs = np.array(sol["x"]).ravel()
            u0 = zs[self.iU:self.iU + NU]
            if ok and not np.any(np.isnan(u0)):
                self._zgood = zs.copy(); self._u_good = u0.copy()
                self.zsol = self._shift(zs)
                self._nfail = 0
            else:
                ok = False
                u0 = self._fallback(urefs)
                self._reseed(xrefs, urefs)
        except Exception:
            u0 = self._fallback(urefs); ok = False
            self._reseed(xrefs, urefs)
        self._uprev = u0.copy()
        return u0, ok

    def _shift(self, zs):
        """Shift the previous solution one stage forward as the next warm start.

        ★ AN UNSHIFTED WARM START IS A WHOLE DT STALE.  The solution was being fed back as-is,
        so stage k of the new horizon was seeded with the OLD stage k -- which describes where
        the aircraft was one control period ago.  At 16 m/s that is 3.2 m of position error in
        every stage, and it does not decay along the horizon.  The standard receding-horizon
        warm start seeds new stage k from old stage k+1 (the plant has moved one step, so the
        old plan's second point is the new plan's first).  Measured, the un-shifted guess cost
        so many iterations that 100% of solve failures were Maximum_Iterations_Exceeded at
        exactly 25 -- in states that had been dead steady for 20 s.
        """
        z = zs.copy()
        for k in range(self.N):
            z[self.iX + NX * k: self.iX + NX * (k + 1)] = \
                zs[self.iX + NX * (k + 1): self.iX + NX * (k + 2)]
        for k in range(self.N - 1):
            z[self.iU + NU * k: self.iU + NU * (k + 1)] = \
                zs[self.iU + NU * (k + 1): self.iU + NU * (k + 2)]
        return z

    def _fallback(self, urefs):
        """Control to apply when the solve fails.

        ★ HOLD THE LAST CONVERGED CONTROL FOR A FEW TICKS, THEN FALL BACK TO THE REFERENCE.
        In steady 90 deg cruise the solve is marginal: offline, t=41.0 converged and t=41.5 did
        not with a NUMERICALLY IDENTICAL state (z 20.0, V 16.0, tilt 90, AoA 5.6) -- IPOPT simply
        ran past MAX_ITER.  So an isolated failure says nothing about the state, and the control
        that was correct 100 ms ago is still correct.
        Neither documented option survives here.  The reference feed-forward zeroes da/dr, and at
        90 deg the surfaces are the ONLY lateral authority (front pair horizontal -> differential
        thrust makes yaw not roll; rear unloaded to 3.6 N): applying zero surfaces took a perfect
        cruise to roll 41 deg in one second and it spiralled out.  Carrying ONLY the last lateral
        control while taking the reference's longitudinal trim was worse still -- roll -39 -> +87
        -> -112 deg -- because a stale surface correction with a fresh trim is not a package.
        A trim is a package (bags full_tr7, full_tr25/27): hold ALL EIGHT channels or none.
        The freeze is BOUNDED.  An indefinitely frozen actuator stream is the worst failure mode
        there is (bag vt1: 17 s of byte-identical commands while it fell), so after `_HOLD_TICKS`
        consecutive failures the state really has moved and we revert to the reference trim.
        """
        ug = getattr(self, "_u_good", None)
        if ug is None or self._diverged():
            return np.asarray(urefs[0], float).copy()
        u = np.asarray(ug, float).copy()
        # ★ NEVER FREEZE THE LATERAL COMMAND.  Holding the last converged control is right for
        # the LONGITUDINAL trim (total thrust, tilt, elevator) -- that is a force-balanced
        # package and the state has barely moved in one tick.  The differential between the
        # rotors is not a trim at all: it is FEEDBACK, a correction for the roll/yaw error of
        # that instant.  Frozen open-loop it becomes a constant rolling/yawing moment.
        # Bag vtol_20260909_215443: the last converged control was correcting a +14 deg yaw with
        # mot=[0.12,0.62,...], the solve then failed and the fallback held [0.79,0.17,0.38,0.72]
        # BYTE-IDENTICAL for two seconds while the aircraft rolled away -- the same frozen
        # actuator stream that killed bag vt1, reintroduced by my own fallback.
        # Keep each PAIR's total (so thrust and pitch balance survive) and null the differential.
        f = 0.5 * (u[0] + u[1]); r = 0.5 * (u[2] + u[3])
        u[0] = u[1] = f; u[2] = u[3] = r
        return u

    def _diverged(self):
        """Is the vehicle far enough from level flight that the last control is meaningless?"""
        x = getattr(self, "_x_last", None)
        if x is None:
            return False
        R = np.asarray(x[3:12], float).reshape(3, 3)
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        pitch = math.degrees(math.asin(float(np.clip(R[2, 0], -1, 1))))
        return abs(roll) > 50.0 or abs(pitch) > 35.0 or R[2, 2] < 0.4

    def _reseed(self, xrefs, urefs):
        """Warm start after a failed solve: restore the last SUCCESSFUL trajectory.

        ★ DO NOT RESEED FROM THE REFERENCE.  It looks right -- the reference is a self-consistent
        trim for where the vehicle is being asked to go, and the stale-warm-start theory (one
        failure poisons every later solve) is plausible.  MEASURED, it is a large regression:
        offline Vc=16 went from 0/79 solve failures to 54/139, and a clean transition (roll 1 deg,
        altitude 19-20 m) became roll 53 deg with the altitude ranging 6.5-36.6 m.  IPOPT recovers
        from an isolated failure far better from the last converged trajectory -- which is close to
        feasible -- than from a reference the vehicle may be nowhere near.  It also did NOT fix
        what motivated it: the --vc 20 failure cascade was bit-identical with and without it,
        which is the tell that the cascade is a REFERENCE problem, not a warm-start problem.
        """
        self._nfail = getattr(self, "_nfail", 0) + 1
        if self._zgood is not None:
            self.zsol = self._zgood




def apply_tilt_limits(u4, V_now, sched, prev_beta, dt, tilt_margin_deg, tilt_commit_v,
                      tilt_lpf_tau, tilt_slew_deg_s, lift_margin=0.0):
    """The APPLIED-tilt path: airspeed clamp -> lift clamp -> low-pass -> slew limit.

    ★ SHARED BY THE ROS LOOP AND THE OFFLINE TEST, and it must stay that way.
    This logic used to live only in the ROS loop, so `--test` fed the solver's raw u4
    STRAIGHT to the plant -- none of the clamping, filtering or rate limiting that the real
    aircraft flies through was exercised.  That is why offline could not reproduce any of the
    tilt behaviour seen in flight: HIL thrashed the applied tilt 40->23->32->38 deg while the
    offline run showed 1.0 deg steps, and the reference-vs-applied gap that drove the pitch
    excursion (30/45, 34/45) simply could not occur.  Fixes to this path were being "validated"
    against a test that did not run it.
    """
    raw_beta = float(_cl(u4, -BETA_MIN, math.pi / 2))
    # airspeed tilt clamp (see tilt_margin_deg): never tilt past the trim tilt for V_now
    tilt_trim = float(sched.at(max(V_now, 0.0))["beta"])
    # ★ COMMIT THE TILT ONCE THERE IS ENOUGH SPEED TO BE SAFE.
    # The clamp exists because over-tilting at LOW airspeed is fatal: the front rotors go
    # horizontal (no pitch moment) and the elevator has no dynamic pressure, so the nose
    # falls -- that is what crashed bag full2.  But that danger is a LOW-SPEED one.  Above
    # `tilt_commit_v` the elevator has real authority and the wing is carrying a useful
    # share, so holding the tilt back just prolongs the corridor: release the clamp and
    # let it go to 90 rather than fighting to hold altitude in the worst part of the
    # envelope.  Blended over 2 m/s so the clamp does not step.
    v_commit = tilt_commit_v
    if v_commit > 0.0 and V_now > v_commit - 2.0:
        f = min(1.0, max(0.0, (V_now - (v_commit - 2.0)) / 2.0))
        margin = math.radians(tilt_margin_deg) * (1.0 - f) + (math.pi / 2) * f
    else:
        margin = math.radians(tilt_margin_deg)
    raw_beta = min(raw_beta, tilt_trim + margin)
    # ★ VERTICAL-FORCE BUDGET CLAMP (bag full_tr16).  The margin clamp above is empirical
    # and let the vehicle sit at 83 deg tilt at 9.6 m/s, where the sustainable tilt is only
    # 75 deg -- at 83 deg the four rotors can supply just 72*cos(83) = 8.8 N of LIFT, and
    # the wing at 9.6 m/s carries 46% of weight even at its CL peak, so the sum is short
    # and it SANK 5 m through the (now wider) altitude band.  This clamp asks the only
    # question that matters: can rotors + wing still hold the aircraft up at this tilt?
    #     T_max*cos(beta) + L_wing_peak(V) >= k * W
    # It needs no tuning, and it self-relaxes to 90 deg at exactly the speed the physics
    # allows (13.5 deg -> 87.6, 14.1 -> 89.9) instead of at an arbitrary margin.
    # DISABLED BY DEFAULT (`lift_margin`=0) -- bag full_tr17 shows it makes things WORSE
    # for two reasons, both worth remembering:
    #  (1) it credits the wing with its PEAK CL, which the wing only delivers at alpha=10
    #      deg; during acceleration the real lift is well below that, so the clamp was most
    #      permissive exactly when the wing was under-delivering;
    #  (2) restricting the TILT in an energy-limited corridor does not remove the demand for
    #      forward force, it MOVES it -- denied tilt, the solver accelerated by pitching
    #      down instead, and the vehicle reached 89.4 deg tilt at 15.0 m/s while SINKING
    #      6.8 m.  Level wing share fell 74.5% -> 24.7% vs full_tr16.
    # To make this idea work it must use the ACTUAL modelled lift at the current state, not
    # the peak, AND be paired with something that stops the solver trading altitude for
    # speed.  Set lift_margin > 0 to re-enable.
    L_pk = 0.5 * 1.225 * V_now * V_now * WING_S * CL_PEAK
    need = lift_margin * so3.M * so3.G - L_pk
    if lift_margin > 0.0 and need > 0.0:
        c = need / (4.0 * so3.MAX_THRUST)
        beta_force = math.acos(min(1.0, c)) if c <= 1.0 else 0.0
        raw_beta = min(raw_beta, beta_force)
    # ★ LOW-PASS THE TILT COMMAND.
    # The solver's raw tilt HUNTS: bag vtol_20260909_145609 shows the applied tilt
    # oscillating +-5 deg about its reference (25/21, 22/22, 17/23, 25/25, 29/26, 21/27),
    # reversing direction on 20% of steps.  That is not a tracking lag -- consecutive
    # solves genuinely disagree (MAX_ITER=25 does not converge to the same answer twice),
    # and w_bcmd only reduced it.  It matters because the trim splits front/rear thrust to
    # balance the PITCH MOMENT AT A GIVEN TILT: when the applied tilt oscillates the
    # balance breaks, the aircraft pitches (+11..+13 deg against a level reference) and
    # the elevator saturates at +-1.00 chasing it.  A first-order filter removes the
    # oscillation without asking the solver for anything, and unlike the slew limit it
    # attenuates rather than merely paces it.
    tau = max(0.0, tilt_lpf_tau)
    prev_b = float(prev_beta)
    if tau > 1e-3:
        a = dt / (tau + dt)
        raw_beta = prev_b + a * (raw_beta - prev_b)
    slew = math.radians(tilt_slew_deg_s) * dt
    return float(_cl(raw_beta, prev_b - slew, prev_b + slew))
def build_refs(sched, nmpc, p0, heading, z_ref, v_cmd, Vc, ttr, s_now, settling,
               v_meas=None,    # v_meas kept for call-site compatibility; unused
               v_wb=14.5, sink=0.0, z_floor=-1e9, accel_thr=0.0,
               decel=False, decel_rate=1.5, flare_deg=0.0, beta_aft=0.0,
               th_prev=None, th_slew=0.0, dt_ref=0.1, pitch_gate_v=0.0,
               tilt_hold=0.0, tilt_hold_v=4.0, tilt_commit_v=0.0, tilt_commit_band=5.0,
               tilt_margin=0.0, b_prev=None, b_slew=0.0, dive_deg=0.0):
    """Build the horizon reference: advance the target speed toward Vc, take the wing-borne trim at
    each stage, and wrap it into a full 19-state reference (roll=0, yaw=heading, pitch=trim).

    `v_cmd` is the COMMANDED speed carried on the node's own clock -- NOT the measured speed.
    This used to read `V_now` straight from mocap, so the horizon could only ever ask for
    V_now + ttr-slope * horizon ~ +0.7 m/s. When the pitch/tilt coupling bled speed off, the
    reference re-anchored to whatever the vehicle had left and followed it DOWN: there was no
    persistent demand to accelerate, and the vehicle limit-cycled at 3-5 m/s for 300+ m
    (HIL bags full_tr3..full_tr5). Same failure as the SMC re-seeding on tracking lag
    (audit finding C7) -- a reference that surrenders to its own tracking error. The node now
    ramps v_cmd on a clock and only BOUNDS how far it may lead the vehicle.
    """
    # ONE speed sequence, and it must stay one. `v_cmd` is the COMMANDED speed carried on
    # the node's own clock -- NOT the measured speed (see the docstring).
    #
    # REVERTED (HIL bag full_tr7): I tried splitting this into a leading "demand" speed for
    # the state reference and a measured-anchored "feed-forward" speed for the control
    # reference, reasoning that the trim split is only valid at the speed it was solved for.
    # That is true, but the split ALSO puts the tilt in two places at once -- x[12] from the
    # demand and u[4] from the feed-forward -- and they then disagreed by ~18 deg. The
    # reference stopped being a self-consistent trim and the solver got contradictory
    # guidance: pitch swung +27/-1 deg and the speed fell back to 3-4 m/s, worse than the
    # single-sequence version which reached 8.45 m/s. A trim is a package; do not take it
    # apart. If the feed-forward is too far ahead of the vehicle, shorten the LEAD
    # (`v_lead`), which moves the whole self-consistent package back together.
    # ★ ROLLING TRANSITION (bag full_tr18).  The altitude reference used to be FLAT at z_ref for
    # every stage, with zero vertical velocity -- i.e. "hold this altitude" throughout the corridor.
    # Below the 14.13 m/s stall speed the wing cannot carry the weight, so obeying that command
    # forces the solver to buy lift with ANGLE OF ATTACK: full_tr18 sat at AoA +28..+35 deg at
    # 8-9 m/s, far past the 10 deg CL peak and deep on the BACK SIDE of the drag curve, where drag
    # is high and it can never accelerate out.  Widening `cruise_alt_band` did NOT help because
    # that only relaxes the settling GATE -- the COST still demanded z_ref, so the solver correctly
    # chose altitude over AoA and mushed.
    # A real aeroplane resolves this by descending ON PURPOSE while it accelerates, then levelling
    # once the wing can carry.  So the reference now COMMANDS a bounded sink while the target speed
    # is below wing-borne, and levels off above it.  The demand is what has to change; permission
    # was never enough.
    fwd = np.array([math.cos(heading), math.sin(heading), 0.0])
    xrefs = []; urefs = []; s = s_now; z = z_ref; b_first = 0.0
    for k in range(nmpc.N + 1):
        if settling:
            vtar = 0.0
        elif decel:
            # ★ BACK-TRANSITION.  Run the SAME trim schedule BACKWARDS: ramp the commanded speed
            # DOWN and the schedule hands back tilt, pitch and thrust for each speed, so the
            # rotors come upright in step with the wing unloading.  Bag full_tr30 shows what the
            # absence of this costs: the harness handed a 16 m/s wing-borne aircraft straight to
            # the multicopter controller, tilt snapped 90 -> 0 in ONE step, lift doubled, it
            # ballooned 10 m, stalled at AoA +78 deg and departed into a dive -- 65.7 m excursion.
            vtar = max(v_cmd - (k * nmpc.DT) * decel_rate, 0.0)
        else:
            vtar = min(max(v_cmd, 0.0) + (k * nmpc.DT) * (Vc / max(ttr, 1e-3)), Vc)
        sc = sched.at(vtar)
        sc_trim_beta = float(sc["beta"])      # the schedule tilt the airspeed clamp is built on
        # ★ THE TRIM MUST BE SOLVED FOR THE PITCH THE REFERENCE ACTUALLY COMMANDS.
        # The commanded pitch is the schedule's theta AFTER the speed gate (and dive); the tilt
        # hold below re-solves the thrust split and was passing the RAW schedule theta. At 2 m/s
        # the gate is (2/12)^2 = 0.028, so the reference flew ~0 deg nose-up while the split was
        # balanced for the full +8 trim -- two different aircraft. Measured as up to 2.0 rad/s^2
        # of residual pitch (scripts/audit_reference.py); a reference is only a trim if every
        # part of the package describes the same state.
        # The commanded sink, hoisted: the tilt hold below needs it to trim for the right
        # FLIGHT PATH. It used to be computed after that block, so referencing it there raised
        # NameError -- which the block's `except Exception` swallowed, silently skipping the
        # thrust re-solve entirely (see below).
        vz_t = -sink if (not settling and vtar < v_wb and z > z_floor) else 0.0
        th_cmd = sc["theta"]
        if pitch_gate_v > 0.0:
            th_cmd = th_cmd * min(1.0, (max(vtar, 0.0) / pitch_gate_v) ** 2)
        if dive_deg > 0.0 and not settling and not decel and v_wb > 1e-3:
            _lo = min(1.0, max(0.0, (vtar - 1.0) / 3.0))
            _hi = min(1.0, max(0.0, (vtar - (v_wb - 3.0)) / 3.0))
            th_cmd -= math.radians(dive_deg) * _lo * (1.0 - _hi)
        # ★ COMMIT THE TILT AND HOLD IT, instead of trailing the trim schedule.
        # `Schedule.at(V)` returns the TRIM tilt -- the angle that BALANCES at V, which by
        # definition makes ~zero net forward force (trim_split solves vertical and pitch and lets
        # Fx fall out as a residual).  So the reference could only ever ask for the tilt the
        # aircraft had already earned, and the tilt trailed the speed instead of causing it:
        # bag vtol_20260909_200439 sat at 21-25 deg applied against a 46-51 deg reference while
        # the speed stalled at 7-8 m/s and never approached the 14.13 m/s it needed.
        # Instead: RAMP to a committed angle, HOLD it while the speed builds (this is the part
        # that actually accelerates -- at 45 deg the front pair makes 25.5 N = 7.4 m/s^2), then
        # go to 90 once fast enough for the wing to carry.  A hold is also inherently smooth:
        # there is nothing for the solver to hunt against while it is constant.
        # (Tf,Tr) are RE-SOLVED at the held tilt -- a trim is a package, and shipping a tilt
        # without its matching thrusts is what broke bags full_tr7 and full_tr25/27.)
        if tilt_hold > 0.0 and not settling and not decel:
            bh = math.radians(tilt_hold)
            if vtar < tilt_hold_v:
                # ramp in with speed.  NEVER apply the full hold angle at zero airspeed: the
                # front rotors go horizontal, they make no pitch moment and the elevator has no
                # dynamic pressure, so the nose simply falls (bag full2).
                b_new = bh * (max(vtar, 0.0) / max(tilt_hold_v, 1e-3))
            elif tilt_commit_v <= 0.0 or vtar < tilt_commit_v:
                b_new = bh                                  # HOLD -- build speed here
            else:
                # ★ COMMIT TO 90 ON A FIXED, FAST BLEND -- do NOT follow the trim up.
                # Tried trim-following (the trim reaches 90 exactly at the cruise speed, so it
                # self-scales) and it was WORSE at every speed: at Vc=20 the reference lingers
                # between 72 and 88 deg all the way from 14 to 20 m/s, and at Vc=16 it turned a
                # clean transition (roll 1 deg, no solve failures) into roll 76 deg and 54/140
                # failures.  This is the documented corridor rule -- LINGERING IN THE CORRIDOR
                # IS THE DANGER, faster through it is better (bag so3d).  The commit speed is a
                # property of the WING (it must be able to carry: stall is 14.13 m/s), not of the
                # cruise speed, so it must NOT scale with Vc.  Once at 90 deg and wing-borne,
                # accelerating further is nearly free -- drag at 20 m/s is only 2.9 N against the
                # front pair's 36 N.
                # ★ THE COMMIT BAND MUST BE SLOWER THAN THE TILT SERVO.
                # A 2 m/s band steps the tilt reference from the 45 deg hold to ~88 deg in ONE
                # tick, while the applied tilt is slew-limited to tilt_slew_deg_s (15 deg/s) and
                # needs 45/15 = 3 s to get there -- roughly twice the 1.6 s horizon.  The solver
                # is then asked every tick for a tilt it cannot reach inside the horizon, and it
                # simply runs out of iterations: every failure was Maximum_Iterations_Exceeded,
                # and they began at exactly the speed where the band opens (V=16.8) regardless of
                # tilt angle, MAX_ITER, warm start or pitch bound.
                # Width it so the speed ramp takes at least as long as the servo does:
                #   t_servo = (90 - hold)/slew,  band >= t_servo * accel = t_servo * Vc/ttr.
                f = min(1.0, max(0.0, (vtar - tilt_commit_v) / max(tilt_commit_band, 0.1)))
                b_new = bh * (1.0 - f) + (math.pi / 2) * f
            # ★ THE REFERENCE MUST RESPECT THE SAME AIRSPEED CLAMP THE APPLIED TILT OBEYS.
            # The applied tilt is capped at trim_tilt(V) + tilt_margin_deg (the low-speed
            # over-tilt guard, without which the nose falls -- bag full2).  The HOLD ignored it
            # and commanded 45 deg from 4 m/s, so the reference asked for a tilt the vehicle was
            # forbidden to reach: bag vtol_20260909_214952 flew 30/45, 34/45, 38/45 -- exactly
            # trim(V)+15 each time.  The thrust split is solved for the REFERENCE tilt, so at a
            # 15 deg gap the pitch moment is unbalanced: pitch ran +13..+17 with AoA +11..+16,
            # past the CL peak where drag doubles, and the aircraft needed 10 s to go from 3 to
            # 7 m/s -- all of it in the mid-corridor gap where the tilted rotors make yaw rather
            # than roll and the rudder has no dynamic pressure yet (yaw hand-over is 12.3 m/s).
            # It departed in yaw at 5.9 m/s.  Clamping the reference keeps (tilt, Tf, Tr) a
            # package the vehicle can actually be in.
            if tilt_margin > 0.0:
                b_new = min(b_new, float(sc_trim_beta) + tilt_margin)
            b_new = max(b_new, 0.0)
            # ★ DO NOT WRAP THIS IN `except Exception`.
            # It used to be, and that hid a NameError for the whole of this work: the re-solve
            # never ran, the reference shipped a tilt with the WRONG thrusts, and every edit to
            # this call was dead code that changed nothing. Only genuinely numerical failures
            # (a singular split) are caught now; a programming error must be loud.
            try:
                # ★ THE TRIM MUST KNOW ABOUT THE COMMANDED SINK.
                # trim_split zeroes the pitch moment for a given FLIGHT PATH; called without
                # gamma it balances LEVEL flight while the reference is descending, so the aero
                # is evaluated at the wrong angle of attack and the moment does not cancel.
                # Measured (scripts/audit_reference.py): 1.29 and 1.50 rad/s^2 of residual pitch
                # at 12 and 14 m/s -- exactly where the sink is active AND dynamic pressure is
                # high -- against ~0.02 at 15-16 m/s where the sink switches off. The 2026-09-06
                # audit treats anything above 0.05 as a reference that is not a trim.
                gam = math.atan2(vz_t, max(vtar, 1.0))
                Tf, Tr = so3.trim_split(b_new, th_cmd, max(vtar, 1.0), sc["de"],
                                        so3.M * so3.G, gam)
                sc = dict(sc); sc["beta"] = b_new
                sc["Tf"] = float(Tf); sc["Tr"] = float(Tr)
            except (np.linalg.LinAlgError, ValueError, ZeroDivisionError):
                sc = dict(sc); sc["beta"] = b_new
        s = s + vtar * nmpc.DT if k > 0 else s
        # (vz_t computed above, before the tilt hold that trims against it)
        # ★ COMMAND THE THRUST, DON'T HOPE FOR IT (bag full_tr25).  The trim solves VERTICAL and
        # PITCH balance and lets forward force fall out as a residual, so at high tilt it asks for
        # 0-10% throttle while 72 N sits unused -- the vehicle coasted down to 12-14 m/s, right on
        # the 14.13 m/s stall boundary where the phugoid lives.  Demanding more SPEED instead
        # (v_lead 15) did not fix it: the solver took the cheapest route to speed, which is
        # GRAVITY -- it drew 0.0 N of thrust and dove 81 m.
        # NOTE only the FRONT pair tilts, so at 90 deg the maximum forward thrust is the front
        # pair alone (2 x MAX_THRUST = 36 N = 50% of installed), not all four rotors.
        if accel_thr > 0.0 and not settling and vtar < Vc - 0.1:
            tf_tot = accel_thr * 2.0 * so3.MAX_THRUST      # commanded FRONT-pair thrust [N]
            sc = dict(sc); sc["Tf"] = tf_tot
        if k > 0:
            z = max(z_floor, z + vz_t * nmpc.DT)
        pos = p0 + fwd * s; pos = np.array([pos[0], pos[1], z])
        vel = fwd * vtar + np.array([0.0, 0.0, vz_t])
        # NOSE-UP FLARE while decelerating.  A tiltrotor cannot tilt backwards (beta >= 0), so
        # with the body LEVEL there is no decelerating force at all.  Pitching nose-up vectors the
        # (near-vertical) rotor thrust BACKWARD -- T*sin(theta) -- and adds high-AoA drag: at 30 deg
        # that is ~17 N = 4.9 m/s^2 of braking.  Faded in with speed so the flare is strongest while
        # there is still speed to kill and goes to zero as it arrives at hover.
        th = th_cmd            # computed above, so the trim and the attitude agree
        # ★ DO NOT PITCH BEFORE THE WING CAN USE IT.
        # The trim schedule commands +2..+6 deg of nose-up between 4 and 8 m/s, where the wing
        # can only carry 8-32% of the weight.  At that speed the pitch is almost pure DRAG: it
        # steals from the acceleration that the transition depends on, and it is what the harsh
        # pitching in the hover->cruise leg actually is.  Wing lift goes as V^2, so scale the
        # commanded pitch the same way -- level while the tilt does the accelerating, and the
        # nose-up trim fading in only as the wing starts carrying real load.

        # ★ COMMAND A NOSE-DOWN DIVE THROUGH THE CORRIDOR.
        # Gating the trim pitch to level is not enough: the reference still says "hold z0", so
        # the solver buys the altitude with ANGLE OF ATTACK. Flown (bag vtol_20260910_093744):
        # altitude pinned at 29.0-29.5 m the whole acceleration while pitch ran +10..+15 and AoA
        # sat at +11..+15 -- past the 10 deg CL peak, where lift FALLS and drag climbs. It needed
        # 30 s to reach 9.7 m/s and was still in the mid-corridor lateral gap when it departed.
        # Below stall speed the wing cannot carry the aircraft, so the honest manoeuvre is to
        # trade height for speed and get out of the corridor quickly. Ask for the nose down.
        # Faded in from rest and back out as the wing loads, so hover and cruise are untouched.

        if decel and Vc > 1e-3:
            frac = min(1.0, vtar / Vc)                 # brake hardest while there is speed to kill
            th = th + math.radians(flare_deg) * frac
            # ★ AFT TILT.  With beta >= 0 the ONLY decelerating force is nose-up pitch, and that
            # buys braking with a ZOOM: bag full_tr31 climbed 18.9 m, and 12.8 m of that is the
            # unavoidable kinetic->potential conversion of 434 J.  Leaning the FRONT pair aft puts
            # the braking force along the flight path instead, so the body can stay closer to
            # level: T*sin(20 deg) = 12.3 N = 3.6 m/s^2 with no zoom.
            # The tilt is COMMANDED in the reference (not left to the solver) because w_beta=60
            # holds the applied tilt to this schedule -- and the (Tf,Tr) split is RE-SOLVED for
            # the new beta by the same trim solver, so the reference stays a balanced package.
            if beta_aft > 0.0:
                b_new = -beta_aft * frac
                try:
                    Tf, Tr = so3.trim_split(b_new, th, max(vtar, 1.0), sc["de"],
                                            so3.M * so3.G, math.atan2(vz_t, max(vtar, 1.0)))
                    sc = dict(sc); sc["beta"] = b_new; sc["Tf"] = float(Tf); sc["Tr"] = float(Tr)
                except Exception:
                    sc = dict(sc); sc["beta"] = b_new      # keep the trim thrusts if it will not solve
        # ★ RATE-LIMIT THE COMMANDED PITCH.  The trim schedule's theta is a function of SPEED
        # (about +8 deg at 12 m/s falling to +4 by 16), so the faster the speed ramp the faster
        # the attitude command moves -- and at the hand-over, where the wing takes the load, it
        # can step.  That is what the harsh pitch at cruise entry is.  Slewing the FIRST stage
        # and carrying it forward keeps the reference a smooth attitude the solver can follow
        # instead of a moving target it chases.
        if th_slew > 0.0 and th_prev is not None:
            lim = math.radians(th_slew) * max(dt_ref, 1e-3) * (k + 1)
            th = max(th_prev - lim, min(th_prev + lim, th))
        # ★ RATE-LIMIT THE COMMANDED TILT, exactly as the pitch is rate-limited above.
        # The applied tilt is slew-limited (tilt_slew_deg_s), so a reference that STEPS is a
        # reference the vehicle cannot reach, and the thrust split -- solved for the reference
        # tilt -- is then unbalanced for the tilt it is actually at.  Forward, that produced the
        # 30/45 gap and a +17 deg pitch excursion.  BACKWARD it is far worse: the aft-tilt brake
        # is `-beta_aft * min(1, vtar/Vc)`, which is FULL at the moment decel starts, so the
        # reference jumped +90 -> -20 deg in one tick.  Every solve failed from that instant and
        # the aircraft simply cruised on at 16 m/s (offline --decel_at 45).
        if b_slew > 0.0 and b_prev is not None:
            lim = math.radians(b_slew) * max(dt_ref, 1e-3) * (k + 1)
            b_lim = max(b_prev - lim, min(b_prev + lim, float(sc["beta"])))
            if abs(b_lim - float(sc["beta"])) > 1e-9:
                try:
                    Tf, Tr = so3.trim_split(b_lim, th, max(vtar, 1.0), sc["de"],
                                            so3.M * so3.G, math.atan2(vz_t, max(vtar, 1.0)))
                    sc = dict(sc); sc["Tf"] = float(Tf); sc["Tr"] = float(Tr)
                except Exception:
                    sc = dict(sc)
                sc["beta"] = b_lim
        Rr = R_ref(heading, th)
        if k == 0:
            th_first = th; b_first = float(sc["beta"])
        xrefs.append(np.concatenate([pos, Rr.reshape(-1), [sc["beta"]], vel, [0.0, 0.0, 0.0]]))
        if k < nmpc.N:
            urefs.append(np.array([sc["Tf"] / 2, sc["Tf"] / 2, sc["Tr"] / 2, sc["Tr"] / 2,
                                   sc["beta"], sc["de"], 0.0, 0.0]))
    return xrefs, urefs, th_first, b_first


# ============================ offline test vs the TRUE plant ============================
def offline_test():
    def argf(n, d): return float(sys.argv[sys.argv.index(n) + 1]) if n in sys.argv else d
    Vc = argf("--vc", 16.0); cap = argf("--cap", 90.0); amax = argf("--amax", 8.0)
    ttr = argf("--ttr", 18.0); dur = argf("--dur", 40.0); z0 = argf("--z0", 20.0)
    LAT = argf("--lat", 0.10); V_LEAD = argf("--vlead", 3.0)
    decel_at = argf("--decel_at", 0.0)      # start the back-transition at this time [s]
    decel_rate_v = argf("--decel_rate", 1.5)
    decel_on = False
    dist_yaw = argf("--dist_yaw", 0.0)      # steady yaw moment [N.m] over the window below
    dist_t0 = argf("--dist_t0", 8.0); dist_t1 = argf("--dist_t1", 20.0)
    roll0 = math.radians(argf("--roll0", 0.0)); yaw0 = math.radians(argf("--yaw0", 0.0))
    sched = Schedule(V_cruise=Vc, tilt_cap_deg=cap, alpha_max_deg=amax)
    nmpc = FullNMPC(); F = so3.build_ode()
    nmpc.aoa_max_deg = argf("--aoamax", nmpc.AOA_MAX_DEG)
    nmpc.aoa_near = argf("--aoanear", 0.0)
    if "--nopitchbound" in sys.argv:
        nmpc.pitch_max_rad = 0.0          # runtime only -- does NOT change the JIT cache key
        print("  [pitch box bound DISABLED]")
    DT = 0.02; nsolve = max(1, int(nmpc.DT / DT)); lat_n = max(1, int(round(LAT / DT)))
    th_ref = None; b_ref = None; beta_cmd = 0.0
    x = np.zeros(NX); x[3:12] = np.eye(3).reshape(-1); x[0:3] = [0, 0, z0]
    if abs(roll0) > 1e-3 or abs(yaw0) > 1e-3:
        cr, sr = math.cos(roll0), math.sin(roll0); cy, sy = math.cos(yaw0), math.sin(yaw0)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        x[3:12] = (Rz @ Rx).reshape(-1)
    p0 = x[0:3].copy(); heading = 0.0; v_cmd = 0.0
    u = np.array([M * G / 4] * 4 + [0, 0, 0, 0]); hist = [u.copy()] * (lat_n + 1)
    print("FULL-STATE NMPC + SMC vs TRUE plant: hover -> 90deg wing-borne. Vc=%.0f cap=%.0f roll0=%.0f yaw0=%.0f"
          % (Vc, cap, math.degrees(roll0), math.degrees(yaw0)))
    print("  t    x     z    V   tilt | roll pitch  yaw | AoA | Tf   Tr  | ok")
    crashed = False
    for it in range(int(dur / DT)):
        R = x[3:12].reshape(3, 3); v = x[13:16]; V = float(np.linalg.norm(v))
        if it % nsolve == 0:
            xp = so3.predict_forward(F, x, hist[-lat_n:], DT) if LAT > 1e-3 else x
            s_now = float(np.dot(xp[0:3] - p0, [math.cos(heading), math.sin(heading), 0.0]))
            settling = (it * DT) < 2.0
            # trigger the back-transition once, at --decel_at seconds (0 = never)
            decel_on = decel_at > 0.0 and (it * DT) >= decel_at
            Vm = float(np.linalg.norm(xp[13:16]))
            if settling:
                v_cmd = Vm
            else:
                # ★ MIRROR THE NODE: during the back-transition the commanded speed must ramp
                # DOWN on its own clock (node line: _v_cmd -= decel_rate*dt).  The offline test
                # only ever ramped UP, so each tick re-anchored the decel at Vc and the horizon
                # asked to shed 2.4 m/s over 1.6 s from 16 -- forever.  The aircraft brought its
                # tilt down to 7 deg and then cruised at 15.3 m/s indefinitely.  That was the
                # HARNESS, not the controller -- the same reference-surrender shape, in my test.
                if decel_on:
                    v_cmd = max(v_cmd - decel_rate_v * nmpc.DT, 0.0)
                else:
                    v_cmd = min(v_cmd + (Vc / max(ttr, 1e-3)) * (nmpc.DT), Vc)
                    v_cmd = max(0.0, min(v_cmd, Vm + V_LEAD))
            xrefs, urefs, th_ref, b_ref = build_refs(sched, nmpc, p0, heading, z0, v_cmd,
                                      Vc, ttr, s_now, settling, v_meas=Vm,
                                      sink=argf("--sink", TRANS_SINK),
                                      z_floor=z0 - DESCENT_BUDGET,
                                      th_prev=th_ref, th_slew=PITCH_SLEW_DEG_S,
                                      dt_ref=nmpc.DT, pitch_gate_v=PITCH_GATE_V,
                                      tilt_hold=(0.0 if "--nohold" in sys.argv else TILT_HOLD_DEG),
                                      tilt_hold_v=TILT_HOLD_V,
                                      tilt_commit_v=argf("--commitv", TILT_COMMIT_V),
                                      tilt_commit_band=argf("--band", TILT_COMMIT_BAND),
                                      tilt_margin=math.radians(argf("--margin", 15.0)),
                                      # ★ BACK-TRANSITION, offline.  build_refs has always
                                      # supported decel/flare/aft-tilt but the offline test never
                                      # passed them, so the cruise->hover half of the mission had
                                      # never been exercised here at all -- the same harness gap
                                      # that made every corridor result unfalsifiable.
                                      decel=decel_on,
                                      decel_rate=decel_rate_v,
                                      flare_deg=argf("--flare", 15.0),
                                      beta_aft=math.radians(argf("--beta_aft", 20.0)),
                                      b_prev=b_ref, b_slew=TILT_SLEW_DEG_S,
                                      dive_deg=argf("--dive", DIVE_DEG))
            u, ok = nmpc.solve(xp, xrefs, urefs,
                               argf("--altauth", -1.0) if "--altauth" in sys.argv
                               else _alt_authority(Vm, settled=not settling))
            # ★ Fly the SAME applied-tilt path the aircraft does.  Feeding the solver's raw
            # u[4] straight to the plant is what made every offline tilt result meaningless.
            beta_cmd = apply_tilt_limits(u[4], Vm, sched, beta_cmd, nmpc.DT,
                                         TILT_MARGIN_DEG, TILT_COMMIT_V,
                                         TILT_LPF_TAU, TILT_SLEW_DEG_S)
            u = np.asarray(u, float).copy(); u[4] = beta_cmd
        hist.append(u.copy()); hist = hist[-200:]
        xdot = np.array(F(x, hist[-1 - lat_n])).ravel()
        # ★ LATERAL DISTURBANCE -- WITHOUT THIS THE OFFLINE TEST CANNOT SEE THE REAL FAILURE.
        # Flown, every departure this campaign has been LATERAL and it develops while the solver
        # is converging happily (bag vtol_20260909_215829: roll -26, yaw -36 with ok=1 at 28 ms).
        # Offline, roll and yaw sit at EXACTLY 0.0 for the whole run -- the plant is undisturbed
        # and symmetric, so the corridor's yaw-authority gap simply never gets excited and every
        # lateral fix "passes" a test that cannot fail.  That is the C5 finding in a new place:
        # a test that validates the controller against a world where the problem does not exist.
        # Inject a yaw moment during the corridor so the gap is actually probed.
        if dist_yaw != 0.0 and dist_t0 <= it * DT <= dist_t1:
            xdot[18] += dist_yaw / so3.IZZ          # N.m -> yaw angular acceleration
        x = x + DT * xdot
        Rn = x[3:12].reshape(3, 3); Uu, _, Vt = np.linalg.svd(Rn); x[3:12] = (Uu @ Vt).reshape(-1)
        if x[2] < 0.3 or x[2] > 60:
            print("  !! CRASH at t=%.1f z=%.1f" % (it * DT, x[2])); crashed = True; break
        if it % 25 == 0:
            vb = R.T @ v
            aoa = math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-6)) if V > 1.0 else 0.0
            pitch = math.degrees(math.asin(_cl(R[2, 0], -1, 1)))      # nose-up +
            roll = math.degrees(math.atan2(R[2, 1], R[2, 2])); yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
            print("%5.1f %5.1f %5.1f %4.1f %5.0f |%5.0f %5.0f %5.0f |%5.1f | %4.1f %4.1f | %d"
                  % (it * DT, x[0], x[2], V, math.degrees(x[12]), roll, pitch, yaw, aoa,
                     u[0] + u[1], u[2] + u[3], int(ok)))
    if not crashed:
        R = x[3:12].reshape(3, 3); vb = R.T @ x[13:16]
        aoa = math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-6))
        print("DONE z=%.1f V=%.1f tilt=%.0f AoA=%.1f rotors=%.1fN (weight %.1f) roll=%.0f yaw=%.0f"
              % (x[2], np.linalg.norm(x[13:16]), math.degrees(x[12]), aoa, float(np.sum(u[0:4])),
                 M * G, math.degrees(math.atan2(R[2, 1], R[2, 2])), math.degrees(math.atan2(R[1, 0], R[0, 0]))))


def _q2R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) + 1e-12
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def run_ros():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                           QoSHistoryPolicy, qos_profile_sensor_data)
    from px4_msgs.msg import (OffboardControlMode, ActuatorMotors, ActuatorServos,
                              VehicleCommand, VehicleStatus)
    from std_msgs.msg import String
    from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference

    class FullNMPCNode(Node):
        def __init__(self):
            super().__init__("nmpc_full_smc_control")
            P = self.declare_parameter
            P("px4_ns", "uav_0"); P("controller_id", "nmpc_full"); P("active_at_start", False)
            # 10 Hz (100 ms budget). MEASURED: this 19-state solve is ~20 ms mean / 26 ms max on an
            # IDLE machine but 48-96 ms in HIL (Isaac+PX4+GS contend for CPU) -- 15 Hz (67 ms) still
            # overran, with ok=0 failures -> loop falls behind -> tumble. Shrinking N barely helped
            # (22->20 ms): the cost is the 19-state RK4, not the horizon. So BUY LOOP TIME instead.
            # latency_s is raised to match (0.25): under-compensation is fatal, over-comp is safe.
            # DEFAULT OFF: arming is the OPERATOR's, given through the ground station's
            # ARM / OFFBOARD / TAKEOFF buttons.  With this on the node re-arms whenever it
            # is active, which silently undoes a DISARM and arms the aircraft without
            # anyone asking.  (It exists because the first ARMs after a restart are
            # rejected while the EKF settles -- with it off you may need to press ARM twice.)
            P("auto_arm", False); P("arm_after", 150); P("rate_hz", 15.0)
            P("vcruise", 16.0); P("cap_deg", 90.0); P("alpha_max_deg", 8.0)
            # Handshake for the ground station: the cruise speed the TRIM SCHEDULE has actually
            # been built for.  `vcruise` alone is not enough to know the node is ready -- see
            # _refresh_schedule() for why the two can disagree.
            P("sched_ready_v", 0.0)
            # latency_s 0.25 -> 0.12: the solve is 17-34 ms in HIL, not the 43-96 ms this node
            # was tuned around, so 0.25 was over-compensating ~2x on a 100 ms loop.
            # 35 s, NOT 18.  ttr sets the speed ramp (Vc/ttr): 18 s = 0.89 m/s^2, which rings
            # the ~10 s phugoid and makes the aircraft porpoise 63-90 m for minutes (bag
            # full_tr20).  It also steepens the trim schedule's PITCH command, which is felt as a
            # harsh pitch at the cruise hand-over.  35 s = 0.46 m/s^2 is the gentle ramp that
            # produced 228 s of steady cruise.
            P("ttr", 40.0); P("settle_s", 2.0); P("latency_s", 0.12); P("cruise_alt", 30.0)
            # how far below z0 the vehicle must sink IN CRUISE before the reference gives up and
            # returns to hover.  1 m (the old implicit value) ended every wing-borne episode.
            P("cruise_alt_band", 15.0)
            # safety factor on the vertical-force tilt clamp: rotors + wing must be able to make
            # this multiple of the weight at the commanded tilt (1.0 = exactly hold altitude).
            P("lift_margin", 0.0)     # 0 = OFF (see the clamp below); 1.0 = exactly hold altitude
            # ROLLING TRANSITION (see build_refs): commanded sink while under-speed, and how much
            # altitude the manoeuvre is allowed to spend. sink=0 restores the old level reference.
            P("trans_sink", TRANS_SINK)          # m/s of commanded descent while below wingborne_v
            P("wingborne_v", WINGBORNE_V) # above this the wing carries the weight -> level off
            P("alt_auth_trans", ALT_AUTH_TRANS)   # see _alt_authority(); 1.0 = old behaviour
            P("descent_budget", DESCENT_BUDGET)     # never command below z0 - this
            # Fraction of the FRONT-pair max thrust (2 x 18 N = 36 N -- only the front rotors
            # tilt, so that is ALL the forward thrust this airframe has) that the reference
            # commands while accelerating toward cruise.  0 = old behaviour: the trim's forward
            # force is a RESIDUAL of the vertical/pitch solve, which asks ~0-10% at high tilt and
            # leaves the vehicle coasting at 12-14 m/s on the stall boundary (bag full_tr25).
            # ★ DEFAULT 0 -- DO NOT SET THIS (bag full_tr27).  Overriding the reference thrust
            # BREAKS THE TRIM: 28.8 N on the front pair against ~2 N on the rear is a huge pitch
            # moment and, at low tilt, a large vertical over-thrust, so (xref,uref) stopped being
            # a balanced state.  The solver fell back for the whole flight (625/625 level samples
            # at hover thrust) and it lost 84 m.  This is the SAME error as full_tr7: a trim is a
            # PACKAGE.  For a given speed in level flight the trim IS the trim -- thrust is not
            # independently commandable.  The correct way to demand forward force is
            # `so3.trim_split(..., Fx_req=M*a_cmd)`, which solves the TILT as a third unknown and
            # keeps vertical+pitch balance intact.  That is a Schedule change, not a uref override.
            P("accel_thr", 0.0)
            # How fast the COMMANDED pitch may move [deg/s].  The trim schedule's theta is a
            # function of speed, so a quick speed ramp makes the attitude command move quickly
            # too -- which is felt as a harsh pitch at the cruise hand-over.  0 = no limit.
            P("pitch_slew_deg_s", PITCH_SLEW_DEG_S)
            # Airspeed above which the tilt clamp is RELEASED so the rotors can commit to 90 deg
            # instead of trailing the trim schedule.  Below it the clamp still protects against
            # the low-speed over-tilt that has no pitch authority.  0 disables the release.
            P("tilt_commit_v", TILT_COMMIT_V)
            P("tilt_hold_deg", TILT_HOLD_DEG)
            P("tilt_hold_v", TILT_HOLD_V)
            P("tilt_commit_band", TILT_COMMIT_BAND)
            P("dive_deg", DIVE_DEG)
            P("aoa_max_deg", 0.0)
            # normalised surface units per second; 3.0 = full travel in ~0.33 s
            P("surf_slew", 0.0)
            # Speed at which the trim schedule's nose-up pitch is applied in FULL.  Below it the
            # commanded pitch is scaled by (V/this)^2 -- the same law the wing's lift follows --
            # so the aircraft stays near level while the TILT accelerates it, instead of pitching
            # up into drag at a speed where the wing cannot use the angle.  0 disables the gate.
            P("pitch_gate_v", PITCH_GATE_V)
            # HARD limit on body pitch during the transition [deg], enforced as a box bound on
            # sin(pitch) inside the NLP (not a cost).  Live-tunable; 0 disables it.
            P("pitch_max_deg", 15.0)
            # First-order filter on the applied tilt command [s].  The solver's raw tilt hunts
            # +-5 deg; this attenuates it instead of merely rate-limiting it.  0 = off.
            P("tilt_lpf_tau", 0.0)
            # BACK-TRANSITION (cruise -> hover).  Set `decel_now` to 1 to run the trim schedule
            # backwards; `decel_rate` is how fast the commanded speed bleeds off [m/s^2] and
            # `flare_deg` the extra NOSE-UP pitch added on top of the trim to actually brake.
            P("decel_now", 0.0); P("decel_rate", 1.5); P("flare_deg", 15.0)
            # how far the FRONT pair leans AFT while braking [deg], 0 = disabled (pitch only)
            P("beta_aft_deg", 20.0)
            # APPLIED-TILT SLEW LIMIT [deg/s]: HIL bag full1 showed the commanded tilt
            # thrashing 32->60->45->90 deg. The sim servo is RATE-limited (~80 deg/s) so the
            # vehicle never realises those steps, while our tilt-state estimate (first-order
            # lag) LEADS it -> the NMPC plans with the wrong thrust direction -> the nose-down
            # moment error. Slew-limit what we APPLY and estimate the SAME signal.
            # 15 deg/s, NOT 60.  The REFERENCE only moves the tilt at ~2.6 deg/s through the
            # whole transition (audited), so a 60 deg/s applied limit lets the solver swing the
            # nacelles 23x faster than anything is asking for -- every small tracking transient
            # becomes a violent tilt movement, which is the "aggressive" feel of the hover->cruise
            # leg.  15 deg/s still gives ~6x the reference rate for genuine corrections, and the
            # back-transition (90 -> -20 deg) then takes ~7 s instead of 1.5, which is gentler
            # anyway.  Raise it only if the aircraft cannot keep up with a commanded manoeuvre.
            P("tilt_slew_deg_s", 0.0)
            P("brake_accel", 2.0)   # decel used for the sqrt stopping profile [m/s^2]
            # How far the COMMANDED speed may lead the achieved speed [m/s]. The reference
            # advances on its own clock so it keeps demanding acceleration, but the lead is
            # bounded so it cannot wind up into a hopeless set-point if the vehicle genuinely
            # cannot follow (which is what an unbounded lead would do).
            # 3.0 is the measured best. HIL: lead 3 -> 8.45 m/s / 36% wing share;
            # lead 5 -> AoA swinging +-46 deg with solve failures (bag full_tr8).
            P("v_lead", 2.5)
            # ★ AIRSPEED TILT CLAMP -- the fix for the HIL pitch divergence (bag full2). The solver
            # tilted to 82-90 deg while the airspeed was still ~1 m/s. At high tilt the front rotors
            # are horizontal (almost no pitch moment) AND the elevator has no dynamic pressure, so
            # there is NO pitch authority left -> the nose fell (pitch -8 -> -17 -> -80 in 6 s, with
            # the elevator saturated at +1.00 fighting it) -> inverted. Clamp the APPLIED tilt to the
            # trim tilt for the CURRENT airspeed + margin, so the front pair is never asked to lift
            # and drive at once. Self-relaxes to 90 deg as speed builds. (Same clamp the SO(3) node
            # needed -- `tilt_margin_deg` -- which I failed to carry over into this node.)
            P("tilt_margin_deg", TILT_MARGIN_DEG)
            ns = self.get_parameter("px4_ns").value
            pre = f"/{ns}" if ns else ""
            self.cid = self.get_parameter("controller_id").value
            self._active = bool(self.get_parameter("active_at_start").value)
            self.auto_arm = bool(self.get_parameter("auto_arm").value)
            self.arm_after = int(self.get_parameter("arm_after").value)
            self.rate = float(self.gp("rate_hz"))
            self.get_logger().info("building FULL-STATE NMPC (JIT; cached after the first build)...")
            self.nmpc = FullNMPC()
            self._sched_key = (self.gp("vcruise"), self.gp("cap_deg"), self.gp("alpha_max_deg"))
            self.sched = Schedule(V_cruise=self._sched_key[0], tilt_cap_deg=self._sched_key[1],
                                  alpha_max_deg=self._sched_key[2])
            self.F = so3.build_ode()
            self._publish_sched_ready()
            self.get_logger().info("NMPC built.")
            self.state = None; self.p0 = None; self.heading = 0.0; self.z0 = 0.0
            self._v_cmd = 0.0
            self.smc_sp = None; self.armed = self.offb = False
            self.counter = self.last = 0; self._eng = 0; self.ubuf = []; self._solve_ms = 0.0
            self._settled = False; self._decel = False
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            self.motp = self.create_publisher(ActuatorMotors, f"{pre}/fmu/in/actuator_motors", qos)
            self.srvp = self.create_publisher(ActuatorServos, f"{pre}/fmu/in/actuator_servos", qos)
            self.ocm = self.create_publisher(OffboardControlMode, f"{pre}/fmu/in/offboard_control_mode", qos)
            self.cmdp = self.create_publisher(VehicleCommand, f"{pre}/fmu/in/vehicle_command", qos)
            self.create_subscription(VehicleStatus, f"{pre}/fmu/out/vehicle_status_v1", self.on_status, qos)
            self.create_subscription(String, f"{pre}/ground_station/active_controller", self.on_active, 10)
            self.create_subscription(Mocap, f"{pre}/mocap", self.on_mocap, qos_profile_sensor_data)
            # OUTER POSITIONAL SMC feed: the converging set-point (destination + velocity FF).
            # Launch-remapped to .../reference_smc so smc_outer_node drives it.
            self.create_subscription(PositionControllerReference,
                                     f"{pre}/fsc_autopilot_ros2/position_controller/reference",
                                     self.on_ref, 10)
            self.create_timer(1.0 / self.rate, self.loop)
            self.get_logger().info(f"nmpc_full_smc up: id={self.cid} rate={self.rate:.0f}Hz")

        def gp(self, n): return float(self.get_parameter(n).value)
        def now_us(self): return int(self.get_clock().now().nanoseconds / 1000)
        def on_status(self, m):
            self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            self.offb = (m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        def on_active(self, m): self._active = (m.data == self.cid)
        def on_ref(self, m): self.smc_sp = np.array([m.position.x, m.position.y, m.position.z])
        def on_mocap(self, m):
            q = m.pose.orientation
            self.state = (np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]),
                          _q2R(q.x, q.y, q.z, q.w),
                          np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]),
                          np.array([m.twist.angular.x, m.twist.angular.y, m.twist.angular.z]))

        def _publish_sched_ready(self):
            """Report the cruise speed the trim schedule is actually built for."""
            try:
                v = float(getattr(self, "_sched_key", (0.0,))[0])
                if abs(float(self.gp("sched_ready_v")) - v) > 1e-6:
                    self.set_parameters([rclpy.parameter.Parameter(
                        "sched_ready_v", rclpy.Parameter.Type.DOUBLE, v)])
            except Exception as e:
                self.get_logger().warn("could not publish sched_ready_v: %s" % e)

        def _refresh_schedule(self):
            """Rebuild the trim schedule if the commanded cruise speed changed.

            ★ `vcruise` DID NOT USED TO REACH THE SCHEDULE.  `self.sched` was built once in the
            constructor from the startup value, so setting `vcruise` later moved the speed TARGET
            (Vc_p, the ramp destination) while the trim schedule -- tilt, pitch, thrust split and
            elevator as a function of speed -- stayed at the old cruise speed.  The reference then
            stopped being self-consistent: the ramp aimed at one speed while every trim it handed the
            solver was solved for another.  A trim is a package (bags full_tr7, full_tr25/27).
            Rebuilt only while STANDBY, never mid-flight -- swapping the schedule under a flying
            aircraft would step the whole reference.  `sched_ready_v` reports what is actually built
            so the ground station can wait for the rebuild instead of guessing.
            """
            if getattr(self, "_active", False):
                return
            key = (self.gp("vcruise"), self.gp("cap_deg"), self.gp("alpha_max_deg"))
            if key == getattr(self, "_sched_key", None):
                # ★ STILL PUBLISH THE HANDSHAKE WHEN NOTHING NEEDS REBUILDING.
                # Returning silently here meant the COMMONEST case never answered: the node boots at
                # vcruise=16, the operator asks for 16, the key is unchanged, no rebuild happens --
                # and `sched_ready_v` stayed at its 0.0 default forever, so the ground station polled
                # until it timed out with "timed out waiting for the trim schedule" on a node that
                # was in fact ready.  Readiness is a STATE to report, not an event to emit.
                self._publish_sched_ready()
                return
            try:
                self.sched = Schedule(V_cruise=key[0], tilt_cap_deg=key[1], alpha_max_deg=key[2])
                self._sched_key = key
                self._publish_sched_ready()
                self.get_logger().info("trim schedule rebuilt for Vc=%.1f m/s" % key[0])
            except Exception as e:
                self.get_logger().error("schedule rebuild FAILED for Vc=%.1f: %s" % (key[0], e))

        def loop(self):
            self.counter += 1
            # Answer the ground station's readiness handshake BEFORE the state guard below.
            # It used to sit after it, so with no mocap yet the node never reported the cruise
            # speed its schedule was built for -- and the ground station's build timed out on a
            # node that was perfectly healthy, just not receiving pose.
            self._refresh_schedule()
            o = OffboardControlMode(); o.direct_actuator = True; o.timestamp = self.now_us()
            if self._active: self.ocm.publish(o)
            if self.state is None:
                if self._active: self.pub([math.nan] * 4, [math.nan] * 5)
                return
            p, R, v, w = self.state
            if not self._active:
                self.p0 = None; return
            if self.p0 is None:
                self.heading = math.atan2(R[1, 0], R[0, 0]); self.p0 = p.copy()
                self.z0 = max(float(p[2]), self.gp("cruise_alt"))   # climb to a safe altitude first
                self._eng = 0; self.ubuf = []; self._v_cmd = float(np.linalg.norm(v))
                self._settled = False; self._decel = False; self._th_ref = None
                # ★ RESET EVERY CARRIED REFERENCE/COMMAND STATE ON ENGAGE.
                # These are rate-limiter memories. Left over from the previous flight, a fresh
                # engage inherits them: seen live, the reference tilt read 27 deg during SETTLE
                # (where the commanded speed is 0 and the schedule tilt is 0) because _b_ref was
                # still carrying the last flight's value and slewing down from it.
                self._b_ref = None; self._beta_cmd = 0.0
                self._ail_cmd = 0.0; self._rud_cmd = 0.0
                self.get_logger().info("FULL-NMPC ENGAGED at (%.1f,%.1f,%.1f) heading=%.0f -> z0=%.1f Vc=%.0f"
                                       % (p[0], p[1], p[2], math.degrees(self.heading), self.z0,
                                          self.gp("vcruise")))
            self._eng += 1
            dt = 1.0 / self.rate
            x = np.concatenate([p, R.reshape(-1), [self._beta_est(v)], v, w])
            lat_n = max(1, int(round(self.gp("latency_s") * self.rate)))
            if self.ubuf and self.gp("latency_s") > 1e-3:
                x = so3.predict_forward(self.F, x, self.ubuf[-lat_n:], dt)
            V_now = float(np.linalg.norm(x[13:16])); z = float(p[2])
            # ★ THE BACK-TRANSITION IS PART OF THE REFERENCE, not only a manual trigger.
            # It arms either explicitly (`decel_now`, e.g. the GS STOP button) OR automatically
            # once the destination is inside the braking distance, so a mission that flies to a
            # point decelerates on the trim schedule by itself instead of arriving at 16 m/s and
            # being handed to the multicopter controller (bag full_tr30: tilt snapped to 0, lift
            # doubled, it stalled at AoA +78 deg and departed, 65.7 m excursion).
            # Distance to stop from V at `decel_rate`, plus a margin for the flare to take effect.
            # ★ EXPLICIT ONLY.  This used to ALSO auto-arm when the destination came inside the
            # braking distance -- but it measured against `self.smc_sp`, which is the SMC's
            # CONVERGING SET-POINT (it leads by only reset_dist, 8-30 m), not the real
            # destination.  d_rem was therefore always tiny, the brake armed during the
            # climb-out, and the reference commanded AFT tilt in the middle of the transition:
            # tilt swung 27 -> -7 deg with the elevator saturating at +-1.00 and the speed stuck
            # at 4 m/s.  The mission (and the GS STOP button) set `decel_now` at the right
            # moment, so nothing needs to guess.  If this is ever re-added it must use the FINAL
            # destination, not the SMC set-point.
            self._decel = self.gp("decel_now") > 0.5
            self.nmpc.pitch_max_rad = math.radians(max(0.0, self.gp("pitch_max_deg")))
            fwd = np.array([math.cos(self.heading), math.sin(self.heading), 0.0])
            s_now = float(np.dot(x[0:3] - self.p0, fwd))
            # Hover until settled AND at altitude -- a low engage cannot crash it.
            #
            # ★ THIS GATE IS ONE-WAY (bag full_tr15).  It used to be re-evaluated every tick, so a
            # 1 m altitude dip in CRUISE threw the vehicle back to the hover reference: tilt was
            # commanded to 0 at 12 m/s (the documented unsurvivable manoeuvre) and `_v_cmd` was
            # re-anchored to the achieved speed.  That is what ended both wing-borne episodes in
            # full_tr15 after ~4 s -- the flight was holding altitude to within 1 m (19.0-21.1 with
            # z0=20) and the REFERENCE punished it for touching the edge of that band.  It is also
            # the FOURTH instance in this project of a reference surrendering to its own tracking
            # error (SMC re-seed C7, GS mission clock, build_refs vtar, now this).
            #
            # So: the pre-transition gate latches OFF once satisfied, and in cruise we only fall
            # back to hover on a REAL altitude loss (`cruise_alt_band`, default 6 m), not on the
            # 1 m ripple a wing-borne transition normally produces.
            if not self._settled:
                settling = (self._eng < int(self.gp("settle_s") * self.rate)) or (z < self.z0 - 1.0)
                if not settling:
                    self._settled = True
            else:
                settling = z < self.z0 - self.gp("cruise_alt_band")
            # COMMANDED speed on its own clock (see build_refs). While settling it tracks the
            # vehicle so the engage is bumpless; after that it ramps toward Vc regardless of
            # what the vehicle achieves, bounded to v_lead ahead of it.
            Vc_p = self.gp("vcruise")
            if settling:
                self._v_cmd = V_now
            else:
                if self._decel:
                    # ★ THE DECEL REFERENCE MUST NOT RUN AHEAD OF THE VEHICLE.
                    # The forward ramp is clamped to V_now + v_lead; the backward ramp had no
                    # bound at all, so it simply ran its clock to zero. Flown (bag night
                    # baseline): solves began failing at the start of the back-transition, the
                    # fallback held ~0.1 throttle, and with tilt at 90 deg the only braking force
                    # is 2.9 N of drag -- the aircraft coasted at 15.9 m/s while the reference
                    # arrived at a standstill and handed back tilt=0. The state was then 90 deg
                    # and 16 m/s from its own reference; the next solve that converged commanded
                    # mot=[0.00,0.58,0.86,0.00], rolled it past 110 deg and it hit the ground --
                    # after 70 s of textbook wing-borne cruise.
                    # Same bounded-lead rule, mirrored: never ask for more than v_lead below the
                    # measured speed. If the aircraft is not slowing, the reference waits for it.
                    self._v_cmd = max(self._v_cmd - self.gp("decel_rate") * dt, 0.0)
                    self._v_cmd = max(self._v_cmd, V_now - self.gp("v_lead"))
                else:
                    self._v_cmd = min(self._v_cmd + (Vc_p / max(self.gp("ttr"), 1e-3)) * dt, Vc_p)
                    self._v_cmd = max(0.0, min(self._v_cmd, V_now + self.gp("v_lead")))
            xrefs, urefs, self._th_ref, self._b_ref = build_refs(self.sched, self.nmpc, self.p0, self.heading, self.z0,
                                      self._v_cmd, Vc_p, self.gp("ttr"), s_now, settling,
                                      v_meas=V_now,
                                      v_wb=self.gp("wingborne_v"),
                                      sink=self.gp("trans_sink"),
                                      z_floor=self.z0 - self.gp("descent_budget"),
                                      accel_thr=self.gp("accel_thr"),
                                      decel=self._decel,
                                      decel_rate=self.gp("decel_rate"),
                                      flare_deg=self.gp("flare_deg"),
                                      beta_aft=math.radians(self.gp("beta_aft_deg")),
                                      th_prev=getattr(self, "_th_ref", None),
                                      th_slew=self.gp("pitch_slew_deg_s"), dt_ref=dt,
                                      pitch_gate_v=self.gp("pitch_gate_v"),
                                      tilt_hold=self.gp("tilt_hold_deg"),
                                      tilt_hold_v=self.gp("tilt_hold_v"),
                                      tilt_commit_v=self.gp("tilt_commit_v"),
                                      tilt_commit_band=self.gp("tilt_commit_band"),
                                      tilt_margin=math.radians(self.gp("tilt_margin_deg")),
                                      b_prev=getattr(self, "_b_ref", None),
                                      b_slew=self.gp("tilt_slew_deg_s"),
                                      dive_deg=self.gp("dive_deg"))
            # OUTER SMC: cap the along-path reference at the SMC destination (so the NMPC flies the
            # wing-borne schedule TOWARD the commanded point instead of forward forever).
            # ROOT CAUSE of the HIL transition failure (bag full1): the old clamp ZEROED the
            # velocity reference (xr[13:16]=0) for every stage past the SMC set-point. But the SMC
            # set-point is a CONVERGING one that leads by only reset_dist (8-30 m) and re-seeds, so
            # the NMPC got a CONTRADICTORY reference -- the schedule said "tilt to 16 m/s cruise"
            # while pos/vel said "stop here, v=0". Result: it could never accelerate (capped ~3.6 m/s)
            # and the attitude loop fought the tilt -> nose-down -> tumble.
            # FIX: only brake when genuinely approaching the DESTINATION. Outside the braking
            # distance the schedule runs free; inside it, ramp the speed down with a sqrt profile.
            if self.smc_sp is not None:
                d_rem = float(np.dot(self.smc_sp - x[0:3], fwd))       # along-path distance to go
                a_br = self.gp("brake_accel")
                for xr_ in xrefs:
                    s_k = float(np.dot(xr_[0:3] - x[0:3], fwd))        # stage distance ahead of us
                    rem = d_rem - s_k
                    v_cap = math.sqrt(max(2.0 * a_br * max(rem, 0.0), 0.0))   # sqrt stopping profile
                    v_ref = float(np.linalg.norm(xr_[13:16]))
                    if v_ref > v_cap:                                  # brake only when needed
                        xr_[13:16] = fwd * v_cap
                    if rem < 0.0:                                      # never command past the target
                        xr_[0:3] = self.smc_sp.copy(); xr_[2] = self.z0
            import time as _t
            _t0 = _t.perf_counter()
            self.nmpc.aoa_max_deg = self.gp("aoa_max_deg")
            alt_auth = _alt_authority(V_now, self.gp("wingborne_v"),
                                      self.gp("alt_auth_trans"), self._settled)
            self._alt_auth = alt_auth
            u, ok = self.nmpc.solve(x, xrefs, urefs, alt_auth)
            self._solve_ms = (_t.perf_counter() - _t0) * 1e3
            self.ubuf.append(u.copy()); self.ubuf = self.ubuf[-lat_n:]
            self._beta_cmd = apply_tilt_limits(
                u[4], V_now, self.sched, getattr(self, "_beta_cmd", 0.0), dt,
                self.gp("tilt_margin_deg"), self.gp("tilt_commit_v"),
                self.gp("tilt_lpf_tau"), self.gp("tilt_slew_deg_s"),
                self.gp("lift_margin"))
            # actuators come STRAIGHT from the NMPC (roll/yaw/pitch all allocated inside the solve)
            mot = [_cl(u[i] / MAX_THRUST, 0.0, 1.0) for i in range(4)]
            # servo command is anchored at zero (see vtol._tilt_target_rad): 0 -> 0 deg,
            # 1 -> 90 deg, and NEGATIVE values give aft tilt down to the airframe's tilt_min.
            tservo = _cl(self._beta_cmd / (math.pi / 2), -BETA_MIN / (math.pi / 2), 1.0)
            elevator = _cl(u[5] / DE_MAX, -1.0, 1.0)
            # ★ RATE-LIMIT THE SURFACES.  They were the ONLY actuator with no limit -- tilt has
            # a slew limit AND a low-pass, the pitch and tilt REFERENCES are both slew-limited,
            # but aileron and rudder went straight from the solver to the servos.
            # MEASURED, in both crashed back-transitions (bags bt_flare25 and bt_lead_tight, the
            # traces are near-identical): the aircraft cruises trimmed at 90 deg tilt with a
            # standing -6 deg yaw error and ZERO surface command, then the instant the tilt
            # starts down the rudder steps 0.00 -> +1.00 in ONE tick. At 15.9 m/s full rudder
            # makes enough sideslip to roll it through dihedral: roll went +2 -> -96 deg in
            # 0.6 s, and the solver dropped out entirely by -47.
            # This is a step input, not marginal stability. Full travel in ~0.3 s still allows a
            # fast correction while making that step impossible.
            a_raw = _cl(u[6] / DA_MAX, -1.0, 1.0)
            r_raw = _cl(u[7] / DR_MAX, -1.0, 1.0)
            sslew = self.gp("surf_slew") * dt
            pa = getattr(self, "_ail_cmd", 0.0); pr = getattr(self, "_rud_cmd", 0.0)
            if sslew > 0.0:
                a_raw = _cl(a_raw, pa - sslew, pa + sslew)
                r_raw = _cl(r_raw, pr - sslew, pr + sslew)
            self._ail_cmd = a_raw; self._rud_cmd = r_raw
            aileron = a_raw
            rudder = r_raw
            self.pub(mot, [tservo, tservo, aileron, elevator, rudder])
            if self.counter % max(1, int(self.rate / 2)) == 0:
                vb = R.T @ v
                aoa = math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-6)) if V_now > 1.0 else 0.0
                pitch = math.degrees(math.asin(_cl(R[2, 0], -1, 1)))      # nose-up +
                roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
                ye = math.atan2(R[1, 0], R[0, 0]) - self.heading
                yaw = math.degrees(math.atan2(math.sin(ye), math.cos(ye)))
                self.get_logger().info(
                    "FULL %s V=%.1f tilt=%.0f/%.0f | pitch=%+.0f/%+.0f AoA=%+.0f roll=%+.0f yaw=%+.0f | z=%.1f "
                    "aa=%.2f ok=%d %.0fms mot=[%.2f,%.2f,%.2f,%.2f] de=%+.2f da=%+.2f dr=%+.2f%s"
                    % ("settle" if settling else "fly", V_now, math.degrees(self._beta_cmd),
                       math.degrees(xrefs[0][12]), pitch,
                       math.degrees(math.asin(_cl(xrefs[0][3:12].reshape(3, 3)[2, 0], -1, 1))),
                       aoa, roll, yaw, z, getattr(self, "_alt_auth", 1.0), int(ok), self._solve_ms,
                       mot[0], mot[1], mot[2], mot[3], elevator, aileron, rudder,
                       # ★ WHY the solve failed, in the STATUS LINE so it reaches /rosout and
                       # therefore the bag. `Maximum_Iterations_Exceeded`, `Restoration_Failed`
                       # and `Infeasible_Problem_Detected` have completely different fixes, and
                       # the previous debug print went to stdout only -- visible in the tmux pane,
                       # absent from every bag, so post-flight analysis could never tell them
                       # apart. Solve failures are the ENTRY cause of the corridor departure
                       # (they begin at 6 m/s and are persistent by 9), so this is the one field
                       # that decides what to fix next.
                       ("" if ok else " why=" + str(getattr(self.nmpc, "_last_status", "?"))[:34])))
            # auto_arm read LIVE (see geometric_hover_node): a cached value makes a GS DISARM
            # impossible, because the node re-arms on the next tick.
            if (self._active and bool(self.get_parameter("auto_arm").value)
                    and self.counter >= self.arm_after and not (self.armed and self.offb)):
                if self.counter - self.last >= 50:
                    self.last = self.counter
                    if not self.offb:
                        self.vcmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    elif not self.armed:
                        self.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        def _beta_est(self, v):
            """Tilt state estimate: the mocap doesn't report the servo, so integrate the commanded
            tilt through the servo lag (same first-order model the plant uses)."""
            b = getattr(self, "_beta_state", 0.0)
            cmd = getattr(self, "_beta_cmd", 0.0)
            step = (cmd - b) * (1.0 / self.rate) / so3.TAU_BETA
            lim = math.radians(80.0) / self.rate      # the SIM servo's rate limit
            b += _cl(step, -lim, lim)
            self._beta_state = b
            return b

        def pub(self, mot, srv):
            if not self._active: return
            mm = ActuatorMotors(); c = [math.nan] * 12
            for i in range(4): c[i] = mot[i]
            mm.control = c; mm.timestamp = mm.timestamp_sample = self.now_us(); self.motp.publish(mm)
            ss = ActuatorServos(); sc = [math.nan] * 8
            for i in range(5): sc[i] = srv[i]
            ss.control = sc; ss.timestamp = ss.timestamp_sample = self.now_us(); self.srvp.publish(ss)

        def vcmd(self, cmd, p1=0.0, p2=0.0):
            m = VehicleCommand(); m.command = cmd; m.param1 = p1; m.param2 = p2
            m.target_system = m.target_component = m.source_system = m.source_component = 1
            m.from_external = True; m.timestamp = self.now_us(); self.cmdp.publish(m)

    rclpy.init()
    try:
        rclpy.spin(FullNMPCNode())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    if "--test" in sys.argv:
        offline_test()
    else:
        run_ros()

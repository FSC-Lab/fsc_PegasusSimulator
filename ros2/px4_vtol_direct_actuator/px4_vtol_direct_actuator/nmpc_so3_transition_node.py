#!/usr/bin/env python3
"""SO(3) path-following transition NMPC for the FSC quad-tiltrotor (direct_actuator, mocap-based).

This is a PORT of the AVTOL SO(3) path-following NMPC
(PhDStudies/.../Aircraft_Model_SO3_pathfollowing/nmpc_vtol_reduced_pos_att_so3.py) onto the FSC
quad-tiltrotor dynamics we built into Isaac/Pegasus. The controller STRUCTURE is kept faithfully:
a full 6-DOF rigid-body predictor (RK4), a cost tracking position + SO(3) attitude + velocity + tilt
against a hover->cruise transition REFERENCE, with the horizon anchored on the path progress derived
from the current tilt (s_from_beta). Only the PLANT is swapped: the AVTOL (1325 kg, 2 tilt rotors,
slosh) -> the FSC quad (3.5 kg, 4 rotors = 2 front-tilt + 2 rear-static, no slosh), with the aero
model + coefficients matched to the sim's vtol_aerodynamics.py.

State x (19):  [pos(3), R_bn(9, row-major), beta(front tilt), vel(3, inertial), omega(3, body)]
Control u (8): [T_FL, T_FR, T_RL, T_RR, betacmd, de, da, dr]
Tilt convention (Isaac sim): beta=0 -> rotors UP (hover); beta=pi/2 -> rotors FORWARD (cruise).
Rotor body thrust = T*[sin(tilt), 0, cos(tilt)] at the rotor position (front tilt=beta, rear tilt=0).

Output: ActuatorMotors + ActuatorServos + OffboardControlMode(direct_actuator). Gated on
/<ns>/ground_station/active_controller == controller_id.
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
import numpy as np
import casadi as ca

# ============================ FSC quad-tiltrotor parameters ============================
M = 3.45; G = 9.81
# ASSEMBLED-VEHICLE inertia about the CG, not the body link's. VTOLConfig.inertia describes
# the BODY LINK only; PhysX assembles the vehicle from that plus ten 0.05 kg articulated
# links at up to 0.6 m from the axis. Using the body-link value under-modelled Ixx by 24%
# and Izz by 29% -- the two axes that stayed fragile through every transition attempt --
# and the delay compensator integrates the attitude forward through the same number.
# Reproduce with tests/plant_double.py (2026-09-06 controller audit, finding C2).
IXX, IYY, IZZ = 0.265, 0.303, 0.490
MAX_THRUST = 18.0                            # per rotor [N]
CG_X = 0.176   # assembled CG: body link (2.95 kg at x=0.195) + 10 links -> 0.176. The sim's
#   own balanced hover confirms it: front 8.6 N > rear 8.4 N is exactly the split that
#   zeroes the pitch moment about x=0.176.
# Rotor HUB positions (where vtol.py applies thrust), CG-relative, body FLU. The z arms were
# all +0.02 -- the front pair is really BELOW the CG at -0.033, i.e. the wrong SIGN. Zero
# effect in hover (thrust is vertical, so a z arm makes no pitch moment) but it IS the pitch
# moment at full tilt, where the thrust is horizontal and My = z_arm * Fx (finding C4).
RPOS = np.array([[0.459 - CG_X,  0.284, -0.033],   # FL (front, tiltable)
                 [0.459 - CG_X, -0.284, -0.031],   # FR (front, tiltable)
                 [-0.112 - CG_X, 0.284,  0.016],   # RL (rear, static)
                 [-0.112 - CG_X,-0.284,  0.018]])  # RR (rear, static)
SPIN = np.array([+1.0, -1.0, -1.0, +1.0])    # FL,FR,RL,RR
TILTABLE = np.array([1.0, 1.0, 0.0, 0.0])    # front pair tilts, rear pair static
CQ = 0.12                                    # = VTOLConfig.rotor_moment_ratio (was 0.06,
#   i.e. HALF the plant's yaw torque -- and in hover this is the only yaw actuator; C3)
TAU_BETA = 0.12                              # tilt servo time constant [s] (small-error slope)
TILT_RATE = math.radians(80.0)               # = RotorConfig.tilt_rate_deg_s -- the sim
#   slews the nacelle at this rate and does nothing else, so the MODEL must saturate here.

# ============================================================================
#  AERODYNAMICS -- STANDARD (FRD / stability-axis) coefficients.
#  Mirrors extensions/.../dynamics/vtol_aerodynamics.py EXACTLY; the equivalence is
#  asserted by tests/test_controller_aero_match.py. Do not "fix" a sign here: the frame
#  conversion happens at the boundary of aero_body(), so every value below is the
#  published one. See the frame note at the top of vtol_aerodynamics.py.
# ============================================================================
RHO = 1.225; S_WING = 0.35; B_WING = 1.4; C_WING = 0.25
CL0 = 0.15; CLA = 4.8; CLQ = 7.0; CLDE = 0.70
CD0 = 0.04; KIND = 0.071; CDB2 = 0.20
CYB = -0.98; CYDR = 0.24; CY_MAX = 0.6
CLB = -0.12; CLDA = 0.49; CLP = -0.5; CLR = 0.1; CLDR = 0.02
CM0 = 0.02; CMA = -1.2; CMQ = -12.0; CMDE = -1.75
CNB = 0.11; CNDR = -0.109; CNR = -0.2; CNP = -0.06; CNDA = 0.02
STALL = math.radians(13.0); BETA_BREAK = math.radians(20.0)
BLEND_M = 25.0; MIN_V = 0.25

NX = 19; NU = 8
THETA_FADE_S = 0.15   # path progress by which the trim-pitch attitude ref is fully faded in
SO3_ALPHA_MAX = 8.0   # profile trim alpha_max [deg] -- lower = trim further from the stall break
S_LEASH = 0.15        # time-ratchet leash: max s the anchor may lead the tilt-derived progress
SPEED_LEASH = 0.04    # max s the tilt may lead the SPEED's trim progress -- keeps tilt<->speed
#                       coordinated so the tilt can't outrun the airspeed (wing can't carry weight
#                       yet -> stall in the corridor). The corrected (13deg) stall makes this bite.
V_THETA_FULL = 12.0   # PITCH FEED-FORWARD: the trim body-pitch (~+8deg nose-up) is only ACHIEVABLE
#                       once the airspeed gives the elevator/wing authority. Below this speed the
#                       tilt-rotor pitch coupling drives the body nose-DOWN (wing downforce -> the
#                       corridor infeasibility). So SCALE the commanded pitch by the ACHIEVED speed:
#                       stay ~level (rotor-borne) while slow, ramp to the trim pitch by V_THETA_FULL.


def skew(w):
    return ca.vertcat(ca.horzcat(0, -w[2], w[1]),
                      ca.horzcat(w[2], 0, -w[0]),
                      ca.horzcat(-w[1], w[0], 0))


def aero_body(vb, omega, de, da, dr):
    """Body-frame aero force + moment. FLU in, FLU out; STANDARD coefficients inside.

    FLU -> FRD is a 180 deg rotation about x, so v, w, q, r (and hence alpha, beta) flip,
    as do Fy, Fz, My, Mz, while u, p and Mx do not. Converting once at each boundary means
    every coefficient above is its published value and no term ever needs a sign patch --
    which is what went wrong before (six of them were patched individually and got it
    wrong, leaving a wing that lifted only at NEGATIVE angle of attack).

    The old forward-speed gate u^2/(u^2+4) is gone. It existed because a LINEAR coefficient
    model is nonsense outside small alpha, so it was faded out at low speed -- but that also
    made every controller blind to vertical descent and lateral translation, where the plant
    produces real forces (16 N in a 6 m/s crosswind against 0 N modelled). The blended
    flat-plate model below is well behaved at every angle, so the gate is unnecessary; the
    plant's own low-speed cutoff is reproduced by flooring V at MIN_V.
    """
    u = vb[0]; v = -vb[1]; w = -vb[2]                       # FLU -> FRD
    p = omega[0]; q = -omega[1]; r = -omega[2]
    # Floor V instead of gating: keeps qbar ~ 0 at rest AND keeps the rate
    # non-dimensionalisation (p*b/2V etc.) bounded, smoothly, for the solver.
    V = ca.sqrt(u * u + v * v + w * w + MIN_V * MIN_V)
    alpha = ca.atan2(w, ca.sqrt(u * u + 1e-4))              # smooth |u|; the NMPC never flies backwards
    beta = ca.asin(ca.fmax(ca.fmin(v / V, 0.99), -0.99))
    qbar = 0.5 * RHO * V * V
    phat = p * B_WING / (2 * V); qhat = q * C_WING / (2 * V); rhat = r * B_WING / (2 * V)

    # linear model blended into a Newtonian flat plate (tanh form of the Beard & McLain
    # sigmoid -- identical to the plant's, and it saturates instead of overflowing)
    sig = 0.5 * (1.0 + ca.tanh(0.5 * BLEND_M * (ca.fabs(alpha) - STALL)))
    sig_b = 0.5 * (1.0 + ca.tanh(0.5 * BLEND_M * (ca.fabs(beta) - BETA_BREAK)))
    sa = ca.sin(alpha); ca_ = ca.cos(alpha)
    CL_lin = CL0 + CLA * alpha
    CL = ((1 - sig) * CL_lin + sig * 2.0 * ca.tanh(50.0 * alpha) * sa * sa * ca_
          + CLQ * qhat + CLDE * de)
    CD = CD0 + (1 - sig) * KIND * CL_lin * CL_lin + sig * 2.0 * sa * sa \
        + (1 - sig_b) * CDB2 * beta * beta
    CY = ((1 - sig_b) * (CYB * beta + CYDR * dr)
          - sig_b * CY_MAX * ca.tanh(50.0 * beta) * ca.sin(beta) ** 2)

    L = qbar * S_WING * CL; D = qbar * S_WING * CD; Y = qbar * S_WING * CY
    vdir = ca.vertcat(u, v, w) / V
    ldir = ca.vertcat(sa, 0, -ca_)                          # "up" is -z in FRD
    F_frd = -D * vdir + L * ldir + ca.vertcat(0, Y, 0)

    a_att = ca.fmax(ca.fmin(alpha, STALL), -STALL)          # moments: attached flow only
    Cl_ = CLB * beta + CLDA * da + CLP * phat + CLR * rhat + CLDR * dr
    Cm = CM0 + CMA * a_att + CMDE * de + CMQ * qhat
    Cn = CNB * beta + CNDR * dr + CNR * rhat + CNP * phat + CNDA * da
    M_frd = qbar * S_WING * ca.vertcat(B_WING * Cl_, C_WING * Cm, B_WING * Cn)

    Fb = ca.vertcat(F_frd[0], -F_frd[1], -F_frd[2])         # FRD -> FLU
    Mm = ca.vertcat(M_frd[0], -M_frd[1], -M_frd[2])
    return Fb, Mm



# ============================================================================
#  THE THRUST SPLIT -- one implementation, used by every reference generator.
# ============================================================================
def _aero_numeric():
    """Numeric handle on aero_body, so a reference can be balanced against the forces the
    vehicle will actually feel rather than against a simplified stand-in."""
    vb = ca.SX.sym("vb", 3); om = ca.SX.sym("om", 3)
    de = ca.SX.sym("de"); da = ca.SX.sym("da"); dr = ca.SX.sym("dr")
    F, Mm = aero_body(vb, om, de, da, dr)
    return ca.Function("aero_num", [vb, om, de, da, dr], [F, Mm])


_AERO_NUM = None


def _split_at_beta(beta, theta, V, de, Fz_req, gamma):
    """The two thrusts that meet the vertical demand and zero the pitch moment at a GIVEN
    tilt, plus the forward force that falls out of them."""
    global _AERO_NUM
    if _AERO_NUM is None:
        _AERO_NUM = _aero_numeric()
    xf, zf, xr = RPOS[0, 0], RPOS[0, 2], RPOS[2, 0]
    c_t, s_t = math.cos(theta), math.sin(theta)
    v_world = np.array([V * math.cos(gamma), 0.0, V * math.sin(gamma)])
    R = np.array([[c_t, 0.0, -s_t], [0.0, 1.0, 0.0], [s_t, 0.0, c_t]])
    vb = R.T @ v_world
    Fa, Ma = _AERO_NUM(ca.DM(vb), ca.DM([0.0, 0.0, 0.0]), float(de), 0.0, 0.0)
    Fa = np.array(Fa).ravel(); Ma = np.array(Ma).ravel()
    A = np.array([[math.cos(beta - theta), c_t],
                  [zf * math.sin(beta) - xf * math.cos(beta), -xr]])
    rhs = np.array([Fz_req - float(R[2, :] @ Fa), -Ma[1]])
    try:
        Tf, Tr = np.linalg.solve(A, rhs)
    except np.linalg.LinAlgError:
        Tf = Tr = 0.5 * Fz_req
    Tmax = 2.0 * MAX_THRUST                           # per PAIR
    Tf = float(np.clip(Tf, 0.0, Tmax)); Tr = float(np.clip(Tr, 0.0, Tmax))
    # forward (world-x) force: front thrust leans by (beta - theta), rear by (-theta)
    Fx = Tf * math.sin(beta - theta) - Tr * math.sin(theta) + float(R[0, :] @ Fa)
    return Tf, Tr, Fx


def trim_split(beta, theta, V, de, Fz_req=None, gamma=0.0, Fx_req=None, beta_max=None):
    """Front/rear PAIR thrusts that meet the vertical force demand AND zero the pitch moment.

    The AVTOL trim generator that supplies (tilt, alpha, theta, de) is written for a
    tiltrotor whose WHOLE rotor set rotates, and it returns a single Ttot on that premise.
    This airframe tilts only the FRONT pair; mid-corridor that under-counts the vertical
    thrust by up to 7.8 N -- 23% of the aircraft's weight. So take the generator's ATTITUDE
    schedule, which is what it is good at, and solve the two thrusts from THIS geometry:

        vertical:  Tf*cos(beta - theta) + Tr*cos(theta) + Fz_aero = Fz_req
        pitch:     Tf*(zf*sin b - xf*cos b) - xr*Tr + My_aero_FLU = 0

    Fz_req defaults to the weight (level flight); a climbing or descending reference passes
    M*(G + z_ddot). Note My_aero comes back NOSE-UP positive from the standard coefficients
    and is negated here into the FLU convention the thrust moment is written in.

    Before this existed, references left up to 5.3 rad/s^2 of residual pitch acceleration and
    10 m/s^2 of vertical -- they were not trims at all (2026-09-06 reference audit).
    """
    if Fz_req is None:
        Fz_req = M * G
    if Fx_req is None:
        Tf, Tr, _ = _split_at_beta(beta, theta, V, de, Fz_req, gamma)
        return Tf, Tr
    # A FORWARD demand needs the third freedom. Vertical and pitch alone fix both thrusts,
    # so the forward force is then whatever falls out -- which is why a reference that also
    # commanded an acceleration was not met (+2.42 m/s^2 asked, +1.13 delivered). Solve the
    # tilt as well: Fx is monotone in beta over [0, beta_max], so bisect.
    if beta_max is None:
        beta_max = math.pi / 2
    lo, hi = 0.0, float(beta_max)
    f_lo = _split_at_beta(lo, theta, V, de, Fz_req, gamma)[2]
    f_hi = _split_at_beta(hi, theta, V, de, Fz_req, gamma)[2]
    if Fx_req <= f_lo:                                # cannot brake below beta = 0
        Tf, Tr, _ = _split_at_beta(lo, theta, V, de, Fz_req, gamma)
        return Tf, Tr, lo
    if Fx_req >= f_hi:                                # saturated at full tilt
        Tf, Tr, _ = _split_at_beta(hi, theta, V, de, Fz_req, gamma)
        return Tf, Tr, hi
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if _split_at_beta(mid, theta, V, de, Fz_req, gamma)[2] < Fx_req:
            lo = mid
        else:
            hi = mid
    b = 0.5 * (lo + hi)
    Tf, Tr, _ = _split_at_beta(b, theta, V, de, Fz_req, gamma)
    return Tf, Tr, b


# ============================ PROPELLER: thrust <-> throttle ============================
# SHARED BY EVERY CONTROLLER. The sim's FSC airframe runs thrust_model="momentum", so a rotor's
# thrust is QUADRATIC in throttle and falls with the airspeed along its own axis, reaching zero
# at the advance ratio J0. The legacy mapping `throttle = T / MAX_THRUST` is wrong twice over
# against that plant, and it is wrong in HOVER too: it asks 0.47 where 0.68 is needed, i.e. it
# delivers HALF the weight. That breaks the SAFE controller -- the one every failure falls back
# to -- so this lives here, in the module all of them already import, rather than in three copies.
#
# Keep in step with RotorConfig.prop_* / VTOLConfig.rotor_omega_max in vtol.py.
PROP_D = 0.3048              # m  (12 in)
PROP_J0 = 0.6                # advance ratio at zero thrust (~ pitch/diameter)
PROP_OMEGA_MAX = 749.0       # rad/s at throttle 1
PROP_INFLOW = True           # False -> legacy flat mapping


def _prop_ct0():
    n_max = PROP_OMEGA_MAX / (2.0 * math.pi)
    return MAX_THRUST / (RHO * n_max ** 2 * PROP_D ** 4)


def thrust_available(v_axial=0.0):
    """Most thrust ONE rotor can make at full throttle, at airspeed `v_axial` along its axis."""
    if not PROP_INFLOW:
        return MAX_THRUST
    x = (PROP_OMEGA_MAX / (2.0 * math.pi)) * PROP_D
    ct0 = _prop_ct0()
    return max(0.0, RHO * PROP_D ** 2 * (ct0 * x * x - (ct0 / PROP_J0) * x * float(v_axial)))


def thrust_to_throttle(T, v_axial=0.0):
    """Throttle delivering thrust `T` [N] at axial airspeed `v_axial`. Exact inverse of the sim's
    momentum model: with x = n*D, T = rho*D^2*( CT0*x^2 - (CT0/J0)*x*V ), so x is the positive
    root of a quadratic. Falls back to the flat mapping when PROP_INFLOW is off."""
    T = max(0.0, float(T))
    if not PROP_INFLOW:
        return min(1.0, max(0.0, T / MAX_THRUST))
    if T <= 0.0:
        return 0.0
    n_max = PROP_OMEGA_MAX / (2.0 * math.pi)
    k = RHO * PROP_D ** 2 * _prop_ct0()
    b = float(v_axial) / PROP_J0
    x = 0.5 * (b + math.sqrt(b * b + 4.0 * T / k))
    return min(1.0, max(0.0, x / (PROP_D * n_max)))


def build_ode():
    x = ca.SX.sym("x", NX); u = ca.SX.sym("u", NU)
    # layout: pos[0:3], R[3:12] (row-major), beta[12], vel[13:16] (inertial), omega[16:19] (body)
    R = ca.reshape(x[3:12], 3, 3).T
    beta = x[12]; vI = x[13:16]; omega = x[16:19]
    T = u[0:4]; betacmd = u[4]; de = u[5]; da = u[6]; dr = u[7]
    vb = ca.mtimes(R.T, vI)
    # --- propulsion: front pair tilt=beta, rear pair tilt=0 ---
    Fthr = ca.SX.zeros(3); Mthr = ca.SX.zeros(3); Myaw = ca.SX.zeros(3)
    for i in range(4):
        tilt = beta * TILTABLE[i]
        st, ct = ca.sin(tilt), ca.cos(tilt)
        Fi = ca.vertcat(T[i] * st, 0, T[i] * ct)   # body-frame thrust of rotor i
        pos = ca.DM(RPOS[i])
        Fthr = Fthr + Fi
        Mthr = Mthr + ca.cross(pos, Fi)
        # yaw reaction torque about the (tilted) spin axis
        k = -SPIN[i] * CQ * T[i]
        Myaw = Myaw + ca.vertcat(k * st, 0, k * ct)
    Fa, Ma = aero_body(vb, omega, de, da, dr)
    Fb = Fthr + Fa
    Mb = Mthr + Myaw + Ma
    Imat = ca.DM(np.diag([IXX, IYY, IZZ])); Iinv = ca.DM(np.diag([1/IXX, 1/IYY, 1/IZZ]))
    posdot = vI
    vdot = (1.0 / M) * ca.mtimes(R, Fb) + ca.vertcat(0, 0, -G)
    omdot = ca.mtimes(Iinv, Mb - ca.cross(omega, ca.mtimes(Imat, omega)))
    Rdot = ca.mtimes(R, skew(omega))
    Rdot_flat = ca.reshape(Rdot.T, 9, 1)
    # TILT SERVO: a RATE LIMIT, which is what the simulator implements (rotor_dynamics off ->
    # vtol.py slews the nacelle toward its target at tilt_rate_deg_s and nothing else). The
    # model used to be a first-order lag with NO rate limit, so the solver believed it could
    # swing the tilt arbitrarily fast and planned accordingly -- and the plant could not
    # follow. That is audit finding C10 ("three different tilt-servo dynamics in one loop"),
    # and it is what the HIL bags full_tr3/full_tr4 show: the applied tilt hunting +-7 deg
    # around its reference every step, dragging the pitch with it, so the vehicle limit-cycled
    # at 3-4 m/s and could never build the speed to enter the corridor.
    # tanh gives a smooth saturation: linear (slope 1/TAU_BETA) for small errors, capped at
    # TILT_RATE, and differentiable everywhere so IPOPT is happy.
    betadot = TILT_RATE * ca.tanh((betacmd - beta) / (TAU_BETA * TILT_RATE))
    xdot = ca.vertcat(posdot, Rdot_flat, betadot, vdot, omdot)
    return ca.Function("ode", [x, u], [xdot])


def rk4(F, x, u, dt, n=2):
    h = dt / n
    for _ in range(n):
        k1 = F(x, u); k2 = F(x + h / 2 * k1, u)
        k3 = F(x + h / 2 * k2, u); k4 = F(x + h * k3, u)
        x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return x


def predict_forward(F, x, ubuf, dt, sub=4):
    """Delay compensation: roll the measured state forward through the IN-FLIGHT commands (the ones
    already sent but not yet realized due to loop latency), so the NMPC solves from where the vehicle
    WILL be when its new command lands. F is the numpy-callable ODE; ubuf = list of recent controls.
    Sub-steps each interval (accurate integration of the fast rotational dynamics)."""
    x = np.asarray(x, float).copy()
    h = dt / sub
    for uu in ubuf:
        for _ in range(sub):
            x = x + h * np.array(F(x, uu)).ravel()
        Rn = x[3:12].reshape(3, 3)
        Uc, _, Vt = np.linalg.svd(Rn); x[3:12] = (Uc @ Vt).reshape(-1)   # keep R orthonormal
    return x


# ============================ transition reference ============================
class TransitionRef:
    """hover->cruise path parameterized by progress s in [0,1]. s anchors on the current tilt
    (the AVTOL path-following design). The SCHEDULES along s now come from the FSC-SCALED AVTOL
    trim generator (fsc_transition_ref / TransitionRefInterp): at s the front tilt is s*beta_cruise,
    and the trim airspeed V(s), total thrust Ttot(s), TRIM PITCH theta(s) and elevator de(s) are the
    force-balanced values at that tilt. The ATTITUDE reference carries the trim pitch -- a LEVEL
    attitude ref fights the wing trim at high tilt (at 90 deg the wing must fly at alpha ~ +7 deg).
    Falls back to the original linear schedules if the profile is unavailable."""
    def __init__(self, x0, y0, z0, yaw, Vcruise=16.0, beta_cruise_deg=70.0, tf=8.0):
        self.x0, self.y0, self.z0, self.yaw = x0, y0, z0, yaw
        self.V = Vcruise; self.bc = math.radians(beta_cruise_deg); self.tf = tf
        self.fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        self.s0 = 0.0                                # forward distance origin
        self.p0 = np.array([x0, y0, z0])
        self.Rlevel = self._R_yaw(yaw)
        self.prof = None
        try:
            from px4_vtol_direct_actuator.lib.fsc_transition_ref import FSCTransitionProfile
            self.prof = FSCTransitionProfile(V_cruise=Vcruise, tf=max(tf, 12.0),
                                             tilt_cap_deg=math.degrees(self.bc),
                                             alpha_max_deg=SO3_ALPHA_MAX)
        except Exception:
            self.prof = None

    @staticmethod
    def _R_yaw(yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    @staticmethod
    def _R_pitch(theta):
        """+theta = NOSE-UP, matching the trim schedules.

        This used to build Ry(+theta), which pitches the nose DOWN -- and the schedules emit
        theta nose-up positive, so the reference attitude was inverted. It cancelled against
        the inverted aero model and the pair looked coherent: a +8 deg cruise trim was flown
        8 deg nose-DOWN, and the old plant made lift there. Both inversions are now gone.
        """
        c, s = math.cos(theta), math.sin(theta)
        return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])

    def s_from_beta(self, beta):
        return float(np.clip(beta / max(self.bc, 1e-6), 0.0, 1.0))

    def s_from_speed(self, V):
        """Path progress implied by the ACHIEVED airspeed (via the trim tilt at that speed)."""
        if self.prof is not None:
            return float(np.clip(self.prof.by_speed(max(V, 0.0))["tilt"] / max(self.bc, 1e-6),
                                 0.0, 1.0))
        return float(np.clip(V / max(self.V, 1e-6), 0.0, 1.0))

    def get(self, s, s_dist_ref, theta_scale=1.0):
        """Return (xref(19), uref(8)) at path progress s. s_dist_ref = forward distance origin.
        theta_scale in [0,1] scales the commanded body-pitch by the ACHIEVED airspeed (pitch FF)."""
        s = float(np.clip(s, 0.0, 1.0))
        beta = s * self.bc                           # tilt schedule (the path anchor)
        if self.prof is not None:
            sch_V = self.prof.speed_at_tilt(beta)    # trim airspeed at this tilt
            sch = self.prof.by_speed(sch_V)
            V = sch_V
            # UNGATED trim pitch, faded in by PATH PROGRESS (full by s=0.15): the trim Ttot(V)
            # was solved assuming alpha=alpha_max, so a speed-gated (reduced) pitch ref is
            # INCONSISTENT with the thrust schedule mid-transition -- the vertical deficit it
            # created is where every sweep died (tilt 35-45 corridor).
            theta = sch["theta"] * min(s / max(THETA_FADE_S, 1e-3), 1.0) * theta_scale
            Ttot = sch["Ttot"]; de = sch["de"]
            R = self.Rlevel @ self._R_pitch(theta)
        else:                                        # legacy linear fallback
            V = s * self.V
            Ttot = M * G * (1.0 - 0.6 * s); de = 0.0
            R = self.Rlevel
        pos = self.p0 + self.fwd * s_dist_ref        # advance forward, hold altitude z0
        pos[2] = self.z0
        vel = self.fwd * V
        Rflat = R.reshape(-1)                         # R row-major (build_ode extracts reshape.T = R)
        xref = np.concatenate([pos, Rflat, [beta], vel, [0, 0, 0]])
        # control ref: split the trim Ttot front/rear so the thrust PITCH MOMENT is balanced.
        # REFERENCE-AUDIT FIX (2026-08-12): the old rear_frac heuristic did NOT pitch-balance the
        # FSC 4-rotor geometry -> the reference carried a huge uncompensated pitch moment
        # (omega_dot ~ -7..-8 rad/s^2 through the whole tilt corridor, NOT a trim), so the vehicle
        # pitched away and dove -- the root cause of every failed full-tilt transition. Solve for
        # the split that zeroes the pitch moment: My_thrust + My_aero = 0 with 2*(Tf+Tr) = Ttot.
        #   My_thrust = 2*Tf*(zf*sin b - xf*cos b) - 2*xr*Tr,  Tr = Ttot/2 - Tf
        #   => Tf = (xr*Ttot - My_aero_FLU) / (2*(zf*sin b - xf*cos b + xr))   [PER ROTOR]
        # The formula was RIGHT for the old (flipped) Cm and is wrong for the corrected one:
        # the standard Cm is NOSE-UP positive while the thrust moment above is written in the
        # FLU (nose-DOWN positive) convention, so My_aero has to be negated on its way in.
        # Nothing else changes -- the factor 2 belongs with the PER-ROTOR Tf below. Without
        # the negation the reference leaves a residual pitch acceleration that grows to
        # -4.9 rad/s^2 through the corridor, i.e. it is not a trim (2026-09-06 audit).
        # Verified: this drives the build_ode() pitch residual to ~0 at every s (was -7..-8).
        alpha_r = float(sch["alpha"]) if self.prof is not None else 0.0
        Tf_pair, Tr_pair = trim_split(beta, theta, V, de)
        Tf, Tr = 0.5 * Tf_pair, 0.5 * Tr_pair        # per rotor
        uref = np.array([Tf, Tf, Tr, Tr, beta, de, 0, 0])
        return xref, uref


# ============================ NMPC (SO(3) path-following) ============================
class SO3TransitionNMPC:
    # horizon + weights (structure from the AVTOL NMPC; SCALES re-tuned to FSC magnitudes)
    # N 25->12, DT 0.06->0.10, NRK4 2->1 + JIT-COMPILED IPOPT: benchmarked at mean 20 ms /
    # p95 36 ms on this machine (vs 240-940 ms before) -> the controller now runs IN-LOOP at
    # 25 Hz. The JIT build takes ~5 min once; the compiled solver is CACHED on disk.
    N = 12; DT = 0.10; NRK4 = 1
    USE_JIT = True
    MAX_ITER = 40
    #   40 with the SMOOTH stall break (60 was needed with the kinked one; 35 fails). Worst-case
    #   jitted solve ~100 ms in the corridor -> compensate latency_s ~0.15 (sv= in the log).
    w_pos = 40.0; w_vel = 60.0; w_att = 120.0; w_beta = 120.0
    #   w_att 40->120: with the SIM-MATCHED aero the attitude must STICK to the trim-pitch
    #   reference through the tilt 35-45 corridor (at 40 it lost the fight and dove).
    w_om = 3.0      # body-rate damping in the plan (roll blew up in the tilt 35-45 corridor
    #                 where front-pair differential turns into YAW authority -- rate damping
    #                 gives the planner an incentive to keep omega small through it)
    w_so3 = 1e3; w_du = 6.0; w_term = 10.0        # w_du raised for HIL: smoother controls (anti-latency-tumble)
    w_Tcmd = 1e-3; w_betacmd = 120.0; w_surf = 0.05
    pos_scale = 5.0; vel_scale = 6.0; beta_scale = 0.4
    du_T_scale = 5.0; du_beta_scale = 0.1; du_surf_scale = 0.05
    T_MIN = 0.0; T_REARMIN = 4.0; T_MAX = MAX_THRUST; BETA_MIN = 0.0; BETA_MAX = math.radians(90.0)
    #   BETA_MAX 85->90: pure-puller cruise. The rear-pair floor RELAXES with path progress in
    #   solve() (the 4 N floor is a transition aid; at s->1 the trim wants the rear unloaded).
    # ★ ALL THREE ARE 25 deg. The SIM is the authority and applies ONE limit to ALL surfaces:
    # vtol_aerodynamics scales the normalised [-1,1] command by
    # `max_surface_deflection_rad = radians(25)` for aileron, elevator AND rudder, and every
    # airframe in vtol.py sets max_deflection_deg=25 on the rudder as well.
    # DR_MAX was 30. The controllers compute `dr = M / (k * DR_MAX)`, so a limit 20% larger than
    # the plant delivers meant EVERY yaw-moment demand came out a factor 25/30 = 0.83 short.
    # Why it survived: tests/test_controller_aero_match.py feeds RADIANS directly to both models,
    # so it checks the aero FUNCTIONS agree but never exercises the [-1,1] -> radians
    # NORMALISATION, which is where the error lived. tests/test_surface_normalisation.py now does.
    # NOTE this changes nmpc_full's and nmpc6's rudder gain by 1.2x -- a CORRECTION, but they were
    # flight-validated with the wrong value, so re-check their yaw behaviour before trusting it.
    DE_MAX = math.radians(25.0); DA_MAX = math.radians(25.0); DR_MAX = math.radians(25.0)

    def __init__(self):
        self.F = build_ode()
        self._build()
        self.zsol = None; self._zsol_good = None   # warm-start + last-GOOD full solution (recovery)
        # last KNOWN-GOOD command (hover trim) — the safe fallback when a solve fails so the
        # vehicle never free-falls on a solver hiccup. front rotors carry a bit more (fwd of CG).
        Tg = M * G
        self._u_good = np.array([0.28 * Tg, 0.28 * Tg, 0.22 * Tg, 0.22 * Tg, 0.0, 0, 0, 0])
        # CRITICAL: seed uprev at the HOVER trim, not zero. The w_du control-rate penalty is
        # centred on uprev; if uprev=0 the optimizer front-loads a near-ZERO first control
        # (16 N instead of 34 N weight) -> the vehicle droops on every cold/reseeded solve.
        self._uprev = self._u_good.copy()
        self._fail = 0

    def _hover_warmstart(self, x0):
        """A clean hover warm-start (used to seed / re-seed after a failed solve)."""
        return np.concatenate([np.tile(x0, self.N + 1),
                               np.tile([M*G/4, M*G/4, M*G/4, M*G/4, x0[12], 0, 0, 0], self.N)])

    def _build(self):
        N, nx, nu = self.N, NX, NU
        X = ca.SX.sym("X", nx, N + 1); U = ca.SX.sym("U", nu, N)
        P = ca.SX.sym("P", nx + (N + 1) * nx + N * nu + nu)   # x0, xref(N+1), uref(N), uprev
        x0 = P[0:nx]
        def xref(k): return P[nx + k * nx: nx + (k + 1) * nx]
        ur0 = nx + (N + 1) * nx
        def uref(k): return P[ur0 + k * nu: ur0 + (k + 1) * nu]
        uprev = P[ur0 + N * nu: ur0 + N * nu + nu]
        g = [X[:, 0] - x0]; J = ca.SX(0)
        for k in range(N):
            xk = X[:, k]; uk = U[:, k]; xr = xref(k); urk = uref(k)
            up = uprev if k == 0 else U[:, k - 1]
            e_pos = (xk[0:3] - xr[0:3]) / self.pos_scale
            e_vel = (xk[13:16] - xr[13:16]) / self.vel_scale
            e_beta = (xk[12] - xr[12]) / self.beta_scale
            Rk = ca.reshape(xk[3:12], 3, 3).T; Rr = ca.reshape(xr[3:12], 3, 3).T
            att = 0.5 * ca.trace(ca.DM.eye(3) - ca.mtimes(Rr.T, Rk))
            so3 = ca.mtimes(Rk.T, Rk) - ca.DM.eye(3)
            e_bc = uk[4] - urk[4]
            du = uk - up
            e_du = ca.vertcat(du[0:4] / self.du_T_scale, du[4] / self.du_beta_scale, du[5:8] / self.du_surf_scale)
            J += (self.w_pos * ca.dot(e_pos, e_pos) + self.w_vel * ca.dot(e_vel, e_vel)
                  + self.w_att * att + self.w_beta * e_beta * e_beta
                  + self.w_Tcmd * ca.sumsqr(uk[0:4] - urk[0:4]) + self.w_betacmd * e_bc * e_bc
                  + self.w_surf * (uk[5] ** 2 + uk[6] ** 2 + uk[7] ** 2)
                  + self.w_du * ca.dot(e_du, e_du) + self.w_so3 * ca.sumsqr(so3)
                  + self.w_om * ca.sumsqr(xk[16:19]))
            g.append(X[:, k + 1] - rk4(self.F, xk, uk, self.DT, self.NRK4))
        xr = xref(N); xk = X[:, N]
        e_pos = (xk[0:3] - xr[0:3]) / self.pos_scale
        e_vel = (xk[13:16] - xr[13:16]) / self.vel_scale
        e_beta = (xk[12] - xr[12]) / self.beta_scale
        Rk = ca.reshape(xk[3:12], 3, 3).T; Rr = ca.reshape(xr[3:12], 3, 3).T
        att = 0.5 * ca.trace(ca.DM.eye(3) - ca.mtimes(Rr.T, Rk))
        J += self.w_term * (self.w_pos * ca.dot(e_pos, e_pos) + self.w_vel * ca.dot(e_vel, e_vel)
                            + self.w_att * att + self.w_beta * e_beta * e_beta)
        # ---- ALPHA-FLOOR as a SMOOTH SOFT PENALTY (kills the negative-alpha dive-trim basin,
        # HIL bags so3e/so3f) -- NOT a hard constraint: hard inequality rows hardened the NLP
        # past the iteration budget and broke the NOMINAL corridor (deterministic offline repro),
        # and a hard floor is infeasible from a deep-dive x0 anyway. The penalty makes the
        # alpha=-40 rotor-carried dive-trim hugely expensive while costing ~nothing on the
        # nominal (+alpha / mild-transient) trajectory. Floor -20 deg (weight 5): the 18.2
        # corridor legitimately dips to ~-15..-18; the dive basin sits at -35..-40.
        tanf = math.tan(math.radians(20.0))
        w_afloor = 25.0
        #   25 (not 5): in HIL the warm-started 40-iteration solves stayed in the dive-hold
        #   local optimum (bag so3g, ok=1 while camped at alpha -30); the penalty must make it
        #   clearly uncompetitive so warm starts migrate out.
        # ---- ALPHA-CEILING (HIL bag so3j): with only a floor, the vehicle found the OPPOSITE
        # trap -- a nose-up STALL equilibrium at AoA +45 (pitch +45, V stuck at 15). Deep-stall
        # flat-plate lift (post-stall CL ~0.72) carries the weight there, and it's self-reinforcing:
        # pitched up -> thrust points up not forward -> can't accelerate -> stays slow -> needs high
        # AoA. Penalize AoA above +ceiling so the solver drives the nose DOWN, thrust points forward,
        # it accelerates to the honest low-AoA (18 m/s) trim. Ceiling 12 deg allows the +4..6 trim.
        tanc = math.tan(math.radians(20.0))
        w_aceil = 0.0   # DISABLED: the NLP ceiling starved the solver in HIL (so3k). The
        #   high-AoA stall trap is handled reactively at the OUTPUT stage instead (elevator unstall).
        for k in range(1, N + 1):
            xk = X[:, k]
            Rk = ca.reshape(xk[3:12], 3, 3).T
            vbk = ca.mtimes(Rk.T, xk[13:16])
            u_s = ca.sqrt(vbk[0] * vbk[0] + 0.01)
            relax = 2.0 * 4.0 / (vbk[0] * vbk[0] + 4.0)             # free AoA near hover
            vsur = vbk[2] + tanf * u_s + relax                     # >0 unless AoA below -floor
            negp = 0.5 * (ca.sqrt(vsur * vsur + 0.04) - vsur)      # smooth negative part
            J += w_afloor * negp * negp
            csur = -vbk[2] + tanc * u_s + relax                    # >0 unless AoA above +ceiling
            cneg = 0.5 * (ca.sqrt(csur * csur + 0.04) - csur)
            J += w_aceil * cneg * cneg
        Z = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        nlp = {"x": Z, "f": J, "g": ca.vertcat(*g), "p": P}
        opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.max_iter": int(self.MAX_ITER),
                "ipopt.tol": 1e-3, "ipopt.acceptable_tol": 1e-2,
                "ipopt.warm_start_init_point": "yes"}
        if self.USE_JIT:
            # JIT-compile the NLP function evaluations (the 10x speedup that makes this
            # real-time). The compiled solver is serialized to a cache file so later runs
            # load in seconds instead of recompiling for ~5 minutes.
            import os as _os
            # the cache key MUST include the cost weights: they are baked into the NLP graph, and
            # a weight change with a stale cache silently loads the OLD solver (found the hard way:
            # a --watt sweep no-op'd because all runs loaded the first build).
            wkey = f"m9_i{self.MAX_ITER}_" + "_".join(f"{v:g}" for v in (self.w_pos, self.w_vel, self.w_att,
                                                       self.w_beta, self.w_so3, self.w_du,
                                                       self.w_term, self.w_Tcmd,
                                                       self.w_betacmd, self.w_surf,
                                                       self.T_REARMIN, self.w_om))
            #   "m2" = model version tag (lift-sign-fixed aero); bump when the ODE changes --
            #   the cache must never serve a solver built on a different model.
            import hashlib as _hl
            whash = _hl.md5(wkey.encode()).hexdigest()[:8]
            cache = _os.path.expanduser(
                f"~/.cache/fsc_so3_nmpc_N{self.N}_DT{int(self.DT*1000)}_RK{self.NRK4}"
                f"_B{int(math.degrees(self.BETA_MAX))}_{whash}.casadi")
            try:
                self.solver = ca.Function.load(cache)
                print(f"[so3-nmpc] loaded cached JIT solver: {cache}")
            except Exception:
                print("[so3-nmpc] JIT-compiling the solver (~5 min, once)...")
                jopts = dict(opts)
                jopts.update({"jit": True, "compiler": "shell",
                              "jit_options": {"flags": ["-O2"], "verbose": False},
                              "jit_serialize": "embed"})
                self.solver = ca.nlpsol("s", "ipopt", nlp, jopts)
                try:
                    _os.makedirs(_os.path.dirname(cache), exist_ok=True)
                    self.solver.save(cache)
                    print(f"[so3-nmpc] cached: {cache}")
                except Exception as e:
                    print(f"[so3-nmpc] cache save failed ({e}) -- continuing uncached")
        else:
            self.solver = ca.nlpsol("s", "ipopt", nlp, opts)
        self.off = nx * (N + 1)
        self.lbx = -ca.inf * ca.DM.ones(Z.shape); self.ubx = ca.inf * ca.DM.ones(Z.shape)
        for k in range(N):
            b = self.off + nu * k
            for j in range(4):
                # rear rotors (j=2,3) keep a thrust FLOOR: the NMPC else cut them to 0 to nose-down-
                # accelerate at the start, which HIL latency turned into a tumble. The floor keeps
                # vertical thrust + differential-pitch authority through the transition.
                self.lbx[b + j] = (self.T_REARMIN if j >= 2 else self.T_MIN)
                self.ubx[b + j] = self.T_MAX
            self.lbx[b + 4] = self.BETA_MIN; self.ubx[b + 4] = self.BETA_MAX
            self.lbx[b + 5] = -self.DE_MAX; self.ubx[b + 5] = self.DE_MAX
            self.lbx[b + 6] = -self.DA_MAX; self.ubx[b + 6] = self.DA_MAX
            self.lbx[b + 7] = -self.DR_MAX; self.ubx[b + 7] = self.DR_MAX
        self.lbg = ca.DM.zeros((nx * (N + 1), 1)); self.ubg = ca.DM.zeros((nx * (N + 1), 1))

    def solve(self, x0, ref, s_dist, hold=False, s_time=None, s_target=None):
        N, nx, nu = self.N, NX, NU
        s_now = 0.0 if hold else ref.s_from_beta(float(x0[12]))
        # TIME-RATCHET with a LEASH: anchored purely on achieved tilt, the corrected-aero system
        # parks in a locally-optimal rotor-borne compromise at ~30 deg (the 1.3 s horizon cannot
        # see the payoff of pushing through the corridor). The AVTOL design intends the path to be
        # traversed in tf seconds (get_state_at_s maps s->t uniformly) -- so let elapsed time
        # advance the anchor, but never more than 0.15 ahead of the tilt-derived progress (the
        # leash prevents a reference runaway if the vehicle genuinely cannot follow).
        if (not hold) and s_time is not None:
            s_now = min(max(s_now, min(s_time, 1.0)), s_now + S_LEASH)
        if not hold:
            # ACHIEVED SPEED also advances the anchor (bypasses the leash -- speed IS reality):
            # with the leash alone, a fast vehicle under a lagging tilt made excess lift ->
            # runaway climb (Vc=18.2 sweep). Tilting to the trim tilt for the CURRENT speed
            # keeps lift matched to weight.
            s_speed = ref.s_from_speed(float(np.linalg.norm(x0[13:16])))
            s_now = min(max(s_now, s_speed), s_speed + SPEED_LEASH)   # tilt tracks speed +leash
        # PITCH FEED-FORWARD: scale the reference body-pitch by the ACHIEVED airspeed so the trim
        # nose-up is only demanded once the wing/elevator can hold it (avoids the corridor nose-down).
        theta_scale = 1.0 if hold else min(float(np.linalg.norm(x0[13:16])) / V_THETA_FULL, 1.0)
        ds = 0.0 if hold else self.DT / max(ref.tf, 1e-6)   # hold=frozen hover reference (settle first)
        # RELAX the rear-pair thrust floor with path progress: 4 N/rotor through the (attitude-
        # critical) first half of the transition, fading to 0.3 N by s=1 so the 90-deg trim can
        # actually unload the rear (the static floor contradicted the pure-puller trim).
        rearmin = max(0.3, self.T_REARMIN * (1.0 - 1.4 * s_now))
        for k in range(N):
            b = self.off + nu * k
            self.lbx[b + 2] = rearmin; self.lbx[b + 3] = rearmin
        pdat = [x0]
        # forward-distance ref advances over the horizon at the scheduled speed (sk*Vcruise)
        for k in range(N + 1):
            sk = min(s_now + k * ds, 1.0)
            srefs = s_dist + sum(min(s_now + j * ds, 1.0) * ref.V * self.DT for j in range(k))
            if s_target is not None:
                srefs = min(srefs, s_target)          # SMC-fed: stop the reference AT the target

            xr, ur = ref.get(sk, srefs, theta_scale)
            pdat.append(xr)
        for k in range(N):
            sk = min(s_now + k * ds, 1.0)
            srefs = s_dist + sum(min(s_now + j * ds, 1.0) * ref.V * self.DT for j in range(k))
            if s_target is not None:
                srefs = min(srefs, s_target)          # SMC-fed: stop the reference AT the target

            _, ur = ref.get(sk, srefs, theta_scale)
            pdat.append(ur)
        pdat.append(self._uprev if hasattr(self, "_uprev") else np.zeros(nu))
        P = np.concatenate([np.asarray(a, float).ravel() for a in pdat])
        return self._run_solver(x0, P)

    def _run_solver(self, x0, P):
        """Solve with the given parameter vector P; apply the trust/fallback guards. Shared by the
        s-anchored solve() and the time-anchored solve_mission()."""
        nu = NU
        if self.zsol is None:
            self.zsol = self._hover_warmstart(x0)
        try:
            sol = self.solver(x0=self.zsol, lbx=self.lbx, ubx=self.ubx, lbg=self.lbg, ubg=self.ubg, p=P)
            st = self.solver.stats()["return_status"]
            ok = ("Solve_Succeeded" in st) or ("Acceptable" in st)
            z = np.array(sol["x"]).ravel()
            u0 = z[self.off:self.off + nu]
            # PHYSICAL SANITY GUARD: below stall speed the wing CANNOT carry the weight, so an
            # "optimal" solution with near-zero total rotor thrust is garbage.
            Vnow = float(np.linalg.norm(np.asarray(x0[13:16], float)))
            if ok and Vnow < 11.0 and float(np.sum(u0[0:4])) < 0.5 * M * G:
                ok = False
            if ok:
                self.zsol = z; self._zsol_good = z.copy(); self._u_good = u0.copy(); self._fail = 0
            else:
                # FAILED solve: NEVER apply the garbage iterate. RE-SEED from the LAST GOOD solution.
                self._fail += 1
                self.zsol = self._zsol_good if self._zsol_good is not None else None
                u0 = self._u_good.copy()
        except Exception:
            self._fail += 1
            self.zsol = self._zsol_good if self._zsol_good is not None else None
            u0 = self._u_good.copy(); ok = False
        self._uprev = u0.copy()
        return u0, ok

    def solve_mission(self, x0, mission, t_now, hold=False, anchor_xy=False):
        """Sample the full-mission reference by TIME along the horizon (AVTOL-style:
        ref.get(t_now + k*DT)) and solve. This is the path for FSCMissionProfile, which is a
        time-parameterised trajectory (climb->transition->cruise->back->hover->land).

        anchor_xy: shift the horizontal reference position so xref(t_now)=vehicle (used through the
        transition). The vehicle then tracks VELOCITY+TILT+ATTITUDE with no phantom position error
        (the schedule's absolute position runs ahead of a vehicle that accelerates slower). Off in
        hover/land so the landing is at the ABSOLUTE target."""
        N, nu = self.N, NU
        tilt_ref = float(mission._iv(t_now, mission.tilt_isaac))
        rearmin = max(0.3, self.T_REARMIN * (1.0 - tilt_ref / (0.5 * math.pi)))
        for k in range(N):
            b = self.off + nu * k
            self.lbx[b + 2] = rearmin; self.lbx[b + 3] = rearmin
        off_xy = np.zeros(2)
        if anchor_xy:
            xr0 = mission.get(t_now)[0]
            off_xy = np.asarray(x0[0:2], float) - xr0[0:2]
        pdat = [x0]
        urefs = []
        for k in range(N + 1):
            xr, ur = mission.get(t_now + k * self.DT)
            xr = np.asarray(xr, float).copy(); xr[0:2] = xr[0:2] + off_xy
            pdat.append(xr)
            if k < N:
                urefs.append(ur)
        pdat.extend(urefs)
        pdat.append(self._uprev if hasattr(self, "_uprev") else np.zeros(nu))
        P = np.concatenate([np.asarray(a, float).ravel() for a in pdat])
        return self._run_solver(x0, P)


# ============================ ROS2 node ============================
def _q2R(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w) + 1e-12
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def run_ros():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                           QoSHistoryPolicy, qos_profile_sensor_data)
    from px4_msgs.msg import (OffboardControlMode, ActuatorMotors, ActuatorServos,
                              VehicleCommand, VehicleStatus)
    from std_msgs.msg import String
    from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference

    def clip(v, lo, hi):
        return lo if v < lo else hi if v > hi else v

    class SO3TransitionNode(Node):
        def __init__(self):
            super().__init__("nmpc_so3_transition_control")
            P = self.declare_parameter
            P("px4_ns", "uav_0"); P("controller_id", "so3_transition"); P("active_at_start", False)
            P("auto_arm", True); P("arm_after", 150); P("rate_hz", 25.0)
            P("Vcruise", 15.0); P("beta_cruise_deg", 90.0); P("transition_tf", 20.0)
            #   tf=20 (path pace) + the time-ratchet anchor: offline (SIM-MATCHED aero) this
            #   completes the FULL 90-deg pure-puller transition and settles a steady cruise
            #   (beta=90.0, V=13.8, z dead-flat, AoA +7.2 = the true trim, rotors ~6 N total).
            #   tf=14 was too aggressive for the plant (corridor crash).
            P("settle_s", 2.0)               # hold HOVER this long after engaging, then start the transition
            P("alpha_max_deg", 6.0)          # profile trim alpha_max (6 = honest-physics 18.2 m/s cruise)
            P("s_leash", 0.08)               # time-ratchet leash (see S_LEASH)
            P("tilt_margin_deg", 12.0)       # applied tilt <= trim_tilt(V) + this (anti-saturation, bag so3i)
            P("rear_floor_cmd", 0.10)        # rear motor floor [frac] for roll/yaw authority in cruise
            P("aoa_unstall_deg", 22.0)       # reactive elevator nose-down above this AoA (unstall, bag so3j)
            P("k_unstall", 0.0)              # elevator gain per (AoA-thresh)/20deg
            P("unstall_z_min", 5.0)          # only unstall above this altitude [m]
            P("k_sas_pitch", 0.30); P("k_sas_roll", 0.20); P("k_sas_yaw", 0.15)   # rate-SAS on surfaces
            P("front_motor_cap", 0.90)       # cap front motors below saturation (attitude headroom, bag so3q)
            P("latency_s", 0.10)             # loop latency to DELAY-COMPENSATE (predict state fwd by this;
            #                                  the key HIL fix -- solve from where the vehicle WILL be)
            P("de_gain", 1.0); P("da_gain", 0.6); P("dr_gain", 0.6)
            # ---- FULL-MISSION mode (FSCMissionProfile: climb->transition->cruise->back->land at a
            # target). Offline-validated to vc~14 (tilt-assisted cruise, precise landing). Enable
            # with mission_mode=1. Recipe: pitch-balanced ref + wing-load gate + position ANCHORING
            # + strong attitude/tilt weights + gentle back-transition + elevator pitch-hold. ----
            P("mission_mode", 0.0)
            P("mission_target_x", 200.0); P("mission_target_y", 0.0)
            P("mission_zcruise", 15.0); P("mission_vcruise", 13.0); P("mission_cap_deg", 55.0)
            P("mission_amax_deg", 6.0); P("mission_tout", 16.0); P("mission_tclimb", 8.0)
            P("mission_tback", 12.0); P("mission_speed_lead", 0.4); P("mission_errhold", 3.0)
            P("mission_winglo", 9.5); P("mission_winghi", 14.5)
            P("kpitch", 3.0)                 # elevator pitch-ATTITUDE hold (mission mode; sole pitch actuator ~90)
            # mission NMPC weights (offline-validated): strong attitude/tilt so the NMPC uses TILT not
            # body-lean for forward motion. Applied ONLY in mission_mode (legacy path keeps its tune).
            P("mission_w_att", 220.0); P("mission_w_beta", 200.0); P("mission_w_om", 18.0)
            ns = self.get_parameter("px4_ns").value
            pre = f"/{ns}" if ns else ""
            self.cid = self.get_parameter("controller_id").value
            self._active = bool(self.get_parameter("active_at_start").value)
            self.auto_arm = bool(self.get_parameter("auto_arm").value)
            self.arm_after = int(self.get_parameter("arm_after").value)
            self.rate = float(self.gp("rate_hz"))
            from px4_vtol_direct_actuator import nmpc_so3_transition_node as _m
            _m.SO3_ALPHA_MAX = float(self.gp("alpha_max_deg"))
            _m.S_LEASH = float(self.gp("s_leash"))
            if self.gp("mission_mode") > 0.5:        # build the NMPC with the mission-validated weights
                SO3TransitionNMPC.w_att = float(self.gp("mission_w_att"))
                SO3TransitionNMPC.w_beta = float(self.gp("mission_w_beta"))
                SO3TransitionNMPC.w_om = float(self.gp("mission_w_om"))
                self.get_logger().info("MISSION mode: w_att=%.0f w_beta=%.0f w_om=%.0f"
                                       % (SO3TransitionNMPC.w_att, SO3TransitionNMPC.w_beta, SO3TransitionNMPC.w_om))
            self.get_logger().info("building SO(3) transition NMPC (this takes a few s)...")
            self.nmpc = SO3TransitionNMPC()
            self.get_logger().info("NMPC built.")
            self.beta = 0.0; self.state = None; self.start = None; self.ref = None
            self._aoa_f = None; self._de_un = 0.0    # unstall: filtered AoA + rate-limited elevator
            self.armed = self.offb = False; self.counter = self.last = 0
            self.ubuf = []                   # recent commands in flight (for delay compensation)
            self.smc_sp = None               # SMC-fed position set-point (outer loop -> WHERE to go)
            self.mission = None; self.t_mission = 0.0    # full-mission profile + mission clock
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            self.ocm = self.create_publisher(OffboardControlMode, f"{pre}/fmu/in/offboard_control_mode", qos)
            self.motp = self.create_publisher(ActuatorMotors, f"{pre}/fmu/in/actuator_motors", qos)
            self.srvp = self.create_publisher(ActuatorServos, f"{pre}/fmu/in/actuator_servos", qos)
            self.cmdp = self.create_publisher(VehicleCommand, f"{pre}/fmu/in/vehicle_command", qos)
            self.create_subscription(VehicleStatus, f"{pre}/fmu/out/vehicle_status_v1", self.on_status, qos)
            self.create_subscription(Mocap, f"{pre}/mocap", self.on_mocap, qos_profile_sensor_data)
            self.create_subscription(String, f"{pre}/ground_station/active_controller", self.on_active, 10)
            # SMC-FED POSITION: the launcher remaps 'reference' -> 'reference_smc', so this reads the
            # SMC outer loop's converging position set-point. It gives the along-path DESTINATION; the
            # SO(3) NMPC still owns the tilt/attitude/velocity transition schedule to get there.
            self.create_subscription(PositionControllerReference,
                                     f"{pre}/fsc_autopilot_ros2/position_controller/reference",
                                     self.on_ref, 10)
            self.create_timer(1.0 / self.rate, self.loop)
            self.get_logger().info(f"so3_transition up: id={self.cid} rate={self.rate:.0f}Hz N={self.nmpc.N}")

        def gp(self, n):
            return float(self.get_parameter(n).value)

        def now_us(self):
            return int(self.get_clock().now().nanoseconds / 1000)

        def on_status(self, m):
            self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            self.offb = (m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        def on_active(self, m):
            self._active = (m.data == self.cid)

        def on_ref(self, m):
            self.smc_sp = np.array([m.position.x, m.position.y, m.position.z])

        def on_mocap(self, m):
            q = m.pose.orientation
            R = _q2R(q.x, q.y, q.z, q.w)
            p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            v = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])
            w = np.array([m.twist.angular.x, m.twist.angular.y, m.twist.angular.z])
            self.state = (p, R, v, w)

        def loop(self):
            self.counter += 1
            o = OffboardControlMode(); o.direct_actuator = True; o.timestamp = self.now_us()
            if self._active:
                self.ocm.publish(o)
            if self.state is None:
                if self._active:
                    self.pub([math.nan]*4, [math.nan]*5)
                return
            p, R, v, w = self.state
            if not self._active:
                self.start = None; self.ref = None; self.mission = None; return
            if self.gp("mission_mode") > 0.5:        # FULL-MISSION path (self-contained; legacy untouched)
                self._mission_step(p, R, v, w); return
            if self.ref is None:                     # snapshot the transition start on activation
                yaw = math.atan2(R[1, 0], R[0, 0])
                self.start = p.copy()
                self.ref = TransitionRef(p[0], p[1], p[2], yaw,
                                         Vcruise=self.gp("Vcruise"),
                                         beta_cruise_deg=self.gp("beta_cruise_deg"),
                                         tf=self.gp("transition_tf"))
                self.beta = 0.0; self._eng = 0
                self.get_logger().info("SO(3) transition ENGAGED at (%.1f,%.1f,%.1f) yaw=%.0f" %
                                       (p[0], p[1], p[2], math.degrees(yaw)))
            self._eng += 1
            hold = self._eng < int(self.gp("settle_s") * self.rate)   # settle in HOVER first, then transition
            x = np.concatenate([p, R.reshape(-1), [self.beta], v, w])   # R row-major (build_ode: reshape.T=R)
            s_dist = float(np.dot(p - self.ref.p0, self.ref.fwd))
            # SMC-FED DESTINATION: project the SMC set-point onto the transition heading -> the
            # along-path distance the reference is allowed to advance to (capped in solve()), so the
            # NMPC flies the schedule TOWARD the SMC target instead of forward forever. None -> the
            # legacy self-generated (open-loop) path (unchanged when no SMC reference is present).
            s_target = (float(np.dot(self.smc_sp - self.ref.p0, self.ref.fwd))
                        if self.smc_sp is not None else None)
            # DELAY COMPENSATION: predict the state forward by the loop latency using the in-flight
            # commands, so the NMPC solves from where the vehicle WILL be. (Predicting the ATTITUDE
            # forward matters: keeping the stale measured attitude added phase lag and made the onset
            # oscillate worse -- the latency compensation on attitude is what stabilises the fast loop.)
            lat_n = max(1, int(round(self.gp("latency_s") * self.rate)))
            if self.ubuf and self.gp("latency_s") > 1e-3:
                x = predict_forward(self.nmpc.F, x, self.ubuf[-lat_n:], 1.0 / self.rate)
            s_time = max(0.0, (self._eng / self.rate) - self.gp("settle_s")) / max(self.gp("transition_tf"), 1e-6)
            import time as _t
            _t0 = _t.perf_counter()
            u, ok = self.nmpc.solve(x, self.ref, s_dist, hold=hold, s_time=s_time, s_target=s_target)
            self._solve_ms = (_t.perf_counter() - _t0) * 1e3
            self.ubuf.append(u.copy()); self.ubuf = self.ubuf[-lat_n:]
            betacmd = float(clip(u[4], SO3TransitionNMPC.BETA_MIN, SO3TransitionNMPC.BETA_MAX))
            # ---- AIRSPEED TILT CLAMP (HIL bag so3i): the solver over-tilted to 70 deg at only
            # 8 m/s to chase the aggressive speed ramp; the wing can't carry there, so the front
            # pair had to do lift+forward at once -> SATURATED (mFL/mFR=1.0) -> lost pitch
            # authority -> nose-down dive (AoA -35) -> near-ground sag. Clamp the applied tilt to
            # the TRIM tilt for the CURRENT airspeed (+margin), so the front pair never needs to
            # both lift and drive forward -> no saturation. Self-relaxes to ~90 as speed builds.
            V_now = float(np.linalg.norm(v))
            if self.ref is not None and self.ref.prof is not None and not hold:
                tilt_trim = self.ref.prof.by_speed(max(V_now, 0.0))["tilt"]
                betacmd = min(betacmd, tilt_trim + math.radians(self.gp("tilt_margin_deg")))
            # FREEZE the tilt integrator on failed solves: replaying a stale betacmd kept walking
            # the tilt forward through a failure cascade (bag so3a) -- hold the path where it is
            # until the solver recovers (s is anchored on beta, so the reference holds too).
            if ok:
                _dt = 1.0 / self.rate
                self.beta += clip((betacmd - self.beta) / TAU_BETA,
                                  -TILT_RATE, TILT_RATE) * _dt
            # REAR-ROTOR FLOOR on the applied command (HIL bag so3i): the plan let the rear pair
            # unload to 0.02 in cruise -> no differential roll/yaw authority -> lateral departure.
            # Keep a small floor so the rear retains control authority (front+wing carry the load).
            rmin_cmd = self.gp("rear_floor_cmd")
            fcap = self.gp("front_motor_cap")
            mot = [clip(u[i] / MAX_THRUST, 0.0, 1.0) for i in range(4)]
            mot[2] = max(mot[2], rmin_cmd); mot[3] = max(mot[3], rmin_cmd)
            # FRONT-MOTOR HEADROOM CAP (HIL bag so3q): during the sink->zoom porpoise the front
            # pair SATURATED (1.00) -> lost differential-pitch authority -> the porpoise diverged to
            # a stall (AoA 89). Capping below saturation always leaves attitude authority.
            mot[0] = min(mot[0], fcap); mot[1] = min(mot[1], fcap)
            tservo = clip(betacmd / (math.pi / 2), 0.0, 1.0)
            elevator = clip(self.gp("de_gain") * u[5] / SO3TransitionNMPC.DE_MAX, -1.0, 1.0)
            # ---- REACTIVE ELEVATOR UNSTALL (HIL bag so3j): the vehicle settled a STABLE but
            # STALLED cruise -- pitch/AoA +45, stuck at 15 m/s -- because pitched up so far the
            # thrust points up (can't accelerate to the honest 18 m/s / AoA 4 trim). The solver's
            # own nose-down effort is too weak at high tilt (front rotors point fwd -> little pitch
            # authority). AUGMENT it with the elevator, which has strong authority at 15 m/s
            # (~18 rad/s^2). de>0 -> Cm_de<0 -> nose-DOWN. Gated on AoA>threshold + safe altitude, so
            # it fires only when stalled and can't dive it into the ground.
            # TIGHTLY GATED to the SETTLED stall cruise only (bag so3l tumbled when it fired on a
            # transition TRANSIENT): filtered AoA, past the transition (tilt>70), not climbing/diving
            # (|vz| small), safe altitude. Gentle + rate-limited so it eases the nose down, never slams.
            vb_m = R.T @ v
            aoa_m = math.degrees(math.atan2(-vb_m[2], abs(vb_m[0]) + 1e-6)) if V_now > 2.0 else 0.0
            self._aoa_f = aoa_m if self._aoa_f is None else 0.9 * self._aoa_f + 0.1 * aoa_m
            settled = (math.degrees(self.beta) > 70.0 and abs(float(v[2])) < 2.0
                       and p[2] > self.gp("unstall_z_min") and V_now > 10.0)
            de_un = 0.0
            if settled and self._aoa_f > self.gp("aoa_unstall_deg"):
                de_un = clip(self.gp("k_unstall") * (self._aoa_f - self.gp("aoa_unstall_deg")) / 20.0,
                             0.0, 0.5)
            self._de_un = clip(de_un, self._de_un - 0.03, self._de_un + 0.03)   # rate-limit
            elevator = clip(elevator + self._de_un, -1.0, 1.0)
            aileron = clip(self.gp("da_gain") * u[6] / SO3TransitionNMPC.DA_MAX, -1.0, 1.0)
            rudder = clip(self.gp("dr_gain") * u[7] / SO3TransitionNMPC.DR_MAX, -1.0, 1.0)
            # ---- RATE-SAS (HIL bags so3k-o): the corridor entry PORPOISES (pitch AoA swings
            # -21 -> +35 in 3 s) faster than the 25 Hz NMPC + HIL latency can stabilise -> the
            # solve fails and it gets stuck. A reactive body-rate damper on the SURFACES runs every
            # mocap update (fast) and damps the oscillation directly, augmenting the NMPC. Gated on
            # airspeed (surfaces need flow). Signs: +elevator -> nose-DOWN (Cm_de<0) opposes +q;
            # aileron opposes roll rate p; rudder opposes yaw rate r.
            sas = clip((V_now - 3.0) / 4.0, 0.0, 1.0)              # fade in with airspeed
            wb = w   # measured body rates (mocap)
            elevator = clip(elevator + sas * self.gp("k_sas_pitch") * float(wb[1]), -1.0, 1.0)
            aileron = clip(aileron - sas * self.gp("k_sas_roll") * float(wb[0]), -1.0, 1.0)
            rudder = clip(rudder - sas * self.gp("k_sas_yaw") * float(wb[2]), -1.0, 1.0)
            self.pub(mot, [tservo, tservo, aileron, elevator, rudder])
            if self.counter % max(1, int(self.rate / 2)) == 0:
                vb = R.T @ v; V = float(np.linalg.norm(v))
                aoa = math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-6)) if V > 0.3 else 0.0
                self.get_logger().info(
                    "p=(%.1f,%.1f,%.1f) V=%.1f tilt=%.0f aoa=%.0f ok=%d sv=%.0fms | "
                    "T=[%.1f,%.1f,%.1f,%.1f] elev=%.2f mot=[%.2f,%.2f,%.2f,%.2f] arm=%s"
                    % (p[0], p[1], p[2], V, math.degrees(self.beta), aoa, int(ok),
                       getattr(self, "_solve_ms", 0.0),
                       u[0], u[1], u[2], u[3], elevator, mot[0], mot[1], mot[2], mot[3], self.armed))
            if self._active and self.auto_arm and self.counter >= self.arm_after and not (self.armed and self.offb):
                if self.counter - self.last >= 50:
                    self.last = self.counter
                    if not self.offb:
                        self.vcmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    elif not self.armed:
                        self.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        def _mission_step(self, p, R, v, w):
            """FULL-MISSION control step: FSCMissionProfile sampled by a state-gated clock + the SO(3)
            solve_mission + the shared output stage. Self-contained so the HIL-validated legacy
            transition path is untouched. Recipe (offline-validated to vc~14): pitch-balanced ref +
            wing-load gate + position ANCHORING + strong attitude/tilt weights + gentle back-
            transition + elevator pitch-hold."""
            dt = 1.0 / self.rate
            if self.mission is None:                 # build the mission at the engage pose (one-time)
                from px4_vtol_direct_actuator.lib.fsc_mission_ref import FSCMissionProfile
                self.mission = FSCMissionProfile(
                    target=(self.gp("mission_target_x"), self.gp("mission_target_y")),
                    z_ground=max(float(p[2]), 0.5), z_cruise=self.gp("mission_zcruise"),
                    V_cruise=self.gp("mission_vcruise"), tilt_cap_deg=self.gp("mission_cap_deg"),
                    alpha_max_deg=self.gp("mission_amax_deg"), T_out=self.gp("mission_tout"),
                    T_climb=self.gp("mission_tclimb"), T_back=self.gp("mission_tback"),
                    wing_load_lo=self.gp("mission_winglo"), wing_load_hi=self.gp("mission_winghi"),
                    x0=float(p[0]), y0=float(p[1]))
                self.t_mission = 0.0; self.beta = 0.0
                self.get_logger().info(
                    "MISSION engaged (%.1f,%.1f,%.1f) -> target (%.1f,%.1f) tf=%.1fs cruise=%.1fs fwd=%.0fm"
                    % (p[0], p[1], p[2], self.gp("mission_target_x"), self.gp("mission_target_y"),
                       self.mission.tf, self.mission.T_cruise, self.mission.D_total))
            m = self.mission
            V_now = float(np.linalg.norm(v))
            x = np.concatenate([p, R.reshape(-1), [self.beta], v, w])   # R row-major (build_ode: reshape.T=R)
            lat_n = max(1, int(round(self.gp("latency_s") * self.rate)))
            if self.ubuf and self.gp("latency_s") > 1e-3:               # delay-compensate the state
                x = predict_forward(self.nmpc.F, x, self.ubuf[-lat_n:], dt)
            # state-gated mission clock: speed-gated in trans_out (ref-tilt tracks achieved speed ->
            # no tilt-leads-speed divergence); time-advance-with-position-freeze elsewhere.
            ph = m.phase_at_time(self.t_mission)
            if ph == "trans_out":
                t_spd = m.time_from_speed(V_now)
                self.t_mission = min(self.t_mission + 2.0 * dt, t_spd + self.gp("mission_speed_lead"))
                self.t_mission = max(self.t_mission, t_spd - 0.2)
            else:
                xr0 = m.get(self.t_mission)[0]
                lag = float(np.linalg.norm(p - xr0[0:3]))
                self.t_mission += dt if lag < self.gp("mission_errhold") else 0.0
            self.t_mission = float(min(self.t_mission, m.tf))
            anchor = ph in ("trans_out", "cruise", "trans_back")       # track velocity, no phantom pos error
            import time as _t
            _t0 = _t.perf_counter()
            u, ok = self.nmpc.solve_mission(x, m, self.t_mission, anchor_xy=anchor)
            self._solve_ms = (_t.perf_counter() - _t0) * 1e3
            self.ubuf.append(u.copy()); self.ubuf = self.ubuf[-lat_n:]
            betacmd = float(clip(u[4], SO3TransitionNMPC.BETA_MIN, SO3TransitionNMPC.BETA_MAX))
            if ok:
                self.beta += clip((betacmd - self.beta) / TAU_BETA,
                                  -TILT_RATE, TILT_RATE) * dt
            # shared output stage: motor caps/floors + elevator (NMPC de + rate-SAS + pitch-ATTITUDE hold)
            rmin_cmd = self.gp("rear_floor_cmd"); fcap = self.gp("front_motor_cap")
            mot = [clip(u[i] / MAX_THRUST, 0.0, 1.0) for i in range(4)]
            mot[2] = max(mot[2], rmin_cmd); mot[3] = max(mot[3], rmin_cmd)
            mot[0] = min(mot[0], fcap); mot[1] = min(mot[1], fcap)
            tservo = clip(betacmd / (math.pi / 2), 0.0, 1.0)
            sas = clip((V_now - 3.0) / 4.0, 0.0, 1.0)
            theta_act = math.asin(clip(R[2, 0], -1.0, 1.0))            # body pitch, nose-up +
            e_pitch = theta_act - m._iv(self.t_mission, m.theta)       # deviation from the trim pitch ref
            elevator = clip(self.gp("de_gain") * u[5] / SO3TransitionNMPC.DE_MAX
                            + sas * (self.gp("k_sas_pitch") * float(w[1]) + self.gp("kpitch") * e_pitch),
                            -1.0, 1.0)
            aileron = clip(self.gp("da_gain") * u[6] / SO3TransitionNMPC.DA_MAX
                           - sas * self.gp("k_sas_roll") * float(w[0]), -1.0, 1.0)
            rudder = clip(self.gp("dr_gain") * u[7] / SO3TransitionNMPC.DR_MAX
                          - sas * self.gp("k_sas_yaw") * float(w[2]), -1.0, 1.0)
            self.pub(mot, [tservo, tservo, aileron, elevator, rudder])
            if self.counter % max(1, int(self.rate / 2)) == 0:
                vb = R.T @ v; aoa = math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-6)) if V_now > 0.3 else 0.0
                self.get_logger().info(
                    "MISSION t=%.1f/%.0f %-9s p=(%.1f,%.1f,%.1f) V=%.1f tilt=%.0f aoa=%.0f ok=%d sv=%.0fms | mot=[%.2f,%.2f,%.2f,%.2f] elev=%.2f"
                    % (self.t_mission, m.tf, ph, p[0], p[1], p[2], V_now, math.degrees(self.beta),
                       aoa, int(ok), getattr(self, "_solve_ms", 0.0),
                       mot[0], mot[1], mot[2], mot[3], elevator))
            if self._active and self.auto_arm and self.counter >= self.arm_after and not (self.armed and self.offb):
                if self.counter - self.last >= 50:
                    self.last = self.counter
                    if not self.offb:
                        self.vcmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    elif not self.armed:
                        self.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        def pub(self, mot, srv):
            if not self._active:
                return
            mm = ActuatorMotors(); c = [math.nan] * 12
            for i in range(4):
                c[i] = mot[i]
            mm.control = c; mm.timestamp = mm.timestamp_sample = self.now_us(); self.motp.publish(mm)
            ss = ActuatorServos(); sc = [math.nan] * 8
            for i in range(5):
                sc[i] = srv[i]
            ss.control = sc; ss.timestamp = ss.timestamp_sample = self.now_us(); self.srvp.publish(ss)

        def vcmd(self, cmd, p1=0.0, p2=0.0):
            m = VehicleCommand(); m.command = cmd; m.param1 = p1; m.param2 = p2
            m.target_system = m.target_component = m.source_system = m.source_component = 1
            m.from_external = True; m.timestamp = self.now_us(); self.cmdp.publish(m)

    rclpy.init()
    try:
        rclpy.spin(SO3TransitionNode())
    except KeyboardInterrupt:
        pass


# ============================ offline self-test ============================
def selftest():
    print("building NMPC...")
    import sys
    def argf(name, d):
        return float(sys.argv[sys.argv.index(name) + 1]) if name in sys.argv else d
    # sweepable via argv: --lat 0.10 --comp 1 --tf 8 --bc 70 --Vc 16 --settle 1.5 --v0 1.5
    #                     --watt --wvel --wpos --wbeta --wdu --wso3   (gains; rebuild the NMPC)
    LAT = argf("--lat", 0.10); COMP = argf("--comp", 1.0)          # 100 ms loop latency; delay-comp on
    tf = argf("--tf", 8.0); bc = argf("--bc", 70.0); Vc = argf("--Vc", 16.0)
    DUR = argf("--dur", 14.0)
    settle = argf("--settle", 1.5); v0 = argf("--v0", 1.5)          # geometric-handoff forward speed
    for nm in ("w_att", "w_vel", "w_pos", "w_beta", "w_du", "w_so3", "w_om"):
        setattr(SO3TransitionNMPC, nm, argf("--w" + nm[2:], getattr(SO3TransitionNMPC, nm)))
    from px4_vtol_direct_actuator import nmpc_so3_transition_node as _self_mod  # allow --thfade to reach TransitionRef
    globals()["THETA_FADE_S"] = argf("--thfade", THETA_FADE_S)
    globals()["SO3_ALPHA_MAX"] = argf("--amax", SO3_ALPHA_MAX)
    globals()["S_LEASH"] = argf("--leash", S_LEASH)
    if "--nojit" in sys.argv:
        SO3TransitionNMPC.USE_JIT = False       # offline tuning: skip the 5-min compile per config
    SO3TransitionNMPC.MAX_ITER = int(argf("--maxiter", SO3TransitionNMPC.MAX_ITER))
    print("LAT=%.0fms comp=%d tf=%.0f bc=%.0f Vc=%.0f settle=%.1f | w_att=%.0f w_vel=%.0f w_du=%.1f"
          % (LAT*1e3, COMP, tf, bc, Vc, settle, SO3TransitionNMPC.w_att, SO3TransitionNMPC.w_vel, SO3TransitionNMPC.w_du))
    roll0 = math.radians(argf("--roll0", 6.0)); vy0 = argf("--vy0", 0.6)   # lateral/roll disturbance (handoff)
    TILT_MARGIN = math.radians(argf("--tiltmargin", 12.0))   # mirror the node's output clamps
    REAR_FLOOR = argf("--rearfloor", 0.10) * MAX_THRUST
    nmpc = SO3TransitionNMPC()
    x = np.zeros(NX); x[3:12] = np.eye(3).reshape(-1); x[12] = 0.0
    cr, sr = math.cos(roll0), math.sin(roll0)
    x[3:12] = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]).reshape(-1)  # initial ROLL
    z0 = argf('--z0', 8.0)
    x[0:3] = [0, 0, z0]; x[13] = v0; x[14] = vy0     # hover at 8 m + forward + lateral drift (handoff)
    ref = TransitionRef(0, 0, z0, 0.0, Vcruise=Vc, beta_cruise_deg=bc, tf=tf)
    Ffull = build_ode()
    DT = 0.02; s_dist = 0.0; nsolve = int(nmpc.DT / DT)
    PLATLAT = argf("--platlat", LAT)                   # ACTUAL plant latency (may differ from COMP est)
    lat_steps = max(1, int(round(LAT / DT)))           # compensation depth
    plat_steps = max(1, int(round(PLATLAT / DT)))      # true plant delay
    u = np.array([M*G/4]*4 + [0.0, 0, 0, 0])
    hist = [u.copy() for _ in range(max(lat_steps, plat_steps) + 1)]
    print(" t    x     z    vz   V   beta AoA roll | T[FL FR RL RR] ok")
    crashed = False
    for it in range(int(DUR / DT)):
        if it % nsolve == 0:
            xp = predict_forward(Ffull, x, hist[-lat_steps:], DT) if COMP > 0.5 else x   # delay compensation
            st = max(0.0, it * DT - settle) / max(tf, 1e-6)
            u, ok = nmpc.solve(xp, ref, s_dist, hold=(it * DT < settle), s_time=st)
            # mirror the node's OUTPUT clamps (airspeed tilt clamp + rear-rotor floor)
            if ref.prof is not None and it * DT >= settle:
                Vn = float(np.linalg.norm(x[13:16]))
                u[4] = min(u[4], ref.prof.by_speed(max(Vn, 0.0))["tilt"] + TILT_MARGIN)
            u[2] = max(u[2], REAR_FLOOR); u[3] = max(u[3], REAR_FLOOR)
        hist.append(u.copy()); hist = hist[-200:]
        u_app = hist[-1 - plat_steps]                  # plant applies the command from PLATLAT ago
        x = x + DT * np.array(Ffull(x, u_app)).ravel()
        R = x[3:12].reshape(3, 3)
        Uc, _, Vt = np.linalg.svd(R); R = Uc @ Vt; x[3:12] = R.reshape(-1)
        s_dist = float(np.dot(x[0:3] - ref.p0, ref.fwd))
        if x[2] < 0.5 or x[2] > 40:
            print("  !! CRASH/RUNAWAY at t=%.1f z=%.1f" % (it*DT, x[2])); crashed = True; break
        if it % 20 == 0:
            vI = x[13:16]; vb = R.T @ vI; V = np.linalg.norm(vI)
            aoa = math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-6)) if V > 0.5 else 0
            roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
            print("%4.1f %5.1f %5.2f %+5.1f %4.1f %5.1f %5.1f %5.0f | %4.1f %4.1f %4.1f %4.1f %d"
                  % (it*DT, x[0], x[2], x[15], V, math.degrees(x[12]), aoa, roll,
                     u[0], u[1], u[2], u[3], int(ok)))
    if not crashed:
        print("OK  final: x=%.1f z=%.2f V=%.1f beta=%.0fdeg" %
              (x[0], x[2], np.linalg.norm(x[13:16]), math.degrees(x[12])))


def mission_selftest():
    """Offline forward-sim of the NMPC flying the FULL MISSION (FSCMissionProfile), sampled by TIME
    along the horizon. Proves end-to-end tracking (climb->transition->cruise->back->hover->land)
    before HIL."""
    print("building mission NMPC...")
    import sys
    def argf(name, d):
        return float(sys.argv[sys.argv.index(name) + 1]) if name in sys.argv else d
    from px4_vtol_direct_actuator.lib.fsc_mission_ref import FSCMissionProfile
    LAT = argf("--lat", 0.10); COMP = argf("--comp", 1.0)
    tx = argf("--tx", 120.0); ty = argf("--ty", 0.0)
    zc = argf("--zc", 15.0); vc = argf("--vc", 15.0); cap = argf("--cap", 90.0)
    amax = argf("--amax", 8.0); tout = argf("--tout", 10.0)
    tclimb = argf("--tclimb", 6.0); tback = argf("--tback", 5.0)
    winglo = argf("--winglo", 9.5); winghi = argf("--winghi", 14.5)
    for nm in ("w_att", "w_vel", "w_pos", "w_beta", "w_du", "w_so3", "w_om"):
        setattr(SO3TransitionNMPC, nm, argf("--w" + nm[2:], getattr(SO3TransitionNMPC, nm)))
    SO3TransitionNMPC.MAX_ITER = int(argf("--maxiter", SO3TransitionNMPC.MAX_ITER))
    if "--nojit" in sys.argv:
        SO3TransitionNMPC.USE_JIT = False
    mission = FSCMissionProfile(target=(tx, ty), z_cruise=zc, V_cruise=vc, tilt_cap_deg=cap,
                               alpha_max_deg=amax, T_out=tout, T_climb=tclimb, T_back=tback,
                               wing_load_lo=winglo, wing_load_hi=winghi)
    print("mission tf=%.1fs cruise_hold=%.1fs fwd=%.0fm" % (mission.tf, mission.T_cruise, mission.D_total))
    nmpc = SO3TransitionNMPC()
    Ffull = build_ode()
    x = np.zeros(NX); x[3:12] = np.eye(3).reshape(-1); x[12] = 0.0
    x[0:3] = [0.0, 0.0, mission.z_ground]                  # start on the ground, hover
    DT = 0.02; nsolve = max(1, int(nmpc.DT / DT))
    lat_steps = max(1, int(round(LAT / DT))); plat_steps = lat_steps
    REAR_FLOOR = argf("--rearfloor", 0.10) * MAX_THRUST     # mirror node rear_floor_cmd
    FRONT_CAP = argf("--frontcap", 0.90) * MAX_THRUST       # mirror node front_motor_cap
    KSAS_P = argf("--ksasp", 0.30); KSAS_R = argf("--ksasr", 0.20); KSAS_Y = argf("--ksasy", 0.15)
    KPITCH = argf("--kpitch", 0.0)    # elevator pitch-ATTITUDE hold (PD): sole pitch actuator at tilt~90
    LIFTGATE = argf("--liftgate", 0.0)  # lift-limited tilt gate (Track B #3b): cap tilt to current lift
    u = np.array([M * G / 4] * 4 + [0.0, 0, 0, 0]); u_nmpc = u.copy()
    hist = [u.copy() for _ in range(max(lat_steps, plat_steps) + 1)]
    LEASH_AHEAD = argf("--leashahead", 1.0)   # reference may lead the vehicle's path progress by [s]
    LEASH_BEHIND = argf("--leashbehind", 0.5)  # reference catches up if the vehicle overshoots by [s]
    SPEED_LEAD = argf("--speedlead", 0.4)     # trans_out: ref leads achieved-speed time by [s]
    print("clock: speed-gated trans_out (lead=%.1fs), leash ahead=%.1fs behind=%.1fs"
          % (SPEED_LEAD, LEASH_AHEAD, LEASH_BEHIND))
    print(" t   wall phase       x   xref   z  zref   V  tilt  AoA | errx errz ok")
    crashed = False; maxerrx = 0.0; maxerrz = 0.0
    t_now = 0.0; wall = 0.0; max_wall = mission.tf * 3.0 + 10.0; it = 0
    while t_now < mission.tf - 1e-3 and wall < max_wall:
        if it % nsolve == 0:
            xp = predict_forward(Ffull, x, hist[-lat_steps:], DT) if COMP > 0.5 else x
            anchor = mission.phase_at_time(t_now) in ("trans_out", "cruise", "trans_back")
            u_nmpc, ok = nmpc.solve_mission(xp, mission, t_now, anchor_xy=anchor)
        # OUTPUT STAGE (mirror the node loop, applied EVERY step): rear floor + front cap on the
        # motors, and an airspeed-gated rate-SAS on the aero surfaces (elevator damps pitch rate,
        # aileron roll, rudder yaw). Without this the offline sim lacks the fast damping the real
        # vehicle has -> the corridor pitch runs away even at zero latency.
        cl = lambda v, a, b: float(np.clip(v, a, b))
        u = u_nmpc.copy()
        u[2] = max(u[2], REAR_FLOOR); u[3] = max(u[3], REAR_FLOOR)
        u[0] = min(u[0], FRONT_CAP); u[1] = min(u[1], FRONT_CAP)
        Vn = float(np.linalg.norm(x[13:16])); wb = x[16:19]
        sas = cl((Vn - 3.0) / 4.0, 0.0, 1.0)
        DEm = SO3TransitionNMPC.DE_MAX; DAm = SO3TransitionNMPC.DA_MAX; DRm = SO3TransitionNMPC.DR_MAX
        # elevator PD: NMPC de + rate-SAS (damps q) + ATTITUDE HOLD (holds body pitch to the ref).
        # At high tilt the rear rotors unload and the front rotors point forward -> the elevator is
        # the ONLY pitch actuator, so it must actively HOLD pitch, not just damp rate. +e_pitch
        # (too nose-up) -> +de -> +Cm -> nose-DOWN (FLU) -> restores. Gated on airspeed (needs flow).
        Rm = x[3:12].reshape(3, 3)
        theta_act = math.asin(cl(Rm[2, 0], -1.0, 1.0))           # body pitch, nose-up +
        e_pitch = theta_act - mission._iv(t_now, mission.theta)  # deviation from the trim pitch ref
        u[5] = cl(cl(1.0 * u[5] / DEm, -1, 1) + sas * (KSAS_P * float(wb[1]) + KPITCH * e_pitch), -1, 1) * DEm
        u[6] = cl(cl(0.6 * u[6] / DAm, -1, 1) - sas * KSAS_R * float(wb[0]), -1, 1) * DAm
        u[7] = cl(cl(0.6 * u[7] / DRm, -1, 1) - sas * KSAS_Y * float(wb[2]), -1, 1) * DRm
        # LIFT-LIMITED TILT GATE (Track B #3b): never command tilt beyond what the CURRENT lift
        # supports (rotor_vertical + measured wing_lift >= weight). As the wing loads with speed the
        # cap relaxes toward 90deg -> anti-corridor (blocks the tilt-loss->drop->saturate->flip).
        if LIFTGATE > 0.5:
            Ttot = float(u[0] + u[1] + u[2] + u[3])
            vbb = x[3:12].reshape(3, 3).T @ x[13:16]
            Vb = float(np.linalg.norm(x[13:16]))
            aoa_m = math.atan2(-vbb[2], abs(vbb[0]) + 1e-6) if Vb > 2.0 else 0.0
            qbar = 0.5 * RHO * Vb * Vb * (Vb * Vb) / (Vb * Vb + 4.0)
            clw = float(np.clip(CL0 + CLA * aoa_m, -CLMAX, CLMAX))
            wing_lift = qbar * S_WING * clw
            cos_min = float(np.clip((M * G - wing_lift) / max(Ttot, 1.0), 0.05, 1.0))
            u[4] = min(u[4], math.acos(cos_min))
        hist.append(u.copy()); hist = hist[-200:]
        u_app = hist[-1 - plat_steps]
        x = x + DT * np.array(Ffull(x, u_app)).ravel()
        R = x[3:12].reshape(3, 3); Uc, _, Vt = np.linalg.svd(R); R = Uc @ Vt; x[3:12] = R.reshape(-1)
        # ---- clock: SPEED-GATED through the transition, position-leashed elsewhere. In trans_out
        # the mission time is tied to the vehicle's ACHIEVED forward speed (t = time_from_speed(V)),
        # so the reference TILT always tracks the real speed and can never lead it -> the
        # tilt-leads-speed corridor divergence cannot happen. A small SPEED_LEAD pulls it forward.
        # In climb/hold/cruise/back/settle/land the clock advances by DT, leashed to the vehicle's
        # projected path progress t_veh (robust to lag and overshoot). ----
        V_now = float(np.linalg.norm(x[13:16]))
        if mission.phase_at_time(t_now) == "trans_out":
            t_spd = mission.time_from_speed(V_now)
            t_now = min(t_now + 2.0 * DT, t_spd + SPEED_LEAD)
            t_now = max(t_now, t_spd - 0.2)
        else:
            t_veh = t_now; best_d = 1e18
            for j in range(-30, 61):                             # window [-0.6, +1.2] s
                tc = min(max(t_now + j * DT, 0.0), mission.tf)
                xrc, _ = mission.get(tc)
                d = float(np.linalg.norm(x[0:3] - xrc[0:3]))
                if d < best_d:
                    best_d = d; t_veh = tc
            t_now = min(t_now + DT, t_veh + LEASH_AHEAD)
            t_now = max(t_now, t_veh - LEASH_BEHIND)
        xr, _ = mission.get(t_now)
        errx = float(np.linalg.norm(x[0:2] - xr[0:2])); errz = float(x[2] - xr[2])
        maxerrx = max(maxerrx, errx); maxerrz = max(maxerrz, abs(errz))
        if x[2] < 0.2 or x[2] > 45:
            print("  !! CRASH at wall=%.1f t=%.1f z=%.1f" % (wall, t_now, x[2])); crashed = True; break
        if it % 25 == 0:
            ph = mission.phase_at_time(t_now)
            vI = x[13:16]; vb = R.T @ vI; V = float(np.linalg.norm(vI))
            aoa = math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-6)) if V > 0.5 else 0.0
            print("%4.1f %-9s %5.1f %5.1f  z%4.1f V%4.1f til%3.0f/%3.0f cmd%3.0f AoA%5.1f | ex%4.1f ez%+4.1f %d" %
                  (t_now, ph, x[0], xr[0], x[2], V, math.degrees(x[12]),
                   math.degrees(xr[12]), math.degrees(u_nmpc[4]), aoa, errx, errz, int(ok)))
        wall += DT; it += 1
    if not crashed:
        print("DONE  final x=%.1f (target %.0f)  z=%.2f  V=%.1f  wall=%.1fs | max|errx|=%.2f  max|errz|=%.2f"
              % (x[0], tx, x[2], float(np.linalg.norm(x[13:16])), wall, maxerrx, maxerrz))


def main(args=None):
    """console_scripts entry point.

    Delegates to the SAME dispatch as `python3 nmpc_so3_transition_node.py`, so `ros2 run` and a
    direct invocation cannot drift apart -- including the offline gate flags,
    which are how this controller is validated before it is ever flown.
    """
    import sys
    if "--mission" in sys.argv:
        mission_selftest()
    elif "--selftest" in sys.argv:
        selftest()
    else:
        run_ros()



if __name__ == "__main__":
    main()

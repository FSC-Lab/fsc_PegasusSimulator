#!/usr/bin/env python3
"""vtol_nmpc.py — STANDALONE full-state NMPC for the FSC quad-tiltrotor (90 deg wing-borne cruise).

ONE FILE, ZERO cross-imports (no nmpc_so3_transition_node / nmpc6_cruise_node / fsc_transition_ref).
Everything it needs is inlined: the sim-matched 6-DOF plant, the force-consistent trim schedule
(baked as a table), the NMPC, the ROS2 node, and an offline self-test.

  state  x(19) = [pos(3), R(9, row-major), tilt(1), vel_inertial(3), omega_body(3)]
  control u(8) = [T_FL, T_FR, T_RL, T_RR, tilt_cmd, elevator, aileron, rudder]
  Isaac tilt convention: 0 = rotors UP (hover), pi/2 = FORWARD (pure-puller cruise).

WHY a full-state NMPC: roll, pitch, yaw, tilt and the body rates are all inside ONE solve, so the
solver allocates 4 rotors + 3 surfaces against every coupled moment at once. A hand-rolled lateral
PD outside the solver is fragile exactly at 90 deg (front rotors horizontal -> differential thrust
makes YAW not roll; rear unloaded -> no roll authority).

HARD-WON CONSTRAINTS baked in (each one cost a HIL crash -- do not remove without re-testing):
  1. AIRSPEED TILT CLAMP: applied tilt <= trim_tilt(V) + margin. Tilting to 90 deg at ~1 m/s leaves
     NO pitch authority (rotors horizontal, elevator has no dynamic pressure) -> the nose falls
     (pitch -8 -> -80 deg in 6 s, elevator saturated) -> inverted. THE crash cause.
  2. REAL-TIME: this solve is ~20 ms on an idle machine but 50-95 ms under HIL load (Isaac + PX4 +
     GS contend for CPU). Run at a rate whose period EXCEEDS the measured worst case, and set
     latency_s >= the true loop latency -- under-compensation is fatal, over-compensation is safe.
  3. NEVER zero the velocity reference to "hold position": that plus a cruise tilt schedule is a
     contradictory command and the vehicle can never accelerate. Brake with a sqrt profile instead.
  4. STATE bounds on the body rates: unbounded, IPOPT explores omega_dot ~ -200 -> ill-conditioned.
  8. The loop MUST NOT be able to die: an uncaught exception stops publishing, PX4 HOLDS the
     last actuator command, and the vehicle tumbles on a stale setting (HIL bag vt1: actuators
     frozen byte-identical for 17 s while it fell). The loop body is wrapped -> log once + hover.
  5. On solve failure apply a SAFE fallback. NOT the cruise trim: HIL bag vt2 had ok=0 on every
     solve while inverted, and blindly applying urefs[0] (tilt=90 at speed) drove it deeper into the
     tumble -> the next solve failed too -> death spiral. Fall back to the trim ONLY while the state
     is near the reference; if the attitude has diverged, command LEVEL HOVER (tilt 0, equal thrust),
     which is the only thing recoverable from an upset.
  6. Penalise the aero surfaces: unpenalised, IPOPT parks them anywhere (rudder pinned at 1.0).
  7. GHOST PUBLISHERS: any other controller node publishing actuators will fight this one and flip
     the vehicle at takeoff. Fly with a minimal stack (scripts/start_vtol_minimal.sh).
"""
import inspect
import math
import os
import sys

import numpy as np
import casadi as ca

# ============================== airframe (FSC quad-tiltrotor) ==============================
M = 3.45; G = 9.81
# ASSEMBLED-vehicle inertia about the CG (body link + ten 0.05 kg articulated links), NOT
# VTOLConfig.inertia, which describes the body link alone. Reproduce with tests/plant_double.py.
IXX, IYY, IZZ = 0.265, 0.303, 0.490
MAX_THRUST = 18.0                      # per rotor [N]
CG_X = 0.176                           # assembled CG; the sim's balanced hover confirms it
# Rotor HUB positions rel CG (thrust is applied at the hub). The z arms used to be +0.02 for
# all four; the front pair is really BELOW the CG at -0.033 -- wrong sign, and it IS the
# pitch moment at full tilt where the thrust is horizontal (My = z_arm * Fx).
RPOS = np.array([[0.459 - CG_X,  0.284, -0.033],   # FL (front, tiltable)
                 [0.459 - CG_X, -0.284, -0.031],   # FR (front, tiltable)
                 [-0.112 - CG_X,  0.284, 0.016],   # RL (rear, fixed)
                 [-0.112 - CG_X, -0.284, 0.018]])  # RR (rear, fixed)
XF, YF, ZF = RPOS[0, 0], RPOS[0, 1], RPOS[0, 2]
XR = RPOS[2, 0]
SPIN = np.array([+1.0, -1.0, -1.0, +1.0])
TILTABLE = np.array([1.0, 1.0, 0.0, 0.0])
CQ = 0.12                              # = VTOLConfig.rotor_moment_ratio (was 0.06 = half)
TAU_BETA = 0.12                        # tilt servo time constant [s]
TILT_RATE = math.radians(80.0)         # sim servo rate limit [rad/s]

# aero -- STANDARD (FRD) coefficients, converted at the boundary of _aero_body(). Mirrors
# extensions/.../dynamics/vtol_aerodynamics.py; asserted by tests/test_controller_aero_match.py.
RHO = 1.225; S_WING = 0.35; B_WING = 1.4; C_WING = 0.25
CL0 = 0.15; CLA = 4.8; CLQ = 7.0; CLDE = 0.70
CD0 = 0.04; KIND = 0.071; CDB2 = 0.20
CYB = -0.98; CYDR = 0.24; CY_MAX = 0.6
CLB = -0.12; CLDA = 0.49; CLP = -0.5; CLR = 0.1; CLDR = 0.02
CM0 = 0.02; CMA = -1.2; CMQ = -12.0; CMDE = -1.75
CNB = 0.11; CNDR = -0.109; CNR = -0.2; CNP = -0.06; CNDA = 0.02
STALL = math.radians(13.0); BETA_BREAK = math.radians(20.0)
BLEND_M = 25.0; MIN_V = 0.25

NX, NU = 19, 8
DE_MAX = math.radians(25.0); DA_MAX = math.radians(25.0); DR_MAX = math.radians(30.0)


def _cl(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ============================== trim schedule (baked table) ==============================
# Hover->cruise trim. The AVTOL coupled solver supplies the ATTITUDE schedule (tilt, alpha,
# theta, elevator) -- what it is good at -- and the two thrusts are then solved from THIS
# airframe's geometry for BOTH balances at once:
#     vertical:  Tf*cos(beta-theta) + Tr*cos(theta) + Fz_aero = W
#     pitch:     Tf*(zf*sin b - xf*cos b) - xr*Tr + My_aero_FLU = 0
# REGENERATED 2026-09-06. The previous table was baked before two errors were found: the
# aero pitch moment entered the split without being negated into the FLU convention, and the
# generator's single Ttot assumes a tiltrotor whose WHOLE rotor set rotates -- this airframe
# tilts only the front pair, which under-counted the vertical thrust by up to 7.8 N (23% of
# weight) mid-corridor. Plugged into build_ode the old table left up to 5.3 rad/s^2 of
# residual pitch acceleration: it was never a trim. This one leaves <0.05.
# Regenerate with nmpc6_cruise_node.Schedule; columns: V, tilt, theta, Tf_pair, Tr_pair, de.
TRIM = np.array([
    (0.0, 0.00001, 0.00000, 17.069, 16.775, -0.13414),
    (0.5, 0.04927, 0.00000, 17.033, 16.828, -0.13414),
    (1.0, 0.09836, 0.00000, 17.024, 16.890, -0.13414),
    (1.5, 0.14745, 0.00000, 17.041, 16.960, -0.13414),
    (2.0, 0.19653, 0.00000, 17.085, 17.040, -0.13414),
    (2.5, 0.24560, 0.00873, 17.115, 17.077, -0.13414),
    (3.0, 0.29468, 0.01745, 17.146, 17.087, -0.13414),
    (3.5, 0.34375, 0.02618, 17.172, 17.062, -0.13414),
    (4.0, 0.39283, 0.03491, 17.189, 16.994, -0.13414),
    (4.5, 0.44190, 0.04363, 17.189, 16.876, -0.13414),
    (5.0, 0.49097, 0.05236, 17.168, 16.700, -0.13414),
    (5.5, 0.54004, 0.06109, 17.117, 16.458, -0.13414),
    (6.0, 0.58911, 0.06981, 17.030, 16.145, -0.13414),
    (6.5, 0.63818, 0.07854, 16.899, 15.755, -0.13414),
    (7.0, 0.68726, 0.08727, 16.717, 15.282, -0.13414),
    (7.5, 0.73633, 0.09599, 16.473, 14.724, -0.13414),
    (8.0, 0.78540, 0.10472, 16.162, 14.078, -0.13414),
    (8.5, 0.83447, 0.11345, 15.774, 13.345, -0.13414),
    (9.0, 0.88354, 0.12217, 15.305, 12.528, -0.13414),
    (9.5, 0.93261, 0.13090, 14.750, 11.634, -0.13414),
    (10.0, 0.98168, 0.13963, 14.111, 10.672, -0.13414),
    (10.5, 1.03075, 0.13963, 13.677, 10.045, -0.13414),
    (11.0, 1.07983, 0.13963, 13.154, 9.388, -0.13414),
    (11.5, 1.12890, 0.13963, 12.515, 8.702, -0.13414),
    (12.0, 1.17797, 0.13963, 11.724, 7.988, -0.13414),
    (12.5, 1.22704, 0.13963, 10.729, 7.247, -0.13414),
    (13.0, 1.27612, 0.13963, 9.456, 6.479, -0.13414),
    (13.5, 1.32519, 0.13963, 7.790, 5.685, -0.13414),
    (14.0, 1.37427, 0.13265, 6.824, 5.077, -0.12652),
    (14.5, 1.42335, 0.12392, 6.270, 4.592, -0.11700),
    (15.0, 1.47243, 0.11519, 6.028, 4.195, -0.10748),
    (15.5, 1.52153, 0.10821, 5.344, 3.773, -0.09987),
    (16.0, 1.57078, 0.09948, 6.611, 3.556, -0.09035),
])


def trim_at(V, tilt_cap=math.pi / 2):
    """Trim (tilt, theta, Tf_pair, Tr_pair, de) at airspeed V, linearly interpolated."""
    V = float(min(max(V, 0.0), TRIM[-1, 0]))
    c = [float(np.interp(V, TRIM[:, 0], TRIM[:, i])) for i in range(1, 6)]
    return dict(beta=min(c[0], tilt_cap), theta=c[1], Tf=c[2], Tr=c[3], de=c[4])


# ============================== plant (matches the sim) ==============================
def _skew(w):
    return ca.vertcat(ca.horzcat(0, -w[2], w[1]),
                      ca.horzcat(w[2], 0, -w[0]),
                      ca.horzcat(-w[1], w[0], 0))


def _aero_body(vb, omega, de, da, dr):
    """FLU in, FLU out; STANDARD coefficients inside. FLU->FRD is 180 deg about x, so
    v, w, q, r (hence alpha, beta) and Fy, Fz, My, Mz flip; u, p and Mx do not.

    No forward-speed gate: the blended flat-plate model is valid at every angle, and gating
    on u alone made the model blind to descent and lateral translation, where the plant
    produces real forces. V is floored at MIN_V instead, which keeps qbar ~ 0 at rest and
    the rate non-dimensionalisation bounded."""
    u = vb[0]; v = -vb[1]; w = -vb[2]
    p = omega[0]; q = -omega[1]; r = -omega[2]
    V = ca.sqrt(u * u + v * v + w * w + MIN_V * MIN_V)
    alpha = ca.atan2(w, ca.sqrt(u * u + 1e-4))
    beta = ca.asin(ca.fmax(ca.fmin(v / V, 0.99), -0.99))
    qbar = 0.5 * RHO * V * V
    phat = p * B_WING / (2 * V); qhat = q * C_WING / (2 * V); rhat = r * B_WING / (2 * V)

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
    F_frd = -D * vdir + L * ca.vertcat(sa, 0, -ca_) + ca.vertcat(0, Y, 0)

    a_att = ca.fmax(ca.fmin(alpha, STALL), -STALL)
    Cl_ = CLB * beta + CLDA * da + CLP * phat + CLR * rhat + CLDR * dr
    Cm = CM0 + CMA * a_att + CMDE * de + CMQ * qhat
    Cn = CNB * beta + CNDR * dr + CNR * rhat + CNP * phat + CNDA * da
    M_frd = qbar * S_WING * ca.vertcat(B_WING * Cl_, C_WING * Cm, B_WING * Cn)

    return (ca.vertcat(F_frd[0], -F_frd[1], -F_frd[2]),
            ca.vertcat(M_frd[0], -M_frd[1], -M_frd[2]))


def build_ode():
    """6-DOF dynamics, byte-matched to the Isaac/Pegasus VTOL plant."""
    x = ca.SX.sym("x", NX); u = ca.SX.sym("u", NU)
    R = ca.reshape(x[3:12], 3, 3).T                 # stored row-major
    beta = x[12]; vI = x[13:16]; om = x[16:19]
    T = u[0:4]; bcmd = u[4]; de, da, dr = u[5], u[6], u[7]
    vb = ca.mtimes(R.T, vI)
    Fthr = ca.SX.zeros(3); Mthr = ca.SX.zeros(3)
    for i in range(4):
        tilt = beta * TILTABLE[i]
        st, ct = ca.sin(tilt), ca.cos(tilt)
        Fi = ca.vertcat(T[i] * st, 0, T[i] * ct)
        Fthr = Fthr + Fi
        Mthr = Mthr + ca.cross(ca.DM(RPOS[i]), Fi)
        k = -SPIN[i] * CQ * T[i]                    # yaw reaction about the tilted axis
        Mthr = Mthr + ca.vertcat(k * st, 0, k * ct)
    Fa, Ma = _aero_body(vb, om, de, da, dr)
    Imat = ca.DM(np.diag([IXX, IYY, IZZ])); Iinv = ca.DM(np.diag([1 / IXX, 1 / IYY, 1 / IZZ]))
    vdot = (1.0 / M) * ca.mtimes(R, Fthr + Fa) + ca.vertcat(0, 0, -G)
    omdot = ca.mtimes(Iinv, (Mthr + Ma) - ca.cross(om, ca.mtimes(Imat, om)))
    Rdot = ca.reshape(ca.mtimes(R, _skew(om)).T, 9, 1)
    return ca.Function("ode", [x, u], [ca.vertcat(vI, Rdot, (bcmd - beta) / TAU_BETA, vdot, omdot)])


def _rk4(F, x, u, dt, n=1):
    h = dt / n
    for _ in range(n):
        k1 = F(x, u); k2 = F(x + h / 2 * k1, u); k3 = F(x + h / 2 * k2, u); k4 = F(x + h * k3, u)
        x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return x


def predict(F, x, ubuf, dt, sub=3):
    """Delay compensation: roll the measured state forward through the in-flight commands."""
    x = np.asarray(x, float).copy(); h = dt / sub
    for uu in ubuf:
        for _ in range(sub):
            x = x + h * np.array(F(x, uu)).ravel()
        U_, _, Vt = np.linalg.svd(x[3:12].reshape(3, 3)); x[3:12] = (U_ @ Vt).reshape(-1)
    return x


def R_ref(heading, theta):
    """+theta = NOSE-UP, matching the TRIM table (which comes from a standard-convention
    trim solver). This used to build Ry(+theta) = nose-DOWN; it cancelled against the
    inverted aero model, so a +8 deg cruise trim was flown 8 deg nose-down."""
    cy, sy = math.cos(heading), math.sin(heading)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[ct, 0, -st], [0, 1, 0], [st, 0, ct]])


# ============================== the NMPC ==============================
class VtolNMPC:
    N = 8; DT = 0.10; NRK4 = 1; MAX_ITER = 25
    w_pos = 25.0; w_vel = 30.0; w_att = 150.0; w_beta = 60.0   # tilt tracks the schedule (the CLAMP owns corridor safety)
    w_om = 12.0; w_u = 0.02; w_de = 1.0; w_surf = 3.0; w_du = 4.0
    w_posT = 40.0; w_attT = 200.0
    # REAR-ROTOR FLOOR [N per rotor]: at full unload Tr->0 and the rear pair is the ONLY remaining
    # pitch/roll authority, so cruise develops a limit cycle (selftest: tilt hunting 64-89 deg, AoA
    # swinging 0.7-12.5, altitude creeping). Keep a little thrust on the rear so authority survives.
    REAR_MIN = 1.5
    USE_JIT = True

    def __init__(self):
        F = build_ode(); N = self.N
        X = ca.SX.sym("X", NX, N + 1); U = ca.SX.sym("U", NU, N)
        P = ca.SX.sym("P", NX + NX * (N + 1) + NU * N + NU)
        xr = lambda k: P[NX + NX * k: NX + NX * (k + 1)]
        ub = NX + NX * (N + 1)
        ur = lambda k: P[ub + NU * k: ub + NU * (k + 1)]
        uprev = P[ub + NU * N: ub + NU * N + NU]
        J = 0; g = [X[:, 0] - P[0:NX]]
        for k in range(N):
            xk, uk, r, ru = X[:, k], U[:, k], xr(k), ur(k)
            Rk = ca.reshape(xk[3:12], 3, 3).T; Rrk = ca.reshape(r[3:12], 3, 3).T
            J += (self.w_pos * ca.sumsqr(xk[0:3] - r[0:3])
                  + self.w_vel * ca.sumsqr(xk[13:16] - r[13:16])
                  + self.w_att * 0.5 * (3.0 - ca.trace(ca.mtimes(Rrk.T, Rk)))   # SO(3) geodesic
                  + self.w_beta * (xk[12] - r[12]) ** 2
                  + self.w_om * ca.sumsqr(xk[16:19])
                  + self.w_u * ca.sumsqr(uk[0:4] - ru[0:4])
                  + self.w_de * (uk[5] - ru[5]) ** 2
                  + self.w_surf * (uk[6] ** 2 + uk[7] ** 2)
                  + self.w_du * ca.sumsqr(uk - (uprev if k == 0 else U[:, k - 1])))
            g.append(X[:, k + 1] - _rk4(F, xk, uk, self.DT, self.NRK4))
        rN = xr(N)
        RN = ca.reshape(X[3:12, N], 3, 3).T; RrN = ca.reshape(rN[3:12], 3, 3).T
        J += self.w_posT * ca.sumsqr(X[0:3, N] - rN[0:3])
        J += self.w_attT * 0.5 * (3.0 - ca.trace(ca.mtimes(RrN.T, RN)))
        gv = ca.vertcat(*g)
        z = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.max_iter": self.MAX_ITER,
                "ipopt.sb": "yes", "ipopt.acceptable_iter": 6, "ipopt.acceptable_tol": 1e-4,
                "ipopt.mu_strategy": "adaptive", "ipopt.warm_start_init_point": "yes"}
        if self.USE_JIT:
            opts.update({"jit": True, "compiler": "shell", "jit_serialize": "embed",
                         "jit_options": {"flags": ["-O2"], "verbose": False}})
        self.iU = NX * (N + 1)
        import hashlib
        key = hashlib.md5(repr([self.N, self.DT, self.NRK4, self.MAX_ITER, self.w_pos, self.w_vel,
                                self.w_att, self.w_beta, self.w_om, self.w_u, self.w_de,
                                self.w_surf, self.w_du, self.w_posT, self.w_attT, self.REAR_MIN,
                                # the key MUST cover everything baked into the compiled
                                # solver, not just the weights: a serialized solver is
                                # reloaded wholesale, so a model change with an unchanged
                                # key silently no-ops (audit C6).
                                inspect.getsource(_aero_body), inspect.getsource(build_ode),
                                M, G, IXX, IYY, IZZ, MAX_THRUST, CQ, TAU_BETA,
                                RPOS.tolist()]).encode()).hexdigest()[:10]
        cache = os.path.expanduser(f"~/.cache/vtol_nmpc_N{self.N}_{key}.casadi")
        self.solver = None
        if self.USE_JIT and os.path.exists(cache):
            try:
                self.solver = ca.Function.load(cache); print(f"[vtol_nmpc] cache hit: {cache}")
            except Exception:
                self.solver = None
        if self.solver is None:
            print("[vtol_nmpc] building (JIT, a few minutes the first time)...")
            self.solver = ca.nlpsol("vtol_nmpc", "ipopt", {"x": z, "f": J, "g": gv, "p": P}, opts)
            if self.USE_JIT:
                try:
                    os.makedirs(os.path.dirname(cache), exist_ok=True); self.solver.save(cache)
                    print(f"[vtol_nmpc] cached: {cache}")
                except Exception as e:
                    print(f"[vtol_nmpc] cache save failed: {e}")
        lbx = -1e9 * np.ones(z.shape[0]); ubx = 1e9 * np.ones(z.shape[0])
        for k in range(N):
            b = self.iU + NU * k
            for i in range(4):
                lbx[b + i] = (self.REAR_MIN if i >= 2 else 0.0); ubx[b + i] = MAX_THRUST
            lbx[b + 4] = 0.0; ubx[b + 4] = math.pi / 2
            lbx[b + 5], ubx[b + 5] = -DE_MAX, DE_MAX
            lbx[b + 6], ubx[b + 6] = -DA_MAX, DA_MAX
            lbx[b + 7], ubx[b + 7] = -DR_MAX, DR_MAX
        for k in range(N + 1):                       # STATE bounds on the stiff rates (constraint 4)
            b = NX * k
            for i in range(3):
                lbx[b + 16 + i], ubx[b + 16 + i] = -3.0, 3.0
            lbx[b + 15], ubx[b + 15] = -12.0, 12.0
        self.lbx, self.ubx = lbx, ubx
        self.lbg = np.zeros(gv.shape[0]); self.ubg = np.zeros(gv.shape[0])
        self.zsol = None; self._zgood = None
        self._uprev = np.array([M * G / 4] * 4 + [0, 0, 0, 0], float)

    @staticmethod
    def safe_hover():
        """Recoverable command for an upset: rotors UP, equal thrust, surfaces neutral."""
        return np.array([M * G / 4] * 4 + [0.0, 0.0, 0.0, 0.0], float)

    @staticmethod
    def diverged(x):
        """True if the attitude is too far off to trust a trim feed-forward."""
        R = np.asarray(x[3:12], float).reshape(3, 3)
        roll = abs(math.degrees(math.atan2(R[2, 1], R[2, 2])))
        pitch = abs(math.degrees(math.asin(_cl(R[2, 0], -1.0, 1.0))))
        return (roll > 50.0) or (pitch > 35.0) or (R[2, 2] < 0.4)

    def solve(self, x0, xrefs, urefs):
        P = np.concatenate([np.asarray(x0, float).ravel()]
                           + [np.asarray(a, float).ravel() for a in xrefs]
                           + [np.asarray(a, float).ravel() for a in urefs] + [self._uprev])
        if self.zsol is None:
            self.zsol = np.zeros(self.iU + NU * self.N)
            for k in range(self.N + 1):
                self.zsol[NX * k:NX * (k + 1)] = xrefs[min(k, len(xrefs) - 1)]
            for k in range(self.N):
                self.zsol[self.iU + NU * k:self.iU + NU * (k + 1)] = urefs[min(k, len(urefs) - 1)]
        try:
            sol = self.solver(x0=self.zsol, lbx=self.lbx, ubx=self.ubx,
                              lbg=self.lbg, ubg=self.ubg, p=P)
            st = self.solver.stats()["return_status"]
            ok = ("Solve_Succeeded" in st) or ("Acceptable" in st)
            zs = np.array(sol["x"]).ravel(); u0 = zs[self.iU:self.iU + NU]
            if ok and not np.any(np.isnan(u0)):
                self.zsol = zs; self._zgood = zs.copy()
            else:
                ok = False; self.zsol = self._zgood                   # constraint 5
                u0 = (self.safe_hover() if self.diverged(x0)
                      else np.asarray(urefs[0], float).copy())
        except Exception:
            ok = False; self.zsol = self._zgood
            u0 = (self.safe_hover() if self.diverged(x0) else np.asarray(urefs[0], float).copy())
        self._uprev = u0.copy()
        return u0, ok


def build_refs(nmpc, p0, heading, z_ref, V_now, Vc, ttr, s_now, hold, tilt_cap=math.pi / 2,
               dest=None, brake_a=2.0):
    """Horizon reference: ramp the target speed toward Vc, take the trim at each stage, wrap into a
    full 19-state ref (roll=0, yaw=heading, pitch=trim). `dest` = along-path destination (outer SMC):
    braked with a sqrt profile -- NEVER by zeroing the velocity (constraint 3)."""
    fwd = np.array([math.cos(heading), math.sin(heading), 0.0])
    xrefs, urefs, s = [], [], s_now
    for k in range(nmpc.N + 1):
        vtar = 0.0 if hold else min(max(V_now, 0.0) + (k * nmpc.DT) * (Vc / max(ttr, 1e-3)), Vc)
        if dest is not None:
            rem = max(dest - s, 0.0)
            vtar = min(vtar, math.sqrt(2.0 * brake_a * rem))
        sc = trim_at(vtar, tilt_cap)
        if k > 0:
            s += vtar * nmpc.DT
        pos = p0 + fwd * s; pos[2] = z_ref
        xrefs.append(np.concatenate([pos, R_ref(heading, sc["theta"]).reshape(-1),
                                     [sc["beta"]], fwd * vtar, [0.0, 0.0, 0.0]]))
        if k < nmpc.N:
            urefs.append(np.array([sc["Tf"] / 2, sc["Tf"] / 2, sc["Tr"] / 2, sc["Tr"] / 2,
                                   sc["beta"], sc["de"], 0.0, 0.0]))
    return xrefs, urefs


# ============================== offline self-test (vs the same plant) ==============================
def selftest():
    def a(n, d): return float(sys.argv[sys.argv.index(n) + 1]) if n in sys.argv else d
    Vc = a("--vc", 16.0); cap = math.radians(a("--cap", 90.0)); ttr = a("--ttr", 18.0)
    dur = a("--dur", 34.0); z0 = a("--z0", 20.0); LAT = a("--lat", 0.25)
    roll0 = math.radians(a("--roll0", 0.0)); yaw0 = math.radians(a("--yaw0", 0.0))
    nmpc = VtolNMPC(); F = build_ode()
    DT = 0.02; nsolve = max(1, int(nmpc.DT / DT)); ln = max(1, int(round(LAT / DT)))
    x = np.zeros(NX); x[3:12] = np.eye(3).reshape(-1); x[0:3] = [0, 0, z0]
    if abs(roll0) > 1e-3 or abs(yaw0) > 1e-3:
        cr, sr = math.cos(roll0), math.sin(roll0); cy, sy = math.cos(yaw0), math.sin(yaw0)
        x[3:12] = (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
                   @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])).reshape(-1)
    p0 = x[0:3].copy(); u = np.array([M * G / 4] * 4 + [0, 0, 0, 0]); hist = [u.copy()] * (ln + 1)
    print("standalone vtol_nmpc: hover -> %.0f deg wing-borne (Vc=%.0f, lat=%.2f)"
          % (math.degrees(cap), Vc, LAT))
    print("   t     x     z    V  tilt | roll pitch  yaw |  AoA |  Tf   Tr | ok")
    for it in range(int(dur / DT)):
        R = x[3:12].reshape(3, 3); v = x[13:16]; V = float(np.linalg.norm(v))
        if it % nsolve == 0:
            xp = predict(F, x, hist[-ln:], DT) if LAT > 1e-3 else x
            s_now = float(np.dot(xp[0:3] - p0, [math.cos(0.0), math.sin(0.0), 0.0]))
            xr_, ur_ = build_refs(nmpc, p0, 0.0, z0, float(np.linalg.norm(xp[13:16])), Vc, ttr,
                                  s_now, (it * DT) < 2.0, cap)
            u, ok = nmpc.solve(xp, xr_, ur_)
            u[4] = min(u[4], trim_at(V, cap)["beta"] + math.radians(12.0))   # constraint 1
        hist.append(u.copy()); hist = hist[-200:]
        x = x + DT * np.array(F(x, hist[-1 - ln])).ravel()
        U_, _, Vt = np.linalg.svd(x[3:12].reshape(3, 3)); x[3:12] = (U_ @ Vt).reshape(-1)
        if x[2] < 0.3 or x[2] > 60:
            print("  !! CRASH t=%.1f z=%.1f" % (it * DT, x[2])); return
        if it % 25 == 0:
            vb = R.T @ v
            aoa = math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-6)) if V > 1 else 0.0
            print("%5.1f %5.1f %5.1f %4.1f %5.0f |%5.0f %5.0f %5.0f |%5.1f | %4.1f %4.1f | %d"
                  % (it * DT, x[0], x[2], V, math.degrees(x[12]),
                     math.degrees(math.atan2(R[2, 1], R[2, 2])),
                     -math.degrees(math.asin(_cl(R[2, 0], -1, 1))),
                     math.degrees(math.atan2(R[1, 0], R[0, 0])), aoa,
                     u[0] + u[1], u[2] + u[3], int(ok)))
    R = x[3:12].reshape(3, 3); vb = R.T @ x[13:16]
    print("DONE z=%.1f V=%.1f tilt=%.0f AoA=%.1f rotors=%.1fN (weight %.1f) roll=%.0f yaw=%.0f"
          % (x[2], np.linalg.norm(x[13:16]), math.degrees(x[12]),
             math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-6)), float(np.sum(u[0:4])), M * G,
             math.degrees(math.atan2(R[2, 1], R[2, 2])), math.degrees(math.atan2(R[1, 0], R[0, 0]))))


# ============================== ROS2 node ==============================
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

    class VtolNMPCNode(Node):
        def __init__(self):
            super().__init__("vtol_nmpc_control")
            P = self.declare_parameter
            P("px4_ns", "uav_0"); P("controller_id", "vtol_nmpc"); P("active_at_start", False)
            P("auto_arm", True); P("arm_after", 150)
            P("rate_hz", 10.0)          # period MUST exceed the worst-case HIL solve (constraint 2)
            P("latency_s", 0.25)        # >= true loop latency; under-comp is fatal
            P("vcruise", 16.0); P("cap_deg", 90.0); P("ttr", 18.0)
            P("settle_s", 2.0); P("cruise_alt", 15.0)
            P("tilt_margin_deg", 12.0)  # airspeed tilt clamp (constraint 1)
            P("tilt_slew_deg_s", 60.0); P("brake_accel", 2.0)
            ns = self.get_parameter("px4_ns").value
            pre = f"/{ns}" if ns else ""
            self.cid = self.get_parameter("controller_id").value
            self._active = bool(self.get_parameter("active_at_start").value)
            self.auto_arm = bool(self.get_parameter("auto_arm").value)
            self.arm_after = int(self.get_parameter("arm_after").value)
            self.rate = float(self.gp("rate_hz"))
            self.get_logger().info("building NMPC...")
            self.nmpc = VtolNMPC(); self.F = build_ode()
            self.get_logger().info("NMPC ready.")
            self.state = None; self.p0 = None; self.heading = 0.0; self.z0 = 0.0
            self.dest = None; self.armed = self.offb = False
            self.counter = self.last = self._eng = 0; self.ubuf = []
            self._beta_cmd = 0.0; self._beta_state = 0.0; self._ms = 0.0
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
            self.create_subscription(PositionControllerReference,
                                     f"{pre}/fsc_autopilot_ros2/position_controller/reference",
                                     self.on_ref, 10)
            self.create_timer(1.0 / self.rate, self.loop)
            self.get_logger().info(f"vtol_nmpc up: id={self.cid} rate={self.rate:.0f}Hz")

        def gp(self, n): return float(self.get_parameter(n).value)
        def now_us(self): return int(self.get_clock().now().nanoseconds / 1000)
        def on_status(self, m):
            self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            self.offb = (m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        def on_active(self, m): self._active = (m.data == self.cid)
        def on_ref(self, m): self.dest = np.array([m.position.x, m.position.y, m.position.z])
        def on_mocap(self, m):
            q = m.pose.orientation
            self.state = (np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]),
                          _q2R(q.x, q.y, q.z, q.w),
                          np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]),
                          np.array([m.twist.angular.x, m.twist.angular.y, m.twist.angular.z]))

        def loop(self):
            # constraint 8: an exception here would stop publishing and PX4 would freeze the last
            # command (bag vt1). Log it ONCE with a traceback, then hold a safe hover instead.
            try:
                self._loop()
            except Exception:
                if not getattr(self, "_died", False):
                    import traceback
                    self._died = True
                    self.get_logger().error("LOOP EXCEPTION (holding hover):\n" + traceback.format_exc())
                h = _cl(M * G / 4.0 / MAX_THRUST, 0.0, 1.0)
                self.pub([h] * 4, [0.0, 0.0, 0.0, 0.0, 0.0])

        def _loop(self):
            self.counter += 1
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
                self.z0 = max(float(p[2]), self.gp("cruise_alt"))
                self._eng = 0; self.ubuf = []; self._beta_cmd = self._beta_state = 0.0
                self.get_logger().info("ENGAGED at (%.1f,%.1f,%.1f) hdg=%.0f z0=%.1f"
                                       % (p[0], p[1], p[2], math.degrees(self.heading), self.z0))
            self._eng += 1
            dt = 1.0 / self.rate
            cap = math.radians(self.gp("cap_deg"))
            # tilt state: mocap gives no servo angle -> integrate the APPLIED cmd with the sim's rate limit
            self._beta_state += _cl((self._beta_cmd - self._beta_state) * dt / TAU_BETA,
                                    -TILT_RATE * dt, TILT_RATE * dt)
            x = np.concatenate([p, R.reshape(-1), [self._beta_state], v, w])
            ln = max(1, int(round(self.gp("latency_s") * self.rate)))
            if self.ubuf and self.gp("latency_s") > 1e-3:
                x = predict(self.F, x, self.ubuf[-ln:], dt)
            V = float(np.linalg.norm(x[13:16])); z = float(p[2])
            fwd = np.array([math.cos(self.heading), math.sin(self.heading), 0.0])
            s_now = float(np.dot(x[0:3] - self.p0, fwd))
            hold = (self._eng < int(self.gp("settle_s") * self.rate)) or (z < self.z0 - 1.0)
            dest_s = float(np.dot(self.dest - self.p0, fwd)) if self.dest is not None else None
            xr_, ur_ = build_refs(self.nmpc, self.p0, self.heading, self.z0, V,
                                  self.gp("vcruise"), self.gp("ttr"), s_now, hold, cap,
                                  dest_s, self.gp("brake_accel"))
            import time as _t
            t0 = _t.perf_counter(); u, ok = self.nmpc.solve(x, xr_, ur_)
            self._ms = (_t.perf_counter() - t0) * 1e3
            self.ubuf.append(u.copy()); self.ubuf = self.ubuf[-ln:]
            # constraint 1: never tilt past what the CURRENT airspeed can support, then slew-limit
            raw = min(float(_cl(u[4], 0.0, cap)),
                      trim_at(V, cap)["beta"] + math.radians(self.gp("tilt_margin_deg")))
            sl = math.radians(self.gp("tilt_slew_deg_s")) * dt
            self._beta_cmd = float(_cl(raw, self._beta_cmd - sl, self._beta_cmd + sl))
            mot = [_cl(u[i] / MAX_THRUST, 0.0, 1.0) for i in range(4)]
            self.pub(mot, [_cl(self._beta_cmd / (math.pi / 2), 0, 1),
                           _cl(self._beta_cmd / (math.pi / 2), 0, 1),
                           _cl(u[6] / DA_MAX, -1, 1), _cl(u[5] / DE_MAX, -1, 1),
                           _cl(u[7] / DR_MAX, -1, 1)])
            if self.counter % max(1, int(self.rate / 2)) == 0:
                vb = R.T @ v
                ye = math.atan2(R[1, 0], R[0, 0]) - self.heading
                self.get_logger().info(
                    "%s V=%.1f tilt=%.0f/%.0f pitch=%+.0f/%+.0f AoA=%+.0f roll=%+.0f yaw=%+.0f z=%.1f ok=%d %.0fms"
                    % ("hold" if hold else "fly", V, math.degrees(self._beta_cmd),
                       math.degrees(xr_[0][12]),
                       -math.degrees(math.asin(_cl(R[2, 0], -1, 1))),
                       -math.degrees(math.asin(_cl(xr_[0][3:12].reshape(3, 3)[2, 0], -1, 1))),
                       math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-6)) if V > 1 else 0.0,
                       math.degrees(math.atan2(R[2, 1], R[2, 2])),
                       math.degrees(math.atan2(math.sin(ye), math.cos(ye))), z, int(ok), self._ms))
            if self._active and self.auto_arm and self.counter >= self.arm_after and not (self.armed and self.offb):
                if self.counter - self.last >= 50:
                    self.last = self.counter
                    if not self.offb:
                        self.vcmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    elif not self.armed:
                        self.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

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
        rclpy.spin(VtolNMPCNode())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        selftest()
    else:
        run_ros()

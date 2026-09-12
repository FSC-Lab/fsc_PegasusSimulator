#!/usr/bin/env python3
"""6-STATE LONGITUDINAL PITCH-DYNAMICS NMPC for the FSC quad-tiltrotor -> genuine 90deg wing-borne.

WHY THIS NODE EXISTS: every controller tried before (SO(3) full-6DOF mission tracking, point-mass
nmpc_cruise) fails the rotor->wing HANDOFF corridor (V~7-13 m/s) because none co-optimize the PITCH
MOMENT dynamics: they plan body pitch `theta` as if directly commandable, but the real plant fights
it (nose-down when the thrust split isn't front-heavy; nose-up from redundant body-lean). The
corridor is only stabilizable if the controller PREDICTS AND COUNTERS the pitch moment.

MODEL: reduced longitudinal state X=[s, z, vs, vz, theta, q], control U=[Tf, Tr, beta, de]
(front-pair thrust, rear-pair thrust, front tilt, elevator). `qdot = My/Iyy` with the full pitch
moment = thrust-split moment (Tf,Tr,beta,geometry) + aero (cm_alpha*alpha + cm_de*de + cm_q*qhat).
Elevator is IN THE LOOP. Aero + geometry are imported from nmpc_so3_transition_node so the model is
byte-identical to the sim-matched plant `build_ode` (validated: `--validate`).

Isaac tilt convention: beta=0 rotors UP (hover), beta=90deg FORWARD (cruise/pure-puller).
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

import numpy as np
import casadi as ca

from px4_vtol_direct_actuator import nmpc_so3_transition_node as so3   # shared constants + build_ode (the TRUE plant, for offline test)

# ---- constants: import verbatim from the SO(3) node so the model == the sim-matched plant ----
M = so3.M; G = so3.G; IYY = so3.IYY
RHO = so3.RHO; S = so3.S_WING; C = so3.C_WING
CL0 = so3.CL0; CLA = so3.CLA; CLQ = so3.CLQ; CLDE = so3.CLDE; CD0 = so3.CD0; KIND = so3.KIND
CM0 = so3.CM0; CMA = so3.CMA; CMQ = so3.CMQ; CMDE = so3.CMDE
BLEND_M = so3.BLEND_M; MIN_V = so3.MIN_V
STALL = so3.STALL
MAX_THRUST = so3.MAX_THRUST
XF = float(so3.RPOS[0, 0]); ZF = float(so3.RPOS[0, 2]); XR = float(so3.RPOS[2, 0])   # rotor arms rel CG
YF = float(so3.RPOS[0, 1]); SPIN = so3.SPIN; CQ = so3.CQ                              # y-arm, spin dirs
TAU_BETA = so3.TAU_BETA


def _cl(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def lateral_control(R, w, beta, V, Tf, Tr, heading, g):
    """TILT/SPEED-AWARE lateral (roll=0 + yaw=heading) hold. Returns (mot[4], aileron, rudder).
    Differential rotor thrust controls roll/yaw ONLY at low tilt+speed -- at 90deg tilt the front
    rotors are horizontal (differential -> YAW not roll) and the rear is unloaded (no roll authority),
    and the spin-reaction torque rotates from yaw into roll. So the roll differential is scaled by
    cos(beta) (fades to 0 by 90deg, killing the yaw cross-coupling) and both fade out with airspeed
    while the AERO SURFACES (aileron/rudder, authority at cruise) fade IN. This is why the vehicle
    tumbled 'as soon as it hit 90deg tilt' with the tilt-naive version."""
    roll = math.atan2(R[2, 1], R[2, 2]); p_rate = float(w[0])
    yaw_cur = math.atan2(R[1, 0], R[0, 0])
    yaw_err = math.atan2(math.sin(yaw_cur - heading), math.cos(yaw_cur - heading)); r_rate = float(w[2])
    Mx = _cl(-g["kR_roll"] * roll - g["kW_roll"] * p_rate, -6.0, 6.0)
    Mz = _cl(-g["kR_yaw"] * yaw_err - g["kW_yaw"] * r_rate, -3.0, 3.0)
    lowspd = _cl((6.0 - V) / 6.0, 0.0, 1.0)      # differential thrust only while slow
    droll = lowspd * Mx * math.cos(beta) / (4.0 * YF)   # roll diff fades with tilt (cos b) AND speed
    dyaw = lowspd * Mz / (4.0 * CQ)              # spin-reaction yaw diff, only while slow
    mot = [Tf / 2 + droll - dyaw * SPIN[0], Tf / 2 - droll - dyaw * SPIN[1],
           Tr / 2 + droll - dyaw * SPIN[2], Tr / 2 - droll - dyaw * SPIN[3]]
    mot = [_cl(mm / MAX_THRUST, 0.0, 1.0) for mm in mot]
    # DYNAMIC-PRESSURE NORMALISED SURFACES. Surface power goes as V^2, so a fixed gain is
    # right at exactly one speed: k_ail / k_rud were hand-tuned constants behind an airspeed
    # ramp -- a gain schedule in disguise. Invert the surface's own moment equation instead,
    # so you ask for the moment you want and the deflection follows:
    #     M = qbar * S * span * C_delta * delta   ->   delta = M / (qbar * S * span * C_delta)
    # The delivered moment then matches the demand at every speed, the SIGN comes out of the
    # coefficient instead of a comment, and saturation becomes an honest "out of surface".
    # (The sign trap this removes was real: CNDR is NEGATIVE, so +dr yaws LEFT, and a
    # +k_rud*Mz rudder AMPLIFIED the yaw to -100 deg in HIL bag myrun.)
    qbarS = 0.5 * so3.RHO * V * V * so3.S_WING
    DA, DR = so3.SO3TransitionNMPC.DA_MAX, so3.SO3TransitionNMPC.DR_MAX
    aileron = _cl(Mx / (qbarS * so3.B_WING * so3.CLDA * DA + 1e-9), -1.0, 1.0)
    rudder = _cl(Mz / (qbarS * so3.B_WING * so3.CNDR * DR - 1e-9), -1.0, 1.0)
    return mot, aileron, rudder


def build_ode6():
    """Reduced 6-DOF longitudinal dynamics. Xdot = f(X,U). Mirrors build_ode() in the pitch plane."""
    X = ca.SX.sym("X", 6); U = ca.SX.sym("U", 4)
    s, z, vs, vz, th, q = X[0], X[1], X[2], X[3], X[4], X[5]
    Tf, Tr, beta, de = U[0], U[1], U[2], U[3]
    c, sth = ca.cos(th), ca.sin(th)
    # THETA IS NOSE-UP POSITIVE (2026-09-06), matching the trim schedule and every other node.
    # It used to be nose-DOWN positive (R = Ry(+theta)), which cancelled against the inverted
    # aero model so a +8 deg cruise trim was flown 8 deg nose-down. Body attitude is now
    # R = Ry(-theta), so v_body = R^T v_world = (c*vs + s*vz, 0, -s*vs + c*vz) in FLU.
    u_b = c * vs + sth * vz
    w_b = -sth * vs + c * vz
    # ---- aerodynamics, in the STANDARD (FRD) frame; FLU -> FRD is 180 deg about x ----
    u_f = u_b; w_f = -w_b
    V = ca.sqrt(vs * vs + vz * vz + MIN_V * MIN_V)      # floored, not gated (see aero_body)
    alpha = ca.atan2(w_f, ca.sqrt(u_f * u_f + 1e-4))
    qbar = 0.5 * RHO * V * V
    q_std = q                        # theta nose-up -> thetadot = q is already the FRD rate
    qhat = q_std * C / (2 * V)
    sig = 0.5 * (1.0 + ca.tanh(0.5 * BLEND_M * (ca.fabs(alpha) - STALL)))
    sa = ca.sin(alpha); ca_ = ca.cos(alpha)
    CL_lin = CL0 + CLA * alpha
    CL = ((1 - sig) * CL_lin + sig * 2.0 * ca.tanh(50.0 * alpha) * sa * sa * ca_
          + CLQ * qhat + CLDE * de)
    CD = CD0 + (1 - sig) * KIND * CL_lin * CL_lin + sig * 2.0 * sa * sa
    L = qbar * S * CL; D = qbar * S * CD
    Vb = ca.sqrt(u_f * u_f + w_f * w_f + MIN_V * MIN_V)
    Fax_f = -D * u_f / Vb + L * sa                       # FRD body x
    Faz_f = -D * w_f / Vb - L * ca_                      # FRD body z
    Fax = Fax_f; Faz = -Faz_f                            # FRD -> FLU
    # propulsion in the FLU body frame: front pair tilted beta, rear pair vertical
    Ftx = Tf * ca.sin(beta)
    Ftz = Tf * ca.cos(beta) + Tr
    Fbx = Ftx + Fax; Fbz = Ftz + Faz
    # world accelerations, v_world_dot = (1/M) Ry(-theta) Fb - g e_z
    vsdot = (c * Fbx - sth * Fbz) / M
    vzdot = (sth * Fbx + c * Fbz) / M - G
    # PITCH MOMENT, nose-UP positive (so it drives thetadot = q directly).
    a_att = ca.fmax(ca.fmin(alpha, STALL), -STALL)
    Cm = CM0 + CMA * a_att + CMQ * qhat + CMDE * de      # +Cm = nose-up (standard)
    Ma = qbar * S * C * Cm
    # thrust moment: this expression is nose-DOWN positive (FLU My), so negate it
    Mthr = -(Tf * (ZF * ca.sin(beta) - XF * ca.cos(beta)) - XR * Tr)
    qdot = (Mthr + Ma) / IYY
    Xdot = ca.vertcat(vs, vz, vsdot, vzdot, q, qdot)
    return ca.Function("ode6", [X, U], [Xdot])


def x6_to_x19(X6, U4):
    """Embed a longitudinal 6-state + 4-control into the full 19-state + 8-control for build_ode."""
    s, z, vs, vz, th, q = [float(a) for a in X6]
    Tf, Tr, beta, de = [float(a) for a in U4]
    c, sth = math.cos(th), math.sin(th)
    Rp = np.array([[c, 0, -sth], [0, 1, 0], [sth, 0, c]])       # Ry(-theta): +theta = nose-UP
    # omega is the FLU BODY rate, where +q_y is NOSE-DOWN; this model's q is nose-UP, so it
    # enters the 19-state negated. (Getting this wrong is invisible in thetadot -- both sides
    # then agree trivially -- and shows up only as a sign-flipped qdot.)
    x19 = np.concatenate([[s, 0.0, z], Rp.reshape(-1), [beta], [vs, 0.0, vz], [0.0, -q, 0.0]])
    u8 = np.array([Tf / 2, Tf / 2, Tr / 2, Tr / 2, beta, de, 0.0, 0.0])
    return x19, u8


def validate_model():
    """Prove build_ode6 reproduces build_ode's longitudinal derivatives (vsdot, vzdot, thetadot, qdot)."""
    F6 = build_ode6(); F19 = so3.build_ode()
    print("Validate 6-state model vs the TRUE plant build_ode (longitudinal derivatives):")
    print(" case (V,tilt,theta,q,Tf,Tr,de)          | vsdot 6/19  vzdot 6/19  thdot 6/19  qdot 6/19")
    cases = [
        (0.0, 0.0, 0.0, 0.0, 17.4, 17.4, 0.0),      # hover
        (8.0, 20.0, 5.0, 0.0, 12.0, 8.0, -5.0),     # mid-transition
        (13.0, 60.0, 7.0, 0.1, 10.0, 3.0, -6.0),    # high tilt
        (16.0, 88.0, 6.9, 0.0, 4.0, 0.5, -6.0),     # near pure-puller cruise
        (10.0, 45.0, -4.0, -0.2, 14.0, 6.0, 2.0),   # nose-down, descending-ish
    ]
    maxerr = 0.0
    for (V, tilt, th_d, q, Tf, Tr, de) in cases:
        th = math.radians(th_d); beta = math.radians(tilt)
        # put the airspeed along the flight path at ~level (vs=V, vz=0) for the probe
        vs = V; vz = 0.0
        X6 = np.array([0.0, 10.0, vs, vz, th, q]); U4 = np.array([Tf, Tr, beta, de])
        xd6 = np.array(F6(X6, U4)).ravel()
        x19, u8 = x6_to_x19(X6, U4)
        xd19 = np.array(F19(x19, u8)).ravel()
        # extract longitudinal derivatives from the 19-state xdot
        vsdot19 = xd19[13]; vzdot19 = xd19[15]; qdot19 = xd19[17]
        # theta is nose-UP here and omega_y is nose-DOWN in the 19-state, so the pitch RATE
        # and its derivative both change sign across the bridge: thetadot = -omega_y = q.
        d = dict(vsdot=(xd6[2], vsdot19), vzdot=(xd6[3], vzdot19),
                 thdot=(xd6[4], q), qdot=(xd6[5], -qdot19))
        # RELATIVE error: the probe points are not trimmed, so qdot legitimately reaches
        # hundreds of rad/s^2 and an absolute bar would be meaningless there.
        errs = [abs(a - b) / max(1.0, abs(b)) for a, b in d.values()]
        maxerr = max(maxerr, max(errs))
        print("  V%4.1f b%2.0f th%+3.0f q%+.1f T(%.0f,%.0f) de%+.0f | "
              "%+6.2f/%+6.2f %+6.2f/%+6.2f %+5.2f/%+5.2f %+7.2f/%+7.2f"
              % (V, tilt, th_d, q, Tf, Tr, de,
                 d["vsdot"][0], d["vsdot"][1], d["vzdot"][0], d["vzdot"][1],
                 d["thdot"][0], d["thdot"][1], d["qdot"][0], d["qdot"][1]))
    print("MAX relative error 6-state vs plant: %.2e  (%s)" % (maxerr, "MATCH" if maxerr < 1e-3 else "MISMATCH!"))
    return maxerr


def rk4_6(F, X, U, dt, n=2):
    h = dt / n
    for _ in range(n):
        k1 = F(X, U); k2 = F(X + h / 2 * k1, U)
        k3 = F(X + h / 2 * k2, U); k4 = F(X + h * k3, U)
        X = X + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return X


DE_MAX = math.radians(25.0)





# ============================ the trim schedule (reference) ============================
class Schedule:
    """Force-consistent transition trim keyed on airspeed V, from FSCTransitionProfile, plus the
    PITCH-BALANCED front/rear split. Gives (beta, theta, Tf, Tr, de) at any V -> the wing-borne
    reference the NMPC tracks. tilt/theta emerge hover@V=0 -> 90deg/+alpha@V=cruise."""
    def __init__(self, V_cruise=16.0, tilt_cap_deg=90.0, alpha_max_deg=8.0):
        from px4_vtol_direct_actuator.fsc_transition_ref import FSCTransitionProfile
        self.prof = FSCTransitionProfile(V_cruise=V_cruise, tf=20.0,
                                         tilt_cap_deg=tilt_cap_deg, alpha_max_deg=alpha_max_deg)
        self.Vc = V_cruise

    def at(self, V):
        V = float(max(V, 0.0))
        s = self.prof.by_speed(V)
        # theta_cmd = SPEED-GATED trim pitch (0 at hover, ramps to the +alpha trim over V~2..10). Using
        # the UNgated theta commands +8deg nose-up at hover -> on a HIL engage from a level hover the
        # body pitches up with NO airspeed for the elevator to hold it -> tumble. Gating -> clean engage.
        beta = float(s["tilt"]); theta = float(s["theta_cmd"])
        de = float(np.clip(s["de"], -DE_MAX, DE_MAX)); alpha = float(s["alpha"])

        # One implementation of the split, shared with TransitionRef and the mission
        # profile -- see nmpc_so3_transition_node.trim_split for why the generator's own
        # Ttot cannot be used on this airframe.
        Tf, Tr = so3.trim_split(beta, theta, V, de)
        return dict(beta=beta, theta=theta, Tf=Tf, Tr=Tr, de=de, V=V)   # Tf/Tr = PAIR totals


# ============================ 6-state longitudinal NMPC ============================
class NMPC6:
    N = 14; DT = 0.06; NRK4 = 2      # 0.84 s horizon; N=20/DT.05 was 53 ms/solve > 40 ms (25Hz budget)
    # weights: PITCH (theta) and TILT (beta) tracking dominate -> the solver holds the trim pitch
    # through the corridor using q/elevator/split (the point-mass nmpc_cruise cannot). Altitude firm.
    # altitude + pitch dominate; TILT is only a SOFT guide (w_beta low) so the NMPC self-limits the
    # tilt through the corridor (the schedule tilt(V) is too aggressive at low speed -> nose-down trap).
    w_z = 60.0; w_vs = 8.0; w_vz = 6.0; w_th = 90.0; w_beta = 12.0; w_q = 3.0
    w_Tf = 0.02; w_Tr = 0.02; w_de = 2.0; w_du = 1.0
    w_zT = 90.0; w_thT = 120.0

    def __init__(self):
        F = build_ode6()
        N, nx, nu = self.N, 6, 4
        X = ca.SX.sym("X", nx, N + 1); U = ca.SX.sym("U", nu, N)
        # params: x0(6) + per-stage ref [z,vs,vz,theta,beta,Tf,Tr,de](8)*(N) + uprev(4)
        P = ca.SX.sym("P", nx + 8 * N + nu)
        x0 = P[0:nx]
        def ref(k): return P[nx + 8 * k: nx + 8 * (k + 1)]
        uprev = P[nx + 8 * N: nx + 8 * N + nu]
        J = 0; g = [X[:, 0] - x0]
        for k in range(N):
            r = ref(k)
            zr, vsr, vzr, thr, betar, Tfr, Trr, der = [r[i] for i in range(8)]
            xk = X[:, k]; uk = U[:, k]
            J += (self.w_z * (xk[1] - zr) ** 2 + self.w_vs * (xk[2] - vsr) ** 2
                  + self.w_vz * (xk[3] - vzr) ** 2 + self.w_th * (xk[4] - thr) ** 2
                  + self.w_q * xk[5] ** 2 + self.w_beta * (uk[2] - betar) ** 2
                  + self.w_Tf * (uk[0] - Tfr) ** 2 + self.w_Tr * (uk[1] - Trr) ** 2
                  + self.w_de * (uk[3] - der) ** 2)
            up = uprev if k == 0 else U[:, k - 1]
            J += self.w_du * ca.sumsqr(uk - up)
            g.append(X[:, k + 1] - rk4_6(F, xk, uk, self.DT, self.NRK4))
        rN = ref(N - 1)
        J += self.w_zT * (X[1, N] - rN[0]) ** 2 + self.w_thT * (X[4, N] - rN[3]) ** 2
        gv = ca.vertcat(*g)
        z = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        nlp = {"x": z, "f": J, "g": gv, "p": P}
        opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.max_iter": 60,
                "ipopt.sb": "yes", "ipopt.acceptable_iter": 6,
                "ipopt.acceptable_tol": 1e-4, "ipopt.mu_strategy": "adaptive",
                "ipopt.warm_start_init_point": "yes",
                "jit": True, "compiler": "shell",
                "jit_options": {"flags": ["-O2"], "verbose": False},
                "jit_serialize": "embed"}       # embed compiled code -> Function.load skips recompile
        # cache the JIT-compiled solver so relaunches don't pay the ~2 min compile. Key MUST include
        # weights/N/DT/aero (all baked into the NLP graph) -- a stale cache would silently load the
        # wrong solver.
        import hashlib as _hl, os as _os
        key = _hl.md5(repr([self.N, self.DT, self.NRK4, self.w_z, self.w_vs, self.w_vz, self.w_th,
                            self.w_beta, self.w_q, self.w_Tf, self.w_Tr, self.w_de, self.w_du,
                            self.w_zT, self.w_thT, CL0, CLA, CM0, CMA, CMDE, float(STALL), KIND]
                           ).encode()).hexdigest()[:10]
        cache = _os.path.expanduser(f"~/.cache/nmpc6_cruise_N{self.N}_DT{int(self.DT*1000)}_{key}.casadi")
        self.solver = None
        if _os.path.exists(cache):
            try:
                self.solver = ca.Function.load(cache)
                print(f"[nmpc6] loaded cached JIT solver: {cache}")
            except Exception:
                self.solver = None
        if self.solver is None:
            self.solver = ca.nlpsol("nmpc6", "ipopt", nlp, opts)
            try:
                _os.makedirs(_os.path.dirname(cache), exist_ok=True); self.solver.save(cache)
                print(f"[nmpc6] cached JIT solver: {cache}")
            except Exception as e:
                print(f"[nmpc6] cache save failed ({e}) -- continuing")
        self.nx, self.nu = nx, nu
        self.iX = 0; self.iU = nx * (N + 1)
        # bounds
        lbx = -1e9 * np.ones(z.shape[0]); ubx = 1e9 * np.ones(z.shape[0])
        for k in range(N):
            b = self.iU + nu * k
            lbx[b + 0] = 0.0;   ubx[b + 0] = 2 * MAX_THRUST     # Tf pair
            lbx[b + 1] = 0.0;   ubx[b + 1] = 2 * MAX_THRUST     # Tr pair
            lbx[b + 2] = 0.0;   ubx[b + 2] = math.pi / 2        # beta
            lbx[b + 3] = -DE_MAX; ubx[b + 3] = DE_MAX           # de
        # STATE bounds: bound the stiff pitch (theta,q) + vertical rate so IPOPT can't explore the
        # huge-qdot region (unbalanced splits give qdot~-200 -> ill-conditioned RK4 -> solve failure).
        for k in range(N + 1):
            b = self.iX + nx * k
            lbx[b + 3] = -12.0; ubx[b + 3] = 12.0                          # vz
            lbx[b + 4] = math.radians(-30); ubx[b + 4] = math.radians(20)  # theta (nose-up +)
            lbx[b + 5] = -3.0;  ubx[b + 5] = 3.0                           # q
        self.lbx, self.ubx = lbx, ubx
        self.lbg = np.zeros(gv.shape[0]); self.ubg = np.zeros(gv.shape[0])
        self.zsol = None; self._u_good = np.array([M * G / 2, M * G / 2, 0.0, 0.0])

    def solve(self, x0, refs):
        """refs: list of N dicts (z,vs,vz,theta,beta,Tf,Tr,de). Returns U0=[Tf,Tr,beta,de], ok."""
        nx, nu, N = self.nx, self.nu, self.N
        P = [np.asarray(x0, float).ravel()]
        for k in range(N):
            r = refs[k]
            P.append(np.array([r["z"], r["vs"], r["vz"], r["theta"], r["beta"],
                               r["Tf"], r["Tr"], r["de"]]))
        P.append(getattr(self, "_uprev", self._u_good))
        P = np.concatenate([a.ravel() for a in P])
        if self.zsol is None:
            self.zsol = np.zeros(self.iU + nu * N)
            self.zsol[self.iU:] = np.tile(self._u_good, N)
        try:
            sol = self.solver(x0=self.zsol, lbx=self.lbx, ubx=self.ubx,
                              lbg=self.lbg, ubg=self.ubg, p=P)
            st = self.solver.stats()["return_status"]
            ok = ("Solve_Succeeded" in st) or ("Acceptable" in st)
            zsol = np.array(sol["x"]).ravel()
            u0 = zsol[self.iU:self.iU + nu]
            if ok:
                self.zsol = zsol; self._u_good = u0.copy()
            else:
                # FALLBACK: apply the REFERENCE feed-forward (a valid trim), not a stale frozen
                # control -- a frozen control diverges as the state moves. Keep zsol for warm-start.
                r0 = refs[0]
                u0 = np.array([r0["Tf"], r0["Tr"], r0["beta"], r0["de"]])
        except Exception:
            r0 = refs[0]
            u0 = np.array([r0["Tf"], r0["Tr"], r0["beta"], r0["de"]]); ok = False
        self._uprev = u0.copy()
        return u0, ok


def offline_test():
    """Drive NMPC6 against the TRUE plant (build_ode, full 19-state) through a hover->90deg-cruise
    transition. Reports whether it reaches + HOLDS a genuine wing-borne cruise (tilt~90, +AoA,
    altitude flat, rotors unloaded)."""
    def argf(n, d): return float(sys.argv[sys.argv.index(n) + 1]) if n in sys.argv else d
    Vc = argf("--vc", 16.0); cap = argf("--cap", 90.0); amax = argf("--amax", 8.0)
    T_tr = argf("--ttr", 14.0); dur = argf("--dur", 40.0); z0 = argf("--z0", 20.0)
    LAT = argf("--lat", 0.0)
    sched = Schedule(V_cruise=Vc, tilt_cap_deg=cap, alpha_max_deg=amax)
    nmpc = NMPC6(); F19 = so3.build_ode(); F6 = build_ode6()
    DT = 0.02; nsolve = max(1, int(nmpc.DT / DT))
    lat_n = max(1, int(round(LAT / DT)))
    def predict6(X6, ubuf):     # delay compensation: roll the state fwd by the latency (in-flight cmds)
        Xp = np.asarray(X6, float).copy()
        for uu in ubuf:
            u4 = np.array([uu[0] + uu[1], uu[2] + uu[3], uu[4], uu[5]])
            for _ in range(3):
                Xp = Xp + (DT / 3) * np.array(F6(Xp, u4)).ravel()
        return Xp
    # start hovering at z0
    x = np.zeros(19); x[3:12] = np.eye(3).reshape(-1); x[0:3] = [0, 0, z0]
    roll0 = math.radians(argf("--roll0", 0.0))          # roll/yaw disturbances to exercise the lateral
    yaw0 = math.radians(argf("--yaw0", 0.0))
    if abs(roll0) > 1e-3 or abs(yaw0) > 1e-3:
        cr, sr = math.cos(roll0), math.sin(roll0); cy, sy = math.cos(yaw0), math.sin(yaw0)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        x[3:12] = (Rz @ Rx).reshape(-1)
    u8 = np.array([M * G / 4] * 4 + [0, 0, 0, 0]); hist = [u8.copy()] * (lat_n + 1)
    t_sched = 0.0
    print("6-state NMPC vs TRUE plant: hover -> 90deg wing-borne. Vc=%.0f cap=%.0f" % (Vc, cap))
    print("  t   x     z    vs   vz   V   tiltCMD/act  AoA  theta  Tf   Tr  de | ok")
    crashed = False
    def mj(tau): tau = min(max(tau, 0), 1); return 6 * tau**5 - 15 * tau**4 + 10 * tau**3
    for it in range(int(dur / DT)):
        p = x[0:3]; R = x[3:12].reshape(3, 3); vI = x[13:16]; om = x[16:19]
        vb = R.T @ vI; V = float(np.linalg.norm(vI))
        vs = float(vI[0]); vz = float(vI[2]); th = -math.asin(max(-1, min(1, R[2, 0]))); q = float(om[1])
        X6 = np.array([p[0], p[2], vs, vz, th, q])
        if it % nsolve == 0:
            # ANCHOR the schedule speed on the ACHIEVED vs (anti-corridor: tilt tracks speed), and
            # advance a target speed over the horizon toward Vc.
            vs_now = max(vs, 0.0)
            refs = []
            for k in range(nmpc.N):
                # target speed advances from achieved vs toward Vc across the transition
                vtar = min(vs_now + (k * nmpc.DT) * (Vc / max(T_tr, 1e-3)), Vc)
                s = sched.at(vtar)
                refs.append(dict(z=z0, vs=vtar, vz=0.0, theta=s["theta"], beta=s["beta"],
                                 Tf=s["Tf"], Tr=s["Tr"], de=s["de"]))
            Xsolve = predict6(X6, hist[-lat_n:]) if LAT > 1e-3 else X6
            U4, ok = nmpc.solve(Xsolve, refs)
            Tf, Tr, beta, de = U4
            g = {"kR_roll": 8.0, "kW_roll": 2.0, "kR_yaw": 4.0, "kW_yaw": 1.5, "k_ail": 0.3, "k_rud": 0.2}
            mot, ail, rud = lateral_control(R, om, beta, V, Tf, Tr, 0.0, g)   # tilt-aware lateral
            u8 = np.array([mot[0] * MAX_THRUST, mot[1] * MAX_THRUST, mot[2] * MAX_THRUST,
                           mot[3] * MAX_THRUST, beta, de,
                           ail * so3.SO3TransitionNMPC.DA_MAX, rud * so3.SO3TransitionNMPC.DR_MAX])
        hist.append(u8.copy()); hist = hist[-200:]
        u_app = hist[-1 - lat_n]
        x = x + DT * np.array(F19(x, u_app)).ravel()
        Rn = x[3:12].reshape(3, 3); Uu, _, Vt = np.linalg.svd(Rn); x[3:12] = (Uu @ Vt).reshape(-1)
        if x[2] < 0.3 or x[2] > 60:
            print("  !! CRASH at t=%.1f z=%.1f" % (it * DT, x[2])); crashed = True; break
        if it % 25 == 0:
            aoa = math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-6)) if V > 1.0 else 0.0
            print("%4.1f %5.1f %5.1f %5.1f %+4.1f %4.1f  %4.0f/%4.0f %5.1f %+5.1f %4.1f %4.1f %+.2f | %d"
                  % (it * DT, x[0], x[2], vs, vz, V, math.degrees(u8[4]), math.degrees(x[12]),
                     aoa, math.degrees(th), u8[0] + u8[1], u8[2] + u8[3], u8[5], int(ok)))
    if not crashed:
        aoa = math.degrees(math.atan2(-(x[3:12].reshape(3, 3).T @ x[13:16])[2],
                                      abs((x[3:12].reshape(3, 3).T @ x[13:16])[0]) + 1e-6))
        print("DONE z=%.1f V=%.1f tilt=%.0f AoA=%.1f rotors=%.1fN (weight %.1f)"
              % (x[2], np.linalg.norm(x[13:16]), math.degrees(x[12]), aoa,
                 sum(u8[0:4]) * MAX_THRUST if u8[0] < 1.5 else sum(u8[0:4]), M * G))


YF = float(so3.RPOS[0, 1]); SPIN = so3.SPIN; CQ = so3.CQ


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
    from fsc_autopilot_ros2_msgs.msg import Mocap

    def clip(v, lo, hi):
        return lo if v < lo else hi if v > hi else v

    class NMPC6Node(Node):
        def __init__(self):
            super().__init__("nmpc6_cruise_control")
            P = self.declare_parameter
            P("px4_ns", "uav_0"); P("controller_id", "nmpc6_cruise"); P("active_at_start", False)
            P("auto_arm", True); P("arm_after", 150); P("rate_hz", 25.0)
            P("vcruise", 16.0); P("cap_deg", 90.0); P("alpha_max_deg", 8.0)
            P("ttr", 18.0); P("settle_s", 2.0); P("latency_s", 0.10)
            P("cruise_alt", 15.0)   # MIN altitude to climb to before transitioning (a low engage is
            #                         fatal -- the transition needs room; HIL bag myrun engaged at 1.5 m)
            P("kR_roll", 8.0); P("kW_roll", 2.0); P("kR_yaw", 4.0); P("kW_yaw", 1.5)
            P("k_ail", 0.3); P("k_rud", 0.2)
            ns = self.get_parameter("px4_ns").value
            pre = f"/{ns}" if ns else ""
            self.cid = self.get_parameter("controller_id").value
            self._active = bool(self.get_parameter("active_at_start").value)
            self.auto_arm = bool(self.get_parameter("auto_arm").value)
            self.arm_after = int(self.get_parameter("arm_after").value)
            self.rate = float(self.gp("rate_hz"))
            self.get_logger().info("building 6-state wing-borne NMPC...")
            self.nmpc = NMPC6()
            self.F6 = build_ode6()
            self.sched = Schedule(V_cruise=self.gp("vcruise"), tilt_cap_deg=self.gp("cap_deg"),
                                  alpha_max_deg=self.gp("alpha_max_deg"))
            self.get_logger().info("NMPC built.")
            self.state = None; self.p0 = None; self.heading = 0.0; self.z0 = 0.0
            self.armed = self.offb = False; self.counter = self.last = 0; self._eng = 0
            self.ubuf = []; self._solve_ms = 0.0
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            self.motp = self.create_publisher(ActuatorMotors, f"{pre}/fmu/in/actuator_motors", qos)
            self.srvp = self.create_publisher(ActuatorServos, f"{pre}/fmu/in/actuator_servos", qos)
            self.ocm = self.create_publisher(OffboardControlMode, f"{pre}/fmu/in/offboard_control_mode", qos)
            self.cmdp = self.create_publisher(VehicleCommand, f"{pre}/fmu/in/vehicle_command", qos)
            self.create_subscription(VehicleStatus, f"{pre}/fmu/out/vehicle_status_v1",
                                     self.on_status, qos)
            self.create_subscription(String, f"{pre}/ground_station/active_controller", self.on_active, 10)
            self.create_subscription(Mocap, f"{pre}/mocap", self.on_mocap, qos_profile_sensor_data)
            self.create_timer(1.0 / self.rate, self.loop)
            self.get_logger().info(f"nmpc6_cruise up: id={self.cid} rate={self.rate:.0f}Hz")

        def gp(self, n): return float(self.get_parameter(n).value)
        def now_us(self): return int(self.get_clock().now().nanoseconds / 1000)
        def on_status(self, m):
            self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            self.offb = (m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        def on_active(self, m): self._active = (m.data == self.cid)
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
            if self._active: self.ocm.publish(o)
            if self.state is None:
                if self._active: self.pub([math.nan] * 4, [math.nan] * 5)
                return
            p, R, v, w = self.state
            if not self._active:
                self.p0 = None; return
            if self.p0 is None:                          # snapshot the engage pose (heading, altitude)
                self.heading = math.atan2(R[1, 0], R[0, 0]); self.p0 = p.copy()
                self.z0 = max(float(p[2]), self.gp("cruise_alt"))   # climb to a safe altitude first
                self._eng = 0; self.ubuf = []
                self.get_logger().info("NMPC6 wing-borne ENGAGED at (%.1f,%.1f,%.1f) heading=%.0f -> Vc=%.0f"
                                       % (p[0], p[1], p[2], math.degrees(self.heading), self.gp("vcruise")))
            self._eng += 1
            dt = 1.0 / self.rate
            fwd = np.array([math.cos(self.heading), math.sin(self.heading), 0.0])
            left = np.array([-math.sin(self.heading), math.cos(self.heading), 0.0])
            # ---- longitudinal projection (mocap -> X6) ----
            s = float(np.dot(p - self.p0, fwd)); z = float(p[2])
            vs = float(np.dot(v, fwd)); vz = float(v[2])
            theta = math.asin(clip(R[2, 0], -1.0, 1.0)); q = -float(w[1])   # nose-up +
            X6 = np.array([s, z, vs, vz, theta, q])
            # ---- delay compensation ----
            lat_n = max(1, int(round(self.gp("latency_s") * self.rate)))
            if self.ubuf and self.gp("latency_s") > 1e-3:
                Xp = X6.copy()
                for uu in self.ubuf[-lat_n:]:
                    for _ in range(3):
                        Xp = Xp + (dt / 3) * np.array(self.F6(Xp, uu)).ravel()
                X6 = Xp
            # ---- reference: hover during settle, then ramp target speed to Vc (anchored on vs) ----
            Vc = self.gp("vcruise"); ttr = self.gp("ttr")
            # stay in HOVER (vs=0, z_ref=z0) until settled AND within 1 m of the target altitude ->
            # the vehicle CLIMBS to z0 before the transition starts, so a low engage can't crash it.
            settling = (self._eng < int(self.gp("settle_s") * self.rate)) or (z < self.z0 - 1.0)
            vs_now = 0.0 if settling else max(vs, 0.0)
            refs = []
            for k in range(self.nmpc.N):
                vtar = 0.0 if settling else min(vs_now + (k * self.nmpc.DT) * (Vc / max(ttr, 1e-3)), Vc)
                sc = self.sched.at(vtar)
                refs.append(dict(z=self.z0, vs=vtar, vz=0.0, theta=sc["theta"], beta=sc["beta"],
                                 Tf=sc["Tf"], Tr=sc["Tr"], de=sc["de"]))
            import time as _t
            _t0 = _t.perf_counter()
            U4, ok = self.nmpc.solve(X6, refs)
            self._solve_ms = (_t.perf_counter() - _t0) * 1e3
            Tf, Tr, beta, de = [float(a) for a in U4]
            self.ubuf.append(np.array([Tf, Tr, beta, de])); self.ubuf = self.ubuf[-lat_n:]
            # ---- TILT-AWARE lateral stabilizer (roll=0 + yaw=heading). The old tilt-naive version
            # tumbled 'as soon as it hit 90deg tilt': the front rotors are horizontal there, so
            # differential front thrust -> YAW not roll, and the rear (roll authority) is unloaded. ----
            V_now = float(np.linalg.norm(v))
            g = {"kR_roll": self.gp("kR_roll"), "kW_roll": self.gp("kW_roll"),
                 "kR_yaw": self.gp("kR_yaw"), "kW_yaw": self.gp("kW_yaw"),
                 "k_ail": self.gp("k_ail"), "k_rud": self.gp("k_rud")}
            mot, aileron, rudder = lateral_control(R, w, beta, V_now, Tf, Tr, self.heading, g)
            tservo = clip(beta / (math.pi / 2), 0.0, 1.0)
            elevator = clip(de / DE_MAX, -1.0, 1.0)
            self.pub(mot, [tservo, tservo, aileron, elevator, rudder])
            if self.counter % max(1, int(self.rate / 2)) == 0:
                vb = R.T @ v
                aoa = math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-6)) if V_now > 1.0 else 0.0
                roll_m = math.degrees(math.atan2(R[2, 1], R[2, 2]))
                ye = math.atan2(R[1, 0], R[0, 0]) - self.heading
                yaw_m = math.degrees(math.atan2(math.sin(ye), math.cos(ye)))
                # ★ MOCAP ATTITUDE CHECK: pitch (theta, what the NMPC tracks) vs AoA (computed the
                # SIM way, v_body=R^T v). In steady cruise these should AGREE (both ~+6deg). If they
                # DISAGREE or flip sign -> the mocap attitude convention differs from the model.
                self.get_logger().info(
                    "NMPC6 %s V=%.1f tilt=%.0f | pitch=%+.0f AoA=%+.0f roll=%+.0f yaw=%+.0f | "
                    "z=%.1f ok=%d %.0fms T=[%.0f,%.0f] de=%.2f mot=[%.2f,%.2f,%.2f,%.2f]"
                    % ("settle" if settling else "fly", V_now, math.degrees(beta),
                       math.degrees(theta), aoa, roll_m, yaw_m, z, int(ok), self._solve_ms,
                       Tf, Tr, de, mot[0], mot[1], mot[2], mot[3]))
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
        rclpy.spin(NMPC6Node())
    except KeyboardInterrupt:
        pass


def main(args=None):
    """console_scripts entry point.

    Delegates to the SAME dispatch as `python3 nmpc6_cruise_node.py`, so `ros2 run` and a
    direct invocation cannot drift apart -- including the offline gate flags,
    which are how this controller is validated before it is ever flown.
    """
    if "--validate" in sys.argv:
        validate_model()
    elif "--test" in sys.argv:
        offline_test()
    else:
        run_ros()



if __name__ == "__main__":
    main()

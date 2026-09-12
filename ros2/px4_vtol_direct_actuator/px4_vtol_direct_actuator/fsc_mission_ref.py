#!/usr/bin/env python3
"""FSC full-MISSION reference, built on the AVTOL SO(3) path-following trim machinery.

Mission phases (one continuous TIME-parameterised trajectory):
    1. CLIMB     - hover-climb from the ground to cruise altitude (tilt 0, V=0)
    2. TRANS-OUT - accelerate + tilt the front rotors forward, hover -> cruise
    3. CRUISE    - hold cruise (front rotors tilted, wing carries the weight)
    4. TRANS-BACK- AGGRESSIVE decelerate + tilt back, cruise -> hover
    5. HOVER/LAND- settle to a hover over the target point, descend, land

Why it is trackable (the property my earlier s-anchored reference lacked): the WHOLE mission is
run through the AVTOL coupled trim solver `_solve_trim_prefer_lift(V, beta_cmd, a_des, ...)`, so at
every instant the tilt / airspeed / alpha / total-thrust are mutually FORCE-consistent AND
consistent with the scheduled forward acceleration a_des = dV/dt. Tilt therefore cannot lead speed:
the solver keeps the rotors hover-ward until the wing can actually carry the weight. Velocity is
integrated to position and the cruise duration is sized so the vehicle decelerates to a hover
exactly over the target, then descends vertically -> "land at a specific position".

get(t) returns the node's (xref[19], uref[8]). The front/rear thrust split is PITCH-BALANCED for
the 4-rotor geometry (My_thrust + My_aero = 0); without it the reference is not a trim
(omega_dot ~ -7 rad/s^2) and the vehicle pitches away in the corridor.

Convention note:
  * AVTOL beta : 90 deg = hover (thrust vertical), 0 deg = cruise (thrust forward).
  * Isaac tilt : 0 = up (hover), 90 deg = forward (cruise).   tilt_isaac = 90 deg - beta_avtol.
"""
import math
import os
import sys

import numpy as np

from fsc_transition_ref import FSC, avtol_dir

# ---- FSC geometry / aero (identical to nmpc_so3_transition_node so the reference is a real trim) ----
M = FSC["m"]; G = FSC["g"]
CG_X = 0.177
XF = 0.459 - CG_X; ZF = 0.02; XR = -0.112 - CG_X          # rotor arms rel. CG [m]
RHO = FSC["rho"]; S_WING = FSC["S"]; C_WING = 0.25
CL0 = FSC["CL0"]; CLA = FSC["CL_alpha"]; CD0 = FSC["CD0"]; KIND = FSC["k_induced"]
CM0 = FSC["Cm0"]; CMA = FSC["Cm_alpha"]; CMDE = FSC["Cm_de"]
CLMAX = 1.2
DE_MAX = math.radians(25.0)


def _load_avtol_trim():
    d = avtol_dir()
    if d not in sys.path:
        sys.path.insert(0, d)
    from transition_reference_consistent import _solve_coupled_thrust_and_beta
    return _solve_coupled_thrust_and_beta


def _mj(tau):
    """min-jerk 0->1 for tau in [0,1] (smoothstep-5)."""
    tau = np.clip(tau, 0.0, 1.0)
    return 6.0 * tau**5 - 15.0 * tau**4 + 10.0 * tau**3


def pitch_balanced_split(Ttot, beta_isaac, V, alpha, de, Fz_req=None, gamma=0.0, theta=0.0):
    """Front/rear thrust PER ROTOR. Delegates to the one shared solver.

    `Ttot` is ignored except as a fallback: it comes from a generator that assumes the whole
    rotor set tilts, which is false here (see nmpc_so3_transition_node.trim_split). What the
    reference actually needs is the VERTICAL DEMAND -- M*(G + z_ddot) for a climbing or
    descending leg -- and the two thrusts solved from this airframe's geometry.
    """
    try:
        import nmpc_so3_transition_node as _so3
        Tf_pair, Tr_pair = _so3.trim_split(beta_isaac, theta, V, de, Fz_req=Fz_req, gamma=gamma)
        return 0.5 * Tf_pair, 0.5 * Tr_pair
    except Exception:
        return 0.25 * Ttot, 0.25 * Ttot


class FSCMissionProfile:
    """Builds + interpolates the full-mission reference. get(t) -> (xref[19], uref[8])."""

    def __init__(self,
                 target=(120.0, 0.0),        # land here (world x,y) [m]
                 z_ground=0.5, z_cruise=15.0, V_cruise=15.0, tilt_cap_deg=90.0,
                 T_climb=6.0, T_hold_top=2.0, T_out=10.0,
                 T_back=5.0, T_settle=3.0, T_land=6.0,
                 accel_cap=2.5, decel_cap=2.0, alpha_max_deg=8.0,
                 alpha_brake_max_deg=12.0, dt=0.02,   # AoA ceiling while braking
                 
                 wing_load_lo=9.5, wing_load_hi=14.5, x0=0.0, y0=0.0):
        self.dt = float(dt)
        # wing-loading gate: ramp the commanded AoA (wing lift share) in over [lo,hi] m/s so the wing
        # is NOT loaded until the vehicle is safely above stall speed (~11.5). Commanding nose-up trim
        # below stall speed drives a stall-induced pitch-up divergence in the corridor.
        self.wing_load_lo = float(wing_load_lo); self.wing_load_hi = float(wing_load_hi)
        alpha_brake_max = math.radians(float(alpha_brake_max_deg))
        self.z_ground = float(z_ground); self.z_cruise = float(z_cruise)
        solve_cb = _load_avtol_trim()          # _solve_coupled_thrust_and_beta

        W = M * G
        W_eff = W + M * FSC["a_margin"]
        beta_hover = math.pi / 2.0                          # AVTOL hover
        beta_cruise = math.pi / 2.0 - math.radians(tilt_cap_deg)   # AVTOL cruise (from isaac cap)
        alpha_max = math.radians(alpha_max_deg)
        margin = FSC["cruise_thrust_margin"]

        # heading points from start straight at the target; mission is 1-D along it
        p0 = np.array([float(x0), float(y0), z_ground])
        dxy = np.array([target[0] - x0, target[1] - y0])
        D_along = float(np.hypot(dxy[0], dxy[1]))
        heading = math.atan2(dxy[1], dxy[0]) if D_along > 1e-6 else 0.0
        self.heading = heading
        self.fwd = np.array([math.cos(heading), math.sin(heading), 0.0])
        self.p0 = p0

        # size the CRUISE hold so the total forward distance == distance to target.
        # min-jerk 0<->Vc average speed is 0.5*Vc, so trans-out and trans-back each cover ~0.5*Vc*T.
        D_fixed = 0.5 * V_cruise * T_out + 0.5 * V_cruise * T_back
        T_cruise = max((D_along - D_fixed) / max(V_cruise, 1e-6), 0.0)
        self.T_cruise = T_cruise
        if D_fixed > D_along + 1e-6:
            # target too close for a full-speed cruise; the transitions alone overshoot it.
            print(f"[mission] WARNING target {D_along:.0f} m < transition distance {D_fixed:.0f} m; "
                  f"cruise=0, will overshoot. Increase target distance or lower V_cruise/T_out/T_back.")

        # ---------------- phase boundaries (time) ----------------
        bounds = [("climb", T_climb), ("hold_top", T_hold_top), ("trans_out", T_out),
                  ("cruise", T_cruise), ("trans_back", T_back), ("settle", T_settle),
                  ("land", T_land)]
        edges = np.cumsum([0.0] + [b[1] for b in bounds])
        self.phase_edges = {bounds[i][0]: (edges[i], edges[i + 1]) for i in range(len(bounds))}
        self.tf = float(edges[-1])
        t = np.arange(0.0, self.tf + dt, dt)
        n = len(t)

        # per-phase MODE: 'hover' = direct multirotor (rotors up, body level, wing not flying);
        # 'wing' = AVTOL accel-aware coupled trim. The hover-mode phases MUST NOT go through the wing
        # trim solver -- at hover (beta=90 AVTOL) cos(beta)~0 blows the forward-thrust term up, and a
        # vertical climb rate fed in as "airspeed" produces a nonsense 90 deg flight-path pitch.
        MODE = {"climb": "hover", "hold_top": "hover", "trans_out": "wing", "cruise": "wing",
                "trans_back": "wing", "settle": "hover", "land": "hover"}

        def phase_at(tk):
            for name, (lo, hi) in self.phase_edges.items():
                if lo <= tk < hi:
                    return name, lo, hi
            lo, hi = self.phase_edges["land"]
            return "land", lo, hi

        # ---------------- nominal HORIZONTAL-speed / tilt / altitude schedules ----------------
        z = np.full(n, z_cruise); Vh = np.zeros(n); beta_nom = np.full(n, beta_hover)
        for i, tk in enumerate(t):
            ph, lo, hi = phase_at(tk); tau = (tk - lo) / max(hi - lo, 1e-9)
            if ph == "climb":
                z[i] = z_ground + (z_cruise - z_ground) * _mj(tau)
            elif ph == "land":
                z[i] = z_cruise + (z_ground - z_cruise) * _mj(tau)
            if ph == "trans_out":
                Vh[i] = V_cruise * _mj(tau)
                beta_nom[i] = beta_hover + (beta_cruise - beta_hover) * _mj(tau)
            elif ph == "cruise":
                Vh[i] = V_cruise; beta_nom[i] = beta_cruise
            elif ph == "trans_back":
                Vh[i] = V_cruise * (1.0 - _mj(tau))
                beta_nom[i] = beta_cruise + (beta_hover - beta_cruise) * _mj(tau)

        zd = np.gradient(z, dt); zdd = np.gradient(zd, dt)

        # ---- make the speed profile obey the SAME caps the trim is solved for ----
        # The min-jerk Vh above and the trim below disagreed: the trim clipped a_des to
        # [-decel_cap, accel_cap] while Vh itself was left free, so its own derivative
        # reached -4.5 m/s^2 against a 3.5 cap. The reference then demanded an acceleration
        # it had not solved a trim for, and the plant produced -0.02 m/s^2 against a
        # commanded -4.53 (2026-09-06 reference audit). Re-integrate Vh from the CLIPPED
        # acceleration so the profile and the trim describe the same manoeuvre. If a leg is
        # too short for its speed change the profile now visibly falls short of V_cruise,
        # which is the honest failure -- lengthen T_out / T_back.
        a_raw = np.gradient(Vh, dt)
        a_fwd = np.clip(a_raw, -decel_cap, accel_cap)
        Vh = np.maximum.accumulate(np.zeros(n))   # placeholder, rebuilt below
        Vh = np.zeros(n)
        for i in range(1, n):
            Vh[i] = max(0.0, Vh[i - 1] + a_fwd[i - 1] * dt)
        a_fwd = np.gradient(Vh, dt)               # forward (horizontal) acceleration

        # ---------------- per-step trim ----------------
        beta_out = np.full(n, beta_hover); alpha = np.zeros(n); theta = np.zeros(n)
        Ttot = np.zeros(n); de = np.zeros(n)
        for i, tk in enumerate(t):
            ph, lo, hi = phase_at(tk)
            if MODE[ph] == "hover":                # rotors up carry weight + vertical accel, level body
                beta_out[i] = beta_hover
                Ttot[i] = float(np.clip(M * (G + zdd[i]), 0.0, FSC["T_max_total"]))
                # alpha/theta/de = 0 (level hover)
            else:                                  # wing: force-EXACT coupled trim (tilt = atan2)
                a_des = float(np.clip(a_fwd[i], -decel_cap, accel_cap))
                V = float(Vh[i]); qS = 0.5 * RHO * V * V * S_WING
                # alpha carries as much weight as the wing can (clipped at alpha_max), but GATED so
                # the wing loads only above stall speed. Then the EXACT tilt/thrust come from
                # beta = atan2(Fz_req, Fx_req): tilt tracks speed, and the body stays LEVEL (alpha 0)
                # through the stall crossing -> no stall-induced pitch-up. As V builds the wing loads,
                # the rotors unload and the tilt increases toward the cruise pusher.
                wing_gate = float(np.clip((V - self.wing_load_lo) /
                                          max(self.wing_load_hi - self.wing_load_lo, 1e-6), 0.0, 1.0))
                if qS > 1.0:
                    cl_need = W_eff / qS
                    al_full = float(np.clip((cl_need - CL0) / CLA, 0.0, alpha_max))
                else:
                    al_full = 0.0
                al = al_full * wing_gate                       # gated wing loading (AoA)
                CL = CL0 + CLA * al
                CD = CD0 + KIND * CL * CL
                L = qS * CL; D = qS * CD
                # solve_cb gives a starting tilt from the AVTOL force balance; the tilt that
                # actually meets THIS airframe's demands is solved below, together with the
                # two thrusts (see nmpc_so3_transition_node.trim_split).
                T, b = solve_cb(L, D, a_des, W_eff, M, FSC["T_max_total"],
                                beta_cruise, beta_hover, margin)
                gamma = math.atan2(zd[i], max(V, 1e-6))        # flight-path angle
                # ---- BRAKING PITCH ----------------------------------------------------
                # A tiltrotor cannot tilt backwards (beta >= 0), so with the body level there
                # is NO force available to slow it down: the profile commanded -3.5 m/s^2 of
                # deceleration and the plant produced -0.02. The reference has to ask for the
                # attitude that actually brakes -- nose UP, which leans the thrust backwards:
                #     F_x = -(Tf + Tr) * sin(theta)   at beta = 0   ->   theta = asin(-a/g)
                # Bounded so the total angle of attack stays inside the linear range: the wing
                # is already carrying `al`, and pitching past the stall break to brake would
                # trade the deceleration for a departure. That bound is what limits this
                # airframe's braking to ~1-2 m/s^2, and it is the reason a mission overshoots
                # a finite target unless the deceleration leg is long enough
                # (2026-09-06 reference audit).
                th_brake = 0.0
                if a_des < 0.0:
                    th_want = math.atan2(-a_des, G)
                    th_brake = float(np.clip(th_want, 0.0, max(0.0, alpha_brake_max - al)))
                al_tot = al + th_brake
                beta_out[i] = b; alpha[i] = al_tot; Ttot[i] = T
                theta[i] = al_tot + gamma                      # body pitch = AoA + flight path
                de[i] = (float(np.clip(-(CM0 + CMA * al_tot) / CMDE, -DE_MAX, DE_MAX))
                         if abs(CMDE) > 1e-9 else 0.0)

        vh = Vh
        s_along = np.concatenate([[0.0], np.cumsum(0.5 * (vh[1:] + vh[:-1]) * dt)])  # forward dist

        # store arrays for interpolation
        self.t = t
        self.tilt_cap = math.radians(float(tilt_cap_deg))
        self.tilt_isaac = np.pi / 2.0 - beta_out            # front tilt [rad], 0=up
        self.alpha = alpha; self.theta = theta; self.Ttot = Ttot; self.de = de
        self.z = z; self.zd = zd; self.zdd = zdd; self.vh = vh; self.s_along = s_along
        self.a_fwd = a_fwd
        self.D_total = float(s_along[-1])

    # ---------------- interpolation + node-format state ----------------
    def _iv(self, tq, arr):
        return float(np.interp(float(tq), self.t, arr))

    def phase_at_time(self, tq):
        for name, (lo, hi) in self.phase_edges.items():
            if lo <= tq < hi:
                return name
        return "land"

    def time_from_speed(self, V):
        """Mission time within TRANS-OUT at which the reference horizontal speed == V. Used to gate
        the mission clock so the reference tilt can never lead the vehicle's ACHIEVED speed (the
        tilt-leads-speed corridor failure)."""
        lo, hi = self.phase_edges["trans_out"]
        mask = (self.t >= lo) & (self.t <= hi)
        tt = self.t[mask]; vv = self.vh[mask]
        if len(tt) < 2:
            return hi
        return float(np.interp(float(np.clip(V, vv[0], vv[-1])), vv, tt))

    def get(self, tq):
        """Reference (xref[19], uref[8]) at mission time tq, in the node's convention."""
        tq = float(np.clip(tq, 0.0, self.tf))
        tilt = self._iv(tq, self.tilt_isaac)
        alpha = self._iv(tq, self.alpha)
        theta = self._iv(tq, self.theta)
        Ttot = self._iv(tq, self.Ttot)
        de = self._iv(tq, self.de)
        z = self._iv(tq, self.z); zd = self._iv(tq, self.zd)
        vh = self._iv(tq, self.vh); s = self._iv(tq, self.s_along)

        pos = self.p0 + self.fwd * s
        pos[2] = z
        vel = self.fwd * vh + np.array([0.0, 0.0, zd])
        # attitude: yaw(heading) then pitch(theta), with +theta = NOSE-UP.
        # This built Ry(+theta) -- nose-DOWN -- while theta is constructed above as
        # `alpha + gamma`, i.e. nose-up positive. The wing therefore flew at NEGATIVE angle of
        # attack through the whole cruise: the reference commanded -10.1 m/s^2 of vertical
        # acceleration, a free fall. It cancelled against the old inverted aero model, which is
        # why it looked fine (2026-09-06 reference audit).
        c, s_ = math.cos(theta), math.sin(theta)
        Rp = np.array([[c, 0, -s_], [0, 1, 0], [s_, 0, c]])
        cy, sy = math.cos(self.heading), math.sin(self.heading)
        Ry = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        R = Ry @ Rp
        V = math.hypot(vh, zd)
        # The vertical AND forward demands both come from the leg itself, so a climb, a
        # traverse and a braking descent are each trimmed for what they are actually doing.
        # Solving the forward demand needs the tilt as a third unknown -- with only the two
        # thrusts, vertical and pitch use up all the freedom and the forward acceleration is
        # whatever falls out (the reference asked +2.42 m/s^2 and got +1.13).
        zdd = self._iv(tq, self.zdd)
        afwd = self._iv(tq, self.a_fwd)
        gamma = math.atan2(zd, max(vh, 1e-6))
        import nmpc_so3_transition_node as _so3
        Tf_pair, Tr_pair, tilt = _so3.trim_split(
            tilt, theta, V, de, Fz_req=M * (G + zdd), gamma=gamma,
            Fx_req=M * afwd, beta_max=self.tilt_cap)
        Tf, Tr = 0.5 * Tf_pair, 0.5 * Tr_pair
        xref = np.concatenate([pos, R.reshape(-1), [tilt], vel, [0, 0, 0]])
        uref = np.array([Tf, Tf, Tr, Tr, tilt, de, 0.0, 0.0])
        return xref, uref


# ============================ selftest / dump ============================
if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--tx", type=float, default=120.0)
    ap.add_argument("--ty", type=float, default=0.0)
    ap.add_argument("--zc", type=float, default=15.0)
    ap.add_argument("--vc", type=float, default=15.0)
    ap.add_argument("--cap", type=float, default=90.0)
    ap.add_argument("--check", action="store_true", help="verify each ref is a trim in build_ode")
    a = ap.parse_args()

    m = FSCMissionProfile(target=(a.tx, a.ty), z_cruise=a.zc, V_cruise=a.vc, tilt_cap_deg=a.cap)
    print(f"Mission: land at ({a.tx:.0f},{a.ty:.0f}), z_cruise={a.zc}, V_cruise={a.vc}, tilt_cap={a.cap}")
    print(f"  tf={m.tf:.1f}s  cruise_hold={m.T_cruise:.1f}s  forward_dist={m.D_total:.1f}m")
    print(f"  phases: " + "  ".join(f"{k}[{v[0]:.0f}-{v[1]:.0f}s]" for k, v in m.phase_edges.items()))
    print("\n   t   phase       x     z    Vh   tilt  alpha theta  Ttot  de   split(F,R)")
    labels = list(m.phase_edges.items())
    for tq in np.arange(0, m.tf + 0.1, 1.0):
        xr, ur = m.get(tq)
        ph = next((k for k, (lo, hi) in labels if lo <= tq < hi), "land")
        tl = math.degrees(xr[12]); al = math.degrees(m._iv(tq, m.alpha))
        th = math.degrees(m._iv(tq, m.theta)); Tt = m._iv(tq, m.Ttot)
        deg = math.degrees(m._iv(tq, m.de))
        print(f"{tq:4.0f}  {ph:10s} {xr[0]:6.1f} {xr[2]:5.1f} "
              f"{math.hypot(xr[13], xr[14]):4.1f}  {tl:4.0f}  {al:4.1f}  {th:5.1f}  {Tt:5.1f} {deg:5.1f}  {ur[0]:4.1f},{ur[2]:4.1f}")

    if a.check:
        print("\nVerifying each reference is a TRIM in the FSC dynamics (build_ode); "
              "omega_dot(pitch) should be ~0:")
        import nmpc_so3_transition_node as node
        F = node.build_ode()
        print("   t   vdot(f,l,v)          omdot(r,p,y)")
        for tq in np.arange(0, m.tf + 0.1, 2.0):
            xr, ur = m.get(tq)
            xd = np.array(F(xr, ur)).ravel()
            vd = xd[13:16]; od = xd[16:19]
            print(f"{tq:4.0f}  ({vd[0]:+5.2f},{vd[1]:+4.2f},{vd[2]:+5.2f})   "
                  f"({od[0]:+4.1f},{od[1]:+5.2f},{od[2]:+4.1f})")

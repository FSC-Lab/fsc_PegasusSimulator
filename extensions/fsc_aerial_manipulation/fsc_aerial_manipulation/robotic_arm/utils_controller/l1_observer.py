#!/usr/bin/env python3
"""
L1 adaptive augmented disturbance observer for the whole-body aerial
manipulator — Python REFERENCE implementation.

This is the numerical reference for the C++ port that flies
(fsc_autopilot_ros2's single_aerial_manipulator_whole_body_direct_actuation/
wb_l1_observer.cpp).  Same split as the law itself: the Python is the source of
truth, the C++ is parity-locked to it.

WHAT IT IMPLEMENTS
------------------
"Decompose the lumped disturbances into end-effector and orthogonal
components" (working note, 2026-08-27), Secs. 3-4.  It REPLACES the GMO update
law `d_hat = K_o (p - p_hat)` with

  predictor    p_hat_dot = h + u + d_hat_Sigma + A_s p_tilde, p_tilde = p_hat - p
  adaptation   d_hat_Sigma,i = -Phi_d^-1 e^{A_s Ts} p_tilde(i Ts)
               d_hat^c       = e^{-A_s Ts} d_hat_Sigma = -Phi_d^-1 p_tilde
  filter       d_hat_f = C(s) d_hat^c,  C(s) = (sI + Omega)^-1 Omega

Phi_d IS THE NOTE'S Phi, DISCRETIZED FOR THE PREDICTOR THAT IS ACTUALLY RUN.
The note derives Phi = A_s^-1(e^{A_s Ts} - I) from the CONTINUOUS error
equation, and the deadbeat property "d_hat^c has unit gain on a constant
residual for every Ts and every A_s" holds only if the predictor is integrated
with the same kernel.  The predictor here advances the (h + u + d_hat) term
with plain Euler at the control step dt -- it must, because the PLANT
integrates h + u exactly and any other weight there leaves a
(weight - dt)(h + u) bias, ~0.15 N on the z channel at hover.  Over the N =
Ts/dt sub-steps of one adaptation interval that gives

    p_tilde_N = e^{A_s Ts} p_tilde_0 + Phi_d (d_hat - dbar_Sigma),
    Phi_d = dt (I - e^{A_s Ts})(I - e^{A_s dt})^-1     (diagonal)

so Phi_d, not Phi, is the exact deadbeat gain.  Phi_d = dt exactly when the
adaptation runs every tick (N = 1) and Phi_d -> Phi as dt -> 0 at fixed Ts, so
this is the note's law with its own O(Ts^2) statement made exact rather than
approximated.  Using Phi at N = 1 instead measures a REAL DC error of
a*Ts/(1 - e^{-a*Ts}) - 1 ~ a*Ts/2: 0.8% at a = 2 and 7% at a = 20, both seen
in the self-test before this was fixed.

CONSEQUENCE WORTH KNOWING: at N = 1 the deadbeat cancellation is exact and A_s
drops out of the estimate entirely -- it only shapes the weighting of dbar
WITHIN an interval, which is the note's own Remark ("the bound is independent
of A_s, which only shapes the weights").  A_s becomes a live knob only when
the adaptation period is held longer than the tick (N > 1), which is also the
only way to buy down the 1/Ts noise gain of the momentum difference.  So
wb_l1_adapt_period_s and wb_l1_as_* are ONE trade, not two.

and adds the ATTRIBUTION CHAIN the impedance law needs and a quadrotor does
not.  The momentum residual only ever measures the SUM d + d_e, so the
manuscript's task-space force estimate

  F_hat_y^lumped = (J_y^#)^T (d_hat + d_hat_e)  ->  F_y + (J_y^#)^T d

carries the internal disturbance as a PHANTOM CONTACT FORCE: the controller
renders compliance against a force the environment never applied.  The repair
is the four wrench-free directions of the residual.  With

  J_e   the 6x10 end-effector Jacobian, V_e = J_e V
  Z_0   a basis of N(J_e) (10x4) — the self-motions that leave the EE at rest
  B_a   the 8 actuated rows (collective, 3 body moments, 4 joints)

any contact wrench is invisible along Z_0 (Z_0^T J_e^T = 0), so
Z_0^T w_Sigma = Z_0^T w EXACTLY, whatever the environment does — no contact
flag is needed.  Steps 1-4 of the note then run

  Step 1  w_hat_Sigma = T^T d_hat^c                        (original coords)
  Step 2  w_hat_dot   = omega_i L_c Z_0^T (w_hat_Sigma - w_hat),
                        L_c = B_a G^+,  G = Z_0^T B_a       (internal estimate)
  Step 3  w_hat_e     = C_x(s) Lambda_e J_e M^-1 (w_hat_Sigma - w_hat)
  Step 4  F_hat_y     = Lambda_y S_e Lambda_e^-1 w_hat_e

with Lambda_e = (J_e M^-1 J_e^T)^-1 and S_e = blkdiag(R_e, e_3^T).  Steps 3-4
are written in the ORIGINAL coordinates on purpose: the note's
(Jbar_e^#)^T (d_hat^c - d_hat) equals Lambda_e J_e M^-1 (w_hat_Sigma - w_hat)
identically (verified to 2e-15 over random states), which avoids ever forming
T^-1.

WHAT IT DOES NOT DO
-------------------
* It does not split the RANGE channel.  The note's alias-family remark proves
  that is impossible from the residual alone: (d + Jbar_e^T delta,
  w_e - delta) produces the identical residual for every delta.  Four
  directions are free; the other six need a rate bound, which is what the
  omega_i observer buys.
* With a STATIONARY arm only 4 of the 8 matched directions are observable
  (rank G = 4).  A moving arm rotates P_G and, if persistently exciting,
  recovers all eight -- in principle.  MEASURED ON THIS PLANT it does not
  help: a vigorous 60 s arm sweep (q1 +-30 deg, fold +-25 deg, wrist +-60 deg)
  gives an averaged P_G with eigenvalues [4e-4, 0.017, 0.051, 0.097, 0.89,
  0.95, 1.00, 1.00], so the four weak directions converge 10--2500x slower
  than omega_i; 400 s of sweeping moved the collective estimate from -0.04 to
  -0.30 N against a true -5.5 N.  The note's PE condition is satisfied and
  practically useless here.  The fix that DOES work is the metric, next.

THE ONE PLACE THIS DEVIATES FROM THE NOTE, AND WHY
--------------------------------------------------
The note's L_c = B_a G^+ is the MINIMUM-NORM solution of G theta = eps_0, and
minimum-norm compares newtons of collective thrust against newton-metres of
joint torque.  On this plant the collective's coupling into the wrench-free
rows is a_3 = (hat(r_0e)J_we + A_e)^T e_3 ~ [0, -0.002, -0.103, -0.001] --
metres of end-effector lift per joint radian -- against the joint rows' exact
I_4.  A 5.5 N thrust deficit and a 10x smaller joint torque therefore explain
the SAME innovation, and minimum-norm picks the joint torque: measured
theta_hat_f = -0.04 for a true -5.5, and the resulting w_hat, subtracted in
Step 3, made the phantom EE force WORSE than not decomposing at all
(|F_hat_y| 0.89 -> 4.00 N).

L_c is therefore generalized to L_c = B_a W G^T (G W G^T)^-1 with a diagonal
prior-variance W = diag(var_f, var_m*1_3, var_q*1_4).  Every property the
note's proof uses survives for ANY W > 0: Z_0^T L_c = G W G^T (G W G^T)^-1 =
I_4, R(L_c) is still inside R(B_a) so B_a^perp^T L_c = 0 and the lateral rows
stay frozen, and the error dynamics stay stable with the Lyapunov function
||theta_tilde||^2 taken in the W^-1 metric instead of the Euclidean one.  The
note itself calls this open ("Four numbers do not determine a ten-vector, so
where the correction is placed is a design choice"); W = 1,1,1 recovers its
own choice exactly and is the code default.

Measured with var = (100, 0.25, 0.0025) -- "a 10 N thrust error is as
plausible as a 0.5 N.m body moment or a 0.05 N.m joint torque" -- on the same
disturbance and a STATIC arm: theta_hat_f = -4.90 of -5.50 (89% identified)
and |F_hat_y| 0.89 -> 0.44 N.  With a real 2.6 N contact wrench also present
the recovered wrench goes from [1.50, -6.18, 1.05, 1.31, -0.01, 0.10]
(unweighted, useless) to [1.50, -1.39, 1.90, 0.18, 0.00, 0.01] against a true
[1.50, -0.80, 2.00, 0.05, -0.03, 0.02].  omega_i does not change the fixed
point, only how fast it is reached.
* The arm channel of d_hat_f is NOT fed to u3.  The note's (u3_aug) marks only
  the F_hat_y term, so this stays faithful; the estimate is still reported so
  the choice can be evaluated.

Pure numpy — no Isaac, no ROS.  Run the file for its self-test.
"""

import numpy as np

__all__ = ["L1Gains", "L1DisturbanceObserver", "end_effector_jacobian",
           "null_motion_basis", "actuation_basis"]


def _hat(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def actuation_basis(n=4):
    """B_a (10x8): the rows the actuators reach — collective along body z, the
    three body moments, the n joint torques.  Constant, because V and tau are
    expressed in the platform frame."""
    B = np.zeros((6 + n, 4 + n))
    B[2, 0] = 1.0                       # collective thrust -> body-z force row
    B[3:6, 1:4] = np.eye(3)             # body moments
    B[6:, 4:] = np.eye(n)               # joint torques
    return B


def end_effector_jacobian(dyn):
    """J_e (6x10), mapping V = [v_0; omega_0; qdot] (platform frame) to
    V_e = [v_e; omega_e] (END-EFFECTOR frame) — the note's (Je_factored)."""
    R0e = dyn["R_e_0"].T                # base -> EE
    inner = np.block([[np.eye(3), -_hat(dyn["r_0e_0"]), dyn["A_e"]],
                      [np.zeros((3, 3)), np.eye(3), dyn["J_q_omega_e"]]])
    Rblk = np.zeros((6, 6))
    Rblk[0:3, 0:3] = R0e
    Rblk[3:6, 3:6] = R0e
    return Rblk @ inner


def null_motion_basis(dyn):
    """Z_0 (10x4), a basis of N(J_e): the platform counter-motions that leave
    the end-effector at rest.  The note's (Z0)."""
    Jwe = dyn["J_q_omega_e"]
    n = Jwe.shape[1]
    return np.vstack([-(_hat(dyn["r_0e_0"]) @ Jwe + dyn["A_e"]),
                      -Jwe,
                      np.eye(n)])




class L1Gains:
    """Every knob of the estimator.  Bandwidths in rad/s, A_s rates in 1/s.

    a_* and omega_c_* are given per CHANNEL GROUP (3 translational, 3
    rotational, n arm) because that is how the note blocks Omega and how the
    yaml exposes them.

    adapt_period_s is the note's T_s: how long d_hat is HELD.  0 (the default)
    means "every control tick", N = 1, where the deadbeat is exact and A_s
    drops out.  Holding longer trades bandwidth for the 1/T_s noise gain of
    the momentum difference and makes A_s live -- see the module docstring.

    lc_var_* are the prior variances of the L_c inversion,
    L_c = B_a W G^T (G W G^T)^-1 with W = diag(var_f, var_m*1_3, var_q*1_4).
    All 1.0 is the note's own minimum-norm gain (its eq. Lc); ANY W > 0 keeps
    Z_0^T L_c = I_4 and B_a^perp^T L_c = 0, so the note's analysis is unchanged
    (its Lyapunov function becomes ||theta_tilde||^2 in the W^-1 metric).
    See the module docstring for why the default is a bad choice on this
    plant.

    decompose=False falls back to the GMO's own task-force formula
    F_hat_y = (J_y^#)^T d_hat_f evaluated on the L1 estimate, so the two ideas
    in the note -- a better ESTIMATE and a correct ATTRIBUTION -- can be
    measured apart.
    """

    def __init__(self, a_t=2.0, a_r=2.0, a_q=2.0,
                 omega_c_t=6.0, omega_c_r=6.0, omega_c_q=6.0,
                 omega_i=1.0, omega_x=20.0, adapt_period_s=0.0, decompose=True,
                 lc_var_f=1.0, lc_var_m=1.0, lc_var_q=1.0,
                 max_force_n=25.0, max_torque_nm=3.0, max_joint_nm=3.0,
                 max_wrench_force_n=25.0, max_wrench_torque_nm=5.0, n=4):
        self.n = int(n)
        self.a = np.concatenate([np.full(3, float(a_t)),
                                 np.full(3, float(a_r)),
                                 np.full(self.n, float(a_q))])
        self.omega_c = np.concatenate([np.full(3, float(omega_c_t)),
                                       np.full(3, float(omega_c_r)),
                                       np.full(self.n, float(omega_c_q))])
        self.omega_i = float(omega_i)
        self.omega_x = float(omega_x)
        self.adapt_period_s = float(adapt_period_s)
        self.decompose = bool(decompose)
        self.lc_var = np.concatenate([[float(lc_var_f)],
                                      np.full(3, float(lc_var_m)),
                                      np.full(self.n, float(lc_var_q))])
        if np.any(self.lc_var <= 0.0):
            raise ValueError("L_c prior variances must be positive")
        self.max_force_n = float(max_force_n)
        self.max_torque_nm = float(max_torque_nm)
        self.max_joint_nm = float(max_joint_nm)
        self.max_wrench_force_n = float(max_wrench_force_n)
        self.max_wrench_torque_nm = float(max_wrench_torque_nm)
        if np.any(self.a <= 0.0):
            raise ValueError("A_s rates must be positive")
        if np.any(self.omega_c <= 0.0):
            raise ValueError("filter bandwidths must be positive")
        if self.omega_i < 0.0 or self.omega_x <= 0.0:
            raise ValueError("omega_i must be >= 0 and omega_x > 0")

    def phi_d(self, dt, Ts):
        """The exact deadbeat gain of the implemented discretization,
        Phi_d = dt (1 - e^{-a Ts}) / (1 - e^{-a dt}), elementwise.  Equals dt
        when Ts == dt and the note's (1 - e^{-a Ts})/a as dt -> 0."""
        a = self.a
        return dt * (1.0 - np.exp(-a * Ts)) / (1.0 - np.exp(-a * dt))


class L1DisturbanceObserver:
    """Stateful estimator.  The call order per control tick mirrors the GMO's:

        est = obs.update(dyn, R_0, xi, dt)   # BEFORE the law forms u
        ... law computes u = [u1*R0*e3; u2; u3] ...
        obs.propagate(dyn, xi, u, dt)        # AFTER, with THIS tick's u

    `update` returns a dict with d_t, d_r, d_rho (the filtered lumped estimate
    the platform loops consume) and F_y (the task force u3 consumes), plus the
    internals w_hat / w_e / d_sigma_c for diagnostics.
    """

    def __init__(self, gains: L1Gains):
        self.g = gains
        self.n = gains.n
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self):
        ng = 6 + self.n
        self._p_hat = None            # predictor momentum; None until primed
        self._d_f = np.zeros(ng)      # LPF state, C(s) d_hat^c
        self._w_hat = np.zeros(ng)    # internal-disturbance estimate (Step 2)
        self._w_e = np.zeros(6)       # wrench estimate (Step 3)
        self._d_c = np.zeros(ng)      # last unit-DC-gain PWC estimate
        self._d_hold = np.zeros(ng)   # d_hat held over the adaptation interval
        self._t_since_adapt = 0.0     # time since the last adaptation
        self._p_at_adapt = None       # p_tilde captured at the interval start

    @property
    def initialized(self):
        return self._p_hat is not None

    def zero_result(self):
        ng = 6 + self.n
        return {"d_t": np.zeros(3), "d_r": np.zeros(3),
                "d_rho": np.zeros(self.n), "d_f": np.zeros(ng),
                "d_sigma_c": self._d_c.copy(), "F_y": np.zeros(4),
                "w_hat": self._w_hat.copy(), "w_e": np.zeros(6),
                "p_tilde": np.zeros(ng), "adapted": False}

    # ------------------------------------------------------- Layer 1 + Layer 2
    def update(self, dyn, R_0, xi, dt):
        """Adapt (when the interval has elapsed), filter, then attribute."""
        g = self.g
        if dt <= 0.0:
            return self.zero_result()
        Ts = g.adapt_period_s if g.adapt_period_s > 0.0 else dt
        if Ts < dt:
            Ts = dt                       # never adapt faster than the tick

        p = dyn["M_tilde"] @ xi
        if self._p_hat is None:
            self._p_hat = p.copy()
        p_tilde = self._p_hat - p

        # ---- Layer 1: piecewise-constant adaptation -------------------------
        # d_hat^c = -Phi_d^-1 p_tilde, held for Ts.  Phi_d is diagonal, so this
        # is elementwise; see the module docstring for why it is Phi_d and not
        # the note's continuous-time Phi.
        self._t_since_adapt += dt
        adapted = self._t_since_adapt >= Ts - 1e-12
        if adapted:
            self._d_c = -p_tilde / g.phi_d(dt, self._t_since_adapt)
            # The PREDICTOR takes d_hat_Sigma = e^{A_s Ts} d_hat^c (the note
            # defines d_hat^c as e^{-A_s Ts} d_hat_Sigma), NOT d_hat^c itself.
            # A_s = -diag(a), so that factor is e^{-a Ts} < 1; using e^{+a Ts}
            # here doubled the DC error of the deadbeat, which the self-test
            # caught as 1.6% at a = 2 instead of 0.
            self._d_hold = np.exp(-g.a * self._t_since_adapt) * self._d_c
            self._t_since_adapt = 0.0

        # ---- Layer 2: the low-pass filter that sets robustness --------------
        alpha = np.exp(-g.omega_c * dt)
        self._d_f = alpha * self._d_f + (1.0 - alpha) * self._d_c

        out = self.attribute(dyn, R_0, dt)
        out["p_tilde"] = p_tilde
        out["adapted"] = adapted
        return out

    # ---------------------------------------------------------- Steps 1 .. 4
    def attribute(self, dyn, R_0, dt, d_c=None):
        """Steps 1-4 of the note, on the current d_hat^c.

        `d_c` overrides the internal estimate -- the self-test uses it to feed
        the chain an exact residual and measure the attribution alone.
        """
        g = self.g
        if d_c is not None:
            self._d_c = np.asarray(d_c, dtype=float)

        d_f = self._d_f.copy()
        d_f[0:3] = np.clip(d_f[0:3], -g.max_force_n, g.max_force_n)
        d_f[3:6] = np.clip(d_f[3:6], -g.max_torque_nm, g.max_torque_nm)
        d_f[6:] = np.clip(d_f[6:], -g.max_joint_nm, g.max_joint_nm)

        # ---- geometry the attribution chain needs ---------------------------
        Je = end_effector_jacobian(dyn)
        Z0 = null_motion_basis(dyn)
        M = dyn["M"]
        Lam_e_inv = Je @ np.linalg.solve(M, Je.T)     # Lambda_e^-1 (6x6, SPD)

        # ---- Step 1: the residual in the ORIGINAL coordinates ---------------
        w_sigma = dyn["T"].T @ self._d_c

        # ---- Step 2: the internal disturbance, from the wrench-free rows ----
        # eps_0 = Z_0^T (w_hat_Sigma - w_hat) cancels the wrench EXACTLY, so no
        # contact detection is needed.  The correction is booked to the
        # actuation channel (L_c = B_a G^+: the minimum actuator effort that
        # explains the innovation), which is what keeps the two lateral rows of
        # w_hat frozen -- the note's "why this gain".
        Ba = actuation_basis(self.n)
        G = Z0.T @ Ba                                 # 4x8, contains I_4
        WGt = g.lc_var[:, None] * G.T                 # W G^T
        eps0 = Z0.T @ (w_sigma - self._w_hat)
        self._w_hat = self._w_hat + (g.omega_i * dt) * (
            Ba @ (WGt @ np.linalg.solve(G @ WGt, eps0)))

        # ---- Step 3: the interaction wrench ---------------------------------
        w_e_raw = np.linalg.solve(
            Lam_e_inv, Je @ np.linalg.solve(M, w_sigma - self._w_hat))
        ax = np.exp(-g.omega_x * dt)
        self._w_e = ax * self._w_e + (1.0 - ax) * w_e_raw
        w_e = self._w_e.copy()
        w_e[0:3] = np.clip(w_e[0:3], -g.max_wrench_force_n,
                           g.max_wrench_force_n)
        w_e[3:6] = np.clip(w_e[3:6], -g.max_wrench_torque_nm,
                           g.max_wrench_torque_nm)

        # ---- Step 4: the task-space force -----------------------------------
        if g.decompose:
            Se = np.zeros((4, 6))
            Se[0:3, 0:3] = R_0 @ dyn["R_e_0"]         # R_e
            Se[3, 5] = 1.0
            F_y = dyn["Lambda_y"] @ (Se @ (Lam_e_inv @ w_e))
        else:
            F_y = (dyn["J_1y"] @ d_f[0:3] + dyn["J_2y"] @ d_f[3:6]
                   + dyn["J_3y"] @ d_f[6:])

        return {"d_t": d_f[0:3], "d_r": d_f[3:6], "d_rho": d_f[6:],
                "d_f": d_f, "d_sigma_c": self._d_c.copy(), "F_y": F_y,
                "w_hat": self._w_hat.copy(), "w_e": w_e,
                "w_e_raw": w_e_raw}

    # --------------------------------------------------------------- propagate
    def propagate(self, dyn, xi, u, dt):
        """Advance the predictor with THIS tick's commanded wrench u.

        The A_s term uses the exact matrix exponential rather than the note's
        forward-Euler `dt*A_s`: it is free (diagonal) and unconditionally
        stable -- the same fix lagged_thrust_curve.py records for the rotor lag
        and the geometric-L1 fork records for its own predictor.  The
        (h + u + d_hat) term keeps the Euler form ON PURPOSE: the plant
        integrates h + u exactly, and any other weight there would leave a
        (weight - dt)(h + u) bias, ~0.15 N on the z channel at hover.
        """
        g = self.g
        if dt <= 0.0 or self._p_hat is None:
            return
        p_tilde = self._p_hat - dyn["M_tilde"] @ xi
        h = dyn["C_tilde"].T @ xi - dyn["g_tilde"]
        decay = np.exp(-g.a * dt) - 1.0               # e^{A_s dt} - I
        self._p_hat = (self._p_hat + dt * (h + u + self._d_hold)
                       + decay * p_tilde)


# ===========================================================================
# Self-test -- run this file.  No Isaac, no ROS; the model comes from the
# extension's own transition_planner.make_params_t650().
# ===========================================================================
def _selftest():
    import os
    import sys
    _HERE = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.abspath(os.path.join(_HERE, "..", "..", "..")))
    from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C
    from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP

    np.set_printoptions(precision=6, suppress=True, linewidth=140)
    rng = np.random.default_rng(11)
    p = TP.make_params_t650()
    ok = True

    def check(name, val, tol):
        nonlocal ok
        good = bool(val <= tol)
        ok = ok and good
        print(f"  {'PASS' if good else 'FAIL'}  {name:<54s} {val:.3e} (tol {tol:.0e})")

    # ---- 1. the geometric identities the note relies on ---------------------
    print("1. geometry (6 random configurations)")
    e_JeZ0 = e_Jy = e_lam = e_rank = 0.0
    for _ in range(6):
        ax = rng.normal(size=3); ax /= np.linalg.norm(ax)
        ang = rng.uniform(-0.6, 0.6)
        K = C.hat(ax)
        R0 = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
        q = rng.uniform(-0.5, 0.8, size=4)
        X = np.concatenate([rng.normal(size=3), R0.reshape(9, order="F"), q,
                            rng.normal(scale=.3, size=3),
                            rng.normal(scale=.3, size=3),
                            rng.normal(scale=.3, size=4)])
        dyn = C.dynamics(X, p)
        Je, Z0 = end_effector_jacobian(dyn), null_motion_basis(dyn)
        Tm, Mt = dyn["T"], dyn["M_tilde"]
        Jbe = Je @ np.linalg.inv(Tm)
        Se = np.zeros((4, 6)); Se[0:3, 0:3] = R0 @ dyn["R_e_0"]; Se[3, 5] = 1.0
        e_JeZ0 = max(e_JeZ0, np.abs(Je @ Z0).max())
        e_Jy = max(e_Jy, np.abs(dyn["J_y"] - Se @ Jbe).max())
        La = np.linalg.inv(Jbe @ np.linalg.solve(Mt, Jbe.T))
        Lb = np.linalg.inv(Je @ np.linalg.solve(dyn["M"], Je.T))
        e_lam = max(e_lam, np.abs(La - Lb).max() / np.abs(La).max())
        e_rank += (np.linalg.matrix_rank(Je) != 6)
    check("J_e Z_0 = 0 (wrench-free directions)", e_JeZ0, 1e-12)
    check("J_y = S_e Jbar_e (task = view of the EE twist)", e_Jy, 1e-12)
    check("Lambda_e agrees in both coordinate sets", e_lam, 1e-12)
    check("rank J_e = 6 everywhere (Lemma 1)", float(e_rank), 0.5)

    # ---- 2. Layer 1 on a frozen plant ---------------------------------------
    # Drive a known constant residual through the momentum equation with the
    # configuration held.  This is the estimator alone, no control loop.
    print("2. Layer 1 (PWC adaptation) against a frozen plant")
    q0 = np.array([0.0, np.deg2rad(40), np.deg2rad(40), 0.0])
    X0 = np.concatenate([np.zeros(3), np.eye(3).reshape(9, order="F"), q0,
                         np.zeros(3), np.zeros(3), np.zeros(4)])
    dyn0 = C.dynamics(X0, p)
    dt = 1.0 / 250.0
    d_true = np.array([0.4, -0.3, -5.5, 0.2, -0.15, 0.05,
                       0.03, -0.02, 0.04, -0.01])
    for a_val, Ts, tag in ((2.0, 0.0, "A_s=2,  N=1"),
                           (20.0, 0.0, "A_s=20, N=1"),
                           (20.0, 0.020, "A_s=20, N=5")):
        g = L1Gains(a_t=a_val, a_r=a_val, a_q=a_val, adapt_period_s=Ts)
        obs = L1DisturbanceObserver(g)
        xi = np.zeros(10)
        u = dyn0["g_tilde"] - dyn0["C_tilde"].T @ xi
        pm = dyn0["M_tilde"] @ xi
        for _ in range(4000):
            est = obs.update(dyn0, np.eye(3), xi, dt)
            obs.propagate(dyn0, xi, u, dt)
            pm = pm + dt * (dyn0["C_tilde"].T @ xi - dyn0["g_tilde"]
                            + u + d_true)
            xi = np.linalg.solve(dyn0["M_tilde"], pm)
        check(f"   |d_hat^c - d_Sigma|/|d| ({tag})",
              np.abs(est["d_sigma_c"] - d_true).max() / np.abs(d_true).max(),
              5e-3)

    # ---- 3. attribution: a pure INTERNAL disturbance must read w_e ~ 0 ------
    # The note's whole point.  Feed the chain the EXACT residual of an
    # internal-only disturbance and read the reported EE wrench.  This also
    # measures the L_c metric: minimum-norm (the note's own gain) mis-books a
    # thrust deficit as joint torque and makes the phantom WORSE.
    print("3. attribution (Steps 2-4) -- the phantom-force test")
    Je = end_effector_jacobian(dyn0)
    Ba = actuation_basis(4)
    Tm = dyn0["T"]
    # A MATCHED internal disturbance: a 5.5 N thrust deficit (this rig's
    # dominant one -- kf mismatch and battery sag), body moments, and joint
    # friction.
    theta_true = np.array([-5.5, 0.20, -0.15, 0.05, 0.03, -0.02, 0.04, -0.01])
    w_true = Ba @ theta_true
    d_c_true = np.linalg.solve(Tm.T, w_true)
    lumped = (dyn0["J_1y"] @ d_c_true[0:3] + dyn0["J_2y"] @ d_c_true[3:6]
              + dyn0["J_3y"] @ d_c_true[6:])
    results = {}
    for tag, var in (("minimum-norm (the note)", (1.0, 1.0, 1.0)),
                     ("prior-variance", (100.0, 0.25, 0.0025))):
        g = L1Gains(omega_i=5.0, omega_x=20.0, lc_var_f=var[0],
                    lc_var_m=var[1], lc_var_q=var[2])
        obs = L1DisturbanceObserver(g)
        for _ in range(int(60.0 / dt)):
            est = obs.attribute(dyn0, np.eye(3), dt, d_c=d_c_true)
        th_hat = Ba.T @ est["w_hat"]
        results[tag] = np.linalg.norm(est["F_y"])
        print(f"     {tag:24s}: theta_hat_f = {th_hat[0]:+7.3f} (true -5.500)"
              f"   |F_y| {np.linalg.norm(lumped):.3f} -> "
              f"{np.linalg.norm(est['F_y']):.3f}")
    check("   minimum-norm makes the phantom WORSE here (recorded)",
          -results["minimum-norm (the note)"] / np.linalg.norm(lumped), -1.0)
    check("   prior-variance W at least halves the phantom",
          results["prior-variance"] / np.linalg.norm(lumped), 0.55)

    # ---- 4. a REAL contact wrench must still come through -------------------
    print("4. a real contact wrench is still reported")
    w_e_true = np.array([1.5, -0.8, 2.0, 0.05, -0.03, 0.02])
    g = L1Gains(omega_i=5.0, omega_x=20.0,
                lc_var_f=100.0, lc_var_m=0.25, lc_var_q=0.0025)
    obs = L1DisturbanceObserver(g)
    d_c_mix = np.linalg.solve(Tm.T, w_true + Je.T @ w_e_true)
    for _ in range(int(60.0 / dt)):
        est = obs.attribute(dyn0, np.eye(3), dt, d_c=d_c_mix)
    print(f"     w_e_hat = {est['w_e']}")
    print(f"     w_e     = {w_e_true}")
    check("   |w_e_hat - w_e|_inf, static arm", np.abs(est["w_e"] - w_e_true).max(), 0.7)

    # ---- 5. the wrench must NOT leak into the internal estimate -------------
    # Z_0^T J_e^T = 0 exactly, so w_hat must be identical with and without a
    # contact wrench.  This is what makes the scheme contact-flag-free.
    print("5. the contact wrench never moves the internal estimate")
    ws = []
    for extra in (np.zeros(6), w_e_true, 10.0 * w_e_true):
        g = L1Gains(omega_i=5.0, lc_var_f=100.0, lc_var_m=0.25, lc_var_q=0.0025)
        obs = L1DisturbanceObserver(g)
        dc = np.linalg.solve(Tm.T, w_true + Je.T @ extra)
        for _ in range(2000):
            est = obs.attribute(dyn0, np.eye(3), dt, d_c=dc)
        ws.append(est["w_hat"])
    check("   |w_hat(no contact) - w_hat(10x contact)|_inf",
          np.abs(ws[0] - ws[2]).max(), 1e-9)

    print("\n" + ("ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_selftest())

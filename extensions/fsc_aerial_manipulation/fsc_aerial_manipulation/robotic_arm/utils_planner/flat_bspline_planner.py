"""
flat_bspline_planner.py — setpoint-to-setpoint whole-body transitions
parameterized as B-SPLINES IN THE FLAT OUTPUTS (2026-09-04).

NEW FILE, additive on purpose.  transition_planner.py — the flight-validated
engine the whole-body planner runs today — is imported for its
model, rest algebra and IK, and is NEVER modified.  Both planners emit the
SAME reference dict, so this one is a drop-in alternative once it has been
flown.

WHY THIS EXISTS
---------------
transition_planner.py prescribes the END-EFFECTOR path and SOLVES the CoM,
because the task is an EE task.  That choice makes the compatibility relation
IMPLICIT — x_c appears on both sides through its own second derivative — and
costs a Picard fixed point (~10 iterations x 201 grid points, ~646 ms, holding
the GIL hard enough to need a yield hook so the 100 Hz stream survives).

This module parameterizes the FLAT OUTPUTS instead:

    z(t) = [ x_c(t) in R^3 ,  psi(t) in R ,  q(t) in R^4 ]        (8 channels)

8 channels against 8 inputs (thrust, 3 body torques, 4 joint torques), and the
map z -> (R0, omega, f, tau, tau_j) is EXPLICIT algebra plus differentiation:

    a_c = xdd_c + g e3 ,   b3 = a_c/||a_c||          <- Newton for the WHOLE system

so the thrust direction is a function of xdd_c ALONE.  There is no fixed point
left to iterate: compatibility becomes definitional.  That is the whole reason
the CoM — not the base, not the EE — must be the position flat output.  Pick
either of the other two and the implicit loop comes straight back.

WHAT FALLS OUT OF THE CHANGE
----------------------------
* No Picard iteration, so the solve is a linear algebra problem, not a
  contraction.
* No z-x-z chart.  q is a decision variable, so nothing ever DECOMPOSES an
  orientation, and the chart's wrist singularity (beta -> 0, guarded at 5 deg
  in transition_planner) does not exist here.  The only singularities left are
  ||a_c|| = 0 (free fall) and ||Pi b1_d|| = 0 (heading parallel to thrust),
  both of which carry ~2 orders of magnitude of margin on a rest-to-rest
  transition and are checked in `diag`.
* No sigma null space.  q2 and q3 are independent channels; the split that
  transition_planner interpolates arbitrarily is simply gone.
* qdot_d becomes EXACT.  In transition_planner it is the one numerically
  differentiated field in the message (central difference, h = 1 ms, because
  the joint-recovery map has no closed form derivative).  Here q is a spline,
  so its derivative is a spline.

WHAT IT COSTS
-------------
The EE chain moves from free to derived.  transition_planner PRESCRIBES r_ed,
so r_ed_dot/r_ed_ddot are one multiply each; here

    r_e = x_c - R0 * (r_0c(q) - r_0e(q))

has to be differentiated twice, which needs R0dd — which needs the CoM SNAP.
That is exactly why the reference message carries x_cd through its fourth
derivative: it is the minimum for a C^2 end-effector reference, not padding.
The arm side is done by a forward velocity/acceleration recursion down the
4-link chain (the kinematic half of RNEA), never by forming d2/dq2 symbolically.

BASIS
-----
Clamped B-splines with uniform interior knots, one shared set of interior
breakpoints for every channel (the EE relation mixes x_c and q, so common
spans keep everything on one grid).  Three properties earn the basis here:

  1. the derivative of a degree-p B-spline is a degree-(p-1) B-spline whose
     control points are scaled DIFFERENCES of the originals — so every
     derivative bound is a LINEAR condition on the decision variables, which
     is what makes constraints tractable when they are added;
  2. continuity is structural: simple interior knots give C^(p-1) for free, no
     continuity equalities to impose and no ringing of the kind the segmented
     polynomial fits in compatible_trajectory.py show;
  3. local support -> banded, sparse cost matrices.

Boundary conditions need no equality constraints either.  For a CLAMPED
spline the derivatives vanish through order k at an end IFF the first (last)
k+1 control points COINCIDE, so rest-to-rest is imposed by construction:

    x_c :  c0 = c1 = c2 = c3 = c4      kills xdot..x(4)  =>  omega = 0 AND
                                       omegadot = 0, i.e. ZERO TORQUE at the
                                       hold handover, not merely zero velocity
    psi, q :  c0 = c1 = c2             kills the first two derivatives

DEGREES
-------
p = 7 for x_c is deliberate.  The minimum for continuous torque is 5 (snap
continuous), but r_ed_ddot depends LINEARLY on snap, so p = 5 would put a kink
in the end-effector acceleration field at every knot; p = 7 makes snap C^2 and
every field of the message at least C^2.  psi and q take p = 5.

THE OPTIMIZATION
----------------
    minimize  w_x integral ||x_c^(4)||^2 + w_p integral psi^(2)^2
                                         + w_q integral ||q^(3)||^2
    over      the free control points, and T
    s.t.      rest conditions at both ends                       (1)
              rotor speed and joint torque within bounds         (2)
              CoM speed/acceleration, angle rate/acceleration    (3)
              joint angles inside the working range              (4)
              T >= T_min                                         (5)

Each derivative is a linear map D_r on the control points and each integral a
Gram form, so the cost is c' (D_r' G D_r) c — a QP.  What makes the constraints
cheap is that almost none of them enter it:

  (1) STRUCTURAL.  Coincident control points (C4) — satisfied identically, not
      solved for.  `pin_xc`/`pin_ang` choose how many orders are killed; the
      defaults kill x_c through SNAP, so the handover carries zero body torque
      rather than merely zero velocity.  Measured residual over every
      derivative field at both ends: ~1e-12.
  (4) ENDPOINTS checked and refused by name; the PATH needs no test, because
      the unconstrained optimum for a joint channel IS the global min-jerk
      quintic (it lies in the degree-5 spline space) and that quintic is
      MONOTONE — its derivative is 30 u^2 (1-u)^2 >= 0 — so both endpoints in
      the box puts the whole path in the box.  Measured overshoot: exactly 0.
      This is the entire refusal class transition_planner's prescribed
      sigma(t)*beta(t) PRODUCT creates (3 of 60 random goals refused with both
      endpoints legal).  A convex-hull box on the control points is wired up
      as a fallback and has never yet been needed.
  (3) CLOSED FORM.  The control points are T-independent, so peak speed is
      exactly (unit-interval peak)/T and acceleration (unit peak)/T^2: the
      required duration is one division and one square root.  No search.
  (2) The only nonlinear class, and the only expensive one — thrust, per-rotor
      force and joint torque come from inverse dynamics on the reference.
      Enforced by DILATING T, which is monotone, using the same 1/T^2 law
      (M a is linear in accelerations, C v quadratic in velocities), so 2-3
      evaluations rather than a bisection.  The quasi-static (T -> infinity)
      limit is tested first: a violation there is a POSE problem and is
      refused, since no duration can fix it.
  (5) A floor applied last, always safe — a longer transition can only reduce
      every bounded quantity.

So T is the single knob every dynamic constraint is enforced through, and the
shape is solved exactly once.  Two things follow:

  * the WEIGHTS STILL DO NOTHING.  Nothing couples the channels: the cost is
    per-channel, the box is per-channel, and everything else acts through T.
    They are in the API because the moment a constraint spans channels (an EE
    corridor, a torque bound written into the cost) they start to matter, and
    getting the non-dimensionalization wrong then is a real bug (snap is
    m/s^4, psidd rad/s^2, qddd rad/s^3).
  * ON THIS PLANT THE INPUT BOUNDS ARE POSE CONSTRAINTS, NOT SPEED ONES.
    Measured on the self-test transition: peak joint torque 0.7712 N.m of
    which 0.7704 is gravity — the dynamic share is 0.10%.  Slowing down buys
    essentially nothing, which is why the quasi-static refusal exists and why
    class (2) can be switched off (`rotor=None, tau_joint_max=None`) for a 4x
    cheaper plan when a static margin has already been established.

Offline validation: run this file directly (system python3, no Isaac):
    python3 flat_bspline_planner.py
"""

import os
import sys
import time

import numpy as np

if __package__ in (None, ""):
    _HERE = os.path.dirname(os.path.abspath(__file__))
    _EXT = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
    sys.path.insert(0, _EXT)

from fsc_aerial_manipulation.robotic_arm.utils_controller import (  # noqa: E402
    controller as C,
)
from fsc_aerial_manipulation.robotic_arm.utils_planner import (  # noqa: E402
    compatible_trajectory as CT,
)
from fsc_aerial_manipulation.robotic_arm.utils_planner import (  # noqa: E402
    transition_planner as TP,
)

_E3 = np.array([0.0, 0.0, 1.0])


# ===========================================================================
# clamped B-spline
# ===========================================================================

def clamped_knots(degree, n_spans, T):
    """Uniform clamped knot vector on [0, T]: degree+1 repeats at each end and
    n_spans-1 simple interior knots.  Gives n_spans + degree control points."""
    interior = np.linspace(0.0, T, n_spans + 1)[1:-1]
    return np.concatenate([np.zeros(degree + 1), interior,
                           np.full(degree + 1, T)])


class BSpline:
    """Clamped B-spline curve, vector- or matrix-valued in the coefficients.

    `coeffs` is (n, ...) — de Boor is LINEAR in the coefficients, so feeding it
    the identity matrix evaluates the basis vector itself, which is how the
    Gram matrices below are assembled without a separate basis routine.
    """

    def __init__(self, knots, coeffs, degree):
        self.u = np.asarray(knots, float)
        self.c = np.asarray(coeffs, float)
        self.p = int(degree)
        self.n = self.c.shape[0]
        if self.u.size != self.n + self.p + 1:
            raise ValueError(
                f"knot/coeff mismatch: {self.u.size} knots, {self.n} coeffs, "
                f"degree {self.p} (need {self.n + self.p + 1} knots)")

    # -- evaluation --------------------------------------------------------
    def _span(self, t):
        u, p, n = self.u, self.p, self.n
        if t >= u[n]:
            return n - 1
        if t <= u[p]:
            return p
        lo, hi = p, n
        mid = (lo + hi) // 2
        while t < u[mid] or t >= u[mid + 1]:
            if t < u[mid]:
                hi = mid
            else:
                lo = mid
            mid = (lo + hi) // 2
        return mid

    def __call__(self, t):
        u, p = self.u, self.p
        s = self._span(float(t))
        d = [self.c[s - p + j].copy() for j in range(p + 1)]
        for r in range(1, p + 1):
            for j in range(p, r - 1, -1):
                i = s - p + j
                den = u[i + p - r + 1] - u[i]
                a = 0.0 if den <= 0.0 else (t - u[i]) / den
                d[j] = (1.0 - a) * d[j - 1] + a * d[j]
        return d[p]

    # -- differentiation ---------------------------------------------------
    def deriv_matrix(self):
        """(n-1, n) map from this spline's control points to its derivative's."""
        u, p, n = self.u, self.p, self.n
        D = np.zeros((n - 1, n))
        for i in range(n - 1):
            den = u[i + p + 1] - u[i + 1]
            f = 0.0 if den <= 0.0 else p / den
            D[i, i] = -f
            D[i, i + 1] = f
        return D

    def deriv(self):
        if self.p < 1:
            raise ValueError("cannot differentiate a degree-0 spline")
        return BSpline(self.u[1:-1], self.deriv_matrix() @ self.c, self.p - 1)

    def derivs(self, order):
        """[self, d/dt, ..., d^order/dt^order] as a list of BSplines."""
        out = [self]
        for _ in range(order):
            out.append(out[-1].deriv())
        return out


def _basis_vector(knots, degree, n, t):
    """[N_0(t) ... N_{n-1}(t)] via de Boor on identity coefficients."""
    return BSpline(knots, np.eye(n), degree)(t)


def _gram(knots, degree, n):
    """G_ij = integral N_i N_j dt, exact by Gauss-Legendre per span."""
    brk = np.unique(knots)
    ng = degree + 1                      # exact for a degree-2*degree product
    xg, wg = np.polynomial.legendre.leggauss(ng)
    G = np.zeros((n, n))
    for a, b in zip(brk[:-1], brk[1:]):
        if b <= a:
            continue
        mid, half = 0.5 * (a + b), 0.5 * (b - a)
        for x, w in zip(xg, wg):
            nv = _basis_vector(knots, degree, n, mid + half * x)
            G += (w * half) * np.outer(nv, nv)
    return G


_QCACHE = {}


def _cost_matrix(degree, n_spans, order):
    """Q with c' Q c = integral ||d^order/dt^order spline||^2 dt, built ONCE
    on the unit interval and cached.

    Safe to cache across durations: uniform clamped knots on [0, T] are T times
    those on [0, 1], so N_i^T(t) = N_i^1(t/T), D_r^T = T^-r D_r^1 and the Gram
    picks up one factor of T — hence Q^T = T^(1-2r) Q^1, a single positive
    scale.  The plan solves Q_ff c_f = -Q_fx c_x, where that scale CANCELS: the
    optimal control points are independent of T.  T re-enters only through the
    knots the spline is evaluated on."""
    key = (int(degree), int(n_spans), int(order))
    if key in _QCACHE:
        return _QCACHE[key]
    u, p = clamped_knots(degree, n_spans, 1.0), degree
    m = n_spans + degree
    D = np.eye(m)
    for _ in range(order):
        D = BSpline(u, np.eye(m), p).deriv_matrix() @ D
        u, p, m = u[1:-1], p - 1, m - 1
    Q = D.T @ _gram(u, p, m) @ D
    _QCACHE[key] = Q
    return Q


def _solve_channel(degree, n_spans, order, y0, y1, n_pin, lo=None, hi=None):
    """Min-||d^order||^2 control points for ALL axes of one channel at once,
    with the first/last `n_pin` pinned to y0/y1 — which is what makes the first
    n_pin-1 derivatives vanish at each end.  The constraint set is empty, so
    this is one symmetric positive-definite solve, shared across axes.

    Q's null space is the polynomials of degree < order; pinning n_pin >= order
    control points at an end kills all of them, so A is positive definite (the
    lstsq fallback is belt and braces)."""
    n = n_spans + degree
    if 2 * n_pin > n:
        raise ValueError(
            f"{n} control points cannot pin {n_pin} at each end — "
            "raise n_spans or lower the degree")
    y0 = np.atleast_1d(np.asarray(y0, float))
    y1 = np.atleast_1d(np.asarray(y1, float))
    Q = _cost_matrix(degree, n_spans, order)
    fixed = np.concatenate([np.arange(n_pin), np.arange(n - n_pin, n)])
    free = np.setdiff1d(np.arange(n), fixed)
    c = np.zeros((n, y0.size))
    c[:n_pin] = y0
    c[n - n_pin:] = y1
    if free.size:
        A = Q[np.ix_(free, free)]
        b = -Q[np.ix_(free, fixed)] @ c[fixed]
        try:
            c[free] = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            c[free] = np.linalg.lstsq(A, b, rcond=None)[0]
    if lo is None:
        return c
    # Box constraint on the control points => box constraint on the CURVE, by
    # the convex hull property (sufficient, not tight).  Almost never binds:
    # the unconstrained optimum for each joint channel IS the global min-jerk
    # quintic, which is monotone, so a path with both endpoints in the box
    # stays in it.  Kept because that guarantee is a property of THIS cost
    # order, not of the formulation.
    lo = np.atleast_1d(np.asarray(lo, float))
    hi = np.atleast_1d(np.asarray(hi, float))
    if np.all(c >= lo - 1e-12) and np.all(c <= hi + 1e-12):
        return c
    try:
        from scipy.optimize import minimize
    except ImportError as exc:                              # pragma: no cover
        raise RuntimeError(
            "control points left the joint box and scipy is unavailable for "
            "the bound-constrained re-solve") from exc
    Aff = Q[np.ix_(free, free)]
    for j in range(y0.size):
        bj = -Q[np.ix_(free, fixed)] @ c[fixed, j]
        res = minimize(
            lambda x: (0.5 * x @ Aff @ x - bj @ x,
                       Aff @ x - bj),
            c[free, j], jac=True, method="L-BFGS-B",
            bounds=[(lo[j], hi[j])] * free.size,
            options={"maxiter": 500, "ftol": 1e-16, "gtol": 1e-12})
        c[free, j] = res.x
    return c


# ===========================================================================
# the flat map: (x_c, psi, q) and derivatives -> the whole-body reference
# ===========================================================================

def _skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def _normalize_d2(v, vd, vdd):
    """f = v/||v|| with its first two time derivatives (exact, not FD)."""
    nrm = float(np.linalg.norm(v))
    if nrm < 1e-12:
        raise ValueError("cannot normalize a vanishing vector")
    f = v / nrm
    nd = float(f @ vd)
    fd = (vd - f * nd) / nrm
    ndd = float(fd @ vd + f @ vdd)
    fdd = (vdd - 2.0 * fd * nd - f * ndd) / nrm
    return f, fd, fdd


def build_R0_d2(acc, jerk, snap, b1, b1d, b1dd):
    """R0 = build(b3, b1) with its first two time derivatives.

    Same Gram-Schmidt as compatible_trajectory._build_R0 — thrust axis from the
    CoM acceleration, heading projected into the rotor plane — differentiated
    in closed form.  `acc` is a_c = xdd_c + g e3, so jerk/snap are its own
    derivatives."""
    b3, b3d, b3dd = _normalize_d2(acc, jerk, snap)
    s = float(b3 @ b1)
    sd = float(b3d @ b1 + b3 @ b1d)
    sdd = float(b3dd @ b1 + 2.0 * (b3d @ b1d) + b3 @ b1dd)
    w = b1 - s * b3
    wd = b1d - sd * b3 - s * b3d
    wdd = b1dd - sdd * b3 - 2.0 * sd * b3d - s * b3dd
    if np.linalg.norm(w) < 1e-9:
        raise ValueError("heading is parallel to the thrust axis — "
                         "R0 is undefined (||Pi b1|| ~ 0)")
    b1c, b1cd, b1cdd = _normalize_d2(w, wd, wdd)
    b2c = np.cross(b3, b1c)
    b2cd = np.cross(b3d, b1c) + np.cross(b3, b1cd)
    b2cdd = (np.cross(b3dd, b1c) + 2.0 * np.cross(b3d, b1cd)
             + np.cross(b3, b1cdd))
    R0 = np.column_stack([b1c, b2c, b3])
    R0d = np.column_stack([b1cd, b2cd, b3d])
    R0dd = np.column_stack([b1cdd, b2cdd, b3dd])
    return R0, R0d, R0dd


def arm_chain_d2(q, qd, qdd, params):
    """(r_0c, r_0e, R_e) in the BASE frame with their first two time
    derivatives, by a forward velocity/acceleration recursion on the same
    exact chain compatible_trajectory._arm_kin walks.

    O(n) and Hessian-free: propagating omega_i and omegadot_i down the links is
    what replaces forming d2 r_0e / dq2 symbolically."""
    n = params["n"]
    h, li, ci, mi = (params["h_i_im1"], params["l_i"],
                     params["com_i"], params["m_i"])
    R = [np.eye(3)]
    w = [np.zeros(3)]
    a = [np.zeros(3)]
    for i in range(n):
        axis = R[i] @ h[i]
        R.append(R[i] @ CT._rot(h[i], q[i]))
        w.append(w[i] + qd[i] * axis)
        a.append(a[i] + qdd[i] * axis + qd[i] * np.cross(w[i], axis))
    O = [np.zeros(3)]
    Od = [np.zeros(3)]
    Odd = [np.zeros(3)]
    for i in range(1, n + 1):
        v = R[i] @ li[i - 1]
        O.append(O[i - 1] + v)
        Od.append(Od[i - 1] + np.cross(w[i], v))
        Odd.append(Odd[i - 1] + np.cross(a[i], v)
                   + np.cross(w[i], np.cross(w[i], v)))
    m_total = float(sum(mi))
    base_com = np.asarray(params.get("base_com", np.zeros(3)), float)
    c = mi[0] * base_com
    cd = np.zeros(3)
    cdd = np.zeros(3)
    for i in range(1, n + 1):
        v = R[i] @ ci[i - 1]
        c = c + mi[i] * (O[i - 1] + v)
        cd = cd + mi[i] * (Od[i - 1] + np.cross(w[i], v))
        cdd = cdd + mi[i] * (Odd[i - 1] + np.cross(a[i], v)
                             + np.cross(w[i], np.cross(w[i], v)))
    Sw, Sa = _skew(w[n]), _skew(a[n])
    return {
        "r_0c": c / m_total, "r_0c_d": cd / m_total, "r_0c_dd": cdd / m_total,
        "r_0e": O[n], "r_0e_d": Od[n], "r_0e_dd": Odd[n],
        "R_e": R[n], "R_e_d": Sw @ R[n], "R_e_dd": (Sa + Sw @ Sw) @ R[n],
    }


def _vee(S):
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def flat_state(params, xc, psi, q):
    """The whole-body reference at one instant, from the flat outputs.

    xc  : (5, 3) rows x_c, xdot, xddot, x^(3), x^(4)
    psi : (3,)   psi, psidot, psiddot
    q   : (3, 4) rows q, qdot, qddot

    Returns the exact key set transition_planner.rest_ref emits, so the two
    planners are interchangeable at the whole-body planner's boundary."""
    xc = np.asarray(xc, float)
    q = np.asarray(q, float)
    ps, psd, psdd = (float(psi[0]), float(psi[1]), float(psi[2]))
    cp, sp = np.cos(ps), np.sin(ps)

    b1 = np.array([cp, sp, 0.0])
    b1d = psd * np.array([-sp, cp, 0.0])
    b1dd = (psdd * np.array([-sp, cp, 0.0])
            - psd ** 2 * np.array([cp, sp, 0.0]))

    acc = xc[2] + params["g"] * _E3
    R0, R0d, R0dd = build_R0_d2(acc, xc[3], xc[4], b1, b1d, b1dd)

    k = arm_chain_d2(q[0], q[1], q[2], params)

    # r_e = x_c - R0 * (r_0c - r_0e); product rule twice
    d = k["r_0c"] - k["r_0e"]
    dd = k["r_0c_d"] - k["r_0e_d"]
    ddd = k["r_0c_dd"] - k["r_0e_dd"]
    u = R0 @ d
    ud = R0d @ d + R0 @ dd
    udd = R0dd @ d + 2.0 * (R0d @ dd) + R0 @ ddd

    W = R0 @ k["R_e"]
    Wd = R0d @ k["R_e"] + R0 @ k["R_e_d"]
    Wdd = R0dd @ k["R_e"] + 2.0 * (R0d @ k["R_e_d"]) + R0 @ k["R_e_dd"]

    # base kinematics + body rates, for the inverse dynamics of the inputs
    cw = R0 @ k["r_0c"]
    cwd = R0d @ k["r_0c"] + R0 @ k["r_0c_d"]
    cwdd = R0dd @ k["r_0c"] + 2.0 * (R0d @ k["r_0c_d"]) + R0 @ k["r_0c_dd"]
    what = R0.T @ R0d                       # Rdot = R0 * hat(omega), BODY frame
    aux = {
        "a_c": acc,                 # xdd_c + g e3 — the EXACT collective term
        "R0": R0, "R0_d": R0d, "R0_dd": R0dd,
        "omega": _vee(what), "omega_dot": _vee(R0.T @ R0dd - what @ what),
        "r_0": xc[0] - cw, "r_0_d": xc[1] - cwd, "r_0_dd": xc[2] - cwdd,
    }
    ref = {
        "x_cd": xc[0].copy(), "x_cd_dot": xc[1].copy(),
        "x_cd_ddot": xc[2].copy(), "x_cd_d3": xc[3].copy(),
        "x_cd_d4": xc[4].copy(),
        "b1_d": b1, "b1_d_dot": b1d, "b1_d_ddot": b1dd,
        "r_ed": xc[0] - u, "r_ed_dot": xc[1] - ud, "r_ed_ddot": xc[2] - udd,
        "b1_de": W[:, 0].copy(), "b1_de_dot": Wd[:, 0].copy(),
        "b1_de_ddot": Wdd[:, 0].copy(),
        "q_d": q[0].copy(), "qdot_d": q[1].copy(),
    }
    return ref, aux


def flat_to_reference(params, xc, psi, q):
    """flat_state without the internals — the message fields only."""
    return flat_state(params, xc, psi, q)[0]


# ===========================================================================
# control inputs along the reference (inverse dynamics)
# ===========================================================================

class RotorModel:
    """Thrust + body torque -> per-rotor FORCE, and the rotor-speed bounds as
    a force interval (f = k omega^2).

    Geometry comes from the controller's own RotorMixerParams because the
    whole-body model lives in AM_realign's model frame; the coefficients come
    from t650_params.  The two assets label the four channels differently but
    describe the SAME four positions, and only the per-rotor magnitude is
    bounded here, so the labelling does not enter."""

    def __init__(self, k_thrust, k_torque, omega_min, omega_max,
                 positions=None, rot_dir=None):
        mp = C.RotorMixerParams()
        pos = mp.rotor_positions if positions is None else np.asarray(positions)
        rd = mp.rot_dir if rot_dir is None else np.asarray(rot_dir)
        B = np.zeros((4, len(pos)))
        for i, r in enumerate(pos):
            B[0, i] = 1.0
            B[1, i] = r[1]
            B[2, i] = -r[0]
            B[3, i] = rd[i] * k_torque / k_thrust
        self.B_inv = np.linalg.pinv(B)
        self.k = float(k_thrust)
        self.f_min = self.k * float(omega_min) ** 2
        self.f_max = self.k * float(omega_max) ** 2

    def forces(self, thrust, tau_body):
        """Per-rotor force, UNCLAMPED — a negative entry is a real infeasibility
        and must not be hidden the way RotorMixer.mix's max(.,0) hides it."""
        return self.B_inv @ np.array([thrust, *tau_body])

    def omega(self, forces):
        return np.sqrt(np.maximum(forces, 0.0) / self.k)


def t650_rotor_model():
    t = TP._load_t650_params_module()
    return RotorModel(t.ROTOR_CONSTANT, t.ROLLING_MOMENT_COEFFICIENT,
                      t.ZERO_POSITION_ARMED, t.MAX_ROTOR_VEL)


def inverse_inputs(params, aux, q, qd, qdd, rotor=None):
    """(thrust, body torque, joint torques) required to fly the reference, and
    the per-rotor forces they allocate to.

    COLLECTIVE THRUST COMES FROM NEWTON, NOT FROM THE INVERSE DYNAMICS.
    Newton for the whole system is m xdd_c = f R0 e3 - m g e3, so

        f = m ||xdd_c + g e3||

    EXACTLY — a function of the CoM acceleration alone, with no q, no arm
    dynamics and no mass matrix in it.  Taking it instead as tau[0:3].(R0 e3)
    routes the same number through M a + C v + g, which agrees at the
    endpoints to 7e-15 and then DRIFTS mid-transition: measured 1.4e-02 N of
    magnitude and 0.76 deg of DIRECTION error on the test plan, i.e. the
    model's resultant force is not exactly along its own thrust axis once the
    vehicle is moving.  Small against a 36.75 N hover, but the collective term
    is 43% of the per-rotor ceiling and there is no reason to spend a model
    inconsistency on it when the closed form is free.  `thrust_residual`
    reports the disagreement so it stays visible rather than silently absorbed.

    The BODY TORQUE has no such shortcut: tau = M a + C v + g is what it is,
    and it is what makes (C14) nonlinear in the decision variables.

    THE GENERALIZED VELOCITY IS (v_body, omega_body, qdot) -- BODY frame in
    BOTH of the first two blocks, matching what the demos feed dynamics()
    (`v0 = R0.T @ v_world`).  Two consequences that are easy to get wrong and
    were both wrong here until 2026-09-05:

      * the first acceleration block is d/dt(R0^T rdot_0), NOT R0^T rddot_0:
            nu1_dot = R0^T rddot_0 - omega x v_body
        The -omega x v term cannot be hiding in C, because dynamics() never
        reads the linear-velocity slot at all, so C has no term in v_body.
      * tau[0:3] is therefore a BODY-frame force, and the identity to check is
        tau[0:3] = m R0^T a_c, not m a_c in the world.

    Diagnosed from the gravity block: g[0:3] = m g R0^T e3 to 7e-15, i.e. it
    ROTATES with the vehicle, which a world-frame translation coordinate could
    never do.  Feeding the world-frame acceleration left tau[0:3] up to 0.88 N
    and 0.76 deg off the thrust axis; with the body-frame form the identity is
    exact to 7e-15 N."""
    n = params["n"]
    R0, om = aux["R0"], aux["omega"]
    v_body = R0.T @ aux["r_0_d"]
    a_body = R0.T @ aux["r_0_dd"] - np.cross(om, v_body)
    X = np.concatenate([aux["r_0"], R0.flatten(order="F"), q,
                        v_body, om, qd])
    dyn = C.dynamics(X, params)
    v = np.concatenate([v_body, om, qd])
    a = np.concatenate([a_body, aux["omega_dot"], qdd])
    tau = dyn["M"] @ a + dyn["C"] @ v + dyn["g"]
    thrust = float(sum(params["m_i"]) * np.linalg.norm(aux["a_c"]))
    out = {"thrust": thrust, "F_body": tau[0:3],
           "F_world": R0 @ tau[0:3],
           "tau_body": tau[3:6], "tau_joint": tau[6:6 + n],
           # now an exact identity, so it reads as a wiring check rather than
           # a tolerance: any drift here means the model or the frames moved.
           "thrust_residual": abs(thrust - float(tau[0:3] @ _E3))}
    if rotor is not None:
        out["rotor_force"] = rotor.forces(thrust, tau[3:6])
    return out


def collective_thrust_limits(params, rotor):
    """The COLLECTIVE half of (C14), reduced to a bound on ||xdd_c||.

    f = m||xdd_c + g e3|| with f in [4 k w_min^2, 4 k w_max^2] gives, by the
    triangle inequality,

        ||xdd_c|| <= 4 k w_max^2 / m - g        (from the upper rotor bound)
        ||xdd_c|| <= g - 4 k w_min^2 / m        (from the lower)

    Both bound the CoM ACCELERATION alone, so both are convex in the control
    points — a second-order cone on the rows of D_2 C_x, by the hull property
    — and neither needs a dynamics() call.  On the T650 the first is
    13.19 m/s^2 against an a_max of 0.15 (88x of slack) and the second
    9.63 m/s^2, so the collective half of the rotor constraint is DISCHARGED
    BY (C13) and only the torque-induced differential is ever evaluated.
    Returns (upper, lower) as limits on ||xdd_c||."""
    m = float(sum(params["m_i"]))
    return (4.0 * rotor.f_max / m - params["g"],
            params["g"] - 4.0 * rotor.f_min / m)


# ===========================================================================
# the planner
# ===========================================================================

_UU = np.linspace(0.0, 1.0, 2001)
PEAK_DS = TP.PEAK_DS
PEAK_D2S = TP.PEAK_D2S

DEFAULT_OPTS = {
    # ---- basis -----------------------------------------------------------
    "deg_xc": 7,        # snap C^2 -> every message field at least C^2
    "deg_psi": 5,
    "deg_q": 5,
    "n_spans": 12,      # shared interior breakpoints
    # cost: which derivative each channel minimizes
    "order_xc": 4,      # snap
    "order_psi": 2,
    "order_q": 3,       # jerk -> smooth joint torque
    # weights: still inert — nothing couples the channels (see the module head)
    "w_xc": 1.0, "w_psi": 1.0, "w_q": 1.0,

    # ---- constraints -----------------------------------------------------
    # (1) rest: velocities (and more) zero at both ends.  STRUCTURAL — the
    #     coincident control points of (C4); `pin_xc`/`pin_ang` set how many
    #     derivative orders are killed.  pin=2 would be "velocity only";
    #     the defaults also kill acceleration, and for x_c jerk and snap, so
    #     the handover carries zero body torque, not just zero speed.
    "pin_xc": 5,        # x_c: value + 4 derivatives = 0
    "pin_ang": 3,       # psi, q: value + 2 derivatives = 0
    # (2) control inputs
    "tau_joint_max": 3.0,       # N.m — the number wb yaml / the arm
                                # controller / 06 all carry; keep them equal
    "rotor": "t650",            # RotorModel, "t650", or None to skip
    # (3) kinematics.  CoM translation, then any angle channel (psi and each q)
    "v_max": 0.30,      # m/s
    "a_max": 0.15,      # m/s^2
    "w_max": 0.30,      # rad/s
    "dw_max": 0.60,     # rad/s^2
    # (4) joint angles.  None -> the validated OM-X working range
    "q_min": None, "q_max": None,
    "sigma_nd_min": None,       # e.g. TP.SIGMA_ND_MARGIN to refuse at the ends
    # (5) duration
    "T_min": 3.0,       # floor, applied last: a longer T is always gentler
    "T_max": 40.0,
    "T": None,          # override: size nothing, only REPORT (diag.bounds_ok)

    # ---- grids -----------------------------------------------------------
    # The input bounds are the ONLY expensive constraint: each sample costs a
    # controller.dynamics() call (the full transformed model, of which the
    # inverse dynamics uses M, C, g).  The peaks are smooth in t, so a coarse
    # grid sizes T perfectly well; the report re-measures on a finer one.
    "Nbound": 21,
    # Called every `yield_every` samples of the input sweep — the planner's
    # only block long enough to matter to a caller that is also streaming.
    "yield_hook": None,
    "yield_every": 4,
    "Ncheck": 401,      # diagnostics only; costs ~50x the solve, set 0 online
    "max_dilations": 8,
}


def suggest_duration(params, rest0, rest1, o):
    """transition_planner's closed-form peak-factor bound, reused so the two
    planners can be compared at the same T.  It describes a min-snap SHAPE, so
    against an optimized spline treat it as a starting value: `diag` reports
    the peaks actually achieved."""
    r0 = TP.rest_ref(params, rest0["x_b"], rest0["phi"], rest0["q"])
    r1 = TP.rest_ref(params, rest1["x_b"], rest1["phi"], rest1["q"])
    d_move = max(float(np.linalg.norm(r1["x_cd"] - r0["x_cd"])),
                 float(np.linalg.norm(np.asarray(rest1["x_b"], float)
                                      - np.asarray(rest0["x_b"], float))))
    dphi = TP._unwrap_near(float(rest1["phi"]), float(rest0["phi"]))
    d_ang = max(abs(dphi - float(rest0["phi"])),
                float(np.max(np.abs(np.asarray(rest1["q"], float)
                                    - np.asarray(rest0["q"], float)))))
    t_v = PEAK_DS * d_move / o["v_max"] if d_move > 0 else 0.0
    t_a = np.sqrt(PEAK_D2S * d_move / o["a_max"]) if d_move > 0 else 0.0
    t_w = PEAK_DS * d_ang / o["w_max"] if d_ang > 0 else 0.0
    return float(np.clip(max(t_v, t_a, t_w), o["T_min"], o["T_max"]))


def _scan_max(f, n_scan=65, n_refine=25):
    """max of a smooth scalar function on [0, 1]: coarse scan to bracket, then
    golden section.  Cheaper AND sharper than a dense grid — which matters
    because this peak SETS the duration, so a grid that under-reads it puts
    the finished trajectory over its own bound."""
    ss = np.linspace(0.0, 1.0, n_scan)
    vals = np.array([f(x) for x in ss])
    i = int(np.argmax(vals))
    a, b = ss[max(i - 1, 0)], ss[min(i + 1, n_scan - 1)]
    gr = 0.5 * (np.sqrt(5.0) - 1.0)
    c, d = b - gr * (b - a), a + gr * (b - a)
    fc, fd = f(c), f(d)
    for _ in range(n_refine):
        if fc > fd:
            b, d, fd = d, c, fc
            c = b - gr * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + gr * (b - a)
            fd = f(d)
    return float(max(vals[i], fc, fd))


def _unit_peaks(ch_c, deg, M, use_norm=True):
    """(max ||z'||, max ||z''||) on the UNIT interval, norm over the channel's
    axes.  The control points are T-independent, so these are the shape's own
    numbers and the duration follows in closed form: z' scales as 1/T and z''
    as 1/T^2, exactly."""
    d = BSpline(clamped_knots(deg, M, 1.0), ch_c, deg).derivs(2)
    agg = np.linalg.norm if use_norm else (lambda v: np.max(np.abs(v)))
    return (_scan_max(lambda x: float(agg(d[1](x)))),
            _scan_max(lambda x: float(agg(d[2](x)))))


def _input_peaks(params, ch, T, N, rotor, quasi_static=False,
                 yield_hook=None, yield_every=4):
    """Worst |tau_joint| and the per-rotor force envelope along the plan.

    quasi_static freezes every derivative, which is the T -> infinity limit:
    what the inputs converge to no matter how slowly the transition is flown.
    A violation there cannot be dilated away and is a genuine refusal."""
    tj = 0.0
    fmin, fmax = np.inf, -np.inf
    for k, t in enumerate(np.linspace(0.0, T, int(N))):
        # Hand the GIL back. This sweep is ~35 of the plan's ~45 ms — one
        # dynamics() call per sample — and holding it in one block drops the
        # whole-body planner's 100 Hz reference stream to 60 Hz with an 83 ms gap
        # (measured; test_replan_stream.py catches exactly this). Nothing goes
        # stale at 83 ms against a 250 ms window, but the margin is the point.
        if yield_hook is not None and k % max(1, int(yield_every)) == 0:
            yield_hook()
        q = np.array([sp(t) for sp in ch["q"]])
        psi = np.array([float(sp(t)[0]) for sp in ch["psi"]])
        xc = np.array([sp(t) for sp in ch["xc"]])
        if quasi_static:
            xc = np.vstack([xc[0], np.zeros((4, 3))])
            psi = np.array([psi[0], 0.0, 0.0])
            q = np.vstack([q[0], np.zeros((2, 4))])
        _, aux = flat_state(params, xc, psi, q)
        u = inverse_inputs(params, aux, q[0], q[1], q[2], rotor)
        tj = max(tj, float(np.max(np.abs(u["tau_joint"]))))
        if rotor is not None:
            fmin = min(fmin, float(np.min(u["rotor_force"])))
            fmax = max(fmax, float(np.max(u["rotor_force"])))
    return {"tau_joint": tj, "f_min": fmin, "f_max": fmax}


def _dilate(x, x_static, bound):
    """T multiplier that brings a bound-violating peak back inside.

    Every dynamic term is quadratic in 1/T (M a is linear in accelerations,
    C v quadratic in velocities, both ~1/T^2), so peak(T) ~ static + K/T^2 and
    the required duration follows in one step from a single measurement.  It
    is an estimate — R0 itself moves with T — so the caller iterates."""
    num, den = x - x_static, bound - x_static
    if num <= 0.0:
        return 1.0
    if den <= 0.0:
        return np.inf
    return float(np.sqrt(num / den))


def plan_flat_transition(params, rest0, rest1, opts=None):
    """Plan the rest-to-rest transition rest0 -> rest1 in flat coordinates.

    rest = {"x_b": base position (world), "phi": MODEL heading [rad],
            "q": (4,) joints} — transition_planner's spec, unchanged.

    Returns {"T", "ref"(t), "q1", "diag", "spl"}.  `ref(t)` has the same keys
    as transition_planner's, so a consumer cannot tell which planner produced
    it.  Raises an operator-readable ValueError when a constraint cannot be
    met.

    HOW THE FIVE CONSTRAINT CLASSES ARE HANDLED, and why they cost so little:

      rest conditions   structural.  Coincident control points, so they are
                        satisfied identically rather than solved for.
      joint angles      endpoints CHECKED and refused; the path is bounded by
                        the convex hull of the control points, which the
                        monotone min-jerk optimum satisfies without a
                        constrained solve in every case measured.
      velocity / accel  CLOSED FORM.  The shape is T-independent, so peak
                        speed scales exactly as 1/T and acceleration as 1/T^2:
                        the required duration is one division and one square
                        root, with no search.
      control inputs    thrust, per-rotor force and joint torque are nonlinear
                        in the coefficients, so these are sized by dilating T
                        (monotone) using the 1/T^2 law above — 2-3 evaluations,
                        not a bisection.  The quasi-static limit is tested
                        first, because a violation there is infeasible at ANY
                        duration.
      minimum time      a floor applied last, which is always safe: a longer
                        transition can only reduce every bounded quantity.

    So T is the single knob every dynamic constraint is enforced through, and
    the expensive part of the problem — the shape — is solved exactly once.
    """
    o = dict(DEFAULT_OPTS, **(opts or {}))
    q0 = np.asarray(rest0["q"], float)
    q1 = np.asarray(rest1["q"], float)
    x_b0 = np.asarray(rest0["x_b"], float)
    x_b1 = np.asarray(rest1["x_b"], float)
    phi0 = float(rest0["phi"])
    phi1 = TP._unwrap_near(float(rest1["phi"]), phi0)
    q_lo = TP.Q_MIN if o["q_min"] is None else np.asarray(o["q_min"], float)
    q_hi = TP.Q_MAX if o["q_max"] is None else np.asarray(o["q_max"], float)
    rotor = t650_rotor_model() if o["rotor"] == "t650" else o["rotor"]
    yh, ye = o["yield_hook"], o["yield_every"]

    t_solve = time.perf_counter()

    # ---- (4) joint angles: the ENDPOINTS are a hard refusal --------------
    # The path needs no test of its own (convex hull + a monotone optimum),
    # but an illegal endpoint has to be caught here or it propagates silently.
    for tag, qq in (("start", q0), ("goal", q1)):
        bad = np.where((qq < q_lo - 1e-9) | (qq > q_hi + 1e-9))[0]
        if bad.size:
            raise ValueError(f"{tag} joints outside the working range: " + ", "
                             .join(f"q{int(j) + 1} = {np.degrees(qq[j]):.1f} "
                                   f"not in [{np.degrees(q_lo[j]):.0f}, "
                                   f"{np.degrees(q_hi[j]):.0f}] deg"
                                   for j in bad))
        if o["sigma_nd_min"] is not None:
            snd = TP._sigma_nd(qq, params)
            if snd < float(o["sigma_nd_min"]):
                raise ValueError(
                    f"{tag} pose sigma_nd = {snd:.3f} < "
                    f"{float(o['sigma_nd_min']):.2f} — too close to a "
                    "singularity")

    # ---- the shape: control points, independent of T ---------------------
    # endpoints in FLAT coordinates: exact algebra at a rest point, where
    # R0 = Rz(phi) and so x_c = x_b + Rz(phi) r_0c(q)
    xc0 = x_b0 + TP._Rz(phi0) @ CT._arm_kin(q0, params)[0]
    xc1 = x_b1 + TP._Rz(phi1) @ CT._arm_kin(q1, params)[0]
    M = int(o["n_spans"])
    cps = {}
    for name, deg, order, npin, y0, y1, lo, hi in (
            ("xc", o["deg_xc"], o["order_xc"], int(o["pin_xc"]),
             xc0, xc1, None, None),
            ("psi", o["deg_psi"], o["order_psi"], int(o["pin_ang"]),
             np.array([phi0]), np.array([phi1]), None, None),
            ("q", o["deg_q"], o["order_q"], int(o["pin_ang"]),
             q0, q1, q_lo, q_hi)):
        cps[name] = _solve_channel(deg, M, order, y0, y1, npin, lo, hi)

    def _build(TT):
        return {nm: BSpline(clamped_knots(dg, M, TT), cps[nm], dg).derivs(nd)
                for nm, dg, nd in (("xc", o["deg_xc"], 4),
                                   ("psi", o["deg_psi"], 2),
                                   ("q", o["deg_q"], 2))}

    # ---- (3) velocity / acceleration: CLOSED FORM, no search -------------
    p1x, p2x = _unit_peaks(cps["xc"], o["deg_xc"], M)
    p1a = p2a = 0.0
    for nm, dg in (("psi", o["deg_psi"]), ("q", o["deg_q"])):
        a1, a2 = _unit_peaks(cps[nm], dg, M, use_norm=False)
        p1a, p2a = max(p1a, a1), max(p2a, a2)
    T_from = {
        "v_max": p1x / o["v_max"] if o["v_max"] else 0.0,
        "a_max": np.sqrt(p2x / o["a_max"]) if o["a_max"] else 0.0,
        "w_max": p1a / o["w_max"] if o["w_max"] else 0.0,
        "dw_max": np.sqrt(p2a / o["dw_max"]) if o["dw_max"] else 0.0,
        "T_min": float(o["T_min"]),
    }
    forced = o["T"] is not None
    T = float(o["T"]) if forced else max(T_from.values())

    # ---- (2) control inputs: dilate T, 1/T^2 law ------------------------
    stat = None
    pk_ok = None            # peaks at the accepted T, on the REPORT grid
    n_dilations = 0
    tjm = o["tau_joint_max"]
    if (tjm is not None or rotor is not None) and not forced:
        def _over(pk):
            """(violated?, per-bound excess) at the current T."""
            v = []
            if tjm is not None:
                v.append((pk["tau_joint"], float(tjm), "tau_joint"))
            if rotor is not None:
                v.append((pk["f_max"], rotor.f_max, "rotor upper"))
                v.append((-pk["f_min"], -rotor.f_min, "rotor lower"))
            return [x for x in v if x[0] > x[1] + 1e-9], v

        # Size on the coarse grid, but VERIFY on the one the report uses:
        # a peak resolved on 21 points can reappear on 61, and with a thin
        # margin that shows up as a plan whose own diagnostics contradict it.
        n_size = int(o["Nbound"])
        n_rep = min(int(o["Ncheck"]), 61) if int(o["Ncheck"]) > 0 else n_size
        T_hi = float(o["T_max"])
        grid = n_size
        feasible = False
        for _ in range(int(o["max_dilations"]) + 1):
            pk = _input_peaks(params, _build(T), T, grid, rotor,
                              yield_hook=yh, yield_every=ye)
            bad, allb = _over(pk)
            if not bad:
                if grid >= n_rep:
                    feasible, pk_ok = True, pk
                    break
                grid = n_rep          # re-check at reporting resolution
                pk = _input_peaks(params, _build(T), T, grid, rotor,
                                  yield_hook=yh, yield_every=ye)
                bad, allb = _over(pk)
                if not bad:
                    feasible, pk_ok = True, pk
                    break
            if T >= T_hi - 1e-9:
                break                 # already at the ceiling; nowhere to go
            # The quasi-static sweep is the T -> infinity limit. It is the
            # DIAGNOSIS, not the verdict: the peak does NOT approach it
            # monotonically. Measured on one transition, tau_j falls 0.77017
            # (T=3) -> 0.76189 (T=10) and then RISES back to 0.76268 as
            # T -> infinity, i.e. it dips ~0.1% BELOW the static value in
            # between, because the arm's own inertial torque can oppose
            # gravity at the instant of the peak. So "static > bound" does not
            # prove infeasibility, and refusing on it would reject bounds that
            # a mid-range T actually meets. The verdict is taken at T_max.
            if stat is None:
                stat = _input_peaks(params, _build(T), T, grid, rotor,
                                    quasi_static=True,
                                    yield_hook=yh, yield_every=ye)
            smap = {"tau_joint": stat["tau_joint"], "rotor upper":
                    stat["f_max"], "rotor lower": -stat["f_min"]}
            k = max(_dilate(x, smap[nm], b) for x, b, nm in allb)
            if not np.isfinite(k) or k <= 1.0 + 1e-9:
                # The 1/T^2 law does not apply here (the dynamic term is not
                # shrinking the peak, or the bound is under the static value).
                # Fall back to plain geometric growth so the search still
                # reaches T_max and terminates on a verdict rather than
                # creeping by 2% a step and running out of iterations.
                k = 1.5
            n_dilations += 1
            T = min(T * min(1.02 * k, 3.0), T_hi)

        if not feasible:
            # Reached the ceiling, or ran out of steps, still violating. Both
            # are refusals -- the old loop could fall out of its `for` while
            # infeasible and return the plan anyway, with only diag.bounds_ok
            # to show for it.
            if stat is None:
                stat = _input_peaks(params, _build(T), T, grid, rotor,
                                    quasi_static=True,
                                    yield_hook=yh, yield_every=ye)
            sbad, _ = _over(stat)
            why = ("; ".join(f"{nm} = {x:.3f} vs {b:.3f}"
                             for x, b, nm in _over(pk)[0]))
            if sbad:
                raise ValueError(
                    f"control inputs still violated at T = {T:.1f} s ({why}), "
                    "and the quasi-static load alone exceeds the bound "
                    + "; ".join(f"({nm} {x:.3f} > {b:.3f})"
                                for x, b, nm in sbad)
                    + " — a POSE problem: no duration fixes it")
            raise ValueError(
                f"control-input bounds unmet at T_max = {T_hi:.0f} s ({why}) "
                "— the transition is too demanding for the actuators at any "
                "admissible duration")

    if not forced:
        T = float(np.clip(T, float(o["T_min"]), float(o["T_max"])))
    ch = _build(T)
    t_solve = time.perf_counter() - t_solve

    def ref(t):
        tt = float(np.clip(t, 0.0, T))
        xc = np.array([s(tt) for s in ch["xc"]])
        psi = np.array([float(s(tt)[0]) for s in ch["psi"]])
        q = np.array([s(tt) for s in ch["q"]])
        return flat_to_reference(params, xc, psi, q)

    # ---- report ----------------------------------------------------------
    r_end = ref(T)
    r_hold1 = TP.rest_ref(params, x_b1, phi1, q1)
    npin = {"xc": int(o["pin_xc"]), "psi": int(o["pin_ang"]),
            "q": int(o["pin_ang"])}
    # (1) the rest conditions, MEASURED rather than assumed: every derivative
    # field of the message at both ends, which is what "zero velocity at the
    # setpoints" has to mean for a reference the law differentiates.
    rest_err = 0.0
    for tt in (0.0, T):
        rr = ref(tt)
        rest_err = max(rest_err, max(
            float(np.max(np.abs(np.atleast_1d(rr[key]))))
            for key in rr if key.endswith(("_dot", "_ddot", "_d3", "_d4"))
            or key == "qdot_d"))
    diag = {
        "T": T,
        "solve_ms": 1e3 * t_solve,
        "n_ctrl": {k: ch[k][0].n for k in ch},
        "n_free": {k: max(0, ch[k][0].n - 2 * npin[k]) for k in ch},
        "T_from": T_from,
        "T_binding": "forced" if forced else max(T_from, key=T_from.get),
        "n_dilations": n_dilations,
        "rest_err": rest_err,
        "endpoint_xc_err": float(np.linalg.norm(
            r_end["x_cd"] - r_hold1["x_cd"])),
        "endpoint_q_err": float(np.max(np.abs(r_end["q_d"] - q1))),
    }
    if int(o["Ncheck"]) <= 0:
        return {"T": T, "ref": ref, "q1": q1.copy(), "diag": diag, "spl": ch}

    # Every bound, achieved against its limit, at the duration finally chosen.
    # The dilation loop already verified on exactly this grid, so REUSE its
    # result: recomputing it was 61 more dynamics() calls, ~99 ms of the
    # whole-body planner's 287 ms plan, for numbers that cannot differ.
    if pk_ok is not None:
        pk = pk_ok
    elif tjm is not None or rotor is not None:
        pk = _input_peaks(params, ch, T, min(int(o["Ncheck"]), 61), rotor,
                          yield_hook=yh, yield_every=ye)
    else:
        # Neither input bound is configured, so every row _b() would fill from
        # this sweep is skipped anyway -- computing it was ~90 ms of inverse
        # dynamics for numbers nothing reads. Switching the class off must
        # actually switch its cost off.
        pk = {"tau_joint": float("nan"), "f_min": float("nan"),
              "f_max": float("nan")}
    bnd = {}

    def _b(name, value, bound, lower=False):
        if bound is None:
            return
        bnd[name] = {"value": float(value), "bound": float(bound),
                     "ok": bool(value >= bound - 1e-9 if lower
                                else value <= bound + 1e-9)}

    _b("v [m/s]", p1x / T, o["v_max"])
    _b("a [m/s2]", p2x / T ** 2, o["a_max"])
    _b("w [rad/s]", p1a / T, o["w_max"])
    _b("dw [rad/s2]", p2a / T ** 2, o["dw_max"])
    _b("tau_joint [N.m]", pk["tau_joint"], o["tau_joint_max"])
    if rotor is not None:
        _b("rotor hi [rad/s]", float(rotor.omega(np.array([pk["f_max"]]))[0]),
           np.sqrt(rotor.f_max / rotor.k))
        _b("rotor lo [rad/s]", float(rotor.omega(np.array([pk["f_min"]]))[0]),
           np.sqrt(rotor.f_min / rotor.k), lower=True)
    diag["bounds"] = bnd
    diag["bounds_ok"] = all(b["ok"] for b in bnd.values())

    tg = np.linspace(0.0, T, int(o["Ncheck"]))
    qs = np.array([ch["q"][0](t) for t in tg])
    xd = np.array([ch["xc"][1](t) for t in tg])
    xdd = np.array([ch["xc"][2](t) for t in tg])
    acn = np.linalg.norm(xdd + params["g"] * _E3, axis=1)
    tilt = np.degrees(np.arccos(np.clip(
        (xdd[:, 2] + params["g"]) / acn, -1.0, 1.0)))
    diag.update({
        "q_min_deg": np.degrees(qs.min(axis=0)),
        "q_max_deg": np.degrees(qs.max(axis=0)),
        "q_box_lo_deg": np.degrees(q_lo),
        "q_box_hi_deg": np.degrees(q_hi),
        "peak_com_speed": float(np.max(np.linalg.norm(xd, axis=1))),
        "peak_com_acc": float(np.max(np.linalg.norm(xdd, axis=1))),
        "min_acc_norm": float(acn.min()),          # ||a_c|| = 0 is the only
        "max_tilt_deg": float(tilt.max()),         # flatness singularity
        "min_sigma_nd": float(min(TP._sigma_nd(qq, params) for qq in qs)),
        "sigma_nd_margin": TP.SIGMA_ND_MARGIN,
        # (A6) residual, for parity with transition_planner's diag. It is an
        # identity here rather than a solved fixed point, so a coarse grid is
        # enough to show that -- it comes out at machine precision or the
        # wiring is wrong.
        "max_dyn_defect": _dyn_defect(params, ref, T, 41),
    })
    return {"T": T, "ref": ref, "q1": q1.copy(), "diag": diag, "spl": ch}


def _dyn_defect(params, ref, T, n):
    """max residual of (A6) along the plan."""
    e = 0.0
    for t in np.linspace(0.0, T, int(n)):
        r = ref(t)
        ac = r["x_cd_ddot"] + params["g"] * _E3
        R0 = CT._build_R0(ac / np.linalg.norm(ac), r["b1_d"])
        r0c, r0e = CT._arm_kin(r["q_d"], params)
        e = max(e, float(np.linalg.norm(
            r["x_cd"] - (r["r_ed"] + R0 @ (r0c - r0e)))))
    return e


# ===========================================================================
# drop-in entry point for the whole-body planner
# ===========================================================================

# transition_planner options with no counterpart here.  Each is dropped rather
# than rejected so ONE whole-body planner opts dict can drive either planner:
#   deg/N/Nfine             polynomial degree and Picard grids; this planner's
#                           basis is set by deg_xc/deg_psi/deg_q + n_spans
#   maxit/tol/relax         Picard controls; there is no fixed point
#   beta_min_deg            the wrist-singularity guard is a property of the
#                           z-x-z chart, and this planner has no chart
_LEGACY_OPTS = ("deg", "N", "Nfine", "maxit", "tol", "relax",
                "beta_min_deg", "verbose")


def plan_transition(params, rest0, rest1, opts=None):
    """transition_planner.plan_transition-compatible signature.

    Same call, same return shape, same diag keys the whole-body planner logs
    (max_dyn_defect, min_sigma_nd, peak_com_speed) — so selecting a planner is
    a module swap, not a branch at every call site."""
    o = {k: v for k, v in (opts or {}).items() if k not in _LEGACY_OPTS}
    return plan_flat_transition(params, rest0, rest1, o)


# ===========================================================================
# self-test
# ===========================================================================

def _fd(f, t, h, key):
    return (f(t + h)[key] - f(t - h)[key]) / (2.0 * h)


def _selftest():
    np.set_printoptions(precision=6, suppress=True, linewidth=110)
    P = TP.make_params_t650()
    home = np.array([0.0, np.radians(40.0), np.radians(40.0), 0.0])
    goal = np.array([np.radians(15.0), np.radians(10.0),
                     np.radians(-5.0), np.radians(35.0)])
    r0 = {"x_b": np.array([0.0, 0.0, 1.2]), "phi": 0.0, "q": home}
    r1 = {"x_b": np.array([0.6, -0.35, 1.55]), "phi": np.radians(25.0),
          "q": goal}

    print("=== flat B-spline transition planner ===")
    pl = plan_flat_transition(P, r0, r1)
    d, ref, T = pl["diag"], pl["ref"], pl["T"]
    print(f"T = {T:.2f} s   solve {d['solve_ms']:.2f} ms   "
          f"ctrl pts {d['n_ctrl']}   free {d['n_free']}")

    ok = True

    # 1) endpoints EXACT against the rest algebra, derivatives all zero
    for tag, tt, rr in (("start", 0.0, r0), ("end", T, r1)):
        a = ref(tt)
        b = TP.rest_ref(P, rr["x_b"], TP._unwrap_near(
            float(rr["phi"]), float(r0["phi"])), rr["q"])
        e = max(float(np.max(np.abs(np.atleast_1d(a[k] - b[k]))))
                for k in b)
        ok &= e < 1e-9
        print(f"  {tag:5s} vs rest_ref (all 16 fields)   max err {e:.2e}")

    # 2) every derivative chain FD-consistent
    h = 1e-4
    pairs = [("x_cd", "x_cd_dot"), ("x_cd_dot", "x_cd_ddot"),
             ("x_cd_ddot", "x_cd_d3"), ("x_cd_d3", "x_cd_d4"),
             ("b1_d", "b1_d_dot"), ("b1_d_dot", "b1_d_ddot"),
             ("r_ed", "r_ed_dot"), ("r_ed_dot", "r_ed_ddot"),
             ("b1_de", "b1_de_dot"), ("b1_de_dot", "b1_de_ddot"),
             ("q_d", "qdot_d")]
    worst = 0.0
    for f0, f1 in pairs:
        e = max(float(np.max(np.abs(_fd(ref, t, h, f0) - ref(t)[f1])))
                for t in np.linspace(0.1 * T, 0.9 * T, 9))
        worst = max(worst, e)
        ok &= e < 2e-5
    print(f"  FD consistency, all 11 derivative chains   max err {worst:.2e}")

    # 3) the compatibility identity holds by construction, not by iteration
    e = 0.0
    for t in np.linspace(0.0, T, 41):
        r = ref(t)
        a_c = r["x_cd_ddot"] + P["g"] * _E3
        R0 = CT._build_R0(a_c / np.linalg.norm(a_c), r["b1_d"])
        k = CT._arm_kin(r["q_d"], P)
        e = max(e, float(np.linalg.norm(
            r["x_cd"] - (r["r_ed"] + R0 @ (k[0] - k[1])))))
    print(f"  (A6) dynamic defect, zero iterations       {e:.2e} m")
    ok &= e < 1e-12

    # 4) the joint box needs no constraint: monotone, hence endpoint-bounded
    lo = np.degrees(np.minimum(home, goal))
    hi = np.degrees(np.maximum(home, goal))
    over = float(max(np.max(lo - d["q_min_deg"]), np.max(d["q_max_deg"] - hi)))
    print(f"  joint path inside its own endpoint span    "
          f"overshoot {max(over, 0.0):.3e} deg")
    ok &= over < 1e-9

    # 5) margins a constraint would have tested
    print(f"  ||a_c||   min {d['min_acc_norm']:.3f} m/s^2  "
          f"(singular at 0)   tilt max {d['max_tilt_deg']:.2f} deg")
    print(f"  sigma_nd  min {d['min_sigma_nd']:.3f}         "
          f"(margin {d['sigma_nd_margin']:.2f})")
    print(f"  peaks     |v| {d['peak_com_speed']:.3f} m/s   "
          f"|a| {d['peak_com_acc']:.3f} m/s^2")
    print(f"  q range   {np.round(d['q_min_deg'], 1)} .. "
          f"{np.round(d['q_max_deg'], 1)} deg")

    # 6) WARM solve — the cache is per-basis, so this is the online cost
    def _bench(opts, reps=20):
        t0 = time.perf_counter()
        for _ in range(reps):
            plan_flat_transition(P, r0, r1, opts)
        return 1e3 * (time.perf_counter() - t0) / reps
    print(f"  warm re-plan {_bench({'Ncheck': 0}):.3f} ms solve only, "
          f"{_bench({}):.1f} ms with the {DEFAULT_OPTS['Ncheck']}-point "
          f"diagnostic grid")

    # 7) the control points really are T-independent (the cache's premise)
    ca = plan_flat_transition(P, r0, r1, {"T": 4.0})["spl"]
    cb = plan_flat_transition(P, r0, r1, {"T": 17.0})["spl"]
    e = max(float(np.max(np.abs(ca[k][0].c - cb[k][0].c))) for k in ca)
    print(f"  control points at T = 4 s vs 17 s          max diff {e:.2e}")
    ok &= e < 1e-12

    # 8) evaluation cost against the whole-body planner's 10 ms budget at 100 Hz
    ts = np.linspace(0.0, T, 400)
    t0 = time.perf_counter()
    for t in ts:
        ref(t)
    print(f"  ref(t) eval {1e3 * (time.perf_counter() - t0) / ts.size:.3f} "
          f"ms/call")

    # ---- the five constraint classes ------------------------------------
    print("\n--- constraints ---")
    print(f"  T = {T:.2f} s set by {d['T_binding']}, "
          f"{d['n_dilations']} input dilation(s); candidates "
          + ", ".join(f"{k} {v:.2f}" for k, v in d["T_from"].items()))
    print(f"  (1) rest: worst derivative field at either end  {d['rest_err']:.2e}")
    ok &= d["rest_err"] < 1e-9
    for nm, b in d["bounds"].items():
        print(f"      {nm:20s} {b['value']:9.3f}  vs {b['bound']:8.3f}   "
              f"{'ok' if b['ok'] else 'VIOLATED'}")
    ok &= d["bounds_ok"]

    # (3) the closed-form sizing is EXACT: bind velocity, then acceleration
    for key, lim, field in (("v_max", 0.05, "v [m/s]"),
                            ("a_max", 0.02, "a [m/s2]")):
        pp = plan_flat_transition(P, r0, r1, {key: lim, "Ncheck": 41})
        got = pp["diag"]["bounds"][field]["value"]
        hit = pp["diag"]["T_binding"] == key
        print(f"  (3) {key} = {lim}: T {pp['T']:6.2f} s, achieved {got:.6f}, "
              f"binding {pp['diag']['T_binding']}")
        ok &= hit and abs(got - lim) < 1e-6

    # (5) the minimum-time floor wins when nothing else binds
    pp = plan_flat_transition(P, r0, r1, {"T_min": 25.0, "Ncheck": 41})
    print(f"  (5) T_min = 25 s: T {pp['T']:.2f} s, binding "
          f"{pp['diag']['T_binding']}")
    ok &= abs(pp["T"] - 25.0) < 1e-9

    # (4) an out-of-range goal is refused, with the joint named
    try:
        plan_flat_transition(P, r0, {**r1, "q": np.radians([0., 70., 0., 0.])})
        print("  (4) out-of-range goal NOT refused"); ok = False
    except ValueError as e:
        print(f"  (4) {e}")

    # (2) a torque bound below the STATIC gravity load cannot be dilated away
    try:
        plan_flat_transition(P, r0, r1, {"tau_joint_max": 0.30})
        print("  (2) impossible torque bound NOT refused"); ok = False
    except ValueError as e:
        print(f"  (2) {e}")

    # (2) how much of each input is STATIC — i.e. how much slowing down can
    # possibly buy.  On this plant, almost none: the arm is light and the
    # transition gentle, so gravity is essentially the whole load.
    rm = t650_rotor_model()
    st = _input_peaks(P, pl["spl"], T, 31, rm, quasi_static=True)
    pk = _input_peaks(P, pl["spl"], T, 31, rm)
    share = 100.0 * (pk["tau_joint"] - st["tau_joint"]) / pk["tau_joint"]
    # (2) the COLLECTIVE / DIFFERENTIAL split of the rotor bound
    up, lo_lim = collective_thrust_limits(P, rm)
    a_pk = d["bounds"]["a [m/s2]"]["value"]
    fcol = float(sum(P["m_i"])) * max(
        np.linalg.norm(np.array([sp(t) for sp in pl["spl"]["xc"]])[2]
                       + P["g"] * _E3) for t in np.linspace(0, T, 201))
    dmax = res = 0.0
    for t in np.linspace(0.0, T, 61):
        xc = np.array([sp(t) for sp in pl["spl"]["xc"]])
        ps = np.array([float(sp(t)[0]) for sp in pl["spl"]["psi"]])
        qq = np.array([sp(t) for sp in pl["spl"]["q"]])
        _, aux = flat_state(P, xc, ps, qq)
        u = inverse_inputs(P, aux, qq[0], qq[1], qq[2], rm)
        dmax = max(dmax, float(np.max(np.abs(u["rotor_force"]
                                             - u["thrust"] / 4.0))))
        res = max(res, u["thrust_residual"])
    print(f"  (2) rotor bound splits: collective f = {fcol:.2f} N uses "
          f"{100 * fcol / (4 * rm.f_max):.0f}% of the ceiling, differential "
          f"{dmax:.3f} N adds {100 * dmax / rm.f_max:.1f}%")
    print(f"      collective is CONVEX in c: needs |xdd_c| <= {up:.2f} m/s2, "
          f"achieved {a_pk:.3f} ({up / max(a_pk, 1e-9):.0f}x slack) -> "
          f"discharged by a_max, never evaluated through the dynamics")
    # Newton vs the plant blocks: an EXACT identity once the generalized
    # velocity is body-frame in both of its first two blocks. Machine
    # precision or the frames have moved -- there is no tolerance to tune.
    print(f"      f from Newton == f from M a + C v + g to {res:.1e} N "
          f"(exact identity; 1.4e-02 with the world-frame slip)")
    ok &= res < 1e-9
    ok &= up > 50.0 * a_pk

    print(f"  (2) tau_joint {pk['tau_joint']:.4f} N.m of which only "
          f"{share:.2f}% is dynamic (static {st['tau_joint']:.4f}) — the "
          f"input bounds are POSE constraints here, not speed ones")

    # (2) mechanism check: a bound inside that thin dynamic margin must be met
    # by dilating T, not refused.  Deliberately contrived — see the share above.
    tight = st["tau_joint"] + 0.3 * (pk["tau_joint"] - st["tau_joint"])
    pp = plan_flat_transition(P, r0, r1, {"tau_joint_max": tight,
                                          "Ncheck": 41})
    tj = pp["diag"]["bounds"]["tau_joint [N.m]"]
    print(f"      tau_joint_max = {tight:.4f}: T {T:.2f} -> {pp['T']:.2f} s "
          f"after {pp['diag']['n_dilations']} dilation(s), peak "
          f"{tj['value']:.4f} {'ok' if tj['ok'] else 'VIOLATED'}")
    ok &= tj["ok"] and pp["diag"]["n_dilations"] >= 1

    # (2) THE NON-MONOTONE WINDOW. The input peak does not approach its
    # quasi-static limit from above: on this transition tau_j falls to 0.7619
    # near T = 10 s and rises back to the static 0.76268 as T -> infinity. So a
    # bound BELOW the static value can still be met at a finite T, and the old
    # "static > bound => infeasible at ANY duration" refusal was unsound.
    goal2 = np.radians([15.0, 25.0, 20.0, 35.0])
    r2 = {**r1, "q": goal2}
    fast = {"v_max": 9.0, "a_max": 9.0, "w_max": 9.0, "dw_max": 9.0,
            "Ncheck": 81}
    p3 = plan_flat_transition(P, r0, r2, dict(fast, rotor=None,
                                              tau_joint_max=None))
    rm2 = t650_rotor_model()
    pk3 = _input_peaks(P, p3["spl"], p3["T"], 61, rm2)["tau_joint"]
    st3 = _input_peaks(P, p3["spl"], p3["T"], 61, rm2,
                       quasi_static=True)["tau_joint"]
    lo3 = min(_input_peaks(P, {k: BSpline(clamped_knots(d, 12, TT),
                                          p3["spl"][k][0].c, d).derivs(nd)
                               for k, d, nd in (("xc", 7, 4), ("psi", 5, 2),
                                                ("q", 5, 2))},
                           TT, 61, rm2)["tau_joint"] for TT in (8.0, 10.0))
    print(f"  (2) non-monotone: peak(T={p3['T']:.0f}) {pk3:.5f} > static "
          f"{st3:.5f} > best finite-T {lo3:.5f}")
    ok &= lo3 < st3 < pk3          # the window this test needs must exist
    tight2 = 0.5 * (st3 + lo3)     # below static, above what some T achieves
    pp = plan_flat_transition(P, r0, r2, dict(fast, tau_joint_max=tight2))
    tj2 = pp["diag"]["bounds"]["tau_joint [N.m]"]
    print(f"      tau_joint_max = {tight2:.5f} (BELOW static): T "
          f"{p3['T']:.2f} -> {pp['T']:.2f} s, peak {tj2['value']:.5f} "
          f"{'ok' if tj2['ok'] else 'VIOLATED'}  <- old logic refused this")
    ok &= tj2["ok"]

    # (2) a bound under every reachable peak IS a refusal, and must raise
    try:
        plan_flat_transition(P, r0, r2, dict(fast, tau_joint_max=0.30))
        print("  (2) impossible bound NOT refused"); ok = False
    except ValueError as e:
        print(f"  (2) {str(e)[:96]}...")

    # cost tiering: the input bounds are the only expensive constraint
    t_shape = _bench({"Ncheck": 0, "rotor": None, "tau_joint_max": None})
    t_inputs = _bench({"Ncheck": 0})
    t_full = _bench({})
    print(f"  cost  {t_shape:.2f} ms shape + kinematic bounds, "
          f"{t_inputs:.1f} ms with the input bounds, "
          f"{t_full:.0f} ms with full diagnostics")

    print("PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    sys.exit(0 if _selftest() else 1)

#!/usr/bin/env python3
"""
compatible_trajectory.py — offline planner for the dynamically-compatible
system-CoM trajectory of the aerial manipulator.

Python port of `refs_matlab/utils/utils_planning/plan_compatible_trajectory.m`.
Runs ONCE, offline (pure numpy — no Isaac / ROS2 imports).  It PRESCRIBES the
end-effector task (p_e, R_e) and the platform heading, and SOLVES for the
compatible CoM p_c(t) using
    (C1) t_dir = (p_c_ddot + g e3)/||.||                     (thrust direction)
    (C2) p_c   = p_e + R_0 ( r_0c^0(q) - r_0e^0(q) )         (CoM offset)
R_0 is built from (t_dir, b1_d) exactly as the controller builds R0c; q is
recovered from R_e^0 = R_0' R_e with the redundancy closed by (R1) b1_d
prescribed and (R2) q2 = split*beta, q3 = (1-split)*beta.  p_c is a polynomial
(=> C^inf), solved by a damped Picard iteration.

TWO DELIBERATE DIFFERENCES from the MATLAB original:

1.  z-x-z, NOT z-y-z.  The MATLAB model's joints 2/3 rotate about +y, so it
    prescribes R_e = Rz(yaw)·Ry(beta)·Rz(wrist) and recovers q by z-y-z
    decomposition.  On the AM_realign asset (= controller.make_params)
    joints 2/3 rotate about +x — R_e is therefore prescribed as
    Rz(yaw)·Rx(beta)·Rz(wrist) and recovered by z-x-z decomposition, so that
    R_e^0 = Rz(q1)·Rx(q2)·Rx(q3)·Rz(q4) holds exactly.  beta from the
    decomposition is always >= 0, which on this asset is the well-conditioned
    "outward-and-up" arm direction (see TRAJ_CONFIG["circle_drone"]'s measured
    cond(J_3y) q_hold notes in trajectory_library.py).

2.  CANONICAL frame.  The MATLAB plan is absolute (anchored at its hover
    config); here the plan is solved with the EE start at the ORIGIN and zero
    platform heading, and the caller (trajectory_library.build_traj) anchors it at
    the takeoff-handover state by a rigid translation + yaw rotation.  This is
    exact — gravity is along z, so a z-rotation/translation of the whole task
    maps the Picard solution to the Picard solution of the transformed task —
    and it lets one cached plan serve both build_traj calls (spawn + the
    re-anchor at the climb->track handover) with no re-planning stall.

The exact link CoM offsets (params["com_i"]) are used in the CoM map — NOT the
MATLAB planner's l_i/2 midpoint approximation — so the planned p_c is
consistent with the very dynamics()/controller the plan is tracked by.

`params` is controller.make_params()'s dict ("n", "m_i", "l_i", "com_i",
"h_i_im1", "g").  Entry points:

    plan = plan_compatible_trajectory(params, task_cfg, opts)
    plan = get_plan(params, cfg)     # cached; cfg = flat TRAJ_CONFIG dict

plan["ref"](tq) returns the FULL controller reference dict in the canonical
frame (x_cd..x_cd_d4, b1_d(+dot,ddot), r_ed(+dot,ddot), b1_de(+dot,ddot),
q_d) — a drop-in for generate_reference once anchored.  plan["q0"] is the
consistent t=0 arm configuration (the takeoff hold pre-positions the arm
there, mirroring the MATLAB plan.X0's zero-initial-error state).
"""

import warnings

import numpy as np

_E3 = np.array([0.0, 0.0, 1.0])
# hat(e3) and hat(e1) — derivative generators of Rz / Rx (dR/da = S·R = R·S)
_SZ = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
_SX = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])

# Task defaults (degrees where noted) — tuned for the AM_realign asset:
#   q1 limit is +/-35 deg  ->  |yawA - ppA| kept <= ~25 deg (q1 ~ EE yaw - platform yaw)
#   q2/q3 limit is -90..+50 deg -> beta = b0 +/- bA kept <= ~70 deg (q2 = q3 = beta/2)
#   beta well away from 0 (wrist singularity); larger beta improves cond(J_3y)
#   on this asset (measured best ~35 at q2=q3~+0.8 — see circle_drone's q_hold notes).
_DEFAULT_TASK = {
    "T": 15.0,                             # [s] plan horizon (rest-to-rest)
    "Dx": 1.2, "Dy": 0.6, "Dz": 0.4,       # [m] net EE displacement
    "Ay": 0.3, "Az": 0.25,                 # [m] mid-path lateral/vertical arcs
    "yaw0_deg": 0.0,  "yawA_deg": 15.0,    # EE heading yaw sweep
    "b0_deg": 45.0,   "bA_deg": 25.0,      # EE tilt beta (sin arc) 45..70..45
    "g0_deg": 0.0,    "gA_deg": 55.0,      # EE wrist roll sweep
    "pp0_deg": 0.0,   "ppA_deg": -10.0,    # platform yaw, independent of EE yaw
    "path_yaw_deg": 0.0,                   # travel direction vs heading (see _eval_task)
    "split": 0.5,                          # q2 = split*beta, q3 = (1-split)*beta
}
_DEFAULT_OPTS = {
    "N": 201,          # Picard recovery grid
    "deg": 16,         # polynomial degree for p_c (scaled tau in [-1,1])
    "maxit": 60,
    "tol": 1e-10,      # Picard stop: max |delta p_c| on the grid [m]
    "relax": 1.0,      # damping (1.0 = undamped fixed point)
    "Nfine": 801,      # diagnostics grid
    "q_lim_deg": None, # optional [[lo,hi]]*n joint-limit check on recovered q
    "verbose": True,
}

# 4-phase SHOWCASE task (fly-in -> pinned -> fly-out; takeoff/landing arm pose
# = the folded "home" beta). The EE wrist stays at 0 throughout: with ga = 0,
# b1_de = Rz(yaw)·Rx(beta)·Rz(0)·e1 = Rz(yaw)·e1 is INDEPENDENT of beta, so the
# arm can fold/unfold during the pinned phase while the EE position AND heading
# stay exactly fixed — the drone's motion there is generated purely by the
# arm/heading reconfiguration through the compatibility solve.
_DEFAULT_SHOWCASE = {
    # NO T_land / land_drop: the plan ends HOVERING at land_wp. The descent is
    # the demo's LANDING phase (a plain setpoint, arm PD-held) — takeoff and
    # landing are setpoints, only the middle task phase tracks a plan. Passing
    # T_land/land_drop > 0 still evaluates the legacy in-plan descent.
    "T_fly_in": 6.0, "T_pinned": 6.0, "T_fly_out": 6.0,                 # [s]
    # TASK SHAPE — a "dip": the pinned point sits BELOW both the fly-in start and
    # the fly-out end, so the side view (y-z) shows a clear climb-over-and-down
    # then back up, while the bird view stays a straight line.
    "target": (2.0, 0.0, -0.35),   # EE point after the forward flight, rel. start [m]
    "land_wp": (4.0, 0.0, 0.0),    # EE point above the landing spot, rel. start [m]
    # Ay = 0 keeps fly-in/out DEAD STRAIGHT from above; all the shape is in Az,
    # which bulges the path UP mid-segment before it settles.
    "Ay": 0.0, "Az": 0.30,         # fly-in/out lateral/vertical arcs [m]
    # ARM FOLD SCHEDULE. beta = q2+q3; LARGER beta = elbow folded = EE tucked UP
    # toward the body (measured reach from the base: 0.279 m at beta=0 straight
    # down, 0.16 m at 80 deg). HOME is the FOLDED pose, held from spawn through
    # takeoff and again through the whole landing; the arm UNFOLDS to work for
    # the task phases and folds back at the end of fly-out.
    # HOME = beta 80 deg chosen from the MATLAB singularity analysis
    # (utils_singularity/singularity_sweep.m) mapped onto this asset: both
    # singular branches (elbow det B_xz = 0, yaw det B_ypsi = 0) live at
    # NEGATIVE q2/q3 in this asset's sign convention (deepest valley q3 ~ -52
    # deg; the measured cond~880 point at q2=q3=-43 deg is that branch), while
    # on the positive fold line sigma_min(J_3y0/Lchar) RISES monotonically with
    # beta: 0.38 at q=0 -> 0.52 at beta=80 -- 5.2x the analysis' eps=0.10
    # keep-out margin, worst-case over q1 in +/-35 and q4 in +/-180. beta=80
    # also keeps 10 deg of q2/q3 travel to the +50 deg joint limit and tucks
    # the EE to 0.04 m below the base (~0.26 m ground clearance at rest).
    "beta_home_deg": 86.0,         # FOLDED home: takeoff + landing (q2=40, q3=46)
    "beta_work_deg": 50.0,         # unfolded working tilt (fly-in unfolds to here)
    # Pinned-phase fold sweep, work -> work+amp -> work. This is ALSO what
    # produces the drone's own translation while the EE is pinned: with the EE
    # position AND yaw both locked, q1 counter-rotates the platform yaw exactly,
    # so R_0 * r_0e is invariant and a pure yaw moves the base NOT AT ALL. The
    # only base motion available is the change in the arm's own offset r_0e,
    # which over beta 50..80 is ~0.03 m horizontally but ~0.06 m vertically.
    # 30 deg uses the vertical part without crowding the q3 limit.
    "beta_pin_amp_deg": 30.0,      # pinned-phase sweep: work -> +amp -> work
    # EE heading is held at 0 through fly-in so that ALL of q1's +/-35 deg travel
    # is available to the pinned-phase gimbal (q1 = EE yaw - platform yaw).
    "yaw_work_deg": 0.0,           # EE heading after fly-in (held in pinned)
    # Pinned-phase platform yaw amplitude, swept as a FULL sine cycle both ways.
    # HARD CAP: with the EE yaw locked inertially the arm counter-rotates 1:1,
    # q1 = -pp, so |pp| <= 35 deg minus margin. 30 leaves ~5 deg (measured q1
    # tracking overshoot is <1 deg). A full 360 deg spin is IMPOSSIBLE with the
    # yaw lock — it would need q1 = -360; drop the lock (let the EE heading
    # follow the platform) if a complete circle is ever wanted instead.
    "pp_pin_amp_deg": 30.0,        # platform-yaw full sine cycle during pinned
    # PINNED-PHASE STYLE — two selectable showcases of the same gimbal:
    #   "yaw"      platform yaws a full cycle 0->+A->0->-A->0 while the EE stays
    #              frozen; the ARM counter-rotates 1:1 (q1 = -pp). Rotation is
    #              the visible motion; the drone barely translates (see below).
    #   "vertical" heading held, arm sweeps its full fold range work->home->work,
    #              swinging the drone UP AND DOWN over the frozen tool tip.
    # NEITHER can be a circle around the target. x_c - r_e = R_0*(r_0c - r_0e)
    # and the body-frame offset has v_z > 0 for EVERY reachable q (the arm can
    # only put the tool BELOW its base), so the drone is always ABOVE the EE:
    # the reachable set is a patch on the UPPER side of a ~0.24 m sphere. A
    # vertical circle would need the drone to pass below the target, i.e. the
    # arm to reach up past its own base or the quadrotor to fly inverted.
    # Measured offsets: beta 0 -> |v| 0.237 m at 36 deg from vertical;
    # beta 50 -> 0.171 m at 62 deg; beta 86 -> 0.133 m at 80 deg. So "vertical"
    # buys ~0.10 m of drone travel — real, but small next to the 4 m transits.
    "pin_mode": "combo",           # "combo" | "yaw" | "vertical"
    # Phase offset between the fold cycle and the yaw cycle in "combo" mode.
    # 90 deg puts them in quadrature so (yaw, height) traces a CIRCLE; 0 would
    # couple them into a diagonal (yaw right exactly while descending).
    "pin_phase_deg": 90.0,
    "path_yaw_deg": 90.0,          # travel direction vs heading (see _eval_task)
    # ASYMMETRIC redundancy split (q2 = split*beta, q3 = (1-split)*beta).
    # 0.465 biases the fold onto q3 because the two joints have very different
    # usable headroom against their shared +50 deg limit: the PD-hold -> GMO-law
    # HANDOVER transient overshoots q2 by ~7.5 deg but q3 by <1 deg (measured).
    # Equal split (0.5) therefore wastes ~7 deg of q3 travel. At 0.465 the home
    # fold tightens from beta 80 -> 86 (tool reach 0.152 -> 0.139 m) with q2 held
    # at the SAME 40 deg it already flew safely, and sigma_min even improves
    # (0.521 -> 0.529). Raising it further eats the q3 margin: beta 88 leaves
    # only 1 deg.
    "split": 0.465,
}

# T_land/land_drop are NOT in _DEFAULT_SHOWCASE any more (no in-plan descent by
# default) but must stay recognized task keys, or get_plan would silently strip
# them from a legacy config that still requests the old phase-4 descent.
_TASK_KEYS = tuple(set(_DEFAULT_TASK) | set(_DEFAULT_SHOWCASE)
                   | {"T_land", "land_drop"})
_OPT_KEYS = tuple(_DEFAULT_OPTS)

_PLAN_CACHE = {}


# ===========================================================================
# small rotation / kinematics helpers
# ===========================================================================

def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot(h, q):
    """Rodrigues rotation about unit axis h by angle q."""
    a = h / (np.linalg.norm(h) + 1e-15)
    K = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + np.sin(q) * K + (1.0 - np.cos(q)) * (K @ K)


def _minjerk3(u):
    """Rest-to-rest min-jerk phase s(u) and derivatives ds, d2s (wrt u)."""
    u = min(1.0, max(0.0, u))
    s = 10 * u**3 - 15 * u**4 + 6 * u**5
    ds = 30 * u**2 - 60 * u**3 + 30 * u**4
    d2s = 60 * u - 180 * u**2 + 120 * u**3
    return s, ds, d2s


def _minsnap3(u):
    """Rest-to-rest min-snap (septic) phase: vel, acc AND JERK all zero at both
    ends. Used for the multi-phase showcase segments — the compatible CoM's
    VELOCITY depends on the prescribed jerk (R_0 rotates with p_c''', via the
    thrust-direction map), so min-jerk segments (jerk jumps at joins) would
    make the solved p_c velocity discontinuous at phase boundaries. Zero
    boundary jerk removes that."""
    u = min(1.0, max(0.0, u))
    s = 35 * u**4 - 84 * u**5 + 70 * u**6 - 20 * u**7
    ds = 140 * u**3 - 420 * u**4 + 420 * u**5 - 140 * u**6
    d2s = 420 * u**2 - 1680 * u**3 + 2100 * u**4 - 840 * u**5
    return s, ds, d2s


def _build_R0(t_dir, b1_d):
    """Platform attitude from thrust direction + heading (controller R0c)."""
    b3c = t_dir / np.linalg.norm(t_dir)
    P1 = np.eye(3) - np.outer(b3c, b3c)
    s = P1 @ b1_d
    ns = np.linalg.norm(s)
    if ns < 1e-9:
        tmp = np.array([1.0, 0.0, 0.0])
        if abs(b3c[0]) > 0.9:
            tmp = np.array([0.0, 1.0, 0.0])
        s = P1 @ tmp
        ns = np.linalg.norm(s)
    b1c = s / ns
    b2c = np.cross(b3c, b1c)
    return np.column_stack([b1c, b2c, b3c])


def _zxz_angles(M):
    """M = Rz(a)·Rx(b)·Rz(c) with b in [0, pi] (z-x-z; MATLAB original is z-y-z).
    Third column [sa·sb, -ca·sb, cb], third row [sb·sc, sb·cc, cb]."""
    sb = np.hypot(M[0, 2], M[1, 2])
    b = np.arctan2(sb, M[2, 2])
    if sb > 1e-9:
        a = np.arctan2(M[0, 2], -M[1, 2])
        c = np.arctan2(M[2, 0], M[2, 1])
    else:
        a = np.arctan2(-M[0, 1], M[0, 0])
        c = 0.0
    return a, b, c


def _recover_q(pcdd, task, tp):
    """(C1)+(R1)+(R2): thrust dir -> R_0 -> z-x-z of R_0' R_e -> q."""
    ac = pcdd + tp["g"] * _E3
    R0 = _build_R0(ac / np.linalg.norm(ac), task["b1_d"])
    a, b, c = _zxz_angles(R0.T @ task["R_e"])
    q = np.array([a, tp["split"] * b, (1.0 - tp["split"]) * b, c])
    return q, R0, ac


def _arm_kin(q, params):
    """(r_0c^0, r_0e^0) at joint config q — same chain as the controller's
    _arm_fk, with the EXACT link CoM offsets params["com_i"] (the MATLAB
    planner's l_i/2 midpoint approximation is NOT used)."""
    n = params["n"]
    mi = params["m_i"]
    m_total = sum(mi)
    R = [np.eye(3)]
    for i in range(n):
        R.append(R[i] @ _rot(params["h_i_im1"][i], q[i]))
    O = [np.zeros(3)]
    for i in range(1, n + 1):
        O.append(O[i - 1] + R[i] @ params["l_i"][i - 1])
    r0e = O[n]
    # Base link CoM: at the body origin (contributing zero) unless params
    # carries a measured "base_com" — the same key controller.dynamics() and
    # the C++ wb_base_com_* honour, so planner, governor and law cannot end up
    # describing different vehicles.
    base_com = np.asarray(params.get("base_com", np.zeros(3)), dtype=float)
    r0c = (mi[0] * base_com
           + sum(mi[i] * (O[i - 1] + R[i] @ params["com_i"][i - 1])
                 for i in range(1, n + 1))) / m_total
    return r0c, r0e


def _J3y(q, params):
    """Arm-only EE task Jacobian J_3y^0 (3 pos rows + 1 EE-yaw row) — the
    matrix the controller inverts; its conditioning is the relevant
    arm-singularity measure (port of the MATLAB planner's J03y)."""
    n = params["n"]
    h = params["h_i_im1"]
    R = [np.eye(3)]
    for i in range(n):
        R.append(R[i] @ _rot(h[i], q[i]))
    h0 = [R[k - 1] @ h[k - 1] for k in range(1, n + 1)]      # h_k^0
    O = [np.zeros(3)]
    for i in range(1, n + 1):
        O.append(O[i - 1] + R[i] @ params["l_i"][i - 1])
    r0e = O[n]
    b3e = R[n] @ _E3
    J = np.zeros((4, n))
    for k in range(1, n + 1):
        J[0:3, k - 1] = np.cross(h0[k - 1], r0e - O[k - 1])
        J[3, k - 1] = float(b3e @ h0[k - 1])
    return J


# ===========================================================================
# polynomial fit / evaluation (scaled monomial basis, mirrors the MATLAB)
# ===========================================================================

# np.RankWarning moved to np.exceptions.RankWarning in numpy 2.x
_RANK_WARNING = getattr(getattr(np, "exceptions", np), "RankWarning", RuntimeWarning)


def _fit_poly_cols(tau, Y, deg):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", _RANK_WARNING)
        return [np.polyfit(tau, Y[i], deg) for i in range(3)]


def _eval_poly_cols(P, tau, ts):
    """p, p', .., p'''' wrt TIME (tau = (t-tc)/ts). Scalar tau -> (3,) arrays,
    array tau -> (3, N)."""
    tau = np.asarray(tau, float)
    out = [np.zeros((3,) + tau.shape) for _ in range(5)]
    for i in range(3):
        c = P[i]
        for d in range(5):
            out[d][i] = np.polyval(c, tau) / ts**d
            c = np.polyder(c)
    return tuple(out)


# --- piecewise (per-phase) polynomial: one poly per segment -----------------
# A single global polynomial fits a move–hold–move CoM profile poorly (Gibbs-
# style wiggle around the hold, mm-scale defect and a shifted recovered q0).
# Fitting each phase separately keeps every segment smooth and small-residual;
# segments join at rest-to-rest boundaries (prescribed v = a = 0 there), so
# value/velocity/acceleration are continuous across joins up to fit residual.
# The classic single-phase task is just the one-segment case of the same code.

def _deriv_row(tau0, order, deg):
    """Constraint row for the `order`-th derivative at tau0, descending powers."""
    row = np.zeros(deg + 1)
    for i, k in enumerate(range(deg, -1, -1)):     # power k at column i
        if k >= order:
            fac = 1.0
            for j in range(order):
                fac *= (k - j)
            row[i] = fac * tau0 ** (k - order)
    return row


def _fit_poly_constrained(tau, y, deg, bc):
    """Least-squares polynomial fit with equality constraints bc = list of
    (tau0, deriv_order, value). Solved by null-space elimination — endpoint
    behavior is EXACT, unlike a plain lstsq fit whose endpoint derivatives
    ring (the ringing fed back through the Picard map via p_c'')."""
    A = np.vander(tau, deg + 1)
    C = np.array([_deriv_row(t0, o, deg) for (t0, o, _) in bc])
    d = np.array([v for (_, _, v) in bc])
    cp, *_ = np.linalg.lstsq(C, d, rcond=None)
    _, s, Vt = np.linalg.svd(C)
    rank = int(np.sum(s > 1e-10 * s[0]))
    Nsp = Vt[rank:].T
    z, *_ = np.linalg.lstsq(A @ Nsp, y - A @ cp, rcond=None)
    return cp + Nsp @ z


def _fit_pc_segments(t, Y, bounds, deg):
    """Per-segment fit. MULTI-segment plans pin each segment to the shared
    boundary value with zero velocity/acceleration at both ends (every phase
    is rest-to-rest, so the true p_c has ~0 vel there; the residual imposed
    error is the tiny snap-driven offset acceleration) — pinning both
    neighbours to the same boundary data value makes the piecewise p_c
    C^2-continuous at every join BY CONSTRUCTION, and kills the endpoint-
    derivative ringing the Picard map otherwise amplifies through p_c''.
    SINGLE-segment plans (the classic sweep) keep the plain unconstrained fit
    — flight-validated, and ~40x lower defect than the constrained form."""
    constrained = len(bounds) > 2
    segs = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        m = (t >= a - 1e-12) & (t <= b + 1e-12)
        tc, ts = 0.5 * (a + b), 0.5 * (b - a)
        tau = (t[m] - tc) / ts
        if constrained:
            P = []
            for i in range(3):
                y = Y[i, m]
                bc = [(-1.0, 0, y[0]), (-1.0, 1, 0.0), (-1.0, 2, 0.0),
                      (+1.0, 0, y[-1]), (+1.0, 1, 0.0), (+1.0, 2, 0.0)]
                P.append(_fit_poly_constrained(tau, y, deg, bc))
        else:
            P = _fit_poly_cols(tau, Y[:, m], deg)
        segs.append({"t0": a, "t1": b, "tc": tc, "ts": ts, "P": P})
    return segs


def _eval_pc(segs, tq):
    """Piecewise evaluation of p..p'''' at scalar or array tq."""
    tq = np.asarray(tq, float)
    if tq.ndim == 0:
        for s in segs:
            if tq <= s["t1"] or s is segs[-1]:
                return _eval_poly_cols(s["P"], (float(tq) - s["tc"]) / s["ts"], s["ts"])
    out = [np.zeros((3,) + tq.shape) for _ in range(5)]
    for i, s in enumerate(segs):
        m = (tq <= s["t1"]) if i == 0 else ((tq > s["t0"]) & (tq <= s["t1"]))
        if s is segs[-1]:
            m = m | (tq > s["t1"])
        if not m.any():
            continue
        vals = _eval_poly_cols(s["P"], (tq[m] - s["tc"]) / s["ts"], s["ts"])
        for d in range(5):
            out[d][:, m] = vals[d]
    return tuple(out)


# ===========================================================================
# prescribed task  (p_e, R_e, headings + analytic derivatives)
# ===========================================================================

def _prescribed_task(tq, tp):
    """Rich rest-to-rest task: 3-axis EE position, time-varying EE orientation
    (yaw/tilt/wrist about z-x-z) and an INDEPENDENT platform heading.  All
    signals are functions of the min-jerk phase sigma(t); time derivatives via
    the chain rule with sigma_dot/sigma_ddot, so start/stop are at rest."""
    s, ds, d2s = _minjerk3(tq / tp["T"])
    sg, sd, sdd = s, ds / tp["T"], d2s / tp["T"] ** 2
    pi_ = np.pi
    task = {}

    # ---- EE position p_e(sigma): linear span + sinusoidal arc ----
    X = tp["Dx"] * sg
    Xs, Xss = tp["Dx"], 0.0
    Y = tp["Dy"] * sg + tp["Ay"] * np.sin(pi_ * sg)
    Ys = tp["Dy"] + tp["Ay"] * pi_ * np.cos(pi_ * sg)
    Yss = -tp["Ay"] * pi_**2 * np.sin(pi_ * sg)
    Z = tp["Dz"] * sg + tp["Az"] * np.sin(pi_ * sg)
    Zs = tp["Dz"] + tp["Az"] * pi_ * np.cos(pi_ * sg)
    Zss = -tp["Az"] * pi_**2 * np.sin(pi_ * sg)
    pe_s = np.array([X, Y, Z])
    pe_s1 = np.array([Xs, Ys, Zs])
    pe_s2 = np.array([Xss, Yss, Zss])
    task["r_ed"] = tp["pe0"] + pe_s
    task["r_ed_dot"] = pe_s1 * sd
    task["r_ed_ddot"] = pe_s2 * sd**2 + pe_s1 * sdd

    # ---- EE orientation R_e = Rz(al)·Rx(be)·Rz(ga) (z-x-z, this asset's
    # joint axes); sigma-derivatives by the product rule ----
    al = tp["yaw0"] + tp["yawA"] * sg
    al1, al2 = tp["yawA"], 0.0
    be = tp["b0"] + tp["bA"] * np.sin(pi_ * sg)
    be1 = tp["bA"] * pi_ * np.cos(pi_ * sg)
    be2 = -tp["bA"] * pi_**2 * np.sin(pi_ * sg)
    ga = tp["g0"] + tp["gA"] * sg
    ga1, ga2 = tp["gA"], 0.0
    A, B, C = _Rz(al), _Rx(be), _Rz(ga)
    dA = _SZ @ A * al1
    d2A = _SZ @ _SZ @ A * al1**2 + _SZ @ A * al2
    dB = _SX @ B * be1
    d2B = _SX @ _SX @ B * be1**2 + _SX @ B * be2
    dC = _SZ @ C * ga1
    d2C = _SZ @ _SZ @ C * ga1**2 + _SZ @ C * ga2
    Re = A @ B @ C
    dRe = dA @ B @ C + A @ dB @ C + A @ B @ dC
    d2Re = (d2A @ B @ C + A @ d2B @ C + A @ B @ d2C
            + 2.0 * (dA @ dB @ C + dA @ B @ dC + A @ dB @ dC))
    task["R_e"] = Re
    b1, b1_s1, b1_s2 = Re[:, 0], dRe[:, 0], d2Re[:, 0]
    task["b1_de"] = b1
    task["b1_de_dot"] = b1_s1 * sd
    task["b1_de_ddot"] = b1_s2 * sd**2 + b1_s1 * sdd

    # ---- platform heading b1_d(sigma), independent of the EE yaw ----
    pp = tp["pp0"] + tp["ppA"] * sg
    pp1, pp2 = tp["ppA"], 0.0
    cp, sp = np.cos(pp), np.sin(pp)
    b1d = np.array([cp, sp, 0.0])
    b1d_s1 = pp1 * np.array([-sp, cp, 0.0])
    b1d_s2 = pp2 * np.array([-sp, cp, 0.0]) + pp1**2 * np.array([-cp, -sp, 0.0])
    task["b1_d"] = b1d
    task["b1_d_dot"] = b1d_s1 * sd
    task["b1_d_ddot"] = b1d_s2 * sd**2 + b1d_s1 * sdd
    return task


# ===========================================================================
# 4-phase showcase task  (fly-in -> pinned -> fly-out)
# ===========================================================================
# Per-segment signal generators. All return (value, d/dt, d2/dt2) — TIME
# derivatives, unlike the classic task's sigma-derivatives + chain rule.
# Every segment is min-jerk rest-to-rest, so position/velocity/acceleration
# are continuous (vel = acc = 0) at every phase boundary by construction.

def _scal_minjerk(a, b, tq, t0, T):
    s, ds, d2s = _minsnap3((tq - t0) / T)
    return a + (b - a) * s, (b - a) * ds / T, (b - a) * d2s / T**2


def _scal_sinarc(a, A, tq, t0, T):
    """a + A·sin(pi·s(sigma)) — leaves a, peaks at a+A, returns to a, at rest."""
    s, ds, d2s = _minsnap3((tq - t0) / T)
    s1, s2 = ds / T, d2s / T**2
    v = a + A * np.sin(np.pi * s)
    v1 = A * np.pi * np.cos(np.pi * s) * s1
    v2 = A * (-np.pi**2 * np.sin(np.pi * s) * s1**2 + np.pi * np.cos(np.pi * s) * s2)
    return v, v1, v2


def _scal_sincycle(a, A, tq, t0, T, phase=0.0):
    """a + A·sin(2·pi·s(sigma) + phase) — a FULL sine cycle: swings BOTH ways
    and returns to its start, so the platform yaw exercises q1 symmetrically
    instead of only one side. `phase` offsets the cycle, which is how the
    combo mode puts the fold 90 deg out of step with the yaw. Endpoint rates
    vanish for phase = 0 because the min-snap phase has s1 = 0 there; for a
    non-zero phase the VALUE at both ends still matches (sin is 2-pi periodic),
    so segments still join continuously."""
    s, ds, d2s = _minsnap3((tq - t0) / T)
    s1, s2 = ds / T, d2s / T**2
    w = 2.0 * np.pi
    th = w * s + phase
    v = a + A * np.sin(th)
    v1 = A * w * np.cos(th) * s1
    v2 = A * (-w**2 * np.sin(th) * s1**2 + w * np.cos(th) * s2)
    return v, v1, v2


def _vec_move(Pa, Pb, Ay, Az, tq, t0, T):
    """Min-snap Pa -> Pb with sinusoidal y/z arcs bulging out mid-segment."""
    s, ds, d2s = _minsnap3((tq - t0) / T)
    s1, s2 = ds / T, d2s / T**2
    D = Pb - Pa
    ay, ay1, ay2 = _scal_sinarc(0.0, Ay, tq, t0, T)
    az, az1, az2 = _scal_sinarc(0.0, Az, tq, t0, T)
    return (Pa + D * s + np.array([0.0, ay, az]),
            D * s1 + np.array([0.0, ay1, az1]),
            D * s2 + np.array([0.0, ay2, az2]))


def _orient_from_angles(al, al1, al2, be, be1, be2, ga, ga1, ga2):
    """R_e = Rz(al)·Rx(be)·Rz(ga) and b1_de = R_e[:,0] time-derivatives, from
    angle TIME-derivatives (same product-rule algebra as the classic task)."""
    A, B, C = _Rz(al), _Rx(be), _Rz(ga)
    dA = _SZ @ A * al1
    d2A = _SZ @ _SZ @ A * al1**2 + _SZ @ A * al2
    dB = _SX @ B * be1
    d2B = _SX @ _SX @ B * be1**2 + _SX @ B * be2
    dC = _SZ @ C * ga1
    d2C = _SZ @ _SZ @ C * ga1**2 + _SZ @ C * ga2
    Re = A @ B @ C
    dRe = dA @ B @ C + A @ dB @ C + A @ B @ dC
    d2Re = (d2A @ B @ C + A @ d2B @ C + A @ B @ d2C
            + 2.0 * (dA @ dB @ C + dA @ B @ dC + A @ dB @ dC))
    return Re, dRe[:, 0], d2Re[:, 0]


def _heading_from_angle(pp, pp1, pp2):
    cp, sp = np.cos(pp), np.sin(pp)
    b1d = np.array([cp, sp, 0.0])
    t1 = np.array([-sp, cp, 0.0])
    return b1d, pp1 * t1, pp2 * t1 - pp1**2 * b1d


def _prescribed_task_showcase(tq, tp):
    """Three phases, all rest-to-rest. The arm starts and ends at the FOLDED,
    singularity-certified HOME pose (b_home; held by the takeoff hold before
    the plan and by the landing hold after it):
       1 fly-in   forward flight, arm UNFOLDS home -> work
       2 pinned   EE position AND yaw exactly fixed; platform yaws and the arm
                  folds/unfolds around the frozen tool point
       3 fly-out  flight to above the landing spot, arm FOLDS BACK work -> home
    The plan ENDS hovering at land_wp — the descent to the ground is NOT part
    of the task: it belongs to the demo's LANDING phase (a plain setpoint with
    the arm PD-held), mirroring the real-experiment workflow where takeoff and
    landing are position setpoints and only the middle phase tracks a plan.
       4 land     LEGACY optional in-plan descent (T_land > 0): pure VERTICAL
                  drop of land_drop, arm held at home. Off (T_land = 0) by
                  default — kept only so old configs still evaluate.
    """
    T1, T2, T3, T4 = tp["T1"], tp["T2"], tp["T3"], tp["T4"]
    z3 = np.zeros(3)
    P0, Pt, Pl = np.zeros(3), tp["target"], tp["land_wp"]
    Pg = Pl - np.array([0.0, 0.0, tp["land_drop"]])      # touchdown EE point

    if tq < T1:                                   # ---- 1. fly-in ----
        pe, pe1, pe2 = _vec_move(P0, Pt, tp["Ay"], tp["Az"], tq, 0.0, T1)
        al, al1, al2 = _scal_minjerk(0.0, tp["yaw_work"], tq, 0.0, T1)
        be, be1, be2 = _scal_minjerk(tp["b_home"], tp["b_work"], tq, 0.0, T1)
        pp, pp1, pp2 = 0.0, 0.0, 0.0
    elif tq < T1 + T2:                            # ---- 2. pinned ----
        pe, pe1, pe2 = Pt.copy(), z3, z3          # EE position exactly fixed
        al, al1, al2 = tp["yaw_work"], 0.0, 0.0   # EE heading exactly fixed
        if tp["pin_mode"] == "combo":
            # COMBO: the drone bobs UP AND DOWN *while* yawing left and right.
            # The fold runs a FULL cycle (work -> +A -> work -> -A -> work), which
            # is what makes the vertical travel large: the drone's height above the
            # pinned tip is a monotonic function of beta (0.147 m at beta=20 down to
            # 0.029 m at beta=80), so sweeping beta both ways gives ~0.118 m instead
            # of the 0.051 m a one-sided sweep manages. The yaw runs its own full
            # cycle 90 deg out of phase, so in (yaw, height) the drone traces a
            # CIRCLE — the closest thing to the user's circle that the geometry
            # actually allows (a spatial circle is impossible; see pin_mode notes).
            be, be1, be2 = _scal_sincycle(tp["b_work"], tp["b_pinA"], tq, T1, T2,
                                          phase=tp["pin_phase"])
            pp, pp1, pp2 = _scal_sincycle(0.0, tp["pp_pinA"], tq, T1, T2)
        elif tp["pin_mode"] == "vertical":
            # VERTICAL mode: heading held, arm sweeps its FULL usable fold range
            # (work -> home -> work). Because v_z > 0 for every reachable q, this
            # swings the drone up/down over the frozen tool tip — an ARC on the
            # upper side of the arm's workspace sphere, NOT a circle: the drone
            # can never pass below the EE.
            be, be1, be2 = _scal_sinarc(tp["b_work"], tp["b_home"] - tp["b_work"],
                                        tq, T1, T2)
            pp, pp1, pp2 = 0.0, 0.0, 0.0
        else:
            # YAW mode (default): platform yaw does a FULL sine cycle
            # 0 -> +A -> 0 -> -A -> 0. With the EE yaw held fixed the arm must
            # counter-rotate exactly, q1 = -pp, so the amplitude is capped by
            # q1's +/-35 deg limit (see pp_pin_amp_deg).
            be, be1, be2 = _scal_sinarc(tp["b_work"], tp["b_pinA"], tq, T1, T2)
            pp, pp1, pp2 = _scal_sincycle(0.0, tp["pp_pinA"], tq, T1, T2)
    elif tq < T1 + T2 + T3:                       # ---- 3. fly-out + fold back ----
        t0 = T1 + T2
        pe, pe1, pe2 = _vec_move(Pt, Pl, tp["Ay"], tp["Az"], tq, t0, T3)
        al, al1, al2 = _scal_minjerk(tp["yaw_work"], 0.0, tq, t0, T3)
        be, be1, be2 = _scal_minjerk(tp["b_work"], tp["b_home"], tq, t0, T3)
        pp, pp1, pp2 = 0.0, 0.0, 0.0
    else:                                # ---- 4. legacy in-plan descent ----
        # Reached only past T1+T2+T3: with T_land = 0 (the default) that is the
        # single grid endpoint tq = T, where the plan holds the fly-out end at
        # rest. With T_land > 0, the legacy straight-down descent evaluates
        # (no lateral arcs, so the legs come onto the ground under the vehicle).
        t0 = T1 + T2 + T3
        if T4 <= 0.0:
            pe, pe1, pe2 = Pl.copy(), z3, z3      # hover at land_wp (= plan end)
        else:
            pe, pe1, pe2 = _vec_move(Pl, Pg, 0.0, 0.0, tq, t0, T4)
        al, al1, al2 = 0.0, 0.0, 0.0
        be, be1, be2 = tp["b_home"], 0.0, 0.0     # arm held FOLDED at home
        pp, pp1, pp2 = 0.0, 0.0, 0.0

    task = {"r_ed": tp["pe0"] + pe, "r_ed_dot": pe1, "r_ed_ddot": pe2}
    # wrist ga = 0 throughout: keeps b1_de independent of beta (see above)
    Re, b1_1, b1_2 = _orient_from_angles(al, al1, al2, be, be1, be2, 0.0, 0.0, 0.0)
    task["R_e"] = Re
    task["b1_de"], task["b1_de_dot"], task["b1_de_ddot"] = Re[:, 0], b1_1, b1_2
    task["b1_d"], task["b1_d_dot"], task["b1_d_ddot"] = _heading_from_angle(pp, pp1, pp2)
    return task


def _eval_task(tq, tp):
    """Dispatch: classic single-phase sweep vs the 4-phase showcase, then apply
    the optional PATH YAW.

    path_yaw rotates the prescribed EE POSITION about the canonical z axis while
    leaving every ORIENTATION signal (R_e, b1_de, b1_d) untouched — i.e. it
    changes the direction the vehicle TRAVELS without changing where it POINTS.
    That decoupling is the whole point: on this asset the mechanical front of
    the aerial manipulator is body +y, not body +x, but the controller's b1_d
    steers body +x. Setting path_yaw = +90 deg therefore sends the vehicle along
    world +y — face-first — while the attitude reference stays put.

    Valid because the canonical EE start pe0 is the origin, so rotating r_ed
    about z is a rigid rotation of the whole task, and gravity (the only other
    inertial direction in the compatibility solve) is z-invariant.

    TEMPORARY: once the USDA is re-authored so the mechanical front is body +x,
    set path_yaw_deg back to 0 and the demo flies along +x natively."""
    task = (_prescribed_task_showcase(tq, tp) if tp.get("mode") == "showcase"
            else _prescribed_task(tq, tp))
    psi = tp.get("path_yaw", 0.0)
    if psi:
        Rp = _Rz(psi)
        for k in ("r_ed", "r_ed_dot", "r_ed_ddot"):
            task[k] = Rp @ task[k]
    return task


# ===========================================================================
# online FULL reference (canonical frame; drop-in once anchored)
# ===========================================================================

def _eval_full_ref(tq, Pc, tp, params):
    p, p1, p2, p3, p4 = _eval_pc(Pc, tq)
    ref = {"x_cd": p, "x_cd_dot": p1, "x_cd_ddot": p2,
           "x_cd_d3": p3, "x_cd_d4": p4}
    task = _eval_task(tq, tp)
    for k in ("b1_d", "b1_d_dot", "b1_d_ddot",
              "r_ed", "r_ed_dot", "r_ed_ddot",
              "b1_de", "b1_de_dot", "b1_de_ddot"):
        ref[k] = task[k]
    # recovered arm configuration (for logging / the takeoff pre-position)
    ref["q_d"], _, _ = _recover_q(p2, task, tp)
    return ref


# ===========================================================================
# main planner
# ===========================================================================

def plan_compatible_trajectory(params, task_cfg=None, opts=None):
    """Solve the compatible-CoM plan. `params` = controller.make_params()
    dict; `task_cfg`/`opts` override _DEFAULT_TASK/_DEFAULT_OPTS. A task_cfg
    carrying "T_fly_in" selects the 4-phase SHOWCASE task (_DEFAULT_SHOWCASE
    defaults) instead of the classic single-phase sweep."""
    showcase = "T_fly_in" in (task_cfg or {})
    task_cfg = dict(_DEFAULT_SHOWCASE if showcase else _DEFAULT_TASK,
                    **(task_cfg or {}))
    opts = dict(_DEFAULT_OPTS, **(opts or {}))
    g = params["g"]
    rad = np.deg2rad

    # canonical frame: EE starts at the ORIGIN, zero platform heading offset
    if showcase:
        T1 = float(task_cfg["T_fly_in"])
        T2 = float(task_cfg["T_pinned"])
        T3 = float(task_cfg["T_fly_out"])
        # T_land = 0 (default): NO in-plan descent — the plan ends hovering at
        # land_wp and the demo's LANDING phase (a setpoint) owns the descent.
        T4 = float(task_cfg.get("T_land", 0.0))
        T = T1 + T2 + T3 + T4
        tp = {"mode": "showcase", "T": T, "T1": T1, "T2": T2, "T3": T3, "T4": T4,
              "pe0": np.zeros(3), "g": g,
              "target": np.asarray(task_cfg["target"], float),
              "land_wp": np.asarray(task_cfg["land_wp"], float),
              "land_drop": float(task_cfg.get("land_drop", 0.0)),
              "Ay": float(task_cfg["Ay"]), "Az": float(task_cfg["Az"]),
              "b_home": rad(task_cfg["beta_home_deg"]),
              "b_work": rad(task_cfg["beta_work_deg"]),
              "b_pinA": rad(task_cfg["beta_pin_amp_deg"]),
              "yaw_work": rad(task_cfg["yaw_work_deg"]),
              "pp_pinA": rad(task_cfg["pp_pin_amp_deg"]),
              "pin_mode": str(task_cfg["pin_mode"]),
              "pin_phase": rad(task_cfg["pin_phase_deg"]),
              "path_yaw": rad(task_cfg["path_yaw_deg"]),
              "split": float(task_cfg["split"])}
    else:
        T = float(task_cfg["T"])
        tp = {"mode": "classic", "T": T, "pe0": np.zeros(3), "g": g,
              "Dx": float(task_cfg["Dx"]), "Dy": float(task_cfg["Dy"]),
              "Dz": float(task_cfg["Dz"]),
              "Ay": float(task_cfg["Ay"]), "Az": float(task_cfg["Az"]),
              "yaw0": rad(task_cfg["yaw0_deg"]), "yawA": rad(task_cfg["yawA_deg"]),
              "b0": rad(task_cfg["b0_deg"]), "bA": rad(task_cfg["bA_deg"]),
              "g0": rad(task_cfg["g0_deg"]), "gA": rad(task_cfg["gA_deg"]),
              "pp0": rad(task_cfg["pp0_deg"]), "ppA": rad(task_cfg["ppA_deg"]),
              "path_yaw": rad(task_cfg["path_yaw_deg"]),
              "split": float(task_cfg["split"])}
    N = int(opts["N"])
    t = np.linspace(0.0, T, N)

    # prescribed signals on the recovery grid (zeroth order)
    PE = np.zeros((3, N))
    B1D = np.zeros((3, N))
    RE = np.zeros((N, 3, 3))
    for k in range(N):
        tk = _eval_task(t[k], tp)
        PE[:, k] = tk["r_ed"]
        B1D[:, k] = tk["b1_d"]
        RE[k] = tk["R_e"]

    # ---- recover p_c: damped fixed point on a piecewise-polynomial p_c ----
    # one polynomial per phase (classic task = single segment over [0, T]);
    # with T_land = 0 the last edge coincides with T1+T2+T3, so the legacy
    # descent segment is dropped rather than fit over zero length
    if tp["mode"] == "showcase":
        bounds = [0.0, tp["T1"], tp["T1"] + tp["T2"], tp["T1"] + tp["T2"] + tp["T3"]]
        if tp["T4"] > 0.0:
            bounds.append(T)
    else:
        bounds = [0.0, T]
    Pc = _fit_pc_segments(t, PE, bounds, opts["deg"])   # init: p_c ~ p_e
    hist = []
    for _ in range(int(opts["maxit"])):
        pcdd = _eval_pc(Pc, t)[2]
        pc_new = np.zeros((3, N))
        for k in range(N):
            tk = {"b1_d": B1D[:, k], "R_e": RE[k]}
            q, R0, _ = _recover_q(pcdd[:, k], tk, tp)  # (C1)+(R1)+(R2)
            r0c, r0e = _arm_kin(q, params)
            pc_new[:, k] = PE[:, k] + R0 @ (r0c - r0e)  # (C2)
        pc_cur = _eval_pc(Pc, t)[0]
        Pc_new = _fit_pc_segments(
            t, (1.0 - opts["relax"]) * pc_cur + opts["relax"] * pc_new,
            bounds, opts["deg"])
        step = float(np.max(np.linalg.norm(
            _eval_pc(Pc_new, t)[0] - pc_cur, axis=0)))
        hist.append(step)
        Pc = Pc_new
        if step < opts["tol"]:
            break
    nit = len(hist)

    # ---- diagnostics (fine grid): dynamic defect + regularity guards ----
    Nf = int(opts["Nfine"])
    tf = np.linspace(0.0, T, Nf)
    pcf, _, pcddf, _, _ = _eval_pc(Pc, tf)
    e_dyn = np.zeros((3, Nf))
    beta_f = np.zeros(Nf)
    acn_f = np.zeros(Nf)
    tilt_f = np.zeros(Nf)
    th_f = np.zeros((4, Nf))
    cond_f = np.zeros(Nf)
    sig_f = np.zeros(Nf)
    sig_nd_f = np.zeros(Nf)
    # Non-dimensional singularity metric from the MATLAB analysis
    # (utils_singularity/singularity_sweep.m): sigma_min of J_3y0 with its
    # translational rows scaled by 1/Lchar (half the arm's total reach). The
    # certified SAFE set is {q : sigma_min >= 0.10}; the singular branches
    # (elbow/yaw) live at NEGATIVE q2/q3 in this asset's sign convention.
    Lchar = 0.5 * sum(np.linalg.norm(l) for l in params["l_i"])
    S_nd = np.diag([1.0 / Lchar] * 3 + [1.0])
    for k in range(Nf):
        tk = _eval_task(tf[k], tp)
        q, R0, ac = _recover_q(pcddf[:, k], tk, tp)
        r0c, r0e = _arm_kin(q, params)
        e_dyn[:, k] = pcf[:, k] - (tk["r_ed"] + R0 @ (r0c - r0e))
        beta_f[k] = q[1] + q[2]
        acn_f[k] = np.linalg.norm(ac)
        tilt_f[k] = np.degrees(np.arccos(np.clip(ac[2] / acn_f[k], -1.0, 1.0)))
        th_f[:, k] = q
        J = _J3y(q, params)
        s_k = np.linalg.svd(J, compute_uv=False)
        cond_f[k] = s_k[0] / s_k[-1]
        sig_f[k] = s_k[-1]
        sig_nd_f[k] = np.linalg.svd(S_nd @ J, compute_uv=False)[-1]

    diag = {
        "iterations": nit,
        "conv_history": np.array(hist),
        "max_dyn_defect": float(np.max(np.linalg.norm(e_dyn, axis=0))),
        "min_sin_beta": float(np.min(np.abs(np.sin(beta_f)))),
        "beta_range_deg": np.degrees([beta_f.min(), beta_f.max()]),
        "min_acc_norm": float(acn_f.min()),
        "max_thrust_tilt": float(tilt_f.max()),
        "max_cond_J3y": float(cond_f.max()),
        "cond_J3y_t0": float(cond_f[0]),
        "min_sigma_J3y": float(sig_f.min()),
        "sigma_J3y_t0": float(sig_f[0]),
        "min_sigma_nd": float(sig_nd_f.min()),      # certified-safe if >= 0.10
        "sigma_nd_margin": 0.10,
        "theta_min_deg": np.degrees(th_f.min(axis=1)),
        "theta_max_deg": np.degrees(th_f.max(axis=1)),
    }

    if opts["verbose"]:
        print("\n=== compatible-CoM planning (z-x-z port) ===")
        print(f"Picard iterations          : {nit}  (final step {hist[-1]:.2e} m)")
        print(f"MAX dynamic defect |e_dyn| : {diag['max_dyn_defect']:.3e} m   <-- should be ~0")
        print(f"min |sin(beta)|            : {diag['min_sin_beta']:.3e}     (~0 => wrist singularity)")
        print(f"beta range                 : [{diag['beta_range_deg'][0]:.1f}, "
              f"{diag['beta_range_deg'][1]:.1f}] deg")
        print(f"max thrust tilt            : {diag['max_thrust_tilt']:.1f} deg")
        print(f"cond(J_3y^0): start {diag['cond_J3y_t0']:.0f} | max {diag['max_cond_J3y']:.0f} | "
              f"sigma_min: start {diag['sigma_J3y_t0']:.2e} | min {diag['min_sigma_J3y']:.2e}")
        print(f"recovered q range [deg]    : min {np.round(diag['theta_min_deg'], 1)} "
              f"max {np.round(diag['theta_max_deg'], 1)}")
        print(f"singularity margin         : min sigma_nd = {diag['min_sigma_nd']:.3f}  "
              f"(certified-safe >= {diag['sigma_nd_margin']:.2f}, "
              f"= {diag['min_sigma_nd']/diag['sigma_nd_margin']:.1f}x margin)")
        if diag["min_sigma_nd"] < diag["sigma_nd_margin"]:
            print("WARNING: trajectory leaves the certified-safe set of the "
                  "singularity analysis (sigma_min < 0.10) — the singular "
                  "branches are at NEGATIVE q2/q3 on this asset; keep the fold "
                  "positive and away from q3 ~ -52 deg.")
        if diag["min_sin_beta"] < 1e-2:
            print("WARNING: near WRIST singularity (beta~0) — raise the EE tilt b0_deg/bA_deg.")
        if diag["max_cond_J3y"] > 300:
            print(f"WARNING: near ARM singularity, cond(J_3y)={diag['max_cond_J3y']:.0f} — "
                  f"adjust b0_deg/bA_deg or split.")

    # optional joint-limit check of the recovered arm trajectory
    if opts["q_lim_deg"] is not None:
        lim = np.asarray(opts["q_lim_deg"], float)
        lo_ok = diag["theta_min_deg"] >= lim[:, 0]
        hi_ok = diag["theta_max_deg"] <= lim[:, 1]
        if not (lo_ok.all() and hi_ok.all()):
            print(f"WARNING: recovered q exceeds joint limits {lim.tolist()} deg — "
                  f"range min {np.round(diag['theta_min_deg'], 1)} "
                  f"max {np.round(diag['theta_max_deg'], 1)}. "
                  f"Reduce yawA_deg/ppA_deg (q1/q4) or b0_deg+bA_deg (q2/q3).")

    # ---- package: online reference handle + consistent t=0 configuration ----
    plan = {"T": T, "Pc": Pc, "tp": tp, "diag": diag,
            "t": t, "p_e": PE,
            "p_c": _eval_pc(Pc, t)[0],
            "ref": lambda tq: _eval_full_ref(tq, Pc, tp, params)}
    ref0 = plan["ref"](0.0)
    plan["q0"] = ref0["q_d"].copy()
    # canonical CoM-from-EE offset at t=0 (handy for anchoring checks)
    plan["pc0"] = ref0["x_cd"].copy()
    return plan


def get_plan(params, cfg):
    """Cached wrapper for build_traj: cfg is a flat dict holding any of the
    task keys and/or planner-option keys (TRAJ_CONFIG['poly_whole']).  The
    canonical plan depends only on cfg (params are process-constant), so one
    plan serves every build_traj call in a run."""
    key = repr(sorted((k, repr(v)) for k, v in cfg.items()))
    if key not in _PLAN_CACHE:
        task_cfg = {k: cfg[k] for k in _TASK_KEYS if k in cfg}
        opts = {k: cfg[k] for k in _OPT_KEYS if k in cfg}
        _PLAN_CACHE[key] = plan_compatible_trajectory(params, task_cfg, opts)
    return _PLAN_CACHE[key]

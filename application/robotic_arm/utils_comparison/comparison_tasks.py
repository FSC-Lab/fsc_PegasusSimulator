#!/usr/bin/env python3
"""ONE task definition per comparison, consumed identically by both controllers.

WHY THIS FILE EXISTS
--------------------
The campaign compares two *control designs* on the SAME plant:

  WB  whole-body coupled impedance + GMO  (autopilot_whole_body_direct_actuation_node)
      -- commands the 4 rotors AND the 4 arm joint torques from one law.
  L1  geometric SE(3) + L1 adaptive       (autopilot_geometric_l1_direct_actuation_node)
      -- commands the 4 rotors only; the arm is a POSITION-mode servo tracking
         a joint reference, and its reaction on the base is a disturbance the
         L1 augmentation has to reject.

A comparison is only meaningful if the two stacks are handed the SAME commanded
motion.  That is what this module guarantees: every test is defined ONCE here,
as a tuple

    (base pose x_b(t), psi(t))   +   (joint vector q(t))   +   (EE pose r_e(t), b1e(t))

and both drivers read the same table.  The WB stack gets the whole tuple as a
`WholeBodyReference` (its law tracks CoM + EE + posture); the L1 stack gets the
base half as a `PositionControllerReference` and the joint half as a servo
position command.  Neither side is given anything the other is not.

TASK CONSTRUCTION (two directions, both exact)
----------------------------------------------
  tests 1 and 3   prescribe the BASE path and the JOINT path; the EE reference
                  is then forward kinematics -- always feasible by construction,
                  which is what "a compatible EE motion relative to the drone"
                  means.
  test 2          prescribes the BASE path and PINS the EE (3 position + 1
                  heading); the joint path is the 4-DOF inverse kinematics of
                  the pinned point.  This is the only one that can be
                  infeasible, and `feasibility_report()` checks it offline.

Both directions produce the identical field set, so the drivers do not care
which was used.

FRAMES
------
World is the shared ENU world.  The whole-body law's MODEL frame is
AM_realign's y-forward body frame while the flying asset (AM_xfwd) is
x-forward, exactly as frame_adapter.hpp / the whole-body planner declare:

    R0_model = R0_actual @ R_MODEL          phi_model = psi_actual - pi/2

World-frame POSITIONS are frame-agnostic and shared.  The heading vectors b1_d
/ b1_de in `WholeBodyReference` are MODEL body-x directions in the world, so
they carry the -90 deg offset; the `PositionControllerReference.yaw` the L1
stack reads is the ACTUAL yaw.  Both are produced here from one phi.

DERIVATIVES
-----------
Every reference derivative (CoM through snap, headings and EE through
acceleration) is a central finite difference taken ALONG the uniform sample
grid, not an independent analytic chain -- so the derivative a controller
receives is by construction the derivative of the position it receives.  The
scalar time functions are C4-smooth (a degree-9 smoothstep envelope on every
phase variable), which is what makes the 3rd/4th differences clean;
`fd_consistency_report()` measures it.

Run this file directly for the offline self-test (pure numpy, no Isaac, no ROS).
"""

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXT = os.path.abspath(os.path.join(_HERE, "..", "..", "..",
                                    "extensions", "fsc_aerial_manipulation"))
if _EXT not in sys.path:
    sys.path.insert(0, _EXT)

from fsc_aerial_manipulation.robotic_arm.utils_planner import (  # noqa: E402
    transition_planner as TP,
)

# ---------------------------------------------------------------------------
# conventions shared with the flight stack
# ---------------------------------------------------------------------------

#: Folded HOME pose -- the arm's spawn/takeoff pose in 05 and 06 (`Q_HOME`),
#: the value both controller yamls carry, and the pose every task starts and
#: ends at.  Changing it here alone would silently disagree with the plant.
Q_HOME = np.array([0.0, np.deg2rad(40.0), np.deg2rad(40.0), 0.0])

#: MODEL heading of an ACTUAL yaw of zero.  The asset's mechanical front (the
#: arm) is body +x, the model frame calls that +y.
PHI_OF_PSI0 = -0.5 * np.pi

#: Sample period of the precomputed table [s].  250 Hz = the inner-loop rate;
#: the drivers stream at 100 Hz by taking the nearest sample, so no
#: interpolation error enters the commanded reference.
DT = 0.004

#: Finite-difference stride used for the reference derivatives [samples].
#: 5 samples = 20 ms: large enough that the 4th difference's roundoff floor
#: (~1e-9 m/s^4) is far below the signal, small enough that the O(h^2)
#: truncation error is negligible for the <0.2 Hz content of these tasks.
GRAVITY = 9.80665

FD_STRIDE = 5
#: Stride for the 3rd/4th differences.  Those amplify sample noise by 1/h^3
#: and 1/h^4, and test 2's joint path comes out of a numerical IK whose
#: residual leaves ~1e-9 rad of jitter -- at the 20 ms stride that surfaced as
#: 2.4e-4 m/s^4 of snap noise.  48 ms drops it ~33x while the O(h^2)
#: truncation error stays negligible for these <0.2 Hz tasks.
FD_STRIDE_HI = 12


def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def smoothstep(x, n=4):
    """Degree-(2n+1) smoothstep on [0, 1]: value 0->1 with the first n
    derivatives vanishing at BOTH ends.  n = 4 gives C4, so a phase variable
    built on it leaves the CoM snap continuous at every hold/move join --
    the same reason the poly_whole planner needs min-snap rather than
    min-jerk phases."""
    x = np.clip(np.asarray(x, float), 0.0, 1.0)
    from math import comb
    out = np.zeros_like(x)
    for k in range(n + 1):
        out = out + comb(n + k, k) * comb(2 * n + 1, n - k) * (-x) ** k
    return out * x ** (n + 1)


# ---------------------------------------------------------------------------
# task definitions
# ---------------------------------------------------------------------------
#
# Every task is a list of PHASES.  A phase is (name, duration).  Within the
# task the scalar drivers are written as functions of absolute time, and each
# is wrapped in a smoothstep envelope so the whole mission starts and ends at
# rest with vanishing derivatives.

TASKS = {}


class _Task(object):
    """A comparison task: base path, joint path, EE path, on a uniform grid."""

    name = "?"
    title = "?"
    #: base setpoint the mission takes off to and hovers at before the task
    base_home = np.array([0.0, 0.0, 1.0])
    psi_home = 0.0
    #: seconds of hold at the start (settling) and end (return to rest)
    lead_s = 6.0
    trail_s = 8.0
    move_s = 24.0

    def __init__(self, **overrides):
        for k, v in overrides.items():
            if not hasattr(self, k):
                raise KeyError(f"{self.name}: unknown option '{k}'")
            setattr(self, k, v)
        self.base_home = np.asarray(self.base_home, float)
        self.params = TP.make_params_t650()

    # -- to be provided by each task -------------------------------------
    def base_and_psi(self, t):
        """(x_b [3], psi_actual) at absolute mission time t."""
        raise NotImplementedError

    def joints(self, t, q_prev):
        """(q [4], ok, note) at absolute mission time t.  `q_prev` is the
        previous grid sample, used as the IK seed where a task needs one."""
        raise NotImplementedError

    # -- shared machinery -------------------------------------------------
    @property
    def duration(self):
        return self.lead_s + self.move_s + self.trail_s

    def marks(self):
        return [("lead", 0.0, self.lead_s),
                ("move", self.lead_s, self.lead_s + self.move_s),
                ("trail", self.lead_s + self.move_s, self.duration)]

    def _u(self, t):
        """Task phase variable: 0 during the lead hold, a C4 ramp 0->1 across
        the move, 1 during the trailing hold."""
        return float(smoothstep((t - self.lead_s) / self.move_s))

    def build(self, dt=DT, verbose=False):
        """Dense reference table.  Returns a dict of (N, ...) arrays."""
        n = int(round(self.duration / dt)) + 1
        t = np.arange(n) * dt
        xb = np.zeros((n, 3))
        psi = np.zeros(n)
        q = np.zeros((n, 4))
        ok = np.ones(n, bool)
        notes = []
        q_prev = Q_HOME.copy()
        for i in range(n):
            xb[i], psi[i] = self.base_and_psi(t[i])
            q[i], ok[i], note = self.joints(t[i], q_prev)
            q_prev = q[i]
            if note:
                notes.append((t[i], note))

        phi = psi + PHI_OF_PSI0
        # forward kinematics on the controller's own chain, per sample
        r0c = np.zeros((n, 3))
        r0e = np.zeros((n, 3))
        b1e_b = np.zeros((n, 3))
        sig = np.zeros(n)
        for i in range(n):
            a, b, re = TP.arm_fk_model(q[i], self.params)
            r0c[i], r0e[i], b1e_b[i] = a, b, re[:, 0]
            sig[i] = TP._sigma_nd(q[i], self.params)

        b1_d = np.stack([np.cos(phi), np.sin(phi), np.zeros(n)], axis=1)

        # ---- attitude-consistent reference (a small Picard fixed point) ----
        #
        # The commanded quantity is the BASE pose (x_b, phi) and the JOINTS.
        # The CoM and the EE then sit at x_b + R0 r_0c(q) and x_b + R0 r_0e(q),
        # where R0 is the attitude the vehicle must actually hold -- which is
        # NOT level whenever the reference accelerates, because the thrust axis
        # has to tilt.  Taking R0 = Rz(phi) instead would put a systematic
        # |r_0e| * tilt bias into the EE reference (4 mm at test 3's peak
        # acceleration), so R0 is solved for: b3 from the CoM acceleration the
        # reference itself demands, b1 from the commanded heading.  Exactly the
        # coupling compatible_trajectory.py's planner iterates, in the easy
        # direction (task prescribed, attitude solved).
        Rz = np.array([_Rz(p) for p in phi])
        x_cd_level = xb + np.einsum("nij,nj->ni", Rz, r0c)
        # ONE correction step, not a fixed point.  Iterating x_cd -> R0(d2/dt2
        # x_cd) -> x_cd is unstable at sample resolution (the map has gain
        # 1/h^2, so it amplifies the FD's own noise until b3 flips -- measured:
        # divergence to a 180 deg "tilt" in 12 iterations).  The proper
        # planner tames that by representing x_cd as a low-order polynomial;
        # here it is unnecessary, because the correction is tiny and its
        # feedback is second order: |r_0c| is only 24 mm, so a 1 deg tilt moves
        # x_cd by 0.4 mm, whose own acceleration is ~1e-4 m/s^2 against the
        # 0.15 m/s^2 that set the tilt.  What the step DOES buy is the EE
        # reference: |r_0e| is 0.25 m, so the same tilt is 4 mm there -- a
        # systematic bias worth removing from a millimetre-scale metric.
        acc = _fd(x_cd_level, dt, 2, stride=FD_STRIDE_HI)
        b3 = acc + np.array([0.0, 0.0, GRAVITY])
        b3 = b3 / np.linalg.norm(b3, axis=1, keepdims=True)
        b2 = np.cross(b3, b1_d)
        b2 = b2 / np.linalg.norm(b2, axis=1, keepdims=True)
        b1 = np.cross(b2, b3)
        R0 = np.stack([b1, b2, b3], axis=2)
        x_cd = xb + np.einsum("nij,nj->ni", R0, r0c)
        r_ed = xb + np.einsum("nij,nj->ni", R0, r0e)
        b1_de = np.einsum("nij,nj->ni", R0, b1e_b)
        picard = np.array([float(np.abs(x_cd - x_cd_level).max())])
        tilt = np.degrees(np.arccos(np.clip(R0[:, 2, 2], -1.0, 1.0)))

        d = {
            "name": self.name, "title": self.title, "dt": dt, "t": t,
            "x_b": xb, "psi": psi, "phi": phi, "q": q, "sigma_nd": sig,
            "feasible": ok, "x_cd": x_cd, "r_ed": r_ed,
            "b1_d": b1_d, "b1_de": b1_de,
            "base_home": self.base_home.copy(), "psi_home": self.psi_home,
            "q_home": Q_HOME.copy(),
            "lead_s": self.lead_s, "move_s": self.move_s,
            "trail_s": self.trail_s,
            "ref_tilt_deg": tilt, "picard": np.array(picard),
            "mark_names": np.array([m[0] for m in self.marks()]),
            "mark_t": np.array([[m[1], m[2]] for m in self.marks()], float),
        }
        # derivative chains: exactly the fields WholeBodyReference carries
        d["x_b_dot"] = _fd(xb, dt, 1)
        d["x_b_ddot"] = _fd(xb, dt, 2)
        d["x_cd_dot"] = _fd(x_cd, dt, 1)
        d["x_cd_ddot"] = _fd(x_cd, dt, 2)
        d["x_cd_d3"] = _fd(x_cd, dt, 3)
        d["x_cd_d4"] = _fd(x_cd, dt, 4)
        d["b1_d_dot"] = _fd(b1_d, dt, 1)
        d["b1_d_ddot"] = _fd(b1_d, dt, 2)
        d["r_ed_dot"] = _fd(r_ed, dt, 1)
        d["r_ed_ddot"] = _fd(r_ed, dt, 2)
        d["b1_de_dot"] = _fd(b1_de, dt, 1)
        d["b1_de_ddot"] = _fd(b1_de, dt, 2)
        d["qdot_d"] = _fd(q, dt, 1)
        if verbose:
            for tt, note in notes[:10]:
                print(f"  [t={tt:6.2f}] {note}")
        return d


def _fd(y, dt, order, stride=None):
    """Central finite difference of a (N, k) sample array along axis 0.

    Endpoint samples that a stencil cannot reach are held at the nearest
    interior value.  Every task starts and ends in a multi-second HOLD, so
    those samples are exactly zero-derivative anyway and the clamp is a
    no-op -- it exists so a table is never handed out with NaNs in it.
    """
    if stride is None:
        stride = FD_STRIDE if order <= 2 else FD_STRIDE_HI
    y = np.asarray(y, float)
    single = y.ndim == 1
    if single:
        y = y[:, None]
    h = stride * dt
    n = len(y)
    out = np.zeros_like(y)
    m = stride

    def sh(k):
        idx = np.clip(np.arange(n) + k * m, 0, n - 1)
        return y[idx]

    if order == 1:
        out = (sh(1) - sh(-1)) / (2 * h)
        pad = m
    elif order == 2:
        out = (sh(1) - 2 * y + sh(-1)) / h ** 2
        pad = m
    elif order == 3:
        out = (sh(2) - 2 * sh(1) + 2 * sh(-1) - sh(-2)) / (2 * h ** 3)
        pad = 2 * m
    elif order == 4:
        out = (sh(2) - 4 * sh(1) + 6 * y - 4 * sh(-1) + sh(-2)) / h ** 4
        pad = 2 * m
    else:
        raise ValueError(order)
    if pad:
        out[:pad] = out[pad]
        out[-pad:] = out[-pad - 1]
    return out[:, 0] if single else out


# ---------------------------------------------------------------------------
# comparison 1 -- hover at a setpoint, swing the arm
# ---------------------------------------------------------------------------

class HoverArmSwing(_Task):
    """Base pinned at (0, 0, 1); the arm sweeps its fold beta = q2 + q3.

    The base reference is CONSTANT for both controllers, so the metric is
    simply "how far does the floating base move while the arm swings".  The
    two stacks see the disturbance differently by design: the whole-body law
    is told the CoM must move (x_cd = x_b + R0 r_0c(q_d(t)) -- it knows the
    arm), while the geometric+L1 law is told only that the base must stay
    put and has to reject the arm's reaction.  That IS the comparison.

    The fold is swept symmetrically (q2 = q3), q1 = q4 = 0, so the excitation
    is a clean in-plane pitch/heave disturbance rather than a mixed one.
    Measured on this model over beta 80 -> 40 deg: the system CoM travels
    10 mm vertically and 0.7 mm horizontally while the EE travels 150 mm
    vertically -- the arm is light next to 3.75 kg, so most of what the base
    feels is the inertial reaction, not the static CoM shift.
    """

    name = "hover_arm_swing"
    title = "1. hover + arm fold sweep"
    base_home = np.array([0.0, 0.0, 1.0])
    lead_s = 6.0
    move_s = 24.0
    trail_s = 10.0
    #: fold half-angle at the extreme of the sweep [rad]; home is 40 deg, so
    #: 20 deg means beta swings 80 -> 40 -> 80 deg.
    fold_min_deg = 20.0
    cycles = 2.0

    def base_and_psi(self, t):
        return self.base_home.copy(), self.psi_home

    def joints(self, t, q_prev):
        u = self._u(t)
        a = 0.5 * (np.deg2rad(40.0) - np.deg2rad(self.fold_min_deg))
        h = np.deg2rad(40.0) - a * (1.0 - np.cos(2.0 * np.pi * self.cycles * u))
        return np.array([0.0, h, h, 0.0]), True, ""


TASKS["hover_arm_swing"] = HoverArmSwing


# ---------------------------------------------------------------------------
# comparison 2 -- base flies a small circle, EE pinned in the world
# ---------------------------------------------------------------------------

class CircleEeHold(_Task):
    """Base flies a small circle AND yaws, while the EE holds one world pose
    (3 position + 1 heading).

    HOW BIG CAN THIS BE?  Measured, not assumed -- and the answer is the
    headline result of this test.  Pinning all four EE DOFs leaves the base
    only the arm's null space, and for this manipulator that is a thin set:

      * radial (along the arm) the reach is a FOLD, so extending it also drops
        the EE; at a fixed base height only ~5 cm of radial travel keeps the
        pinned point inside the joint limits and the certified singularity
        margin.  Circles larger than R = 25 mm leave it -- verified by running
        the IK around the whole circle.
      * lateral is much freer (q1 gives +-35 deg of azimuth), which is why the
        circle is round rather than a slit.
      * yaw is capped near +-20..25 deg, and NOT by the airframe: locking the
        EE heading forces q1 ~ -psi, and q1's stop is +-35 deg.  q4 cannot
        take over, because its authority over the EE AZIMUTH scales with
        cos(beta) -- at the folded home pose (beta = 80 deg) that is 0.17, so
        undoing 1 rad of q1 would need ~5.7 rad of wrist.  This is exactly the
        bound poly_whole's pinned phase reported.

    That last point is why the task does NOT run at the folded home pose: it
    UNFOLDS to beta = 60 deg first (where cos(beta) = 0.5 and the wrist has
    real authority), pins there, and refolds at the end.  The unfold/refold
    segments are prescribed joint motion with an FK end-effector reference, so
    they are always feasible; only the pinned segment uses IK.

    The pinned window is what the EE-error ball is measured over; the
    surrounding segments are logged but reported separately.
    """

    name = "circle_ee_hold"
    title = "2. base circle + yaw, EE pinned in the world"
    psi_home = 0.0
    hover_z = 1.20
    #: fold the pinned segment runs at [deg of q2 = q3]; beta = 2x this
    work_half_deg = 30.0
    unfold_s = 8.0
    settle_s = 4.0
    move_s = 40.0
    trail_s = 8.0
    radius = 0.025          # [m]
    revolutions = 2.0
    yaw_amp_deg = 20.0
    #: circle centre offset along the arm (actual +x).  The feasible radial
    #: band is not centred on the arm's nominal reach, so the circle is
    #: shifted to sit inside it symmetrically.
    centre_shift = 0.010    # [m]

    def __init__(self, **kw):
        # lead_s/trail_s of the base class are unused here: this task has its
        # own segment schedule.
        self.lead_s = 0.0
        super().__init__(**kw)
        self.q_work = np.array([0.0, np.deg2rad(self.work_half_deg),
                                np.deg2rad(self.work_half_deg), 0.0])
        centre = np.array([self.centre_shift, 0.0, self.hover_z])
        self._centre = centre
        self._th0 = -0.5 * np.pi
        # start the circle at its bottom so the hover setpoint the mission
        # takes off to is ON the circle -- no lead-in move is needed.
        self.base_home = centre + self.radius * np.array(
            [np.cos(self._th0), np.sin(self._th0), 0.0])
        rz = _Rz(self.psi_home + PHI_OF_PSI0)
        _, r0e_w, re_w = TP.arm_fk_model(self.q_work, self.params)
        #: the pinned world EE pose -- exactly where the EE lands at the end of
        #: the unfold, so the pinned segment starts with zero EE error.
        self.p_e = self.base_home + rz @ r0e_w
        b = rz @ re_w[:, 0]
        self.az_e = float(np.arctan2(b[1], b[0]))

    # -- segment schedule -------------------------------------------------
    @property
    def t_pin0(self):
        return self.unfold_s + self.settle_s

    @property
    def t_pin1(self):
        return self.t_pin0 + self.move_s

    @property
    def duration(self):
        return (self.unfold_s + self.settle_s + self.move_s
                + self.settle_s + self.unfold_s + self.trail_s)

    def marks(self):
        t = 0.0
        out = []
        for nm, dur in (("unfold", self.unfold_s), ("settle", self.settle_s),
                        ("pinned", self.move_s), ("settle2", self.settle_s),
                        ("refold", self.unfold_s), ("trail", self.trail_s)):
            out.append((nm, t, t + dur))
            t += dur
        return out

    def base_and_psi(self, t):
        if t <= self.t_pin0 or t >= self.t_pin1:
            return self.base_home.copy(), self.psi_home
        u = float(smoothstep((t - self.t_pin0) / self.move_s))
        th = self._th0 + 2.0 * np.pi * self.revolutions * u
        xb = self._centre + self.radius * np.array(
            [np.cos(th), np.sin(th), 0.0])
        psi = self.psi_home + np.deg2rad(self.yaw_amp_deg) * np.sin(
            th - self._th0)
        return xb, psi

    def joints(self, t, q_prev):
        if t <= self.unfold_s:
            u = float(smoothstep(t / self.unfold_s))
            return Q_HOME + u * (self.q_work - Q_HOME), True, ""
        if t <= self.t_pin0:
            return self.q_work.copy(), True, ""
        if t < self.t_pin1:
            xb, psi = self.base_and_psi(t)
            q, info = TP.ik_world(self.params, xb, psi + PHI_OF_PSI0,
                                  self.p_e, self.az_e, q_prev)
            if not info["ok"]:
                return q, False, f"IK not ok: {info.get('reason', '')}"
            return q, True, ""
        t2 = t - self.t_pin1
        if t2 <= self.settle_s:
            return self.q_work.copy(), True, ""
        u = float(smoothstep((t2 - self.settle_s) / self.unfold_s))
        return self.q_work + u * (Q_HOME - self.q_work), True, ""


TASKS["circle_ee_hold"] = CircleEeHold


# ---------------------------------------------------------------------------
# comparison 3 -- both move: figure-8 base, EE bobbing relative to the drone
# ---------------------------------------------------------------------------

class Figure8EeUpDown(_Task):
    """Base flies a Gerono figure-8 in x-y while the arm bobs the EE up and
    down relative to the drone.

    Both halves move, so this is the tracking test rather than a
    disturbance-rejection test.  The EE reference is forward kinematics of
    the commanded base path and the commanded fold, which is what makes the
    up-and-down "compatible" -- it is exactly the motion the arm can produce
    from where the base is, at every instant, with no IK and no feasibility
    question.

    The figure-8 is the same Gerono lemniscate the flight catalogue's
    figure8_drone uses (x = A sin th, y = (B/2) sin 2th), flown at constant
    heading: the EE-heading channel is left to the arm so the two stacks are
    not additionally separated by yaw-axis fidelity, which is this
    simulator's known-weak axis.
    """

    name = "figure8_ee_updown"
    title = "3. figure-8 base + EE up/down"
    base_home = np.array([0.0, 0.0, 1.2])
    lead_s = 6.0
    move_s = 32.0
    trail_s = 10.0
    amp_x = 0.50            # [m] half-span along x
    amp_y = 0.30            # [m] full span across y
    laps = 1.0
    arm_cycles = 3.0
    fold_min_deg = 25.0     # home is 40 deg -> beta 80 -> 50 deg

    def base_and_psi(self, t):
        u = self._u(t)
        th = 2.0 * np.pi * self.laps * u
        xb = self.base_home + np.array([
            self.amp_x * np.sin(th),
            0.5 * self.amp_y * np.sin(2.0 * th),
            0.0])
        return xb, self.psi_home

    def joints(self, t, q_prev):
        u = self._u(t)
        a = 0.5 * (np.deg2rad(40.0) - np.deg2rad(self.fold_min_deg))
        h = np.deg2rad(40.0) - a * (
            1.0 - np.cos(2.0 * np.pi * self.arm_cycles * u))
        return np.array([0.0, h, h, 0.0]), True, ""


TASKS["figure8_ee_updown"] = Figure8EeUpDown


# ---------------------------------------------------------------------------
# offline checks
# ---------------------------------------------------------------------------

def feasibility_report(d):
    """Joint limits, singularity margin, commanded rates -- the things that
    make a task unflyable before any controller is involved."""
    q, qd = d["q"], d["qdot_d"]
    lo = np.degrees(TP.Q_MIN)
    hi = np.degrees(TP.Q_MAX)
    qdeg = np.degrees(q)
    out = {
        "all_ik_ok": bool(np.all(d["feasible"])),
        "q_min_deg": qdeg.min(axis=0), "q_max_deg": qdeg.max(axis=0),
        "limit_lo_deg": lo, "limit_hi_deg": hi,
        "limit_ok": bool(np.all(qdeg >= lo - 1e-6) and
                         np.all(qdeg <= hi + 1e-6)),
        "sigma_nd_min": float(d["sigma_nd"].min()),
        "sigma_ok": bool(d["sigma_nd"].min() >= TP.SIGMA_ND_MARGIN),
        "peak_qdot_dps": np.degrees(np.abs(qd).max(axis=0)),
        "peak_base_speed": float(np.linalg.norm(d["x_b_dot"], axis=1).max()),
        "peak_base_accel": float(np.linalg.norm(d["x_b_ddot"], axis=1).max()),
        "peak_yaw_rate_dps": float(np.degrees(
            np.abs(_fd(d["psi"], d["dt"], 1)).max())),
        "ee_span_m": (d["r_ed"].max(axis=0) - d["r_ed"].min(axis=0)),
        "com_span_m": (d["x_cd"].max(axis=0) - d["x_cd"].min(axis=0)),
        "base_span_m": (d["x_b"].max(axis=0) - d["x_b"].min(axis=0)),
    }
    #: the servo emulation in 05 rate-limits its reference at 0.5 rad/s; a
    #: commanded joint rate above that would be silently distorted on the L1
    #: side only, which would be a comparison artifact rather than a result.
    out["servo_slew_ok"] = bool(np.abs(qd).max() < 0.5)
    return out


def fd_consistency_report(d):
    """Every derivative must be the derivative of the field it belongs to.

    Each published array is recomputed DIRECTLY from its base field with a
    DIFFERENT stride and the matching stencil order -- never by
    re-differentiating another published derivative, which would compound one
    stencil's noise into the check and make a clean 4th difference look
    broken.  A stencil bug therefore cannot agree with itself.
    """
    dt = d["dt"]
    res = {}
    checks = [("x_cd", 1, "x_cd_dot"), ("x_cd", 2, "x_cd_ddot"),
              ("x_cd", 3, "x_cd_d3"), ("x_cd", 4, "x_cd_d4"),
              ("r_ed", 1, "r_ed_dot"), ("r_ed", 2, "r_ed_ddot"),
              ("b1_d", 1, "b1_d_dot"), ("b1_d", 2, "b1_d_ddot"),
              ("b1_de", 1, "b1_de_dot"), ("b1_de", 2, "b1_de_ddot"),
              ("q", 1, "qdot_d"),
              ("x_b", 1, "x_b_dot"), ("x_b", 2, "x_b_ddot")]
    k = 4 * FD_STRIDE_HI
    for base, order, field in checks:
        stride = (FD_STRIDE + 2) if order <= 2 else (FD_STRIDE_HI + 3)
        ref = _fd(d[base], dt, order, stride=stride)
        err = float(np.abs(ref[k:-k] - d[field][k:-k]).max())
        scale = max(float(np.abs(d[field][k:-k]).max()), 1e-12)
        res[field] = (err, err / scale)
    # A channel whose true derivative is identically zero (a constant
    # heading, a pinned base) has no meaningful RELATIVE error -- its scale is
    # roundoff.  Judge those on the absolute number instead, or a 1e-12
    # residual reads as "100% wrong".
    res["worst_rel"] = max(v[1] for v in res.values() if v[0] > 1e-6)
    res["b1_d_unit"] = float(np.abs(
        np.linalg.norm(d["b1_d"], axis=1) - 1.0).max())
    res["b1_de_unit"] = float(np.abs(
        np.linalg.norm(d["b1_de"], axis=1) - 1.0).max())
    return res


def _selftest():
    ok = True
    for name, cls in TASKS.items():
        print(f"\n=== {name} ===")
        task = cls()
        d = task.build(verbose=True)
        f = feasibility_report(d)
        print(f"  duration {d['t'][-1]:.1f} s, {len(d['t'])} samples")
        print(f"  q range   {np.round(f['q_min_deg'],1)} .. "
              f"{np.round(f['q_max_deg'],1)} deg  "
              f"(limits {np.round(f['limit_lo_deg'],0)} .. "
              f"{np.round(f['limit_hi_deg'],0)})")
        print(f"  peak |qdot| {np.round(f['peak_qdot_dps'],1)} deg/s   "
              f"servo-slew ok: {f['servo_slew_ok']}")
        print(f"  sigma_nd min {f['sigma_nd_min']:.3f} "
              f"(margin {TP.SIGMA_ND_MARGIN})")
        print(f"  base span {np.round(f['base_span_m'],3)} m, "
              f"peak |v| {f['peak_base_speed']:.3f} m/s, "
              f"|a| {f['peak_base_accel']:.3f} m/s^2, "
              f"yaw rate {f['peak_yaw_rate_dps']:.1f} deg/s")
        print(f"  EE span   {np.round(f['ee_span_m'],3)} m   "
              f"CoM span {np.round(f['com_span_m'],4)} m")
        print(f"  attitude correction: CoM moved {d['picard'][-1]*1000:.3f} mm "
              f"vs the level-frame reference; peak reference tilt "
              f"{d['ref_tilt_deg'].max():.2f} deg")
        c = fd_consistency_report(d)
        worst = c["worst_rel"]
        print(f"  FD consistency worst relative error {worst:.2e}; "
              f"|b1_d|-1 {c['b1_d_unit']:.1e}, "
              f"|b1_de|-1 {c['b1_de_unit']:.1e}")
        bad = []
        if not f["all_ik_ok"]:
            bad.append("IK infeasible samples")
        if not f["limit_ok"]:
            bad.append("joint limits")
        if not f["sigma_ok"]:
            bad.append("singularity margin")
        if not f["servo_slew_ok"]:
            bad.append("commanded joint rate over the servo slew limit")
        if worst > 5e-3:
            bad.append(f"FD inconsistency {worst:.1e}")
        # start and end must be exactly the hover/home state
        if np.linalg.norm(d["q"][0] - Q_HOME) > 1e-9:
            bad.append("does not start at HOME")
        if np.linalg.norm(d["x_b"][0] - d["base_home"]) > 1e-9:
            bad.append("does not start at the hover setpoint")
        if np.abs(d["x_cd_dot"][0]).max() > 1e-6:
            bad.append("does not start at rest")
        if bad:
            ok = False
            print("  FAIL: " + "; ".join(bad))
        else:
            print("  PASS")
    print("\nSELF-TEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(_selftest())

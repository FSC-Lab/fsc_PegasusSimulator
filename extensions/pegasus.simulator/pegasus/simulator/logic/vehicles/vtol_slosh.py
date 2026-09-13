"""
| File: vtol_slosh.py
| Description: The VTOL airframe with a free-surface liquid tank.  This is the
|              copy the tank work happens on; `vtol.py` is the flying baseline
|              and stays untouched.
|
| ============================================================================
|  WHY A SUBCLASS AND NOT A FORK OF vtol.py
| ============================================================================
|
|  The ask was "a new copy of the vtol model", and this is one -- a new module,
|  new classes, selected explicitly, with the baseline model unmodified.  What
|  it deliberately is NOT is a byte copy of vtol.py's 1268 lines.
|
|  This project's most expensive bugs are all the same shape: a second copy of a
|  model that drifts from the first.  The controllers carried their own aero and
|  flew the wrong plant for a month (the FLU/FRD frame fix, 2026-09-06); the
|  inertia was the body link's, not the assembled vehicle's; CQ was half the
|  sim's value.  A forked plant would inherit every one of those bugs the day
|  someone fixes them in vtol.py and not here -- and slosh results measured on a
|  stale plant are worth nothing.
|
|  So: the shared physics has ONE definition, in vtol.py, and this class adds
|  exactly the tank.  If you later WANT the plant frozen (e.g. to compare two
|  plants side by side rather than to develop the tank), fork it then -- from
|  the `pre-slosh` tag, which is what that tag is for.
|
| ============================================================================
|  WHAT IS AND IS NOT IMPLEMENTED  (phase 1 of the tank plan)
| ============================================================================
|
|    phase 1  THIS FILE: the copy exists, is selectable, and is behaviourally
|             IDENTICAL to the baseline.  `tests/test_slosh_scaffold.py` proves
|             that, and a configured tank REFUSES TO CONSTRUCT rather than fly
|             physics that does not exist yet.
|    phase 2  tank as DEAD MASS: fold TankConfig.fluid_mass into the mass, CG
|             and inertia -- and propagate to the controllers in the SAME
|             commit (`so3.M` is 3.45; if the plant and the control model
|             disagree the reference stops being a trim).
|    phase 3  the pendulum itself, in `_apply_slosh` below.
|    phase 4  into the offline gate, swept over fill and zeta, BEFORE any flight.
|
|  The refusal in `__init__` is the important part of phase 1.  A tank that is
|  configured but not modelled is exactly the silent-wrong-physics failure this
|  codebase keeps paying for, so it is made loud and it happens at CONSTRUCTION
|  -- not on the first physics step, with the vehicle already in the air.
"""

import math
import os

import numpy as np

from pegasus.simulator.logic.dynamics.slosh import (        # noqa: F401 (re-exported)
    TankConfig,
    SloshParameters,
    PendulumSlosh,
    slosh_parameters,
    describe as describe_tank,
)
from pegasus.simulator.logic.vehicles.vtol import VTOL, VTOLConfig

from dataclasses import dataclass, replace


# ============================================================================
#  CONFIG
# ============================================================================

@dataclass
class VTOLSloshConfig(VTOLConfig):
    """`VTOLConfig` plus a tank.  Every other field, and every default, is
    inherited -- so a new field added to the baseline config reaches this one
    with no edit here, and the two cannot drift.

    `tank=None` means "no tank", and then this config describes exactly the
    baseline aircraft."""

    tank: TankConfig | None = None


# ============================================================================
#  MASS PROPERTIES
# ============================================================================
#
#  The plant describes the airframe as a BODY LINK plus ten small links (two per
#  tiltable rotor, one per fixed rotor, one per surface).  `VTOLConfig.mass`,
#  `.inertia` and `.body_com_offset` are the BODY LINK's; what the controllers
#  model -- and what flies -- is the ASSEMBLY.
#
#  Getting that distinction wrong is this project's most expensive single bug
#  class: the controllers carried the BODY LINK's inertia (0.20, 0.28, 0.35) as
#  if it were the vehicle's for a month (Ixx -24%, Izz -29%), and a CG taken from
#  the rotor centroid instead of the assembly put a 0.6 N.m nose-up moment into
#  every hover.  So the tank is added THROUGH the assembly, and
#  `assembled_mass_properties` is checked against the controllers' own constants
#  in `tests/test_slosh_import.py`: it reproduces M = 3.450 kg, CG_X = 0.1764 m
#  and I = (0.265, 0.303, 0.490) exactly.


def _parallel_axis(m, d):
    """Diagonal inertia of a point mass `m` at offset `d` about the reference."""
    d = np.asarray(d, dtype=float)
    return np.array([m * (d[1] ** 2 + d[2] ** 2),
                     m * (d[0] ** 2 + d[2] ** 2),
                     m * (d[0] ** 2 + d[1] ** 2)])


def _link_positions(cfg):
    """Where `_init_prim` puts each small link, in body-link-origin coordinates.

    Mirrors the builder exactly -- nacelle + prop for a tiltable rotor, a single
    prop link for a fixed one, one link per surface -- because a link counted
    here and not built there (or the reverse) is a silent mass error."""
    pts = []
    for rotor in cfg.rotors:
        p = np.asarray(rotor.position, dtype=float)
        pts.append(p)
        if rotor.tiltable:
            pts.append(p + np.asarray(rotor.spin_offset, dtype=float))
    for surface in cfg.surfaces:
        pts.append(np.asarray(surface.position, dtype=float))
    return pts


def assembled_mass_properties(cfg):
    """(total mass [kg], CG [m], diagonal inertia about that CG [kg m^2]).

    Body-link-origin coordinates, the same frame as `body_com_offset`."""
    pts = _link_positions(cfg)
    m_b = float(cfg.mass)
    c_b = np.asarray(cfg.body_com_offset, dtype=float)
    i_b = np.asarray(cfg.inertia, dtype=float)
    i_l = np.asarray(cfg.link_inertia, dtype=float)
    m_l = float(cfg.link_mass)

    mass = m_b + m_l * len(pts)
    cg = (m_b * c_b + m_l * sum(pts)) / mass if pts else c_b
    inertia = i_b + _parallel_axis(m_b, c_b - cg)
    for p in pts:
        inertia = inertia + i_l + _parallel_axis(m_l, p - cg)
    return mass, cg, inertia


def fold_tank_into_body(cfg, tank):
    """A COPY of `cfg` whose BODY LINK also carries the tank's liquid.

    The liquid has to go onto a LINK, because that is what PhysX integrates -- so
    the body link's mass, CoM offset and inertia all move together.  Returns the
    new config and the (before, after) assembly properties for logging.

    A COPY, never a mutation: configs are shared (`fsc_cad_vtol_slosh_config`
    hands out the same nested rotor/surface objects as the baseline airframe),
    and quietly rewriting the caller's config is how one vehicle ends up changing
    another's mass."""
    m_f, cg0, i0 = assembled_mass_properties(cfg)
    params = slosh_parameters(tank)
    m_w = params.fluid_mass
    # None -> at the assembled CG: adds mass without moving the CG.
    p_t = np.asarray(tank.position, dtype=float) if tank.position is not None else cg0

    m_b = float(cfg.mass)
    c_b = np.asarray(cfg.body_com_offset, dtype=float)
    i_b = np.asarray(cfg.inertia, dtype=float)

    m_b2 = m_b + m_w
    c_b2 = (m_b * c_b + m_w * p_t) / m_b2
    i_b2 = (i_b + _parallel_axis(m_b, c_b - c_b2)
            + np.asarray(params.inertia_rigid, dtype=float) + _parallel_axis(m_w, p_t - c_b2))

    new_cfg = replace(cfg,
                      mass=float(m_b2),
                      body_com_offset=tuple(float(v) for v in c_b2),
                      inertia=tuple(float(v) for v in i_b2))
    return new_cfg, (m_f, cg0, i0), assembled_mass_properties(new_cfg)


# ============================================================================
#  VEHICLE
# ============================================================================

class VTOLSlosh(VTOL):
    """The baseline VTOL with a free-surface tank.

    With `config.tank = None` (the default) this is the baseline aircraft, step
    for step: `update()` adds one call, and that call returns immediately.
    """

    def __init__(
        self,
        vehicle_id: int = 0,
        init_pos=(0.0, 0.0, 0.5),
        init_orientation=(0.0, 0.0, 0.0, 1.0),
        config=None,
    ):
        # CLAUDE.md: default config argument must be None, never a constructed
        # config -- Python evaluates default arguments once at import time.
        if config is None:
            config = VTOLSloshConfig()

        tank = getattr(config, "tank", None)
        self._tank: TankConfig | None = tank
        self._slosh: SloshParameters | None = (
            slosh_parameters(tank) if tank is not None else None
        )
        self._mass_props = None          # (mass, cg, inertia) once a tank is folded in

        self._pend: PendulumSlosh | None = None
        self._tank_pos = None
        self._slosh_dbg_ctr = 0

        if tank is not None and tank.enabled:
            # PHASE 3: the free surface as a pendulum. The whole liquid stays folded
            # into the body as dead mass (phase 2); the pendulum adds ONLY the force
            # of the bob's motion relative to the airframe, -m1 * x1_ddot, which is
            # zero in every steady state -- see slosh.PendulumSlosh for the algebra.
            if tank.slosh:
                self._pend = PendulumSlosh(self._slosh)

            config, before, after = fold_tank_into_body(config, tank)
            # where the slosh force acts: the tank (None -> the assembled CG), in the
            # body-link-origin frame apply_force expects
            self._tank_pos = [float(v) for v in (tank.position if tank.position is not None
                                                  else after[1])]
            self._mass_props = after
            m0, c0, i0 = before
            m1, c1, i1 = after
            stall = 14.13 * math.sqrt(m1 / m0)
            msg = (f"[TANK] {describe_tank(tank)}  --  "
                   f"{'SLOSH (phase 3, pendulum)' if tank.slosh else 'DEAD MASS (phase 2)'}\n"
                   f"[TANK] dead mass (phase 2): {m0:.3f} -> {m1:.3f} kg "
                   f"(+{100 * (m1 / m0 - 1):.1f}%), "
                   f"CG ({c0[0]:.4f},{c0[1]:.4f},{c0[2]:.4f}) -> "
                   f"({c1[0]:.4f},{c1[1]:.4f},{c1[2]:.4f}) m "
                   f"[{1000 * float(np.linalg.norm(c1 - c0)):.2f} mm], "
                   f"I ({i0[0]:.3f},{i0[1]:.3f},{i0[2]:.3f}) -> "
                   f"({i1[0]:.3f},{i1[1]:.3f},{i1[2]:.3f})\n"
                   f"[TANK] stall 14.13 -> {stall:.2f} m/s against a 16.0 m/s cruise; "
                   f"hover needs {m1 * 9.81:.2f} N of 72 N installed. "
                   f"THE CONTROLLERS MUST CARRY THE SAME MASS (so3.M) or the "
                   f"reference stops being a trim.")
            print(msg)
            try:                                  # Isaac's own log, when we are in it
                import carb
                carb.log_warn(msg)
            except Exception:                     # noqa: BLE001 - headless tests
                pass

        super().__init__(
            vehicle_id=vehicle_id,
            init_pos=init_pos,
            init_orientation=init_orientation,
            config=config,
        )

    # ========================================================================
    #  PER-STEP
    # ========================================================================

    def update(self, dt: float):
        """Baseline step, then the tank.

        The slosh step is appended AFTER `VTOL.update` rather than spliced
        between its propulsion and aerodynamics blocks, and that is
        deliberate: every `apply_force` in a step ACCUMULATES on the link and
        is integrated by the single PhysX step that follows, so where in the
        step the force is added makes no difference to the physics -- while
        splicing would mean copying the whole orchestration and owning a second
        copy of it forever.  The backends ticked by `VTOL.update` publish the
        state from the PREVIOUS integration, so they are unaffected too.
        """
        super().update(dt)

        if self._cfg.enable_forces:
            self._apply_slosh(dt)

    def _apply_slosh(self, dt: float):
        """STEP 3 -- the tank's free surface, as the pendulum analogue.

        The excitation is the vehicle's SPECIFIC FORCE in the body frame,
        f = R^T (a_world - g_vec): what an accelerometer bolted to the tank reads.
        `State.linear_acceleration` is the plant's finite-difference of its own
        world velocity (one physics step old, which at 250 Hz is 4 ms against a
        300 ms slosh period). The bob is integrated in the body frame and the
        force it adds beyond dead mass, -m1 * x1_ddot, is applied at the tank.

        No pendulum (tank=None, enabled=False, or slosh=False) -> nothing is
        applied and the aircraft is exactly the phase-2 dead-mass one.
        """
        if self._pend is None:
            return
        st = self._state
        try:
            a_w = np.asarray(st.linear_acceleration, dtype=float)
            q = np.asarray(st.attitude, dtype=float)              # x, y, z, w  body -> world
        except Exception:                                          # noqa: BLE001 - first steps
            return
        if not np.all(np.isfinite(a_w)) or not np.all(np.isfinite(q)):
            return
        f_w = a_w + np.array([0.0, 0.0, 9.81])                    # ENU: g_vec = (0, 0, -g)
        # R^T f_w without scipy: rotate by the conjugate quaternion
        x, y, z, w = q
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        f_b = R.T @ f_w
        # the tank's own motion: body rates from the state, their derivative by finite
        # difference, and the arm from the ASSEMBLED CG (the fold's), not the link origin
        try:
            w_b = np.asarray(st.angular_velocity, dtype=float)
        except Exception:                                          # noqa: BLE001
            w_b = np.zeros(3)
        w_prev = getattr(self, "_slosh_w_prev", None)
        w_dot = (w_b - w_prev) / float(dt) if (w_prev is not None and dt > 0.0) else np.zeros(3)
        self._slosh_w_prev = w_b.copy()
        r_tank = np.asarray(self._tank_pos, dtype=float) - np.asarray(self._mass_props[1], dtype=float)
        F = self._pend.step(f_b, float(dt), omega=w_b, omega_dot=w_dot, r_tank=r_tank)
        if abs(F[0]) > 1e-9 or abs(F[1]) > 1e-9:
            self.apply_force([float(F[0]), float(F[1]), 0.0], pos=self._tank_pos, body_part="/body")
        # bring-up diagnostic, like [PROP]/[AERO]: PEGASUS_SLOSH_DEBUG=1 prints ~2 Hz
        self._slosh_dbg_ctr += 1
        if os.environ.get("PEGASUS_SLOSH_DEBUG") and self._slosh_dbg_ctr % 120 == 0:
            try:
                import carb
                carb.log_warn(f"[SLOSH] f_b=({f_b[0]:+.2f},{f_b[1]:+.2f},{f_b[2]:+.2f}) "
                              f"bob=({1000 * self._pend.x[0]:+.1f},{1000 * self._pend.x[1]:+.1f}) mm "
                              f"F=({F[0]:+.2f},{F[1]:+.2f}) N  f1={self._pend.omega(f_b[2]) / (2 * math.pi):.2f} Hz")
            except Exception:                                      # noqa: BLE001
                pass

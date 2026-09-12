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

from pegasus.simulator.logic.dynamics.slosh import (        # noqa: F401 (re-exported)
    TankConfig,
    SloshParameters,
    slosh_parameters,
    describe as describe_tank,
)
from pegasus.simulator.logic.vehicles.vtol import VTOL, VTOLConfig

from dataclasses import dataclass


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

        if tank is not None and tank.enabled:
            p = slosh_parameters(tank)
            raise NotImplementedError(
                "VTOLSlosh: a tank is configured but the slosh physics is not "
                "implemented yet (tank plan phase 2 = dead mass, phase 3 = "
                "pendulum).\n"
                f"    {describe_tank(tank)}\n"
                f"    it would add {p.fluid_mass:.3f} kg, taking the stall speed "
                f"from 14.13 to {14.13 * math.sqrt((3.45 + p.fluid_mass) / 3.45):.2f} m/s "
                "against a 16.0 m/s cruise.\n"
                "Refusing to construct rather than flying an aircraft whose "
                "payload the physics does not know about.  Set tank=None to fly "
                "the baseline aircraft from this class."
            )

        self._tank: TankConfig | None = tank
        self._slosh: SloshParameters | None = (
            slosh_parameters(tank) if tank is not None else None
        )

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
        """STEP 3 -- the tank reaction.  Nothing here yet (phase 1).

        What goes here in phase 3:

          * integrate a 2-DOF pendulum (x and y) of mass `self._slosh.slosh_mass`
            on length `self._slosh.pendulum_length`, driven by the body's linear
            acceleration and rotation, damped at `self._slosh.zeta`;
          * apply the reaction force at `self._tank.position` via
            `apply_force(..., body_part="")`, the same call `_apply_propulsion`
            and `_apply_aerodynamics` use;
          * the rigid remainder `self._slosh.rigid_mass` needs NO force here --
            it rides with the airframe and belongs in the mass properties
            (phase 2), which is why the phases are in that order.

        It is a no-op, not an abstract method, so that the scaffold is provably
        identical to the baseline today.
        """
        return

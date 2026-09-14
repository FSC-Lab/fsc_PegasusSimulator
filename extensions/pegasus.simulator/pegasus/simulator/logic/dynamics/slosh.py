"""
| File: slosh.py
| Description: Free-surface liquid tank -- geometry, and the EQUIVALENT MECHANICAL MODEL
|              parameters derived from it.  Pure maths: no Isaac, no pxr, no omni.
|
| ============================================================================
|  WHY THIS FILE IS SEPARATE FROM THE VEHICLE
| ============================================================================
|
|  `vtol.py` cannot be imported outside Isaac Sim (it pulls `pxr` and
|  `omni.isaac.dynamic_control`).  Everything in this project that needed to
|  check the plant's maths headlessly therefore RE-DERIVED it in the test --
|  see `tests/test_actuator_models.py`, which re-implements the thrust model
|  from the same formulas.  That works, and it is also precisely the pattern
|  that produced this project's most expensive bugs: a second copy of a model
|  that silently drifts from the first (the FLU/FRD aero frame, the body-link
|  inertia, CQ at half value, the controllers' own `build_ode`).
|
|  So the slosh maths lives HERE, where a plain `python3` can import it by file
|  path and test the SHIPPED code rather than a paraphrase of it.  The vehicle
|  (`vehicles/vtol_slosh.py`) consumes this module; the tests consume it too;
|  and if a controller ever needs to know the slosh frequency, it reads the same
|  function.  One definition.
|
| ============================================================================
|  THE MODEL, AND WHY IT IS NOT A FLUID SIMULATION
| ============================================================================
|
|  A PhysX particle fluid cannot run at this loop's rate, and coupling it back
|  into the rigid body is fragile.  Real aircraft and launch-vehicle GNC use an
|  EQUIVALENT MECHANICAL MODEL instead: the first antisymmetric slosh mode of a
|  vertical circular cylinder is exactly equivalent, in force and moment, to a
|  PENDULUM of mass m1 on a rod of length L1, plus a RIGID mass m0 that rides
|  with the tank.  Higher modes carry a few percent of the mass and are ignored.
|
|      first mode:   w1^2 = (g * e1 / R) * tanh(e1 * h / R),   e1 = 1.8412
|      slosh mass:   m1   = m_f * 2*tanh(x) / ((e1^2 - 1) * x),   x = e1*h/R
|      pendulum:     L1   = R / (e1 * tanh(x))          [ so that w1 = sqrt(g/L1) ]
|      remainder:    m0   = m_f - m1                    [ rigid, exact ]
|
|  (Abramson, NASA SP-106; the damping estimate follows Stephens/Mikishev.)
|
|  NOTE the frequency depends on the tank RADIUS, not on how much liquid is in
|  it: tanh saturates above h/R ~ 1.5, so a 75%-full and a 60%-full tank of the
|  same radius ring at the same pitch.  Fill changes the MASS, not the note.
|
| ============================================================================
|  WHAT THIS AIRFRAME CAN CARRY  (computed, not assumed -- see the tank plan)
| ============================================================================
|
|  MASS binds first.  Stall scales as sqrt(W) and cruise is 16 m/s:
|
|      +0.25 kg -> stall 14.63 m/s, margin +1.37     ok
|      +0.50 kg -> stall 15.12 m/s, margin +0.88     ok, the recommended cap
|      +1.00 kg -> stall 16.05 m/s, margin -0.05     INFEASIBLE, one litre does
|                                                    not fly on this aircraft
|
|  FREQUENCY is the other constraint, and it is comfortable.  The mode this
|  aircraft is marginal on is the 1.42 Hz short period; the default tank rings
|  at 3.32 Hz, 2.34x above it.  Hitting resonance would need a 45 cm tank on a
|  1.4 m aeroplane.  The two constraints happen to point the same way.
|
|  DAMPING is the thing to watch: a smooth wall gives zeta ~ 0.5%, a resonator
|  that takes ~23 cycles to halve, and at 3.3 Hz it sits below the elevator
|  servo corner (~5.3 Hz) -- i.e. INSIDE the control loop's reach.  `baffled()`
|  is here from the start so raising zeta is a config change, not a retrofit.
"""

# NOTE: deliberately NO `from __future__ import annotations`.  The sibling
# `vtol_aerodynamics.py` has none either, and both are loaded BY FILE PATH in
# the headless tests (importlib spec_from_file_location).  With the future
# import, @dataclass resolves its annotations through `sys.modules[__module__]`
# -- which a by-path load has not registered -- and the module fails to import
# with a bare AttributeError.  Keeping it out means the simplest possible
# loader works, exactly as it does for the aero model.
import math
import os
from dataclasses import dataclass, replace

# First root of J1'(x) -- the first antisymmetric mode of a circular tank.
EPS_1 = 1.8412
G = 9.81
NU_WATER = 1.0e-6          # kinematic viscosity [m^2/s] at ~20 C


# ============================================================================
#  CONFIG  -- pure data (CLAUDE.md: no live resources in a config)
# ============================================================================

@dataclass
class TankConfig:
    """A cylindrical tank with a free surface, mounted in the airframe.

    Defaults are the recommended baseline from the tank plan: the largest tank
    that keeps real cruise margin on this aircraft.  Changing `radius` or
    `fill_frac` changes the derived physics -- nothing downstream is hardcoded.
    """

    name: str = "tank"

    # --- geometry -------------------------------------------------------
    radius: float = 0.0414           # [m] 41 mm -> 3.32 Hz, clear of the short period
    height: float = 0.1242           # [m] H = 3R, a 0.67 L tank
    fill_frac: float = 0.75          # [-] 75% full -> 0.50 kg of water
    fluid_density: float = 1000.0    # [kg/m^3] water

    # --- mounting -------------------------------------------------------
    # Position of the tank's CENTRE OF FLUID in body-link-origin coordinates, the
    # same frame `VTOLConfig.body_com_offset` and `aero_center` are written in.
    #
    # None (the default) means AT THE ASSEMBLED VEHICLE CG -- not the body-link
    # origin.  Those differ by 176 mm on this airframe, and defaulting to the
    # origin would have hung half a kilogram on a 176 mm lever arm silently. That
    # is precisely the bug the 2026-09-06 audit found in `aero_center`, which used
    # to default to the body-link origin and thereby invented an aerodynamic
    # centre at 71% MAC.
    #
    # At the CG the tank adds mass WITHOUT moving the CG, so phase 2 changes one
    # thing rather than three, and the slosh force in phase 3 acts on the shortest
    # possible lever arm.
    position: tuple | None = None
    axis: tuple = (0.0, 0.0, 1.0)    # tank axis in body frame (vertical by default)

    # --- damping --------------------------------------------------------
    # None  -> the smooth-wall closed form (~0.5% here), which is what a bare
    #          plastic bottle does.
    # 0.02-0.10 -> a baffled tank.  Baffles are the standard fix if slosh turns
    #          out to interact with the loops.
    damping_zeta: float | None = None

    enabled: bool = True

    # Model the FREE SURFACE (the pendulum analogue) or treat the liquid as rigid?
    #   False (default) -> DEAD MASS: the liquid is frozen to the airframe. This is
    #                      tank-plan phase 2, and it is a real aircraft -- a full,
    #                      baffled-to-death tank behaves this way.
    #   True            -> phase 3, not implemented. `VTOLSlosh` REFUSES to construct
    #                      rather than pretend, because a tank that is configured but
    #                      not modelled is silent wrong physics.
    slosh: bool = False

    # WATER BY MASS instead of by fill fraction (the operator's number). Given, it sets the
    # depth in this radius of tank; if that is deeper than the tank, the tank grows to hold it
    # at the plan's 75% (a 0.7 kg fill needs a 0.93 L tank, the 0.67 L one holds 0.67 kg full).
    # None (default) keeps height x fill_frac = the plan's 0.502 kg, byte-identical to before.
    water_kg: float | None = None

    def __post_init__(self):
        if self.radius <= 0.0 or self.height <= 0.0:
            raise ValueError(f"tank '{self.name}': radius and height must be > 0")
        if self.water_kg is not None:
            if self.water_kg < 0.0:
                raise ValueError(f"tank '{self.name}': water_kg must be >= 0")
            depth = self.water_kg / (self.fluid_density * math.pi * self.radius ** 2)
            if depth > self.height:
                self.height = depth / 0.75
            self.fill_frac = depth / self.height
        if not 0.0 <= self.fill_frac <= 1.0:
            raise ValueError(f"tank '{self.name}': fill_frac must be in [0, 1]")
        if self.fluid_density <= 0.0:
            raise ValueError(f"tank '{self.name}': fluid_density must be > 0")
        if self.damping_zeta is not None and not 0.0 <= self.damping_zeta < 1.0:
            raise ValueError(f"tank '{self.name}': damping_zeta must be in [0, 1)")

    # -- convenience ------------------------------------------------------
    @property
    def fill_height(self) -> float:
        """Depth of liquid [m]."""
        return self.fill_frac * self.height

    @property
    def volume(self) -> float:
        """Tank volume [m^3] (not the liquid volume -- see `fluid_mass`)."""
        return math.pi * self.radius ** 2 * self.height

    @property
    def fluid_mass(self) -> float:
        """Total liquid mass [kg].  This is what the mass budget spends."""
        return self.fluid_density * math.pi * self.radius ** 2 * self.fill_height

    def baffled(self, zeta: float = 0.05) -> "TankConfig":
        """The same tank with baffles: identical geometry, higher damping.

        A copy, not a mutation, so a sweep can hold the baseline tank and the
        baffled one at the same time without one of them changing underfoot."""
        return replace(self, damping_zeta=zeta)


# ============================================================================
#  DERIVED PARAMETERS  -- the equivalent mechanical model
# ============================================================================

@dataclass(frozen=True)
class SloshParameters:
    """What the pendulum analogue needs, all derived from `TankConfig`.

    Frozen: these are consequences of the tank, not knobs.  Retune the tank."""

    fluid_mass: float        # [kg]  total liquid                 m_f = m0 + m1
    slosh_mass: float        # [kg]  first-mode (moving) mass     m1
    rigid_mass: float        # [kg]  remainder, rides with the airframe   m0
    pendulum_length: float   # [m]   L1, set so sqrt(g/L1) == omega
    omega: float             # [rad/s] first-mode natural frequency
    freq_hz: float           # [Hz]  the same number, for reading
    zeta: float              # [-]   damping ratio actually in force
    fluid_cg_height: float   # [m]   liquid centroid above the tank base
    inertia_rigid: tuple     # [kg m^2] (Ixx, Iyy, Izz) of the liquid column
    wall: float = 0.0        # [m]   bob travel limit: the liquid hits the wall (0 = none)
                             #          about its own centroid, treated as solid

    @property
    def slosh_mass_frac(self) -> float:
        return self.slosh_mass / self.fluid_mass if self.fluid_mass > 0 else 0.0

    @property
    def half_amplitude_cycles(self) -> float:
        """Cycles for a free oscillation to decay to half amplitude.  The blunt
        way to see whether a tank is a resonator (23 cycles) or a damper (2)."""
        if self.zeta <= 0.0:
            return float("inf")
        return math.log(2.0) / (2.0 * math.pi * self.zeta)


def smooth_wall_zeta(radius: float, nu: float = NU_WATER) -> float:
    """Damping ratio of a smooth-walled circular tank (Stephens/Mikishev).

        zeta ~ 0.79 * sqrt( nu / (R^1.5 * sqrt(g)) )

    ~0.5% at R = 41 mm.  Baffles or a ring damper take this to 2-10%, which is
    the standard fix and why `TankConfig.baffled()` exists."""
    return 0.79 * math.sqrt(nu / (radius ** 1.5 * math.sqrt(G)))


def slosh_parameters(tank: TankConfig) -> SloshParameters:
    """Equivalent-mechanical-model parameters for `tank`.

    An EMPTY tank (fill_frac = 0) returns an all-zero model with a zero
    frequency rather than a division by zero -- the fill sweep in the offline
    gate runs 0 / 25 / 50 / 75 / 100 %, so the endpoints have to be valid.

    CAVEAT for the 100% case: this is the model for a tank with a FREE SURFACE.
    A completely full, SEALED tank has no free surface and does not slosh at
    all; the formulas below do not know that.  Treat fill_frac = 1.0 as "vented
    and brim-full" or exclude it."""
    R, h = tank.radius, tank.fill_height
    m_f = tank.fluid_mass
    zeta = tank.damping_zeta if tank.damping_zeta is not None else smooth_wall_zeta(R)

    # Liquid column, treated as a rigid solid cylinder about its own centroid.
    # This is what phase 2 ("tank as dead mass") needs, and it is also the m0
    # contribution once the pendulum is live.
    ixx = iyy = m_f * (3.0 * R * R + h * h) / 12.0
    izz = m_f * R * R / 2.0

    if h <= 0.0 or m_f <= 0.0:
        return SloshParameters(
            fluid_mass=0.0, slosh_mass=0.0, rigid_mass=0.0,
            pendulum_length=0.0, omega=0.0, freq_hz=0.0, zeta=zeta,
            fluid_cg_height=0.0, inertia_rigid=(0.0, 0.0, 0.0),
        )

    x = EPS_1 * h / R
    t = math.tanh(x)

    omega = math.sqrt(G * EPS_1 / R * t)
    m1 = m_f * (2.0 * t) / ((EPS_1 * EPS_1 - 1.0) * x)
    L1 = R / (EPS_1 * t)                      # equals g / omega^2, checked in the tests

    return SloshParameters(
        fluid_mass=m_f,
        slosh_mass=m1,
        rigid_mass=m_f - m1,
        pendulum_length=L1,
        omega=omega,
        freq_hz=omega / (2.0 * math.pi),
        zeta=zeta,
        fluid_cg_height=0.5 * h,
        inertia_rigid=(ixx, iyy, izz),
        # ★ THE LIQUID HITS THE WALL. The equivalent pendulum is a LINEAR, small-amplitude
        # model; past a few tenths of the radius the wave breaks and the real force saturates
        # -- and an unlimited pendulum on a tumbling airframe integrates to 10^12 mm (gate,
        # 2026-09-13, an 80 mm nose tank). Half the radius is where the model's validity ends
        # and is used as an inelastic stop: the bob stops there, its kinetic energy is lost
        # (breaking), and the reaction force is what that arrest actually applies.
        wall=0.5 * R,
    )


# ONE environment variable selects the water mass on BOTH sides of the HIL boundary: the
# plant reads it here, the controllers read the same name (nmpc_so3_transition_node,
# lib/fsc_transition_ref, geometric_hover_node, vtol_nmpc: M = 3.449572 + VTOL_TANK_KG) and
# tests/test_tank_model_pairing.py holds them together for every mode. Two names would be a
# trap -- the launchers pass env INLINE on tmux send-keys lines, and a name forwarded to one
# pane and not the other flies a plant the controller does not model.
TANK_KG_ENV = "VTOL_TANK_KG"
DEFAULT_WATER_KG = 0.502          # the plan's tank: 75% of a 0.67 L, 41 mm cylinder
TANK_MODES = (0.1, 0.502, 0.7)    # the flown/gated masses; anything else is untested
# VTOL_TANK_R [m]: re-shape the tank at the SAME water mass -- the gate's --slosh_radius, for
# the scene. The plan's 41 mm tank moves 0.10 kg at 3.3 Hz (inert to this controller); 80 mm
# moves 0.38 kg at 1.7 Hz, 100 mm 0.41 kg at 1.1 Hz, below the 1.4 Hz short period. Unset or
# empty = the plan's 41 mm. Mount it off the CG with VTOL_TANK_DX to give the reaction a lever.
TANK_R_ENV = "VTOL_TANK_R"


def tank_kg_from_env() -> float:
    """The water mass asked for by VTOL_TANK_KG (default the plan's 0.502)."""
    v = os.environ.get(TANK_KG_ENV, "").strip()
    return float(v) if v else DEFAULT_WATER_KG


def tank_radius_from_env() -> float | None:
    """The tank radius asked for by VTOL_TANK_R [m], None = the plan's."""
    v = os.environ.get(TANK_R_ENV, "").strip()
    return float(v) if v else None


def default_tank(**kw) -> TankConfig:
    """The tank every entry point should construct: the plan's geometry with the water mass
    from VTOL_TANK_KG, re-shaped to VTOL_TANK_R if that is set (same water, 75% fill, the
    tank grown or shrunk to hold it -- what the gate's --slosh_radius builds). kwargs pass
    through (slosh=True, damping_zeta=..)."""
    kw.setdefault("water_kg", tank_kg_from_env())
    R = tank_radius_from_env()
    if R is not None and "radius" not in kw:
        if R <= 0.0:
            raise ValueError(f"{TANK_R_ENV}={R} must be > 0 m")
        depth = kw["water_kg"] / (1000.0 * math.pi * R * R)
        kw.update(radius=R, height=depth / 0.75)
    return TankConfig(**kw)


def describe(tank: TankConfig) -> str:
    """One-line human summary -- for a scene banner or a test log."""
    p = slosh_parameters(tank)
    return (f"tank '{tank.name}': R={1000 * tank.radius:.0f} mm H={1000 * tank.height:.0f} mm "
            f"fill={100 * tank.fill_frac:.0f}% -> {p.fluid_mass:.3f} kg "
            f"(m1 {p.slosh_mass:.3f} + m0 {p.rigid_mass:.3f}), "
            f"f={p.freq_hz:.2f} Hz  L={1000 * p.pendulum_length:.1f} mm  "
            f"zeta={100 * p.zeta:.2f}%")


# ============================================================================
#  THE PENDULUM  -- tank-plan phase 3
# ============================================================================
#
#  The first antisymmetric mode as a 2-DOF pendulum (body x and y) of mass m1 on
#  length L1, hung in the TANK (body) frame.  What drives it is the vehicle's
#  SPECIFIC FORCE f = a - g_vec (what an accelerometer reads): in the vehicle
#  frame the bob feels an apparent field -f, so with the tank axis along body z
#
#      x1_ddot = -(f_z / L1) * x1 - 2*zeta*w*x1_dot - f_xy          (per lateral axis)
#
#  where f_z / L1 is omega1^2 when f_z = g -- i.e. the restoring "gravity" is the
#  thrust-driven axial specific force, so a tank in free fall has no slosh mode and
#  one under 2 g rings sqrt(2) faster.  Both are physical.
#
#  ★ WHAT FORCE TO APPLY, AND WHY IT IS NOT k*x1 + c*x1_dot.
#  Phase 2 folded the ENTIRE liquid mass into the body link as dead mass, so PhysX
#  already carries m1's inertial reaction.  The force the moving bob exerts on the
#  vehicle is F = -m1*f_xy - m1*x1_ddot; a FROZEN bob (dead mass) exerts -m1*f_xy.
#  The slosh adds only the difference:
#
#      F_slosh = -m1 * x1_ddot
#
#  which is identically zero in every steady state -- hover, a banked turn, a
#  constant acceleration -- and so cannot double-count the mass.  The unit tests
#  check exactly that.
#
#  Integration is semi-implicit Euler at the physics step: at 3.3 Hz and 4 ms steps
#  that is 75 steps per cycle, and symplectic, so an undamped tank neither gains
#  nor loses energy over a long run (also tested).

class PendulumSlosh:
    """Runtime state of the pendulum analogue for one tank."""

    def __init__(self, params: SloshParameters, g: float = G):
        self.p = params
        self.g = g
        self.x = [0.0, 0.0]          # bob displacement relative to the hinge, body x/y [m]
        self.v = [0.0, 0.0]          # its rate [m/s]
        self.a = [0.0, 0.0]          # its last relative acceleration [m/s^2]
        self.f_last = [0.0, 0.0, 0.0]

    def omega(self, f_axial: float) -> float:
        """Instantaneous natural frequency [rad/s] for the axial specific force in play."""
        if self.p.pendulum_length <= 0.0:
            return 0.0
        return math.sqrt(max(f_axial, 0.0) / self.p.pendulum_length)

    def step(self, f_body, dt: float, omega=None, omega_dot=None, r_tank=None):
        """Advance one physics step.

        f_body: specific force of the vehicle's CG in the BODY (FLU) frame [m/s^2], tank
                axis = +z. In hover this is (0, 0, +g).
        omega, omega_dot, r_tank: body rates [rad/s], their derivative [rad/s^2] and the
                tank position relative to the CG [m]. ★ A TANK OFF THE CG feels the rotation:
                the specific force AT THE TANK is f_cg + omega_dot x r + omega x (omega x r),
                and the terms are not small -- 5 rad/s^2 of pitch at 0.3 m is 1.5 m/s^2,
                the size of the corridor's own acceleration. Omit them only for a tank at
                the CG (the plan's default), where they vanish.
        Returns the slosh force on the vehicle, body frame [N], lateral only."""
        f = [float(f_body[0]), float(f_body[1]), float(f_body[2])]
        if r_tank is not None and omega is not None:
            wx, wy, wz = (float(v) for v in omega)
            rx, ry, rz = (float(v) for v in r_tank)
            # omega x (omega x r)
            c1 = (wy * rz - wz * ry, wz * rx - wx * rz, wx * ry - wy * rx)
            f[0] += wy * c1[2] - wz * c1[1]
            f[1] += wz * c1[0] - wx * c1[2]
            f[2] += wx * c1[1] - wy * c1[0]
            if omega_dot is not None:
                ax, ay, az = (float(v) for v in omega_dot)
                f[0] += ay * rz - az * ry
                f[1] += az * rx - ax * rz
                f[2] += ax * ry - ay * rx
        fx, fy, fz = f
        self.f_last = [fx, fy, fz]
        if self.p.slosh_mass <= 0.0 or dt <= 0.0:
            self.a = [0.0, 0.0]
            return [0.0, 0.0, 0.0]
        w = self.omega(fz)
        w2 = w * w
        zeta = self.p.zeta
        wall = float(getattr(self.p, "wall", 0.0))
        for i, f_lat in enumerate((fx, fy)):
            v0 = self.v[i]
            a = -w2 * self.x[i] - 2.0 * zeta * w * self.v[i] - f_lat
            self.v[i] += a * dt                   # semi-implicit: velocity first,
            self.x[i] += self.v[i] * dt           # then position with the NEW velocity
            if wall > 0.0 and abs(self.x[i]) > wall:
                # the wall: an inelastic stop. The reaction is the acceleration that actually
                # happened, i.e. the arrest, not the free pendulum's.
                self.x[i] = math.copysign(wall, self.x[i])
                self.v[i] = 0.0
                a = (self.v[i] - v0) / dt
            self.a[i] = a
        m1 = self.p.slosh_mass
        return [-m1 * self.a[0], -m1 * self.a[1], 0.0]

    @property
    def displacement(self) -> float:
        """Lateral bob displacement magnitude [m] -- how hard the tank is sloshing."""
        return math.hypot(self.x[0], self.x[1])

    def energy(self, f_axial: float = G) -> float:
        """Mechanical energy of the free pendulum [J], for the conservation test."""
        w2 = self.omega(f_axial) ** 2
        m1 = self.p.slosh_mass
        return sum(0.5 * m1 * v * v + 0.5 * m1 * w2 * x * x for x, v in zip(self.x, self.v))

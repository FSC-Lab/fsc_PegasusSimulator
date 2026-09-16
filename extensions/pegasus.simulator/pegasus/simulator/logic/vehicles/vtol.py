"""
| File: vtol.py
| Description: Generic, *config-driven* articulated VTOL vehicle for Isaac Sim / Pegasus.
|
| ============================================================================
|  MAP OF INTERACTIONS  (read this first)
| ============================================================================
|
|  A VTOL airframe here is described purely by DATA:
|
|      VTOLConfig
|        ├── rotors:   list[RotorConfig]     # each rotor: where it is, does it tilt?
|        └── surfaces: list[SurfaceConfig]   # each control surface: where, which axis
|
|  Nothing about "tiltrotor" vs "quadplane" vs "your own design" is hard-coded.
|  To add YOUR OWN VTOL you only build a different VTOLConfig (see
|  `_default_tiltrotor_rotors()` for a worked example) — you do NOT touch the
|  class below.
|
|  The class turns that data into a physical robot and drives it every step:
|
|    1. _init_prim()        ── reads the config and BUILDS the USD articulation
|                              (body link + a nacelle/prop per rotor + a link per
|                              surface, joined by revolute joints).  This is the
|                              ONLY place geometry/physics prims are created.
|
|    2. update(dt)          ── called every physics step.  It:
|         a) _read_command()        pulls a {channel_name: value} dict from the
|                                    active backend (PX4, ROS2, scripted, ...).
|         b) _drive_actuators()     moves the JOINTS: tilt servos (rate-limited),
|                                    spinning props (visual), surface deflection.
|         c) _apply_propulsion()    [STEP 2] thrust forces on each prop link.
|         d) _apply_aerodynamics()  [STEP 2] wing/body lift-drag-moment.
|         e) ticks the backends.
|
|  >>> WHERE THINGS PLUG IN <<<
|    - New airframe geometry .......... build a VTOLConfig (data only).
|    - New control source ............. write a Backend whose input_reference()
|                                       returns the channel dict (keys must match
|                                       the *_channel names in your configs).
|    - Real flight physics ............ flip `enable_forces=True` and tune the
|                                       _apply_propulsion / _apply_aerodynamics
|                                       methods (Step 2 of the bring-up).
|
|  NOTE: This module builds a PhysX articulation procedurally.  It must be run
|  inside Isaac Sim to validate; it cannot be exercised by a plain Python import.
| ============================================================================
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Mapping, Sequence

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema

# Isaac "dynamic control" interface — used at runtime to read/command joint DOFs.
from omni.isaac.dynamic_control import _dynamic_control

from pegasus.simulator.logic.vehicles.vehicle import Vehicle
from pegasus.simulator.logic.sensors import Barometer, GPS, IMU, Magnetometer
from pegasus.simulator.logic.dynamics.vtol_aerodynamics import (
    VTOLAerodynamics,
    VTOLAerodynamicsConfig,
)


# ============================================================================
#  CONFIG DATACLASSES  — this is the "language" you use to describe an airframe.
# ============================================================================

# ---------------------------------------------------------------------------------------------
# PROPELLER CATALOGUE. The AIRFRAME is the authority on which prop it has; the controllers read
# the same table so their thrust<->throttle inversion matches the plant (a mismatch makes the
# controller silently under- or over-deliver).
#
# Each entry is (diameter [m], zero-thrust advance ratio J0). J0 is where thrust reaches zero and
# it tracks PITCH: J0 ~ 0.72*(pitch/diameter) + 0.24, which puts a 12x6 at 0.60 and a 12x10 at
# 0.84 -- in the usual 0.6-0.9 band for props of this class. CT0 is NOT tabulated: it is derived
# per rotor from (max_thrust, rotor_omega_max, diameter) so STATIC thrust is preserved exactly and
# only the airspeed behaviour changes between props.
#
# ★ WHY PITCH MATTERS HERE. At 90 deg tilt the prop axis points FORWARD, so the cruise airspeed is
# the axial inflow and a low-pitch prop runs out of thrust early. Cruise throttle at 16 m/s, and
# the margin left over for the inner loop's attitude differential:
#       12x6   0.835 -> 17% spare      12x8   0.727 -> 27%      12x10  0.653 -> 35%
# A flight on the 12x6 overspeeded to 19.4 m/s, where that prop SATURATES at 100% throttle, the
# roll/yaw differential had nothing left to work with and the aircraft departed in pitch
# (bag vtol_20260911_122940). On a 12x8 the same 19.4 m/s still leaves 14%.
PROPS = {
    "12x6":  (0.3048, 0.60),
    "12x8":  (0.3048, 0.72),
    "12x10": (0.3048, 0.84),
    "13x8":  (0.3302, 0.68),
    "13x10": (0.3302, 0.79),
}


def prop_spec(name, D_fallback=0.3048, J0_fallback=0.6):
    """(diameter, J0) for a catalogue name, falling back to the explicit values."""
    return PROPS.get(str(name), (D_fallback, J0_fallback))


@dataclass
class RotorConfig:
    """Describes a single rotor/propeller and (optionally) its tilt mechanism.

    Frame convention: body is FLU (x-forward, y-left, z-up).  A rotor at tilt
    angle 0 points its thrust along +Z (hover).  A *tiltable* rotor rotates its
    nacelle about `tilt_axis`; at `tilt_max_deg` (typically 90°) the thrust has
    swung to +X (forward / cruise).
    """

    # --- identity ---------------------------------------------------------
    name: str                                   # unique; used to name prims & joints

    # --- placement on the body -------------------------------------------
    position: tuple = (0.0, 0.0, 0.0)           # tilt-hub position in body frame [m]
    spin_dir: int = 1                           # +1 CCW / -1 CW (visual + future yaw torque)
    # Prop-shaft offset from the tilt hub [m, FLU], for tiltable rotors whose prop
    # sits away from the tilt axis. The prop (spin) link is placed here relative to
    # the nacelle, so the prop spins about its own shaft instead of orbiting the hub.
    spin_offset: tuple = (0.0, 0.0, 0.0)

    # --- thrust -----------------------------------------------------------
    max_thrust: float = 12.0                    # [N] produced at throttle = 1.0 AT ZERO AIRSPEED
    # --- propeller geometry (used by thrust_model="momentum"; ignored otherwise) ---
    # Representative of the rotor CLASS rather than a specific part: an 18 N (1.8 kgf) static
    # rotor on a ~4 kg airframe is a 12 in prop at ~7000 rpm. CT0 is DERIVED from
    # (max_thrust, rotor_omega_max, prop_diameter_m) so static thrust is preserved EXACTLY and
    # only the airspeed behaviour is new -- an airframe that never moves is unaffected.
    # Pick from PROPS by name; the two numbers below are then IGNORED unless prop is None.
    # Higher pitch holds thrust to higher airspeed, which is what buys the attitude-control
    # margin at 90 deg tilt -- see the PROPS table above.
    prop: str | None = "12x6"
    prop_diameter_m: float = 0.3048             # used only when prop is None
    prop_zero_thrust_J: float = 0.6             # used only when prop is None

    @property
    def prop_D(self) -> float:
        return prop_spec(self.prop, self.prop_diameter_m, self.prop_zero_thrust_J)[0] \
            if self.prop else self.prop_diameter_m

    @property
    def prop_J0(self) -> float:
        return prop_spec(self.prop, self.prop_diameter_m, self.prop_zero_thrust_J)[1] \
            if self.prop else self.prop_zero_thrust_J

    # --- tilt mechanism (set tiltable=False for a fixed rotor) ------------
    tiltable: bool = False
    tilt_axis: str = "Y"                        # principal axis the nacelle rotates about
    tilt_min_deg: float = 0.0                   # 0  => thrust up   (hover)
    tilt_max_deg: float = 90.0                  # 90 => thrust fwd  (cruise)
    tilt_rate_deg_s: float = 80.0               # servo slew-rate limit [deg/s]

    # --- actuator routing: which command channel(s) drive this rotor ------
    #   `throttle_channel` -> value in [0, 1].
    #   `tilt_channel`     -> value in [0, 1]  (0 = tilt_min, 1 = tilt_max).
    throttle_channel: str = ""
    tilt_channel: str | None = None             # ignored when tiltable is False

    # --- imported CAD geometry (optional; None -> procedural box) ---------
    #   mesh         : the SPINNING part (propeller) -> prop link. For a tiltable
    #                  rotor the prop link is a child of the nacelle, so it also
    #                  tilts.
    #   nacelle_mesh : the TILT-ONLY part (arm/bracket/motor) -> nacelle link
    #                  (tiltable rotors only). Tilts with the nacelle but does NOT
    #                  spin. Leave None to keep the nacelle invisible.
    #   scale/offset/rpy are shared by both (they come from one CAD assembly, each
    #   recentered on the same rotor hub) — convert units (mm->m via 0.001) and
    #   orient the CAD frame to FLU.
    mesh: str | None = None
    nacelle_mesh: str | None = None
    mesh_scale: tuple = (1.0, 1.0, 1.0)
    mesh_offset: tuple = (0.0, 0.0, 0.0)
    mesh_rpy_deg: tuple = (0.0, 0.0, 0.0)

    # ---- derived joint / prim names (kept here so build & drive stay in sync) ----
    @property
    def prop_link(self) -> str:
        return f"prop_{self.name}"

    @property
    def nacelle_link(self) -> str:
        return f"nacelle_{self.name}"

    @property
    def spin_joint(self) -> str:                # revolute joint that spins the prop (visual)
        return f"spin_{self.name}"

    @property
    def tilt_joint(self) -> str:                # revolute joint that tilts the nacelle
        return f"tilt_{self.name}"


@dataclass
class SurfaceConfig:
    """Describes one aerodynamic control surface (aileron / elevon / rudder...).

    The surface is a hinged link rotating about `hinge_axis`.  In Step 1 the
    hinge motion is purely visual; in Step 2 `deflect_channel` also feeds the
    aerodynamics model.
    """

    name: str                                   # unique; names prim & joint
    position: tuple = (0.0, 0.0, 0.0)           # hinge location in body frame [m]
    hinge_axis: str = "Y"                       # principal axis the surface rotates about
    max_deflection_deg: float = 25.0            # |deflection| at command = ±1
    deflect_channel: str = ""                   # command value in [-1, 1]
    deflect_scale: float = 1.0                  # sign/gain (e.g. -1 for the opposite aileron)
    size: tuple | None = None                   # box L,W,H [m]; None -> cfg.surface_size

    # --- imported CAD geometry (optional; None -> procedural box) ---------
    mesh: str | None = None                     # USD for the control-surface part
    mesh_scale: tuple = (1.0, 1.0, 1.0)
    mesh_offset: tuple = (0.0, 0.0, 0.0)
    mesh_rpy_deg: tuple = (0.0, 0.0, 0.0)

    @property
    def surface_link(self) -> str:
        return f"surface_{self.name}"

    @property
    def hinge_joint(self) -> str:
        return f"hinge_{self.name}"


# ---- Default airframe: PX4-style tri-tiltrotor (2 tilting front + 1 fixed tail) ----
#
#   front-L (tilt)      front-R (tilt)
#         \                /
#          +--- body  ---+        elevon_L / elevon_R
#                |
#             tail (fixed, vertical)
#
# Replace these two factories (or pass your own lists into VTOLConfig) to define
# a completely different VTOL — the rest of the class does not change.

def _default_tiltrotor_rotors() -> list[RotorConfig]:
    return [
        RotorConfig(
            name="front_L", position=(0.30, 0.55, 0.05), spin_dir=+1, max_thrust=14.0,
            tiltable=True, tilt_axis="Y", tilt_min_deg=0.0, tilt_max_deg=90.0,
            throttle_channel="throttle_front_L", tilt_channel="tilt",
        ),
        RotorConfig(
            name="front_R", position=(0.30, -0.55, 0.05), spin_dir=-1, max_thrust=14.0,
            tiltable=True, tilt_axis="Y", tilt_min_deg=0.0, tilt_max_deg=90.0,
            throttle_channel="throttle_front_R", tilt_channel="tilt",
        ),
        RotorConfig(
            name="tail", position=(-0.70, 0.0, 0.05), spin_dir=+1, max_thrust=14.0,
            tiltable=False,
            throttle_channel="throttle_tail",
        ),
    ]


def _default_tiltrotor_surfaces() -> list[SurfaceConfig]:
    return [
        SurfaceConfig(name="elevon_L", position=(-0.10, 0.70, 0.0), hinge_axis="Y",
                      max_deflection_deg=25.0, deflect_channel="elevon_L"),
        SurfaceConfig(name="elevon_R", position=(-0.10, -0.70, 0.0), hinge_axis="Y",
                      max_deflection_deg=25.0, deflect_channel="elevon_R"),
    ]



def _gps_config_from_env():
    """The sim GPS's rate and noise from VTOL_GPS_HZ / VTOL_GPS_SIGMA / VTOL_GPS_VSIGMA (2026-09-15;
    unset = Pegasus defaults). Pegasus draws noise as density * sqrt(dt) * randn per sample."""
    hz = float(os.environ.get("VTOL_GPS_HZ") or 250.0)
    cfg = {"update_rate": hz}
    sig = os.environ.get("VTOL_GPS_SIGMA")
    if sig:
        cfg["gps_xy_noise_density"] = float(sig) * math.sqrt(hz)
        cfg["gps_z_noise_density"] = 2.0 * float(sig) * math.sqrt(hz)
    vsig = os.environ.get("VTOL_GPS_VSIGMA")
    if vsig:
        cfg["gps_vxy_noise_density"] = float(vsig) * math.sqrt(hz)
        cfg["gps_vz_noise_density"] = 2.0 * float(vsig) * math.sqrt(hz)
    # VTOL_GPS_WALK: the slow position wander's random-walk magnitude [(m/s)/sqrt(Hz)] (Pegasus's 2.0
    # is Gazebo's; at 10 Hz it is 6 cm per sample and EKF2's GPS drift check rejects the receiver --
    # 2026-09-15). A real receiver: per-sample jitter of centimetres (VTOL_GPS_SIGMA 0.05) and a
    # wander of tens of centimetres over a minute (VTOL_GPS_WALK 0.05, tau 60 s).
    walk = os.environ.get("VTOL_GPS_WALK")
    if walk:
        cfg["gps_xy_random_walk"] = float(walk)
        cfg["gps_z_random_walk"] = 2.0 * float(walk)
    return cfg

def _wind_from_env():
    """VTOL_WIND="wx,wy,wz": a steady WORLD (ENU) wind [m/s] -- the air's velocity -- the plant's
    aerodynamics and propeller inflow feel through v_air = v - w (hardware-transfer piece 4,
    2026-09-15). Unset = (0,0,0): still air, every earlier flight repeats."""
    raw = os.environ.get("VTOL_WIND", "").strip()
    if not raw:
        return (0.0, 0.0, 0.0)
    w = tuple(float(v) for v in raw.split(","))
    if len(w) != 3:
        raise ValueError(f"VTOL_WIND must be 'wx,wy,wz' [m/s], got {raw!r}")
    return w


@dataclass
class VTOLConfig:
    """Everything needed to spawn and run a VTOL.  Pure data (CLAUDE.md rule:
    no live resources / backends instantiated here)."""

    # --- identity / spawning ---------------------------------------------
    stage_prefix: str = "vtol"
    usd_file: str | None = None                 # if set, reference an external USD instead of building one

    # --- bulk inertial properties of the BODY link -----------------------
    mass: float = 3.5                           # [kg]
    inertia: tuple = (0.20, 0.28, 0.35)         # diagonal body inertia [kg m^2]
    body_com_offset: tuple = (0.0, 0.0, 0.0)    # body center-of-mass offset [m]; set to
                                                # the rotor centroid so hover thrust is
                                                # balanced (headroom for yaw)
    # Point (body frame, relative to the BODY LINK ORIGIN) where the lumped
    # aerodynamic force is applied.  None -> the centre of mass, i.e. the aero
    # force produces NO moment of its own and all pitch/roll/yaw aerodynamics
    # come from the moment coefficients, where they are visible and tunable.
    #
    # This used to be hard-coded to (0,0,0) -- the body link ORIGIN -- while
    # PhysX takes moments about the CoM.  On the CAD airframe (CG at x=0.176)
    # that put the whole aerodynamic force on a 0.176 m lever arm: an implicit
    # aerodynamic centre at 71% MAC, contributing ~3x more pitch moment than
    # the entire Cm model, invisible in the config, and silently re-tuned every
    # time body_com_offset was adjusted (2026-09-06 audit).
    #
    # To model a real static margin, set this a few cm BEHIND the CG, e.g.
    # com + (-0.03, 0, 0) for ~12% MAC on a 0.25 m chord -- and then reduce
    # cm_alpha accordingly so the margin is not counted twice.
    aero_center: tuple | None = None
    disable_gravity: bool = False               # handy for visual-only bring-up (Step 1)

    # --- imported CAD geometry for the BODY (optional; None -> box) -------
    body_mesh: str | None = None                # USD for the fuselage/body part
    body_mesh_scale: tuple = (1.0, 1.0, 1.0)
    body_mesh_offset: tuple = (0.0, 0.0, 0.0)
    body_mesh_rpy_deg: tuple = (0.0, 0.0, 0.0)
    # Optional invisible box collider (L,W,H [m]) for a MESH body, so it can land
    # / not fall through the floor (a triangle mesh can't be a dynamic collider).
    body_collision_size: tuple | None = None

    # --- procedural geometry (only used when a link has no mesh) ----------
    body_size: tuple = (1.20, 0.30, 0.12)       # body box L,W,H [m]
    nacelle_size: tuple = (0.10, 0.08, 0.08)    # tilt-nacelle box [m]
    prop_size: tuple = (0.28, 0.02, 0.01)       # prop "blade" box L,W,H [m] (long in x so spin is visible)
    surface_size: tuple = (0.10, 0.40, 0.01)    # control-surface box [m]
    link_mass: float = 0.05                     # [kg] mass of each small link (nacelle/prop/surface)
    link_inertia: tuple = (1e-4, 1e-4, 1e-4)    # diagonal inertia of small links

    # --- joint drive gains (PD servos; tune in Isaac) --------------------
    tilt_drive_stiffness: float = 2.0e3         # position drive for tilt nacelles
    tilt_drive_damping: float = 1.0e2
    surface_drive_stiffness: float = 1.0e3      # position drive for control surfaces
    surface_drive_damping: float = 5.0e1
    spin_drive_damping: float = 1.0e-2          # velocity drive for prop spin (visual)

    # --- the airframe itself ---------------------------------------------
    rotors: list = field(default_factory=_default_tiltrotor_rotors)
    surfaces: list = field(default_factory=_default_tiltrotor_surfaces)
    aerodynamics: VTOLAerodynamicsConfig = field(default_factory=VTOLAerodynamicsConfig)

    # --- physics master switch -------------------------------------------
    #   Step 1 (this commit): False -> joints animate, NO forces applied.
    #   Step 2: set True (and disable_gravity=False) and tune the force methods.
    enable_forces: bool = False

    # --- rotor + servo dynamics (opt-in) ---------------------------------
    #   False -> legacy instantaneous model: thrust = throttle * max_thrust,
    #            visual-only prop spin, rate-limited tilt.
    #   True  -> each rotor has a spinning-inertia state omega [rad/s]:
    #              omega follows omega_cmd = throttle * rotor_omega_max with a
    #              first-order lag (rotor_tau_spin), thrust = max_thrust *
    #              (omega/omega_max)^2 (so thrust LAGS throttle and scales with
    #              RPM^2), and a yaw reaction torque = rotor_moment_ratio * thrust
    #              is applied about the prop spin axis. Tilt servos become a
    #              first-order lag (tilt_tau) with the same slew-rate cap.
    rotor_dynamics: bool = False
    # ★ THRUST MODEL (selects how throttle becomes force). `rotor_dynamics` above is kept as a
    # back-compatible alias: True behaves as "lag".
    #   "instant"  T = throttle * max_thrust. LINEAR and immediate. The legacy model, and what
    #              every controller here is currently tuned against.
    #   "lag"      omega chases throttle*rotor_omega_max with rotor_tau_spin; T = max_thrust *
    #              (omega/omega_max)^2, so thrust LAGS throttle and is QUADRATIC in it.
    #   "momentum" "lag" PLUS an inflow term: the propeller loses thrust as the air already
    #              moving through the disk approaches the speed it can accelerate it to.
    #              T = rho*D^2*( CT0*(nD)^2 - (CT0/J0)*(nD)*V_axial ), clipped at zero, with
    #              V_axial the component of airspeed along THIS rotor's (tilted) thrust axis.
    #              At J = V/(nD) = J0 the rotor makes no thrust at all.
    # ★ WHY THIS MATTERS HERE: at 90 deg tilt the prop axis points FORWARD, so V_axial IS the
    # cruise airspeed. With "instant" a rotor makes its full 18 N at 16 m/s exactly as it does
    # standing still. With "momentum" and a representative 12 in prop, 16 m/s leaves only
    # 4.8 N/rotor at FULL throttle, and no positive thrust at all below ~74% throttle -- against
    # a controller that currently cruises at 11%. Switching this on WILL invalidate the cruise
    # trim and require re-tuning; that is the point of it.
    thrust_model: str = "instant"
    # ★ CONTROL-SURFACE ACTUATOR. Defaults 0 = the legacy INSTANTANEOUS surface (no rate limit,
    # no lag), which is what the controllers were tuned against. Representative small digital
    # servo: rate 300-500 deg/s, tau 0.02-0.05 s. A small stepper is slower: 100-200 deg/s.
    # Until these are set, a controller can slew a surface at any rate it likes -- and one did:
    # the HIL elevator sign-flipped near full deflection every ~0.25 s, which needs several
    # hundred deg/s to be physical.
    surface_rate_deg_s: float = 0.0             # 0 = unlimited (legacy)
    surface_tau: float = 0.0                    # 0 = no lag (legacy)
    rotor_omega_max: float = 1000.0             # [rad/s] prop speed at throttle = 1
    rotor_tau_spin: float = 0.12                # [s] spin-up/-down time constant
    rotor_moment_ratio: float = 0.12            # yaw reaction torque per N of thrust [m] (raised for MC yaw authority; sim-only, real props ~0.02)
    tilt_tau: float = 0.06                      # [s] tilt-servo time constant

    # --- plumbing (kept empty per CLAUDE.md; backends created by caller) --
    # VTOL_GPS_HZ (2026-09-15): the sim GPS's update rate -- Pegasus's default is 250 Hz, which no
    # receiver does; 5-10 Hz is a GPS. VTOL_GPS_SIGMA / VTOL_GPS_VSIGMA: a per-SAMPLE 1-sigma position
    # [m] and velocity [m/s] white noise (Pegasus scales its densities by sqrt(dt), so the density is
    # sigma * sqrt(rate)); its slow random-walk bias stays at the Pegasus default. Empty/unset keeps
    # every default so every earlier flight repeats.
    sensors: list = field(default_factory=lambda: [Barometer(), IMU(), Magnetometer(), GPS(_gps_config_from_env())])
    # --- WIND (piece 4, 2026-09-15): a steady world wind plus a first-order coloured gust ---------
    # VTOL_WIND="wx,wy,wz" [m/s, ENU]; VTOL_TURB=sigma [m/s, 1-sigma per axis] of a Gauss-Markov
    # gust with correlation time VTOL_TURB_TAU [s] (default 1 s), drawn per physics step. The aero
    # model gets wind_body = R^T w, the momentum prop's axial inflow uses v_air, and the mavlink
    # backend's synthesised pitot reads the AIR-relative forward speed (state.wind_world).
    wind_world: tuple = field(default_factory=_wind_from_env)
    turb_sigma: float = field(default_factory=lambda: float(os.environ.get("VTOL_TURB") or 0.0))
    turb_tau: float = field(default_factory=lambda: float(os.environ.get("VTOL_TURB_TAU") or 1.0))
    graphical_sensors: list = field(default_factory=list)
    graphs: list = field(default_factory=list)
    backends: list = field(default_factory=list)


# ----------------------------------------------------------------------------
#  AIRFRAME PRESET: dual INDEPENDENT tiltrotor + aileron / elevator / rudder
#
#  This is the production target airframe (a USD from SolidWorks will replace the
#  procedural body later by setting config.usd_file).  It is a worked example of
#  "define a new VTOL with data only".
# ----------------------------------------------------------------------------

def dual_tiltrotor_config() -> VTOLConfig:
    """Two INDEPENDENT tilt rotors + conventional aileron / elevator / rudder.

    Layout (body FLU, x-fwd / y-left / z-up):

            rotor_left (tilt_left)          rotor_right (tilt_right)
                    \\                            /
                     +---------- body ----------+
                     |                          |
              aileron (wing)            elevator + rudder (tail)

    Independent tilt gives yaw-by-differential-tilt in hover; the conventional
    surfaces take over in cruise.

    Channel names — the PX4 channel_map (Pegasus side) and the ROS2
    direct_actuator node (PX4 side) must BOTH produce exactly these:

        motors  : motor_left, motor_right          range [0, 1]
        tilts   : tilt_left,  tilt_right            range [0, 1]  (0=hover, 1=cruise)
        surfaces: aileron, elevator, rudder         range [-1, 1]
    """
    rotors = [
        RotorConfig(
            name="left", position=(0.10, 0.50, 0.05), spin_dir=+1, max_thrust=18.0,
            tiltable=True, tilt_axis="Y", tilt_min_deg=0.0, tilt_max_deg=90.0,
            throttle_channel="motor_left", tilt_channel="tilt_left",
        ),
        RotorConfig(
            name="right", position=(0.10, -0.50, 0.05), spin_dir=-1, max_thrust=18.0,
            tiltable=True, tilt_axis="Y", tilt_min_deg=0.0, tilt_max_deg=90.0,
            throttle_channel="motor_right", tilt_channel="tilt_right",
        ),
    ]
    surfaces = [
        SurfaceConfig(name="aileron", position=(-0.05, 0.45, 0.0), hinge_axis="Y",
                      max_deflection_deg=25.0, deflect_channel="aileron"),
        SurfaceConfig(name="elevator", position=(-0.75, 0.0, 0.05), hinge_axis="Y",
                      max_deflection_deg=25.0, deflect_channel="elevator"),
        SurfaceConfig(name="rudder", position=(-0.75, 0.0, 0.15), hinge_axis="Z",
                      max_deflection_deg=25.0, deflect_channel="rudder"),
    ]
    return VTOLConfig(
        stage_prefix="dual_tiltrotor",
        mass=4.0,
        inertia=(0.25, 0.30, 0.45),
        rotors=rotors,
        surfaces=surfaces,
    )


# ----------------------------------------------------------------------------
#  AIRFRAME PRESET: 2 FRONT tilting rotors + 2 REAR static (lift) rotors,
#  driven by imported CAD meshes. This matches a "two tilting rotor components +
#  two static rotor components" airframe.
# ----------------------------------------------------------------------------

def quad_tiltrotor_config(
    body_mesh: str | None = None,
    front_left_mesh: str | None = None,
    front_right_mesh: str | None = None,
    rear_left_mesh: str | None = None,
    rear_right_mesh: str | None = None,
    front_left_arm_mesh: str | None = None,
    front_right_arm_mesh: str | None = None,
    mesh_scale: tuple = (1.0, 1.0, 1.0),
    mesh_rpy_deg: tuple = (0.0, 0.0, 0.0),
) -> VTOLConfig:
    """VTOL with 2 FRONT tilting rotors + 2 REAR static (vertical-lift) rotors.

    Pass the converted-USD path for each CAD component (see
    application/vtol/convert_meshes_to_usd.py). Any left None falls back to the
    procedural box for that link, so you can bring the airframe up incrementally.
    `mesh_scale` converts CAD units (0.001 for mm); `mesh_rpy_deg` orients the CAD
    frame to FLU and is applied to the body AND every rotor mesh (they share one
    CAD frame) — e.g. (90, 0, 90) for a model authored with the nose along +Z.

    Default rotor `position`s are the hub locations measured off the FSC VTOL STL
    (split by application/vtol/… → parts). Channels:
        motors : motor_front_L/R, motor_rear_L/R   [0, 1]
        tilts  : tilt_left, tilt_right             [0, 1]  (0=hover, 1=cruise)
    """
    r = dict(mesh_scale=mesh_scale, mesh_rpy_deg=mesh_rpy_deg)
    rotors = [
        RotorConfig(
            name="front_L", position=(0.459, 0.285, -0.068), spin_offset=(-0.002, 0.0, 0.090),
            # ★ 12x8, not the RotorConfig default 12x6 (2026-09-13). At the corrected 749 rad/s
            # a 12x6 makes 4.8 N per front rotor at 16 m/s against a 2.3 N cruise need, and at
            # 19 m/s (a bank) its maximum EQUALS the need -- nothing left for the inner loop's
            # differential. The 12x8 (J0 0.72) leaves 5 N there and takes the cruise throttle
            # 0.88 -> 0.78; every gate number (trim, corridor, back-transition, upsets, CG
            # hover, latency) is unchanged. The controllers, plant double and GS override carry
            # J0 = 0.72 with it; tests/test_prop_pairing.py holds them together.
            spin_dir=+1, max_thrust=18.0, prop="12x8",
            # tilt_min -20: the front pair may lean BACKWARDS.  With beta >= 0 the only way to
            # decelerate is to pitch nose-up, which converts speed into HEIGHT (bag full_tr31
            # ballooned 18.9 m, of which 12.8 m is unavoidable kinetic->potential conversion).
            # Tilting the rotors aft puts the braking force in the airstream direction instead,
            # so the body can stay level: T*sin(20 deg) = 12.3 N = 3.6 m/s^2 with no zoom.
            tiltable=True, tilt_axis="Y", tilt_min_deg=-20.0, tilt_max_deg=90.0,
            throttle_channel="motor_front_L", tilt_channel="tilt_left",
            mesh=front_left_mesh, nacelle_mesh=front_left_arm_mesh, **r,
        ),
        RotorConfig(
            name="front_R", position=(0.459, -0.284, -0.066), spin_offset=(-0.002, -0.001, 0.088),
            spin_dir=-1, max_thrust=18.0, prop="12x8",
            tiltable=True, tilt_axis="Y", tilt_min_deg=-20.0, tilt_max_deg=90.0,
            throttle_channel="motor_front_R", tilt_channel="tilt_right",
            mesh=front_right_mesh, nacelle_mesh=front_right_arm_mesh, **r,
        ),
        RotorConfig(
            name="rear_L", position=(-0.112, 0.284, -0.019), spin_dir=-1, max_thrust=18.0, prop="12x8",
            tiltable=False, throttle_channel="motor_rear_L",
            mesh=rear_left_mesh, **r,
        ),
        RotorConfig(
            name="rear_R", position=(-0.112, -0.284, -0.017), spin_dir=+1, max_thrust=18.0, prop="12x8",
            tiltable=False, throttle_channel="motor_rear_R",
            mesh=rear_right_mesh, **r,
        ),
    ]
    # Control surfaces (procedural boxes, sized for the ~1.9 m-span airframe):
    #   2 ailerons on the wing trailing edge (opposite deflection -> roll),
    #   1 elevator (horizontal, aft) and 1 rudder (vertical, aft). Positions in
    #   body FLU: wing ~x+0.1, tail ~x-0.4.
    surfaces = [
        SurfaceConfig(name="aileron_L", position=(-0.02, 0.60, 0.0), hinge_axis="Y",
                      max_deflection_deg=25.0, deflect_channel="aileron", deflect_scale=1.0,
                      size=(0.12, 0.35, 0.01)),
        SurfaceConfig(name="aileron_R", position=(-0.02, -0.60, 0.0), hinge_axis="Y",
                      max_deflection_deg=25.0, deflect_channel="aileron", deflect_scale=-1.0,
                      size=(0.12, 0.35, 0.01)),
        SurfaceConfig(name="elevator", position=(-0.45, 0.0, 0.05), hinge_axis="Y",
                      max_deflection_deg=25.0, deflect_channel="elevator",
                      size=(0.14, 0.45, 0.01)),
        SurfaceConfig(name="rudder", position=(-0.45, 0.0, 0.18), hinge_axis="Z",
                      max_deflection_deg=25.0, deflect_channel="rudder",
                      size=(0.14, 0.01, 0.22)),
    ]
    return VTOLConfig(
        # ★ ACTUATOR FIDELITY ON FOR THIS AIRFRAME (2026-09-10). The class-wide defaults stay
        # legacy so other airframes are untouched; the FSC quadplane opts in here.
        #   momentum  -- the propeller loses thrust to its own inflow. At 90 deg tilt the prop
        #                axis points FORWARD, so cruise airspeed IS the axial inflow: a 12 in
        #                prop drops 18.0 -> 4.8 N/rotor between 0 and 16 m/s, and makes NO
        #                positive thrust below ~74% throttle at 16. The old flat model gave the
        #                full 18 N at any speed, which is what made an 11%-throttle cruise trim
        #                possible. Expect cruise throttle ~0.8, not ~0.1.
        #   surface   -- a representative small digital servo. tau alone is only half a servo
        #                (it would still permit unbounded slew), so the rate limit goes with it.
        thrust_model="momentum",
        # ★ 749 rad/s (7150 rpm), NOT the VTOLConfig default of 1000. CT0 is derived so static
        # thrust is 18 N either way, but the ADVANCE RATIO at a given throttle is not: at 1000
        # the prop at 0.80 throttle and 16.5 m/s makes 3.5 N where the controllers, the plant
        # double, the offline gate and tests/test_actuator_models.py -- all written for 749 --
        # expect 0.7. Flown (2026-09-13 10:36 bag, row ke_cap8b): the aircraft cruised 0.45 m/s above its
        # command on less throttle than the gate needs, and the back-transition onset braked
        # from behind its reference (+21 deg pitch, gate 11-13). The default had never been set
        # for this airframe; tests/test_prop_pairing.py now holds sim and controller together.
        rotor_omega_max=749.0,
        surface_tau=0.03,                # [s] small digital servo
        surface_rate_deg_s=400.0,        # [deg/s] its no-load slew rate
        stage_prefix="quad_tiltrotor",
        mass=2.95,                       # measured FSC VTOL mass (matches autopilot vehicle_mass)
        inertia=(0.20, 0.23, 0.36),      # scaled ~mass ratio from the old 4.5 kg placeholder
        body_com_offset=(0.195, 0.0, -0.043),  # BODY-link CG chosen so the TOTAL vehicle CG
                                                # (body + all articulated links) lands on the rotor
                                                # centroid (x=0.1735), where thrust acts. The links
                                                # pull it AFT (front rotors carry 2 links -- tiltable
                                                # nacelle + prop -- vs 1 for the rear, plus 4 aft
                                                # surfaces), so the body offset must sit FORWARD at
                                                # ~0.195. Setting it to 0.174 (rotor centroid, ignoring
                                                # links) put the true CG ~2 cm AFT of the thrust ->
                                                # symmetric thrust made a ~0.6 Nm nose-up moment = the
                                                # -38deg liftoff pitch disturbance. (0.195 = the value
                                                # that was wrongly reverted to 0.174; see git log.)
        rotors=rotors,
        surfaces=surfaces,
        body_mesh=body_mesh,
        body_mesh_scale=mesh_scale,
        body_mesh_rpy_deg=mesh_rpy_deg,
        body_collision_size=(0.8, 0.2, 0.2),     # fuselage box collider (land/no fall-through)
    )


# ============================================================================
#  VEHICLE
# ============================================================================

# ============================================================================
#  The ASSEMBLY: body link + ten small links = what flies and what the controllers model
# ============================================================================
#  `VTOLConfig.mass`, `.inertia` and `.body_com_offset` describe the BODY LINK only; PhysX
#  assembles the vehicle from that plus the nacelle / prop / surface links. The controllers
#  carried the body link's inertia for a month (Ixx -24%, Izz -29%) before this was written
#  (in the model copy's module, 2026-09-12); it lives here since 2026-09-14 because the base class needs
#  the assembled CG too: the lumped aero force defaulted to `body_com_offset`, the body link's
#  CoM, which sits 18.6 mm AHEAD of the assembled CG on the FSC quad -- 38 N of cruise lift on
#  that arm is a 0.45 N.m nose-up pitch moment the controllers' Cm has no term for, measured
#  on every bag by scripts/ana_force_model.py (the moment balance closes on roll and yaw and
#  leaves exactly a lift-proportional pitch residual). The 2026-09-06 aero_center fix meant
#  "the centre of mass -> zero accidental moment arm"; for an articulated vehicle that is the
#  assembly's, and it is now the default.


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


class VTOL(Vehicle):
    """A generic articulated VTOL.

    The class is intentionally airframe-agnostic: it loops over
    `config.rotors` / `config.surfaces` for both building (`_init_prim`) and
    driving (`update`).  See the MAP OF INTERACTIONS at the top of this file.
    """

    # ---- principal-axis string -> Gf vector (for joint axis attributes) ----
    _AXES = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}

    def __init__(
        self,
        vehicle_id: int = 0,
        init_pos=(0.0, 0.0, 0.5),
        init_orientation=(0.0, 0.0, 0.0, 1.0),
        config: VTOLConfig | None = None,
    ):
        if config is None:
            config = VTOLConfig()

        # Stash config locally BEFORE super().__init__ — because the base class
        # calls self._init_prim() from inside its constructor and that needs
        # these values to build the articulation.
        self._cfg = config
        self._rotors: list[RotorConfig] = list(config.rotors)
        self._surfaces: list[SurfaceConfig] = list(config.surfaces)
        self._build_procedurally = config.usd_file is None

        # Aerodynamics model (Step 2 consumer).
        self._aero = VTOLAerodynamics(config.aerodynamics)

        # Where the lumped aero force acts, in body-link-origin coordinates.
        # Defaults to the centre of mass -> zero accidental moment arm.
        self._aero_center = [
            float(v) for v in (config.aero_center if config.aero_center is not None
                               else assembled_mass_properties(config)[1])
        ]

        # Per-tilt-rotor servo state (current rate-limited angle in radians),
        # keyed by rotor name.  Initialised to the "hover" limit.
        self._tilt_state: dict[str, float] = {
            # start at 0 deg (hover), NOT tilt_min -- with a negative tilt_min the nacelles
            # would otherwise spawn leaning backwards.
            r.name: 0.0 for r in self._rotors if r.tiltable
        }

        # Per-rotor spinning speed [rad/s] (used by thrust_model 'lag' and 'momentum').
        self._omega: dict[str, float] = {r.name: 0.0 for r in self._rotors}
        # ACTUAL control-surface deflection [rad]. Distinct from the COMMAND: with a surface
        # actuator configured, the aero must see what the surface is DOING, not what it was
        # asked to do. Previously the aero read the raw command, so the surface moment appeared
        # the instant the command did, with no rate limit and no lag.
        self._surface_state: dict[str, float] = {sf.name: 0.0 for sf in self._surfaces}

        # One-shot guard for the channel-mismatch diagnostic (see _validate_channels).
        self._channels_checked = False

        super().__init__(
            stage_prefix=f"{config.stage_prefix}_{vehicle_id}",
            usd_path=config.usd_file,
            init_pos=init_pos,
            init_orientation=init_orientation,
            sensors=config.sensors,
            graphical_sensors=config.graphical_sensors,
            graphs=config.graphs,
            backends=config.backends,
        )

    # ========================================================================
    #  (1) BUILD  — turn the config into a USD articulation.
    #      Runs once, from Vehicle.__init__, BEFORE the physics handles exist.
    # ========================================================================

    def _init_prim(self, root_prim):
        """Build the articulation: one base link `/body`, plus a nacelle+prop
        per rotor and a link per surface, all joined to `/body` by revolute
        joints.  Only runs when no external USD was supplied."""
        if not self._build_procedurally:
            return

        cfg = self._cfg
        stage = self._current_stage
        prefix = self._stage_prefix

        # --- Articulation ROOT lives on the top Xform (NOT a rigid body).
        #     The base link is the child `/body` rigid body.  (Multi-link
        #     articulations put the root API on an ancestor Xform; this differs
        #     from the single-link case in CLAUDE.md.)
        UsdPhysics.ArticulationRootAPI.Apply(root_prim)

        # --- Base link: the airframe body -----------------------------------
        body_path = f"{prefix}/body"
        self._add_rigid_link(
            path=body_path, size=cfg.body_size, mass=cfg.mass,
            inertia=cfg.inertia, color=(0.20, 0.35, 0.65), collision=True,
            mesh=cfg.body_mesh, mesh_scale=cfg.body_mesh_scale,
            mesh_offset=cfg.body_mesh_offset, mesh_rpy_deg=cfg.body_mesh_rpy_deg,
            com_offset=cfg.body_com_offset,
        )
        # A mesh body has no PhysX collider; add an invisible box collider so it
        # can land / not fall through the floor.
        if cfg.body_mesh is not None and cfg.body_collision_size is not None:
            col = UsdGeom.Cube.Define(stage, Sdf.Path(f"{body_path}/collision"))
            col.GetSizeAttr().Set(1.0)
            UsdGeom.Xformable(col).AddScaleOp().Set(Gf.Vec3f(*cfg.body_collision_size))
            UsdGeom.Imageable(col).MakeInvisible()
            UsdPhysics.CollisionAPI.Apply(col.GetPrim())

        # A tidy scope to hold all the joint prims (purely organisational).
        UsdGeom.Scope.Define(stage, Sdf.Path(f"{prefix}/joints"))

        # --- One nacelle/prop chain per rotor -------------------------------
        for rotor in self._rotors:
            self._build_rotor(rotor)

        # --- One hinged link per control surface ----------------------------
        for surface in self._surfaces:
            self._build_surface(surface)

    # ---- builder helpers ---------------------------------------------------

    def _build_rotor(self, rotor: RotorConfig):
        """Build the prim/joint chain for a single rotor.

        Tiltable:   body --[tilt joint]--> nacelle --[spin joint]--> prop
        Fixed:      body --[spin joint]--------------------------->  prop
        """
        cfg = self._cfg
        prefix = self._stage_prefix

        prop_path = f"{prefix}/{rotor.prop_link}"

        # Shared mesh transform (both prop + nacelle come from one CAD assembly).
        mesh_xform = dict(
            mesh_scale=rotor.mesh_scale, mesh_offset=rotor.mesh_offset,
            mesh_rpy_deg=rotor.mesh_rpy_deg,
        )
        # The SPINNING part (prop) goes on the prop link.
        prop_mesh_kwargs = dict(mesh=rotor.mesh, **mesh_xform)
        prop_size = None if rotor.mesh is not None else cfg.prop_size
        # A rotor is "mesh-driven" if it supplies either a prop or a nacelle mesh.
        has_mesh = rotor.mesh is not None or rotor.nacelle_mesh is not None

        if rotor.tiltable:
            # 1) Nacelle link carries the tilt DOF AND the TILT-ONLY arm/bracket
            #    mesh (nacelle_mesh) — it tilts but does not spin. Box when purely
            #    procedural, invisible when mesh-driven but no arm mesh was given.
            nacelle_path = f"{prefix}/{rotor.nacelle_link}"
            nacelle_size = None if has_mesh else cfg.nacelle_size
            self._add_rigid_link(
                path=nacelle_path, size=nacelle_size, mass=cfg.link_mass,
                inertia=cfg.link_inertia, color=(0.85, 0.55, 0.15),
                mesh=rotor.nacelle_mesh, **mesh_xform,
            )
            # 2) Tilt joint: body -> nacelle, anchored at the hub position.
            #    Position-driven servo, limited to [tilt_min, tilt_max].
            self._add_revolute_joint(
                name=rotor.tilt_joint,
                parent=f"{prefix}/body", child=nacelle_path,
                anchor_in_parent=rotor.position, axis=rotor.tilt_axis,
                lower_deg=rotor.tilt_min_deg, upper_deg=rotor.tilt_max_deg,
                drive="position",
                stiffness=cfg.tilt_drive_stiffness, damping=cfg.tilt_drive_damping,
            )
            # 3) Prop link, offset from the nacelle to the prop shaft so it spins
            #    in place (not orbiting the tilt hub). Child of nacelle -> tilts too.
            self._add_rigid_link(
                path=prop_path, size=prop_size, mass=cfg.link_mass,
                inertia=cfg.link_inertia, color=(0.90, 0.90, 0.90), **prop_mesh_kwargs,
            )
            self._unclamp_prop_spin(prop_path)
            # 4) Spin joint: nacelle -> prop, at the prop shaft, velocity-driven.
            self._add_revolute_joint(
                name=rotor.spin_joint,
                parent=nacelle_path, child=prop_path,
                anchor_in_parent=rotor.spin_offset, axis="Z",
                drive="velocity", damping=cfg.spin_drive_damping,
            )
        else:
            # Fixed rotor: prop hangs straight off the body.
            self._add_rigid_link(
                path=prop_path, size=prop_size, mass=cfg.link_mass,
                inertia=cfg.link_inertia, color=(0.90, 0.90, 0.90), **prop_mesh_kwargs,
            )
            self._unclamp_prop_spin(prop_path)
            self._add_revolute_joint(
                name=rotor.spin_joint,
                parent=f"{prefix}/body", child=prop_path,
                anchor_in_parent=rotor.position, axis="Z",
                drive="velocity", damping=cfg.spin_drive_damping,
            )

    def _unclamp_prop_spin(self, prop_path: str):
        """Let the prop link spin as fast as the rotor model says it does.

        ★ PhysX caps every rigid body's angular velocity at 100 rad/s by default
        (physxRigidBody:maxAngularVelocity = 5729.58 deg/s). The props are driven to
        400-750 rad/s, so the velocity drive pushed against that cap FOREVER with
        torque D*(omega_cmd - 100) per prop, reacted onto the airframe about the spin
        axis. Equal throttles cancel; any DIFFERENTIAL does not: measured in
        application/vtol/09_vtol_inertia_probe.py, a +-0.20 yaw-diagonal step at hover
        with the modelled reaction torque OFF still yawed the aircraft at 17 rad/s^2 --
        8.3 N.m of torque no controller knows about, 3.3x the modelled reaction. In HIL
        that is the hover yaw limit cycle (rail-to-rail diagonal at 2.7 Hz, the 2026-09-12 23:55 hover bag,
        fitted as an effective Izz of 0.11 against the true
        0.49) and, through the tilted front pair, a spurious ROLL torque in cruise.
        The offline gate never had it. The cap is raised to 1.5x omega_max; nothing
        else changes -- the modelled reaction torque (rotor_moment_ratio) stays."""
        stage = self._current_stage
        prim = stage.GetPrimAtPath(prop_path)
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        cap = 1.5 * max(float(self._cfg.rotor_omega_max), 100.0)
        api.CreateMaxAngularVelocityAttr().Set(float(math.degrees(cap)))

    def _build_surface(self, surface: SurfaceConfig):
        """Build a hinged control-surface link off the body."""
        cfg = self._cfg
        prefix = self._stage_prefix
        surf_path = f"{prefix}/{surface.surface_link}"

        surf_box = None if surface.mesh is not None else (surface.size or cfg.surface_size)
        self._add_rigid_link(
            path=surf_path, size=surf_box,
            mass=cfg.link_mass, inertia=cfg.link_inertia, color=(0.45, 0.45, 0.50),
            mesh=surface.mesh, mesh_scale=surface.mesh_scale,
            mesh_offset=surface.mesh_offset, mesh_rpy_deg=surface.mesh_rpy_deg,
        )
        self._add_revolute_joint(
            name=surface.hinge_joint,
            parent=f"{prefix}/body", child=surf_path,
            anchor_in_parent=surface.position, axis=surface.hinge_axis,
            lower_deg=-surface.max_deflection_deg, upper_deg=surface.max_deflection_deg,
            drive="position",
            stiffness=cfg.surface_drive_stiffness, damping=cfg.surface_drive_damping,
        )

    def _add_rigid_link(self, path, mass, inertia, size=None, color=(0.85, 0.85, 0.85),
                        collision=False, mesh=None, mesh_scale=(1.0, 1.0, 1.0),
                        mesh_offset=(0.0, 0.0, 0.0), mesh_rpy_deg=(0.0, 0.0, 0.0),
                        com_offset=None):
        """Create one rigid-body link with explicit mass+inertia.

        Geometry is one of:
          - imported CAD mesh   (`mesh` = USD path)  -> referenced as a child so it
                                                        can be scaled/offset/oriented
          - procedural box      (`size` given)
          - nothing             (invisible connector link, e.g. a mesh rotor's tilt
                                  nacelle — still a rigid body with mass/inertia)

        Links carry explicit inertia so they can be collision-free, avoiding
        spurious self-collisions. Mesh links are visual-only (no PhysX collision;
        a triangle mesh can't be a dynamic collider — add convex collision in the
        GUI if you need ground contact)."""
        stage = self._current_stage

        if mesh is not None:
            # Rigid body = Xform; geometry is a referenced child, so the mesh
            # transform never touches the link/joint frame.
            prim = UsdGeom.Xform.Define(stage, Sdf.Path(path)).GetPrim()
            # `geo` carries OUR scale/offset/orient (fresh xformOps). The CAD mesh
            # is referenced into a grandchild so ITS own xformOps don't clash with
            # ours (referencing a USD into a prim pulls its xformOpOrder in).
            geo = UsdGeom.Xform.Define(stage, Sdf.Path(f"{path}/geometry"))
            geo.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in mesh_offset]))
            geo.AddRotateXYZOp().Set(Gf.Vec3f(*[float(v) for v in mesh_rpy_deg]))
            geo.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in mesh_scale]))
            stage.DefinePrim(f"{path}/geometry/mesh", "Xform").GetReferences().AddReference(mesh)
        elif size is not None:
            cube = UsdGeom.Cube.Define(stage, Sdf.Path(path))
            cube.GetSizeAttr().Set(1.0)
            UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(*size))
            cube.CreateDisplayColorAttr().Set([Gf.Vec3f(*color)])
            prim = cube.GetPrim()
        else:
            # Invisible connector link (has mass/inertia, no geometry).
            prim = UsdGeom.Xform.Define(stage, Sdf.Path(path)).GetPrim()

        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.GetMassAttr().Set(float(mass))
        mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*inertia))
        # Center-of-mass offset (relative to the link frame). Used on the body so
        # the vehicle CG sits at the rotor centroid -> balanced hover thrust across
        # all rotors -> actuator headroom for yaw (a front-heavy rotor layout
        # otherwise runs the front rotors near idle and starves yaw authority).
        if com_offset is not None:
            mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*[float(v) for v in com_offset]))

        # Collision only for a procedural BOX body (a dynamic triangle-mesh
        # collider is invalid in PhysX; mesh bodies stay collision-free here).
        if collision and mesh is None and size is not None:
            UsdPhysics.CollisionAPI.Apply(prim)

        if self._cfg.disable_gravity:
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).GetDisableGravityAttr().Set(True)

        return prim

    def _add_revolute_joint(self, name, parent, child, anchor_in_parent, axis,
                            lower_deg=None, upper_deg=None,
                            drive=None, stiffness=0.0, damping=0.0):
        """Create a single-DOF revolute joint named `name` connecting
        `parent` -> `child`.

        - `anchor_in_parent`: joint location in the parent's local frame [m].
          The child link's origin sits at the same point, so its local anchor
          is (0,0,0).
        - `axis`: "X"/"Y"/"Z" principal axis of rotation.
        - `drive`: None | "position" | "velocity".  A drive must exist for the
          runtime DOF target calls in `_drive_actuators` to take effect.

        The leaf name `name` is also the DOF name used by the dynamic-control
        interface (`find_articulation_dof`), which is how build & runtime stay
        connected."""
        stage = self._current_stage
        joint_path = f"{self._stage_prefix}/joints/{name}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, Sdf.Path(joint_path))

        joint.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        joint.CreateAxisAttr(axis)

        # Joint frame placement: anchored at `anchor_in_parent` in the parent,
        # and at the child's own origin.
        joint.CreateLocalPos0Attr(Gf.Vec3f(*anchor_in_parent))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        if lower_deg is not None and upper_deg is not None:
            joint.CreateLowerLimitAttr(float(lower_deg))
            joint.CreateUpperLimitAttr(float(upper_deg))

        if drive is not None:
            d = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            d.CreateTypeAttr("force")
            d.CreateStiffnessAttr(float(stiffness))   # position drive -> high; velocity drive -> 0
            d.CreateDampingAttr(float(damping))
            d.CreateTargetPositionAttr(0.0)
            d.CreateTargetVelocityAttr(0.0)

        return joint

    # ========================================================================
    #  (2) DRIVE  — called every physics step by the base Vehicle class.
    # ========================================================================

    def start(self):
        """No-op hook (kept for symmetry with other vehicles)."""
        pass

    def stop(self):
        pass

    def _spin_model(self) -> str:
        """Which thrust model is active, honouring the `rotor_dynamics` back-compat alias.
        ONE place decides, because the answer gates BOTH the omega integration and the thrust
        computation -- and getting those out of step means omega stays 0 while the thrust model
        divides by it, i.e. no thrust at all."""
        model = getattr(self._cfg, "thrust_model", "instant")
        if self._cfg.rotor_dynamics and model == "instant":
            model = "lag"
        return model

    def update(self, dt: float):
        """Per-step orchestration.  Each numbered block maps 1:1 to the MAP OF
        INTERACTIONS at the top of the file."""

        # (a) WHERE COMMANDS COME IN ----------------------------------------
        command = self._read_command()
        self._validate_channels(command)

        # (a.5) ROTOR SPIN-UP DYNAMICS (opt-in) -----------------------------
        #   Integrate each rotor's speed toward its commanded speed so thrust and
        #   the visual spin both lag throttle. No-op for thrust_model='instant'.
        if self._spin_model() in ("lag", "momentum"):
            self._update_rotor_speeds(command, dt)

        # (b) WHERE JOINTS ARE MOVED (tilt servos, prop spin, surfaces) ------
        self._drive_actuators(command, dt)

        # (c)/(d) WHERE PHYSICS FORCES ARE APPLIED (Step 2) ------------------
        self._update_wind(dt)
        if self._cfg.enable_forces:
            self._apply_propulsion(command)
            self._apply_aerodynamics(command)

        # (e) WHERE BACKENDS ARE TICKED -------------------------------------
        for backend in self._backends:
            backend.update(dt)

    def _read_command(self) -> dict:
        """Pull the actuator command from the first backend and normalise it to
        a `{channel_name: float}` dict.

        A backend may return either that dict directly, or a flat list (in
        which case it is zipped onto the declared channel order).  Unknown /
        missing channels default to 0.0."""
        raw = self._backends[0].input_reference() if self._backends else {}

        if isinstance(raw, Mapping):
            return {k: float(v) for k, v in raw.items()}

        # Flat list/array -> map onto channels in declaration order.
        if isinstance(raw, Sequence) and not isinstance(raw, (str, bytes)):
            return dict(zip(self.actuator_channels, [float(v) for v in raw]))

        return {}

    def _validate_channels(self, command: dict):
        """One-shot bring-up diagnostic: on the first NON-EMPTY command, warn if
        none of its keys match this airframe's declared channels.

        This is the cheapest catch for the #1 VTOL gotcha — a backend channel_map
        (or PX4 actuator index order) that doesn't line up with the airframe.
        Without it the symptom is a silent 'no actuator moves / vehicle falls'
        with no log.  An empty command is the legitimate disarmed/safe state, so
        it is ignored."""
        if self._channels_checked or not command:
            return
        self._channels_checked = True
        declared = set(self.actuator_channels)
        if declared.isdisjoint(command.keys()):
            import carb
            carb.log_warn(
                f"[VTOL {self._stage_prefix}] command channels {sorted(command)} do NOT "
                f"match any declared airframe channel {sorted(declared)} — no actuator "
                f"will move. Check the backend channel_map / PX4 actuator index order "
                f"(re-run with PEGASUS_HIL_DEBUG=1)."
            )

    def _update_rotor_speeds(self, command: dict, dt: float):
        """First-order rotor spin-up model (thrust_model 'lag' and 'momentum' only).

        Each rotor's speed omega [rad/s] chases its commanded speed
        omega_cmd = throttle * rotor_omega_max with time constant rotor_tau_spin:

            omega += (omega_cmd - omega) * dt / tau

        The integrated omega then drives BOTH the visual prop spin
        (_drive_actuators) and the thrust (_apply_propulsion, thrust ~ omega^2),
        so both lag throttle the way a real rotor's inertia makes them."""
        if dt <= 0.0:
            return
        cfg = self._cfg
        tau = max(cfg.rotor_tau_spin, 1e-3)
        for rotor in self._rotors:
            throttle = float(np.clip(command.get(rotor.throttle_channel, 0.0), 0.0, 1.0))
            omega_cmd = throttle * cfg.rotor_omega_max
            omega = self._omega[rotor.name]
            omega += (omega_cmd - omega) * min(dt / tau, 1.0)
            self._omega[rotor.name] = omega

    def _drive_actuators(self, command: dict, dt: float):
        """Move every joint to match the command:  tilt servos slew, props spin,
        surfaces deflect.  Spin rate and tilt response depend on whether
        the thrust model spins rotors up (real spin-up / servo lag) or not (legacy
        instantaneous visual spin + pure rate limit)."""
        dc = self.get_dc_interface()
        art = dc.get_articulation(self._stage_prefix)
        if not art:                              # articulation not ready yet
            return
        dc.wake_up_articulation(art)
        dynamics = self._spin_model() in ("lag", "momentum")

        # --- rotors: spin (always) + tilt (if tiltable) --------------------
        for rotor in self._rotors:
            throttle = float(np.clip(command.get(rotor.throttle_channel, 0.0), 0.0, 1.0))

            # Prop spin (velocity-driven DOF): true modelled omega when dynamics
            # are on, else a visual-only heuristic from throttle.
            spin_speed = self._omega[rotor.name] if dynamics else self._visual_spin_speed(throttle)
            spin_dof = dc.find_articulation_dof(art, rotor.spin_joint)
            if spin_dof:
                dc.set_dof_velocity_target(spin_dof, spin_speed * rotor.spin_dir)

            # Tilt servo (position-driven DOF).
            if rotor.tiltable:
                target = self._tilt_target_rad(rotor, command)
                current = self._tilt_state[rotor.name]
                max_step = math.radians(rotor.tilt_rate_deg_s) * dt
                if dynamics:
                    # First-order servo response, then clamped to the slew rate.
                    step = (target - current) * min(dt / max(self._cfg.tilt_tau, 1e-3), 1.0)
                    current += float(np.clip(step, -max_step, max_step))
                else:
                    # Legacy: pure rate limit toward the target.
                    current += float(np.clip(target - current, -max_step, max_step))
                self._tilt_state[rotor.name] = current

                tilt_dof = dc.find_articulation_dof(art, rotor.tilt_joint)
                if tilt_dof:
                    dc.set_dof_position_target(tilt_dof, current)

        # --- control surfaces: deflect to commanded angle ------------------
        for surface in self._surfaces:
            cmd = float(np.clip(command.get(surface.deflect_channel, 0.0), -1.0, 1.0))
            target = cmd * surface.deflect_scale * math.radians(surface.max_deflection_deg)
            # SERVO: first-order lag, then a slew-rate clamp -- the same order a real servo
            # behaves in (the loop closes on position, but the output shaft cannot exceed its
            # no-load speed). Both default to 0, which reproduces the legacy instant surface.
            cur = self._surface_state.get(surface.name, 0.0)
            tau = float(self._cfg.surface_tau)
            step = (target - cur) * (min(dt / tau, 1.0) if tau > 1e-6 else 1.0)
            rate = float(self._cfg.surface_rate_deg_s)
            if rate > 0.0:
                lim = math.radians(rate) * dt
                step = float(np.clip(step, -lim, lim))
            cur += step
            self._surface_state[surface.name] = cur
            hinge_dof = dc.find_articulation_dof(art, surface.hinge_joint)
            if hinge_dof:
                dc.set_dof_position_target(hinge_dof, cur)

    # ---- small command helpers --------------------------------------------

    @staticmethod
    def _visual_spin_speed(throttle: float) -> float:
        """Map throttle -> a believable prop spin rate [rad/s] (visual only)."""
        if throttle < 0.02:
            return 0.0
        return 30.0 + 120.0 * throttle

    @staticmethod
    def _tilt_target_rad(rotor: RotorConfig, command: dict) -> float:
        """Map the tilt channel value to an absolute nacelle angle [rad].

        ANCHORED AT ZERO: `cmd` scales against tilt_max, so **cmd = 0 is always 0 deg
        (hover)** and cmd = 1 is tilt_max.  The result is then clipped to
        [tilt_min_deg, tilt_max_deg], which lets an airframe expose NEGATIVE tilt
        (rotors leaning backwards) without moving the zero point.

        This is deliberately NOT `lo + (hi-lo)*cmd`.  With that form, giving a rotor
        tilt_min_deg = -20 would silently make hover's cmd = 0 mean -20 deg -- and the
        SAFE controller writes literal zeros on the tilt channels.  The project has
        already paid for exactly this once: a channel map of (4, 0.5, 0.5) mapped
        hover's 0 to 0.5 = 45 deg and the aircraft tilted on arm.
        For every airframe with tilt_min_deg = 0 the two forms are identical, so this
        change is backward compatible.
        """
        cmd = float(command.get(rotor.tilt_channel, 0.0))
        ang = cmd * math.radians(rotor.tilt_max_deg)
        return float(np.clip(ang, math.radians(rotor.tilt_min_deg),
                                  math.radians(rotor.tilt_max_deg)))

    # ========================================================================
    #  (3) PHYSICS FORCES  — Step 2.  Implemented but gated by enable_forces.
    # ========================================================================

    def _update_wind(self, dt: float):
        """The world wind this step: the steady VTOL_WIND plus a Gauss-Markov gust (VTOL_TURB,
        VTOL_TURB_TAU), and its body-frame image for the aero and the props. Still air -> zeros,
        and every consumer subtracts a zero: byte-identical to the flights before piece 4. The
        world wind is also written to the state (state.wind_world) for the backend's pitot."""
        cfg = self._cfg
        steady = np.asarray(getattr(cfg, "wind_world", (0.0, 0.0, 0.0)), dtype=float)
        sig = float(getattr(cfg, "turb_sigma", 0.0) or 0.0)
        if sig > 0.0 and dt > 0.0:
            a = math.exp(-dt / max(float(getattr(cfg, "turb_tau", 1.0) or 1.0), 1e-3))
            self._turb = a * getattr(self, "_turb", np.zeros(3)) + math.sqrt(max(0.0, 1.0 - a * a)) * sig * np.random.normal(0.0, 1.0, 3)
        else:
            self._turb = np.zeros(3)
        self._wind_world = steady + self._turb
        if np.any(self._wind_world != 0.0):
            try:
                from scipy.spatial.transform import Rotation
                self._wind_body = Rotation.from_quat(self._state.attitude).inv().apply(self._wind_world)
            except Exception:                  # noqa: BLE001 - state not ready on the first steps
                self._wind_body = np.zeros(3)
        else:
            self._wind_body = np.zeros(3)
        try:
            self._state.wind_world = self._wind_world
        except Exception:                      # noqa: BLE001
            pass
        if (sig > 0.0 or np.any(steady != 0.0)) and not getattr(self, "_wind_logged", False):
            self._wind_logged = True
            import carb
            carb.log_warn(f"[VTOL] wind: steady ({steady[0]:+.1f},{steady[1]:+.1f},{steady[2]:+.1f}) m/s ENU"
                          + (f", turbulence {sig:.2f} m/s 1-sigma tau {float(getattr(cfg, 'turb_tau', 1.0)):.1f} s" if sig > 0 else ""))

    def _apply_propulsion(self, command: dict):
        """Apply each rotor's thrust along the prop link's LOCAL +Z (and, with
        rotor_dynamics, its yaw reaction torque about the prop spin axis).

        Why local +Z works for a tiltrotor: the prop link is a child of the
        (tiltable) nacelle, so PhysX has already rotated the link by the tilt
        angle.  A constant body-local +Z force therefore points 'up' in hover
        and 'forward' in cruise automatically — no thrust-vector trig needed here.

        Thrust model:
          - rotor_dynamics OFF: thrust = throttle * max_thrust (instantaneous).
          - rotor_dynamics ON : thrust = max_thrust * (omega/omega_max)^2, using
            the spun-up omega, so it lags throttle and scales with RPM^2. A yaw
            reaction torque = rotor_moment_ratio * thrust * (-spin_dir) about the
            prop's local +Z gives differential-torque yaw authority in hover."""
        cfg = self._cfg
        model = self._spin_model()
        dynamics = model in ("lag", "momentum")
        omega_max_sq = max(cfg.rotor_omega_max, 1e-6) ** 2
        # body-frame airspeed, for the inflow term. Each rotor sees the component along its OWN
        # (tilted) thrust axis -- at 90 deg tilt that is the cruise airspeed, which is exactly
        # where a flat thrust model flatters the aircraft most.
        try:
            v_body = np.asarray(self._state.linear_body_velocity, dtype=float) - getattr(self, "_wind_body", np.zeros(3))   # AIR-relative (piece 4)
        except Exception:                      # noqa: BLE001 - state not ready on the first steps
            v_body = np.zeros(3)
        # ONE source of truth for air density: the aero model's own config, so propulsion and
        # aerodynamics can never disagree about what air they are in.
        rho = float(getattr(getattr(self._aero, "config", None), "rho", 1.225))
        _dbg = os.environ.get("PEGASUS_PROP_DEBUG")
        _dbg_now = _dbg and (getattr(self, "_prop_dbg_ctr", 0) % 120 == 0)
        self._prop_dbg_ctr = getattr(self, "_prop_dbg_ctr", 0) + 1
        _dbg_parts = []
        for rotor in self._rotors:
            th_now = self._tilt_state.get(rotor.name, 0.0) if rotor.tiltable else 0.0
            if dynamics:
                thrust = rotor.max_thrust * (self._omega[rotor.name] ** 2) / omega_max_sq
                if model == "momentum":
                    # Propeller thrust falls with ADVANCE RATIO J = V_axial / (n*D). Written as
                    #     T = rho*D^2*( CT0*(nD)^2 - (CT0/J0)*(nD)*V_axial )
                    # which is the standard linear-in-J form with no singularity at n = 0, and
                    # reduces to CT0*rho*n^2*D^4 = max_thrust when V_axial = 0 and omega = omega_max.
                    # CT0 is DERIVED from the rotor's own (max_thrust, omega_max, D), so STATIC
                    # thrust is preserved exactly and only the airspeed behaviour is added.
                    D = float(rotor.prop_D)
                    J0 = max(float(rotor.prop_J0), 1e-3)
                    n_max = max(cfg.rotor_omega_max, 1e-6) / (2.0 * math.pi)
                    ct0 = rotor.max_thrust / max(rho * n_max ** 2 * D ** 4, 1e-9)
                    n = self._omega[rotor.name] / (2.0 * math.pi)
                    v_ax = float(v_body[0] * math.sin(th_now) + v_body[2] * math.cos(th_now))
                    thrust = rho * D * D * (ct0 * (n * D) ** 2
                                                    - (ct0 / J0) * (n * D) * v_ax)
                    thrust = max(0.0, thrust)   # a windmilling prop is not modelled as drag here
            else:
                throttle = float(np.clip(command.get(rotor.throttle_channel, 0.0), 0.0, 1.0))
                thrust = throttle * rotor.max_thrust
            if _dbg_now:
                _th = self._tilt_state.get(rotor.name, 0.0) if rotor.tiltable else 0.0
                _dbg_parts.append("%s T=%.1f tilt=%.0f" % (rotor.name, thrust, math.degrees(_th)))
            if thrust <= 0.0:
                continue
            # Thrust direction in the BODY frame, computed EXPLICITLY from the tilt
            # angle (NOT the prop link's USD +Z). A tiltable nacelle rotates about +Y,
            # so its thrust axis is (sin th, 0, cos th): up at th=0 (hover), forward at
            # th=90deg (cruise). Fixed rotors are always vertical. Applying to the body
            # at the rotor position is robust to how PhysX resolves the prop-link frame
            # (relying on the prop link's local +Z aimed the front-rotor thrust off
            # vertical, making pitch uncontrollable).
            th = th_now
            sx, cz = math.sin(th), math.cos(th)
            pos = [float(v) for v in rotor.position]
            self.apply_force([thrust * sx, 0.0, thrust * cz], pos=pos, body_part="/body")
            # Yaw reaction (rotor drag) torque about the (tilted) spin/thrust axis.
            # Real physics regardless of the spin-up LAG (rotor_dynamics) — gives
            # differential-torque yaw authority in hover.
            if cfg.rotor_moment_ratio != 0.0:
                k = -rotor.spin_dir * cfg.rotor_moment_ratio * thrust
                self.apply_torque([k * sx, 0.0, k * cz], body_part="/body")
        if _dbg_now and _dbg_parts:
            import carb
            carb.log_warn("[PROP] " + " | ".join(_dbg_parts))

    def _apply_aerodynamics(self, command: dict):
        """Apply lumped wing/body aerodynamic force + moment to the body link.

        The VTOLAerodynamics model expects aileron/elevator/rudder inputs, so we
        mix the two elevons into virtual aileron/elevator here.  (Step 2: refine
        the elevon mixing / add slipstream effects.)"""
        # ★ FEED THE AERO WHAT THE SURFACES ARE ACTUALLY DOING. With a servo configured the
        # actual deflection lags and rate-limits behind the command; reading the command here
        # would model an instantaneous surface no matter what the actuator is set to.
        aero_command = self._aero_inputs_from_surfaces(self._surface_command_equiv())
        _wb = getattr(self, "_wind_body", None)
        force_body, moment_body = self._aero.update(self._state, aero_command,
                                                    wind_body=(_wb if _wb is not None and np.any(_wb != 0.0) else None))
        # `pos` is measured from the body link ORIGIN; PhysX takes the moment
        # about the CoM.  Applying at self._aero_center (default: the CoM)
        # keeps the aerodynamic moment entirely in the moment coefficients
        # instead of in an accidental lever arm -- see VTOLConfig.aero_center.
        self.apply_force(np.asarray(force_body).tolist(),
                         pos=self._aero_center, body_part="/body")
        self.apply_torque(np.asarray(moment_body).tolist(), body_part="/body")
        if os.environ.get("PEGASUS_PROP_DEBUG") and getattr(self, "_prop_dbg_ctr", 0) % 120 == 1:
            import carb
            st = self._aero.last_state
            # NOTE: aoa/beta are the STANDARD aerodynamic angles (aoa > 0 =
            # nose-up / wind from below).  Bags and logs from before the
            # 2026-09-06 frame fix printed the opposite sign.
            carb.log_warn("[AERO] V=%.1f aoa=%.1f beta=%.1f Fbody=(%.1f,%.1f,%.1f) coeffs=%s"
                          % (st.airspeed, math.degrees(st.alpha), math.degrees(st.beta),
                             force_body[0], force_body[1], force_body[2], st.coefficients))

    def _surface_command_equiv(self) -> dict:
        """The ACTUAL deflections, re-expressed as the normalised [-1,1] values the aero model
        consumes (it multiplies by its own `max_surface_deflection_rad`). `deflect_scale` is
        divided back out so a -1 mirrored aileron reports the same sign convention it was
        commanded in."""
        out = {}
        for sf in self._surfaces:
            span = math.radians(sf.max_deflection_deg) * (sf.deflect_scale or 1.0)
            out[sf.deflect_channel] = (self._surface_state.get(sf.name, 0.0) / span
                                       if abs(span) > 1e-9 else 0.0)
        return out

    @staticmethod
    def _aero_inputs_from_surfaces(command: dict) -> dict:
        """Build the {aileron, elevator, rudder} inputs the aero model wants
        from whatever surfaces the airframe has:

        - Conventional airframe (surfaces literally named aileron/elevator/
          rudder, e.g. dual_tiltrotor_config) -> pass through directly.
        - Elevon airframe (the default tri-tiltrotor) -> mix the two elevons:
          aileron = (L - R)/2, elevator = (L + R)/2.
        """
        if any(k in command for k in ("aileron", "elevator", "rudder")):
            return {
                "aileron": float(command.get("aileron", 0.0)),
                "elevator": float(command.get("elevator", 0.0)),
                "rudder": float(command.get("rudder", 0.0)),
            }
        left = float(command.get("elevon_L", 0.0))
        right = float(command.get("elevon_R", 0.0))
        return {
            "aileron": 0.5 * (left - right),
            "elevator": 0.5 * (left + right),
            "rudder": 0.0,
        }

    # ========================================================================
    #  Introspection helpers
    # ========================================================================

    @property
    def aerodynamics(self) -> VTOLAerodynamics:
        return self._aero

    @property
    def total_mass(self) -> float:
        """Best-effort total mass [kg] from the config: body link + the small
        links built per rotor/surface.  Handy for hover-thrust feed-forward.

        Link count: 2 per tiltable rotor (nacelle + prop), 1 per fixed rotor
        (prop), 1 per control surface — matching what `_init_prim` builds."""
        n_links = sum(2 if r.tiltable else 1 for r in self._rotors) + len(self._surfaces)
        return self._cfg.mass + n_links * self._cfg.link_mass

    @property
    def actuator_channels(self) -> list[str]:
        """The ordered list of command channel names this airframe consumes.
        Backends should produce exactly these keys.  Useful when wiring a new
        control source (e.g. mapping PX4 actuator outputs)."""
        channels: list[str] = []
        for rotor in self._rotors:
            channels.append(rotor.throttle_channel)
            if rotor.tiltable and rotor.tilt_channel and rotor.tilt_channel not in channels:
                channels.append(rotor.tilt_channel)
        for surface in self._surfaces:
            channels.append(surface.deflect_channel)
        return channels

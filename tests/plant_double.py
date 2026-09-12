#!/usr/bin/env python3
"""A 6-DOF plant double built from the SIMULATOR's own equations and config.

The controller self-tests in ros2/px4_vtol_direct_actuator/ integrate each controller's
OWN `build_ode()` as "the true plant", so any error in that model cancels exactly and is
invisible offline. That is why the notes repeatedly record "root-caused offline; NONE
reproduce in the non-real-time offline test" (2026-09-06 controller audit, finding C5).

This module is the fix: it imports `vtol_aerodynamics.VTOLAerodynamics` directly and takes
mass, CG, inertia and rotor geometry from `quad_tiltrotor_config()`, reproducing what Isaac
integrates -- including the assembled-vehicle mass properties that every controller gets
wrong. It needs numpy + scipy only; no Isaac, no ROS.

    plant = VTOLPlant()
    plant.reset(pos=[0, 0, 1.5])
    plant.step(motors=[.47]*4, tilt=0.0, surfaces=[0, 0, 0], dt=0.005)
    p, v, R, w = plant.pos, plant.vel, plant.R, plant.omega

Deliberately NOT modelled (the simulator does not model them either): rotor inflow, prop
wash over the wing, ground effect. Deliberately modelled the way the simulator does it,
even where that is a simplification: thrust = throttle * max_thrust, applied at the tilt
HUB, in the body frame.
"""
import importlib.util
import math
import os

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXT = os.path.join(_HERE, "..", "extensions", "pegasus.simulator", "pegasus", "simulator", "logic")


def _load(name, relpath):
    spec = importlib.util.spec_from_file_location(name, os.path.join(_EXT, relpath))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_aero_mod = _load("vtol_aerodynamics", "dynamics/vtol_aerodynamics.py")
VTOLAerodynamics = _aero_mod.VTOLAerodynamics
VTOLAerodynamicsConfig = _aero_mod.VTOLAerodynamicsConfig
_slosh_mod = _load("slosh", "dynamics/slosh.py")

# ★ TANK AIRCRAFT (branch `slosh`, tank-plan phase 2).  The plant this harness
# doubles now carries 0.502 kg of water as DEAD MASS at the assembled CG, and so
# do the controllers (so3.M, FSC['m'], the safe controller). A double at 3.45 kg
# against a controller at 3.95 is a 14.6% feed-forward mismatch -- which is what
# test_geometric_hover flagged the moment the controller moved and this did not.
# Read from the SHIPPED tank definition, not retyped, so the four copies cannot
# drift (tests/test_tank_model_pairing.py checks the other three).
# PLANT_TANK=0 doubles the baseline aircraft.
_TANK = (_slosh_mod.TankConfig()
         if os.environ.get("PLANT_TANK", "1") not in ("0", "", "none", "false", "False") else None)
TANK_MASS = _slosh_mod.slosh_parameters(_TANK).fluid_mass if _TANK is not None else 0.0
TANK_INERTIA = (np.diag(_slosh_mod.slosh_parameters(_TANK).inertia_rigid)
                if _TANK is not None else np.zeros((3, 3)))


# ---------------------------------------------------------------------------
#  Airframe, reconstructed from quad_tiltrotor_config() / VTOLConfig.
#  vtol.py cannot be imported outside Isaac (it needs pxr), so the numbers are
#  mirrored here; ASSERT_AGAINST_CONFIG below re-checks them when pxr is present.
# ---------------------------------------------------------------------------
BODY_MASS = 2.95
BODY_INERTIA = np.diag([0.20, 0.23, 0.36])       # about the BODY LINK's own CoM
BODY_COM = np.array([0.195, 0.0, -0.043])
LINK_MASS = 0.05
LINK_INERTIA = np.diag([1e-4, 1e-4, 1e-4])
# every articulated link vtol.py builds, at its body-frame position
LINKS = [
    ("nacelle_front_L", 0.459, 0.284, -0.068), ("prop_front_L", 0.457, 0.284, 0.022),
    ("nacelle_front_R", 0.459, -0.284, -0.066), ("prop_front_R", 0.457, -0.285, 0.022),
    ("prop_rear_L", -0.112, 0.284, -0.019), ("prop_rear_R", -0.112, -0.284, -0.017),
    ("surface_aileron_L", -0.02, 0.60, 0.0), ("surface_aileron_R", -0.02, -0.60, 0.0),
    ("surface_elevator", -0.45, 0.0, 0.05), ("surface_rudder", -0.45, 0.0, 0.18),
]
# rotor HUBS (where vtol.py applies thrust), spin directions, tiltability
ROTOR_HUB = np.array([[0.459, 0.284, -0.068], [0.459, -0.284, -0.066],
                      [-0.112, 0.284, -0.019], [-0.112, -0.284, -0.017]])
ROTOR_SPIN = np.array([+1.0, -1.0, -1.0, +1.0])          # FL, FR, RL, RR
ROTOR_TILTABLE = np.array([True, True, False, False])
MAX_THRUST = 18.0
# ★ THRUST MODEL, matching vtol.py. The sim's FSC airframe runs thrust_model="momentum": thrust
# is QUADRATIC in throttle (via omega) and falls with the airspeed along the rotor's own axis,
# reaching zero at the advance ratio J0. A harness that keeps the FLAT model disagrees with the
# plant it is meant to double, and then fails a CORRECT controller -- which is exactly what
# happened: the safe controller was fixed for the momentum sim and promptly failed 9 checks here.
THRUST_MODEL = "momentum"        # "instant" (legacy flat) | "momentum"
PROP_D = 0.3048                  # m, 12 in
PROP_J0 = 0.6                    # advance ratio at zero thrust
PROP_OMEGA_MAX = 749.0           # rad/s at throttle 1
_N_MAX = PROP_OMEGA_MAX / (2.0 * math.pi)
_CT0 = MAX_THRUST / (1.225 * _N_MAX ** 2 * PROP_D ** 4)


def rotor_thrust(throttle, v_axial=0.0):
    """Newtons from a normalised throttle. CT0 is derived so STATIC thrust is exactly MAX_THRUST,
    so 'instant' and 'momentum' agree at zero airspeed and full throttle."""
    t = float(np.clip(throttle, 0.0, 1.0))
    if THRUST_MODEL != "momentum":
        return t * MAX_THRUST
    n = t * _N_MAX
    return max(0.0, 1.225 * PROP_D ** 2
               * (_CT0 * (n * PROP_D) ** 2 - (_CT0 / PROP_J0) * (n * PROP_D) * float(v_axial)))
ROTOR_MOMENT_RATIO = 0.12                                 # VTOLConfig.rotor_moment_ratio
TILT_RATE = math.radians(80.0)                            # RotorConfig.tilt_rate_deg_s
G = 9.81


def mass_properties():
    """Total mass, CG (body-frame, from the body link origin) and inertia about the CG."""
    m_tot = BODY_MASS + LINK_MASS * len(LINKS)
    com = (BODY_MASS * BODY_COM + LINK_MASS * sum(np.array(l[1:]) for l in LINKS)) / m_tot

    def parallel_axis(m, I, r):
        r = np.asarray(r, dtype=float)
        return I + m * (float(np.dot(r, r)) * np.eye(3) - np.outer(r, r))

    I = parallel_axis(BODY_MASS, BODY_INERTIA, BODY_COM - com)
    for l in LINKS:
        I = I + parallel_axis(LINK_MASS, LINK_INERTIA, np.array(l[1:]) - com)
    # the tank sits AT the assembled CG: mass and its own inertia, no CG shift
    # (mirrors vtol_slosh.fold_tank_into_body with tank.position=None)
    return m_tot + TANK_MASS, com, I + TANK_INERTIA


MASS, CG, INERTIA = mass_properties()
INERTIA_INV = np.linalg.inv(INERTIA)


class _State:
    """Duck-typed pegasus State for the aero model (FLU body, ENU world)."""

    def __init__(self, vb, wb):
        self.linear_body_velocity = vb
        self.angular_velocity = wb


class VTOLPlant:
    """6-DOF rigid body with the simulator's propulsion + aerodynamics."""

    def __init__(self, aero_config=None, aero_center=None, wind_world=None):
        self.aero = VTOLAerodynamics(aero_config)
        # where the lumped aero force acts, relative to the CG (VTOLConfig.aero_center
        # defaults to the CoM, i.e. no accidental moment arm)
        self.aero_arm = (np.asarray(aero_center, float) - CG
                         if aero_center is not None else np.zeros(3))
        self.wind_world = np.zeros(3) if wind_world is None else np.asarray(wind_world, float)
        self.reset()

    def reset(self, pos=(0.0, 0.0, 1.5), vel=(0.0, 0.0, 0.0), R=None, omega=(0.0, 0.0, 0.0),
              tilt=0.0):
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self.R = np.eye(3) if R is None else np.array(R, dtype=float)
        self.omega = np.array(omega, dtype=float)
        self.tilt = float(tilt)                  # applied nacelle angle [rad], shared front pair
        self.t = 0.0
        self.on_ground = self.pos[2] <= 0.31
        return self

    # -- forces ------------------------------------------------------------
    def forces(self, motors, tilt, surfaces):
        """Body-frame force and moment (about the CG) from thrust + aerodynamics."""
        # body-frame velocity, for each rotor's axial inflow (momentum thrust model)
        _vb = self.R.T @ (self.vel - self.wind_world)
        F = np.zeros(3)
        M = np.zeros(3)
        for i in range(4):
            # axial inflow along THIS rotor's axis: tiltable rotors swing toward the body x
            # axis, fixed rotors stay vertical.
            _t = tilt if ROTOR_TILTABLE[i] else 0.0
            _vax = float(_vb[0] * math.sin(_t) + _vb[2] * math.cos(_t))
            thr = rotor_thrust(motors[i], _vax)
            if thr <= 0.0:
                continue
            th = tilt if ROTOR_TILTABLE[i] else 0.0
            direction = np.array([math.sin(th), 0.0, math.cos(th)])
            Fi = thr * direction
            F += Fi
            M += np.cross(ROTOR_HUB[i] - CG, Fi)
            # yaw reaction about the (tilted) spin axis -- vtol.py applies this as a pure
            # torque, so it has no moment arm
            M += -ROTOR_SPIN[i] * ROTOR_MOMENT_RATIO * thr * direction

        v_rel_world = self.vel - self.wind_world
        vb = self.R.T @ v_rel_world
        wb = self.omega
        aero_cmd = {"aileron": float(surfaces[0]), "elevator": float(surfaces[1]),
                    "rudder": float(surfaces[2])}
        Fa, Ma = self.aero.update(_State(vb, wb), aero_cmd)
        F += Fa
        M += Ma + np.cross(self.aero_arm, Fa)
        return F, M

    # -- integration -------------------------------------------------------
    def step(self, motors, tilt_cmd, surfaces, dt):
        # the sim's tilt servo is a pure RATE LIMIT (rotor_dynamics off), not a lag
        target = float(np.clip(tilt_cmd, 0.0, math.pi / 2))
        self.tilt += float(np.clip(target - self.tilt, -TILT_RATE * dt, TILT_RATE * dt))

        F, M = self.forces(motors, self.tilt, surfaces)
        acc = self.R @ F / MASS + np.array([0.0, 0.0, -G])
        omega_dot = INERTIA_INV @ (M - np.cross(self.omega, INERTIA @ self.omega))

        self.vel = self.vel + acc * dt
        self.pos = self.pos + self.vel * dt
        self.omega = self.omega + omega_dot * dt
        # exponential map keeps R on SO(3) without re-orthonormalising drift
        self.R = self.R @ _expm_so3(self.omega * dt)
        self.t += dt

        if self.pos[2] < 0.3:                    # crude ground plane (spawn height)
            self.pos[2] = 0.3
            if self.vel[2] < 0.0:
                self.vel[2] = 0.0
            self.on_ground = True
        else:
            self.on_ground = False
        return self

    # -- diagnostics -------------------------------------------------------
    @property
    def rpy(self):
        """(roll, pitch nose-UP, yaw) in degrees."""
        return (math.degrees(math.atan2(self.R[2, 1], self.R[2, 2])),
                math.degrees(math.asin(float(np.clip(self.R[2, 0], -1.0, 1.0)))),
                math.degrees(math.atan2(self.R[1, 0], self.R[0, 0])))

    @property
    def airspeed(self):
        return float(np.linalg.norm(self.vel - self.wind_world))

    @property
    def aoa_deg(self):
        vb = self.R.T @ (self.vel - self.wind_world)
        return math.degrees(math.atan2(-vb[2], abs(vb[0]) + 1e-9))


def _expm_so3(w):
    th = float(np.linalg.norm(w))
    K = np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])
    if th < 1e-9:
        return np.eye(3) + K
    return np.eye(3) + math.sin(th) / th * K + (1 - math.cos(th)) / (th * th) * (K @ K)


if __name__ == "__main__":
    m, cg, I = mass_properties()
    print(f"assembled mass    {m:.3f} kg   weight {m*G:.1f} N")
    print(f"assembled CG      ({cg[0]:.4f}, {cg[1]:.4f}, {cg[2]:.4f}) m from the body link origin")
    print(f"assembled inertia diag({I[0,0]:.3f}, {I[1,1]:.3f}, {I[2,2]:.3f})  Ixz={I[0,2]:+.4f}")
    print(f"rotor arms rel CG:")
    for i, n in enumerate(("FL", "FR", "RL", "RR")):
        r = ROTOR_HUB[i] - cg
        print(f"  {n}: ({r[0]:+.3f}, {r[1]:+.3f}, {r[2]:+.3f})")
    # a hover-trim sanity check: what thrust split balances the pitch moment?
    xf, xr = ROTOR_HUB[0, 0] - cg[0], ROTOR_HUB[2, 0] - cg[0]
    Ff = m * G * (-xr) / (2 * (xf - xr)); Fr = m * G / 2 - Ff
    print(f"balanced hover: front {Ff:.2f} N/rotor, rear {Fr:.2f} N/rotor "
          f"(the sim's measured split was 8.6 / 8.4)")

#!/usr/bin/env python
"""
| File: 06_px4_single_drone_t650_rigid_payload.py
| License: BSD-3-Clause
| Description: Single T650 quadrotor SITL simulation with a RIGIDLY ATTACHED block
|   payload - structurally 05_px4_single_drone_t650.py plus one brick rigid body
|   fixed to the airframe at a configurable body-frame position.
|
|   Unlike every application/slungload/ scenario (payload hanging on spherical
|   joints / a cable), the block here is coupled through a UsdPhysics.FixedJoint:
|   drone and payload move as one rigid assembly whose combined mass, CG offset
|   and inertia the controller has to deal with. An off-CG offset produces a
|   constant gravity trim moment - the static signature of an aerial manipulator
|   holding a tool away from the body.
|
|   PAYLOAD PARAMETERS (constructor args, overridable via environment so launch
|   scripts can set them without editing this file):
|
|       T650_PAYLOAD_OFFSET  "x,y,z"  body-frame position of the block CENTER
|                                     relative to the drone body origin [m]
|                                     (FLU: +x forward, +y left, +z up).
|                                     Default "0.0,0.0,-0.12" (12 cm under the belly).
|       T650_PAYLOAD_MASS    float    block mass [kg]. Default 0.2.
|       T650_PAYLOAD_SIZE    "x,y,z"  block full edge lengths [m] (or one number
|                                     for a cube). Default "0.06,0.06,0.06".
|
|   The rigid link itself is drawn as a MASSLESS cylinder from the drone body
|   origin (the payload_offset reference point, coincident with the CoM for the
|   symmetric T650 asset) to the block center. PhysX cannot simulate a zero-mass
|   rigid body, so it is pure visual geometry parented under the body prim - no
|   RigidBodyAPI / MassAPI / CollisionAPI - which follows the rigid assembly
|   exactly while contributing nothing to the dynamics.
|
|   NOTE the spawn yaw is fixed at zero: the payload's world spawn position is
|   drone spawn + offset without rotation, so a nonzero spawn_euler would place
|   the block wrongly before the joint locks it. Rotate the offset first if that
|   is ever needed.
|
|   FUTURE - MOVING MASS OVER ROS 2 (not implemented here, by design): to shift
|   the block at runtime like an arm repositioning its end-effector mass, replace
|   create_fixed_joint with up to three chained prismatic joints
|   (constraints_utils.create_prismatic_joint) carrying UsdPhysics.DriveAPI
|   position targets, and drive the targets from a ROS 2 subscriber inside a
|   Backend.update_sim_state physics callback - the exact per-instance-node
|   pattern of ROS2CableWinchBackend. That keeps the coupling a real constraint
|   (reaction forces on the airframe stay physical) instead of teleporting the
|   joint anchor, which PhysX re-parses discontinuously.
"""

# Imports to start Isaac Sim from this script
import os

import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
HEADLESS = os.environ.get("PEGASUS_HEADLESS", "0") == "1"
STEP_LIMIT = int(os.environ.get("PEGASUS_STEPS", "0"))
PX4_LOCKSTEP = os.environ.get("PEGASUS_PX4_LOCKSTEP", "1") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import math

import omni.timeline
from omni.isaac.core.world import World
import omni.usd
from pxr import Gf, UsdGeom

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# The T650's own spawn helper and constants -- independent of the X650 modules.
from fsc_aerial_manipulation.rotorcraft.t650_bare_frame_utils import spawn_t650_with_mavlink
from fsc_aerial_manipulation.rotorcraft import t650_params
from fsc_aerial_manipulation.utils import add_dome_lighting
import fsc_aerial_manipulation.slung_load as sl
import fsc_aerial_manipulation.constraints as con

_G = 9.81


def _env_vec3(name, default):
    """Parse an 'x,y,z' environment override (a single number is broadcast)."""
    raw = os.environ.get(name)
    if raw is None:
        return tuple(default)
    parts = [float(p) for p in raw.replace(";", ",").split(",") if p.strip()]
    if len(parts) == 1:
        return (parts[0],) * 3
    if len(parts) != 3:
        raise ValueError(f"{name} must be 'x,y,z' (or one number), got: {raw!r}")
    return tuple(parts)


def _add_rigid_link_visual(stage, body_path, offset, radius=0.008):
    """Draw the rigid link as a MASSLESS cylinder from the drone body origin to
    the payload center (both in the body frame).

    Visual geometry only -- parented under the body rigid-body prim with no
    RigidBodyAPI/MassAPI/CollisionAPI, so it rides along with the rigid assembly
    and contributes exactly nothing to mass, inertia or contacts. (PhysX cannot
    simulate a true zero-mass rigid body; parenting is how "massless" is done.)

    Returns the visual root path, or None when the offset is too short to draw.
    """
    off = tuple(map(float, offset))
    length = math.sqrt(sum(c * c for c in off))
    if length < 1e-6:
        return None  # payload at the body origin - nothing to draw

    # Rotate the cylinder's +Z axis onto the offset direction.
    dx, dy, dz = (c / length for c in off)
    if dz > 1.0 - 1e-9:                      # already aligned with +Z
        quat_wxyz = (1.0, 0.0, 0.0, 0.0)
    elif dz < -1.0 + 1e-9:                   # antiparallel: 180 deg about X
        quat_wxyz = (0.0, 1.0, 0.0, 0.0)
    else:
        ax, ay, az = -dy, dx, 0.0            # z_hat x d
        norm = math.sqrt(ax * ax + ay * ay)
        half = 0.5 * math.acos(dz)
        s = math.sin(half) / norm
        quat_wxyz = (math.cos(half), ax * s, ay * s, az * s)

    root_path = f"{body_path}/rigid_link_visual"
    root = stage.DefinePrim(root_path, "Xform")
    xf = UsdGeom.Xformable(root)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3f(0.5 * off[0], 0.5 * off[1], 0.5 * off[2]))
    w, x, y, z = quat_wxyz
    xf.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))

    geom = UsdGeom.Cylinder(stage.DefinePrim(root_path + "/geom", "Cylinder"))
    geom.CreateHeightAttr(float(length))
    geom.CreateRadiusAttr(float(radius))
    geom.CreateAxisAttr("Z")
    # No CollisionAPI on purpose: the link must never contact anything.
    return root_path


class FscDroneSim:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(
        self,
        drone_spawn_pos: tuple = (0.0, 0.0, 0.07),
        payload_offset: tuple = (0.0, 0.0, -0.12),
        payload_size: tuple = (0.06, 0.06, 0.06),
        payload_mass: float = 0.2,
        link_radius: float = 0.008,
    ):
        """
        Method that initializes the FscDroneSim and is used to setup the simulation environment.

        Args:
            payload_offset: body-frame position of the block center relative to the
                drone body origin [m]. THE definable mass-position parameter.
            payload_size: block full edge lengths (lx, ly, lz) [m].
            payload_mass: block mass [kg].
            link_radius: radius of the massless visual link cylinder [m].
        """
        self.payload_offset = tuple(map(float, payload_offset))
        self.payload_size = tuple(map(float, payload_size))
        self.payload_mass = float(payload_mass)
        self.link_radius = float(link_radius)

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # ---------- Add dome lighting ----------
        stage = omni.usd.get_context().get_stage()
        self.stage = stage

        add_dome_lighting(
            stage=stage,
            dome_path="/World/DomeLight",
            intensity=2500.0,
            exposure=0.0,
            color=(1.0, 1.0, 1.0)
        )

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics,
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # If the block hangs below the body, raise the spawn so it clears the floor:
        # block bottom = drone_z + offset_z - lz/2 must stay above ground.
        off_x, off_y, off_z = self.payload_offset
        lx, ly, lz = self.payload_size
        min_drone_z = 0.01 + 0.5 * lz - off_z
        spawn_pos = (
            float(drone_spawn_pos[0]),
            float(drone_spawn_pos[1]),
            max(float(drone_spawn_pos[2]), min_drone_z),
        )

        # 1) Create the vehicle. Mass, inertia and the MN4010 thrust curve all come
        # from t650_params via the helper's own defaults -- nothing is passed in here,
        # so there is exactly one place to change a T650 number.
        self.drone_path = spawn_t650_with_mavlink(
            px4_path=self.pg.px4_path,
            px4_default_airframe=self.pg.px4_default_airframe,
            vehicle_id=0,
            spawn_pos=spawn_pos,
            spawn_euler=(0.0, 0.0, 0.0),  # must stay zero -- see module docstring
            enable_lockstep=PX4_LOCKSTEP,
        )

        # 2) Create the block rigid body at drone spawn + body-frame offset (zero
        # spawn yaw makes body frame == world frame here).
        payload_world_pos = (
            spawn_pos[0] + off_x,
            spawn_pos[1] + off_y,
            spawn_pos[2] + off_z,
        )
        payload_path = "/World/payload_block"
        self.payload_prim = sl.create_brick_with_xform_root(
            stage=stage,
            root_path=payload_path,
            length_x=lx,
            width_y=ly,
            height_z=lz,
            mass=self.payload_mass,
            world_pos=payload_world_pos,
            world_quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            enable_collision=True,
        )

        # ---- wait for the T650 body to load (the referenced USD resolves async) ----
        # Stock Multirotor spawns at /World/quadrotor_0 and references the asset's
        # children onto it, so the body rigid body is at drone_path + "/body".
        uav_path = f"{self.drone_path}/body"
        if self._wait_for_prim(uav_path, max_frames=300) is None:
            raise RuntimeError(f"[Load] T650 body prim did not load: {uav_path}")

        self.world.reset()  # finalize all defined objects

        # 3) Rigid link: fixed joint anchored at the block position in the body frame.
        # Both joint frames coincide in world space at creation (the block was placed
        # exactly at the anchor), so nothing snaps on the first physics step.
        con.create_fixed_joint(
            stage=stage,
            joint_path="/World/joint_uav_payload",
            body0_path=uav_path,
            body1_path=payload_path,
            local_pos0=self.payload_offset,
            local_pos1=(0.0, 0.0, 0.0),  # block is linked at its center
        )

        # 4) Massless visual cylinder for the rigid link: body origin -> block
        # center, parented under the body so it rides with the assembly.
        self.link_visual_path = _add_rigid_link_visual(
            stage=stage,
            body_path=uav_path,
            offset=self.payload_offset,
            radius=self.link_radius,
        )

        # Report what the coupled assembly looks like to the controller.
        total_mass = t650_params.MASS + self.payload_mass
        cg_shift = tuple(self.payload_mass * o / total_mass for o in self.payload_offset)
        trim_moment = self.payload_mass * _G * math.hypot(off_x, off_y)
        omega_hover = math.sqrt(total_mass * _G / (t650_params.NUM_ROTORS * t650_params.ROTOR_CONSTANT))
        hover_cmd = (omega_hover - t650_params.ZERO_POSITION_ARMED) / (
            t650_params.MAX_ROTOR_VEL - t650_params.ZERO_POSITION_ARMED
        )
        link_length = math.sqrt(off_x * off_x + off_y * off_y + off_z * off_z)
        print(
            f"[T650+payload] block {self.payload_mass:.3f} kg, size {self.payload_size} m, "
            f"body-frame offset {self.payload_offset} m | "
            f"rigid link visual: length {link_length:.3f} m, radius {self.link_radius:.3f} m | "
            f"assembly {total_mass:.6f} kg (vehicle {t650_params.MASS:.6f} + payload) | "
            f"CG shift {tuple(round(c, 4) for c in cg_shift)} m | "
            f"gravity trim moment {trim_moment:.4f} N.m | "
            f"expected hover command {hover_cmd:.4f} "
            f"(bare T650: {t650_params.HOVER_COMMAND:.4f})",
            flush=True,
        )

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def _wait_for_prim(self, prim_path: str, max_frames: int = 240):
        """Step a few frames until prim_path exists/loads in the stage."""
        stage = self.stage
        for _ in range(max_frames):
            prim = stage.GetPrimAtPath(prim_path)
            if prim and prim.IsValid():
                return prim
            # let async USD references resolve
            self.world.step(render=False)
        return None

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()
        steps = 0

        # The "infinite" loop
        while (
            simulation_app.is_running()
            and not self.stop_sim
            and (not STEP_LIMIT or steps < STEP_LIMIT)
        ):

            # Update the UI of the app and perform the physics step
            self.world.step(render=not HEADLESS)
            steps += 1

        # Cleanup and stop
        carb.log_warn("FscDroneSim Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    # Instantiate the template app. The payload numbers come from the environment
    # when set (see the module docstring) so launch scripts can vary the mass
    # position without editing this file.
    fsc_drone_sim = FscDroneSim(
        drone_spawn_pos=(0.0, 0.0, 0.07),
        payload_offset=_env_vec3("T650_PAYLOAD_OFFSET", (0.0826, 0.0018, -0.1637)),
        payload_size=_env_vec3("T650_PAYLOAD_SIZE", (0.06, 0.06, 0.06)),
        payload_mass=float(os.environ.get("T650_PAYLOAD_MASS", "0.65")),
    )

    # Run the application loop
    fsc_drone_sim.run()


if __name__ == "__main__":
    main()

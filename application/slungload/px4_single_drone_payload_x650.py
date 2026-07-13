#!/usr/bin/env python
"""Single X650 quadrotor with a slung-load payload — PX4 SITL simulation.

The vehicle is the FSC X650 asset (x650.usd), flown on the stock Pegasus
Multirotor/Vehicle: the asset already uses the naming those classes expect
(/body, /rotor0../rotor3, joint0..joint3). PX4/QGroundControl drives the motors;
a slung payload hangs from the drone body by a two-link cable of spherical joints.
"""
# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
# Enable the PhysX Debug Visualization extension (needed for _setup_physics_debug_viz
# to draw colliders / frames). Must happen after SimulationApp is created.
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.physx.ui")

import omni.timeline
from omni.isaac.core.world import World
import omni.usd

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
# FSC aerial manipulation lib — scene helpers only (payload, cable, joints, lighting).
from fsc_aerial_manipulation.utils import add_dome_lighting
# Bare X650 airframe spawn helper (stock Multirotor, MN4014+15x5" bench-test-calibrated
# thrust curve) - shared with application/px4_base/03_px4_single_drone_x650.py so the
# calibration only lives in one place.
from fsc_aerial_manipulation.rotorcraft.x650_bare_frame_utils import spawn_x650_with_mavlink
import fsc_aerial_manipulation.slung_load as sl
import fsc_aerial_manipulation.constraints as con

# ── PhysX debug visualization ───────────────────────────────────────────────
# Turn on the PhysX debugger to draw the coordinate frames (and/or collision
# shapes) in the viewport, selected by the SHOW_* flags below. Frames only render
# while the sim is PLAYING; PhysX wipes the params on each Play, so
# _setup_physics_debug_viz re-applies them on every resume.
SHOW_PHYSICS_DEBUG  = True    # master switch (False = don't touch the debugger)
SHOW_COLLIDERS      = False   # collision shapes (convex-decomposition hulls)
SHOW_WORLD_AXES     = False    # world origin frame (RGB = XYZ)
SHOW_BODY_AXES      = True     # each rigid body's link frame  <- the only one on
PHYSICS_FRAME_SCALE = 5.0      # body-axes length multiplier (0 -> frames invisible)


class FscDroneSim:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self,
                 drone_spawn_pos: tuple=(0.0, 0.0, 0.07),
                 drone_spawn_euler: tuple=(0.0, 0.0, 0.0),
                 cable_length: float=1.0,
                 cable_radius: float=0.01,
                 cable_mass: float=0.03,
                 payload_size: float=0.03,
                 payload_mass: float=0.3,
                 uav_hook_local=(0.0, 0.0, 0.0)
        ):
        """
        Method that initializes the FscDroneSim and is used to setup the simulation environment.
        """
        self.uav_hook_local = tuple(map(float, uav_hook_local))
        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

         # ---------- Add dome lighting ----------
        stage = omni.usd.get_context().get_stage()
        self.stage=stage

        add_dome_lighting(
            stage=stage,
            dome_path = "/World/DomeLight",
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

        # 1) Create the vehicle (X650 on the STOCK Multirotor). px4_primary: PX4 SITL /
        #    QGroundControl drives the motors, exactly like the Iris scenario.
        self.drone_path = spawn_x650_with_mavlink(
            px4_path=self.pg.px4_path,
            px4_default_airframe=self.pg.px4_default_airframe,
            vehicle_id=0,
            spawn_pos=drone_spawn_pos,
            spawn_euler=drone_spawn_euler,
        )

        # 2) Create payload
        payload_pos, cable_pose, cable_quat_wxyz = sl.setup_single_drone_payload(
            L=cable_length,
            drone_pose=drone_spawn_pos
        )
        payload_path = "/World/payload_link"
        self.payload_prim = sl.create_brick_with_xform_root(
            stage=stage,
            root_path=payload_path,
            length_x=payload_size,
            width_y=payload_size,
            height_z=payload_size,
            mass=payload_mass,
            world_pos=payload_pos,
            world_quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            enable_collision=True,
        )

        # 3) Create cable using cylinders
        mid = sl.get_mid_point(payload_pos, drone_spawn_pos)
        cable_path = f"/World/cable_link"
        self.cable_prim = sl.create_cylinder_with_xform_root(
            stage=stage,
            root_path=f"/World/cable_link",
            length=cable_length,
            radius=cable_radius,
            mass=cable_mass,
            world_pos=mid,
            world_quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            enable_collision=False,
            axis="X", # cylinder axis in Xfrom prim frame
        )

        # ---- wait for the X650 body to load (the referenced USD resolves async) ----
        # Stock Multirotor spawns at /World/quadrotor_0 and references /x650's
        # children onto it, so the body rigid body is at drone_path + "/body".
        uav_path = f"{self.drone_path}/body"
        if self._wait_for_prim(uav_path, max_frames=300) is None:
            raise RuntimeError(f"[Load] X650 body prim did not load: {uav_path}")

        self.world.reset() # finalize all defined objects

        # 4) define joints
        cable_end_to_uav = (-0.5 * cable_length, 0.0, 0.0) # cylinder in X direction
        cable_end_to_payload = (0.5 * cable_length, 0.0, 0.0)

        # The X650 body rigid-body path is deterministic (drone_path + "/body"), so
        # use it directly instead of searching for the first rigid body under the
        # drone (which could resolve to a rotor).
        print(f"uav_path: {uav_path}")
        print(f"self.drone_path: {self.drone_path}")
        con.create_spherical_joint(
                stage=stage,
                joint_path=f"/World/joint_uav_cable",
                body0_path=uav_path,
                body1_path=cable_path,
                local_pos0=self.uav_hook_local,
                local_pos1=cable_end_to_uav,
        )

        con.create_spherical_joint(
                stage=stage,
                joint_path=f"/World/joint_cable_payload",
                body0_path=cable_path,
                body1_path=payload_path,
                local_pos0=cable_end_to_payload,
                local_pos1=(0.0, 0.0, 0.0), # cable connected to the center of the payload
        )

        # 5) Turn on the PhysX debugger (body-frame axes; see the SHOW_* flags above).
        self._setup_physics_debug_viz()

        # # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def _setup_physics_debug_viz(self):
        """Enable the PhysX debug visualization — the world/body/mass coordinate
        frames and/or collision shapes selected by the SHOW_* flags.

        Two subtleties (from omni.physxui's own PhysxDebugView):
          1. Drawing needs BOTH interfaces — the physx visualization interface
             (computes the lines) AND the physxui interface's
             enable_debug_visualization (actually renders them in the viewport).
          2. PhysX resets the frame parameters on every sim Play, so a one-shot
             call before Play is wiped — we re-apply on each RESUMED event.
        Colliders are toggled via the persistent setting the Physics Debug window
        uses (/persistent/physics/visualizationDisplayColliders = VisualizerMode).
        """
        if not SHOW_PHYSICS_DEBUG:
            return
        try:
            from omni.physx import get_physx_visualization_interface, get_physx_interface
            from omni.physxui import get_physxui_interface
            from omni.physx.bindings._physx import SimulationEvent, VisualizerMode
        except ImportError:
            carb.log_warn("[PhysX] omni.physx.ui not loaded; cannot draw debug viz. "
                          "Keep the omni.physx.ui extension enabled at startup.")
            return

        import carb as _carb
        settings = _carb.settings.get_settings()

        def _apply():
            viz = get_physx_visualization_interface()
            get_physxui_interface().enable_debug_visualization(True)   # (1) render
            viz.enable_visualization(True)                             # (1) compute
            viz.set_visualization_scale(PHYSICS_FRAME_SCALE)
            viz.set_visualization_parameter("WorldAxes",    SHOW_WORLD_AXES)
            viz.set_visualization_parameter("BodyAxes",     SHOW_BODY_AXES)
            viz.set_visualization_parameter("BodyMassAxes", False)   # mass axes off
            # Collision shapes: VisualizerMode ALL(2) = draw all, NONE(0) = off.
            settings.set_int(
                "/persistent/physics/visualizationDisplayColliders",
                int(VisualizerMode.ALL) if SHOW_COLLIDERS else int(VisualizerMode.NONE),
            )

        _apply()  # take effect immediately if the sim is already playing

        # (2) re-apply on every Play/resume — PhysX wipes the params on sim start.
        def _on_sim_event(event):
            if event.type == int(SimulationEvent.RESUMED):
                _apply()

        events = get_physx_interface().get_simulation_event_stream_v2()
        # keep the subscription alive for the process lifetime (else it's GC'd).
        self._physx_viz_sub = events.create_subscription_to_pop(_on_sim_event)

        print(f"[PhysX] debugger on: colliders={SHOW_COLLIDERS} world={SHOW_WORLD_AXES} "
              f"body={SHOW_BODY_AXES} scale={PHYSICS_FRAME_SCALE} "
              f"(re-applied on every Play)", flush=True)

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

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)

        # Cleanup and stop
        carb.log_warn("FscDroneSim Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():
    # Instantiate the template app
    fsc_app = FscDroneSim(
        drone_spawn_pos=(0.0, 0.0, 0.07),
        drone_spawn_euler=(0.0, 0.0, 0.0),
        cable_length=1.0,
        cable_radius=0.01,
        cable_mass=0.03,
        payload_size=0.08,
        payload_mass=0.3
    )

    # Run the application loop
    fsc_app.run()

if __name__ == "__main__":
    main()

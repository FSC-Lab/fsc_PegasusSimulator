#!/usr/bin/env python
"""
| File: 06_px4_single_drone_t650_gate_splat.py
| Description: Single T650 quadrotor SITL inside the RECONSTRUCTED DRONE-GATE SPLAT
|   scene (3D Gaussian splat of the A2RL x DCL practice gate, captured by phone and
|   reconstructed with spirulae-splat, rendered natively by Isaac Sim's NuRec).
|
|   Same vehicle, calibration and PX4 wiring as 05_px4_single_drone_t650.py. What
|   differs:
|
|     1. ENVIRONMENT: loads gate_metric.usda instead of the stock Curved Gridroom.
|        The scene is metric (scale fixed from the gate's measured 1.50 m inner
|        opening width), gravity-aligned, floor at z=0, gate opening centred at
|        (x=0, y=0, z=1.75). The gate plane is the YZ plane: flying "through the
|        gate" is motion along world X. Cameras/capture side is +X.
|     2. PHYSICS: the splat is VISION-ONLY - it has no collision geometry. An
|        invisible PhysX ground plane at z=0 gives the drone something to sit on.
|        Nothing else collides (the drone can fly through the gate frame; add a
|        mesh collision proxy later if crashes should be physical).
|     3. CAMERA: a forward-looking MonocularCamera is attached to the drone body
|        and shown in its own "Drone Camera" viewport next to the main view.
|
|   The scene asset resolves from the repo itself, via
|   fsc_aerial_manipulation.worlds.gate_splat. Like every other FSC USD it is not in
|   git; see that package's README.md for how to obtain it.
|
|   Environment overrides:
|     GATE_SPLAT_USD    path to the metric splat USD (default: the in-repo asset)
|     PEGASUS_CAM_POS   camera position in body FLU, "x,y,z" metres (default 0.25,0,0.10)
|     PEGASUS_CAM_PITCH camera down-pitch in degrees (default 0 = level)
|     PEGASUS_CAM_ROS2  "1" publishes camera on the ROS 2 backend (default off)
|
|   plus the usual PEGASUS_HEADLESS / PEGASUS_STEPS / PEGASUS_PX4_LOCKSTEP.
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
CAM_POS = [float(x) for x in os.environ.get("PEGASUS_CAM_POS", "0.25,0.0,0.10").split(",")]
CAM_PITCH = float(os.environ.get("PEGASUS_CAM_PITCH", "0.0"))
CAM_ROS2 = os.environ.get("PEGASUS_CAM_ROS2", "0") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import numpy as np
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.objects import GroundPlane
import omni.usd

# Import the Pegasus API for simulating drones
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera

# The T650's own spawn helper and constants -- independent of the X650 modules.
from fsc_aerial_manipulation.rotorcraft.t650_bare_frame_utils import spawn_t650_with_mavlink
from fsc_aerial_manipulation.rotorcraft import t650_params
from fsc_aerial_manipulation.utils import add_dome_lighting

# The reconstructed gate scene: in-repo asset path + its metric geometry.
from fsc_aerial_manipulation.worlds import gate_splat


class FscGateSplatSim:
    """T650 SITL inside the metric gate-splat scene, with an onboard camera view."""

    def __init__(self):

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Resolves env override -> in-repo asset, and raises with download/regenerate
        # instructions if the (ungitted) payload is absent.
        scene_usd = gate_splat.gate_splat_usd_path()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # ---------- The reconstructed gate scene ----------
        # NuRec splat: photoreal visuals, ZERO collision. Floor at z=0, gate plane
        # = YZ plane, opening centred at OPENING_CENTRE.
        # NOTE load_environment is asyncio.ensure_future -- the /World/layout prim
        # does NOT exist until the app steps, so nothing may query it before reset().
        self.pg.load_environment(scene_usd)
        print(
            f"[gate] scene {scene_usd}\n"
            f"[gate] opening centre {np.round(gate_splat.OPENING_CENTRE, 3).tolist()} m, "
            f"{gate_splat.OPENING_WIDTH:.3f} x {gate_splat.OPENING_HEIGHT:.3f} m, "
            f"sill {gate_splat.SILL_HEIGHT:.3f} m | fly-through axis = X",
            flush=True,
        )

        # The splat bakes the hangar's real lighting into its radiance; the dome
        # light is for the DRONE asset, which is a mesh and needs actual lights.
        stage = omni.usd.get_context().get_stage()
        add_dome_lighting(
            stage=stage,
            dome_path="/World/DomeLight",
            intensity=2500.0,
            exposure=0.0,
            color=(1.0, 1.0, 1.0),
        )

        # Invisible physics floor: the splat cannot collide, so give PhysX the
        # tarp plane the scene was aligned to (z=0).
        GroundPlane(prim_path="/World/groundPlane", size=25.0, z_position=0.0, visible=False)

        # ---------- Onboard camera ----------
        # Body-frame FLU position from PEGASUS_CAM_POS. Orientation: the Pegasus
        # example default [0, 0, 180] yields a level forward-facing upright image
        # (examples/8_camera_vehicle.py); down-pitch composes as [0, -pitch, 180].
        camera = MonocularCamera(
            "camera",
            config={
                "position": np.array(CAM_POS),
                "orientation": np.array([0.0, -CAM_PITCH, 180.0]),
                "resolution": (1280, 720),
                "frequency": 30,
                # 90 deg horizontal FOV pinhole (fx = W/2), principal point centred
                "intrinsics": np.array([[640.0, 0.0, 640.0], [0.0, 640.0, 360.0], [0.0, 0.0, 1.0]]),
                "distortion_coefficients": [0.0] * 8,
                "diagonal_fov": 98.0,
            },
        )

        # ---------- Vehicle ----------
        # Spawn on the capture (+X) side, yawed 180 so body-forward (and the camera)
        # faces the gate. Both come from the scene module, not hardcoded here.
        self.drone_path = spawn_t650_with_mavlink(
            px4_path=self.pg.px4_path,
            px4_default_airframe=self.pg.px4_default_airframe,
            vehicle_id=0,
            spawn_pos=gate_splat.DEFAULT_SPAWN_POS,
            spawn_euler=gate_splat.DEFAULT_SPAWN_EULER,
            enable_lockstep=PX4_LOCKSTEP,
            graphical_sensors=[camera],
            pub_graphical_sensors=CAM_ROS2,
        )
        print(
            f"[T650] total mass {t650_params.MASS:.6f} kg | "
            f"expected hover command {t650_params.HOVER_COMMAND:.4f} | "
            f"spawn {gate_splat.DEFAULT_SPAWN_POS} facing -X toward the gate | "
            f"camera at body FLU {CAM_POS}, pitch {CAM_PITCH} deg down",
            flush=True,
        )

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # ---------- Drone-camera viewport ----------
        # A second viewport window showing what the onboard camera sees.
        if not HEADLESS:
            try:
                from omni.kit.viewport.utility import create_viewport_window

                cam_prim = self.drone_path + "/body/camera"
                vp = create_viewport_window("Drone Camera", width=640, height=360)
                vp.viewport_api.set_active_camera(cam_prim)
                print(f"[camera] 'Drone Camera' viewport bound to {cam_prim}", flush=True)
            except Exception as exc:  # viewport API differences should not kill the sim
                carb.log_warn(f"Could not create drone-camera viewport: {exc}")

        self.stop_sim = False

    def run(self):
        """Application main loop - identical to the stock T650 scenario."""

        self.timeline.play()
        steps = 0

        while (
            simulation_app.is_running()
            and not self.stop_sim
            and (not STEP_LIMIT or steps < STEP_LIMIT)
        ):
            self.world.step(render=not HEADLESS)
            steps += 1

        carb.log_warn("FscGateSplatSim Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    sim = FscGateSplatSim()
    sim.run()


if __name__ == "__main__":
    main()

#!/usr/bin/env python
"""
VTOL hover scene  (Step 2 of the VTOL bring-up):  PHYSICS FORCES ON.

What changed vs Step 1 (01_basic_vtol.py):
  - gravity ON          (config.disable_gravity = False)
  - flight forces ON    (config.enable_forces  = True)  -> VTOL._apply_propulsion
                                                           + VTOL._apply_aerodynamics
  - a closed-loop HOVER CONTROLLER keeps the airframe upright and at altitude.

>>> WHY A CONTROLLER IS HERE <<<
A tiltrotor in hover is *unstable* open-loop (exactly like a quadrotor): constant
throttle just tips over.  To VISUALLY CONFIRM that thrust / moments / tilt vector
correctly, something has to stabilise attitude + altitude.  In the finished
system that "something" is PX4 (Step 4, full HIL).  `HoverControllerBackend`
below is a deliberately small, throwaway STAND-IN for PX4 so you can validate the
*plant* (the VTOL vehicle) on its own first.

It is generic: it reads the vehicle's RotorConfig list and builds a hover control
allocation from the geometry, so it works for any hover-capable VTOL config, not
just the default tri-tiltrotor.

Success criteria to look for:
  - airframe holds ~1.5 m altitude and stays level (does not flip)        -> thrust + roll/pitch moments correct
  - with FORWARD_PULSE = True, a gentle nose-down-free forward tilt makes
    it accelerate forward while staying upright, then return to hover     -> thrust vectoring correct
"""

import numpy as np
from scipy.spatial.transform import Rotation

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.vtol import VTOLConfig
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from fsc_aerial_manipulation.rotorcraft import spawn_vtol

from hover_controller import HoverControllerBackend   # shared stabiliser (same dir)

# Set True to add a brief forward-tilt pulse (demonstrates thrust vectoring).
# Keep False first to confirm a clean stationary hover.
FORWARD_PULSE = False


class VTOLHoverApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        simulation_app.update()

        # --- Step 2 config: gravity + forces ON --------------------------
        config = VTOLConfig()
        config.disable_gravity = False
        config.enable_forces = True
        config.backends = [HoverControllerBackend(1.5, forward_pulse=FORWARD_PULSE)]

        # Spawn AT the target altitude so there is no big initial transient.
        self.vtol_path = spawn_vtol(
            vehicle_id=0,
            spawn_pos=(0.0, 0.0, 1.5),
            config=config,
        )

        self.world.reset()
        self.pg.set_viewport_camera([5.0, -5.0, 3.0], [0.0, 0.0, 1.4])

    def run(self):
        self.timeline.play()
        carb.log_warn("VTOLHoverApp: airframe should hold ~1.5 m and stay level.")
        while simulation_app.is_running():
            self.world.step(render=True)

        carb.log_warn("VTOLHoverApp closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    app = VTOLHoverApp()
    app.run()


if __name__ == "__main__":
    main()

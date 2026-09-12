#!/usr/bin/env python
"""
Basic standalone VTOL *articulation* scene  (Step 1 of the VTOL bring-up).

Purpose: spawn the generic articulated VTOL (default = tri-tiltrotor) and watch
its JOINTS move under scripted commands — the two front rotors sweep from hover
(0) to cruise (90°) and back, the props spin, and the elevons wiggle.

No flight forces and no PX4 yet: gravity is disabled (config.enable_forces=False
+ config.disable_gravity=True) so the airframe floats in place and the joint
motion is easy to see.  Steps 2/3 add propulsion, aerodynamics and PX4 HIL.

Interaction points (see the MAP OF INTERACTIONS in vtol.py):
  - This file provides a Backend whose `input_reference()` returns a
    {channel_name: value} dict.  The channel names MUST match the *_channel
    fields in the VTOL's RotorConfig/SurfaceConfig (here: throttle_front_L,
    throttle_front_R, throttle_tail, tilt, elevon_L, elevon_R).
"""

import math

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World

from pegasus.simulator.logic.backends.backend import Backend
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.vtol import VTOLConfig
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from fsc_aerial_manipulation.rotorcraft import spawn_vtol


class ScriptedVTOLBackend(Backend):
    """Drives the VTOL with time-varying commands so the joints visibly move.

    This is the simplest possible 'control source'.  Swap it for the PX4 MAVLink
    backend later — the vehicle only cares that `input_reference()` returns the
    channel dict it expects."""

    # How long a full hover -> cruise -> hover tilt sweep takes [s].
    TILT_PERIOD = 8.0
    # How long a full elevon wiggle takes [s].
    ELEVON_PERIOD = 4.0

    def __init__(self):
        super().__init__(config=None)
        self._t = 0.0

    def input_reference(self):
        # tilt: 0 -> 1 -> 0 (hover -> cruise -> hover), smooth via (1-cos)/2.
        tilt = 0.5 * (1.0 - math.cos(2.0 * math.pi * self._t / self.TILT_PERIOD))
        # elevons deflect oppositely (a "roll"-like wiggle) so motion is obvious.
        elevon = 0.5 * math.sin(2.0 * math.pi * self._t / self.ELEVON_PERIOD)
        return {
            # gentle throttle just so the props visibly spin (no force in Step 1).
            "throttle_front_L": 0.3,
            "throttle_front_R": 0.3,
            "throttle_tail": 0.3,
            # shared tilt channel for both front nacelles.
            "tilt": tilt,
            "elevon_L": elevon,
            "elevon_R": -elevon,
        }

    def update(self, dt: float):
        self._t += dt

    # --- remaining Backend interface: nothing to do for a scripted source ---
    def update_sensor(self, sensor_type: str, data):
        pass

    def update_graphical_sensor(self, sensor_type: str, data):
        pass

    def update_state(self, state):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def reset(self):
        self._t = 0.0


class BasicVTOLApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        simulation_app.update()

        # --- Build the airframe config -------------------------------------
        #   Default VTOLConfig() == tri-tiltrotor.  For Step 1 we float it:
        config = VTOLConfig()
        config.disable_gravity = True      # float so joint motion is easy to see
        config.enable_forces = False       # no thrust / aero yet
        config.backends = [ScriptedVTOLBackend()]

        self.vtol_path = spawn_vtol(
            vehicle_id=0,
            spawn_pos=(0.0, 0.0, 1.5),
            config=config,
        )

        self.world.reset()
        self.pg.set_viewport_camera([4.0, -4.0, 2.5], [0.0, 0.0, 1.4])

    def run(self):
        self.timeline.play()
        carb.log_warn("BasicVTOLApp: watch the front rotors tilt hover<->cruise.")
        while simulation_app.is_running():
            self.world.step(render=True)

        carb.log_warn("BasicVTOLApp closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    app = BasicVTOLApp()
    app.run()


if __name__ == "__main__":
    main()

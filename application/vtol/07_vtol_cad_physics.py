#!/usr/bin/env python
"""
CAD VTOL as a PHYSICS MODEL: the imported airframe with real forces on — rotor
spin-up dynamics + thrust(RPM^2) + yaw torque + wing/body aerodynamics + gravity —
held up by the throwaway hover controller (stand-in for PX4). This is the CAD
counterpart of 02_vtol_hover.py; use it to validate the plant (thrust vectoring,
aero, mass/inertia) before flying it under PX4 HIL.

Run like the other scenes (clean shell + Isaac python):
    env -u PYTHONHOME -u PYTHONPATH -u CONDA_PREFIX -u VIRTUAL_ENV \\
      PYTHONEXE=~/workspaces/isaacsim/kit/python/bin/python3 \\
      ~/workspaces/isaacsim/python.sh application/vtol/07_vtol_cad_physics.py

Success: the airframe holds ~1.5 m and stays roughly level (doesn't flip). Set
FORWARD_PULSE=True to tilt the front rotors forward briefly and see it accelerate
then return. Tune gains in hover_controller.py; tune the rotor model / max_thrust
in quad_tiltrotor_config().
"""

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.vtol import VTOL
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from fsc_aerial_manipulation.rotorcraft import fsc_cad_vtol_config

from hover_controller import HoverControllerBackend   # same dir (added to sys.path)


FORWARD_PULSE = False        # True -> brief forward tilt to show thrust vectoring
TARGET_ALTITUDE = 1.5


class CADPhysicsApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        simulation_app.update()

        # CAD airframe, PHYSICS ON: gravity + thrust/aero + rotor dynamics.
        config = fsc_cad_vtol_config()
        config.enable_forces = True
        config.disable_gravity = False
        config.rotor_dynamics = True
        config.rotor_tau_spin = 0.12
        config.backends = [HoverControllerBackend(TARGET_ALTITUDE, forward_pulse=FORWARD_PULSE)]

        # Spawn at the target altitude so there is no big initial transient.
        self.vehicle = VTOL(vehicle_id=0, init_pos=(0.0, 0.0, TARGET_ALTITUDE), config=config)
        self.omega_max = config.rotor_omega_max

        self.world.reset()
        self.pg.set_viewport_camera([5.0, -5.0, 3.0], [0.0, 0.0, 1.4])

    def _log(self):
        st = self.vehicle.state
        alt = float(st.position[2])
        thrust = sum(r.max_thrust * (self.vehicle._omega[r.name] / self.omega_max) ** 2
                     for r in self.vehicle._rotors)
        need = self.vehicle.total_mass * 9.81
        carb.log_warn(f"[cad-phys] alt={alt:5.2f} m  total_thrust={thrust:5.1f} N  (need ~{need:.1f} N)")

    def run(self):
        self.timeline.play()
        carb.log_warn("CAD physics: airframe should hold ~1.5 m and stay level (not flip).")
        step = 0
        while simulation_app.is_running():
            self.world.step(render=True)
            step += 1
            if step % 30 == 0:
                self._log()
        self.timeline.stop()
        simulation_app.close()


def main():
    CADPhysicsApp().run()


if __name__ == "__main__":
    main()

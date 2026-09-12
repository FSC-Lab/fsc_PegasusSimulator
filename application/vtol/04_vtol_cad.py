#!/usr/bin/env python
"""
CAD VTOL viewer (Step 1 for an imported airframe): spawn the quad tiltrotor
(2 front tilting + 2 rear static rotors) built from IMPORTED CAD MESHES and watch
the joints move — gravity off, no forces — so you can verify the mesh import,
scale, orientation, and articulation before adding physics.

Workflow:
  1. Convert your CAD components to USD (each part separately):
       env -u PYTHONHOME -u PYTHONPATH -u CONDA_PREFIX -u VIRTUAL_ENV \\
         PYTHONEXE=~/workspaces/isaacsim/kit/python/bin/python3 \\
         ~/workspaces/isaacsim/python.sh application/vtol/convert_meshes_to_usd.py \\
         --input /path/to/cad_dir --output /path/to/usd_dir
  2. Point the paths below at the resulting .usd files (leave any as None to use a
     placeholder box for that link).
  3. Run this scene the same way you ran 01_basic_vtol.py.

Tuning: if a part is the wrong size, set MESH_SCALE (e.g. 0.001 for mm). If a part
is offset/rotated from its hub, adjust that RotorConfig's mesh_offset / mesh_rpy_deg
(or fix the part origin in your mesh tool). Rotor hub positions live in
quad_tiltrotor_config() — measure them off your CAD.
"""

import math
import os

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World

from pegasus.simulator.logic.backends.backend import Backend
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.vtol import quad_tiltrotor_config
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from fsc_aerial_manipulation.rotorcraft import spawn_vtol


# --- POINT THESE AT YOUR CONVERTED USD FILES (None -> placeholder box) -------
USD_DIR = os.path.expanduser("~/vtol_cad_usd")          # where convert_meshes_to_usd.py wrote them
def _u(name):
    p = os.path.join(USD_DIR, name)
    return p if os.path.isfile(p) else None

BODY_MESH        = _u("body.usd")
FRONT_LEFT_MESH  = _u("front_left_rotor.usd")
FRONT_RIGHT_MESH = _u("front_right_rotor.usd")
REAR_LEFT_MESH   = _u("rear_left_rotor.usd")
REAR_RIGHT_MESH  = _u("rear_right_rotor.usd")
MESH_SCALE       = (1.0, 1.0, 1.0)                      # (0.001,)*3 if CAD is in mm
# -----------------------------------------------------------------------------


class ScriptedCADBackend(Backend):
    """Sweeps the front tilt servos hover<->cruise and spins all props, so you can
    confirm the imported parts tilt/spin about the right axes."""

    TILT_PERIOD = 8.0

    def __init__(self):
        super().__init__(config=None)
        self._t = 0.0

    def input_reference(self):
        tilt = 0.5 * (1.0 - math.cos(2.0 * math.pi * self._t / self.TILT_PERIOD))
        return {
            "motor_front_L": 0.3, "motor_front_R": 0.3,
            "motor_rear_L": 0.3, "motor_rear_R": 0.3,
            "tilt_left": tilt, "tilt_right": tilt,
        }

    def update(self, dt: float):
        self._t += dt

    def update_sensor(self, sensor_type, data): pass
    def update_graphical_sensor(self, sensor_type, data): pass
    def update_state(self, state): pass
    def start(self): pass
    def stop(self): pass
    def reset(self): self._t = 0.0


class CADVTOLApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        simulation_app.update()

        config = quad_tiltrotor_config(
            body_mesh=BODY_MESH,
            front_left_mesh=FRONT_LEFT_MESH,
            front_right_mesh=FRONT_RIGHT_MESH,
            rear_left_mesh=REAR_LEFT_MESH,
            rear_right_mesh=REAR_RIGHT_MESH,
            mesh_scale=MESH_SCALE,
        )
        config.disable_gravity = True       # float so the joint motion is easy to see
        config.enable_forces = False        # no thrust/aero yet
        config.backends = [ScriptedCADBackend()]

        meshes = [m for m in (BODY_MESH, FRONT_LEFT_MESH, FRONT_RIGHT_MESH,
                              REAR_LEFT_MESH, REAR_RIGHT_MESH) if m]
        carb.log_warn(f"CAD VTOL: {len(meshes)}/5 parts loaded from {USD_DIR} "
                      f"(missing parts fall back to boxes)")

        self.vtol_path = spawn_vtol(vehicle_id=0, spawn_pos=(0.0, 0.0, 1.5), config=config)

        self.world.reset()
        self.pg.set_viewport_camera([4.0, -4.0, 2.5], [0.0, 0.0, 1.4])

    def run(self):
        self.timeline.play()
        carb.log_warn("CADVTOLApp: front rotors should tilt hover<->cruise; check scale/orientation.")
        while simulation_app.is_running():
            self.world.step(render=True)
        self.timeline.stop()
        simulation_app.close()


def main():
    CADVTOLApp().run()


if __name__ == "__main__":
    main()

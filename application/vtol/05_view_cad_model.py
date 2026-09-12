#!/usr/bin/env python
"""
View the imported CAD VTOL as a STATIC USD (no articulation) to check scale and
orientation before splitting it into articulated parts. Run like the other scenes
(clean shell + Isaac python), e.g.:

    env -u PYTHONHOME -u PYTHONPATH -u CONDA_PREFIX -u VIRTUAL_ENV \\
      PYTHONEXE=~/workspaces/isaacsim/kit/python/bin/python3 \\
      ~/workspaces/isaacsim/python.sh application/vtol/05_view_cad_model.py

The STL measured 1878 x 242 x 978 mm, so SCALE = 0.001 (mm -> m). The part's own
origin is at a corner; adjust RPY_DEG / POS until it sits nose-forward, wings level
in the FLU frame (x-fwd, y-left, z-up). Note the numbers that look right — they are
the body_mesh_scale / body_mesh_rpy_deg / body_mesh_offset you'll feed the config.
"""

import os

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.timeline
import omni.usd
from omni.isaac.core.world import World
from pxr import UsdGeom, Gf

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS


USD_PATH = os.path.expanduser("~/vtol_cad_usd/UAV_VTOL_Fixedwing.usd")
SCALE = 0.001                    # STL is in millimetres
# CAD is authored with the fuselage along +Z (nose points up at rpy=0). This maps
# CAD Z->world X (nose fwd), CAD X->world Y (span), CAD Y->world Z (up). If the
# nose ends up pointing BACKWARD, use (90,0,-90); if it's UPSIDE-DOWN, use (-90,0,90).
RPY_DEG = (90.0, 0.0, 90.0)
# Applied AFTER rotate+scale (world frame): recenters the corner-origin model to
# roughly (0,0,0.8). This value IS the body_mesh_offset for the config.
POS = (-0.49, -0.94, 0.78)


class ViewApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        simulation_app.update()

        if not os.path.isfile(USD_PATH):
            carb.log_error(f"USD not found: {USD_PATH} — run convert_meshes_to_usd.py first")

        stage = omni.usd.get_context().get_stage()
        # Parent Xform carries OUR placement (fresh xformOps); the CAD USD is
        # referenced into a CHILD so its own xformOps don't clash with ours
        # (referencing a USD into a prim pulls that USD's xformOpOrder in).
        root = UsdGeom.Xform.Define(stage, "/World/vtol_cad")
        root.AddTranslateOp().Set(Gf.Vec3d(*POS))
        root.AddRotateXYZOp().Set(Gf.Vec3f(*RPY_DEG))
        root.AddScaleOp().Set(Gf.Vec3f(SCALE, SCALE, SCALE))
        stage.DefinePrim("/World/vtol_cad/model", "Xform").GetReferences().AddReference(USD_PATH)
        carb.log_warn(f"Loaded {USD_PATH}  scale={SCALE}  rpy={RPY_DEG}  pos={POS}")

        self.world.reset()
        self.pg.set_viewport_camera([3.0, -3.0, 2.0], [0.0, 0.0, 1.0])

    def run(self):
        self.timeline.play()
        carb.log_warn("View CAD model: check scale/orientation; tune SCALE/RPY_DEG/POS at the top.")
        while simulation_app.is_running():
            self.world.step(render=True)
        self.timeline.stop()
        simulation_app.close()


def main():
    ViewApp().run()


if __name__ == "__main__":
    main()

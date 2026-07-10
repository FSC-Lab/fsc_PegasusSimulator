#!/usr/bin/env python
"""
| File: 22_x650_with_manip_minimal.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: Minimal PX4 SITL environment for custom x650 with integrated manipulator USD.
"""

import os
import sys

from isaacsim import SimulationApp

# Isaac Sim app must be created immediately after import.
simulation_app = SimulationApp({"headless": False})

import omni.timeline
import omni.usd
from omni.isaac.core.world import World
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from fsc_aerial_manipulation.utils import add_dome_lighting

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
UTILS_DIR = os.path.join(SCRIPT_DIR, "utils")
if UTILS_DIR not in sys.path:
    sys.path.insert(0, UTILS_DIR)

from x650_rotorcraft_utils import spawn_rotorcraft_with_mavlink


def wait_for_prim(world, stage, prim_path: str, max_steps: int = 240):
    """Wait for a prim to become valid while stepping simulation updates."""
    for _ in range(max_steps):
        prim = stage.GetPrimAtPath(prim_path)
        if prim and prim.IsValid():
            return prim
        world.step(render=False)
    return None


class FscX650MinimalSim:

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

       
        stage = omni.usd.get_context().get_stage()
        add_dome_lighting(
            stage=stage,
            dome_path="/World/DomeLight",
            intensity=2500.0,
            exposure=0.0,
            color=(1.0, 1.0, 1.0),
        )

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        self.drone_path = spawn_rotorcraft_with_mavlink(
            px4_path=self.pg.px4_path,
            px4_default_airframe=self.pg.px4_default_airframe,
            vehicle_id=0,
            spawn_pos=(0.0, 0.0, 0.30),
            spawn_euler=(0.0, 0.0, 0.0),
            usd_file=r"/home/fsc-jupiter/Source/fsc_PegasusSimulator/application/robotic_arm/assets/isaac_sim_models/x650_frame_2025_0825.usd",
            articulation_path="/x650_frame_2025_0825",
            body_path="/x650_frame_2025_0825/body",
            rotor_paths=[
                "/x650_frame_2025_0825/rotor0",
                "/x650_frame_2025_0825/rotor1",
                "/x650_frame_2025_0825/rotor2",
                "/x650_frame_2025_0825/rotor3",
            ],
            rotor_joint_names=["joint0", "joint1", "joint2", "joint3"],
            # enable_propeller_visual=False,
        )

        self.world.reset()

        stage = omni.usd.get_context().get_stage()

        self.stop_sim = False

    def run(self):
        self.timeline.pause()
        # self.timeline.play()

        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)

        self.timeline.stop()
        simulation_app.close()


def main():
    app = FscX650MinimalSim()
    app.run()


if __name__ == "__main__":
    main()

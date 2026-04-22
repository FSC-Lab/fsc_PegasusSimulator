#!/usr/bin/env python
"""
| File: 01_ideal_single_drone.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: Single ideal quadrotor simulation.  The vehicle is a box rigid body
|   with user-specified mass and inertia.  Commands are sent over ROS 2 as a
|   geometry_msgs/WrenchStamped on  drone0/control/wrench
|     wrench.force.z        – lift force  [N]  in body FLU frame
|     wrench.torque.{x,y,z} – body torques [N·m]
|
|   State is published on:
|     drone0/state/pose   (geometry_msgs/PoseStamped)   – ENU position + attitude
|     drone0/state/twist  (geometry_msgs/TwistStamped)  – body-FLU velocity
"""

# Must be instantiated before any Isaac Sim / Pegasus imports
import carb
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script starts here
# -----------------------------------
import omni.timeline
import omni.usd
from omni.isaac.core.world import World

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from pegasus.simulator.logic.vehicles.multirotors.ideal_quadrotor import (
    IdealQuadrotor,
    IdealQuadrotorConfig,
)
from pegasus.simulator.logic.backends.body_wrench_ros2_backend import BodyWrenchROS2Backend

from fsc_aerial_manipulation.utils import add_dome_lighting


class IdealDroneSim:

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        # Lighting
        stage = omni.usd.get_context().get_stage()
        add_dome_lighting(
            stage=stage,
            dome_path="/World/DomeLight",
            intensity=2500.0,
            exposure=0.0,
            color=(1.0, 1.0, 1.0),
        )

        # Pegasus / World setup
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # Vehicle configuration
        cfg = IdealQuadrotorConfig()
        cfg.backends = [BodyWrenchROS2Backend(vehicle_id=0)]

        # Spawn on the ground: z = half box height (0.11 / 2 = 0.055 m)
        IdealQuadrotor(
            vehicle_id=0,
            init_pos=(0.0, 0.0, 0.055),
            init_orientation=(0.0, 0.0, 0.0, 1.0),
            config=cfg,
        )

        self.world.reset()
        self.stop_sim = False

    def run(self):
        self.timeline.play()

        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)

        carb.log_warn("IdealDroneSim is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = IdealDroneSim()
    pg_app.run()


if __name__ == "__main__":
    main()

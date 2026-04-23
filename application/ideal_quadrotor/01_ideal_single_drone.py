#!/usr/bin/env python
"""
| File: 01_ideal_single_drone.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: Single ideal quadrotor simulation interfacing with fsc_geometric_controller.
|
|   Commands received from the geometric controller on:
|     drone0/control/output/f  (std_msgs/Float64)            – thrust [N] along body z
|     drone0/control/output/M  (geometry_msgs/Vector3Stamped)– body moments [N·m]
|
|   State published to the geometric controller on:
|     drone0/state/pose           (geometry_msgs/PoseStamped)   – ENU position + attitude
|     drone0/state/twist          (geometry_msgs/TwistStamped)  – body angular velocity
|     drone0/state/twist_inertial (geometry_msgs/TwistStamped)  – inertial linear velocity
|     drone0/state/accel          (geometry_msgs/AccelStamped)  – inertial linear acceleration
|     drone0/state/jerk           (geometry_msgs/Vector3Stamped)– inertial linear jerk
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
from pegasus.simulator.logic.backends.geometric_controller_ros2_backend import GeometricControllerROS2Backend

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
        cfg.backends = [GeometricControllerROS2Backend(vehicle_id=0)]

        # Spawn on the ground (z = half box height = 0.055 m)
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

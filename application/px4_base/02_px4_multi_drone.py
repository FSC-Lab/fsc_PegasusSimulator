#!/usr/bin/env python
"""
| File: 02_px4_multi_drone.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: Multi-drone quadrotor SITL simulation environment.
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
import omni.timeline
from omni.isaac.core.world import World
import omni.usd
import omni.kit.commands as kit_cmd
from pxr import UsdLux, Sdf, Gf, UsdGeom

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Import FSC aerial manipulation lib
from fsc_aerial_manipulation.rotorcraft import spawn_rotorcraft_with_mavlink
from fsc_aerial_manipulation.utils import add_dome_lighting

# Auxiliary scipy and numpy modules
import os.path

class FscDroneSim:
    def __init__(self, num_of_drones, init_pos, base_tcp_port):
        
        self.num_of_drones = num_of_drones
        self.init_pose = init_pos
        self.base_tcp_port = base_tcp_port

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        stage = omni.usd.get_context().get_stage()
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

        # Spawn 3 drones
        self.drone_path = []
        for i in range(self.num_of_drones):
            self.drone_path.append(
                spawn_rotorcraft_with_mavlink(
                    px4_path=self.pg.px4_path,
                    px4_default_airframe=self.pg.px4_default_airframe,
                    vehicle_id=i,
                    spawn_pos=init_pos[i],
                    spawn_euler=(0.0, 0.0, 0.0),
                    connection_ip="127.0.0.1",
                    connection_baseport=self.base_tcp_port,
                    vehicle_type="Iris"
                )
            )

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

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
    # set the initial condition of drones
    NUM_DRONES = 3          # <<< exactly 3 drones
    BASE_TCP_PORT = 4560    # PX4 SITL will connect to 4560, 4561, 4562

    # x, y, z for each drone
    initial_pose = [
        (1.0, 0.0, 0.07),   # drone 0
        (0.0, 1.0, 0.07),   # drone 1
        (0.0, -1.0, 0.07),  # drone 2
    ]

    # Instantiate the template app
    fsc_app = FscDroneSim(NUM_DRONES, initial_pose, BASE_TCP_PORT)

    # Run the application loop
    fsc_app.run()

if __name__ == "__main__":
    main()
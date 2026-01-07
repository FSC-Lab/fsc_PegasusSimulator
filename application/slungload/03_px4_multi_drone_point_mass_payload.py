#!/usr/bin/env python
"""
| File: 03_px4_multi_drone_point_mass_payload.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: 3 quadrotor with a single point-mass payload
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

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
# Import FSC aerial manipulation lib
from fsc_aerial_manipulation.rotorcraft import spawn_rotorcraft_with_mavlink
from fsc_aerial_manipulation.utils import add_dome_lighting
import fsc_aerial_manipulation.slung_load as sl
import fsc_aerial_manipulation.constraints as con

class FscDroneSim:

    def __init__(self,
                 base_tcp_port,
                 cable_length=1.0,
                 z_payload=0.70,
                 z_uavs=0.90,
                 center_xy=(0.0, 0.0),
                 payload_mass=1.0,
                 payload_size: float=0.03,
                 cable_mass=0.05,
                 cable_radius=0.01,
                 uav_hook_local=(0.0, 0.0, 0.0)):
     
        self.num_of_drones = 3
        self.base_tcp_port = int(base_tcp_port)

        self.cable_length = float(cable_length)
        self.z_payload = float(z_payload)
        self.z_uavs = float(z_uavs)
        self.center_xy = (float(center_xy[0]), float(center_xy[1]))

        self.payload_mass = float(payload_mass)
        self.cable_mass = float(cable_mass)
        self.cable_radius = float(cable_radius)
        self.uav_hook_local = tuple(map(float, uav_hook_local))

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

         # ---------- Add dome lighting ----------
        stage = omni.usd.get_context().get_stage()
        self.stage=stage

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

        Z0 = 0.7  # spawn the payload and drones at the same height
        
        self.payload_init_pos, self.uav_init_positions = sl.setup_same_height_payload_and_triangle_uavs(
            L=self.cable_length,
            z=Z0,
            center_xy=self.center_xy,
        )

        # 1) spawn drones
        self.drone_body_path = []
        for i in range(self.num_of_drones):
            drone_body_path = spawn_rotorcraft_with_mavlink(
                    px4_path=self.pg.px4_path,
                    px4_default_airframe=self.pg.px4_default_airframe,
                    vehicle_id=i,
                    spawn_pos=tuple(self.uav_init_positions[i]),
                    spawn_euler=(0.0, 0.0, 0.0),
                    connection_ip="127.0.0.1",
                    connection_baseport=self.base_tcp_port,
                    vehicle_type="Iris"
            )

            # determine the body_prim
            body_prim = con.find_rigidbody_prim(stage, drone_body_path)
            if body_prim is None:
                raise RuntimeError(f"No rigid body found under {drone_body_path}")
        
            self.drone_body_path.append(body_prim.GetPath().pathString)

        # 2) spwan payload
        payload_path = "/World/payload_link"
        self.payload_prim = sl.create_brick_with_xform_root(
            stage=stage,
            root_path=payload_path,
            length_x=payload_size,
            width_y=payload_size,
            height_z=payload_size,         
            mass=payload_mass,
            world_pos=self.payload_init_pos,
            world_quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            enable_collision=True,
        )

        self.world.reset() # finalize all defined objects

        # 4) create joint


        # Auxiliar variable for the timeline callback example
        self.stop_sim = False


    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        # self.timeline.play()
            # Start paused
        self.timeline.pause()

        print("[SIM] Simulation is paused after spawn.", flush=True)
        print("[SIM] Inspect payload, cables, joints. Press Play when ready.", flush=True)
        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            # Update the UI of the app and perform the physics step
            if self.timeline.is_playing():
                self.world.step(render=True)
            else:
                # Render without advancing physics
                simulation_app.update()
            
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():
    # set the initial condition of drones
    BASE_TCP_PORT = 4560    # PX4 SITL will connect to 4560, 4561, 4562
    
    # Choose geometry so dz < cable_length
    CABLE_LENGTH = 1.0
    Z_PAYLOAD = 0.20
    Z_UAVS = 0.90


    # Instantiate the template app
    fsc_app = FscDroneSim(NUM_DRONES, BASE_TCP_PORT)

    # Run the application loop
    fsc_app.run()

if __name__ == "__main__":
    main()
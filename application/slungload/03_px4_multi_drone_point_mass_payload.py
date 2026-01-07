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
import fsc_aerial_manipulation.utils as ul
class FscDroneSim:

    def __init__(self,
                 base_tcp_port,
                 cable_length: float=1.0,
                 z_uavs: float=0.90,
                 center_xy: tuple=(0.0, 0.0),
                 payload_mass: float=1.0,
                 payload_size: float=0.03,
                 cable_mass: float=0.05,
                 cable_radius: float=0.01,
                 uav_hook_local: tuple=(0.0, 0.0, 0.0)):
     
        self.num_of_drones = 3
        self.base_tcp_port = int(base_tcp_port)

        self.cable_length = float(cable_length)
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

   
        self.payload_init_pos, self.uav_init_positions = sl.setup_same_height_payload_and_triangle_uavs(
            L=self.cable_length,
            z=self.z_uavs,
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

            # determine the drone body_prim (joints can only be attached to body prim)
            body_prim = con.find_rigidbody_prim(stage, drone_body_path)
            if body_prim is None:
                raise RuntimeError(f"No rigid body found under {drone_body_path}")
        
            self.drone_body_path.append(body_prim.GetPath().pathString)

        # 2) spawn payload
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

        # 3) spawn cables
        self.cable_path = []
        cable_z_rotation = [0.0, 120.0, -120.0]
        for i in range(self.num_of_drones):
            mid = sl.get_mid_point(self.payload_init_pos, self.uav_init_positions[i])
            cable_root = f"/World/cable_link_{i}"
            self.cable_path.append(cable_root)
            sl.create_cylinder_with_xform_root(
                stage=stage,
                root_path=cable_root,
                length=cable_length,
                radius=cable_radius,
                mass=cable_mass,
                world_pos=mid,
                world_quat_wxyz=ul.quat_from_z_deg(cable_z_rotation[i]),
                enable_collision=False,
                axis="X", # cylinder axis in Xfrom prim frame
            )

        # ---- NEW: wait for UAV prims to load ----
        for drone_path in self.drone_body_path:
            if self._wait_for_prim(drone_path, max_frames=300) is None:
                raise RuntimeError(f"[Load] Drone prim did not load: {drone_path}")
        
        self.world.reset() # finalize all defined objects

        # 4) create joints
        cable_end_to_uav = (0.5 * cable_length, 0.0, 0.0) # cylinder in X direction
        cable_end_to_payload = (-0.5 * cable_length, 0.0, 0.0)
        for i in range(self.num_of_drones):
            print(f"drone_body_path: {self.drone_body_path[i]}")
            print(f"cable_path: {self.cable_path[i]}")
            con.create_spherical_joint(
                    stage=stage,
                    joint_path=f"/World/joint_uav_cable_{i}",
                    body0_path=self.drone_body_path[i],
                    body1_path=self.cable_path[i],
                    local_pos0=self.uav_hook_local,
                    local_pos1=cable_end_to_uav,
            )

            con.create_spherical_joint(
                    stage=stage,
                    joint_path=f"/World/joint_cable_payload_{i}",
                    body0_path=self.cable_path[i],
                    body1_path=payload_path,
                    local_pos0=cable_end_to_payload,
                    local_pos1=(0.0, 0.0, 0.0), # cable connected to the center of the payload
            )

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def _wait_for_prim(self, prim_path: str, max_frames: int = 240):
        """Step a few frames until prim_path exists/loads in the stage."""
        stage = self.stage
        for _ in range(max_frames):
            prim = stage.GetPrimAtPath(prim_path)
            if prim and prim.IsValid():
                return prim
            # let async USD references resolve
            self.world.step(render=False)
        return None

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

    # Instantiate the template app
    fsc_app = FscDroneSim(
                 base_tcp_port = BASE_TCP_PORT,
                 cable_length=CABLE_LENGTH,
                 z_uavs=0.07,
                 center_xy=(0.0, 0.0),
                 payload_mass=1.0,
                 payload_size=0.1,
                 cable_mass=0.05,
                 cable_radius=0.01,
                 uav_hook_local=(0.0, 0.0, 0.0)
    )

    # Run the application loop
    fsc_app.run()

if __name__ == "__main__":
    main()
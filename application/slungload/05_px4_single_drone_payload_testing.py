#!/usr/bin/env python
"""
| File: 01_px4_single_drone_payload.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: Single quadrotor with a slung load SITL simulation environment.
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

# For testing
import omni.usd
from omni.usd import get_stage_next_free_path
from omni.isaac.dynamic_control import _dynamic_control

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
# Import FSC aerial manipulation lib
from fsc_aerial_manipulation.rotorcraft import spawn_rotorcraft_with_mavlink
from fsc_aerial_manipulation.utils import add_dome_lighting
from fsc_aerial_manipulation.utils import ROS2RigidBodyBackend
import fsc_aerial_manipulation.slung_load as sl
import fsc_aerial_manipulation.constraints as con
class FscDroneSim:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self,
                 drone_spawn_pos: tuple=(0.0, 0.0, 0.07),
                 drone_spawn_euler: tuple=(0.0, 0.0, 0.0),
                 cable_length: float=1.0,
                 cable_radius: float=0.01,
                 cable_mass: float=0.03,
                 payload_size: float=0.03,
                 payload_mass: float=0.3,
                 uav_hook_local=(0.0, 0.0, 0.0)
        ):
        """
        Method that initializes the FscDroneSim and is used to setup the simulation environment.
        """
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

        # 1) Create the vehicle
        self.drone_path = spawn_rotorcraft_with_mavlink(
            px4_path=self.pg.px4_path,
            px4_default_airframe=self.pg.px4_default_airframe,
            vehicle_id=0,
            spawn_pos=drone_spawn_pos,
            spawn_euler=drone_spawn_euler,
        )

        # 2) Create payload
        payload_pos, cable_pose, cable_quat_wxyz = sl.setup_single_drone_payload(
            L=cable_length,
            drone_pose=drone_spawn_pos
        )
        payload_path = "/World/payload_link"
        self.payload_prim = sl.create_brick_with_xform_root(
            stage=stage,
            root_path=payload_path,
            length_x=payload_size,
            width_y=payload_size,
            height_z=payload_size,         
            mass=payload_mass,
            world_pos=payload_pos,
            world_quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            enable_collision=True,
        )

        # 3) Create cable using cylinders
        mid = sl.get_mid_point(payload_pos, drone_spawn_pos)
        cable_path = f"/World/cable_link"
        self.cable_prim = sl.create_cylinder_with_xform_root(
            stage=stage,
            root_path=f"/World/cable_link",
            length=cable_length,
            radius=cable_radius,
            mass=cable_mass,
            world_pos=mid,
            world_quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            enable_collision=False,
            axis="X", # cylinder axis in Xfrom prim frame
        )

        config_ros2_rigid_body = {
            "topic_prefix": "payload",
            "pub_state": True,
            "sub_force": True,
        }
        payload = ROS2RigidBodyBackend(world=self.world, payload_path=payload_path, config=config_ros2_rigid_body)

        # ---- NEW: wait for UAV prims to load ----
        if self._wait_for_prim(self.drone_path, max_frames=300) is None:
            raise RuntimeError(f"[Load] Drone prim did not load: {self.drone_path}")

        self.world.reset() # finalize all defined objects

        # 4) define joints
        cable_end_to_uav = (-0.5 * cable_length, 0.0, 0.0) # cylinder in X direction
        cable_end_to_payload = (0.5 * cable_length, 0.0, 0.0)

        # determine the rigid-body path of the drone
        body_prim = con.find_rigidbody_prim(stage, self.drone_path)
        if body_prim is None:
            raise RuntimeError(f"No rigid body found under {self.drone_path}")
        
        uav_path = body_prim.GetPath().pathString

        print(f"uav_path: {uav_path}")
        print(f"self.drone_path: {self.drone_path}")
        con.create_spherical_joint(
                stage=stage,
                joint_path=f"/World/joint_uav_cable",
                body0_path=uav_path,
                body1_path=cable_path,
                local_pos0=self.uav_hook_local,
                local_pos1=cable_end_to_uav,
        )

        con.create_spherical_joint(
                stage=stage,
                joint_path=f"/World/joint_cable_payload",
                body0_path=cable_path,
                body1_path=payload_path,
                local_pos0=cable_end_to_payload,
                local_pos1=(0.0, 0.0, 0.0), # cable connected to the center of the payload
        )
        # # Auxiliar variable for the timeline callback example
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
    # Instantiate the template app
    fsc_app = FscDroneSim(
        drone_spawn_pos=(0.0, 0.0, 0.07),
        drone_spawn_euler=(0.0, 0.0, 0.0),
        cable_length=1.0,
        cable_radius=0.01,
        cable_mass=0.03,
        payload_size=0.08,
        payload_mass=0.3
    )

    # Run the application loop
    fsc_app.run()

if __name__ == "__main__":
    main()
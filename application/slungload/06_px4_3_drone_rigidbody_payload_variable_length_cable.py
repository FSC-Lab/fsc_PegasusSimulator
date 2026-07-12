#!/usr/bin/env python
"""
| File: 06_px4_3_drone_rigidbody_payload_variable_length_cable.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: 3 quadrotors, each connected via its own variable-length winch cable (2 rigid
|   rods + a prismatic joint, same mechanism as
|   02_px4_single_drone_payload_variable_length_cable.py) to a single shared rigid-body
|   payload. Each winch exposes its own ROS2CableWinchBackend (cable_winch_0/, cable_winch_1/,
|   cable_winch_2/) so each drone's cable can be driven independently (e.g. bridged to a
|   real/emulated AK40-10 winch driver per drone).
"""

# Imports to start Isaac Sim from this script
import os
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
_headless = os.environ.get("PEGASUS_HEADLESS", "0").lower() in ("1", "true", "yes")
simulation_app = SimulationApp({"headless": _headless})

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
from fsc_aerial_manipulation.utils import ROS2CableWinchBackend
from fsc_aerial_manipulation.utils import ROS2RigidBodyBackend
import fsc_aerial_manipulation.utils as ul
import fsc_aerial_manipulation.slung_load as sl
import fsc_aerial_manipulation.constraints as con


class FscDroneSim:
    """
    3 quadrotors, each with its own variable-length winch cable, sharing a single rigid-body
    payload.
    """

    def __init__(self,
                 base_tcp_port,
                 cable_length: float = 1.0,
                 z_uavs: float = 0.9,
                 center_xy: tuple = (0.0, 0.0),
                 payload_mass: float = 1.0,
                 payload_size: float = 0.1,
                 rod_a_mass: float = 0.02,
                 rod_b_mass: float = 0.02,
                 cable_radius: float = 0.01,
                 uav_hook_local: tuple = (0.0, 0.0, 0.0),
                 max_cable_extension: float = 2.0,
        ):
        """
        Method that initializes the FscDroneSim and is used to setup the simulation environment.
        """
        self.num_of_drones = 3
        self.base_tcp_port = int(base_tcp_port)
        self.cable_length = float(cable_length)
        self.uav_hook_local = tuple(map(float, uav_hook_local))
        self.max_cable_extension = float(max_cable_extension)

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # ---------- Add dome lighting ----------
        stage = omni.usd.get_context().get_stage()
        self.stage = stage

        add_dome_lighting(
            stage=stage,
            dome_path="/World/DomeLight",
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

        # Same triangle layout as 03_px4_multi_drone_point_mass_payload.py: payload at the
        # center, 3 drones at the same height, each exactly cable_length away from the payload,
        # at 0/120/-120 degrees around it.
        self.payload_init_pos, self.uav_init_positions = sl.setup_same_height_payload_and_triangle_uavs(
            L=self.cable_length,
            z=z_uavs,
            center_xy=center_xy,
        )
        # Matches setup_same_height_payload_and_triangle_uavs's own triangle formula (uav_i =
        # center + L*(cos(uav_angle_deg[i]), sin(uav_angle_deg[i]), 0)) - needed below to orient
        # each winch correctly.
        uav_angle_deg = [0.0, 120.0, -120.0]

        # 1) spawn drones
        self.drone_body_path = []
        for i in range(self.num_of_drones):
            drone_path = spawn_rotorcraft_with_mavlink(
                px4_path=self.pg.px4_path,
                px4_default_airframe=self.pg.px4_default_airframe,
                vehicle_id=i,
                spawn_pos=tuple(self.uav_init_positions[i]),
                spawn_euler=(0.0, 0.0, 0.0),
                connection_ip="127.0.0.1",
                connection_baseport=self.base_tcp_port,
                vehicle_type="Iris",
            )

            # determine the drone body_prim (joints can only be attached to body prim)
            body_prim = con.find_rigidbody_prim(stage, drone_path)
            if body_prim is None:
                raise RuntimeError(f"No rigid body found under {drone_path}")

            self.drone_body_path.append(body_prim.GetPath().pathString)

        # 2) spawn the shared rigid-body payload
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

        # Publish the payload's own pose (not covered by any per-drone backend) - same rationale
        # as 02_px4_single_drone_payload_variable_length_cable.py.
        config_ros2_rigid_body = {
            "topic_prefix": "payload",
            "pub_state": True,
            "sub_force": False,
        }
        self.payload_backend = ROS2RigidBodyBackend(world=self.world, payload_path=payload_path, config=config_ros2_rigid_body)

        # 3) spawn one variable-length winch (rod_a/rod_b pair) per drone, connecting it to the
        #    shared payload - same mechanism as 02_px4_single_drone_payload_variable_length_cable.py,
        #    replicated 3x, each winch oriented from its own drone toward the payload center.
        self.rod_a_length = 0.5 * self.cable_length
        self.rod_b_length = 0.5 * self.cable_length
        self.rod_a_path = []
        self.rod_b_path = []
        for i in range(self.num_of_drones):
            winch_root_pos = sl.get_mid_point(self.payload_init_pos, self.uav_init_positions[i])
            # create_prismatic_rod_pair puts rod_a on the -local-X side (drone side) and rod_b
            # on the +local-X side (payload side) - so local +X (after this quat) must point
            # from the winch root TOWARD the payload, i.e. radially INWARD: the opposite
            # direction from the drone's own outward placement angle, hence +180 deg here versus
            # the fixed-length cables in 03 (whose local +X instead points toward the UAV).
            winch_quat_wxyz = ul.quat_from_z_deg(uav_angle_deg[i] + 180.0)
            winch_root_path = f"/World/cable_link_{i}"
            rod_a_path, rod_b_path = sl.create_prismatic_rod_pair(
                stage=stage,
                root_path=winch_root_path,
                rod_a_length=self.rod_a_length,
                rod_b_length=self.rod_b_length,
                radius=cable_radius,
                mass_a=rod_a_mass,
                mass_b=rod_b_mass,
                world_pos=winch_root_pos,
                world_quat_wxyz=winch_quat_wxyz,
                axis="X",
                enable_collision=False,
            )
            self.rod_a_path.append(rod_a_path)
            self.rod_b_path.append(rod_b_path)

        # ---- wait for UAV prims to load ----
        for drone_path in self.drone_body_path:
            if self._wait_for_prim(drone_path, max_frames=300) is None:
                raise RuntimeError(f"[Load] Drone prim did not load: {drone_path}")

        self.world.reset()  # finalize all defined objects

        # 4) create joints: drone <-spherical-> rod_a <-prismatic-> rod_b <-spherical-> payload,
        #    and a ROS2CableWinchBackend per winch, one per drone.
        rod_a_end_to_uav = (-0.5 * self.rod_a_length, 0.0, 0.0)   # rod_a's own drone-facing end
        rod_a_end_to_rod_b = (0.5 * self.rod_a_length, 0.0, 0.0)  # rod_a's own payload-facing end
        rod_b_end_to_rod_a = (-0.5 * self.rod_b_length, 0.0, 0.0) # rod_b's own drone-facing end
        rod_b_end_to_payload = (0.5 * self.rod_b_length, 0.0, 0.0)  # rod_b's own payload-facing end

        self.winch_backends = []
        for i in range(self.num_of_drones):
            con.create_spherical_joint(
                stage=stage,
                joint_path=f"/World/joint_uav_rod_a_{i}",
                body0_path=self.drone_body_path[i],
                body1_path=self.rod_a_path[i],
                local_pos0=self.uav_hook_local,
                local_pos1=rod_a_end_to_uav,
            )

            con.create_prismatic_joint(
                stage=stage,
                joint_path=f"/World/joint_rod_a_rod_b_{i}",
                body0_path=self.rod_a_path[i],
                body1_path=self.rod_b_path[i],
                local_pos0=rod_a_end_to_rod_b,
                local_pos1=rod_b_end_to_rod_a,
                axis="X",
                # Cable can't retract past flush, but is free to extend up to max_cable_extension
                # - same physically-correct one-sided limit as the single-drone scenario.
                lower_limit=0.0,
                upper_limit=self.max_cable_extension,
            )

            con.create_spherical_joint(
                stage=stage,
                joint_path=f"/World/joint_rod_b_payload_{i}",
                body0_path=self.rod_b_path[i],
                body1_path=payload_path,
                local_pos0=rod_b_end_to_payload,
                local_pos1=(0.0, 0.0, 0.0),  # cable connected to the center of the payload
            )

            # ROS2 interface exposing this winch's extension/velocity/force
            config_cable_winch = {
                "topic_prefix": f"cable_winch_{i}/",
                "pub_state": True,
                "sub_force": True,
            }
            winch_backend = ROS2CableWinchBackend(
                world=self.world,
                rod_a_path=self.rod_a_path[i],
                rod_b_path=self.rod_b_path[i],
                joint_axis="X",
                winch_id=i,
                config=config_cable_winch,
            )
            self.winch_backends.append(winch_backend)

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
        self.timeline.play()

        # No window to show when headless - skip the render pass entirely rather than paying
        # for it anyway (see fsc_PegasusSimulator/CLAUDE.md's real-time-factor investigation -
        # this is now the permanent default across all scenarios, not just the one it was found in).
        render_flag = not _headless

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            # Update the UI of the app and perform the physics step
            if self.timeline.is_playing():
                self.world.step(render=render_flag)
            else:
                # Render without advancing physics
                simulation_app.update()

        # Cleanup and stop
        carb.log_warn("FscDroneSim Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    # set the initial condition of drones
    BASE_TCP_PORT = 4560    # PX4 SITL will connect to 4560, 4561, 4562

    # Choose geometry so dz < cable_length
    CABLE_LENGTH = 1.0

    # Instantiate the template app
    fsc_app = FscDroneSim(
        base_tcp_port=BASE_TCP_PORT,
        cable_length=CABLE_LENGTH,
        z_uavs=0.07,
        center_xy=(0.0, 0.0),
        payload_mass=1.0,
        payload_size=0.1,
        rod_a_mass=0.02,
        rod_b_mass=0.02,
        cable_radius=0.01,
        uav_hook_local=(0.0, 0.0, 0.0),
        max_cable_extension=2.0,
    )

    # Run the application loop
    fsc_app.run()


if __name__ == "__main__":
    main()

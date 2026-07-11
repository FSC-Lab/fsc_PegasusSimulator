#!/usr/bin/env python
"""
| File: 02_px4_single_drone_payload_variable_length_cable.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: Single quadrotor with a variable-length slung-load cable SITL simulation
|   environment. The cable is modeled as two rigid rods (rod_a: proximal/drone-side,
|   rod_b: distal/payload-side) connected by a prismatic joint, whose constraint force is
|   the cable tension and whose position/velocity are the cable extension/extension rate.
|   A ROS2CableWinchBackend exposes this joint's state and accepts a commanded tension force,
|   for bridging to a real/emulated AK40-10 cable-actuator ROS2 driver.
"""

# Imports to start Isaac Sim from this script
import os
import time
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
# PEGASUS_HEADLESS=1 runs with no GUI/rendering (e.g. to A/B real-time performance against a
# rendered run, or for CI) - defaults to a normal windowed run, unchanged from before.
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
import fsc_aerial_manipulation.slung_load as sl
import fsc_aerial_manipulation.constraints as con


class FscDroneSim:
    """
    Single quadrotor + variable-length winch cable + payload SITL simulation environment.
    """

    def __init__(self,
                 drone_spawn_pos: tuple=(0.0, 0.0, 0.07),
                 drone_spawn_euler: tuple=(0.0, 0.0, 0.0),
                 cable_length: float=1.0,
                 cable_radius: float=0.01,
                 rod_a_mass: float=0.02,
                 rod_b_mass: float=0.02,
                 payload_size: float=0.08,
                 # rod_b_mass + payload_mass = 0.565 kg to match cable_torque_ctrl_node's deployed
                 # "mass" parameter (AK40-10-ROS2-Bridge/config/cable_torque_ctrl_params.yaml) -
                 # its gravity feedforward/L1-adaptive nominal values are computed from that param
                 # at startup and can't be corrected after the fact, so the sim's actual load must
                 # match it instead (see fsc_PegasusSimulator/CLAUDE.md's WIP section).
                 payload_mass: float=0.545,
                 uav_hook_local=(0.0, 0.0, 0.0),
                 max_cable_extension: float=2.0,
        ):
        """
        Method that initializes the FscDroneSim and is used to setup the simulation environment.
        """
        self.uav_hook_local = tuple(map(float, uav_hook_local))
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

        # 1) Create the vehicle
        self.drone_path = spawn_rotorcraft_with_mavlink(
            px4_path=self.pg.px4_path,
            px4_default_airframe=self.pg.px4_default_airframe,
            vehicle_id=0,
            spawn_pos=drone_spawn_pos,
            spawn_euler=drone_spawn_euler,
        )

        # 2) Create payload and winch (rod_a/rod_b) geometry from the initial cable length
        payload_pos, winch_root_pos, winch_quat_wxyz = sl.setup_variable_length_cable_geometry(
            L0=cable_length,
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

        # Publish the payload's own pose (not covered by the drone's TF broadcast, and not
        # something ROS2CableWinchBackend does - it only exposes the winch joint's extension/
        # force, not the payload rigid body's world pose) so it's visible for external monitoring
        # (e.g. scripts/view_drone_3d.sh's RViz view - add a Pose display on payload/state/pose).
        config_ros2_rigid_body = {
            "topic_prefix": "payload",
            "pub_state": True,
            "sub_force": False,
        }
        self.payload_backend = ROS2RigidBodyBackend(world=self.world, payload_path=payload_path, config=config_ros2_rigid_body)

        # 3) Create the winch: two rods (rod_a: drone-side, rod_b: payload-side), coaxial along X,
        #    flush end-to-end and together spanning cable_length, centered on the drone/payload midpoint.
        self.rod_a_length = 0.5 * cable_length
        self.rod_b_length = 0.5 * cable_length
        winch_root_path = "/World/cable_link"
        self.rod_a_path, self.rod_b_path = sl.create_prismatic_rod_pair(
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

        # ---- wait for UAV prims to load ----
        if self._wait_for_prim(self.drone_path, max_frames=300) is None:
            raise RuntimeError(f"[Load] Drone prim did not load: {self.drone_path}")

        self.world.reset()  # finalize all defined objects

        # 4) define joints: drone <-spherical-> rod_a <-prismatic-> rod_b <-spherical-> payload
        rod_a_end_to_uav = (-0.5 * self.rod_a_length, 0.0, 0.0)   # rod_a's own drone-facing end
        rod_a_end_to_rod_b = (0.5 * self.rod_a_length, 0.0, 0.0)  # rod_a's own payload-facing end
        rod_b_end_to_rod_a = (-0.5 * self.rod_b_length, 0.0, 0.0) # rod_b's own drone-facing end
        rod_b_end_to_payload = (0.5 * self.rod_b_length, 0.0, 0.0)  # rod_b's own payload-facing end

        # determine the rigid-body path of the drone
        body_prim = con.find_rigidbody_prim(stage, self.drone_path)
        if body_prim is None:
            raise RuntimeError(f"No rigid body found under {self.drone_path}")

        uav_path = body_prim.GetPath().pathString

        print(f"uav_path: {uav_path}")
        print(f"self.drone_path: {self.drone_path}")

        con.create_spherical_joint(
            stage=stage,
            joint_path="/World/joint_uav_rod_a",
            body0_path=uav_path,
            body1_path=self.rod_a_path,
            local_pos0=self.uav_hook_local,
            local_pos1=rod_a_end_to_uav,
        )

        con.create_prismatic_joint(
            stage=stage,
            joint_path="/World/joint_rod_a_rod_b",
            body0_path=self.rod_a_path,
            body1_path=self.rod_b_path,
            local_pos0=rod_a_end_to_rod_b,
            local_pos1=rod_b_end_to_rod_a,
            axis="X",
            # Cable can't retract past flush (rod_b sliding back past rod_a's far end), but is
            # free to extend up to max_cable_extension. (A symmetric ±max_cable_extension range
            # was tried as a diagnostic for the force/extension bug below, but that turned out to
            # be a red herring - see CLAUDE.md's WIP section - so this is back to the physically
            # correct one-sided limit.)
            lower_limit=0.0,
            upper_limit=max_cable_extension,
        )

        con.create_spherical_joint(
            stage=stage,
            joint_path="/World/joint_rod_b_payload",
            body0_path=self.rod_b_path,
            body1_path=payload_path,
            local_pos0=rod_b_end_to_payload,
            local_pos1=(0.0, 0.0, 0.0),  # cable connected to the center of the payload
        )

        # 5) ROS2 interface exposing the winch's extension/velocity/force
        config_cable_winch = {
            "topic_prefix": "cable_winch_0/",
            "pub_state": True,
            "sub_force": True,
        }
        self.winch_backend = ROS2CableWinchBackend(
            world=self.world,
            rod_a_path=self.rod_a_path,
            rod_b_path=self.rod_b_path,
            joint_axis="X",
            winch_id=0,
            config=config_cable_winch,
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
        self.timeline.play()

        # PEGASUS_PROFILE=1: print wall-clock step-time stats every ~250 steps, to profile
        # where per-step time actually goes (physics + PX4 lockstep + rendering + our own
        # ROS2CableWinchBackend callback, timed separately - see cable_winch_backend_utils.py)
        # rather than guessing from the published-topic rate alone.
        profile = os.environ.get("PEGASUS_PROFILE", "0").lower() in ("1", "true", "yes")
        step_count = 0
        step_time_sum = 0.0
        step_time_max = 0.0

        # No window to show when headless - skip the render pass entirely rather than paying
        # for it anyway. Profiling (2026-07-11) showed world.step(render=True) costing ~22ms/call
        # even headless (Isaac Sim's GPU usage stayed high headless too - "no window" isn't "no
        # rendering work"), well above our own ROS2CableWinchBackend callback's ~1ms - render is
        # the dominant per-step cost, worth testing render=False against directly.
        render_flag = not _headless

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            # Update the UI of the app and perform the physics step
            if profile:
                t0 = time.perf_counter()
                self.world.step(render=render_flag)
                dt_wall = time.perf_counter() - t0
                step_count += 1
                step_time_sum += dt_wall
                step_time_max = max(step_time_max, dt_wall)
                if step_count % 250 == 0:
                    avg_hz = step_count / step_time_sum if step_time_sum > 0 else 0.0
                    print(
                        f"[PROFILE] world.step(render={render_flag}): n={step_count} "
                        f"avg={step_time_sum/step_count*1000:.2f}ms ({avg_hz:.1f} Hz) "
                        f"max={step_time_max*1000:.2f}ms "
                        f"winch_cb_avg={self.winch_backend.profile_avg_ms():.3f}ms "
                        f"winch_cb_max={self.winch_backend.profile_max_ms():.3f}ms",
                        flush=True,
                    )
                    step_count = 0
                    step_time_sum = 0.0
                    step_time_max = 0.0
            else:
                self.world.step(render=render_flag)

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
        rod_a_mass=0.02,
        rod_b_mass=0.02,
        payload_size=0.08,
        payload_mass=0.545,
    )

    # Run the application loop
    fsc_app.run()


if __name__ == "__main__":
    main()

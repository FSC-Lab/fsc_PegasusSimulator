"""
| File: cable_winch_backend_utils.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: ROS2 backend that exposes a variable-length winch cable's prismatic-joint
|   state (extension, extension velocity) and accepts a commanded tension force, for bridging
|   to a real/emulated AK40-10 cable-actuator ROS2 driver.
"""

import time

import numpy as np

import carb
from pxr import Usd, Gf

import omni.usd
from omni.isaac.dynamic_control import _dynamic_control

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from pegasus.simulator.logic.backends.backend import Backend

_AXIS_VEC = {
    "X": Gf.Vec3d(1.0, 0.0, 0.0),
    "Y": Gf.Vec3d(0.0, 1.0, 0.0),
    "Z": Gf.Vec3d(0.0, 0.0, 1.0),
}


class ROS2CableWinchBackend(Backend):

    def __init__(self, world, rod_a_path: str, rod_b_path: str, joint_axis: str = "X", winch_id: int = 0, config: dict = {}):
        """Initialize the ROS2 Cable Winch backend for a variable-length slung-load cable.

        Args:
            world: The Isaac Sim World instance driving the physics simulation.
            rod_a_path (str): Prim path of the proximal (drone-side) winch rod.
            rod_b_path (str): Prim path of the distal (payload-side) winch rod.
            joint_axis (str): Local axis ("X"/"Y"/"Z") the rods slide along. Defaults to "X".
            winch_id (int): Unique id for this winch instance, used to build a unique ROS2 node
                name/topic prefix so multiple winches don't collide.
            config (dict): A dictionary that contains all the parameters for configuring this backend.

            Examples:
                The dictionary default parameters are

                >>> {"topic_prefix": f"cable_winch_{winch_id}/", # Namespace to prepend to the topics
                >>>  "pub_state": True,                          # Publish the cable extension/velocity/force
                >>>  "sub_force": True}                          # Subscribe to the commanded tension force
        """

        self.rod_a_path = rod_a_path
        self.rod_b_path = rod_b_path
        self._axis_local = _AXIS_VEC[joint_axis.upper()]
        self.winch_id = winch_id

        # Save the configurations for this backend
        self._topic_prefix = config.get("topic_prefix", f"cable_winch_{winch_id}/")
        self._pub_state = config.get("pub_state", True)
        self._sub_force = config.get("sub_force", True)

        # Start the actual ROS2 setup here
        try:
            rclpy.init()
        except Exception:
            # If rclpy is already initialized, just ignore the exception
            pass

        # Node name must be unique per instance to avoid colliding with other winches/backends
        self.node = rclpy.create_node(f"simulator_cable_winch_{winch_id}")

        self.initialize_publishers()
        self.initialize_subscribers()

        self._dc_interface = None
        self._commanded_force = 0.0
        self._extension_offset = None  # captured lazily on first physics step / after reset()

        # Profiling only (see PEGASUS_PROFILE in the application script) - wall-clock time spent
        # in update_sim_state() itself, to separate "our own callback cost" from "everything else
        # in a physics step" (PhysX solve, PX4 lockstep, rendering) when investigating why this
        # scenario doesn't sustain its nominal physics_dt in real time.
        self._profile_sum_s = 0.0
        self._profile_max_s = 0.0
        self._profile_count = 0

        self._world = world
        self._world.add_physics_callback(f"cable_winch_{winch_id}_state", self.update_sim_state)

    def profile_avg_ms(self) -> float:
        """Average update_sim_state() wall-clock time (ms) since the last call, then resets."""
        avg = (self._profile_sum_s / self._profile_count * 1000.0) if self._profile_count else 0.0
        self._profile_sum_s = 0.0
        self._profile_count = 0
        return avg

    def profile_max_ms(self) -> float:
        """Max update_sim_state() wall-clock time (ms) since the last call, then resets."""
        m = self._profile_max_s * 1000.0
        self._profile_max_s = 0.0
        return m

    def initialize_publishers(self):
        if self._pub_state:
            self.cable_state_pub = self.node.create_publisher(
                JointState, self._topic_prefix + "state/cable", rclpy.qos.qos_profile_sensor_data
            )
            self.force_state_pub = self.node.create_publisher(
                Float32, self._topic_prefix + "state/force", rclpy.qos.qos_profile_sensor_data
            )

    def initialize_subscribers(self):
        if self._sub_force:
            self.force_sub = self.node.create_subscription(
                Float32, self._topic_prefix + "command/force", self.force_callback, rclpy.qos.qos_profile_sensor_data
            )

    def force_callback(self, msg: Float32):
        """
        Callback that caches the latest commanded tension force. Applied on the next physics step.

        Args:
            msg (Float32): Commanded tension force in N. Positive pulls rod_b toward rod_a (retracting).
        """
        self._commanded_force = msg.data

    def get_dc_interface(self):
        if self._dc_interface is None:
            self._dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        return self._dc_interface

    def update_sim_state(self, dt: float):
        """
        Method called at every physics step: reads the current cable extension/velocity from the
        two winch rods, publishes them, and applies the last commanded tension force. Thin timing
        wrapper around _update_sim_state_impl() for PEGASUS_PROFILE (see application script).
        """
        t0 = time.perf_counter()
        self._update_sim_state_impl(dt)
        elapsed = time.perf_counter() - t0
        self._profile_sum_s += elapsed
        self._profile_count += 1
        self._profile_max_s = max(self._profile_max_s, elapsed)

    def _update_sim_state_impl(self, dt: float):
        """
        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        # Drain any pending ROS2 messages (e.g. force_callback) before applying the force below.
        rclpy.spin_once(self.node, timeout_sec=0)

        dc = self.get_dc_interface()
        rb_a = dc.get_rigid_body(self.rod_a_path)
        rb_b = dc.get_rigid_body(self.rod_b_path)

        pose_a = dc.get_rigid_body_pose(rb_a)
        pose_b = dc.get_rigid_body_pose(rb_b)
        vel_a = np.array(dc.get_rigid_body_linear_velocity(rb_a))
        vel_b = np.array(dc.get_rigid_body_linear_velocity(rb_b))

        # World-frame joint axis direction, derived from rod_a's current orientation (rod_a and
        # rod_b never rotate relative to each other - the prismatic joint forbids it - but the
        # whole assembly can swing, so this must be recomputed every step).
        prim_a = self._world.stage.GetPrimAtPath(self.rod_a_path)
        world_transform = omni.usd.get_world_transform_matrix(prim_a)
        rotation: Gf.Rotation = world_transform.ExtractRotation()
        axis_world = np.array(rotation.TransformDir(self._axis_local))
        axis_world = axis_world / np.linalg.norm(axis_world)

        pos_a = np.array(pose_a.p)
        pos_b = np.array(pose_b.p)

        raw_extension = float(np.dot(pos_b - pos_a, axis_world))
        extension_vel = float(np.dot(vel_b - vel_a, axis_world))

        # Zero the extension reading at the geometric baseline the rods were authored at
        # (analogous to the real driver's zero_position semantics).
        if self._extension_offset is None:
            self._extension_offset = raw_extension
        extension = raw_extension - self._extension_offset

        self.update_state(extension, extension_vel)

        # Apply the last commanded tension as an equal-and-opposite LOCAL-frame force pair, at
        # each rod's center of mass (no induced torque). Using each rod's own local axis vector
        # (rather than axis_world with global=True) matches the only precedented apply_body_force
        # convention in this codebase (vehicle.py's apply_force/apply_torque, always global=False).
        # This is valid because rod_a and rod_b never rotate relative to each other, so the local
        # +axis direction is the sliding direction in both rods' own body frames at all times.
        # Positive commanded force pulls rod_b toward rod_a (retracting), rod_a toward rod_b
        # (Newton's third law).
        force_local_b = (-self._commanded_force * np.array(self._axis_local)).tolist()
        force_local_a = (self._commanded_force * np.array(self._axis_local)).tolist()
        dc.apply_body_force(rb_b, carb._carb.Float3(force_local_b), carb._carb.Float3([0.0, 0.0, 0.0]), False)
        dc.apply_body_force(rb_a, carb._carb.Float3(force_local_a), carb._carb.Float3([0.0, 0.0, 0.0]), False)

    def update_state(self, extension: float, extension_vel: float):
        """
        Publish the current cable extension/velocity and the applied force.
        """
        if not self._pub_state:
            return

        joint_state = JointState()
        joint_state.header.stamp = self.node.get_clock().now().to_msg()
        joint_state.name = ["cable_extension"]
        joint_state.position = [extension]
        joint_state.velocity = [extension_vel]

        force_msg = Float32()
        force_msg.data = float(self._commanded_force)

        self.cable_state_pub.publish(joint_state)
        self.force_state_pub.publish(force_msg)

    def update(self, dt: float):
        """
        Method that is called on every physics step to update the state of the backend. The actual
        ROS2 spin/publish/force-apply work happens in `update_sim_state` (a physics callback), so
        it always runs regardless of whether the owning application script calls `update()` too.
        """
        rclpy.spin_once(self.node, timeout_sec=0)

    def start(self):
        self._commanded_force = 0.0

    def stop(self):
        self._commanded_force = 0.0

    def reset(self):
        self._commanded_force = 0.0
        self._extension_offset = None

    def input_reference(self):
        """Method that returns input reference. Not used for the cable winch backend."""
        return []

    def update_sensor(self, sensor_type: str, data):
        """Method that handles sensor data. Not used for the cable winch backend."""
        pass

    def update_graphical_sensor(self, sensor_type: str, data):
        """Method that handles graphical sensor data. Not used for the cable winch backend."""
        pass

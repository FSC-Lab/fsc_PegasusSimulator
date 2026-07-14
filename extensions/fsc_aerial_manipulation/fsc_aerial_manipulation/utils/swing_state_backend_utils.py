"""
| File: swing_state_backend_utils.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2026, Longhao Qian. All rights reserved.
| Description: ROS2 backend that publishes the normalized swing state (r, v) used by the
|   JGCD paper "Robust Neural Contraction Slung Load Manipulation With Active Cable Length
|   Control" (~/source/JGCD-paper-longhao/main.tex).
|
|   Definitions (main.tex): the cable vector l_c = x_q - x_p (quadrotor position minus payload
|   position, Eq. l_c = x_p + l_c rearranged from main.tex:793), cable length l = ||l_c||, cable
|   direction unit vector n = l_c / l = [r; sqrt(1 - r^T r)] (Eq. eq:def_cable_vector), so r is
|   simply the horizontal (x,y) components of n - "the 2-D offset of the quadrotor when the
|   cable is 1m long" (main.tex:130). v = rdot is computed analytically (NOT by finite-
|   differencing r, which would amplify noise - see fsc_PegasusSimulator/CLAUDE.md's lessons on
|   preferring measured velocities over differentiation) from the directly-available drone/
|   payload linear velocities:
|       l_dot = n . (v_q - v_p)                          (rate of change of cable length)
|       n_dot = ((v_q - v_p) - n * l_dot) / l             (Eq. eq: B_property, ndot = B @ v)
|       v     = n_dot[:2]                                 (== rdot, by construction of B)
|   Both r and v are dimensionless/unit-vector-derived quantities (bounded by ||r|| <= 1),
|   independent of the actual cable length l - they describe the swing geometry only.
"""

import numpy as np

import rclpy
from geometry_msgs.msg import Vector3Stamped

from omni.isaac.dynamic_control import _dynamic_control

from pegasus.simulator.logic.backends.backend import Backend


class ROS2SwingStateBackend(Backend):

    def __init__(self, world, drone_body_path: str, payload_path: str, swing_id: int = 0, config: dict = {}):
        """Initialize the ROS2 swing-state backend.

        Args:
            world: The Isaac Sim World instance driving the physics simulation.
            drone_body_path (str): Prim path of the quadrotor's rigid body (e.g. the path
                returned by constraints.find_rigidbody_prim(stage, drone_path)).
            payload_path (str): Prim path of the payload's rigid body.
            swing_id (int): Unique id for this instance, used to build a unique ROS2 node name
                so multiple instances (e.g. a multi-drone scenario) don't collide.
            config (dict): A dictionary that contains all the parameters for configuring this
                backend.

            Examples:
                The dictionary default parameters are

                >>> {"topic_prefix": f"swing_state_{swing_id}/", # Namespace to prepend to the topics
                >>>  "pub_state": True}                          # Publish r/v every physics step
        """

        self.drone_body_path = drone_body_path
        self.payload_path = payload_path
        self.swing_id = swing_id

        self._topic_prefix = config.get("topic_prefix", f"swing_state_{swing_id}/")
        self._pub_state = config.get("pub_state", True)

        try:
            rclpy.init()
        except Exception:
            # If rclpy is already initialized, just ignore the exception
            pass

        # Node name must be unique per instance to avoid colliding with other swing-state backends
        self.node = rclpy.create_node(f"simulator_swing_state_{swing_id}")

        self.initialize_publishers()

        self._dc_interface = None

        self._world = world
        self._world.add_physics_callback(f"swing_state_{swing_id}_state", self.update_sim_state)

    def initialize_publishers(self):
        if self._pub_state:
            # r, v are 2-D (z always 0) - Vector3Stamped matches this codebase's existing
            # convention for small vector quantities (see rigid_body_backend_utils.py).
            self.r_pub = self.node.create_publisher(
                Vector3Stamped, self._topic_prefix + "r", rclpy.qos.qos_profile_sensor_data
            )
            self.v_pub = self.node.create_publisher(
                Vector3Stamped, self._topic_prefix + "v", rclpy.qos.qos_profile_sensor_data
            )

    def get_dc_interface(self):
        if self._dc_interface is None:
            self._dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        return self._dc_interface

    def update_sim_state(self, dt: float):
        """
        Method called at every physics step: reads the drone/payload rigid-body poses and linear
        velocities, computes the normalized swing state (r, v), and publishes them.
        """
        if not self._pub_state:
            return

        dc = self.get_dc_interface()

        drone_rb = dc.get_rigid_body(self.drone_body_path)
        payload_rb = dc.get_rigid_body(self.payload_path)

        x_q = np.array(dc.get_rigid_body_pose(drone_rb).p)
        x_p = np.array(dc.get_rigid_body_pose(payload_rb).p)
        v_q = np.array(dc.get_rigid_body_linear_velocity(drone_rb))
        v_p = np.array(dc.get_rigid_body_linear_velocity(payload_rb))

        l_c = x_q - x_p
        l = np.linalg.norm(l_c)
        if l < 1e-6:
            # Degenerate (drone and payload coincident) - nothing sensible to publish, skip
            # this step rather than dividing by ~0.
            return
        n = l_c / l

        l_dot = float(n @ (v_q - v_p))
        n_dot = ((v_q - v_p) - n * l_dot) / l

        r = n[:2]
        v = n_dot[:2]

        stamp = self.node.get_clock().now().to_msg()

        r_msg = Vector3Stamped()
        r_msg.header.stamp = stamp
        r_msg.header.frame_id = "map"
        r_msg.vector.x = float(r[0])
        r_msg.vector.y = float(r[1])
        r_msg.vector.z = 0.0
        self.r_pub.publish(r_msg)

        v_msg = Vector3Stamped()
        v_msg.header.stamp = stamp
        v_msg.header.frame_id = "map"
        v_msg.vector.x = float(v[0])
        v_msg.vector.y = float(v[1])
        v_msg.vector.z = 0.0
        self.v_pub.publish(v_msg)

    def update(self, dt: float):
        """Poll for new ROS2 messages in a non-blocking way (no subscribers on this backend)."""
        rclpy.spin_once(self.node, timeout_sec=0)

    def start(self):
        pass

    def stop(self):
        pass

    def reset(self):
        pass

    def input_reference(self):
        """Method that returns input reference. Not used for this backend."""
        return []

    def update_state(self, state):
        """Method that receives the vehicle's own State object each step. Not used for this
        backend - it reads drone/payload poses/velocities directly via dynamic_control in
        update_sim_state() instead (matching ROS2CableWinchBackend's pattern), since it needs
        both the drone AND payload rigid bodies, not just the single vehicle this callback
        would hand it."""
        pass

    def update_sensor(self, sensor_type: str, data):
        """Method that handles sensor data. Not used for this backend."""
        pass

    def update_graphical_sensor(self, sensor_type: str, data):
        """Method that handles graphical sensor data. Not used for this backend."""
        pass

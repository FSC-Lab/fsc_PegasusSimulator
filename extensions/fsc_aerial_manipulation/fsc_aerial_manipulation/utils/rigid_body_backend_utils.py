"""
| File: ros2_backend.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: File that implements the ROS2 Backend for communication/control with/of the vehicle simulation through ROS2 topics
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
"""

# Numerical computations
import numpy as np
from scipy.spatial.transform import Rotation

from pxr import Usd, Gf

# Make sure the ROS2 extension is enabled
import carb
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

import omni.usd
from omni.isaac.dynamic_control import _dynamic_control

# ROS2 imports
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped, Vector3Stamped

from pegasus.simulator.logic.backends.backend import Backend
from pegasus.simulator.logic.state import State


class ROS2RigidBodyBackend(Backend):

    def __init__(self, world, payload_path, config: dict = {}):
        """Initialize the ROS2 Rigid Body Payload class

        Args:
            camera_prim_path (str): Path to the camera prim. Global path when it starts with `/`, else local to vehicle prim path
            config (dict): A Dictionary that contains all the parameters for configuring the ROS2Camera - it can be empty or only have some of the parameters used by the ROS2Camera.

        Examples:
            The dictionary default parameters are

            >>> {"namespace": "drone"                           # Namespace to append to the topics
            >>>  "pub_pose": True,                              # Publish the pose of the vehicle
            >>>  "pub_twist": True,                             # Publish the twist of the vehicle
            >>>  "pub_twist_inertial": True,                    # Publish the twist of the vehicle in the inertial frame
            >>>  "pub_accel": True,                             # Publish the acceleration of the vehicle
            >>>  "pose_topic": "state/pose",                    # Position and attitude of the vehicle in ENU
            >>>  "twist_topic": "state/twist",                  # Linear and angular velocities in the body frame of the vehicle
            >>>  "twist_inertial_topic": "state/twist_inertial" # Linear velocity of the vehicle in the inertial frame
            >>>  "accel_topic": "state/accel",                  # Linear acceleration of the vehicle in the inertial frame
            >>>  "pub_sensors": True,                           # Publish the sensors
            >>>  "pub_state": True,                             # Publish the state of the vehicle
            >>>  "pub_tf": False,                               # Publish the TF of the vehicle
        """

        # Save the configurations for this backend
        self._topic_prefix = config.get("topic_prefix", "payload/")

        # Save what whould be published/subscribed
        self._pub_state = config.get("pub_state", True)
        self._sub_force = config.get("sub_force", True)

        # # Check if the tf2_ros library is loaded and if the flag is set to True
        # self._pub_tf = config.get("pub_tf", False) and tf2_ros_loaded

        # Start the actual ROS2 setup here
        try:
            rclpy.init()
        except:
            # If rclpy is already initialized, just ignore the exception
            pass

        self.node = rclpy.create_node("simulator_payload")

        # Initialize the publishers and subscribers
        self.initialize_publishers(config)
        self.initialize_subscribers()

        # Get the current world at which we want to spawn the vehicle
        self._payload_dc_interface = None
        self.payload_path = payload_path
        # self._current_stage = self.pg._world.stage
        # # print(f"self._current_stage: {self._current_stage}")
        # # Save the name with which the vehicle will appear in the stage
        # # and the name of the .usd file that contains its description
        # self._stage_prefix = get_stage_next_free_path(self._current_stage, payload_path, False)

        self._world = world
        self._world.add_physics_callback(payload_path, self.update_sim_state)

        # Variable that will hold the current state of the vehicle
        self._state = State()
    
    
    def initialize_publishers(self, config: dict):

        # ----------------------------------------------------- 
        # Create publishers for the state of the vehicle in ENU
        # -----------------------------------------------------
        if self._pub_state:
            if config.get("pub_pose", True):
                self.pose_pub = self.node.create_publisher(PoseStamped, self._topic_prefix +  "/" + "state/pose", rclpy.qos.qos_profile_sensor_data)
            
            if config.get("pub_twist", True):
                self.twist_pub = self.node.create_publisher(TwistStamped, self._topic_prefix + "/" + "state/twist", rclpy.qos.qos_profile_sensor_data)

            if config.get("pub_twist_inertial", True):
                self.twist_inertial_pub = self.node.create_publisher(TwistStamped, self._topic_prefix + "/" + "state/twist_inertial", rclpy.qos.qos_profile_sensor_data)

            if config.get("pub_accel", True):
                self.accel_pub = self.node.create_publisher(AccelStamped, self._topic_prefix + "/" + "state/accel", rclpy.qos.qos_profile_sensor_data)
        

    def initialize_subscribers(self):
        self.force_sub = self.node.create_subscription(Vector3Stamped, self._topic_prefix + "/" + "disturbance/force", self.force_callback, rclpy.qos.qos_profile_sensor_data)
        

    def force_callback(self, msg: Vector3Stamped):
        """
        Callback that is called when a new force message is received from ROS2 topic

        Args:
            msg (Vector3Stamped): Message containing the force to be applied to the vehicle in N in X, Y, Z directions
        """
        self.input_force = [msg.vector.x, msg.vector.y, msg.vector.z]

    def update_state(self):
        """
        Method that when implemented, should handle the receivel of the state of the vehicle using this callback
        """

        # Publish the state of the vehicle only if the flag is set to True
        if not self._pub_state:
            return

        pose = PoseStamped()
        twist = TwistStamped()
        twist_inertial = TwistStamped()
        accel = AccelStamped()

        # Update the header
        pose.header.stamp = self.node.get_clock().now().to_msg()
        twist.header.stamp = pose.header.stamp
        twist_inertial.header.stamp = pose.header.stamp
        accel.header.stamp = pose.header.stamp

        pose.header.frame_id = "map"
        twist.header.frame_id = self._topic_prefix + "_" + "base_link"
        twist_inertial.header.frame_id = "map"
        accel.header.frame_id = "map"

        # Fill the position and attitude of the vehicle in ENU
        pose.pose.position.x = self._state.position[0]
        pose.pose.position.y = self._state.position[1]
        pose.pose.position.z = self._state.position[2]

        pose.pose.orientation.x = self._state.attitude[0]
        pose.pose.orientation.y = self._state.attitude[1]
        pose.pose.orientation.z = self._state.attitude[2]
        pose.pose.orientation.w = self._state.attitude[3]

        # Fill the linear and angular velocities in the body frame of the vehicle
        twist.twist.linear.x = self._state.linear_body_velocity[0]
        twist.twist.linear.y = self._state.linear_body_velocity[1]
        twist.twist.linear.z = self._state.linear_body_velocity[2]

        twist.twist.angular.x = self._state.angular_velocity[0]
        twist.twist.angular.y = self._state.angular_velocity[1]
        twist.twist.angular.z = self._state.angular_velocity[2]

        # Fill the linear velocity of the vehicle in the inertial frame
        twist_inertial.twist.linear.x = self._state.linear_velocity[0]
        twist_inertial.twist.linear.y = self._state.linear_velocity[1]
        twist_inertial.twist.linear.z = self._state.linear_velocity[2]

        # Fill the linear acceleration in the inertial frame
        accel.accel.linear.x = self._state.linear_acceleration[0]
        accel.accel.linear.y = self._state.linear_acceleration[1]
        accel.accel.linear.z = self._state.linear_acceleration[2]

        # Publish the messages containing the state of the vehicle
        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)
        self.twist_inertial_pub.publish(twist_inertial)
        self.accel_pub.publish(accel)
        

    def update(self, dt: float):
        """
        Method that when implemented, should be used to update the state of the backend and the information being sent/received
        from the communication interface. This method will be called by the simulation on every physics step
        """

        # In this case, do nothing as we are sending messages as soon as new data arrives from the sensors and state
        # and updating the reference for the thrusters as soon as receiving from ROS2 topics
        # Just poll for new ROS 2 messages in a non-blocking way
        rclpy.spin_once(self.node, timeout_sec=0)

    def start(self):
        """
        Method that when implemented should handle the begining of the simulation of vehicle
        """
        # Reset the reference for the thrusters
        self.input_force = [0.0, 0.0, 0.0]  # Force in N in X, Y, Z directions

    def stop(self):
        """
        Method that when implemented should handle the stopping of the simulation of vehicle
        """
        # Reset the reference for the thrusters
        self.input_force = [0.0, 0.0, 0.0]  # Force in N in X, Y, Z directions

    def reset(self):
        """
        Method that when implemented, should handle the reset of the vehicle simulation to its original state
        """
        # Reset the reference for the thrusters
        self.input_force = [0.0, 0.0, 0.0]  # Force in N in X, Y, Z directions

    def input_reference(self):
        """Method that returns input reference. Not used for rigid body payload."""
        return []

    def update_sensor(self, sensor_type: str, data):
        """Method that handles sensor data. Not used for rigid body payload."""
        pass

    def update_graphical_sensor(self, sensor_type: str, data):
        """Method that handles graphical sensor data. Not used for rigid body payload."""
        pass

    def update_sim_state(self, dt: float):
        """
        Method that is called at every physics step to retrieve and update the current state of the vehicle, i.e., get
        the current position, orientation, linear and angular velocities and acceleration of the vehicle.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Get the body frame interface of the vehicle (this will be the frame used to get the position, orientation, etc.)
        payload = self.get_dc_interface().get_rigid_body(self.payload_path)

        # Get the current position and orientation in the inertial frame
        pose = self.get_dc_interface().get_rigid_body_pose(payload)

        # Get the attitude according to the convention [w, x, y, z]
        prim = self._world.stage.GetPrimAtPath(self.payload_path)
        rotation_quat = self.get_world_transform_xform(prim).GetQuaternion()
        rotation_quat_real = rotation_quat.GetReal()
        rotation_quat_img = rotation_quat.GetImaginary()

        # Get the angular velocity of the vehicle expressed in the body frame of reference
        ang_vel = self.get_dc_interface().get_rigid_body_angular_velocity(payload)

        # The linear velocity [x_dot, y_dot, z_dot] of the vehicle's body frame expressed in the inertial frame of reference
        linear_vel = self.get_dc_interface().get_rigid_body_linear_velocity(payload)

        # Get the linear acceleration of the body relative to the inertial frame, expressed in the inertial frame
        # Note: we must do this approximation, since the Isaac sim does not output the acceleration of the rigid body directly
        linear_acceleration = (np.array(linear_vel) - self._state.linear_velocity) / dt

        # Update the state variable X = [x,y,z]
        self._state.position = np.array(pose.p)

        # Get the quaternion according in the [qx,qy,qz,qw] standard
        self._state.attitude = np.array(
            [rotation_quat_img[0], rotation_quat_img[1], rotation_quat_img[2], rotation_quat_real]
        )

        # Express the velocity of the vehicle in the inertial frame X_dot = [x_dot, y_dot, z_dot]
        self._state.linear_velocity = np.array(linear_vel)

        # The linear velocity V =[u,v,w] of the vehicle's body frame expressed in the body frame of reference
        # Note that: x_dot = Rot * V
        self._state.linear_body_velocity = (
            Rotation.from_quat(self._state.attitude).inv().apply(self._state.linear_velocity)
        )

        # omega = [p,q,r]
        self._state.angular_velocity = Rotation.from_quat(self._state.attitude).inv().apply(np.array(ang_vel))

        # The acceleration of the vehicle expressed in the inertial frame X_ddot = [x_ddot, y_ddot, z_ddot]
        self._state.linear_acceleration = linear_acceleration

        self.update_state()
    
    def get_dc_interface(self):

        if self._payload_dc_interface is None:
            self._payload_dc_interface = _dynamic_control.acquire_dynamic_control_interface()

        return self._payload_dc_interface
    
    def get_world_transform_xform(self, prim: Usd.Prim):
        """
        Get the local transformation of a prim using omni.usd.get_world_transform_matrix().
        See https://docs.omniverse.nvidia.com/kit/docs/omni.usd/latest/omni.usd/omni.usd.get_world_transform_matrix.html
        Args:
            prim (Usd.Prim): The prim to calculate the world transformation.
        Returns:
            A tuple of:
            - Translation vector.
            - Rotation quaternion, i.e. 3d vector plus angle.
            - Scale vector.
        """
        world_transform: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
        rotation: Gf.Rotation = world_transform.ExtractRotation()
        return rotation
"""
Pegasus backend: drive the VTOL from an external ROS 2 controller.

Subscribes to /<ns><id>/vtol/actuator_cmd (std_msgs/Float64MultiArray) published by
the fsc_vtol_controller node, and returns it to the VTOL as the {channel: value}
command dict. This is backend[0] (the VTOL reads its command from here); pair it
with a state-publishing ROS2Backend so the controller gets feedback:

    Isaac VTOL --[ROS2Backend]--> /uav_0/state/* --[mocap emulator]--> /uav_0/mocap
              --[fsc_vtol_controller]--> /uav_0/vtol/actuator_cmd --[THIS backend]--> VTOL

Runs rclpy inside Isaac (uses Isaac's ROS2 bridge), so launch the scene from a
clean shell (unset PYTHONPATH/conda), not a ROS-sourced one.
"""

from pegasus.simulator.logic.backends.backend import Backend

# Must match the controller's Float64MultiArray order + the airframe channels.
DEFAULT_CHANNELS = [
    "motor_front_L", "motor_front_R", "motor_rear_L", "motor_rear_R",
    "tilt_left", "tilt_right", "aileron", "elevator", "rudder",
]


class VTOLCommandROS2Backend(Backend):
    """Return the latest actuator command received over ROS 2 as the VTOL command."""

    def __init__(self, vehicle_id: int = 0, namespace: str = "uav_",
                 channels=None, topic: str | None = None):
        super().__init__(config=None)
        import rclpy
        from std_msgs.msg import Float64MultiArray

        try:
            rclpy.init()
        except Exception:
            pass  # already initialised

        self._channels = list(channels or DEFAULT_CHANNELS)
        self._cmd: dict = {}
        self._node = rclpy.create_node(f"vtol_command_sub_{vehicle_id}")
        topic = topic or f"/{namespace}{vehicle_id}/vtol/actuator_cmd"
        self._sub = self._node.create_subscription(
            Float64MultiArray, topic, self._on_cmd, 10)

    def _on_cmd(self, msg):
        self._cmd = {ch: float(v) for ch, v in zip(self._channels, msg.data)}

    def input_reference(self):
        return self._cmd

    def update(self, dt: float):
        import rclpy
        rclpy.spin_once(self._node, timeout_sec=0.0)   # process incoming commands

    # --- unused Backend interface ---
    def update_sensor(self, sensor_type, data): pass
    def update_graphical_sensor(self, sensor_type, data): pass
    def update_state(self, state): pass
    def start(self): pass
    def stop(self): pass

    def reset(self):
        self._cmd = {}

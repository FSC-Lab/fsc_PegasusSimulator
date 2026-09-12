#!/usr/bin/env python3
"""Attitude-setpoint MUX for a live inner-loop controller switch.

Two PX4-inner-loop controllers -- the fsc attitude autopilot (rock-solid sub-mm
hover, the "safe" one) and the attitude-NMPC (tracks the trajectory reference) --
each publish their VehicleAttitudeSetpoint to a PRIVATE topic via a launch-time
remap:

    fsc  autopilot_sv_baseline_node : ... -r /<ns>/fmu/in/vehicle_attitude_setpoint \
                                             :=/<ns>/mux/fsc/vehicle_attitude_setpoint
    nmpc nmpc_attitude_node.py       : ... -r /<ns>/fmu/in/vehicle_attitude_setpoint \
                                             :=/<ns>/mux/nmpc/vehicle_attitude_setpoint

This node forwards ONLY the currently-selected controller's setpoints to the REAL
/<ns>/fmu/in/vehicle_attitude_setpoint, so exactly one drives PX4's fast loop at a
time (no fighting). Both controllers keep publishing OffboardControlMode(attitude)
+ arm/offboard to the REAL topics (idempotent + consistent -- both request attitude
offboard), so only the attitude SETPOINT needs muxing. That means NO controller code
change -- the whole switch is launch remaps + this relay.

Selection: std_msgs/String on /<ns>/ground_station/active_controller
    "nmpc"                     -> forward the NMPC   (trajectory tracking)
    "fsc"/"geometric"/"safe"   -> forward the fsc    (safe hover)   [default]

The GS Trajectory tab's existing "Safe Geometric"/"NMPC" buttons publish
"geometric"/"nmpc" -- so this drops straight in with NO ground-station change (the
"geometric" alias maps to the fsc safe controller). HOVER->safe, Play->nmpc, as before.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy)
from std_msgs.msg import String
from px4_msgs.msg import VehicleAttitudeSetpoint

# active_controller string -> which private input to forward
NMPC_IDS = {"nmpc"}
FSC_IDS = {"fsc", "geometric", "safe", "hover"}   # "geometric" = the GS safe button


class AttitudeSetpointMux(Node):
    def __init__(self):
        super().__init__("attitude_setpoint_mux")
        self.declare_parameter("px4_ns", "uav_0")
        self.declare_parameter("default", "fsc")          # which controller drives at start
        ns = self.get_parameter("px4_ns").get_parameter_value().string_value
        pre = f"/{ns}" if ns else ""
        self.active = "nmpc" if self.get_parameter("default").value in NMPC_IDS else "fsc"

        # inputs: VOLATILE + BEST_EFFORT accepts whatever either controller publishes
        qin = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        # output: match what the controllers used publishing straight to PX4 (works live)
        qout = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.pub = self.create_publisher(
            VehicleAttitudeSetpoint, f"{pre}/fmu/in/vehicle_attitude_setpoint", qout)
        self.create_subscription(
            VehicleAttitudeSetpoint, f"{pre}/mux/fsc/vehicle_attitude_setpoint",
            lambda m: self._fwd("fsc", m), qin)
        self.create_subscription(
            VehicleAttitudeSetpoint, f"{pre}/mux/nmpc/vehicle_attitude_setpoint",
            lambda m: self._fwd("nmpc", m), qin)
        self.create_subscription(
            String, f"{pre}/ground_station/active_controller", self._on_sel, 10)
        self._fwd_count = 0
        self.create_timer(2.0, self._heartbeat)
        self.get_logger().info(
            f"attitude mux up (ns={ns}): active={self.active}  "
            f"fsc<-mux/fsc, nmpc<-mux/nmpc -> fmu/in/vehicle_attitude_setpoint")

    def _on_sel(self, m):
        d = m.data.strip().lower()
        new = "nmpc" if d in NMPC_IDS else "fsc" if d in FSC_IDS else None
        if new is None:
            self.get_logger().warn(f"unknown active_controller '{m.data}' (ignored)")
            return
        if new != self.active:
            self.active = new
            self.get_logger().info(f"SWITCH active controller -> {new} (from '{m.data}')")

    def _fwd(self, who, m):
        if who == self.active:
            self.pub.publish(m)
            self._fwd_count += 1

    def _heartbeat(self):
        self.get_logger().info(f"active={self.active}  forwarded={self._fwd_count} sp/2s")
        self._fwd_count = 0


def main():
    rclpy.init()
    try:
        rclpy.spin(AttitudeSetpointMux())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

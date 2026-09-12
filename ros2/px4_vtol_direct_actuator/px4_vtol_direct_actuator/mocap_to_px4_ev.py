#!/usr/bin/env python3
"""Mocap -> PX4 external-vision (EKF2 EV) bridge.

Feeds the mocap pose to PX4's estimator so its EKF has a correct attitude/yaw. This is
what makes the ATTITUDE-setpoint inner loop (B, nmpc_attitude_node) viable in HIL: B hands
attitude control to PX4, which tracks setpoints against its EKF -- and without a good
mag/GPS the HIL EKF yaw drifts (that was B's fly-away root cause). With this bridge the EKF
fuses the mocap pose (POSE_FRAME_NED) and yaw converges.

Frames: mocap is ENU world / FLU body; PX4 VehicleOdometry is NED world / FRD body.
  R_ned(FRD->NED) = M_en . R_enu(FLU->ENU) . M_bf     (M_en=ENU->NED, M_bf=FLU->FRD)
  pos_ned = M_en . pos_enu ;  vel_ned = M_en . vel_enu ;  omega_frd = M_bf . omega_flu

Enable in PX4 (see start_vtol_smc_hil.sh / the param window), then `ekf2 stop; ekf2 start`:
  EKF2_EV_CTRL=11 (hpos+vpos+yaw)  EKF2_HGT_REF=3(EV)  EKF2_GPS_CTRL=0  EKF2_EV_DELAY~50
The EV timestamp is put in PX4 time by learning the ROS<->PX4 clock offset from
/fmu/out/vehicle_attitude, so UXRCE_DDS_SYNCT can be either 0 or 1.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy, qos_profile_sensor_data)
from px4_msgs.msg import VehicleOdometry, VehicleAttitude
from fsc_autopilot_ros2_msgs.msg import Mocap

M_EN = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])   # ENU -> NED (coords)
M_BF = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])  # FLU -> FRD (body)


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s; x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return (w, x, y, z)


class Bridge(Node):
    def __init__(self):
        super().__init__("mocap_to_px4_ev")
        self.declare_parameter("px4_ns", "uav_0")
        ns = self.get_parameter("px4_ns").value
        pre = f"/{ns}" if ns else ""
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(VehicleOdometry, f"{pre}/fmu/in/vehicle_visual_odometry", qos)
        self.create_subscription(Mocap, f"{pre}/mocap", self.cb, qos_profile_sensor_data)
        # Learn the ROS<->PX4 clock offset from a PX4 output topic so the EV timestamp is in
        # PX4 time REGARDLESS of UXRCE_DDS_SYNCT (a wrong-time EV sample is silently rejected).
        self.px4_dt_us = None
        self.create_subscription(VehicleAttitude, f"{pre}/fmu/out/vehicle_attitude",
                                 self.on_px4_time, qos_profile_sensor_data)
        self.n = 0
        self.get_logger().info(f"mocap->px4 EV bridge up: {pre}/mocap -> {pre}/fmu/in/vehicle_visual_odometry")

    def on_px4_time(self, m):
        # offset = px4_hrt - ros_clock (both us); EV timestamp = ros_now + offset -> PX4 time.
        self.px4_dt_us = int(m.timestamp) - int(self.get_clock().now().nanoseconds / 1000)

    def _px4_now_us(self):
        ros = int(self.get_clock().now().nanoseconds / 1000)
        return ros + self.px4_dt_us if self.px4_dt_us is not None else ros

    def cb(self, m):
        p, o = m.pose.position, m.pose.orientation
        pos_ned = M_EN @ np.array([p.x, p.y, p.z])
        R_ned = M_EN @ quat_to_R(o.x, o.y, o.z, o.w) @ M_BF
        qw, qx, qy, qz = R_to_quat(R_ned)
        vel_ned = M_EN @ np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])
        om_frd = M_BF @ np.array([m.twist.angular.x, m.twist.angular.y, m.twist.angular.z])
        msg = VehicleOdometry()
        t = self._px4_now_us()
        msg.timestamp = t; msg.timestamp_sample = t
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = [float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2])]
        msg.q = [float(qw), float(qx), float(qy), float(qz)]
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        msg.velocity = [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])]
        msg.angular_velocity = [float(om_frd[0]), float(om_frd[1]), float(om_frd[2])]
        msg.position_variance = [0.01, 0.01, 0.01]
        msg.orientation_variance = [0.01, 0.01, 0.02]
        msg.velocity_variance = [0.04, 0.04, 0.04]
        msg.reset_counter = 0
        msg.quality = 100
        self.pub.publish(msg)
        self.n += 1
        if self.n % 240 == 1:
            lock = "px4-clk" if self.px4_dt_us is not None else "ros-clk(no px4 time yet)"
            self.get_logger().info("EV pos_ned=(%.1f,%.1f,%.1f) yaw sent [%s] (#%d)"
                                   % (pos_ned[0], pos_ned[1], pos_ned[2], lock, self.n))


def main():
    rclpy.init()
    n = Bridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()

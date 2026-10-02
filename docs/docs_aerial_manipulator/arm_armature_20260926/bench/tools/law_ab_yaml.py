#!/usr/bin/env python3
"""Swap the 4-D mirror yaml's LAW between the 2026-09-26 configuration and the
one flown 0918-0924, leaving the PLANT (sim_* keys, incl. the bench armature)
untouched -- so an Isaac A/B flies the same arm under the two laws.

    python3 law_ab_yaml.py old|new [yaml]

old: link armature (wb_armature_joint_diag false), Present Velocity
     (wb_arm_velocity_topic empty), K_y/D_y 20/12, M_r_d as flown.
new: the committed 2026-09-26 values (joint-diagonal bench armature, the arm's
     velocity observer, K_y/D_y 80/24, M_r_d rescaled).
Only the /**/fsc_autopilot_ros2 section's values change; comments stay.
"""
import re
import sys

Y = (sys.argv[2] if len(sys.argv) > 2 else
     "/home/shiqi/ros2_ws/src/fsc_autopilot_ros2/config/"
     "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml")
NEW = {"wb_armature_joint_diag": "true", "wb_arm_velocity_topic":
       '"fsc_open_manipulator/external_torque_controller/velocity_observer"',
       "wb_ky_x": "80.0", "wb_ky_y": "80.0", "wb_ky_z": "80.0",
       "wb_dy_x": "24.0", "wb_dy_y": "24.0", "wb_dy_z": "24.0",
       "wb_mrd_x": "0.130710", "wb_mrd_y": "0.135962", "wb_mrd_z": "0.134261",
       "armature_joint_diag": "true"}
OLD = {"wb_armature_joint_diag": "false", "wb_arm_velocity_topic": '""',
       "wb_ky_x": "20.0", "wb_ky_y": "20.0", "wb_ky_z": "20.0",
       "wb_dy_x": "12.0", "wb_dy_y": "12.0", "wb_dy_z": "12.0",
       "wb_mrd_x": "0.116522", "wb_mrd_y": "0.136107", "wb_mrd_z": "0.125102",
       "armature_joint_diag": "false"}

if __name__ == "__main__":
    which = sys.argv[1]
    vals = {"old": OLD, "new": NEW}[which]
    s = open(Y).read()
    for k, v in vals.items():
        s, n = re.subn(rf"^(\s*{k}:\s*)(\S+)", lambda m: m.group(1) + v, s, count=1, flags=re.M)
        if n != 1:
            raise SystemExit(f"key {k} not found exactly once")
    open(Y, "w").write(s)
    print(f"law set to {which}: " + ", ".join(f"{k}={v}" for k, v in vals.items()))

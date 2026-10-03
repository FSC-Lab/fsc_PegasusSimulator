#!/usr/bin/env python3
"""Align the 0912 whole-body-L1 hardware flight onto one clock and one frame.

Two clocks are in the bag: the recorder (laptop) clock, and the vehicle
computer's, exactly 1415.4796 s behind it (measured: median receive-minus-header
over every stamped vehicle-side topic, spread 13 ms = transport jitter, no
drift).  Everything here is expressed on the recorder clock with t=0 at the
first `wb_control_debug` sample.

Two orderings are in the bag too.  The joint_state_broadcaster publishes
[j2, j3, j1, j4]; everything else is j1..j4.  And `wb_control_debug` carries the
law's MODEL convention while the arm topics carry the HARDWARE one, related by
`wb_arm_sign = [-1, +1, +1, -1]`.  Arm torques are compared in the HARDWARE
convention, which is the one the servos actually see.

Run with the system python3 (PYTHONNOUSERSITE=1); no ROS needed.
"""
import numpy as np

CLOCK_OFFSET = 1415.4796           # vehicle-computer clock -> recorder clock [s]
BCAST_TO_J1234 = [2, 0, 1, 3]      # [j2,j3,j1,j4] -> [j1,j2,j3,j4]
ARM_SIGN = np.array([-1.0, 1.0, 1.0, -1.0])          # wb_arm_sign_j1..j4
NM_TO_COUNTS = np.array([162.4, 154.0, 150.5, 153.4])  # nm_to_effort_joints
MAX_EFFORT_COUNTS = np.array([57.6117, 364.8743, 192.0391, 57.6117])
TAU_CLAMP_NM = MAX_EFFORT_COUNTS / NM_TO_COUNTS       # the binding arm-side clamp

# wb_control_debug index map (97 elements as flown; the client header documents
# 0..88, and 89..96 are the per-joint observer share added for this flight).
IDX = dict(
    mode=0, hold=1, gmo=2, armref_fresh=3, armstate_fresh=4,
    q=slice(5, 9), q_d=slice(9, 13), tau_joint=slice(13, 17),
    u1=17, tau_body_model=slice(18, 21), tau_body_flu=slice(21, 24),
    e_y=slice(24, 28), e_R=slice(28, 31),
    d_f=slice(31, 41),                 # filtered lumped estimate the loops see
    motors=slice(41, 45), x_cd=slice(45, 48), x_c=slice(48, 51),
    n_sat=51, unalloc_tau=slice(52, 55), unalloc_thrust=55, stream_fresh=56,
    l1_active=57, F_hat_y=slice(58, 62),
    d_sigma_c=slice(62, 72),           # unfiltered deadbeat estimate
    w_e=slice(72, 78),                 # EE-frame interaction wrench (6)
    w_hat=slice(78, 88),               # internal disturbance (10)
    n_clamped=88,
    obs_share_raw=slice(89, 93), obs_share_applied=slice(93, 97),
)


def load(src, out):
    d = np.load(src)
    tw = d["wb_control_debug_t"]
    t0 = tw[0]
    w = d["wb_control_debug_v"].astype(np.float64)
    o = {"t": tw - t0, "dbg": w}

    D = w[:, IDX["mode"]] > 0.5
    o["direct"] = D
    o["t_direct"] = (float((tw - t0)[D][0]), float((tw - t0)[D][-1]))

    # ---- arm, hardware convention -----------------------------------------
    js = d["joint_states_v"]
    o["t_js"] = d["joint_states_t"] + CLOCK_OFFSET - t0
    o["q_meas"] = js[:, 0:4][:, BCAST_TO_J1234]
    o["qdot_meas"] = js[:, 4:8][:, BCAST_TO_J1234]
    o["tau_applied"] = js[:, 8:12][:, BCAST_TO_J1234] / NM_TO_COUNTS

    o["t_cmd"] = d["joint_torque_command_t"] + CLOCK_OFFSET - t0
    o["tau_cmd"] = d["joint_torque_command_v"][:, 8:12]

    o["t_armref"] = d["reference_joint_trajectory_t"] + CLOCK_OFFSET - t0
    o["q_armref"] = d["reference_joint_trajectory_v"][:, 0:4]

    # ---- streamed whole-body reference (MODEL frame, vehicle clock) --------
    o["t_ref"] = d["reference_t"] + CLOCK_OFFSET - t0
    R = d["reference_v"]
    for i, k in enumerate(["x_cd", "x_cd_dot", "x_cd_ddot", "b1_d",
                           "r_ed", "r_ed_dot", "b1_de"]):
        o["ref_" + k] = R[:, 3 * i:3 * i + 3]
    o["ref_q_d"] = R[:, 21:25]
    o["ref_qdot_d"] = R[:, 25:29]

    # ---- base state, recorder clock already --------------------------------
    o["t_odom"] = d["odom_t"] - t0
    o["odom"] = d["odom_v"]

    o["t_batt"] = d["battery_status_t"] + CLOCK_OFFSET - t0
    o["batt"] = d["battery_status_v"]

    o["t_ude"] = d["ude_t"] - t0
    o["ude"] = d["ude_v"]

    o["mode_t"] = d["mode_t"] - t0
    o["mode_v"] = d["mode_v"]
    o["status_t"] = d["status_t"] - t0
    o["status_v"] = d["status_v"]

    np.savez_compressed(out, **o)
    return o


if __name__ == "__main__":
    import sys
    o = load(sys.argv[1], sys.argv[2])
    print("DIRECT %.2f .. %.2f s" % o["t_direct"])
    print("arm clamp (N.m) =", np.round(TAU_CLAMP_NM, 4))
    print("wrote", sys.argv[2])

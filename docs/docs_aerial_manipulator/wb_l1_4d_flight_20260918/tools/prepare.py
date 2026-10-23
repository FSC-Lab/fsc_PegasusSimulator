#!/usr/bin/env python3
"""Align the 0918 whole-body-L1-4D hardware flight onto one clock and one frame.

Input: the npz written by extract_bag.py (every topic flattened by field name).
Output: a numeric-only npz (loads under numpy 1.x, which the apt matplotlib needs).

Clocks: vehicle-side stamped topics run 0.348 s AHEAD of the recorder (median
receive-minus-header, spread 30 ms); the state estimator's odometry is stamped on
the recorder side. Everything is put on the recorder clock, t=0 at the first
wb_control_debug sample. wb_control_debug has no header, so it keeps receive time.

Orderings: joint_state_broadcaster publishes [j2, j3, j1, j4]; everything else
j1..j4. wb_control_debug carries the law's MODEL convention, the arm topics the
HARDWARE one (wb_arm_sign = [-1, +1, +1, -1]); arm torques are compared in the
hardware convention.

Run with /usr/bin/python3 and the user-site numpy 2.x (the extractor's).
"""
import sys
import numpy as np

KT = np.array([162.4, 154.0, 150.5, 153.4])         # nm_to_effort_joints: current counts per N.m
KPWM = np.array([169.47, 149.70, 135.25, 148.51])   # duty counts per N.m (kappa_pwm, the arm's own log)
MAX_EFFORT = np.array([57.6117, 364.8743, 192.0391, 57.6117])  # duty counts
ARM_SIGN = np.array([-1.0, 1.0, 1.0, -1.0])

def main(src, out):
    d = np.load(src, allow_pickle=True)
    t0 = d["wb__recv"][0]
    OFF = float(np.median(d["js__recv"] - d["js__hdr"]))
    def T(key, hdr=True):
        if hdr and f"{key}__hdr" in d: return d[f"{key}__hdr"] + OFF - t0
        return d[f"{key}__recv"] - t0
    o = {"clock_offset": OFF}
    o["t"] = T("wb", hdr=False); o["dbg"] = d["wb__data"]; o["direct"] = o["dbg"][:, 0] > 0.5
    # odometry (recorder clock)
    o["t_odom"] = d["odom__recv"] - t0
    o["odom_pos"] = np.column_stack([d[f"odom__pose.pose.position.{k}"] for k in "xyz"])
    o["odom_quat"] = np.column_stack([d[f"odom__pose.pose.orientation.{k}"] for k in "wxyz"])
    o["odom_vel"] = np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"])
    o["odom_omega"] = np.column_stack([d[f"odom__twist.twist.angular.{k}"] for k in "xyz"])
    # arm, hardware convention, j1..j4
    names = list(d["js__name"][0]); order = [names.index(f"joint{k}") for k in (1, 2, 3, 4)]
    o["t_js"] = T("js"); o["q_meas"] = d["js__position"][:, order]; o["qdot_meas"] = d["js__velocity"][:, order]
    o["tau_applied"] = d["js__effort"][:, order] / KT
    o["t_cmd"] = T("tcmd"); o["tau_cmd"] = d["tcmd__effort"]
    o["t_law"] = T("law"); o["law"] = d["law__data"]
    o["tau_duty_nm"] = d["law__data"][:, 13:17] / KPWM
    o["t_armref"] = T("armref"); o["q_armref"] = d["armref__points[0].positions"]
    o["qdot_armref"] = d["armref__points[0].velocities"]          # the planner's reference rate, rad/s
    vn = list(d["velobs__name"][0]); vo = [vn.index(f"joint{k}") for k in (1, 2, 3, 4)]
    o["t_velobs"] = T("velobs"); o["qdot_observer"] = d["velobs__velocity"][:, vo]
    # streamed whole-body reference (model frame)
    o["t_ref"] = T("wbref")
    for k in ["x_cd", "x_cd_dot", "b1_d", "r_ed", "r_ed_dot", "b1_de"]:
        o["ref_" + k] = np.column_stack([d[f"wbref__{k}.{a}"] for a in "xyz"])
    o["ref_q_d"] = d["wbref__q_d"]
    # battery, ude, planner phases, mode
    o["t_batt"] = d["batt__recv"] - t0; o["batt_v"] = d["batt__voltage_v"]; o["batt_i"] = d["batt__current_a"]
    o["t_ude"] = d["ude__recv"] - t0
    o["ude_dhat"] = np.column_stack([d[f"ude__disturbance_estimate.{k}"] for k in "xyz"])
    ts = d["pl_status__recv"] - t0; sv = [str(x) for x in d["pl_status__data"]]
    legs = []
    for i, s in enumerate(sv):
        if s.startswith("EXECUTING"):
            j = i + 1
            while j < len(sv) and not sv[j].startswith("HOLD"): j += 1
            legs.append((ts[i], ts[j]))
    o["legs"] = np.array(legs)
    tm = d["wbmode__recv"] - t0; mv = [str(x) for x in d["wbmode__data"]]
    o["t_direct"] = np.array([tm[[i for i in range(len(mv)) if mv[i] == "DIRECT"][0]], tm[[i for i in range(len(mv)) if mv[i] == "DIRECT"][-1]]])
    np.savez_compressed(out, **o)
    D = o["direct"]; print("DIRECT %.2f .. %.2f s" % (o["t"][D][0], o["t"][D][-1])); print("legs", np.round(o["legs"], 2)); print("clock offset", OFF)
    # a few timings the report quotes
    law = d["law__data"]; tl = o["t_law"]
    for j in range(4):
        m = np.abs(law[:, 13 + j]) >= MAX_EFFORT[j] - 0.5
        if m.any(): print(f"joint {j+1} duty at max_effort: {m.sum()} ticks, t = {tl[m].min():.2f}..{tl[m].max():.2f} s")
    Fr = o["dbg"][D, 97:100]; td = o["t"][D]; k = np.argmax(np.linalg.norm(Fr, axis=1)); print(f"raw F_hat max {np.linalg.norm(Fr[k]):.1f} N at t = {td[k]:.3f} s")
    print("wrote", out)

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])

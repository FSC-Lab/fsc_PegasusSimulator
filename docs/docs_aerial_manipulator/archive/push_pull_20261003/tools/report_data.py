#!/usr/bin/env python3
"""Collect the 2026-10-05/06 push-and-pull runs into one JSON for the summary page.
    PYTHONNOUSERSITE=1 /usr/bin/python3 report_data.py > ../runs/report_data.json"""
import json, os, sys
import numpy as np
import pl_metrics as PM
from pl_score import STEPS, tilt_deg, yaw_deg, col, D_EY
from pl_metrics import mark

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs")
# tag -> (feedback, configuration label, group key)
CFG = {
 "pl_9":  ("raw", "shipped", "raw_shipped"), "pl_15": ("raw", "shipped", "raw_shipped"),
 "pl_23": ("raw", "shipped", "raw_shipped"),
 "pl_20": ("raw", "rotational observer 0.4", "raw_gain"), "pl_21": ("raw", "heading K/D 0.6/0.5", "raw_gain"),
 "pl_22": ("raw", "heading 0.6/0.5 + rot. observer 0.4", "raw_gain"), "pl_24": ("raw", "base damping k_v 20", "raw_gain"),
 "pl_25": ("raw", "EE damping D_y 40", "raw_gain"), "pl_26": ("raw", "push 20 s", "raw_gain"),
 "pl_27": ("raw", "attitude from mocap", "raw_att"), "pl_28": ("raw", "attitude from mocap", "raw_att"),
 "pl_29": ("raw", "attitude correction 1 rad/s", "raw_corr1"), "pl_30": ("raw", "attitude correction 1 rad/s", "raw_corr1"),
 "pl_31": ("raw", "attitude correction 1 rad/s", "raw_corr1"),
 "pl_32": ("raw", "attitude correction 3 rad/s", "raw_corr3"), "pl_33": ("raw", "attitude correction 3 rad/s", "raw_corr3"),
 "pl_10": ("fused", "shipped", "fused_shipped"), "pl_19": ("fused", "shipped", "fused_shipped"),
 "pl_34": ("fused", "shipped", "fused_shipped"),
 "pl_11": ("fused", "transl. observer 1.0", "fused_gain"), "pl_12": ("fused", "contact reading 0.5", "fused_gain"),
 "pl_13": ("fused", "contact reading 5.0", "fused_gain"), "pl_14": ("fused", "EE K_y/D_y 80/24", "fused_gain"),
 "pl_16": ("fused", "contact reading 5.0", "fused_gain"), "pl_17": ("fused", "contact 5.0 + attitude gains", "fused_gain"),
 "pl_18": ("fused", "contact 5.0 + rot. observer 0.4", "fused_gain"),
 "pl_35": ("fused", "EKF2 position trust 1 cm", "fused_ekf"), "pl_36": ("fused", "EKF2 full mocap trust", "fused_ekf"),
 # 2026-10-06: the real box (240 x 160 x 95 mm) with a post handle on its top centre, push pose [0, 20, 40, 0]
 "pl_37": ("raw", "post design, 1.0 m takeoff", "post_raw"),
 "pl_38": ("raw", "post design", "post_raw"), "pl_39": ("raw", "post design", "post_raw"),
 "pl_40": ("fused", "post design (approach at grasp height)", "post_fused"),
 "pl_41": ("fused", "post design (approach at grasp height)", "post_fused"),
 "pl_42": ("raw", "post design + gripper contact switch", "post_raw_grip"),
 "pl_45": ("fused", "post design", "post_fused"), "pl_46": ("fused", "post design", "post_fused"),
 "pl_47": ("fused", "post design + gripper contact switch", "post_fused_grip"),
 "pl_48": ("fused", "post design + gripper contact switch", "post_fused_grip"),
 "pl_49": ("raw", "post design + gripper contact switch", "post_raw_grip"),
 "pl_50": ("raw", "post, 70 deg fold + gripper switch", "post_raw_b70"),
 "pl_51": ("raw", "post, 70 deg fold + gripper switch", "post_raw_b70"),
 # 2026-10-06: the T650 landing gear (AM_T650.usda, planner gear depth 0.2425), gripper switch
 "pl_52": ("raw", "T650 gear, push, 70 deg", "t650_push70"), "pl_56": ("raw", "T650 gear, push, 70 deg", "t650_push70"),
 "pl_54": ("raw", "T650 gear, push, 80 deg", "t650_push80"), "pl_55": ("raw", "T650 gear, push, 80 deg", "t650_push80"),
 "pl_57": ("raw", "T650 gear, push, 70 deg, deep grasp", "t650_push70_deep"),
 "pl_58": ("raw", "T650 gear, push, 80 deg, deep grasp", "t650_push80_deep"),
 # 2026-10-06: the real arm's q2 range [-20, 45] deg authored as hard stops in the scene
 "pl_62": ("raw", "q2 stops, push, beta 50 [0,26,24,0]", "q2lim_push50"),
 "pl_63": ("raw", "q2 stops, push, beta 40 [0,22,18,0]", "q2lim_push40"),
 "pl_64": ("raw", "q2 stops, PULL, beta 50", "q2lim_pull50"),
 "pl_65": ("raw", "q2 stops, PULL, beta 40", "q2lim_pull40"),
 "pl_66": ("raw", "q2 stops, push, beta 70 [0,30,40,0]", "q2lim_push70"),
 # + the translational observer rows held in contact (wb_l1_contact_hold_translation)
 "pl_67": ("raw", "q2 stops + hold, push, beta 50", "hold_push50"),
 # the translational observer slowed (wb_l1_omega_c_t 2.927 -> 1.0)
 "pl_71": ("raw", "q2 stops + omega_c_t 1.0, push, beta 50", "slowobs_push50"),
 "pl_72": ("raw", "q2 stops + omega_c_t 1.0, PULL, beta 50", "slowobs_pull50"),
 # pull at the 70 deg pose with the asset's stops (q2 <= 50), as the earlier pushes flew
 "pl_73": ("raw", "asset stops, PULL, beta 70", "asset_pull70"),
 "pl_74": ("raw", "q2 stops + omega_c_t 1.0, push, beta 50", "slowobs_push50"),
 "pl_75": ("raw", "q2 stops + omega_c_t 1.0, PULL, beta 50", "slowobs_pull50"),
 "pl_76": ("raw", "q2 stops, PULL, beta 70", "q2lim_pull70"),
}
EXTRA = {}          # tags added later by the campaign (filled below if their npz exists)
YAML_DEFAULT = PM.YAML
out = []
for tag, (fb, label, grp) in CFG.items():
    if not os.path.exists(os.path.join(RUNS, tag + ".npz")):
        continue
    z = np.load(os.path.join(RUNS, tag + ".npz"), allow_pickle=True)
    reason = str(z["reason"]); aborted = bool(z["aborted"])
    o, b = z["odom"], z["box"]
    k = STEPS.index("push"); po = o[o[:, -1] == k]; pb = b[b[:, -1] == k]
    r = dict(tag=tag, feedback=fb, label=label, group=grp, aborted=aborted, reason=reason[:120])
    if len(pb) > 1:
        r["box_mm"] = float(np.linalg.norm(pb[-1, 1:3] - pb[0, 1:3]) * 1e3)
        r["box_yaw_deg"] = float(yaw_deg(*pb[-1, 4:8]) - yaw_deg(*pb[0, 4:8]))
    if len(po) > 1:
        r["push_tilt_deg"] = float(tilt_deg(po[:, 7], po[:, 8], po[:, 9], po[:, 10]).max())
    # q2 over the DIRECT mission (Isaac joint truth, model convention) against the user's [-20, 45] deg
    jt = z["joints"]
    if jt.size > 1:
        live = jt[jt[:, -1] >= 0]
        if len(live):
            q2 = np.degrees(live[:, 2])
            r["q2_range_deg"] = [float(q2.min()), float(q2.max())]
    # the joints over the push step (grip -> push end, or -> just before the tilt first
    # passes 8 deg in a run that diverged): range and the time on a stop
    ref = z["ref"]
    tps0, tpe0 = mark(z, "push:start"), mark(z, "push:end")
    if tps0 is not None and tpe0 is None:
        ends = []
        oo = o[o[:, 0] > tps0]
        big = oo[tilt_deg(oo[:, 7], oo[:, 8], oo[:, 9], oo[:, 10]) > 8.0] if len(oo) else oo
        if len(big):
            ends.append(float(big[0, 0]))
        for e in z["events"]:                      # "[  54.86s] push_pull: ABORTING (...)"
            e = str(e)
            if "push_pull: ABORTING" in e:
                try:
                    ends.append(float(e[1:e.index("s]")]))
                except ValueError:
                    pass
                break
        ends = [e for e in ends if e > tps0]
        if ends:
            r["failed_after_s"] = min(ends) - tps0
            tpe0 = min(ends) - 0.25
            bb = b[(b[:, 0] >= tps0) & (b[:, 0] <= min(ends))]
            if len(bb) > 1:                           # box travel up to the failure (not the fling after)
                r["box_mm_at_fail"] = float(np.linalg.norm(bb[-1, 1:3] - bb[0, 1:3]) * 1e3)
    if jt.size > 1 and tps0 is not None and tpe0 is not None:
        m = (jt[:, 0] > tps0) & (jt[:, 0] < tpe0)
        if m.sum() > 10:
            qq = np.degrees(jt[m, 1:5])
            r["push_q2_deg"] = [float(qq[:, 1].min()), float(qq[:, 1].max())]
            r["push_q3_deg"] = [float(qq[:, 2].min()), float(qq[:, 2].max())]
            r["push_q3_stop_pct"] = float(np.mean(qq[:, 2] > 49.5) * 100)
            r["push_q2_over45_pct"] = float(np.mean(qq[:, 1] > 44.5) * 100)
            r["push_q2_std_deg"] = float(qq[:, 1].std())
    # the base against its plan over the push step (the motion the world-held claw makes the arm absorb)
    bt = z["base_truth"] if "base_truth" in z.files else np.zeros(0)
    if bt.size > 1 and ref.size > 1 and tps0 is not None:
        t_end = tpe0 if tpe0 is not None else min(tps0 + 8.0, bt[-1, 0])
        t_end = min(t_end, bt[-1, 0], ref[-1, 0])
        xb = np.array([rw[4:7] - PM.Rz(np.arctan2(rw[12], rw[11])) @ PM.TP.arm_fk_model(rw[7:11], PM.P)[0]
                       for rw in ref])
        tu = np.arange(tps0, t_end, 0.01)
        if len(tu) > 100:
            eb = PM.interp(tu, bt, (1, 2, 3)) - np.column_stack([np.interp(tu, ref[:, 0], xb[:, i]) for i in range(3)])
            ey = eb[:, 1] - eb[:, 1].mean()
            f = np.fft.rfftfreq(len(ey), 0.01)
            pw = np.abs(np.fft.rfft(ey * np.hanning(len(ey))))
            r["push_base_std_mm"] = [float(eb[:, i].std() * 1e3) for i in range(3)]
            r["push_base_f_hz"] = float(f[np.argmax(pw[2:]) + 2])
    # the EE reference during the slide: its height span (0 = parallel to the table) and travel
    tps, tpe = mark(z, "push:start"), mark(z, "push:end")
    if ref.size > 1 and tps is not None and tpe is not None:
        m = (ref[:, 0] > tps + 1.5) & (ref[:, 0] < tpe)
        if m.sum() > 5:
            r["ref_z_span_mm"] = float(np.ptp(ref[m, 3]) * 1e3)
            r["ref_travel_mm"] = float(np.linalg.norm(ref[m][-1, 1:3] - ref[m][0, 1:3]) * 1e3)
    # where it failed
    if aborted:
        for st in ["go_to_start", "ready", "close", "push", "release", "exit"]:
            if ("while " + st) in reason or ("while %s " % st) in reason: r["failed_in"] = st
        if "push complete" in reason: r["failed_in"] = "push"
        if "exit response" in reason or "while exit" in reason: r["failed_in"] = "release/exit"
        if "go_to_start" in reason: r["failed_in"] = "Go To Start (free flight)"
        if "disturbed" in reason: r["failed_in"] = "approach knocked the box"
        if "INFEASIBLE" in reason: r["failed_in"] = "plan refused (gear vs table)"
        if "SAFETY GUARD" in reason: r["failed_in"] = "push (force guard)"
    if "contact" in z.files and z["contact"].size > 1:
        y = os.path.join(RUNS, tag + ".yaml")
        g = PM.yaml_gains(y if os.path.exists(y) else YAML_DEFAULT)
        try:
            a = PM.analyse(os.path.join(RUNS, tag + ".npz"), g, 5.0)
            for w, v in a["imp"].items():
                r[w] = dict(e_xyz_rms=v["e_xyz"][0], e_xyz_pk=v["e_xyz"][1], e_psi_rms=v["e_psi"][0],
                            F_rms=v["F_xyz"][0], ey_rms_mm=v["ey_mm"][0], ey_pk_mm=v["ey_mm"][1],
                            e_comp=[float(x) for x in v["e_comp"]])
            r["eps"] = {w: dict(max_mm=mx * 1e3, rms_mm=rm * 1e3, rho_max=mx / PM.L) for w, (mx, rm) in a["eps"].items()}
            # the contact force's direction in the steady slide (true F_ext on the claw)
            ser = a["series"]; tt = ser["t"]
            t_push = PM.mark(z, "push:start"); t_end = PM.mark(z, "push:end")
            if t_push is not None and t_end is not None:
                m = (tt > t_push + 1.5 + 1.0) & (tt < t_end - 1.0)
                if m.sum() > 10:
                    Fm = ser["F"][m, :3].mean(0)
                    r["slide_force"] = [float(x) for x in Fm]
                    r["force_angle_deg"] = float(np.degrees(np.arctan2(Fm[2], np.hypot(Fm[0], Fm[1]))))
                    # net PITCH moment of that force about the system CoM, from the
                    # run's push pose on the planner's model (claw ahead d_f, below d_z)
                    import re as _re
                    if os.path.exists(y):
                        ytxt = open(y).read()
                        qd = [float(v) for v in _re.search(r"push_pull_push_pose_deg:\s*\[([^\]]*)\]", ytxt).group(1).split(",")]
                    else:                                  # runs before per-run yaml copies: the side-fin pose
                        qd = [0.0, -20.0, 30.0, 0.0]
                    r0c, r0e, _ = PM.TP.arm_fk_model(np.radians(qd), PM.P)
                    dfw, dz = (r0e - r0c)[1], (r0e - r0c)[2]
                    yaw = np.arctan2(*(np.array([np.interp(0.5 * (t_push + t_end), z["odom"][:, 0], z["odom"][:, 7 + i]) for i in range(4)])[[0, 3]][::-1])) * 2
                    nose = np.array([np.cos(yaw), np.sin(yaw), 0.0])
                    F_fwd = float(Fm @ nose)
                    r["push_pose_deg"] = qd
                    r["pitch_moment_Nm"] = float(dfw * Fm[2] - dz * F_fwd)
                    r["pitch_from_push_Nm"] = float(-dz * F_fwd)
        except Exception as exc:
            r["metrics_error"] = str(exc)
    out.append(r)
json.dump(dict(L=PM.L, runs=out), sys.stdout, indent=1)

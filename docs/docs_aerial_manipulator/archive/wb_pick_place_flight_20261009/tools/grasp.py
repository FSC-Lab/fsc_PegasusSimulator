"""Pick and place geometry from the mocap bodies (uav_0, obj_0) and the planner's own claw estimate.

claw (mocap)   = planner FK on the vehicle's MOCAP pose + measured joints: same frame as obj_0
claw (planner) = planner FK on the fused odometry: what the planner / arm GS gate on
The basket's solved orientation flips by 180 deg when the claw is near it, so at the pick the basket's REST pose
(mean of the clean samples before the approach) is the reference frame; in the carry the orientation is de-flipped
by continuity.
Writes ../analysis/grasp.json.
"""
import json
from pp_common import *
from scipy.spatial.transform import Rotation as Rot
import grasp_geom as GG


def deflip(Qb):
    """Continuous basket orientation: where the solver jumps by ~180 deg about the body z axis, undo it."""
    R = Rot.from_quat(Qb[:, [1, 2, 3, 0]]).as_matrix()
    F = np.diag([-1.0, -1.0, 1.0])
    out = np.zeros_like(R); ok = np.ones(len(R), bool)
    prev = R[0] if abs(np.degrees(np.arctan2(R[0][1, 0], R[0][0, 0]))) < 90 else R[0] @ F
    for k in range(len(R)):
        cand = [R[k], R[k] @ F]
        ang = [np.degrees(np.arccos(np.clip((np.trace(prev.T @ c) - 1) / 2, -1, 1))) for c in cand]
        j = int(np.argmin(ang))
        if ang[j] > 25.0:
            ok[k] = False; out[k] = prev; continue
        out[k] = cand[j]; prev = cand[j]
    return out, ok


def run(nm):
    S = GG.series(nm); d, t0, t = S["d"], S["t0"], S["t"]
    ph = pp_phases(d, t0); res = dict(run=RUNS[nm][0])
    span = lambda key: next(((a, b) for a, b, l in ph if l.startswith(key)), None)
    # --- basket rest pose on the pick platform: clean samples before the approach
    tb, Pb, Qb, _ = mocap_pose(d, t0, "obj")
    yawb = np.degrees(yaw_of(Qb)); clean = (np.abs(yawb) < 90) & (tilt_of(Qb) < 2.0)
    ap = span("FLYING execute_pick (approach)")
    m0 = clean & (tb > 2.0) & (tb < ap[0])
    p_rest = Pb[m0].mean(0); yaw_rest = np.radians(yawb[m0].mean())
    res["pick_rest"] = dict(p=p_rest.round(4).tolist(), yaw_deg=float(np.degrees(yaw_rest)), std_mm=(1e3 * Pb[m0].std(0)).round(2).tolist())
    I = d["pp_info__data"]; k = np.argmax(I[:, I_PLANNED] > 0.5); info = I[k]
    cap = info[I_CAP + 4:I_CAP + 7]; res["capture"] = dict(p=cap.round(4).tolist(), yaw_deg=float(info[I_PICKYAW]))
    res["capture_minus_rest_mm"] = (1e3 * (cap - p_rest)).round(1).tolist()
    pick_goal = info[I_GOAL + 7 + 4:I_GOAL + 7 + 7]; place_goal = info[I_GOAL + 21 + 4:I_GOAL + 21 + 7]
    cy, sy = np.cos(np.radians(info[I_PICKYAW])), np.sin(np.radians(info[I_PICKYAW]))
    dxy = pick_goal - cap
    off = np.array([cy * dxy[0] + sy * dxy[1], -sy * dxy[0] + cy * dxy[1], dxy[2]])
    res["ee_offset"] = off.round(3).tolist(); res["pick_goal"] = pick_goal.round(3).tolist(); res["place_goal"] = place_goal.round(3).tolist()
    res["place_point"] = (place_goal - np.array([-off[0], -off[1], off[2]])).round(3).tolist()   # yaw -180: Rz(-180) offset
    c, s = np.cos(yaw_rest), np.sin(yaw_rest)
    def rel(re):       # claw in the basket's rest frame [mm]
        dd = re - p_rest
        return 1e3 * np.column_stack([c * dd[:, 0] + s * dd[:, 1], -s * dd[:, 0] + c * dd[:, 1], dd[:, 2]])
    rm, ro = rel(S["re_m"]), rel(S["re_o"])
    # --- pick events
    lg = rosout(d, t0, contains="gripper CLOSE"); t_close_cmd = lg[0][0] if lg else None
    done = span("DONE execute_pick"); ex = span("FLYING exit_pick"); wait = span("WAITING execute_pick"); desc = span("FLYING execute_pick (descent)")
    g = S["g"][:, 0]
    res["pick"] = dict(wait_s=wait[1] - wait[0], on_handle_s=(ex[0] - desc[1]), t_close_cmd=t_close_cmd, t_exit=ex[0])
    def stats(m, tag):
        return {tag + "_mocap": dict(mean=rm[m].mean(0).round(1).tolist(), std=rm[m].std(0).round(1).tolist(), min=rm[m].min(0).round(1).tolist(), max=rm[m].max(0).round(1).tolist()),
                tag + "_planner": dict(mean=ro[m].mean(0).round(1).tolist(), std=ro[m].std(0).round(1).tolist(), min=ro[m].min(0).round(1).tolist(), max=ro[m].max(0).round(1).tolist())}
    res["pick"].update(stats((t >= wait[0]) & (t < wait[1]), "ready_hold"))
    res["pick"].update(stats((t >= desc[1]) & (t < ex[0]), "on_handle"))
    i = np.searchsorted(t, t_close_cmd)
    res["pick"]["at_close_mocap"] = rm[i].round(1).tolist(); res["pick"]["at_close_planner"] = ro[i].round(1).tolist()
    j = np.searchsorted(t, ex[0]); res["pick"]["at_exit_mocap"] = rm[j].round(1).tolist(); res["pick"]["at_exit_planner"] = ro[j].round(1).tolist()
    # --- lift-off: the basket leaves its rest height
    zb = S["pb"][:, 2]; after = (t >= ex[0]) & (t < ex[0] + 8.0)
    up = np.where(after & (zb > p_rest[2] + 0.003))[0]
    lifted = len(up) > 20 and zb[min(up[0] + 100, len(zb) - 1)] > p_rest[2] + 0.05
    res["pick"]["lifted"] = bool(lifted)
    if lifted:
        k0 = up[0]
        res["pick"].update(t_liftoff=float(t[k0]), claw_z_at_liftoff_mocap=float(rm[k0, 2]), claw_z_at_liftoff_planner=float(ro[k0, 2]),
                           claw_xy_at_liftoff_mocap=rm[k0, :2].round(1).tolist())
        # hang: claw relative to the basket while carried (the hold after the exit), basket tilt
        hd = span("DONE exit_pick"); mh = (t >= hd[0]) & (t < hd[1])
        Rb, okb = deflip(interp(t, tb, Qb) / np.linalg.norm(interp(t, tb, Qb), axis=1)[:, None])
        dm = S["re_m"] - S["pb"]; do = S["re_o"] - S["pb"]
        hang_b = 1e3 * np.einsum("nji,nj->ni", Rb, dm)          # claw in the basket's own (tilted) frame
        Ra = Rot.from_quat(S["Qv"][:, [1, 2, 3, 0]]).as_matrix()
        tiltv = np.degrees(np.arccos(np.clip(Rb[:, 2, 2], -1, 1)))
        # tilt direction in the basket frame: world z seen from the basket
        zb_in_b = np.einsum("nji,j->ni", Rb, np.array([0, 0, 1.0]))
        roll_b = np.degrees(np.arctan2(zb_in_b[:, 1], zb_in_b[:, 2])); pitch_b = np.degrees(np.arctan2(-zb_in_b[:, 0], zb_in_b[:, 2]))
        mm = mh & okb
        res["hang"] = dict(window=[hd[0], hd[1]], claw_minus_basket_world_mm=(1e3 * dm[mm].mean(0)).round(1).tolist(),
                           claw_minus_basket_world_planner_mm=(1e3 * do[mm].mean(0)).round(1).tolist(),
                           claw_in_basket_frame_mm=hang_b[mm].mean(0).round(1).tolist(), claw_in_basket_frame_std=hang_b[mm].std(0).round(1).tolist(),
                           basket_tilt_deg=float(tiltv[mm].mean()), tilt_about_basket_x_deg=float(roll_b[mm].mean()), tilt_about_basket_y_deg=float(pitch_b[mm].mean()),
                           tilt_std=float(tiltv[mm].std()))
        # whole carry
        ps = span("FLYING go_to_place_start"); mc = (t >= hd[0]) & (t < ps[1]) & okb
        res["carry"] = dict(window=[hd[0], ps[1]], claw_minus_basket_z_mm=[float(1e3 * dm[mc, 2].mean()), float(1e3 * dm[mc, 2].std())],
                            claw_minus_basket_z_planner_mm=[float(1e3 * do[mc, 2].mean()), float(1e3 * do[mc, 2].std())],
                            claw_in_basket_frame_mm=hang_b[mc].mean(0).round(1).tolist(), tilt_mean=float(tiltv[mc].mean()), tilt_max=float(tiltv[mc].max()),
                            tilt_x=float(roll_b[mc].mean()), tilt_y=float(pitch_b[mc].mean()))
        res["_series"] = dict(t=t, rm=rm, ro=ro, zb=zb, tilt=tiltv, hang_b=hang_b, okb=okb, g=g, pb=S["pb"], re_m=S["re_m"], re_o=S["re_o"])
    else:
        res["_series"] = dict(t=t, rm=rm, ro=ro, zb=zb, g=g, pb=S["pb"], re_m=S["re_m"], re_o=S["re_o"])
    return res, S, ph


if __name__ == "__main__":
    out = {}
    for nm in ("p2", "p3"):
        r, S, ph = run(nm); r.pop("_series"); out[nm] = r
        print("=====", nm); print(json.dumps(r, indent=1))
    json.dump(out, open(os.path.join(OUT, "grasp.json"), "w"), indent=1)

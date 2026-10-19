"""How the basket hangs under the claw, seen from the side: PP-2 (10-09) against PP-4 (10-10).

frame      the vehicle's heading frame with the claw's grasp point at the origin: a = along the heading (+ = ahead of
           the claw, away from the vehicle), l = to the left, u = up. Vehicle pose and basket pose both from mocap, the
           claw by the planner's kinematics on the vehicle's mocap pose and the measured joints.
basket     position of its mocap body origin; orientation = its body z axis in that frame, after taking out the
           body's own offset (its orientation while it stood on the pick hat, assumed level):
           tilt_side = atan2(z_a, z_u) (+ = top forward, far edge down), tilt_roll = atan2(z_l, z_u)
claw       direction of the fingers = the wrist's -z axis (the planner's EE x axis), pitch below the horizontal
arch       where the hanger's arch contact would be for that pose: centre + R(tilt_side) * (-27.5 mm, h_contact),
           h_contact = the claw height above the basket origin at which the basket left the pick hat that day
Writes ../analysis/fig_hang.json and prints the table.
"""
import json
from pp10_common import *
import grasp_geom as GG
from scipy.spatial.transform import Rotation as Rot

STEM_X = -27.5                      # hanger plane behind the basket's centre [mm], CAD
STAGES = [("after_lift", "Hover after the lift"), ("place_start", "Hover at the place start"), ("above_place", "Above the place"), ("descent", "Descent, until the touchdown")]
DRAWN = ["after_lift", "above_place", "descent"]      # the side view: the three stages flown with the arm straight ahead (the carry pose yaws it 12 deg)
WIN = {"p2": dict(after_lift=(49.6, 53.2), place_start=(69.5, 73.5), above_place=(80.8, 82.4), descent=(86.0, 89.3), rest=(2.0, 24.0)),
       "p4": dict(after_lift=(33.9, 34.3), place_start=(50.5, 52.5), above_place=(62.6, 65.6), descent=(71.2, 72.6), rest=(2.0, 16.0))}
CONTACT = {"p2": 163.0, "p4": 170.8}           # claw height above the basket origin at lift-off, mocap [mm]
Rz = lambda a: np.array([[np.cos(a), -np.sin(a), 0.0], [np.sin(a), np.cos(a), 0.0], [0.0, 0.0, 1.0]])


def run(nm):
    S = GG.series(nm); d, t0, t = S["d"], S["t0"], S["t"]
    tb, Pb, Qb, _ = mocap_pose(d, t0, "obj")
    Rb_all = Rot.from_quat(Qb[:, [1, 2, 3, 0]]).as_matrix(); yawb = yaw_of(Qb)
    tm, Pm, Qm, _ = mocap_pose(d, t0, "mocap"); yawv_b = np.interp(tb, tm, np.unwrap(yaw_of(Qm)))
    # the body's own offset: its orientation at rest on the pick hat, in its own yaw frame (level hat assumed)
    a, b = WIN[nm]["rest"]; m = (tb > a) & (tb < b) & (np.abs(np.degrees(yawb)) < 90)
    zb_rest = np.array([Rz(-yawb[k]) @ Rb_all[k][:, 2] for k in np.where(m)[0][::10]]).mean(0)
    rest_tilt = float(np.degrees(np.arccos(zb_rest[2] / np.linalg.norm(zb_rest))))
    # R_off = Rz(-yaw) R_rest: a small tilt; z_true(body frame) = R_off^T applied -> world z of the level basket
    k0 = np.where(m)[0][len(np.where(m)[0]) // 2]; R_off = Rz(-yawb[k0]) @ Rb_all[k0]
    # the claw's pitch: fingers = wrist -z (planner EE x), from the model FK on the mocap attitude
    out = dict(run=RUNS[nm][0], rest_tilt=rest_tilt, contact=CONTACT[nm], stages={})
    yawv = yaw_of(S["Qv"])
    for key, lab in STAGES:
        a, b = WIN[nm][key]; m = (t >= a) & (t <= b); mb = (tb >= a) & (tb <= b)
        rel_yaw = np.degrees(wrap(yawb[mb] - yawv_b[mb])); good = np.abs(rel_yaw) < 60        # unflipped orientation samples
        idx = np.where(mb)[0][good]
        za = []; zr = []
        for k in idx[::3]:
            Rh = Rz(-yawv_b[k])                                   # world -> vehicle heading frame
            za.append(Rh @ Rb_all[k][:, 2]); zr.append(Rh @ (Rb_all[k] @ R_off.T)[:, 2])
        za, zr = np.array(za), np.array(zr)
        side = lambda Z: float(np.degrees(np.arctan2(Z[:, 0], Z[:, 2])).mean()); roll = lambda Z: float(np.degrees(np.arctan2(Z[:, 1], Z[:, 2])).mean())
        tot = lambda Z: float(np.degrees(np.arccos(np.clip(Z[:, 2], -1, 1))).mean())
        c, s = np.cos(yawv[m]), np.sin(yawv[m]); dd = S["pb"][m] - S["re_m"][m]
        al = 1e3 * (c * dd[:, 0] + s * dd[:, 1]); lf = 1e3 * (-s * dd[:, 0] + c * dd[:, 1]); up = 1e3 * dd[:, 2]
        # fingers' pitch
        pit = []
        for k in np.where(m)[0][::10]:
            Ra = quat_R(*S["Qv"][k]); Rm = Ra @ RZM90; _, _, Re = TP.arm_fk_model(S["q"][k], PARAMS)
            f = Rz(-yawv[k]) @ (Rm @ (-Re[:, 2])); pit.append(np.degrees(np.arctan2(-f[2], np.hypot(f[0], f[1]))))
        th = np.radians(side(zr)); h = CONTACT[nm]
        arch = [float(al.mean() + STEM_X * np.cos(th) + h * np.sin(th)), float(up.mean() - STEM_X * np.sin(th) + h * np.cos(th))]
        out["stages"][key] = dict(label=lab, window=[a, b], along=[float(al.mean()), float(al.std())], left=[float(lf.mean()), float(lf.std())],
                                  up=[float(up.mean()), float(up.std())], tilt_side=side(zr), tilt_roll=roll(zr), tilt_total=tot(zr),
                                  tilt_side_raw=side(za), tilt_roll_raw=roll(za), tilt_total_raw=tot(za), unflipped=float(good.mean()), n=int(len(idx)),
                                  claw_pitch=float(np.mean(pit)), arch=arch, q=np.degrees(S["q"][m].mean(0)).round(1).tolist())
    return out


if __name__ == "__main__":
    res = {nm: run(nm) for nm in ("p2", "p4")}
    for nm, r in res.items():
        print(f"== {r['run']}: body tilt at rest on the pick hat {r['rest_tilt']:.1f} deg, arch contact {r['contact']:.0f} mm above the origin")
        for key, s in r["stages"].items():
            print(f"   {s['label']:30s} centre along {s['along'][0]:+6.1f} +- {s['along'][1]:4.1f}  left {s['left'][0]:+6.1f}  up {s['up'][0]:+7.1f} | tilt side {s['tilt_side']:+5.1f} roll {s['tilt_roll']:+5.1f} total {s['tilt_total']:4.1f}"
                  f" (raw side {s['tilt_side_raw']:+5.1f} roll {s['tilt_roll_raw']:+5.1f} total {s['tilt_total_raw']:4.1f}) | claw pitch {s['claw_pitch']:4.1f} down | arch at ({s['arch'][0]:+5.1f}, {s['arch'][1]:+5.1f})"
                  f" | unflipped {100 * s['unflipped']:.0f} % of {s['n']} | q {s['q']}")
    json.dump(dict(runs=res, drawn=DRAWN, stem_x=STEM_X, box=[110.0, 65.0], assumed=dict(along=20.0, up=-160.0, contact=167.0)), open(os.path.join(OUT10, "fig_hang.json"), "w"), indent=1)
    for key in DRAWN:
        a, b = res["p2"]["stages"][key], res["p4"]["stages"][key]
        print(f"   10-10 minus 10-09, {a['label']:30s}: along {b['along'][0] - a['along'][0]:+5.1f} mm, tilt {b['tilt_side'] - a['tilt_side']:+4.1f} deg, arch {b['arch'][0] - a['arch'][0]:+5.1f} mm")
    print("wrote fig_hang.json")

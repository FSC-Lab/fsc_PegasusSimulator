"""Numbers behind the EE Offset / pick point / place point fine-tuning (2026-10-10 follow-up).

a) pick, across the stem: the claw's sideways error from the start of the slide-in to the gripper's close command
   (what a wider stem has to survive), planner frame (what the planner / GS see) and mocap frame (true geometry)
b) pick, height and depth: the claw relative to the basket's rest pose through the same span; the arch contact height
c) hang: the basket centre relative to the claw while carried, along the vehicle's heading (+ = ahead of the claw)
d) place, height: the claw's lowest height against the arch-contact height
Writes ../analysis/setpoint_tuning.json.
"""
import json
from pp_common import *
import grasp as G
import grasp_geom as GG

out = {}
# ---- a, b: the pick (PP-2 and PP-3 have the basket; PP-1 against its captured point)
for nm in ("p2", "p3"):
    r, S, ph = G.run(nm); s = r["_series"]; t = s["t"]
    span = lambda key: next(((a, b) for a, b, l in ph if l.startswith(key)), None)
    de = span("FLYING execute_pick (descent)"); tc = r["pick"]["t_close_cmd"]
    off = np.array(r["ee_offset"]) * 1e3
    res = {}
    for frame, X in (("planner", s["ro"]), ("mocap", s["rm"])):
        for tag, m in (("slide_in", (t >= de[0]) & (t <= de[1])), ("on_handle", (t > de[1]) & (t <= tc)), ("both", (t >= de[0]) & (t <= tc))):
            e = X[m] - off          # claw minus its target, in the basket's rest frame [mm]
            res[f"{frame}_{tag}"] = dict(along=[float(e[:, 0].mean()), float(e[:, 0].std()), float(e[:, 0].min()), float(e[:, 0].max())],
                                         across=[float(e[:, 1].mean()), float(e[:, 1].std()), float(e[:, 1].min()), float(e[:, 1].max())],
                                         height=[float(e[:, 2].mean()), float(e[:, 2].std()), float(e[:, 2].min()), float(e[:, 2].max())],
                                         z_abs=[float(X[m][:, 2].mean()), float(X[m][:, 2].min()), float(X[m][:, 2].max())],
                                         x_abs=[float(X[m][:, 0].mean()), float(X[m][:, 0].min()), float(X[m][:, 0].max())],
                                         across_abs_p95=float(np.percentile(np.abs(e[:, 1]), 95)), across_abs_max=float(np.abs(e[:, 1]).max()))
    out[nm] = dict(ee_offset=off.tolist(), pick=res, liftoff_z=dict(planner=r["pick"].get("claw_z_at_liftoff_planner"), mocap=r["pick"].get("claw_z_at_liftoff_mocap")))
    print(f"== {RUNS[nm][0]} pick, EE Offset {off.round(0)} mm; arch contact (lift-off) at claw z = {out[nm]['liftoff_z']}")
    for k, v in res.items():
        print(f"   {k:18s} along {v['along'][0]:+6.1f} +- {v['along'][1]:4.1f} ({v['along'][2]:+.0f}..{v['along'][3]:+.0f}) | across {v['across'][0]:+6.1f} +- {v['across'][1]:4.1f} ({v['across'][2]:+.0f}..{v['across'][3]:+.0f}), |.| p95 {v['across_abs_p95']:.0f} max {v['across_abs_max']:.0f}"
              f" | height {v['height'][0]:+6.1f} +- {v['height'][1]:4.1f} ({v['height'][2]:+.0f}..{v['height'][3]:+.0f}); claw z abs {v['z_abs'][1]:.0f}..{v['z_abs'][2]:.0f}, x abs {v['x_abs'][1]:.0f}..{v['x_abs'][2]:.0f}")

# ---- c: the hang, PP-2
S = GG.series("p2"); t = S["t"]; d, t0 = S["d"], S["t0"]
yaw = yaw_of(S["Qv"]); c, s_ = np.cos(yaw), np.sin(yaw)
for frame, re in (("mocap", S["re_m"]), ("planner", S["re_o"])):
    dd = S["pb"] - re
    al = 1e3 * (c * dd[:, 0] + s_ * dd[:, 1]); lf = 1e3 * (-s_ * dd[:, 0] + c * dd[:, 1]); dz = 1e3 * dd[:, 2]
    hang = {}
    print(f"== PP-2 hang, basket centre minus claw in the vehicle's heading frame ({frame} claw): + along = basket ahead of the claw; the planner assumes +20, 0, -160")
    for a, b, lab in ((49.6, 53.2, "hover after the lift (yaw 0, pick pose)"), (69.5, 73.5, "hover at the place start (yaw -180, carry pose)"), (80.8, 82.4, "above the place (place pose)"),
                      (82.4, 86.0, "descent, first part"), (86.0, 89.3, "descent, until the touchdown")):
        m = (t >= a) & (t <= b)
        hang[lab] = dict(along=[float(al[m].mean()), float(al[m].std())], left=[float(lf[m].mean()), float(lf[m].std())], dz=[float(dz[m].mean()), float(dz[m].std())])
        print(f"   {lab:46s} along {al[m].mean():+6.1f} +- {al[m].std():4.1f}  left {lf[m].mean():+6.1f} +- {lf[m].std():4.1f}  dz {dz[m].mean():+7.1f} +- {dz[m].std():4.1f}")
    out[f"hang_{frame}"] = hang

# ---- d: the place bottom, PP-2 and PP-1 (claw height above the place point = typed z)
for nm in ("p2", "p1"):
    Sx = GG.series(nm) if nm != "p2" else S; tt = Sx["t"]; dx, tx0 = Sx["d"], Sx["t0"]
    I = dx["pp_info__data"]; info = I[np.argmax(I[:, I_PLANNED] > 0.5)]; goal = info[I_GOAL + 21 + 4:I_GOAL + 21 + 7]
    ph = pp_phases(dx, tx0); de = next((a, b) for a, b, l in ph if l.startswith("FLYING execute_place (descent)"))
    m = (tt >= de[1] - 1.0) & (tt <= de[1] + 0.4)
    zo, zm = Sx["re_o"][m][:, 2], Sx["re_m"][m][:, 2]
    out[f"place_bottom_{nm}"] = dict(goal_z=float(goal[2]), claw_z_planner=[float(zo.mean()), float(zo.min())], claw_z_mocap=[float(zm.mean()), float(zm.min())])
    print(f"== {RUNS[nm][0]} place bottom: claw target z {goal[2]:.3f}; claw z planner {zo.mean():.3f} (min {zo.min():.3f}), mocap {zm.mean():.3f} (min {zm.min():.3f})")
json.dump(out, open(os.path.join(OUT, "setpoint_tuning.json"), "w"), indent=1)

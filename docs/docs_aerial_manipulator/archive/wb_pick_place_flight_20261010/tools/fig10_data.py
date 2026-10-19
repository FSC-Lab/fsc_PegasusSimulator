"""Figure data for PP-4 (2026-10-10) -> ../analysis/fig_p4.json, in the layouts the 10-09 report's figures.js reads
(3d, err, views) plus three new ones (trim, setdown, carry)."""
import json
from pp10_common import *
from scipy.spatial.transform import Rotation as Rot
import grasp_geom as GG

nm = "p4"
A = json.load(open(os.path.join(OUT10, "pp4.json")))
SHORT = {"FLYING go_to_start": "to start", "FLYING execute_pick (approach)": "to pick", "WAITING execute_pick": "wait",
         "FLYING execute_pick (descent)": "slide in", "FLYING exit_pick (up the vertical margin)": "lift",
         "FLYING go_to_place_start": "to place start", "FLYING execute_place (approach)": "to place",
         "WAITING execute_place": "wait", "FLYING execute_place (descent)": "descend",
         "FLYING exit_place (up the vertical margin)": "exit"}
r3 = lambda a, n=3: np.round(np.asarray(a, float), n).tolist()
r1 = lambda a: np.round(np.asarray(a, float), 1).tolist()
ds = lambda t, X, tq: interp(tq, t, X)

d, t0 = load(nm); a, b = direct_span(d, t0)
S = GG.series(nm); t = S["t"]
tr, ref = wbref(d, t0)
ph_all = pp_phases(d, t0); span = lambda key: next(((x, y) for x, y, l in ph_all if l.startswith(key)), None)
out = {}

# ---------------- 3-D and the error time series (same layout as fig_3d.json / fig_err.json)
tq = np.arange(a, b, 0.1)
ph = [(x, y, l) for x, y, l in ph_all if y > a and x < b]
e = dict(t=r3(tq, 2), ref=r3(ds(tr, ref["r_ed"], tq)), claw=r3(ds(t, S["re_m"], tq)), base=r3(ds(t, S["Pm"], tq)),
         phases=[[round(x, 2), round(y, 2), SHORT.get(l, "")] for x, y, l in ph if l in SHORT])
tbq = np.arange(max(a, t[0]), 74.7, 0.1)
e["basket"] = r3(ds(t, S["pb"], tbq)); e["tb"] = r3(tbq, 2)
tm, Pm, Qm, Vm = mocap_pose(d, t0, "mocap"); tf = np.arange(A["takeover"]["t_stab"], 78.2, 0.05)
e["flyaway"] = r3(ds(tm, Pm, tf)); e["tf"] = r3(tf, 2); e["fly_name"] = "after the pilot took over"
st_ = A["settings"]
e["pick_goal"] = r3(st_["pick_goal"]); e["place_goal"] = r3(st_["place_goal"]); e["capture"] = r3(A["basket_rest"]["at_grasp"])
e["place_pillar"] = r3([A["place"]["hat_centre_assumed"][0], A["place"]["hat_centre_assumed"][1], A["place"]["basket_flat_on_hat"]["z"] - 0.0325 - 0.03])
out["3d"] = e
s = json.load(open(os.path.join(OUT10, f"series_{nm}.json"))); ts = np.array(s["t"])
g = lambda k: ds(ts, np.array(s[k]), tq)
out["err"] = dict(t=r3(tq, 2), e_com=r3(1e3 * g("e_com"), 1), e_task=r3(1e3 * g("e_task"), 1), e_head=r3(g("e_head"), 2), e_yaw=r3(g("e_yaw"), 2),
                  tilt=r3(g("tilt"), 2), u1=r3(g("u1"), 2), tau=r3(g("tau")[:, 1:3], 3), phases=e["phases"])

# ---------------- the pick and the place from above and from the side (fig_views.json layout)
BOX_HALF_H = 0.0325
def frame(o, yaw_deg, ztop):
    c, s_ = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    def f(P):
        P = np.atleast_2d(np.asarray(P, float)); dd = P - np.array([o[0], o[1], ztop])
        return 1e3 * np.column_stack([c * dd[:, 0] + s_ * dd[:, 1], -s_ * dd[:, 0] + c * dd[:, 1], dd[:, 2]])
    return f
tg, G = gripper(d, t0); gg = G[:, 0]
cross = lambda x, y, up: next((float(tg[k]) for k in np.where(np.diff((gg > 0.005).astype(int)) == (1 if up else -1))[0] if x <= tg[k] <= y), None)

def task(name, keys, o, yaw, goal, ztop, tail, t_end=None):
    a0 = span(keys[0]); w = span(keys[1]); de = span(keys[2]); ex = span(keys[3])
    f = frame(o, yaw, ztop); end = t_end if t_end else ex[1] + tail
    stg = [("ready", a0[0], w[1]), (name, de[0], ex[0]), ("exit", ex[0], end)]
    tq_ = np.arange(a0[0], end, 0.05)
    claw = f(interp(tq_, t, S["re_m"])); rf = f(interp(tq_, tr, ref["r_ed"])); keep = claw[:, 0] > -520
    ee = dict(t=np.round(tq_[keep], 2).tolist(), claw=r1(claw[keep]), ref=r1(rf[keep]), basket=r1(f(interp(tq_, t, S["pb"]))[keep]),
              stages=[[k, round(x, 2), round(y, 2)] for k, x, y in stg], origin=[round(float(v), 3) for v in o[:2]], yaw=round(yaw, 1),
              goal=r1(f(goal)[0]), z_world0=round(float(ztop), 4))
    return ee, f

rest = np.array(A["basket_rest"]["at_grasp"]); ztop_pick = rest[2] - BOX_HALF_H
ep, f = task("pick", ("FLYING execute_pick (approach)", "WAITING execute_pick", "FLYING execute_pick (descent)", "FLYING exit_pick"),
             rest, st_["pick_yaw"], np.array(st_["pick_goal"]), ztop_pick, 3.0)
de = span("FLYING execute_pick (descent)"); ex = span("FLYING exit_pick")
ep["ev"] = dict(pressed=de[0], close=cross(de[0], ex[0] + 1, False), liftoff=A["pick"]["t_liftoff"])
ep["rest"] = r1(f(rest)[0]); ep["capture"] = r1(f(np.array(st_["pick_point"]))[0]); ep["contact_h"] = round(A["pick"]["contact_height"]["mocap"] + 32.5, 1)
ep["rest_early"] = r1(f(np.array(A["basket_rest"]["early"]))[0]); ep["basket_yaw_rel"] = round(A["basket_rest"]["at_grasp_yaw"] - st_["pick_yaw"], 1)
place_pt = np.array(st_["place_point"]); hat = np.array(A["place"]["hat_centre_assumed"]); ztop_place = A["place"]["basket_flat_on_hat"]["z"] - BOX_HALF_H
el, f = task("place", ("FLYING execute_place (approach)", "WAITING execute_place", "FLYING execute_place (descent)", "FLYING exit_place"),
             place_pt, st_["place_yaw"], np.array(st_["place_goal"]), ztop_place, 0.0, t_end=A["takeover"]["t_stab"])
de = span("FLYING execute_place (descent)"); ex = span("FLYING exit_place")
el["ev"] = dict(pressed=de[0], touchdown=A["place"]["t_touchdown"], open=A["setdown"]["t_fingers_open"])
el["place_point"] = r1(f(place_pt)[0]); el["hat"] = r1(f(np.array([hat[0], hat[1], ztop_place]))[0][:2]); el["contact_h"] = ep["contact_h"]
el["basket_end"] = A["setdown"]["t_floor"]
out["views"] = dict(ee_offset=r1(1e3 * np.array(st_["ee_offset"])), hat_top_z=float(ztop_pick), payload=True, pick=ep, place=el)

# ---------------- the descent trim at the place: the claw along the approach, against its typed target
goal = np.array(st_["place_goal"]); cp, sp = np.cos(np.radians(st_["place_yaw"])), np.sin(np.radians(st_["place_yaw"]))
al = lambda P: 1e3 * ((P[:, 0] - goal[0]) * cp + (P[:, 1] - goal[1]) * sp)          # + = beyond the target, - = short of it (toward the vehicle)
ac = lambda P: 1e3 * (-(P[:, 0] - goal[0]) * sp + (P[:, 1] - goal[1]) * cp)
tq = np.arange(56.0, 73.4, 0.05)
claw_o = ds(t, S["re_o"], tq); rf = ds(tr, ref["r_ed"], tq)
w = span("WAITING execute_place")
out["trim"] = dict(t=r3(tq, 2), claw=r1(al(claw_o)), ref=r1(al(rf)), claw_across=r1(ac(claw_o)), ref_across=r1(ac(rf)),
                   ev=dict(arrive=w[0], press=w[1], touchdown=A["place"]["t_touchdown"]), window=[w[1] - 5.0, w[1]],
                   mean5=round(float(-A["place"]["mean_5s_mm"][0] * cp), 1), settled=round(float(-A["place"]["mean_settled_last2s_mm"][0] * cp), 1),
                   trim_along=round(float(A["settings"]["trim_mm"]["place"][0] * cp + A["settings"]["trim_mm"]["place"][1] * sp), 1))

# ---------------- the set-down
td = A["place"]["t_touchdown"]; tq = np.arange(71.4, 74.9, 0.02)
eul = Rot.from_quat(S["Qv"][:, [1, 2, 3, 0]]).as_euler("ZYX", degrees=True)
zt = ztop_place; contact = A["pick"]["contact_height"]["mocap"] / 1e3
sd = A["setdown"]
out["setdown"] = dict(t=r3(tq, 2), claw_h=r1(1e3 * (ds(t, S["re_m"][:, 2], tq) - zt)), ref_h=r1(1e3 * (ds(tr, ref["r_ed"][:, 2], tq) - zt)),
                      arch_h=r1(1e3 * (ds(t, S["pb"][:, 2], tq) + contact - zt)), basket_h=r1(1e3 * (ds(t, S["pb"][:, 2], tq) - BOX_HALF_H - zt)),
                      veh_back=r1(1e3 * (ds(t, S["Pm"][:, 0], tq) - sd["x_pre"])), basket_back=r1(1e3 * (ds(t, S["pb"][:, 0], tq) - A["place"]["basket_at_td"][0])),
                      pitch=r3(ds(t, eul[:, 1], tq) - sd["pitch_pre"], 2), grip=r3(np.clip((ds(tg, gg, tq) + 0.010) / 0.029, 0, 1), 3),
                      basket_tilt=r1(ds(t, S["tiltb"], tq)),
                      ev=dict(touchdown=td, vehicle=sd["t_vehicle_starts"], rim=sd["t_tip"], leg_end=sd["t_leg_end"], open_cmd=sd["t_open_cmd"], open=sd["t_fingers_open"], floor=sd["t_floor"]),
                      rim=round(float(np.sqrt(100.3 ** 2 - A["place"]["basket_minus_hat_at_td_mm"][1] ** 2) - A["place"]["basket_minus_hat_at_td_mm"][0]), 1))

# ---------------- the carry: joint 2 sticks and slips, torque at the cap
tw, D = wbdebug(d, t0); tj, qj, _ = joints_model(d, t0)
tq = np.arange(44.0, 60.0, 0.04)
s2 = json.load(open(os.path.join(OUT10, f"series_{nm}.json"))); ts2 = np.array(s2["t"])
out["carry"] = dict(t=r3(tq, 2), q2=r1(np.degrees(ds(tj, qj[:, 1], tq))), q3=r1(np.degrees(ds(tj, qj[:, 2], tq))), q4=r1(np.degrees(ds(tj, qj[:, 3], tq))),
                    q2_ref=r1(np.degrees(ds(tr, ref["q_d"][:, 1], tq))), q3_ref=r1(np.degrees(ds(tr, ref["q_d"][:, 2], tq))),
                    tau2=r3(ds(tw, D[:, WB["tau"]][:, 1], tq), 2), tau3=r3(ds(tw, D[:, WB["tau"]][:, 2], tq), 2), caps=[2.44, 1.42],
                    e_com=r1(1e3 * ds(ts2, np.array(s2["e_com"]), tq)[:, :2]), guard=45.0,
                    ev=dict(arrive=span("DONE go_to_place_start")[0], ready=span("FLYING execute_place (approach)")[0]))
# ---------------- the takeover and the fall (fig_land.json layout)
tk = A["takeover"]; ta = 74.0; tq = np.arange(ta, 80.0, 0.02)
em = Rot.from_quat(Qm[:, [1, 2, 3, 0]]).as_euler("ZYX", degrees=True); tje = d["js__recv"] - t0; eff = d["js__effort"][:, :4][:, JS_IDX] / KT
out["land"] = dict(t=r3(tq - ta, 2), t0=ta, z=r3(ds(tm, Pm[:, 2], tq)), pitch=r3(ds(tm, em[:, 1], tq), 1), roll=r3(ds(tm, em[:, 2], tq), 1),
                   tau2=r3(ds(tje, eff[:, 1], tq), 2), tau3=r3(ds(tje, eff[:, 2], tq), 2), t_stab=tk["t_stab"] - ta, t_disarm=tk["t_disarm"] - ta,
                   extra=[[tk["t_current_drop"] - ta, "motors stop"]])
json.dump(out, open(os.path.join(OUT10, "fig_p4.json"), "w"))
print("fig_p4.json", os.path.getsize(os.path.join(OUT10, "fig_p4.json")) // 1024, "kB")
for k in ("pick", "place"):
    e_ = out["views"][k]; c = np.array(e_["claw"]); bq = np.array(e_["basket"]); tt_ = np.array(e_["t"])
    print(f"  {k}: origin {e_['origin']} yaw {e_['yaw']} goal {e_['goal']} events {e_['ev']} hat {e_.get('hat')} contact_h {e_.get('contact_h')}")
    for key, tv in e_["ev"].items():
        i = int(np.argmin(np.abs(tt_ - tv))); print(f"     at {key} ({tv:.2f} s): claw {c[i]} basket {bq[i]}")
print("trim:", {k: v for k, v in out["trim"].items() if k not in ("t", "claw", "ref", "claw_across", "ref_across")})
print("setdown ev:", out["setdown"]["ev"], "rim", out["setdown"]["rim"])

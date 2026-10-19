"""PP-4 (2026-10-10 19:07): the pick, the hang, the loaded hover, the descent trim and the set-down, in numbers.

frames     planner = the planner's kinematics on the fused odometry (what the planner, the law and the GS gate on)
           mocap   = the same kinematics on the vehicle's mocap pose (the frame the basket obj_0 is measured in)
approach   pick: along the captured pick yaw (world +x here); place: along the place yaw (-180 deg: world -x)
Writes ../analysis/pp4.json.
"""
import json
from pp10_common import *
import grasp_geom as GG
import tracking as T
import hover_error as HE
from scipy.spatial.transform import Rotation as Rot

nm = "p4"
S = GG.series(nm); d, t0, t = S["d"], S["t0"], S["t"]; dt = t[1] - t[0]
ph = pp_phases(d, t0)
span = lambda key: next(((a, b) for a, b, l in ph if l.startswith(key)), None)
at = lambda ts: int(np.searchsorted(t, ts))
win = lambda a, b: (t >= a) & (t <= b)
R = {}
mm = lambda v, n=1: np.round(1e3 * np.asarray(v, float), n).tolist()

# ------------------------------------------------------------------ settings
I = d["pp_info__data"]; info = I[np.argmax(I[:, I_PLANNED] > 0.5)]
cap = info[I_CAP + 4:I_CAP + 7]; pick_yaw = float(info[I_PICKYAW])
pick_goal = info[I_GOAL + 7 + 4:I_GOAL + 7 + 7]; place_goal = info[I_GOAL + 21 + 4:I_GOAL + 21 + 7]; place_yaw = float(info[I_GOAL + 21 + 3])
cy, sy = np.cos(np.radians(pick_yaw)), np.sin(np.radians(pick_yaw)); dg = pick_goal - cap
off = np.array([cy * dg[0] + sy * dg[1], -sy * dg[0] + cy * dg[1], dg[2]])
cp, sp = np.cos(np.radians(place_yaw)), np.sin(np.radians(place_yaw))
place_pt = place_goal - np.array([cp * off[0] - sp * off[1], sp * off[0] + cp * off[1], off[2]])
w_pick = span("WAITING execute_pick"); w_place = span("WAITING execute_place")
de_pick = span("FLYING execute_pick (descent)"); de_place = span("FLYING execute_place (descent)")
ex_pick = span("FLYING exit_pick"); ex_place = span("FLYING exit_place")
trim = {}
for tt, n_, m_ in rosout(d, t0, contains="goal trimmed by"):
    if "whole_body" in n_ and "descending (goal" in m_:
        v = [float(x) for x in m_.split("trimmed by [")[1].split("]")[0].split(",")]
        trim["pick" if "execute_pick" in m_ else "place"] = v
R["settings"] = dict(pick_point=cap.round(4).tolist(), pick_yaw=pick_yaw, ee_offset=off.round(3).tolist(), pick_goal=pick_goal.round(4).tolist(),
                     place_point=place_pt.round(4).tolist(), place_goal=place_goal.round(4).tolist(), place_yaw=place_yaw,
                     start_xy=info[I_GOAL:I_GOAL + 2].round(3).tolist(), adjust=info[0:4].round(4).tolist(),
                     wait_pick=w_pick[1] - w_pick[0], wait_place=w_place[1] - w_place[0], trim_mm=trim, leg_T=info[I_T:I_T + 6].round(2).tolist())
print("settings:", json.dumps(R["settings"]))

# ------------------------------------------------------------------ the basket on the pick hat
tb, Pb, Qb, _ = mocap_pose(d, t0, "obj"); yawb = np.degrees(yaw_of(Qb)); tiltb = tilt_of(Qb)
tm, Pm, Qm, Vm = mocap_pose(d, t0, "mocap")
m_early = (tb > 2) & (tb < 16); m_grasp = (tb > 23.5) & (tb < de_pick[0])
rest0 = Pb[m_early].mean(0); rest = Pb[m_grasp].mean(0); yaw0 = yawb[m_early].mean(); yaw1 = yawb[m_grasp].mean()
k = np.where((tb > 16) & (tb < 24) & (yawb < 0.5 * (yaw0 + yaw1)))[0][0]; t_move = float(tb[k])
k0 = np.where((tb > 16) & (yawb < yaw0 - 0.5))[0][0]; k1 = np.where((tb > 16) & (yawb < yaw1 + 0.5))[0][0]
pv = interp(np.array([t_move]), tm, Pm)[0]
R["basket_rest"] = dict(early=rest0.round(4).tolist(), early_std_mm=mm(Pb[m_early].std(0), 2), early_yaw=float(yaw0), early_tilt=float(tiltb[m_early].mean()),
                        at_grasp=rest.round(4).tolist(), at_grasp_std_mm=mm(Pb[m_grasp].std(0), 2), at_grasp_yaw=float(yaw1), at_grasp_tilt=float(tiltb[m_grasp].mean()),
                        moved_mm=mm(rest - rest0), moved_xy_mm=float(1e3 * np.linalg.norm((rest - rest0)[:2])), yaw_change=float(yaw1 - yaw0),
                        t_move=[float(tb[k0]), float(tb[k1])], vehicle_at_move=pv.round(3).tolist(), horiz_dist_vehicle=float(np.linalg.norm(pv[:2] - rest0[:2])),
                        height_above_basket=float(pv[2] - rest0[2]),
                        pick_point_minus_early_mm=mm(cap - rest0), pick_point_minus_at_grasp_mm=mm(cap - rest))
print("basket rest:", json.dumps(R["basket_rest"]))

# ------------------------------------------------------------------ the pick, in the approach frame at the basket's pose at the grasp
def fr(o, yaw_deg):
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    return lambda P: 1e3 * np.column_stack([c * (P[:, 0] - o[0]) + s * (P[:, 1] - o[1]), -s * (P[:, 0] - o[0]) + c * (P[:, 1] - o[1]), P[:, 2] - o[2]])
fp = fr(rest, pick_yaw)
cm, co = fp(S["re_m"]), fp(S["re_o"])
tgt = fp(pick_goal[None, :])[0]                               # the planner's claw target in this frame (pick point was typed before the basket moved)
dyaw = np.radians(yaw1 - pick_yaw); stem = np.array([-27.5 * np.cos(dyaw), -27.5 * np.sin(dyaw)])   # hanger plane, 27.5 mm behind the centre in the CAD
lg = rosout(d, t0, contains="gripper CLOSE"); t_close = lg[0][0]
st = lambda X, m: dict(mean=X[m].mean(0).round(1).tolist(), std=X[m].std(0).round(1).tolist(), min=X[m].min(0).round(1).tolist(), max=X[m].max(0).round(1).tolist())
zb = S["pb"][:, 2]; up = np.where((t >= ex_pick[0]) & (t < ex_pick[0] + 8) & (zb > rest[2] + 0.003))[0]; kl = up[0]
contact = dict(mocap=float(cm[kl, 2]), planner=float(co[kl, 2]))
P = dict(target_in_frame=tgt.round(1).tolist(), stem_in_frame=stem.round(1).tolist(), t_close=t_close, t_exit=ex_pick[0], t_liftoff=float(t[kl]),
         on_handle_s=ex_pick[0] - de_pick[1], contact_height=contact)
for tag, m in (("wait", win(*w_pick)), ("slide_in", win(*de_pick)), ("on_handle", win(de_pick[1], t_close))):
    P[tag] = dict(mocap=st(cm, m), planner=st(co, m))
    for f_, X in (("mocap", cm), ("planner", co)):
        gap = contact[f_] - X[m][:, 2]
        P[tag][f_]["gap_under_arch"] = [float(gap.mean()), float(gap.min()), float((gap < 0).mean())]
P["at_close"] = dict(mocap=cm[at(t_close)].round(1).tolist(), planner=co[at(t_close)].round(1).tolist())
P["at_exit"] = dict(mocap=cm[at(ex_pick[0])].round(1).tolist(), planner=co[at(ex_pick[0])].round(1).tolist())
P["claw_vs_planner_target_at_close_mm"] = dict(planner=(co[at(t_close)] - tgt).round(1).tolist(), mocap=(cm[at(t_close)] - tgt).round(1).tolist())
P["claw_vs_stem_at_close_mm"] = dict(mocap=(cm[at(t_close)][:2] - stem).round(1).tolist(), planner=(co[at(t_close)][:2] - stem).round(1).tolist())
bk = fp(S["pb"])
P["basket_dragged_by_close_mm"] = (bk[at(ex_pick[0])] - bk[at(t_close)]).round(1).tolist()
# with other EE Offset z values: the same claw heights shifted
P["ee_z_what_if"] = {}
for dz in (-10, 0, 10):
    r_ = {}
    for f_, X in (("mocap", cm), ("planner", co)):
        ms, mw = win(*de_pick), win(de_pick[1], t_close)
        r_[f_] = dict(slide_gap_mean=float((contact[f_] - X[ms][:, 2] - dz).mean()), slide_gap_min=float((contact[f_] - X[ms][:, 2] - dz).min()),
                      slide_above=float(((contact[f_] - X[ms][:, 2] - dz) < 0).mean()), wait_gap_mean=float((contact[f_] - X[mw][:, 2] - dz).mean()),
                      wait_gap_min=float((contact[f_] - X[mw][:, 2] - dz).min()), wait_above=float(((contact[f_] - X[mw][:, 2] - dz) < 0).mean()))
    P["ee_z_what_if"][f"{off[2] + dz / 1e3:.2f}"] = r_
R["pick"] = P
print("pick:", json.dumps(P, indent=0)[:2600])

# ------------------------------------------------------------------ tracking series (planner frame), the lift
dT, _, TS = T.build(nm); tt = TS["t"]
wt = lambda a, b: (tt >= a) & (tt <= b)
ml = wt(ex_pick[0], ex_pick[1] + 0.5)
ec = TS["e_com"][ml]; k_ = np.argmax(np.linalg.norm(ec, axis=1))
tj, qj, _ = joints_model(d, t0); qd = np.degrees(interp(tj, *(lambda tr_, ref_: (tr_, ref_["q_d"]))(*wbref(d, t0))))
mj = (tj >= ex_pick[0]) & (tj <= ex_pick[1] + 0.5); eq = np.degrees(qj[mj]) - qd[mj]
bl = fp(S["pb"]); kk = at(t[kl] + 0.5)
R["lift"] = dict(com_err_max_mm=float(1e3 * np.linalg.norm(ec, axis=1).max()), com_err_at_max_mm=mm(ec[k_]), t_max=float(tt[ml][k_]), tilt_max=float(TS["tilt"][ml].max()),
                 q_err_min=eq.min(0).round(1).tolist(), q_err_max=eq.max(0).round(1).tolist(), q2_min=float(np.degrees(qj[mj][:, 1]).min()),
                 basket_at_14mm_up=bl[at(32.02)].round(0).tolist(),
                 u1_before=float(TS["u1"][wt(27, 30)].mean()), u1_after=float(TS["u1"][wt(60.6, 65.6)].mean()),
                 dz_before=float(TS["dhat"][wt(27, 30)][:, 2].mean()), dz_after=float(TS["dhat"][wt(60.6, 65.6)][:, 2].mean()))
print("lift:", json.dumps(R["lift"]))

# ------------------------------------------------------------------ how the basket hangs (vehicle heading frame)
yawv = yaw_of(S["Qv"]); c_, s_ = np.cos(yawv), np.sin(yawv)
hang = {}
for frame, re in (("mocap", S["re_m"]), ("planner", S["re_o"])):
    dd = S["pb"] - re
    al = 1e3 * (c_ * dd[:, 0] + s_ * dd[:, 1]); lf = 1e3 * (-s_ * dd[:, 0] + c_ * dd[:, 1]); dzv = 1e3 * dd[:, 2]
    hang[frame] = {}
    for a, b, lab in ((33.9, 34.3, "hover after the lift"), (50.5, 52.5, "hover at the place start"), (62.6, 65.6, "above the place, last 3 s before Place"),
                      (67.6, 71.2, "after the trim move, before the descent"), (71.2, 72.6, "descent, until the touchdown")):
        m = win(a, b)
        hang[frame][lab] = dict(along=[float(al[m].mean()), float(al[m].std())], left=[float(lf[m].mean()), float(lf[m].std())], dz=[float(dzv[m].mean()), float(dzv[m].std())],
                                tilt=[float(S["tiltb"][m].mean()), float(S["tiltb"][m].max())])
        if frame == "mocap":
            print(f"   hang {lab:42s} along {al[m].mean():+6.1f} +- {al[m].std():4.1f}  left {lf[m].mean():+6.1f} +- {lf[m].std():4.1f}  dz {dzv[m].mean():+7.1f}  tilt {S['tiltb'][m].mean():.1f}")
R["hang"] = hang
R["hang_tilt_carry"] = dict(mean=float(S["tiltb"][win(34.3, 72.6)].mean()), max=float(S["tiltb"][win(34.3, 72.6)].max()))

# ------------------------------------------------------------------ settled hover with / without the basket
HE.LOADED["p4"] = (float(t[kl]), 72.7); HE.EXCLUDE["p4"] = []
_, _, HS, W = HE.windows_of(nm); th = HS["t"]; rows = []; pool = {"none": [], "payload": []}
for a, b, l in W:
    cuts = sorted([a, b] + [x for x in HE.LOADED["p4"] if a < x < b])
    for i in range(len(cuts) - 1):
        ca, cb = cuts[i], cuts[i + 1]
        if cb > 74.0:
            continue
        stt = HE.load_state(nm, ca, cb); ms = (th >= ca + HE.SETTLE) & (th <= cb); ma = (th >= ca) & (th <= cb)
        if ma.sum() < 50:
            continue
        r = dict(t0=float(ca), t1=float(cb), state=stt, phase=l, all=dict(com=HE.stats(HS["e_com"][ma]), ee=HE.stats(HS["e_ee"][ma])))
        if ms.sum() >= 50:
            r["settled"] = dict(com=HE.stats(HS["e_com"][ms]), ee=HE.stats(HS["e_ee"][ms]))
            if stt in pool:
                pool[stt].append(HS["e_ee"][ms])
        rows.append(r); c = r.get("settled", r["all"])
        print(f"   hover {ca:6.1f}-{cb:6.1f} {stt:8s} {'settled' if 'settled' in r else 'all    '} claw bias {c['ee']['bias']:5.0f} scatter {c['ee']['scatter']:5.0f} rms {c['ee']['rms']:5.0f} p95 {c['ee']['p95']:5.0f} max {c['ee']['max']:5.0f} mean_xy {c['ee']['mean_xy']} | {l[:40]}")
summ = {}
for k_, v in pool.items():
    if v:
        E = np.vstack(v); s = HE.stats(E); dev = np.vstack([e[:, :2] - e[:, :2].mean(0) for e in v]); means = np.array([e[:, :2].mean(0) for e in v])
        s.update(wander=float(1e3 * np.sqrt((dev ** 2).sum(1).mean())), offset_rms=float(1e3 * np.sqrt((means ** 2).sum(1).mean())), seconds=float(len(E) * 0.02), windows=len(v))
        summ[k_] = s
        print(f"   pooled settled {k_}: {s['seconds']:.1f} s in {s['windows']} hovers, offset rms {s['offset_rms']:.0f}, wander {s['wander']:.0f}, total rms {s['rms']:.0f}, p95 {s['p95']:.0f}, max {s['max']:.0f}, within 20/50/70: {100*s['within20']:.0f}/{100*s['within50']:.0f}/{100*s['within70']:.0f}")
R["hover"] = dict(windows=rows, pooled=summ)

# ------------------------------------------------------------------ the place: the wait, the trim, the touchdown
e_ee = TS["e_ee"]                                             # claw minus the planner's claw reference, planner frame, world axes
mw = wt(w_place[0], w_place[1]); tw_ = tt[mw] - w_place[0]
tr5 = -e_ee[mw].mean(0)                                       # what a 5 s average of (target - claw) gives over the whole wait
sett = -e_ee[wt(w_place[0] + 3.0, w_place[1])].mean(0)        # the same over the settled part (from 3 s after arrival)
k_ = np.argmax(np.linalg.norm(e_ee[mw][:, :2], axis=1))
# touchdown: the basket stops descending
vzb = np.gradient(S["pb"][:, 2], t); kd = np.where((t > de_place[0] + 5.0) & (t < de_place[1]) & (vzb > -0.03) & (S["pb"][:, 2] < 0.95))[0][0]; t_td = float(t[kd])
i_td = at(t_td); j_td = int(np.searchsorted(tt, t_td))
ref_bottom = TS["r_ed"][int(np.searchsorted(tt, de_place[1] - 0.05))]
claw_o_td = S["re_o"][i_td]; claw_m_td = S["re_m"][i_td]; b_td = S["pb"][i_td]
approach = np.array([cp, sp, 0.0])                            # unit vector of the approach (world -x)
short = lambda v: float(-1e3 * (v @ approach))                # + = short of the target, toward the vehicle
trim_vec = np.array(trim["place"]) * 1e-3
parts = dict(trim=short(trim_vec), tracking=short(claw_o_td - TS["r_ed"][j_td]),
             hang=short((b_td - claw_m_td) - np.array([cp * (-off[0]), sp * (-off[0]), 0.0])))
hat = place_pt - np.array([cp * 0.03, sp * 0.03, 0.0]) + np.array([0, 0, 0.0])     # the basket's Get position on the place hat: typed x = Get - 0.03
flat = win(73.30, 73.45)
R["place"] = dict(t_arrive=w_place[0], t_press=w_place[1], wait_s=w_place[1] - w_place[0], claw_err_at_arrival_mm=mm(e_ee[mw][0]), claw_err_at_press_mm=mm(e_ee[mw][-1]),
                  overshoot_max_mm=float(1e3 * np.linalg.norm(e_ee[mw][:, :2], axis=1).max()), t_overshoot_after_arrival=float(tw_[k_]),
                  overshoot_vec_mm=mm(e_ee[mw][k_]), trim_logged_mm=trim["place"], mean_5s_mm=mm(tr5), mean_settled_last2s_mm=mm(sett),
                  settled_window=[w_place[0] + 3.0, w_place[1]],
                  t_touchdown=t_td, claw_minus_ref_at_td_mm=mm(claw_o_td - TS["r_ed"][j_td]), claw_minus_goal_at_td_mm=mm(claw_o_td - place_goal),
                  claw_mocap_minus_goal_at_td_mm=mm(claw_m_td - place_goal), basket_minus_place_point_at_td_mm=mm(b_td - place_pt), basket_at_td=b_td.round(4).tolist(),
                  short_parts_mm=parts, short_total_mm=short(b_td - place_pt), side_mm=float(1e3 * ((b_td - place_pt) @ np.array([-sp, cp, 0.0]))),
                  hat_centre_assumed=hat.round(4).tolist(), basket_minus_hat_at_td_mm=mm(b_td - hat),
                  basket_from_hat_centre_mm=float(1e3 * np.linalg.norm((b_td - hat)[:2])),
                  basket_flat_on_hat=dict(z=float(S["pb"][flat][:, 2].mean()), tilt=float(S["tiltb"][flat].mean())),
                  ref_bottom=ref_bottom.round(4).tolist(), claw_z_goal=float(place_goal[2]))
print("place:", json.dumps(R["place"]))

# what the landing would have been with other trims (same claw motion relative to its reference)
e_td = claw_o_td - TS["r_ed"][j_td]                           # the claw's error against the reference it was following at the touchdown
alt = {}
for lab, tv in (("as flown, 5 s window pressed at 5.0 s", trim_vec), ("trim off", np.zeros(3)), ("average of the settled part only", sett)):
    alt[lab] = dict(trim_mm=mm(tv), claw_from_typed_target_mm=mm(e_td + tv), dist_xy=float(1e3 * np.linalg.norm((e_td + tv)[:2])))
    print(f"   {lab:42s} trim {mm(tv)}  claw from its typed target at the touchdown {mm(e_td + tv)} = {alt[lab]['dist_xy']:.0f} mm")
R["place"]["trim_what_if"] = alt

# ------------------------------------------------------------------ the set-down
eulm = Rot.from_quat(S["Qv"][:, [1, 2, 3, 0]]).as_euler("ZYX", degrees=True)        # mocap attitude, pitch + = nose down
vx = np.gradient(S["Pm"][:, 0], t); kern = np.ones(5) / 5; vxs = np.convolve(vx, kern, "same")
x_pre = float(S["Pm"][win(t_td - 0.6, t_td)][:, 0].mean()); pitch_pre = float(eulm[win(t_td - 0.6, t_td)][:, 1].mean())
k_start = np.where((t > t_td) & (S["Pm"][:, 0] - x_pre > 0.010))[0][0]
lgo = rosout(d, t0, contains="gripper OPEN"); t_open_cmd = lgo[0][0]
k_open = np.where((t > t_open_cmd) & (S["g"][:, 0] > 0.005))[0][0]
m_sd = win(t_td, t_open_cmd + 0.5); k_max = np.where(m_sd)[0][np.argmax(S["Pm"][m_sd][:, 0])]
k_tip = np.where((t > t_td + 0.3) & (S["tiltb"] > 8.0) & (zb < R["place"]["basket_flat_on_hat"]["z"] - 0.01))[0][0]
k_floor = np.where((t > t_td) & (zb < 0.12))[0][0]
claw_above_ref = 1e3 * (S["re_o"][:, 2] - interp(t, tt, TS["r_ed"][:, 2]))
contact_abs = dict(mocap=R["place"]["basket_flat_on_hat"]["z"] + contact["mocap"] / 1e3, planner=R["place"]["basket_flat_on_hat"]["z"] + contact["planner"] / 1e3)
R["setdown"] = dict(t_touchdown=t_td, x_pre=x_pre, pitch_pre=pitch_pre, t_vehicle_starts=float(t[k_start]), t_open_cmd=t_open_cmd, t_fingers_open=float(t[k_open]),
                    t_leg_end=de_place[1], touchdown_to_leg_end_s=de_place[1] - t_td, touchdown_to_open_cmd_s=t_open_cmd - t_td,
                    vehicle_back_at_open_cmd_mm=float(1e3 * (S["Pm"][at(t_open_cmd), 0] - x_pre)), vehicle_back_max_mm=float(1e3 * (S["Pm"][k_max, 0] - x_pre)), t_back_max=float(t[k_max]),
                    speed_back_max=float(vxs[m_sd].max()), pitch_min=float(eulm[win(t_td, t_td + 0.8)][:, 1].min()), pitch_change=float(eulm[win(t_td, t_td + 0.8)][:, 1].min() - pitch_pre),
                    pitch_max_after=float(eulm[win(t_td + 0.8, t_open_cmd + 0.3)][:, 1].max()), tilt_max=float(tilt_of(S["Qv"])[win(t_td, t_open_cmd + 1.0)].max()),
                    claw_above_ref_mm=[float(claw_above_ref[win(t_td + 0.2, t_td + 0.8)].mean()), float(claw_above_ref[win(t_td + 0.2, t_td + 0.8)].max())],
                    claw_z_ref_bottom=float(ref_bottom[2]), arch_contact_z=contact_abs, claw_z_planner_min_before_tip=float(S["re_o"][win(t_td, t[k_tip])][:, 2].min()),
                    claw_z_mocap_min_before_tip=float(S["re_m"][win(t_td, t[k_tip])][:, 2].min()),
                    t_tip=float(t[k_tip]), basket_dragged_mm=float(1e3 * np.linalg.norm((S["pb"][k_tip] - b_td)[:2])), basket_at_tip=S["pb"][k_tip].round(4).tolist(),
                    basket_from_hat_at_tip_mm=float(1e3 * np.linalg.norm((S["pb"][k_tip] - hat)[:2])), t_floor=float(t[k_floor]),
                    basket_final=S["pb"][win(80, 82)].mean(0).round(3).tolist(),
                    q_at_td=np.degrees(S["q"][i_td]).round(1).tolist(), q_min_max_after=[np.degrees(S["q"][win(t_td, t_td + 0.8)]).min(0).round(1).tolist(), np.degrees(S["q"][win(t_td, t_td + 0.8)]).max(0).round(1).tolist()],
                    dz_before=float(TS["dhat"][wt(t_td - 1.0, t_td)][:, 2].mean()), dz_min_mag_after=float(TS["dhat"][wt(t_td, t_td + 0.8)][:, 2].max()))
print("setdown:", json.dumps(R["setdown"]))

# ------------------------------------------------------------------ joint 2 with the basket: stick-slip and the position guard
tjj = tj; q2 = np.degrees(qj[:, 1]); q3 = np.degrees(qj[:, 2]); q4 = np.degrees(qj[:, 3])
ml = (tjj > t[kl] + 2.5) & (tjj < 72.6)
te = d["js__recv"] - t0; eff = d["js__effort"][:, :4][:, JS_IDX] * HW_SIGN / KT
cap_nm = np.array([57.6, 365.0, 192.0, 57.6]) / KPWM
me = (te > t[kl]) & (te < 72.6); mu = (te > 6) & (te < 30)
mc = (tjj > 46.0) & (tjj < 53.5); q2c = q2[mc]; tc = tjj[mc]
up_ = np.where(np.diff((q2c > 40.5).astype(int)) == 1)[0]
R["joint2"] = dict(q2_max=float(q2.max()), s_over_44=float((q2[(tjj > 5.8) & (tjj < 74.8)] > 44.0).sum() * 0.004), s_over_45=float((q2[(tjj > 5.8) & (tjj < 74.8)] > 45.0).sum() * 0.004),
                   episodes_near_guard=json.load(open(os.path.join(OUT10, "q2_guard_p4.json")))["episodes"] if os.path.exists(os.path.join(OUT10, "q2_guard_p4.json")) else None,
                   stick_slip_jumps_at=tc[up_].round(2).tolist(), stick_slip_period=float(np.diff(tc[up_]).mean()) if len(up_) > 1 else None,
                   q2_low_high=[float(np.percentile(q2c, 10)), float(np.percentile(q2c, 90))], q3_low_high=[float(np.percentile(q3[mc], 10)), float(np.percentile(q3[mc], 90))],
                   q4_low_high=[float(np.percentile(q4[mc], 5)), float(np.percentile(q4[mc], 95))], q_ref_carry=[12.0, 38.0, 42.0, 0.0],
                   applied_max=np.abs(eff[me]).max(0).round(2).tolist(), applied_mean_loaded=eff[me].mean(0).round(2).tolist(), applied_mean_unloaded=eff[mu].mean(0).round(2).tolist(),
                   caps=cap_nm.round(2).tolist(), applied_share_of_cap=(np.abs(eff[me]).max(0) / cap_nm).round(2).tolist(),
                   s_above_80pct=[float((np.abs(eff[me][:, i]) > 0.8 * cap_nm[i]).sum() * 0.004) for i in range(4)],
                   law_tau_max=np.abs(TS["tau"][wt(t[kl], 72.6)]).max(0).round(2).tolist())
print("joint2:", json.dumps(R["joint2"]))

# ------------------------------------------------------------------ PX4 attitude vs mocap attitude (body-fixed part), battery
Ro = Rot.from_quat(S["Qo"][:, [1, 2, 3, 0]]); Rm_ = Rot.from_quat(S["Qv"][:, [1, 2, 3, 0]])
rel = (Rm_.inv() * Ro).as_euler("ZYX", degrees=True)
mh = win(10, 30)
R["attitude_offset"] = dict(roll=float(rel[mh][:, 2].mean()), pitch=float(rel[mh][:, 1].mean()), yaw=float(rel[mh][:, 0].mean()),
                            roll_loaded=float(rel[win(60, 71)][:, 2].mean()), pitch_loaded=float(rel[win(60, 71)][:, 1].mean()),
                            claw_z_planner_minus_mocap_mm=dict(pick=float(1e3 * (S["re_o"] - S["re_m"])[win(*de_pick)][:, 2].mean()), place=float(1e3 * (S["re_o"] - S["re_m"])[win(67, 72.6)][:, 2].mean())))
tbt = d["batt__recv"] - t0; V = d["batt__voltage_v"]; A = d["batt__current_a"]
bw = lambda a, b: (float(V[(tbt > a) & (tbt < b)].mean()), float(A[(tbt > a) & (tbt < b)].mean()))
R["battery"] = dict(start=bw(6, 10), before_lift=bw(26, 30), carry=bw(60, 66), after_cut=bw(77.5, 80))
print("attitude offset:", json.dumps(R["attitude_offset"])); print("battery:", json.dumps(R["battery"]))

# ------------------------------------------------------------------ the takeover and the fall
tv = d["vstatus__recv"] - t0
t_stab = float(tv[np.argmax(d["vstatus__nav_state"] == 15)]); t_disarm = float(tv[np.argmax((d["vstatus__arming_state"] == 1) & (tv > 20))])
z = Pm[:, 2]; vz = np.convolve(np.gradient(z, tm), np.ones(12) / 12, "same"); tiltv = tilt_of(Qm)
ma = (tm > t_stab) & (tm < t_stab + 4)
kz = np.where(ma)[0][np.argmax(z[ma])]
kcut = np.where((tbt > t_stab) & (A < 5.0))[0][0]
kimp = np.where((tm > tm[kz]) & (z < 0.26))[0][0]
R["takeover"] = dict(t_stab=t_stab, z_at_stab=float(np.interp(t_stab, tm, z)), pos_at_stab=interp(np.array([t_stab]), tm, Pm)[0].round(3).tolist(),
                     t_top=float(tm[kz]), z_top=float(z[kz]), t_current_drop=float(tbt[kcut]), current_before=float(A[(tbt > t_stab) & (tbt < tbt[kcut] - 0.15)].mean()),
                     t_impact=float(tm[kimp]), vz_impact=float(vz[(tm > tm[kz]) & (tm < tm[kimp] + 0.05)].min()), tilt_max=float(tiltv[ma].max()),
                     t_disarm=t_disarm, rest_pos=Pm[(tm > 78.5) & (tm < 82)].mean(0).round(3).tolist(), rest_tilt=float(tiltv[(tm > 78.5) & (tm < 82)].mean()),
                     travel_m=float(np.linalg.norm((Pm[(tm > 78.5) & (tm < 82)].mean(0) - interp(np.array([t_stab]), tm, Pm)[0])[:2])),
                     abort_at=next((x for x, y, l in ph if l.startswith("ABORTING")), None),
                     arm_torque_on_ground_max=np.abs(eff[(te > 77.2) & (te < 87)]).max(0).round(2).tolist(),
                     wbmode_after=[v for _, v in edges(d, t0, "wbmode")][-1])
print("takeover:", json.dumps(R["takeover"]))

# ------------------------------------------------------------------ the link
gaps = {}
for key in ("vatt", "sc", "vodom", "odom", "wb", "thr_in"):
    tk = d[f"{key}__recv"] - t0; mk = (tk > 5.8) & (tk < t_stab)
    gaps[key] = dict(rate=float(mk.sum() / (t_stab - 5.8)), max_gap_ms=float(1e3 * np.diff(tk[mk]).max()), p999_ms=float(1e3 * np.percentile(np.diff(tk[mk]), 99.9)))
R["link"] = gaps
print("link:", json.dumps(gaps))
json.dump(R, open(os.path.join(OUT10, "pp4.json"), "w"), indent=1)
print("wrote pp4.json")

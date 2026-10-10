"""Data for the report's interactive figures -> ../analysis/fig_*.json (downsampled, rounded)."""
import json
from pp_common import *
from scipy.spatial.transform import Rotation as Rot
import grasp as G
import grasp_geom as GG

SHORT = {"FLYING go_to_start": "to start", "FLYING execute_pick (approach)": "to pick", "WAITING execute_pick": "wait",
         "FLYING execute_pick (descent)": "slide in", "FLYING exit_pick (up the vertical margin)": "lift",
         "FLYING go_to_place_start": "to place start", "FLYING execute_place (approach)": "to place",
         "WAITING execute_place": "wait", "FLYING execute_place (descent)": "descend",
         "FLYING exit_place (up the vertical margin)": "exit", "FLYING go_to_land_start": "to land start",
         "FLYING execute_land": "land hover"}
r3 = lambda a, n=3: np.round(np.asarray(a, float), n).tolist()


def ds(t, X, tq):
    return interp(tq, t, X)


out3d, outerr = {}, {}
for nm in RUNS:
    d, t0 = load(nm); a, b = direct_span(d, t0)
    S = GG.series(nm); t = S["t"]
    tr, ref = wbref(d, t0)
    tq = np.arange(a, b, 0.1)
    red = ds(tr, ref["r_ed"], tq); xcd = ds(tr, ref["x_cd"], tq)
    claw = ds(t, S["re_m"], tq); base = ds(t, S["Pm"], tq)
    ph = [(x, y, l) for x, y, l in pp_phases(d, t0) if y > a and x < b]
    e = dict(t=r3(tq, 2), ref=r3(red), claw=r3(claw), base=r3(base),
             phases=[[round(x, 2), round(y, 2), SHORT.get(l, "")] for x, y, l in ph if l in SHORT])
    if nm != "p1":                       # the basket was streamed in flights 2 and 3 only
        tb = np.arange(max(a, t[0]), min(t[-1], b + (5 if nm == "p2" else 0)), 0.1)
        if nm == "p2":
            tb = tb[tb < 119.0]          # removed from the claw at ~119 s
        e["basket"] = r3(ds(t, S["pb"], tb)); e["tb"] = r3(tb, 2)
    if nm == "p2":                       # the uncontrolled track after the link froze
        tm, Pm, _, _ = mocap_pose(d, t0, "mocap"); tf = np.arange(144.67, 149.0, 0.05)
        e["flyaway"] = r3(ds(tm, Pm, tf)); e["tf"] = r3(tf, 2)
    I = d["pp_info__data"]; info = I[np.argmax(I[:, I_PLANNED] > 0.5)]
    e["pick_goal"] = r3(info[I_GOAL + 7 + 4:I_GOAL + 7 + 7]); e["place_goal"] = r3(info[I_GOAL + 21 + 4:I_GOAL + 21 + 7])
    e["capture"] = r3(info[I_CAP + 4:I_CAP + 7])
    out3d[nm] = e
    s = json.load(open(os.path.join(OUT, f"series_{nm}.json"))); ts = np.array(s["t"])
    g = lambda k: ds(ts, np.array(s[k]), tq)
    outerr[nm] = dict(t=r3(tq, 2), e_com=r3(1e3 * g("e_com"), 1), e_task=r3(1e3 * g("e_task"), 1), e_head=r3(g("e_head"), 2),
                      e_yaw=r3(g("e_yaw"), 2), tilt=r3(g("tilt"), 2), u1=r3(g("u1"), 2), tau=r3(g("tau")[:, 1:3], 3),
                      phases=e["phases"])
json.dump(out3d, open(os.path.join(OUT, "fig_3d.json"), "w")); json.dump(outerr, open(os.path.join(OUT, "fig_err.json"), "w"))

# --- pick: the claw in the basket's rest frame (mocap FK), flights 2 and 3
pick = {}
for nm in ("p2", "p3"):
    r, S, ph = G.run(nm); s = r["_series"]; t = s["t"]
    w = next((x, y) for x, y, l in ph if l.startswith("WAITING execute_pick"))
    ex = next((x, y) for x, y, l in ph if l.startswith("FLYING exit_pick"))
    de = next((x, y) for x, y, l in ph if l.startswith("FLYING execute_pick (descent)"))
    tq = np.arange(w[0], ex[0] + 2.2, 0.05)
    pick[nm] = dict(t=r3(tq, 2), claw=r3(ds(t, s["rm"], tq), 1), claw_planner=r3(ds(t, s["ro"], tq), 1),
                    basket_dz=r3(1e3 * (ds(t, s["zb"], tq) - r["pick_rest"]["p"][2]), 1),
                    t_slide=[de[0], de[1]], t_close=r["pick"]["t_close_cmd"], t_exit=ex[0], t_liftoff=r["pick"].get("t_liftoff"),
                    ee_offset=[1e3 * v for v in r["ee_offset"]])
json.dump(pick, open(os.path.join(OUT, "fig_pick.json"), "w"))

# --- place, flight 2: distances along world x (the approach axis at yaw -180), relative to the targets
r, S, ph = G.run("p2"); s = r["_series"]; t = s["t"]; d, t0 = S["d"], S["t0"]
tr, ref = wbref(d, t0); goal = np.array(r["place_goal"]); place = np.array(r["place_point"])
tq = np.arange(79.0, 97.5, 0.05)
cl = ds(t, s["re_m"], tq); bk = ds(t, s["pb"], tq); rd = ds(tr, ref["r_ed"], tq)
json.dump(dict(t=r3(tq, 2), claw=r3(1e3 * (cl - goal), 1), basket=r3(1e3 * (bk - place), 1), ref=r3(1e3 * (rd - goal), 1),
               basket_z=r3(bk[:, 2], 4), claw_z=r3(cl[:, 2], 4), place=r3(place), goal=r3(goal),
               ev=dict(arrive=81.62, press=82.43, touchdown=89.5, open=91.15, exit_end=97.13)),
          open(os.path.join(OUT, "fig_place.json"), "w"))

# --- the link freeze, flight 2
tm, Pm, Qm, Vm = mocap_pose(d, t0, "mocap"); em = Rot.from_quat(Qm[:, [1, 2, 3, 0]]).as_euler("ZYX", degrees=True)
tq = np.arange(143.5, 149.5, 0.02)
json.dump(dict(t=r3(tq, 2), pos=r3(ds(tm, Pm, tq)), speed=r3(np.linalg.norm(ds(tm, Vm, tq), axis=1), 2),
               roll=r3(ds(tm, em[:, 2], tq), 1), pitch=r3(ds(tm, em[:, 1], tq), 1), yaw=r3(ds(tm, np.degrees(np.unwrap(np.radians(em[:, 0]))), tq), 1),
               ev=dict(freeze=144.67, revert=145.67, touchdown=147.35)), open(os.path.join(OUT, "fig_freeze.json"), "w"))

# --- the two manual touchdowns
land = {}
for nm, (ta, tb_) in (("p1", (186.5, 192.4)), ("p3", (80.0, 86.0))):
    d, t0 = load(nm); tm, Pm, Qm, Vm = mocap_pose(d, t0, "mocap"); em = Rot.from_quat(Qm[:, [1, 2, 3, 0]]).as_euler("ZYX", degrees=True)
    tq = np.arange(ta, tb_, 0.02); tj = d["js__recv"] - t0; eff = d["js__effort"][:, :4][:, JS_IDX] / KT
    st = d["vstatus__recv"] - t0
    t_stab = float(st[np.argmax(d["vstatus__nav_state"] == 15)]) if nm else None
    land[nm] = dict(t=r3(tq - ta, 2), t0=ta, z=r3(ds(tm, Pm[:, 2], tq)), pitch=r3(ds(tm, em[:, 1], tq), 1), roll=r3(ds(tm, em[:, 2], tq), 1),
                    tau2=r3(ds(tj, eff[:, 1], tq), 2), tau3=r3(ds(tj, eff[:, 2], tq), 2),
                    t_disarm=float(st[np.argmax((d["vstatus__arming_state"] == 1) & (st > ta))]) - ta,
                    t_stab=float(st[np.argmax((d["vstatus__nav_state"] == 15) & (st > 20))]) - ta)
json.dump(land, open(os.path.join(OUT, "fig_land.json"), "w"))
for f in ("fig_3d", "fig_err", "fig_pick", "fig_place", "fig_freeze", "fig_land"):
    print(f, os.path.getsize(os.path.join(OUT, f + ".json")) // 1024, "kB")
print(land["p1"]["t_stab"], land["p1"]["t_disarm"], land["p3"]["t_stab"], land["p3"]["t_disarm"])

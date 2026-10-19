"""A few more PP-4 numbers for the report: the claw's error at each press, the skids against the place hat, the arrival swings.
Writes ../analysis/extra_p4.json."""
import json
from pp10_common import *
import tracking as T
import grasp_geom as GG
A = json.load(open(os.path.join(OUT10, "pp4.json")))
d, t0, TS = T.build("p4"); tt = TS["t"]; S = GG.series("p4"); t = S["t"]
ph = pp_phases(d, t0); span = lambda key: next(((a, b) for a, b, l in ph if l.startswith(key)), None)
out = {}
for k, key in (("pick", "WAITING execute_pick"), ("place", "WAITING execute_place")):
    w = span(key); m = (tt >= w[0]) & (tt <= w[1]); e = TS["e_ee"][m]; i = len(e) - 1
    W = 5.0; mw = (tt >= max(w[0], w[1] - W)) & (tt <= w[1]); avg = TS["e_ee"][mw].mean(0)
    out[k] = dict(wait=w[1] - w[0], err_at_press_mm=float(1e3 * np.linalg.norm(e[i][:2])), err_at_arrival_mm=float(1e3 * np.linalg.norm(e[0][:2])),
                  avg_mm=float(1e3 * np.linalg.norm(avg[:2])), claw_vs_avg_mm=float(1e3 * np.linalg.norm((e[i] - avg)[:2])), avg_vec=(1e3 * avg).round(1).tolist(),
                  err_vec=(1e3 * e[i]).round(1).tolist(), half_window_filled=bool(w[1] - w[0] >= 0.5 * W))
    print(k, json.dumps(out[k]))
# the pick slide-in: sideways error of the claw against its reference (planner frame) and against the stem line (mocap)
de = span("FLYING execute_pick (descent)"); tc = A["pick"]["t_close"]; m = (tt >= de[0]) & (tt <= tc)
yaw = np.radians(A["settings"]["pick_yaw"]); ac = 1e3 * (-np.sin(yaw) * TS["e_ee"][m][:, 0] + np.cos(yaw) * TS["e_ee"][m][:, 1])
out["pick_slide_across_mm"] = dict(max=float(np.abs(ac).max()), p95=float(np.percentile(np.abs(ac), 95)), at_close=float(ac[-1]))
print("slide-in across:", out["pick_slide_across_mm"])
# the skids against the place hat (T650 gear: 242.5 mm below the body origin, +-163 mm fore-aft, inner faces 142 mm from the centreline)
hat = np.array(A["place"]["hat_centre_assumed"]); ztop = A["place"]["basket_flat_on_hat"]["z"] - 0.0325
td = A["place"]["t_touchdown"]; i = int(np.searchsorted(t, td)); pm = S["Pm"][i]
tip = pm[0] - 0.163; rim = hat[0] + 0.1003
base_bottom = float(interp(np.array([span("FLYING execute_place (descent)")[1] - 0.05]), tt, TS["base"])[0][2]) if False else None
out["skids"] = dict(body_at_td=pm.round(3).tolist(), skid_front_tip_x=float(tip), hat_near_rim_x=float(rim), tip_behind_rim_mm=float(1e3 * (tip - rim)),
                    skid_bottom_above_hat_at_td_mm=float(1e3 * (pm[2] - 0.2425 - ztop)), lateral_offset_mm=float(1e3 * (pm[1] - hat[1])),
                    skid_inner_face_from_hat_rim_mm=[float(1e3 * (0.142 - abs(pm[1] - hat[1]) - 0.1003)), float(1e3 * (0.142 + abs(pm[1] - hat[1]) - 0.1003))],
                    basket_ahead_of_skid_tip_mm=float(1e3 * (tip - A["place"]["basket_at_td"][0])))
tr, ref = wbref(d, t0); xb, R0, yawr = ref_base({k: v[-400:] for k, v in ref.items()}) if False else (None, None, None)
print("skids:", json.dumps(out["skids"]))
# the claw's arrival overshoot above each site (max |error| in the first 2.5 s after the approach leg ends), with and without the basket
for k, key in (("pick", "WAITING execute_pick"), ("place", "WAITING execute_place")):
    w = span(key); m = (tt >= w[0]) & (tt <= w[0] + 2.5); e = np.linalg.norm(TS["e_ee"][m][:, :2], axis=1)
    out[f"arrival_{k}"] = dict(max_mm=float(1e3 * e.max()), t_after=float(tt[m][np.argmax(e)] - w[0]))
    ws = (tt >= w[0]) & (tt <= w[1]); es = np.linalg.norm(TS["e_ee"][ws][:, :2], axis=1); under = np.where(es < 0.025)[0]
    print(k, out[f"arrival_{k}"])
m = (tt >= span("WAITING execute_place")[0] + 3.0) & (tt <= span("WAITING execute_place")[1])
out["place_settled_from"] = 3.0
de_pl = span("FLYING execute_place (descent)"); m = (tt >= de_pl[0]) & (tt <= td)
out["descent_before_td"] = dict(com_max=float(1e3 * np.linalg.norm(TS["e_com"][m], axis=1).max()), com_rms=float(1e3 * rms(np.linalg.norm(TS["e_com"][m], axis=1))),
                                tilt_max=float(TS["tilt"][m].max()))
print("descent before the touchdown:", out["descent_before_td"])
json.dump(out, open(os.path.join(OUT10, "extra_p4.json"), "w"), indent=1)

"""Top view and side view of the pick and of the place, three stages each -> ../analysis/fig_views.json.

frame      origin = centre of the hat, on its top surface; s = along the approach (the claw's slide-in direction),
           c = across (to the left of the approach), h = height above the hat top. All in mm.
           pick  : hat centre = the basket's resting position (PP-2, PP-3) or the captured pick point (PP-1, basket
                   not streamed); approach axis = the captured pick yaw; hat top = basket centre - 32.5 mm (CAD)
           place : hat centre = the typed place point; approach axis = the place yaw (-180 deg: world -x);
                   hat top = the same height as the pick hat (the place hat's own height is not measured -- the
                   basket touched it at a centre height of 0.860-0.863 m in PP-2, the pick hat's resting height)
claw       the planner's grasp point by its kinematics on the vehicle's MOCAP pose and the measured joints (the same
           frame as the basket), and the planner's streamed claw reference
stages     Ready = the approach leg and the wait; Pick / Place = from the button to the start of the exit;
           Exit = the exit leg and the following 3 s
"""
import json
from pp_common import *
import grasp_geom as GG

BOX_HALF_H = 0.0325
r1 = lambda a: np.round(np.asarray(a, float), 1).tolist()


def frame(o, yaw_deg, ztop):
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    def f(P):
        P = np.atleast_2d(np.asarray(P, float)); d = P - np.array([o[0], o[1], ztop])
        return 1e3 * np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]])
    return f


def build(nm):
    S = GG.series(nm); d, t0, t = S["d"], S["t0"], S["t"]
    ph = pp_phases(d, t0); span = lambda key: next(((a, b) for a, b, l in ph if l.startswith(key)), None)
    tr, ref = wbref(d, t0)
    I = d["pp_info__data"]; info = I[np.argmax(I[:, I_PLANNED] > 0.5)]
    cap = info[I_CAP + 4:I_CAP + 7]; pick_yaw = float(info[I_PICKYAW])
    pick_goal = info[I_GOAL + 7 + 4:I_GOAL + 7 + 7]; place_goal = info[I_GOAL + 21 + 4:I_GOAL + 21 + 7]; place_yaw = float(info[I_GOAL + 21 + 3])
    cy, sy = np.cos(np.radians(pick_yaw)), np.sin(np.radians(pick_yaw)); dg = pick_goal - cap
    off = np.array([cy * dg[0] + sy * dg[1], -sy * dg[0] + cy * dg[1], dg[2]])            # EE Offset as flown
    cp, sp = np.cos(np.radians(place_yaw)), np.sin(np.radians(place_yaw))
    place_pt = place_goal - np.array([cp * off[0] - sp * off[1], sp * off[0] + cp * off[1], off[2]])
    has_b = nm != "p1"
    tb, Pb, Qb, _ = mocap_pose(d, t0, "obj")
    if has_b:
        ap = span("FLYING execute_pick (approach)")
        m0 = (np.abs(np.degrees(yaw_of(Qb))) < 90) & (tilt_of(Qb) < 2.0) & (tb > 2.0) & (tb < ap[0])
        rest = Pb[m0].mean(0)
    else:
        rest = cap
    ztop = rest[2] - BOX_HALF_H
    tg, G = gripper(d, t0); g = G[:, 0]
    cross = lambda a, b, up: next((float(tg[k]) for k in np.where(np.diff((g > 0.005).astype(int)) == (1 if up else -1))[0] if a <= tg[k] <= b), None)
    out = dict(ee_offset=r1(1e3 * off), hat_top_z=float(ztop), payload=has_b)

    def task(name, keys, o, yaw, goal, tail=3.0):
        a0 = span(keys[0]); w = span(keys[1]); de = span(keys[2]); ex = span(keys[3])
        if a0 is None or ex is None:
            return None
        f = frame(o, yaw, ztop)
        st = [("ready", a0[0], w[1]), (name, de[0], ex[0]), ("exit", ex[0], ex[1] + tail)]
        tq = np.arange(a0[0], ex[1] + tail, 0.05)
        claw = f(interp(tq, t, S["re_m"])); rf = f(interp(tq, tr, ref["r_ed"])); base = f(interp(tq, t, S["Pm"]))
        keep = claw[:, 0] > -520
        e = dict(t=np.round(tq[keep], 2).tolist(), claw=r1(claw[keep]), ref=r1(rf[keep]), base_h=r1(base[keep][:, 2]),
                 stages=[[k, round(x, 2), round(y, 2)] for k, x, y in st], origin=[round(float(v), 3) for v in o[:2]], yaw=round(yaw, 1),
                 goal=r1(f(goal)[0]), z_world0=round(float(ztop), 4))
        if has_b and not (name == "place" and nm == "p3"):
            e["basket"] = r1(f(interp(tq, t, S["pb"]))[keep])
        return e, f

    r = task("pick", ("FLYING execute_pick (approach)", "WAITING execute_pick", "FLYING execute_pick (descent)", "FLYING exit_pick"), rest, pick_yaw, pick_goal)
    if r:
        e, f = r; ex = span("FLYING exit_pick"); de = span("FLYING execute_pick (descent)")
        e["ev"] = dict(pressed=de[0], close=cross(de[0], ex[0] + 1, False))
        e["rest"] = r1(f(rest)[0]); e["capture"] = r1(f(cap)[0])
        if has_b:
            zb = S["pb"][:, 2]; up = np.where((t >= ex[0]) & (t < ex[0] + 8) & (zb > rest[2] + 0.003))[0]
            if len(up) > 20 and zb[min(up[0] + 100, len(zb) - 1)] > rest[2] + 0.05:
                e["ev"]["liftoff"] = float(t[up[0]])
        out["pick"] = e
    if nm != "p3":
        r = task("place", ("FLYING execute_place (approach)", "WAITING execute_place", "FLYING execute_place (descent)", "FLYING exit_place"),
                 place_pt, place_yaw, place_goal)
        if r:
            e, f = r; de = span("FLYING execute_place (descent)"); ex = span("FLYING exit_place")
            e["ev"] = dict(pressed=de[0], open=cross(de[1] - 1, ex[1], True))
            e["place_point"] = r1(f(place_pt)[0])
            if nm == "p2":
                zb = S["pb"][:, 2]; m = (t > de[0] + 3) & (t < ex[0] + 0.5); vz = np.gradient(zb, t)
                k = np.where(m & (vz > -0.02) & (zb < zb[np.searchsorted(t, de[0])] - 0.10))[0]
                if len(k):
                    e["ev"]["touchdown"] = float(t[k[0]])
            out["place"] = e
    return out


if __name__ == "__main__":
    res = {nm: build(nm) for nm in RUNS}
    json.dump(res, open(os.path.join(OUT, "fig_views.json"), "w"))
    for nm, r in res.items():
        print("==", RUNS[nm][0], "EE offset", r["ee_offset"], "hat top z", round(r["hat_top_z"], 4))
        for k in ("pick", "place"):
            if k not in r:
                continue
            e = r[k]; c = np.array(e["claw"]); tt = np.array(e["t"])
            print(f"  {k}: origin {e['origin']} yaw {e['yaw']}  goal {e['goal']}  events {e['ev']}  stages {e['stages']}")
            print(f"     claw s {c[:, 0].min():.0f}..{c[:, 0].max():.0f}  c {c[:, 1].min():.0f}..{c[:, 1].max():.0f}  h {c[:, 2].min():.0f}..{c[:, 2].max():.0f}")
            if "basket" in e:
                b = np.array(e["basket"]); print(f"     basket s {b[:, 0].min():.0f}..{b[:, 0].max():.0f}  c {b[:, 1].min():.0f}..{b[:, 1].max():.0f}  h {b[:, 2].min():.0f}..{b[:, 2].max():.0f}")
            for key, tv in e["ev"].items():
                if tv is None:
                    continue
                i = int(np.argmin(np.abs(tt - tv)))
                print(f"     at {key} ({tv:.2f} s): claw {c[i]}" + (f"  basket {np.array(e['basket'])[i]}" if "basket" in e else ""))
    print(os.path.getsize(os.path.join(OUT, "fig_views.json")) // 1024, "kB")

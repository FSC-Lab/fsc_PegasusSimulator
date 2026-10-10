"""Hover RMSE at the pick and at the place, horizontal and vertical, claw and airframe.

pick hover   reference still, between the arrival behind the stem and the start of Exit To Pick (the wait behind the
             stem and the wait on the handle); no payload on the claw; all three flights
place hover  reference still at the place site: above the place before Place is pressed, and after Exit To Place;
             PP-2 with the basket on the claw (leaving out 89.5-96.5 s, basket resting on the hat's rim), PP-1 without
rmse         sqrt(mean(e^2)) of measured minus reference: horizontal = sqrt(ex^2 + ey^2), vertical = ez
settled      without the first 3 s after the reference stops
Writes ../analysis/hover_pick_place.json.
"""
import json
import hover_error as H
from pp_common import *

PICK = ("WAITING execute_pick", "DONE execute_pick")
PLACE = ("WAITING execute_place", "DONE exit_place")
PLACE_START = ("DONE go_to_place_start",)


def rm(E):
    E = np.asarray(E)
    return dict(n=int(len(E)), s=float(len(E) * 0.02), h=float(1e3 * np.sqrt((E[:, :2] ** 2).sum(1).mean())), x=float(1e3 * np.sqrt((E[:, 0] ** 2).mean())),
                y=float(1e3 * np.sqrt((E[:, 1] ** 2).mean())), z=float(1e3 * np.sqrt((E[:, 2] ** 2).mean())), z_mean=float(1e3 * E[:, 2].mean()),
                h_max=float(1e3 * np.linalg.norm(E[:, :2], axis=1).max()), z_max=float(1e3 * np.abs(E[:, 2]).max()))


if __name__ == "__main__":
    pools = {}; rows = []
    for nm in RUNS:
        d, t0, S, W = H.windows_of(nm); t = S["t"]
        for a, b, l in W:
            grp = "pick" if l.startswith(PICK) else "place" if l.startswith(PLACE) else "place_start" if l.startswith(PLACE_START) else None
            if grp is None:
                continue
            cuts = sorted([a, b] + [x for x in H.LOADED.get(nm, ()) if a < x < b] + [x for r in H.EXCLUDE.get(nm, []) for x in r if a < x < b])
            for ca, cb in zip(cuts[:-1], cuts[1:]):
                if any(x0 <= 0.5 * (ca + cb) <= x1 for x0, x1 in H.EXCLUDE.get(nm, [])):
                    continue
                st = H.load_state(nm, ca, cb)
                for tag, m in (("all", (t >= ca) & (t <= cb)), ("settled", (t >= ca + (H.SETTLE if ca == a else 0.0)) & (t <= cb))):
                    if m.sum() < 25:
                        continue
                    key = (grp, st, tag)
                    pools.setdefault(key, {"ee": [], "com": []})
                    pools[key]["ee"].append(S["e_ee"][m]); pools[key]["com"].append(S["e_com"][m])
                    rows.append(dict(run=RUNS[nm][0], grp=grp, state=st, tag=tag, t0=float(ca), t1=float(cb), phase=l, ee=rm(S["e_ee"][m]), com=rm(S["e_com"][m])))
    out = {"windows": rows, "pooled": {}}
    print("per window (claw | airframe), mm:  horizontal rmse (x, y)  vertical rmse (mean)")
    for r in rows:
        e, c = r["ee"], r["com"]
        print(f"  {r['run']} {r['grp']:11s} {r['state']:7s} {r['tag']:7s} {r['t0']:6.1f}-{r['t1']:6.1f} {e['s']:5.1f} s | claw h {e['h']:5.1f} (x {e['x']:5.1f}, y {e['y']:5.1f}) z {e['z']:4.1f} ({e['z_mean']:+5.1f})"
              f" | airframe h {c['h']:5.1f} (x {c['x']:5.1f}, y {c['y']:5.1f}) z {c['z']:4.1f} ({c['z_mean']:+5.1f}) | {r['phase'][:28]}")
    print("\npooled")
    for key in sorted(pools):
        e = rm(np.vstack(pools[key]["ee"])); c = rm(np.vstack(pools[key]["com"]))
        out["pooled"]["/".join(key)] = dict(ee=e, com=c, windows=len(pools[key]["ee"]))
        print(f"  {key[0]:11s} {key[1]:7s} {key[2]:7s} {e['s']:5.1f} s in {len(pools[key]['ee'])} windows | claw h {e['h']:5.1f} (x {e['x']:5.1f}, y {e['y']:5.1f}) max {e['h_max']:5.1f}  z {e['z']:4.1f} (mean {e['z_mean']:+5.1f}, max {e['z_max']:4.1f})"
              f" | airframe h {c['h']:5.1f} (x {c['x']:5.1f}, y {c['y']:5.1f}) z {c['z']:4.1f} (mean {c['z_mean']:+5.1f})")
    json.dump(out, open(os.path.join(OUT, "hover_pick_place.json"), "w"), indent=1)

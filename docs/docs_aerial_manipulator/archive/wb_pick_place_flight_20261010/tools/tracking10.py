"""Tracking of the whole-body controller through PP-4's DIRECT span, per mission phase (the 10-09 tracking.py on p4).
Writes ../analysis/tracking_p4.json and ../analysis/series_p4.json (20 Hz series for the figures)."""
import json
from pp10_common import *
import tracking as T

if __name__ == "__main__":
    nm = "p4"
    d, t0, S = T.build(nm); t = S["t"]
    ph = [(x, y, lab) for x, y, lab in pp_phases(d, t0) if y > S["a"] and x < S["b"]]
    rows = []
    for x, y, lab in ph:
        m = (t >= x) & (t < y)
        if m.sum() < 10:
            continue
        r = T.metrics(S, m); r.update(label=lab, t0=x, t1=y); rows.append(r)
    # the flight up to the place touchdown (72.7 s) is the part where the law flew a planned reference undisturbed
    spans = {"whole DIRECT": np.ones(len(t), bool), "to the place touchdown (72.7 s)": t < 72.7,
             "before the lift (no payload)": t < 30.5, "carry and place (payload on the claw)": (t >= 34.3) & (t < 72.7)}
    tot = {k: T.metrics(S, m) for k, m in spans.items()}
    extra = dict(direct=[S["a"], S["b"]], n_sat_ticks=int((S["n_sat"] > 0).sum()), n_clamp_ticks=int((S["n_clamped"] > 0).sum()),
                 ticks=int(len(S["n_sat"])), stream_stale_ticks=int((S["fresh"] < 0.5).sum()))
    print(f"===== {nm}  DIRECT {S['a']:.1f}-{S['b']:.1f} s  sat ticks {extra['n_sat_ticks']} clamp ticks {extra['n_clamp_ticks']} of {extra['ticks']}, stale-stream ticks {extra['stream_stale_ticks']}")
    for k, r in tot.items():
        print(f"   {k:40s}: CoM rms {r['com']['rms']:.1f} max {r['com']['max']:.1f} | EE rms {r['ee']['rms']:.1f} max {r['ee']['max']:.1f} | task rms {r['task']['rms']:.1f} | yaw rms {r['e_yaw']['rms']:.2f} max {r['e_yaw']['max']:.2f} | head rms {r['e_head']['rms']:.2f} | tilt max {r['tilt_max']:.1f} | eR rms {r['eR_rms']:.3f} | motors {r['motor_min']:.2f}-{r['motor_max']:.2f} | tau max {r['tau_max']}")
    for r in rows:
        print(f"   {r['t0']:7.2f}-{r['t1']:7.2f} {r['label'][:34]:34s} CoM {r['com']['rms']:5.1f}/{r['com']['max']:5.1f}  EE {r['ee']['rms']:5.1f}/{r['ee']['max']:5.1f}  task {r['task']['rms']:4.1f}/{r['task']['max']:5.1f}  yaw {r['e_yaw']['rms']:4.2f}/{r['e_yaw']['max']:4.2f} head {r['e_head']['rms']:4.2f}/{r['e_head']['max']:5.2f}  q-rms {r['e_q']['rms']}  q-max {r['e_q']['max']} tilt {r['tilt_max']:4.1f}  dz {r['dhat_z']:6.2f}")
    k = slice(None, None, 3)
    ser = {key: np.round(S[key][k], 4).tolist() for key in ("t", "e_com", "e_ee", "e_task", "e_yaw", "e_head", "e_q", "q", "q_d", "tilt", "eR",
                                                             "motors", "tau", "u1", "x_c", "x_cd", "r_e", "r_ed", "r_e_mocap", "base", "base_mocap", "yaw", "yaw_ref")}
    ser["dhat"] = np.round(S["dhat"][k], 3).tolist(); ser["Fraw"] = np.round(S["Fraw"][k], 3).tolist()
    ser["phases"] = [[x, y, lab] for x, y, lab in ph]
    json.dump(ser, open(os.path.join(OUT10, f"series_{nm}.json"), "w"))
    json.dump(dict(tag="PP-4", time="19:07", totals=tot, extra=extra, phases=rows), open(os.path.join(OUT10, "tracking_p4.json"), "w"), indent=1)

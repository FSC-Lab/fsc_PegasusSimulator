#!/usr/bin/env python3
"""margins.py -- actuator and attitude margins of a whole-body figure-8 run, the same computation on the 1005
hardware bags and on the Isaac sweep: the law's commanded collective u1 and body torque (FLU,
wb_control_debug [17], [21..23]), rotor commands rebuilt through the HARDWARE yaml's allocator (the mirror
sim yaml is section-2-identical, so both rigs allocate the same way; checked against the logged hardware
motors_debug), joint torques [13..16], the law's |e_R| [28..30], over the run (EXECUTING) window.

  hardware:  AM_NPZ=<dir> PYTHONNOUSERSITE=1 /usr/bin/python3 margins.py hw w3 w4 w5 w6
  sim:       /usr/bin/python3 margins.py sim <run.npz> ...        (numpy 2 for the pickled debug array)
"""
import json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))

# allocator (hardware 4-D yaml, FRD): rotor (px, py, km); f = kf w^2, command = (w - w_idle)/(w_max - w_idle)
ROT = np.array([[0.22990663, 0.22990663, 0.018164], [-0.22990663, -0.22990663, 0.018164],
                [0.22990663, -0.22990663, -0.018164], [-0.22990663, 0.22990663, -0.018164]])
KF, W_IDLE, W_MAX = 4.260431e-05, 64.0603, 730.0507
Bm = np.vstack([np.ones(4), -ROT[:, 1], ROT[:, 0], ROT[:, 2]])          # [u1; tau_frd] = B f
Binv = np.linalg.inv(Bm)
F_MAX = KF * W_MAX ** 2


def rotor_cmd(u1, tau_flu):
    tau_frd = np.column_stack([tau_flu[:, 0], -tau_flu[:, 1], -tau_flu[:, 2]])
    f = (Binv @ np.column_stack([u1, tau_frd]).T).T
    w = np.sqrt(np.clip(f, 0, None) / KF)
    return np.clip((w - W_IDLE) / (W_MAX - W_IDLE), -1, 2), f


def yaw_budget(u1):
    """largest |tau_z| the allocator can add at collective u1 with no rotor below idle or above max"""
    fi = u1 / 4.0; f_idle = KF * W_IDLE ** 2
    return 4 * 0.018164 * np.minimum(fi - f_idle, F_MAX - fi)


def summarise(t, D, motors=None):
    u1 = D[:, 17]; tau = D[:, 21:24]; tj = D[:, 13:17]; eR = np.linalg.norm(D[:, 28:31], axis=1)
    c, f = rotor_cmd(u1, tau)
    p = lambda x, q=99.5: float(np.percentile(x, q))
    out = dict(n=int(len(t)), u1_min=float(u1.min()), u1_max=float(u1.max()),
               tau_x_p99=p(np.abs(tau[:, 0])), tau_y_p99=p(np.abs(tau[:, 1])), tau_z_p99=p(np.abs(tau[:, 2])),
               tau_z_max=float(np.abs(tau[:, 2]).max()),
               yaw_budget_min=float(yaw_budget(u1).min()),
               yaw_use_p99_pct=100 * p(np.abs(tau[:, 2]) / yaw_budget(u1)),
               rotor_cmd_min=float(c.min()), rotor_cmd_max=float(c.max()),
               rotor_cmd_p01=float(np.percentile(c, 0.5)), rotor_cmd_p99=float(np.percentile(c, 99.5)),
               tau_joint_max=np.abs(tj).max(0).tolist(), eR_max=float(eR.max()), eR_rms=float(np.sqrt(np.mean(eR ** 2))),
               sat=int((D[:, 51] > 0).sum()))
    if motors is not None:
        out["motors_logged_min"] = float(np.nanmin(motors)); out["motors_logged_max"] = float(np.nanmax(motors))
    return out


def hw(names):
    sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_vs_decoupled_figure8_flight_20261005", "tools"))
    import f1005 as F
    res = {}
    for nm in names:
        d, t0 = F.load(nm); a, b, c0, c1 = F.windows(nm)
        tw = d["wb__recv"] - t0; D = d["wb__data"]; m = (tw > c0) & (tw < c1)
        tm = d["motors__recv"] - t0; mm = (tm > c0) & (tm < c1); mot = d["motors__control"][mm, :4]
        r = summarise(tw[m], D[m], mot)
        # allocator check: rebuild at the logged motor stamps
        cm, _ = rotor_cmd(np.interp(tm[mm], tw[m], D[m, 17]), np.column_stack([np.interp(tm[mm], tw[m], D[m, 21 + i]) for i in range(3)]))
        r["alloc_check_rms"] = float(np.sqrt(np.nanmean((cm - mot) ** 2)))
        res[nm] = r
    return res


def sim(paths):
    res = {}
    for p in paths:
        d = np.load(p, allow_pickle=True)
        marks = dict(str(x).split("=") for x in d["marks"]); t0, t1 = float(marks["run_start"]), float(marks["run_end"])
        raw = d["dbg"]
        if raw.dtype == object:
            n = max(len(x) for x in raw); D = np.array([np.asarray(x, float) for x in raw if len(x) == n])
        else:
            D = np.asarray(raw, float)
        m = (D[:, 0] >= t0) & (D[:, 0] <= t1)
        res[os.path.basename(p)] = summarise(D[m, 0], D[m, 1:])
    return res


def show(res):
    for k, r in res.items():
        print(f"{k:22s} u1 {r['u1_min']:5.1f}..{r['u1_max']:5.1f} N  tau p99 x/y/z {r['tau_x_p99']:.3f}/{r['tau_y_p99']:.3f}/{r['tau_z_p99']:.3f} "
              f"(z max {r['tau_z_max']:.3f}, budget >= {r['yaw_budget_min']:.2f}, use p99 {r['yaw_use_p99_pct']:.0f}%)  rotor cmd {r['rotor_cmd_min']:.3f}..{r['rotor_cmd_max']:.3f}"
              + (f" [logged {r['motors_logged_min']:.3f}..{r['motors_logged_max']:.3f}, rebuild rms {r['alloc_check_rms']:.4f}]" if "motors_logged_min" in r else "")
              + f"  tau_j max {np.round(r['tau_joint_max'], 2)}  |e_R| rms/max {r['eR_rms']:.3f}/{r['eR_max']:.3f}  sat {r['sat']}")


if __name__ == "__main__":
    mode, args = sys.argv[1], sys.argv[2:]
    out = sys.argv[sys.argv.index("--json") + 1] if "--json" in args else None
    args = [a for a in args if a != "--json" and a != out]
    res = hw(args) if mode == "hw" else sim(args)
    show(res)
    if out: json.dump(res, open(out, "w"), indent=1)

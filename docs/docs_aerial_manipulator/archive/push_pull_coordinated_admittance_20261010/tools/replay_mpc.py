#!/usr/bin/env python3
"""replay_mpc.py -- re-run the coordinated planner's MPC chain offline on a
recorded run (runs/<tag>.npz + runs/<tag>.yaml) in the planner's "full" mode
(x0 = its own previous command), with the recorded admittance output P_r, the
recorded weights and q_d, and the recorded attitude. Prints the offline command
beside the recorded one, so a solver change can be judged before a flight.

    PYTHONWARNINGS=ignore /usr/bin/python3 tools/replay_mpc.py runs/cadm_08 [--t-end 8] [--every 10]
"""
import argparse, math, os, sys
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_controller import coordinated_admittance as CA  # noqa: E402

np.set_printoptions(precision=3, suppress=True, linewidth=220)


def mpc_from_yaml(path):
    p = yaml.safe_load(open(path))["/**/coordinated_admittance_planner"]["ros__parameters"]
    g = lambda k: p[k]  # noqa: E731
    return CA.MpcConfig(dt=float(g("mpc_dt")), n_p=int(g("mpc_n_p")), n_c=int(g("mpc_n_c")),
                        k_p=float(g("mpc_k_p")), w_e=np.array(g("mpc_w_e")), w_c=np.array(g("mpc_w_c")),
                        w_u_base=np.array(g("mpc_w_u_base")), w_u_scale=float(g("mpc_w_u_scale")),
                        w_du=float(g("mpc_w_du")), w_psi=float(g("mpc_w_psi")),
                        w_q_post=np.array(g("mpc_w_q_post")), u_max=np.array(g("mpc_u_max")),
                        du_max=np.array(g("mpc_du_max")), v_max=np.array(g("mpc_v_max")),
                        p_min=np.array(g("mpc_p_min")), p_max=np.array(g("mpc_p_max")),
                        q_min=np.radians(np.array(g("mpc_q_min_deg"))),
                        q_max=np.radians(np.array(g("mpc_q_max_deg"))),
                        ee_body_min=np.array(g("mpc_ee_body_min")), ee_body_max=np.array(g("mpc_ee_body_max")),
                        l_m=float(g("mpc_l_m")), base_com=np.array(p.get("base_com", [0.0, -0.017854, 0.0])))


def R_of(od, t):
    k = np.argmin(np.abs(od[:, 0] - t))
    w, x, y, zz = od[k, 7:11]
    return np.array([[1 - 2 * (y * y + zz * zz), 2 * (x * y - zz * w), 2 * (x * zz + y * w)],
                     [2 * (x * y + zz * w), 1 - 2 * (x * x + zz * zz), 2 * (y * zz - x * w)],
                     [2 * (x * zz - y * w), 2 * (y * zz + x * w), 1 - 2 * (x * x + y * y)]])


def replay(tag, mcfg=None, qp=None, t_end=8.0, every=10, quiet=False, clamp_x0=None):
    z = np.load(tag + ".npz", allow_pickle=True)
    mcfg = mcfg or mpc_from_yaml(tag + ".yaml")
    c, od, ref = z["cadm"], z["odom"], z["ref"]
    mpc = CA.CoordinatedMpc(mcfg, qp)
    up = np.zeros(7)
    last = None
    rows = []
    for k in range(len(c)):
        r = c[k]
        t = r[0]
        if t - c[0, 0] > t_end:
            break
        x0 = np.concatenate([r[57:64], np.zeros(7)]) if last is None else np.concatenate([last["h1"], last["hdot1"]])
        if clamp_x0 is not None:
            x0 = clamp_x0(x0, mcfg)
        R = R_of(od, t)
        psi_b = math.atan2(R[1, 0], R[0, 0]) - math.pi / 2
        kr = np.argmin(np.abs(ref[:, 0] - t))
        psi_e = math.atan2(ref[kr, 17], ref[kr, 16])
        out = mpc.solve(x0, R, r[17:20], np.zeros(3), psi_b, psi_e, r[52:56], r[7:14], up)
        up = out["u0"]
        last = out
        km = np.argmin(np.abs(od[:, 0] - t))
        rows.append((t - c[0, 0], out["info"]["iters"], out["info"]["r_prim"], np.linalg.norm(out["u0"][:3]),
                     np.linalg.norm(r[17:20] - out["p_e_now"]), np.linalg.norm(r[17:20] - r[23:26])))
        if not quiet and k % every == 0:
            print(f"  t={t - c[0, 0]:5.2f} off: it {out['info']['iters']:4d} rp {out['info']['r_prim']:.0e} u0 {out['u0'][:3]} "
                  f"q {np.degrees(out['h1'][3:]).round(1)} |Pr-Pe| {rows[-1][4] * 1e3:5.0f} | rec: it {r[47]:3.0f} u0 {r[40:43]} "
                  f"q {np.degrees(r[29:33]).round(1)} |Pr-Pe| {rows[-1][5] * 1e3:5.0f} | meas q {np.degrees(r[60:64]).round(1)} "
                  f"|cmd-meas| {np.linalg.norm(r[26:29] - od[km, 1:4]) * 1e3:4.0f} mm")
    a = np.array(rows)
    print(f"  summary {os.path.basename(tag)} ({a[-1, 0]:.1f} s): offline iters mean {a[:, 1].mean():.0f} (400 on {np.mean(a[:, 1] >= 400) * 100:.0f}%), "
          f"r_prim max {a[:, 2].max():.1e}, |u0| mean {a[:, 3].mean():.2f}, |Pr-Pe| rms {np.sqrt((a[:, 4] ** 2).mean()) * 1e3:.0f} mm "
          f"(recorded {np.sqrt((a[:, 5] ** 2).mean()) * 1e3:.0f} mm)")
    return a


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("tags", nargs="+")
    ap.add_argument("--t-end", type=float, default=8.0)
    ap.add_argument("--every", type=int, default=10)
    a = ap.parse_args()
    for tag in a.tags:
        print(f"=== {tag}")
        replay(tag, t_end=a.t_end, every=a.every)

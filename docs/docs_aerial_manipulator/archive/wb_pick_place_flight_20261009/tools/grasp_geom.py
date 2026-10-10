"""Claw vs basket geometry through the pick and the place of each flight.

claw  = the model's grasp point by FK (planner model), from (a) the fused odometry the planner/law use and
        (b) the vehicle's mocap pose (same frame as obj_0), both with the measured joints
rel   = Rz(yaw_obj)^T (claw - obj_0): the claw in the basket's own frame -> compare with the EE Offset
prints the series at 0.25 s steps around the pick and the place, and key instants.
"""
from pp_common import *
from scipy.spatial.transform import Rotation as Rot


def series(nm):
    d, t0 = load(nm)
    to, P, Q, V, W = odom(d, t0)
    tm, Pm, Qm, Vm = mocap_pose(d, t0, "mocap")
    tb, Pb, Qb, Vb = mocap_pose(d, t0, "obj")
    tj, qj, _ = joints_model(d, t0)
    tg, G = gripper(d, t0)
    t = np.arange(max(to[0], tm[0], tb[0], tj[0]) + 0.05, min(to[-1], tm[-1], tb[-1], tj[-1]) - 0.05, 0.02)
    q = interp(t, tj, qj)
    Qo = quat_cont(interp(t, to, Q)); Qo /= np.linalg.norm(Qo, axis=1)[:, None]
    Qv = quat_cont(interp(t, tm, Qm)); Qv /= np.linalg.norm(Qv, axis=1)[:, None]
    xc_o, re_o, b1_o, _ = fk_world(interp(t, to, P), Qo, q)
    xc_m, re_m, b1_m, _ = fk_world(interp(t, tm, Pm), Qv, q)
    pb = interp(t, tb, Pb); Qbi = quat_cont(interp(t, tb, Qb)); Qbi /= np.linalg.norm(Qbi, axis=1)[:, None]
    yb = yaw_of(Qbi); tiltb = tilt_of(Qbi)
    c, s = np.cos(yb), np.sin(yb)
    def rel(re):
        dxy = re - pb
        return np.column_stack([c * dxy[:, 0] + s * dxy[:, 1], -s * dxy[:, 0] + c * dxy[:, 1], dxy[:, 2]])
    g = interp(t, tg, G)
    return dict(d=d, t0=t0, t=t, q=q, re_o=re_o, re_m=re_m, pb=pb, yb=yb, tiltb=tiltb, rel_o=rel(re_o), rel_m=rel(re_m),
                g=g, Pm=interp(t, tm, Pm), Po=interp(t, to, P), Qv=Qv, Qo=Qo, b1_o=b1_o, b1_m=b1_m, xc_o=xc_o)


if __name__ == "__main__":
    for nm in RUNS:
        S = series(nm); d, t0 = S["d"], S["t0"]
        ph = pp_phases(d, t0)
        print("=====", nm, RUNS[nm][0])
        for a, b, lab in ph:
            print(f"   {a:7.2f} {b:7.2f}  {lab}")
        t = S["t"]
        def show(ta, tb_, step=0.5):
            for ts in np.arange(ta, tb_, step):
                i = np.searchsorted(t, ts)
                if i >= len(t): break
                print(f"   t={t[i]:7.2f} obj [{S['pb'][i,0]:7.3f} {S['pb'][i,1]:7.3f} {S['pb'][i,2]:6.3f}] yaw {np.degrees(S['yb'][i]):7.2f} tilt {S['tiltb'][i]:5.2f} | "
                      f"claw-in-basket mocapFK [{1e3*S['rel_m'][i,0]:7.1f} {1e3*S['rel_m'][i,1]:7.1f} {1e3*S['rel_m'][i,2]:7.1f}] odomFK [{1e3*S['rel_o'][i,0]:7.1f} {1e3*S['rel_o'][i,1]:7.1f} {1e3*S['rel_o'][i,2]:7.1f}] mm | grip {np.round(S['g'][i],3)} | q {np.round(np.degrees(S['q'][i]),1)}")
        for a, b, lab in ph:
            if lab.startswith("WAITING execute_pick"):
                print("  -- pick: from the end of the wait to 8 s after"); show(b - 2.0, b + 12.0)
            if lab.startswith("WAITING execute_place"):
                print("  -- place: from the end of the wait"); show(b - 1.0, b + 20.0)

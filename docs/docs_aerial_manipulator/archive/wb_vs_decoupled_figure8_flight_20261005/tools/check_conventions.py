"""Verify the conventions common.py assumes on the 1005 figure-8 bags (the 0928 check, run on the new runs): joint order/sign against the whole-body law, odometry velocity frame, FK against the planner current_ee and the law x_c / task error, and the airframe-reference conversion against the decoupled bridge."""
import f1005
from f1005 import *
for nm in ("w3", "w5", "r5", "r6"):
    d, t0 = load(nm); a, b, c0, c1 = windows(nm)
    print(f"\n== {nm}  DIRECT {a:.2f}-{b:.2f}  circle {c0:.2f}-{c1:.2f}")
    print("  js names:", d["js__name"][100])
    tj, qj, vj = joints_model(d, t0)
    if "wb__recv" in d.files:
        tw = d["wb__recv"] - t0; D = d["wb__data"]; m = (tw > c0) & (tw < c1)
        qw = D[m, 5:9]; qi = interp(tw[m], tj, qj)
        print("  joints: model(js) - wb[5..8] mean/rms deg", np.degrees((qi - qw).mean(0)), np.degrees(rms(qi - qw)))
    to, P, Q, V, W = odom(d, t0); m = (to > c0) & (to < c1)
    # velocity frame: compare world-frame twist with the position derivative
    tu = np.arange(c0, c1, 0.01); Pu = interp(tu, to, P); Vd = np.gradient(Pu, 0.01, axis=0)
    Vu = interp(tu, to, V)
    print("  odom twist vs dP/dt rms (world) m/s", rms(Vu - Vd), " |V| rms", rms(Vu))
    # FK vs planner current_ee (15 Hz)
    tc = d["pl_current_ee__recv"] - t0; mc = (tc > c0) & (tc < c1)
    Pe = np.column_stack([d[f"pl_current_ee__pose.position.{k}"] for k in "xyz"])[mc]
    Pc = interp(tc[mc], to, P); Qc = quat_cont(interp(tc[mc], to, Q)); Qc /= np.linalg.norm(Qc, axis=1)[:, None]
    qc = interp(tc[mc], tj, qj)
    xc, re, b1e, _ = fk_world(Pc, Qc, qc)
    print("  FK r_e - planner current_ee mm mean/rms", 1e3 * (re - Pe).mean(0), 1e3 * rms(re - Pe))
    if "wb__recv" in d.files:
        # CoM: law x_c (wb[48..50]) vs FK x_c
        tw = d["wb__recv"] - t0; D = d["wb__data"]; mw = (tw > c0) & (tw < c1)
        Pw = interp(tw[mw], to, P); Qw = quat_cont(interp(tw[mw], to, Q)); Qw /= np.linalg.norm(Qw, axis=1)[:, None]
        qw = D[mw, 5:9]
        xc, re, b1e, _ = fk_world(Pw, Qw, qw)
        print("  FK x_c - law x_c (wb[48..50]) mm mean/rms", 1e3 * (xc - D[mw, 48:51]).mean(0), 1e3 * rms(xc - D[mw, 48:51]))
        # EE: FK r_e - (x_cd + e_y + (x_c - x_cd)) i.e. law r_e = x_c + e_y ... check e_y sign
        tr, ref = wbref(d, t0); red = interp(tw[mw], tr, ref["r_ed"]); xcd_s = interp(tw[mw], tr, ref["x_cd"])
        print("  law x_cd (wb[45..47]) - stream x_cd mm rms", 1e3 * rms(D[mw, 45:48] - xcd_s))
        eabs = re - red; e_law = D[mw, 24:27] + (D[mw, 48:51] - D[mw, 45:48])
        print("  EE abs err FK vs (e_y + e_com) mm mean/rms of diff", 1e3 * (eabs - e_law).mean(0), 1e3 * rms(eabs - e_law), " |eabs| rms", 1e3 * rms(np.linalg.norm(eabs, axis=1)))
    else:
        tr, ref = wbref(d, t0); pr = d["pcref_dir__recv"] - t0
        xb, R0, yaw = ref_base(ref)
        pc = np.column_stack([d[f"pcref_dir__position.{k}"] for k in "xyz"])
        mm = (pr > c0) & (pr < c1)
        print("  bridge x_b - my x_b (interp at bridge recv) mm rms", 1e3 * rms(pc[mm] - interp(pr[mm], tr, xb)))
        yb = d["pcref_dir__yaw"][mm]; print("  bridge yaw - my yaw deg rms", np.degrees(rms(wrap(yb - np.interp(pr[mm], tr, np.unwrap(yaw))))))
        L = d["l1__data"]; tl = d["l1__recv"] - t0; ml = (tl > c0) & (tl < c1)
        # l1 e_p = p - p_ref ? compare with odom - bridge ref
        ep = interp(tl[ml], to, P) - interp(tl[ml], pr, pc)
        print("  l1 e_p [0..2] vs (odom - bridge ref) mm mean", 1e3 * L[ml, 0:3].mean(0), 1e3 * ep.mean(0), " corr", [np.corrcoef(L[ml, k], ep[:, k])[0, 1] for k in range(3)])

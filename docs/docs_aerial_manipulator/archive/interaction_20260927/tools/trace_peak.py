import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import interaction_bench as IB
for chi, rd in [("gripper", None), ("gripper", 5.0), ("free", None)]:
    r = IB.simulate("payload", chi, m=0.1, rd=rd, keep=True)
    H, H3, mk = r["H"], r["H3"], r["marks"]; t = H["t"]
    act = t > 14.9
    i = np.argmax(np.where(act, H["ex"], 0)); j = np.argmax(np.where(act, H["ee"], 0))
    print(f"{chi} rd={rd}: ex_pk {H['ex'][i]*1e3:.0f} mm at t={t[i]:.2f} ex={np.round(H3['e_x'][i]*1e3)}; ee_pk {H['ee'][j]*1e3:.0f} at {t[j]:.2f}")
    for a, b, nm in [(15, 17, "grasp"), (17, 21, "lift"), (21, 27, "carry"), (28, 31, "descend"), (31, 39, "press"), (39, 41, "open"), (41, 44, "rise"), (44, 55, "after")]:
        w = (t >= a) & (t < b)
        print(f"   {nm:8} ex pk {H['ex'][w].max()*1e3:5.0f}  ee pk {H['ee'][w].max()*1e3:5.0f}  tilt pk {H['tilt'][w].max():4.1f}  Fhat_z [{H3['F_hat_f'][w,2].min():5.2f},{H3['F_hat_f'][w,2].max():5.2f}] Ftrue_z [{H3['F_true'][w,2].min():5.2f},{H3['F_true'][w,2].max():5.2f}]")

import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import interaction_bench as IB
for chi in ["gripper", "free"]:
    r = IB.simulate("payload", chi, m=0.1, keep=True)
    H, H3, H4, mk = r["H"], r["H3"], r["H4"], r["marks"]
    t = H["t"]
    print(f"=== {chi}  marks", {k: round(v, 2) for k, v in mk.items()})
    for tt in np.arange(mk["t_close"] - 1, mk["t_rise"] + 4, 1.0):
        i = np.searchsorted(t, tt)
        if i >= len(t): break
        print(f"t={t[i]:5.1f} chi={int(H['chi'][i])} ex={np.round(H3['e_x'][i]*1e3,1)} ee={np.round(H3['e_ee'][i]*1e3,1)} "
              f"Ftrue_z={H3['F_true'][i,2]:6.2f} Fhat={np.round(H3['F_hat_f'][i],2)} Fraw_z={H3['F_raw'][i,2]:6.2f} dt_z={H3['d_t'][i,2]:6.2f} "
              f"tau={np.round(H4['tau'][i],3)} wq={np.round(H4['wq'][i],3)}")

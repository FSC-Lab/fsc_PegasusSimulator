import sys, numpy as np, modular_bench as MB
from fsc_aerial_manipulation.robotic_arm.utils_controller import modular_adaptive as MA
def show(cfg, name, **kw):
    r = MB.run(cfg, keep=True, **kw)
    H = r["H"]; Hq = r["Hq"]; Hqd = r["Hqd"]
    print(name, r["verdict"], "t_end", round(r["t_end"],2))
    idx = np.linspace(0, len(H["t"])-1, 12).astype(int)
    for i in idx:
        print(f"  t={H['t'][i]-H['t'][0]:6.2f} ex={H['ex'][i]*1e3:7.1f}mm tilt={H['tilt'][i]:5.1f} eR={H['eR'][i]:.3f} "
              f"qerr={np.round(np.degrees(Hq[i]-Hqd[i]),1)} tau={H['tau'][i]:.2f}")
    return r
if __name__ == "__main__":
    show(MA.table2_config(), "table2")

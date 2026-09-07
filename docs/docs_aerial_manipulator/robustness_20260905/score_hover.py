import json, sys, numpy as np
def score(f, name):
    d=json.load(open(f)); R=[r for r in d["rows"] if r["phase"]=="DIRECT"]
    if not R: return print(f"{name:28} NO DIRECT SAMPLES  ({d.get('reason','')})")
    t=np.array([r["t"] for r in R]); t-=t[0]
    ce=np.linalg.norm(np.array([r["x_c"] for r in R])-np.array([r["x_cd"] for r in R]),axis=1)
    ey=np.linalg.norm(np.array([r["e_y"] for r in R])[:,:3],axis=1)
    eR=np.linalg.norm(np.array([r["e_R"] for r in R]),axis=1)
    tilt=np.array([r["tilt"] for r in R]); tau=np.array([r["tau"] for r in R])
    ns=np.array([r["n_sat"] for r in R]); u1=np.array([r["u1"] for r in R])
    qm=np.degrees(np.array([r["qm"] for r in R])); L=slice(-int(len(R)*0.2),None)
    ok = "STABLE " if not d["aborted"] else "DIVERGED"
    print(f"{name:28} {ok}  t={t[-1]:5.1f}s  CoM {ce[L].mean()*1000:7.2f} mm (max {ce.max()*1000:7.1f})"
          f"  EE {ey[L].mean()*1000:6.2f} mm  tilt {tilt[L].mean():5.3f}/{tilt.max():5.2f} deg"
          f"  |eR|max {eR.max():.4f}  tau {np.abs(tau).max():.2f}  clamp {100*(ns>0).mean():5.2f}%"
          f"  u1 {u1[L].mean():6.2f} N  q {np.round(qm[L].mean(axis=0),1).tolist()}"
          + ("" if not d["aborted"] else f"   <- {d['reason']}"))
for a in sys.argv[1:]:
    f, n = a.split("=", 1); score(f, n)

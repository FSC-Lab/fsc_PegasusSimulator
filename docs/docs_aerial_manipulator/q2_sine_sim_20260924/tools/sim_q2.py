"""Score an EE-circle run (sim or hardware npz from extract_bag.py) on the arm, EE and base.
   PYTHONNOUSERSITE=1 /usr/bin/python3 sim_q2.py <name> [<name> ...]   (npz in the scratch dir)"""
import sys; sys.path.insert(0,"<scratch>")
from common import *
Q_MAX=np.array([35.0,50.0,50.0,120.0]); Q_MIN=np.array([-35.0,-80.0,-40.0,-120.0])
for nm in sys.argv[1:]:
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]
    print(f"\n################ {nm}: DIRECT {a:.1f}-{b:.1f} s ({b-a:.1f} s)")
    for ti,s in zip(d["pl_status__recv"]-t0,d["pl_status__data"]): print(f"   planner {ti:7.2f} {s}")
    if "ee_status__data" in d:
        for ti,s in zip(d["ee_status__recv"]-t0,d["ee_status__data"]): print(f"   ee_traj {ti:7.2f} {str(s)[:110]}")
    run=[(x,s) for x,s in ex if "T=" in s]; r=[x for x,s in ex]
    # circle run = the EXECUTING with the longest T
    Ts=[float(s.split("T=")[1].rstrip("s")) for x,s in ex]; k=int(np.argmax(Ts)); r0=ex[k][0]; r1=r0+Ts[k]
    g0=ex[k-1][0] if k>0 else None
    print(f"   circle run {r0:.1f}-{r1:.1f} s (T={Ts[k]:.1f}); go-to-start at {g0}")
    m=(t>r0)&(t<min(r1,b-0.05)); q=np.degrees(D[:,5:9]); qd=np.degrees(D[:,9:13]); tau=D[:,13:17]
    print("   joint | q_d range | q range | realised span % | err mean/rms/max deg | lag ms | tau mean/max | margin to stop deg (min over run)")
    for j in range(4):
        e=q[m,j]-qd[m,j]; x=q[m,j]-q[m,j].mean(); y=qd[m,j]-qd[m,j].mean(); best=(0,-2)
        if y.std()>0.5:
            for L in range(0,500,2):
                c=np.corrcoef(x[L:],y[:len(y)-L])[0,1]
                if c>best[1]: best=(L,c)
        span_d=qd[m,j].max()-qd[m,j].min(); span=q[m,j].max()-q[m,j].min()
        marg=min(Q_MAX[j]-q[m,j].max(), q[m,j].min()-Q_MIN[j])
        print(f"   q{j+1}    | {qd[m,j].min():6.1f}..{qd[m,j].max():6.1f} | {q[m,j].min():6.1f}..{q[m,j].max():6.1f} | {100*span/span_d if span_d>0.5 else float('nan'):5.0f} | {e.mean():+5.2f}/{np.sqrt((e**2).mean()):5.2f}/{np.abs(e).max():5.2f} | {best[0]*4:4d} (c {best[1]:.2f}) | {tau[m,j].mean():+.3f}/{np.abs(tau[m,j]).max():.3f} | {marg:5.1f}")
    qdot_d=np.gradient(qd[m],t[m],axis=0); print(f"   peak |qdot_d| deg/s {np.abs(qdot_d).max(0).round(1)}; peak |qdot| (FD of q) {np.abs(np.gradient(q[m],t[m],axis=0)).max(0).round(1)}")
    ep=(D[m,48:51]-D[m,45:48])*1000; ey=D[m,24:27]*1000; ab=ep+ey; hd=np.degrees(np.arcsin(np.clip(D[m,27],-1,1)))
    eR=np.linalg.norm(D[m,28:31],axis=1)
    print(f"   base CoM err rms {np.sqrt((ep**2).mean(0)).round(1)} peak {np.linalg.norm(ep,axis=1).max():.0f} mm")
    print(f"   EE task err rms {np.sqrt((ey**2).mean(0)).round(1)} max {np.abs(ey).max(0).round(1)} mm | EE absolute rms {np.sqrt((ab**2).mean(0)).round(1)} max {np.abs(ab).max(0).round(0)} mm | yaw rms {np.sqrt((hd**2).mean()):.2f} max {np.abs(hd).max():.2f} mean {hd.mean():+.2f} deg")
    print(f"   |e_R| mean {eR.mean():.3f} max {eR.max():.3f} | u1 {D[m,17].mean():.1f}+-{D[m,17].std():.2f} | motors {D[m,41:45].min():.2f}..{D[m,41:45].max():.2f} | n_sat {(D[m,51]>0).sum()} | tau clamp {(np.abs(D[m,13:17])>2.99).sum()} | est-bound {(D[m,88]>0).sum()} | stream fresh {np.mean(D[m,56]>0.5):.3f}")
    if "vatt__q" in d:
        tv=d["vatt__recv"]-t0; mv=(tv>r0)&(tv<r1); print(f"   tilt max {tilt_deg(d['vatt__q'][mv]).max():.2f} deg")
    # EE radius from the law-derived absolute EE
    tr=d["wbref__recv"]-t0; red=np.column_stack([d[f"wbref__r_ed.{k}"] for k in "xyz"]); redi=np.column_stack([np.interp(t[m],tr,red[:,k]) for k in range(3)])
    ee=redi+ab/1000; mr=(t[m]>r0+4)&(t[m]<r1-4); rr=np.hypot(ee[mr,0],ee[mr,1]); print(f"   EE radius (steady part) {rr.mean():.3f} +- {rr.std()*1000:.0f} mm (plan 0.500); EE z {ee[mr,2].mean():.3f}")
    # completion
    tails=[(round(x,1),s) for x,s in hold if x>r0]; print(f"   after run: {tails[:2]} ; DIRECT ended {b:.1f} s (run end planned {r1:.1f})")

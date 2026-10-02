import sys; sys.path.insert(0,"<scratch>")
from common import *
for nm in ["a3","a4"]:
    d,t0,a,b,ex,hold=load(nm)
    tv=d["vrpn__hdr"]-t0; tvr=d["vrpn__recv"]-t0; Pv=np.column_stack([d[f"vrpn__pose.position.{k}"] for k in "xyz"]); Qv=np.column_stack([d[f"vrpn__pose.orientation.{k}"] for k in "wxyz"])
    dt=np.diff(tv); Ev=eul(Qv)
    print(f"\n#### {nm}: /vrpn_mocap/uav_0/pose n={len(tv)} over {tv[-1]-tv[0]:.1f} s -> {len(tv)/(tv[-1]-tv[0]):.1f} Hz; hdr-recv median {np.median(tvr-tv)*1000:.2f} ms")
    print("   stamp dt (ms) p1/10/50/90/99/max:",np.percentile(dt*1000,[1,10,50,90,99]).round(2),(dt.max()*1000).round(1),"| dt<=1ms pairs",(dt<=0.001).sum(),"| identical consecutive poses",np.mean(np.all(np.diff(Pv,axis=0)==0,axis=1)).round(4))
    g=np.where(dt>0.020)[0]; print(f"   gaps >20 ms: {len(g)}  (in DIRECT: {((tv[g]>a)&(tv[g]<b)).sum()})")
    for i in g[:25]:
        m=(tv>tv[i]-0.06)&(tv<=tv[i]); v=np.polyfit(tv[m]-tv[i],Pv[m],1)[0] if m.sum()>=3 else np.zeros(3)
        ex_=Pv[i]+v*dt[i]; print(f"      t={tv[i]:7.3f} dur {dt[i]*1000:6.1f} ms  yaw {Ev[i,2]:7.1f}  z {Pv[i,2]:.2f}  next-frame vs extrapolation {np.linalg.norm(Pv[i+1]-ex_)*1000:5.1f} mm  vs last {np.linalg.norm(Pv[i+1]-Pv[i])*1000:5.1f} mm  {'DIRECT' if a<tv[i]<b else ''}")
    stp=qstep(Qv); print(f"   orientation steps >3 deg: {(stp>3).sum()} (DIRECT {((stp>3)&(tv[1:]>a)&(tv[1:]<b)).sum()}); pos steps >5 cm: {(np.linalg.norm(np.diff(Pv,axis=0),axis=1)>0.05).sum()}")
    # per-heading skip rate in DIRECT
    mD=(tv[:-1]>a)&(tv[:-1]<b); print("   DIRECT per-heading: sector | n | dt>12ms | dt>20ms")
    for lo in range(-180,180,45):
        s=mD&(Ev[:-1,2]>=lo)&(Ev[:-1,2]<lo+45)
        if s.sum()>50: print(f"      [{lo:4d},{lo+45:4d}) {s.sum():5d} {(dt[s]>0.012).sum():4d} {(dt[s]>0.020).sum():3d}")
    # processed /uav_0/mocap: frozen runs + NO DATA
    tm=d["mocap__hdr"]-t0; P=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); rep=np.all(np.diff(P,axis=0)==0,axis=1)
    runs=[];k=0
    while k<len(rep):
        if rep[k]:
            j=k
            while j<len(rep) and rep[j]: j+=1
            if j-k>=2: runs.append((round(tm[k],2),j-k,round(tm[j]-tm[k],3)))
            k=j
        else: k+=1
    print("   /uav_0/mocap frozen runs >=2:",runs[:10]); print("   /uav_0/mocap rate",len(tm)/(tm[-1]-tm[0]),"; NO DATA lines:",sum("NO DATA" in m_ for m_ in d["rosout__msg"]), "; mocap_status uniq",np.unique(d["mocap_status__data"]))
    # vrpn vs mocap: does every vrpn frame produce a mocap frame? compare counts and the position match
    Pm_i=np.column_stack([np.interp(tv,tm,P[:,k]) for k in range(3)]); print(f"   vrpn n {len(tv)} vs mocap n {len(tm)}; vrpn pos vs mocap(interp) rms {np.sqrt(((Pv-Pm_i)**2).mean(0))*1000} mm")
    # a4: the 74.18 stale-stream event
    if nm=="a4":
        t=d["wb__recv"]-t0; dtw=np.diff(t); i=np.argmax(dtw); print(f"   a4 law tick max gap {dtw.max()*1000:.0f} ms at {t[i]:.2f} s; tcmd gaps>50ms:",[(round((d['tcmd__recv']-t0)[k],2),round(x*1000)) for k,x in enumerate(np.diff(d['tcmd__recv'])) if x>0.05][:5])
        tmo=d["wbmode__recv"]-t0; print("   mode edges:",[(round(x,2),y) for x,y in zip(tmo,d["wbmode__data"]) if 70<x<77][:10])

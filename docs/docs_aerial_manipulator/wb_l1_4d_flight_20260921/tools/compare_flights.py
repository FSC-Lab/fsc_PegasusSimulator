import numpy as np, sys
np.set_printoptions(precision=3, suppress=True, linewidth=200)
from numpy.fft import rfft, rfftfreq
def eul(q):
    w,x,y,z=q.T
    return np.degrees(np.column_stack([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),np.arcsin(np.clip(2*(w*y-z*x),-1,1)),np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))]))
def peakf(x,fs):
    x=x-x.mean(); f=rfftfreq(len(x),1/fs); X=np.abs(rfft(x*np.hanning(len(x))))**2; return f[np.argmax(X[1:])+1], X[f>5].sum()/X[1:].sum()
for nm,fn in [("0918",f"{sys.argv[1]}/f18.npz"),("0921 #1 112328",f"{sys.argv[1]}/c1.npz"),("0921 #2 115207",f"{sys.argv[1]}/c2.npz"),("0921 #3 123637",f"{sys.argv[1]}/c3.npz")]:
    d=np.load(fn,allow_pickle=True); t0=d["wb__recv"][0]
    print(f"\n################ {nm} ################")
    tmo=d["wbmode__recv"]-t0; mo=d["wbmode__data"]; last=None; ed=[]
    for ti,vi in zip(tmo,mo):
        if vi!=last: ed.append((ti,vi)); last=vi
    print("mode edges:",[(round(a,2),b) for a,b in ed])
    for ti,n_,m_ in zip(d["rosout__recv"]-t0,d["rosout__name"],d["rosout__msg"]):
        if any(k in m_ for k in ["WATCHDOG","Reverting","saturated","Returned to SAFETY","NO DATA","stale","planned:","READY","Go To","DIRECT engaged","refus","INFEASIBLE","EXECUTING","gate","timeout"]): print(f"   {ti:7.2f} {n_.split('.')[-1]}: {m_[:150]}")
    st=[b for a,b in ed]; 
    if "DIRECT" in st:
        i=st.index("DIRECT"); a=ed[i][0]+0.5; b=(ed[i+1][0] if i+1<len(ed) else tmo[-1])-0.2
    else: a,b=1,10
    # hold window: DIRECT up to first EXECUTING status or full
    ps=d["pl_status__data"]; tps=d["pl_status__recv"]-t0
    ex=[ti for ti,s in zip(tps,ps) if str(s).startswith("EXECUTING") and ti>a]
    bh=min(ex[0]-0.2,b) if ex else b
    print(f"DIRECT {a:.1f}-{b:.1f}; hold window {a:.1f}-{bh:.1f}; first EXECUTING at {ex[0] if ex else None}")
    tm=d["mocap__hdr"]-t0; dtm=np.diff(tm); P=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); Q=np.column_stack([d[f"mocap__pose.orientation.{k}"] for k in "wxyz"])
    n=np.linalg.norm(np.diff(P,axis=0),axis=1); ang=2*np.degrees(np.arccos(np.clip(np.abs(np.sum(Q[1:]*Q[:-1],axis=1)),-1,1)))
    print(f"mocap: rate {1/np.median(dtm):.0f} Hz, dt max {dtm.max()*1000:.0f} ms, gaps>50ms {(dtm>0.05).sum()}, identical-pos repeats {np.mean(np.all(np.diff(P,axis=0)==0,axis=1))*100:.1f}%, |dp|>0.05m spikes {(n>0.05).sum()} (in DIRECT {((n>0.05)&(tm[1:]>a)&(tm[1:]<b)).sum()}), orient step>3deg {(ang>3).sum()} ({((ang>3)&(tm[1:]>a)&(tm[1:]<b)).sum()} in DIRECT)")
    mh=(tm>a)&(tm<bh)
    print(f"   hold: mocap pos per-sample |dp| median {np.median(n[mh[1:]])*1000:.2f} mm; orient step median {np.median(ang[mh[1:]]):.3f} p99 {np.percentile(ang[mh[1:]],99):.2f} deg")
    to=d["odom__hdr"]-t0; Vo=np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"]); m=(to>a)&(to<bh)
    fo=1/np.median(np.diff(to)); pk=[peakf(Vo[m,k],fo) for k in range(3)]
    print(f"   odom {fo:.0f} Hz; hold vel std {Vo[m].std(0)} m/s; peak freq / frac>5Hz: {[(round(p[0],1),round(p[1],2)) for p in pk]}; |v| max in DIRECT {np.linalg.norm(Vo[(to>a)&(to<b)],axis=1).max():.2f}")
    Vfd=np.diff(P,axis=0)/np.diff(tm)[:,None]; print(f"   FD-of-mocap-pos hold vel std {Vfd[mh[1:]].std(0)}")
    t=d["wb__recv"]-t0; D=d["wb__data"]; mw=(t>a)&(t<bh); mD=(t>a)&(t<b)
    eR=np.linalg.norm(D[mw,28:31],axis=1); ep=D[mw,48:51]-D[mw,45:48]
    print(f"   law hold: |e_R| mean {eR.mean():.3f} p99 {np.percentile(eR,99):.3f} | e_R_x peak {peakf(D[mw,28],250)} | u1 mean {D[mw,17].mean():.2f} std {D[mw,17].std():.2f} peak {peakf(D[mw,17],250)} | motors std {D[mw,41:45].std(0)} | e_p rms {np.sqrt((ep**2).mean(0))*1000} mm")
    print(f"   d_hat filt [31..33] std {D[mw,31:34].std(0)} peak {peakf(D[mw,31],250)}; d_hat^c [62..64] std {D[mw,62:65].std(0)}; Fraw [97..99] std {D[mw,97:100].std(0)}; tau_j std {D[mw,13:17].std(0)} max {np.abs(D[mw,13:17]).max(0)}; tau_body FLU std {D[mw,21:24].std(0)}")
    print(f"   whole DIRECT: |e_R| max {np.linalg.norm(D[mD,28:31],axis=1).max():.3f}, u1 min/max {D[mD,17].min():.1f}/{D[mD,17].max():.1f}, motor min/max {D[mD,41:45].min():.2f}/{D[mD,41:45].max():.2f}, n_sat ticks {(D[mD,51]>0).sum()}, tau_j clamp ticks {(np.abs(D[mD,13:17])>2.99).sum()}, est-bound ticks {(D[mD,88]>0).sum()}")
    tv=d["vatt__recv"]-t0; Ev=eul(d["vatt__q"]); mv=(tv>a)&(tv<bh); fv=1/np.median(np.diff(tv))
    R22=1-2*(d["vatt__q"][:,1]**2+d["vatt__q"][:,2]**2); tilt=np.degrees(np.arccos(np.clip(R22,-1,1)))
    print(f"   PX4 attitude {fv:.0f} Hz: hold rp std {Ev[mv,:2].std(0)}, yaw std {Ev[mv,2].std():.2f}; tilt max in DIRECT {tilt[(tv>a)&(tv<b)].max():.2f}; roll frac>5Hz {peakf(Ev[mv,0],fv)[1]:.2f}")
    ti=d["imu__hdr"]-t0; G=np.column_stack([d[f"imu__angular_velocity.{k}"] for k in "xyz"]); mi=(ti>a)&(ti<bh); fi=1/np.median(np.diff(ti))
    print(f"   imu {fi:.0f} Hz: gyro hold std (deg/s) {np.degrees(G[mi].std(0))}; frac>5Hz x/y {peakf(G[mi,0],fi)[1]:.2f}/{peakf(G[mi,1],fi)[1]:.2f}")
    tsc=d["sc__recv"]-t0; print(f"   sensor_combined rate {1/np.median(np.diff(tsc)):.0f} Hz")
    if "batt__voltage_v" in d: print(f"   battery {d['batt__voltage_v'][0]:.2f} -> {d['batt__voltage_v'][-1]:.2f} V")

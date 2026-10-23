import sys; sys.path.insert(0,"<scratch>")
from common import *
import os
SEL=os.environ.get("SEL","f18,c3,a1,a2").split(",")
for nm,lab in [("f18","0918 raw-mocap 60Hz"),("c3","0921#3 raw-mocap 120Hz"),("a1","0924#1 FUSED (2 laps)"),("a2","0924#2 FUSED (1 lap)")]:
    if nm not in SEL: continue
    d,t0,a,b,ex,hold=load(nm)
    print(f"\n################ {nm}: {lab}  DIRECT {a:.2f}-{b:.2f} s ({b-a:.1f} s)")
    ah=a+0.5; bh=(ex[0][0]-0.2) if ex else b
    print(f"hold window {ah:.1f}-{bh:.1f}; executing: {[(round(t,1),s) for t,s in ex]}")
    # ---- mocap
    tm=d["mocap__hdr"]-t0; P=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); Q=np.column_stack([d[f"mocap__pose.orientation.{k}"] for k in "wxyz"]); V=np.column_stack([d[f"mocap__twist.linear.{k}"] for k in "xyz"])
    dtm=np.diff(tm); n=np.linalg.norm(np.diff(P,axis=0),axis=1); ang=qstep(Q)
    rep=np.all(np.diff(P,axis=0)==0,axis=1)
    inD=(tm[1:]>a)&(tm[1:]<b)
    print(f"mocap: {1/np.median(dtm):.0f} Hz | hdr dt max {dtm.max()*1000:.0f} ms, gaps>50ms {(dtm>0.05).sum()} (DIRECT {((dtm>0.05)&inD).sum()}) | identical-pos repeats {rep.mean()*100:.1f}% | |dp|>0.05 spikes {(n>0.05).sum()} (DIRECT {((n>0.05)&inD).sum()}) | orient step>3deg {(ang>3).sum()} (DIRECT {((ang>3)&inD).sum()})")
    gi=np.where(dtm>0.05)[0]
    if len(gi): print("   gaps (t, ms):", [(round(tm[i],2),int(dtm[i]*1000)) for i in gi][:20])
    # runs of identical positions (frozen) longer than 5 samples
    runs=[];k=0
    while k<len(rep):
        if rep[k]:
            j=k
            while j<len(rep) and rep[j]: j+=1
            if j-k>=5: runs.append((round(tm[k],2),j-k,round(tm[j]-tm[k],2)))
            k=j
        else: k+=1
    print(f"   frozen-pose runs >=5 samples: {len(runs)} ->", runs[:15])
    # ---- odom
    to=d["odom__hdr"]-t0; tor=d["odom__recv"]-t0
    off=np.median(d["odom__hdr"]-d["odom__recv"])
    if abs(off)>0.5: print(f"   NOTE odom header stamps are on another clock (hdr-recv median {off:.3f} s) -> using receive time"); to=tor
    else: print(f"   odom hdr-recv median {off*1000:.1f} ms")
    Po=np.column_stack([d[f"odom__pose.pose.position.{k}"] for k in "xyz"]); Vo=np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"])
    dto=np.diff(to); fo=1/np.median(dto)
    # identical to mocap?
    Pm_i=np.column_stack([np.interp(to,tm,P[:,k]) for k in range(3)])
    same=np.mean([np.any(np.all(np.abs(P-Po[i])<1e-9,axis=1)) for i in range(0,len(Po),max(1,len(Po)//400))])
    mD=(to>a)&(to<b); diff=Po-Pm_i
    # lag estimate odom vs mocap on x in DIRECT via cross-corr of velocity
    vm_i=np.column_stack([np.interp(to,tm,V[:,k]) for k in range(3)])
    def lag(x,y,fs,maxl=40):
        x=x-x.mean(); y=y-y.mean(); best=(0,-2)
        for L in range(-maxl,maxl+1):
            if L>=0: c=np.corrcoef(x[L:],y[:len(y)-L])[0,1]
            else: c=np.corrcoef(x[:L],y[-L:])[0,1]
            if c>best[1]: best=(L,c)
        return best[0]/fs*1000, best[1]
    lg=[lag(Po[mD,k],Pm_i[mD,k],fo) for k in range(3)]
    print(f"odom: {fo:.0f} Hz (hdr), recv-rate {1/np.median(np.diff(tor)):.0f} Hz | hdr dt max {dto.max()*1000:.0f} ms | frac samples bit-identical to a mocap sample {same*100:.0f}% | odom-mocap pos diff in DIRECT rms {np.sqrt((diff[mD]**2).mean(0))*1000} mm max {np.abs(diff[mD]).max(0)*1000} mm | odom lags mocap by (ms, corr) x/y/z {[(round(l,0),round(c,3)) for l,c in lg]}")
    mh=(to>ah)&(to<bh)
    print(f"   hold: odom vel std {Vo[mh].std(0)*100} cm/s | mocap twist std {V[(tm>ah)&(tm<bh)].std(0)*100} cm/s | odom identical-consecutive-pos {np.mean(np.all(np.diff(Po[mD],axis=0)==0,axis=1))*100:.1f}%")
    Vfd=np.diff(P,axis=0)/np.diff(tm)[:,None]; mhm=(tm[1:]>ah)&(tm[1:]<bh)
    print(f"   hold: FD-of-mocap vel std {Vfd[mhm].std(0)*100} cm/s | peak freq/frac>5Hz odom vx {peakf(Vo[mh,0],fo)} vy {peakf(Vo[mh,1],fo)}")
    # ---- EV input into PX4 and EKF flags
    if "vvo__recv" in d:
        tv=d["vvo__recv"]-t0; dtv=np.diff(tv); print(f"EV input (fmu/in/vehicle_visual_odometry): {1/np.median(dtv):.0f} Hz, gaps>50ms {(dtv>0.05).sum()} ->", [(round(tv[i],2),int(dtv[i]*1000)) for i in np.where(dtv>0.05)[0]][:12])
    if "esf__recv" in d:
        te=d["esf__recv"]-t0
        for f in ["cs_ev_pos","cs_ev_vel","cs_ev_yaw","cs_ev_hgt","cs_inertial_dead_reckoning","cs_in_air","cs_vehicle_at_rest"]:
            x=d[f"esf__{f}"]; ch=np.where(np.diff(x)!=0)[0]
            if len(ch) or f=="cs_ev_pos": print(f"   esf {f}: initial {x[0]:.0f}, changes at", [(round(te[i+1],2),int(x[i+1])) for i in ch][:12])
    if "vodom_px__recv" in d:
        tp=d["vodom_px__recv"]-t0; rc=d["vodom_px__reset_counter"]; ch=np.where(np.diff(rc)!=0)[0]
        print("   PX4 vehicle_odometry reset_counter changes:", [(round(tp[i+1],2),int(rc[i+1])) for i in ch]); 
        pv=d["vodom_px__position_variance"]; mp=(tp>a)&(tp<b); print(f"   PX4 EKF pos variance in DIRECT: median {np.median(pv[mp],axis=0)} max {pv[mp].max(0)}; vel std hold {d['vodom_px__velocity'][(tp>ah)&(tp<bh)].std(0)*100} cm/s")
    if "tsync__recv" in d:
        print(f"   timesync: n={len(d['tsync__recv'])}, estimated_offset us median {np.median(d['tsync__estimated_offset']):.0f} span {d['tsync__estimated_offset'].max()-d['tsync__estimated_offset'].min():.0f}, rtt us median {np.median(d['tsync__round_trip_time']):.0f} max {d['tsync__round_trip_time'].max():.0f}")
    # ---- rates and timing
    t=d["wb__recv"]-t0; D=d["wb__data"]; dt=np.diff(t[(t>a)&(t<b)])
    print(f"law tick (wb_control_debug recv) in DIRECT: {1/np.median(dt):.0f} Hz, dt p50/p99/max {np.percentile(dt,50)*1000:.2f}/{np.percentile(dt,99)*1000:.2f}/{dt.max()*1000:.1f} ms, ticks>8ms {(dt>0.008).sum()}")
    for key,lab2 in [("sc","sensor_combined"),("vatt","vehicle_attitude"),("imu","imu/data"),("motors","motors_debug"),("act_motors","fmu/in/actuator_motors"),("js","joint_states"),("wbref","wb reference")]:
        if f"{key}__recv" in d:
            tt=d[f"{key}__recv"]-t0; dd=np.diff(tt[(tt>a)&(tt<b)]); print(f"   {lab2:24s} {1/np.median(dd):6.1f} Hz  dt p99 {np.percentile(dd,99)*1000:6.2f} ms max {dd.max()*1000:6.1f} ms")
    # ---- attitude / gyro noise in hold
    tv=d["vatt__recv"]-t0; Qv=d["vatt__q"]; Ev=eul(Qv); mv=(tv>ah)&(tv<bh); fv=1/np.median(np.diff(tv)); tl=tilt_deg(Qv)
    print(f"PX4 attitude {fv:.0f} Hz: hold roll/pitch std {Ev[mv,:2].std(0)} deg, yaw std {Ev[mv,2].std():.2f}; frac>5Hz roll/pitch {peakf(Ev[mv,0],fv)[1]:.3f}/{peakf(Ev[mv,1],fv)[1]:.3f}; tilt max DIRECT {tl[(tv>a)&(tv<b)].max():.2f} deg; quat_reset_counter uniq {np.unique(d['vatt__quat_reset_counter'])}")
    ti=d["imu__hdr"]-t0; G=np.column_stack([d[f"imu__angular_velocity.{k}"] for k in "xyz"]); mi=(ti>ah)&(ti<bh); fi=1/np.median(np.diff(ti))
    print(f"   gyro (imu/data {fi:.0f} Hz) hold std {np.degrees(G[mi].std(0))} deg/s, frac>5Hz {peakf(G[mi,0],fi)[1]:.2f}/{peakf(G[mi,1],fi)[1]:.2f}/{peakf(G[mi,2],fi)[1]:.2f}")
    tsc=d["sc__recv"]-t0; A=d["sc__accelerometer_m_s2"]; ms=(tsc>ah)&(tsc<bh); print(f"   accel std hold {A[ms].std(0)} m/s2 ; gyro_clipping any {np.any(d['sc__gyro_clipping']!=0)}")
    # ---- law stats in hold and whole DIRECT
    mw=(t>ah)&(t<bh); mD=(t>a+0.3)&(t<b-0.05)
    eR=np.linalg.norm(D[mw,28:31],axis=1); ep=D[mw,48:51]-D[mw,45:48]; ey=D[mw,24:27]
    print(f"LAW hold: |e_R| mean {eR.mean():.4f} p99 {np.percentile(eR,99):.4f} | e_R_x peak f/frac>5 {peakf(D[mw,28],250)} | u1 mean {D[mw,17].mean():.2f} std {D[mw,17].std():.2f} | motors std {D[mw,41:45].std(0)} | e_p rms {np.sqrt((ep**2).mean(0))*1000} mm | e_y pos rms {np.sqrt((ey**2).mean(0))*1000} mm | heading err rms {np.degrees(np.arcsin(np.sqrt((D[mw,27]**2).mean()))):.2f} deg")
    print(f"   d_hat filt t std {D[mw,31:34].std(0)} N mean {D[mw,31:34].mean(0)} | d_hat filt r mean {D[mw,34:37].mean(0)} std {D[mw,34:37].std(0)} | deadbeat d^c t std {D[mw,62:65].std(0)} N | Fraw std {D[mw,97:100].std(0)} mean {D[mw,97:100].mean(0)} | tau_j std {D[mw,13:17].std(0)} max {np.abs(D[mw,13:17]).max(0)} | w_hat_q late {D[mw,101:105][-250:].mean(0)}")
    eRD=np.linalg.norm(D[mD,28:31],axis=1); epD=D[mD,48:51]-D[mD,45:48]
    print(f"   whole DIRECT: |e_R| max {eRD.max():.3f} mean {eRD.mean():.4f}, u1 min/max {D[mD,17].min():.1f}/{D[mD,17].max():.1f}, motor min/max {D[mD,41:45].min():.3f}/{D[mD,41:45].max():.3f}, n_sat ticks {(D[mD,51]>0).sum()}, tau clamp ticks {(np.abs(D[mD,13:17])>2.99).sum()}, est-bound ticks {(D[mD,88]>0).sum()}, chi free frac {np.mean(D[mD,105]>0.5):.3f}, stream fresh frac {np.mean(D[mD,56]>0.5):.3f}, |e_p| max {np.linalg.norm(epD,axis=1).max()*1000:.0f} mm")
    if "batt__voltage_v" in d: print(f"   battery {d['batt__voltage_v'][0]:.2f} -> {d['batt__voltage_v'][-1]:.2f} V, current mean {d['batt__current_a'][(d['batt__recv']-t0>a)&(d['batt__recv']-t0<b)].mean():.1f} A")

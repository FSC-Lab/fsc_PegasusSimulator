import sys; sys.path.insert(0,"<scratch>")
from common import *
for nm in ["a1","a2"]:
    d,t0,a,b,ex,hold=load(nm)
    print(f"\n################ {nm}  DIRECT {a:.2f}-{b:.2f}")
    t=d["wb__recv"]-t0; D=d["wb__data"]
    tm=d["mocap__hdr"]-t0; P=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); Q=np.column_stack([d[f"mocap__pose.orientation.{k}"] for k in "wxyz"]); Em=eul(Q); Vm=np.column_stack([d[f"mocap__twist.linear.{k}"] for k in "xyz"])
    to=d["odom__recv"]-t0; Po=np.column_stack([d[f"odom__pose.pose.position.{k}"] for k in "xyz"]); Vo=np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"])
    tp=d["vodom_px__recv"]-t0; Ppx=d["vodom_px__position"]; Vpx=d["vodom_px__velocity"]; pvar=d["vodom_px__position_variance"]; vvar=d["vodom_px__velocity_variance"]
    tvv=d["vvo__recv"]-t0; Pvv=d["vvo__position"]
    # frame check: x_c (48..50) vs odom position
    mD=(t>a+0.5)&(t<b-0.5); Po_i=np.column_stack([np.interp(t,to,Po[:,k]) for k in range(3)])
    print("x_c[48..50] - odom pos rms (m):", np.sqrt(((D[mD,48:51]-Po_i[mD])**2).mean(0)), " -> world frame if ~0")
    # all mocap frozen runs incl short
    rep=np.all(np.diff(P,axis=0)==0,axis=1); runs=[];k=0
    while k<len(rep):
        if rep[k]:
            j=k
            while j<len(rep) and rep[j]: j+=1
            if j-k>=2: runs.append((round(tm[k],2),j-k,round(tm[j]-tm[k],3), round(Em[k,2],1)))
            k=j
        else: k+=1
    print("mocap frozen runs >=2 samples (t, n, dur s, mocap yaw deg):", runs)
    # rosout NO DATA times
    for ti,n_,m_ in zip(d["rosout__recv"]-t0,d["rosout__name"],d["rosout__msg"]):
        if "NO DATA" in m_: 
            i=np.searchsorted(tm,ti); print(f"   NO DATA at {ti:.2f}: mocap yaw {Em[min(i,len(Em)-1),2]:.1f} deg, pos {P[min(i,len(P)-1)]}")
    # ---- heading through the run: b1_d reference yaw and mocap yaw
    tr=d["wbref__recv"]-t0; b1=np.column_stack([d["wbref__b1_d.x"],d["wbref__b1_d.y"]]); yr=np.degrees(np.arctan2(b1[:,1],b1[:,0]))
    xr=np.column_stack([d[f"wbref__x_cd.{k}"] for k in "xyz"]); red=np.column_stack([d[f"wbref__r_ed.{k}"] for k in "xyz"])
    # ---- 1 s table through DIRECT
    print("\n t    | x_c-x_cd (mm) x y z | |ep| | e_y pos mm | hd err deg | |e_R| | tilt | u1  | d_t x y z | d_r x y z | motors min max | ref yaw(model) | mocap yaw | odom-mocap mm | vodom var")
    tv=d["vatt__recv"]-t0; tl=tilt_deg(d["vatt__q"])
    for tc in np.arange(a+0.5, b+3.0, 1.0):
        m=(t>tc-0.5)&(t<tc+0.5)
        if not m.any(): continue
        ep=(D[m,48:51]-D[m,45:48]).mean(0)*1000; eyv=np.sqrt((D[m,24:27]**2).mean(0))*1000; hd=np.degrees(np.arcsin(np.sqrt((D[m,27]**2).mean()))); eR=np.linalg.norm(D[m,28:31],axis=1).mean()
        ti_=np.abs(tv-tc)<0.5; mt=np.abs(tm-tc)<0.5; mo=np.abs(to-tc)<0.5; mr=np.abs(tr-tc)<0.5; mp=np.abs(tp-tc)<0.5
        pm=np.column_stack([np.interp(to[mo],tm,P[:,k]) for k in range(3)]); dom=np.sqrt(((Po[mo]-pm)**2).mean(0))*1000 if mo.any() else np.zeros(3)
        print(f"{tc:5.1f} | {ep[0]:6.0f} {ep[1]:6.0f} {ep[2]:5.0f} | {np.linalg.norm(ep):4.0f} | {eyv[0]:4.1f} {eyv[1]:4.1f} {eyv[2]:4.1f} | {hd:4.1f} | {eR:.3f} | {tl[ti_].max() if ti_.any() else 0:4.1f} | {D[m,17].mean():5.1f} | {D[m,31:34].mean(0)} | {D[m,34:37].mean(0)} | {D[m,41:45].min():.2f} {D[m,41:45].max():.2f} | {yr[mr].mean() if mr.any() else 0:6.1f} | {Em[mt,2].mean() if mt.any() else 0:6.1f} | {dom} | {pvar[mp].max(0) if mp.any() else 0}")
    # ---- zoom on the last 8 s of DIRECT at 0.1 s
    print(f"\n==== zoom {b-9:.1f} .. {b+2:.1f} s at 0.1 s: mocap pos (raw) | odom fused pos | PX4 EKF vel | x_cd | e_p mm | u1 | |e_R| | tilt | motors")
    for tc in np.arange(b-9, b+2, 0.1):
        m=(t>tc-0.05)&(t<tc+0.05); i=np.searchsorted(tm,tc); j=np.searchsorted(to,tc); k2=np.searchsorted(tp,tc); ti_=np.abs(tv-tc)<0.05
        if not m.any(): 
            print(f"{tc:6.2f} | mocap {P[min(i,len(P)-1)]} | odom {Po[min(j,len(Po)-1)]} | (no law tick)"); continue
        ep=(D[m,48:51]-D[m,45:48]).mean(0)*1000
        print(f"{tc:6.2f} | {P[min(i,len(P)-1)]} | {Po[min(j,len(Po)-1)]} | {Vpx[min(k2,len(Vpx)-1)]} | {D[m,45:48].mean(0)} | {ep[0]:5.0f} {ep[1]:5.0f} {ep[2]:4.0f} | {D[m,17].mean():5.1f} | {np.linalg.norm(D[m,28:31],axis=1).mean():.3f} | {tl[ti_].max() if ti_.any() else 0:4.1f} | {D[m,41:45].mean(0)}")
    # EKF variance / flags during freeze
    if nm=="a1":
        fz=(tp>75.0)&(tp<79.0); print("\nPX4 vehicle_odometry during freeze 75-79 s: pos variance min/max", pvar[fz].min(0), pvar[fz].max(0), " vel var max", vvar[fz].max(0), " reset_counter uniq", np.unique(d["vodom_px__reset_counter"][fz]), " quality uniq", np.unique(d["vodom_px__quality"][fz]))
        te=d["esf__recv"]-t0; fe=(te>74)&(te<80)
        for f in ["cs_ev_pos","cs_ev_vel","cs_ev_yaw","cs_ev_hgt","cs_inertial_dead_reckoning"]: print(f"   esf {f} 74-80 s:", d[f"esf__{f}"][fe])
        # EV input during the freeze: was the frozen pose still being sent to PX4?
        fv=(tvv>75.3)&(tvv<78.6); dP=np.linalg.norm(np.diff(Pvv[fv],axis=0),axis=1); print(f"   EV input to PX4 during 75.3-78.6: n={fv.sum()}, identical consecutive samples {(dP==0).mean()*100:.0f}%, rate {fv.sum()/3.3:.0f} Hz")
        # where was the TRUE vehicle (mocap after re-acquisition) vs the fused estimate at re-acquisition
        i0=np.searchsorted(tm,75.51); i1=np.searchsorted(tm,78.36)
        print(f"   mocap frozen at {P[i0]} yaw {Em[i0,2]:.1f}; first fresh sample at {tm[i1]:.2f}: {P[i1]} -> jump {np.linalg.norm(P[i1]-P[i0])*1000:.0f} mm ; mocap twist at re-acq {Vm[i1]}")
        j0=np.searchsorted(to,75.5); j1=np.searchsorted(to,78.1); print(f"   fused odom 75.5 -> 78.1: {Po[j0]} -> {Po[j1]} (moved {np.linalg.norm(Po[j1]-Po[j0])*1000:.0f} mm) ; reference x_cd moved {np.linalg.norm(np.interp(78.1,tr,xr[:,0])-np.interp(75.5,tr,xr[:,0])):.3f} x")
        # what did the reference do 75.5-78.1 (circle continues)
        mr=(tr>75.5)&(tr<78.1); print("   x_cd 75.5->78.1:", xr[mr][0], "->", xr[mr][-1], " |d| mm", np.linalg.norm(xr[mr][-1]-xr[mr][0])*1000)
        # where would the vehicle really have been: integrate? use PX4 EKF velocity vs zero
        # compare the mocap-implied position at re-acq with the fused estimate at that time
        print(f"   at re-acq {tm[i1]:.2f}: fused odom {Po[np.searchsorted(to,tm[i1])]} vs fresh mocap {P[i1]} -> estimate error {np.linalg.norm(Po[np.searchsorted(to,tm[i1])]-P[i1])*1000:.0f} mm")
        # the 55.12 episode
        m55=(tm>54.8)&(tm<55.6); dd=np.diff(tm[m55]); print(f"   55 s episode: mocap dt max {dd.max()*1000:.0f} ms, identical runs:", [r for r in runs if 54<r[0]<56], " odom-mocap diff max 54-57 s:", np.abs(np.column_stack([np.interp(to[(to>54)&(to<57)],tm,P[:,k]) for k in range(3)])-Po[(to>54)&(to<57)]).max(0)*1000, "mm")

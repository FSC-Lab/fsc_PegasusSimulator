import sys; sys.path.insert(0,"<scratch>")
from common import *
def stats(nm,D,t,a,b,tv,tl,Ev,tm,Em):
    m=(t>a)&(t<b)
    if not m.any(): return
    ep=D[m,48:51]-D[m,45:48]; ey=D[m,24:27]; hd=np.degrees(np.arcsin(np.clip(np.sqrt((D[m,27]**2).mean()),-1,1))); eR=np.linalg.norm(D[m,28:31],axis=1)
    mv=(tv>a)&(tv<b); q=D[m,5:9]; qd=D[m,9:13]
    print(f"{nm:22s} | CoM err rms {np.sqrt((ep**2).mean(0))*1000} peak {np.linalg.norm(ep,axis=1).max()*1000:5.0f} mm | EE rel err rms {np.sqrt((ey**2).mean(0))*1000} max {np.abs(ey).max(0)*1000} mm | hd err rms {hd:4.2f} max {np.degrees(np.arcsin(np.abs(D[m,27]).max())):4.1f} deg | |e_R| mean {eR.mean():.3f} max {eR.max():.3f} | tilt max {tl[mv].max():4.1f} mean {tl[mv].mean():4.1f} | u1 {D[m,17].mean():5.1f}+-{D[m,17].std():.2f} | mot min/max {D[m,41:45].min():.2f}/{D[m,41:45].max():.2f} | arm q-qd rms deg {np.degrees(np.sqrt(((q-qd)**2).mean(0)))} | tau max {np.abs(D[m,13:17]).max(0)}")
for nm in ["a1","a2"]:
    d,t0,a,b,ex,hold=load(nm)
    t=d["wb__recv"]-t0; D=d["wb__data"]
    tm=d["mocap__hdr"]-t0; P=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); Q=np.column_stack([d[f"mocap__pose.orientation.{k}"] for k in "wxyz"]); Em=eul(Q)
    tv=d["vatt__recv"]-t0; tl=tilt_deg(d["vatt__q"]); Ev=eul(d["vatt__q"])
    tr=d["wbref__recv"]-t0; b1=np.column_stack([d["wbref__b1_d.x"],d["wbref__b1_d.y"]]); yr=np.unwrap(np.arctan2(b1[:,1],b1[:,0]))
    print(f"\n################ {nm}  DIRECT {a:.2f}-{b:.2f}; exec {[(round(x,1),s) for x,s in ex]}")
    # EV timestamp_sample during freeze / mocap hdr during freeze
    if nm=="a1":
        tvv=d["vvo__recv"]-t0; ts=d["vvo__timestamp_sample"]; f=(tvv>75.6)&(tvv<78.3); print(f"EV timestamp_sample during freeze: monotone increasing? {np.all(np.diff(ts[f])>0)}, unique {len(np.unique(ts[f]))}/{f.sum()}, mean step {np.mean(np.diff(ts[f]))/1000:.1f} ms; mocap hdr during freeze unique {len(np.unique(d['mocap__hdr'][(tm>75.6)&(tm<78.3)]))}/{((tm>75.6)&(tm<78.3)).sum()}, mocap recv rate {((d['mocap__recv']-t0>75.6)&(d['mocap__recv']-t0<78.3)).sum()/2.7:.0f} Hz")
        # run progress at the freeze: accumulated ref yaw since run start
        rs=ex[1][0]; mr=(tr>rs)&(tr<75.51); print(f"run started {rs:.2f}; at 75.51 s ({75.51-rs:.1f} s of 52.0) the reference heading had advanced {np.degrees(yr[mr][-1]-yr[mr][0]):.0f} deg = {np.degrees(yr[mr][-1]-yr[mr][0])/360:.2f} laps; mocap yaw at freeze {Em[np.searchsorted(tm,75.51),2]:.1f}")
    # ---- phases
    print("phase                  | metrics")
    ah=a+0.5; bh=ex[0][0]-0.2; stats("DIRECT hold",D,t,ah,bh,tv,tl,Ev,tm,Em)
    g0=ex[0][0]; g1=hold[[i for i,(x,s) in enumerate(hold) if x>g0][0]][0]; stats("go-to-start",D,t,g0,g1,tv,tl,Ev,tm,Em)
    stats("start hold",D,t,g1+0.2,ex[1][0]-0.2,tv,tl,Ev,tm,Em)
    r0=ex[1][0]; 
    if nm=="a1":
        stats("run ramp+lap1 (0-28s)",D,t,r0,r0+28,tv,tl,Ev,tm,Em); stats("run lap2 to freeze",D,t,r0+28,75.5,tv,tl,Ev,tm,Em); stats("run freeze 75.5-78.1",D,t,75.5,78.09,tv,tl,Ev,tm,Em); stats("run all to freeze",D,t,r0,75.5,tv,tl,Ev,tm,Em)
    else:
        r1=hold[[i for i,(x,s) in enumerate(hold) if x>r0][0]][0]; stats("run (1 lap, 28 s)",D,t,r0,r1,tv,tl,Ev,tm,Em); stats("post-run hold",D,t,r1+0.5,b-0.2,tv,tl,Ev,tm,Em)
        f=(tm>36.5)&(tm<38.5); rep=np.all(np.diff(P[f],axis=0)==0,axis=1); print(f"a2 37.24 s episode: mocap yaw {Em[np.searchsorted(tm,37.24),2]:.1f}, identical consecutive samples in 36.5-38.5: {rep.sum()}, max dt {np.diff(tm[f]).max()*1000:.0f} ms")
    # ---- lap-periodic base error: e_p vs heading
    m=(t>r0)&(t<(75.5 if nm=="a1" else r0+28)); ep=D[m,48:51]-D[m,45:48]; yaw_i=np.interp(t[m],tm,np.unwrap(np.radians(Em[:,2])))
    # express e_p in the body-yaw frame: e_body = Rz(-yaw) e_world
    c,s=np.cos(yaw_i),np.sin(yaw_i); ebx=c*ep[:,0]+s*ep[:,1]; eby=-s*ep[:,0]+c*ep[:,1]
    print(f"base error during run: world-frame mean {ep.mean(0)*1000} mm std {ep.std(0)*1000}; in BODY-yaw frame mean [{ebx.mean()*1000:.0f} {eby.mean()*1000:.0f}] std [{ebx.std()*1000:.0f} {eby.std()*1000:.0f}] mm  (a body-fixed offset shows as constant here)")
    # d_hat_t in body frame
    dt_=D[m,31:34]; dbx=c*dt_[:,0]+s*dt_[:,1]; dby=-s*dt_[:,0]+c*dt_[:,1]; print(f"   d_hat_t world std {dt_.std(0)} N; body-yaw frame mean [{dbx.mean():.2f} {dby.mean():.2f}] std [{dbx.std():.2f} {dby.std():.2f}] N ; d_hat_r world mean {D[m,34:37].mean(0)} std {D[m,34:37].std(0)}")
    dr=D[m,34:37]; drx=c*dr[:,0]+s*dr[:,1]; dry=-s*dr[:,0]+c*dr[:,1]; print(f"   d_hat_r body-yaw frame mean [{drx.mean():.3f} {dry.mean():.3f}] std [{drx.std():.3f} {dry.std():.3f}] N.m")
    # correlation of e_p with e_R
    eR=D[m,28:31]; print(f"   corr(e_p_x, e_R_y) {np.corrcoef(ep[:,0],eR[:,1])[0,1]:.2f}, corr(e_p_y, e_R_x) {np.corrcoef(ep[:,1],eR[:,0])[0,1]:.2f}; e_R x/y std {eR[:,:2].std(0)} ; yaw err (e_R_z) mean {eR[:,2].mean():.4f} std {eR[:,2].std():.4f}")
    # velocity error: reference vs fused
    to=d["odom__recv"]-t0; Vo=np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"]); vr=np.column_stack([d[f"wbref__x_cd_dot.{k}"] for k in "xyz"])
    mo=(to>r0)&(to<(75.5 if nm=="a1" else r0+28)); vri=np.column_stack([np.interp(to[mo],tr,vr[:,k]) for k in range(3)]); ev=Vo[mo]-vri; print(f"   velocity err (odom - ref) rms {np.sqrt((ev**2).mean(0))*100} cm/s; ref |v| max {np.linalg.norm(vr,axis=1).max():.3f} m/s")
    # EE measured circle: pl_current_ee (FK) radius about origin at EE height
    te=d["pl_current_ee__recv"]-t0; Pe=np.column_stack([d[f"pl_current_ee__pose.position.{k}"] for k in "xyz"]); me=(te>r0+4)&(te<(75.5 if nm=="a1" else r0+24)); r=np.hypot(Pe[me,0],Pe[me,1])
    red=np.column_stack([d[f"wbref__r_ed.{k}"] for k in "xyz"]); mr=(tr>r0+4)&(tr<(75.5 if nm=="a1" else r0+24)); rr=np.hypot(red[mr,0],red[mr,1])
    print(f"   EE (FK of measured) radius about origin: mean {r.mean():.3f} std {r.std()*1000:.0f} mm min/max {r.min():.3f}/{r.max():.3f}; z mean {Pe[me,2].mean():.3f} std {Pe[me,2].std()*1000:.0f} mm | reference r_ed radius mean {rr.mean():.3f} std {rr.std()*1000:.0f} mm z {red[mr,2].mean():.3f}")
    # absolute EE error: FK EE vs reference r_ed (world, interpolated)
    redi=np.column_stack([np.interp(te[me],tr,red[:,k]) for k in range(3)]); eabs=Pe[me]-redi; print(f"   ABSOLUTE EE error (FK - planned) rms {np.sqrt((eabs**2).mean(0))*1000} mm, |e| mean {np.linalg.norm(eabs,axis=1).mean()*1000:.0f} max {np.linalg.norm(eabs,axis=1).max()*1000:.0f} mm  (this includes the base error; e_y above is CoM-anchored)")
    # heading sectors where mocap lost tracking / jittered (all flights)
    ang=qstep(Q); rep=np.all(np.diff(P,axis=0)==0,axis=1)
    print("   mocap health per yaw sector (all bag): sector | samples | frozen% | orient step>3deg%")
    for lo in range(-180,180,45):
        mm=(Em[1:,2]>=lo)&(Em[1:,2]<lo+45)
        if mm.sum()>50: print(f"      [{lo:4d},{lo+45:4d}) {mm.sum():6d} {100*rep[mm].mean():5.1f}% {100*(ang[mm]>3).mean():5.1f}%")

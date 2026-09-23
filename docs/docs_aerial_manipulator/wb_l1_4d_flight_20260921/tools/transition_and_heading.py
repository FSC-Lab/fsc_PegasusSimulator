import numpy as np
np.set_printoptions(precision=3, suppress=True, linewidth=200)
d=np.load("/tmp/claude-1000/-home-shiqi-fsc-PegasusSimulator/439adc13-0e06-4d06-ac66-d9a5a6af280f/scratchpad/c3.npz",allow_pickle=True); t0=d["wb__recv"][0]
def eul(q):
    w,x,y,z=q.T
    return np.degrees(np.column_stack([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),np.arcsin(np.clip(2*(w*y-z*x),-1,1)),np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))]))
tm=d["mocap__hdr"]-t0; P=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); Em=eul(np.column_stack([d[f"mocap__pose.orientation.{k}"] for k in "wxyz"]))
n=np.linalg.norm(np.diff(P,axis=0),axis=1); sp=np.where(n>0.05)[0]
print("=== spikes vs heading ===")
for i in sp: print(f"  t={tm[i+1]:6.2f}  mocap yaw before {Em[i,2]:7.1f}  offset {P[i+1]-P[i]}")
print("orientation-step>3deg count per yaw bin (DIRECT+after):")
ang=2*np.degrees(np.arccos(np.clip(np.abs(np.sum(np.column_stack([d[f'mocap__pose.orientation.{k}'] for k in 'wxyz'])[1:]*np.column_stack([d[f'mocap__pose.orientation.{k}'] for k in 'wxyz'])[:-1],axis=1)),-1,1)))
for lo in range(-180,180,30):
    m=(Em[1:,2]>=lo)&(Em[1:,2]<lo+30); 
    if m.sum(): print(f"  yaw [{lo:4d},{lo+30:4d}): samples {m.sum():5d}  steps>3deg {(ang[m]>3).sum():4d} ({100*(ang[m]>3).mean():.1f}%)  median step {np.median(ang[m]):.2f}")
print("\n=== PX4 side reaction to spikes ===")
tp=d["vodom_px__recv"]-t0; rc=d["vodom_px__reset_counter"]; Pp=d["vodom_px__position"]
ch=np.where(np.diff(rc)!=0)[0]; print("PX4 vehicle_odometry reset_counter changes at:", [(round(tp[i+1],2),rc[i+1]) for i in ch])
for tc in [20.28,24.27]:
    m=(tp>tc-0.05)&(tp<tc+0.4); print(f" around {tc}: PX4 pos NED (x,y,z) steps:", np.abs(np.diff(Pp[m],axis=0)).max(0))
print("estimator_status_flags changes:")
for k in d.files:
    if k.startswith("esf__"): pass
te=d["vstatus__recv"]-t0
print("nav_state unique:", np.unique(d["vstatus__nav_state"]), "failsafe unique", np.unique(d["vstatus__failsafe"]))
print("\n=== transition tracking 13.6-20.2 (before first spike) ===")
t=d["wb__recv"]-t0; D=d["wb__data"]; m=(t>13.6)&(t<20.2)
ep=D[m,48:51]-D[m,45:48]; print("base CoM err rms", np.sqrt((ep**2).mean(0))*1000, "mm  max", np.abs(ep).max(0)*1000)
ey=D[m,24:28]; print("EE task err e_y pos rms", np.sqrt((ey[:,:3]**2).mean(0))*1000, "mm max", np.abs(ey[:,:3]).max(0)*1000, " heading sin rms", np.sqrt((ey[:,3]**2).mean()), "-> deg", np.degrees(np.arcsin(np.sqrt((ey[:,3]**2).mean()))))
q=D[m,5:9]; qd=D[m,9:13]; print("arm q-q_d rms (deg)", np.degrees(np.sqrt(((q-qd)**2).mean(0))), " max", np.degrees(np.abs(q-qd).max(0)))
print("q range (deg)", np.degrees(q.min(0)), np.degrees(q.max(0)))
eR=D[m,28:31]; print("|e_R| mean", np.linalg.norm(eR,axis=1).mean(), " e_R_z mean/std", eR[:,2].mean(), eR[:,2].std())
# yaw reference vs actual: from b1_d (model) vs PX4 yaw; earlier we saw ref tracks. print planned yaw span
tr=d["wbref__hdr"]+0.048-t0; b1=np.column_stack([d["wbref__b1_d.x"],d["wbref__b1_d.y"]]); yr=np.degrees(np.unwrap(np.arctan2(b1[:,1],b1[:,0])))
mr=(tr>13.4)&(tr<25.2); print("ref yaw (model frame) 13.4 s -> 25.1 s:", yr[mr][0], "->", yr[mr][-1], " delta", yr[mr][-1]-yr[mr][0], " max rate deg/s", np.abs(np.gradient(yr[mr],tr[mr])).max())
xr=np.column_stack([d[f"wbref__x_cd.{k}"] for k in "xyz"]); print("x_cd 13.4 ->25.1:", xr[mr][0], "->", xr[mr][-1], " |d|", np.linalg.norm(xr[mr][-1]-xr[mr][0]))
qd_r=d["wbref__q_d"]; print("q_d 13.4->25.1 (deg):", np.degrees(qd_r[mr][0]), "->", np.degrees(qd_r[mr][-1]))
red=np.column_stack([d[f"wbref__r_ed.{k}"] for k in "xyz"]); print("r_ed 13.4->25.1:", red[mr][0], "->", red[mr][-1])
# viz_path: planned path (n x ?) 
vp=d["pl_viz_path__data"]; print("viz_path msgs shapes:", [np.sum(~np.isnan(r)) for r in vp])
# velocity noise vs law: k_v * e_v contribution
to=d["odom__hdr"]-t0; Vo=np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"]); mh=(to>4.6)&(to<13.3)
print("\nhold: k_v*std(v) per axis N:", 20*Vo[mh].std(0), " vs d_hat filt std", D[(t>4.6)&(t<13.3),31:34].std(0), " vs u1 std", D[(t>4.6)&(t<13.3),17].std())
# autocorrelation of odom vx in hold to find period
x=Vo[mh,0]-Vo[mh,0].mean(); ac=np.correlate(x,x,'full')[len(x)-1:len(x)+12]/np.dot(x,x); print("odom vx autocorr lags 0..12 (8ms):", ac)
xp=P[(tm>4.6)&(tm<13.3),0]; dxp=np.diff(xp); dxp=dxp-dxp.mean(); ac2=np.correlate(dxp,dxp,'full')[len(dxp)-1:len(dxp)+12]/np.dot(dxp,dxp); print("mocap dx autocorr lags 0..12:", ac2)
# mocap dt vs recv: check timestamp regularity
dtm=np.diff(tm); print("mocap hdr dt histogram (ms):", np.unique(np.round(dtm*1000),return_counts=True))

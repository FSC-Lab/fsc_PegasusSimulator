import sys; sys.path.insert(0,"<scratch>")
from common import *
d,t0,a,b,ex,hold=load("a1")
t=d["wb__recv"]-t0; D=d["wb__data"]
tm=d["mocap__hdr"]-t0; tmr=d["mocap__recv"]-t0; P=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); Q=np.column_stack([d[f"mocap__pose.orientation.{k}"] for k in "wxyz"]); Em=eul(Q); Vm=np.column_stack([d[f"mocap__twist.linear.{k}"] for k in "xyz"])
to=d["odom__recv"]-t0; Po=np.column_stack([d[f"odom__pose.pose.position.{k}"] for k in "xyz"]); Vo=np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"])
tp=d["vodom_px__recv"]-t0; Ppx=d["vodom_px__position"]; Vpx=d["vodom_px__velocity"]
tvv=d["vvo__recv"]-t0; Pvv=d["vvo__position"]
tv=d["vatt__recv"]-t0; Ev=eul(d["vatt__q"]); tl=tilt_deg(d["vatt__q"])
tr=d["wbref__recv"]-t0; xr=np.column_stack([d[f"wbref__x_cd.{k}"] for k in "xyz"])
ta=d["attsp__recv"]-t0; qsp=d["attsp__q_d"]; Esp=eul(qsp); thr=d["attsp__thrust_body"]
tpc=d["pcstate__recv"]-t0
print("pcstate keys:", [k for k in d.files if k.startswith("pcstate__")][:30])
print("ude keys:", [k for k in d.files if k.startswith("ude__")][:30])
tvs=d["vstatus__recv"]-t0; ns=d["vstatus__nav_state"]; arm=d["vstatus__arming_state"]
print("nav_state changes:", [(round(tvs[i+1],2),int(ns[i+1])) for i in np.where(np.diff(ns)!=0)[0]], " arming changes:", [(round(tvs[i+1],2),int(arm[i+1])) for i in np.where(np.diff(arm)!=0)[0]])
print("mocap hdr-recv median (s):", np.median(d["mocap__hdr"]-d["mocap__recv"]))
print("\n t     | mocap pos (hdr t)         | mocap yaw | mocap same? | EV->PX4 pos (NED)        | PX4 EKF pos NED           | fused odom ENU            | PX4 vel NED        | x_cd (ref)             | e_p mm        | u1   | |e_R| | tilt | PX4 rpy            | SAFETY att sp rpy / thrust")
for tc in np.arange(74.0, 92.0, 0.2):
    i=np.searchsorted(tm,tc)-1; j=np.searchsorted(to,tc)-1; k=np.searchsorted(tp,tc)-1; v=np.searchsorted(tvv,tc)-1; m=(t>tc-0.1)&(t<tc+0.1); iv=np.searchsorted(tv,tc)-1; ia=np.searchsorted(ta,tc)-1
    same = "FROZEN" if (i>0 and np.all(P[i]==P[i-1])) else "fresh"
    ep=(D[m,48:51]-D[m,45:48]).mean(0)*1000 if m.any() else np.full(3,np.nan)
    u1=D[m,17].mean() if m.any() else np.nan; eR=np.linalg.norm(D[m,28:31],axis=1).mean() if m.any() else np.nan
    ir=np.searchsorted(tr,tc)-1
    print(f"{tc:6.1f} | {P[i]} | {Em[i,2]:7.1f} | {same:6s} | {Pvv[v]} | {Ppx[k]} | {Po[j]} | {Vpx[k]} | {xr[ir] if ir<len(xr) else ''} | {ep[0]:5.0f} {ep[1]:5.0f} {ep[2]:4.0f} | {u1:5.1f} | {eR:.3f} | {tl[iv]:4.1f} | {Ev[iv]} | {Esp[ia]} / {thr[ia]}")

"""Circle-phase RMSE of the full state and the EE 4-D pose for the four 0924 hardware flights."""
import sys, json; sys.path.insert(0,"<scratch>")
from common import *
RUNS=[("a1","F1 12:02",(35.9,75.5)),("a2","F2 12:05",(22.94,50.94)),("a3","F3 16:56",(20.62,48.62)),("a4","F4 17:21",(20.09,48.08))]
rows={}
for nm,lab,(r0,r1) in RUNS:
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]; m=(t>r0)&(t<r1)
    q=np.degrees(D[m,5:9]); qd=np.degrees(D[m,9:13])
    ep=(D[m,48:51]-D[m,45:48]); ey=D[m,24:27]; ab=ep+ey
    tr=d["wbref__recv"]-t0; vr=np.column_stack([d[f"wbref__x_cd_dot.{k}"] for k in "xyz"])
    to=d["odom__recv"]-t0; Vo=np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"]); mo=(to>r0)&(to<r1)
    ev=Vo[mo]-np.column_stack([np.interp(to[mo],tr,vr[:,k]) for k in range(3)])
    eR=np.degrees(np.arcsin(np.clip(D[m,28:31],-1,1)))  # body-axis attitude error angles
    hd=np.degrees(np.arcsin(np.clip(D[m,27],-1,1)))
    # q2 design from the reference
    des=f"q2 {qd[:,1].min():.0f}–{qd[:,1].max():.0f}°, q3 {qd[:,2].min():.0f}–{qd[:,2].max():.0f}°"
    x=qd[:,1]-(qd[:,1].max()+qd[:,1].min())/2; zc=t[m][np.where(np.diff(np.sign(x))!=0)[0]]; per=2*np.median(np.diff(zc)) if len(zc)>2 else float("nan")
    rms=lambda v: float(np.sqrt((v**2).mean()))
    r={"design":des,"q2_period_s":round(per,1),"laps":round((r1-r0)/24,2),
       "com_pos_m":[rms(ep[:,k]) for k in range(3)],"com_vel_mps":[rms(ev[:,k]) for k in range(3)],
       "att_deg":[rms(eR[:,k]) for k in range(3)],"joints_deg":[rms(q[:,k]-qd[:,k]) for k in range(4)],
       "ee_task_m":[rms(ey[:,k]) for k in range(3)],"ee_abs_m":[rms(ab[:,k]) for k in range(3)],"ee_yaw_deg":rms(hd),
       "com_pos_norm_m":rms(np.linalg.norm(ep,axis=1)),"ee_abs_norm_m":rms(np.linalg.norm(ab,axis=1)),"ee_task_norm_m":rms(np.linalg.norm(ey,axis=1)),
       "tau_max":[float(np.abs(D[m,13+k]).max()) for k in range(4)],"sat":int((D[m,51]>0).sum()),"clamp":int((np.abs(D[m,13:17])>2.99).sum()),
       "q_span_pct":[float(100*(q[:,k].max()-q[:,k].min())/max(qd[:,k].max()-qd[:,k].min(),1e-9)) if (qd[:,k].max()-qd[:,k].min())>1 else None for k in range(4)]}
    rows[lab]=r
    print(f"\n{lab} [{nm}] circle {r0:.1f}-{r1:.1f} ({r['laps']} laps): {des}, q2 period {per:.1f} s")
    print(f"  CoM pos rmse x/y/z mm {np.round(np.array(r['com_pos_m'])*1000,1)} |norm| {r['com_pos_norm_m']*1000:.1f} | CoM vel mm/s {np.round(np.array(r['com_vel_mps'])*1000,1)} | att err deg r/p/y {np.round(r['att_deg'],2)}")
    print(f"  joints deg {np.round(r['joints_deg'],2)} span% {r['q_span_pct']} | EE task mm {np.round(np.array(r['ee_task_m'])*1000,1)} |norm| {r['ee_task_norm_m']*1000:.1f} | EE abs mm {np.round(np.array(r['ee_abs_m'])*1000,1)} |norm| {r['ee_abs_norm_m']*1000:.1f} | EE yaw deg {r['ee_yaw_deg']:.2f} | tau max {np.round(r['tau_max'],2)} sat {r['sat']} clamp {r['clamp']}")
json.dump(rows,open("rmse_rows.json","w"),indent=1)

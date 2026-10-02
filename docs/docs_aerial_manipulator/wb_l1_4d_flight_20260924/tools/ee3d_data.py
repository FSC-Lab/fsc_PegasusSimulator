import sys, json; sys.path.insert(0,"<scratch>")
from common import *
# (npz, key, circle run window [s since bag start], end of the PLANNED run, label)
FLIGHTS=[("a1","f1",(35.9,75.5),78.09,None),
         ("a2","f2",(22.94,50.94),50.94,"Flight 2 — one lap, completed (arm 30° ± 10°, 48 s: half a cycle)"),
         ("a3","f3",(20.62,48.62),48.62,"Flight 3 — one lap, completed (arm 25° ± 15°, 12 s)"),
         ("a4","f4",(20.09,48.08),48.08,"Flight 4 — one lap, completed (arm 25° ± 15°, 6 s)")]
out={}
for nm,key,(r0,r1),rend,label in FLIGHTS:
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]
    tr=d["wbref__recv"]-t0; red=np.column_stack([d[f"wbref__r_ed.{k}"] for k in "xyz"]); xcd=np.column_stack([d[f"wbref__x_cd.{k}"] for k in "xyz"])
    b1e=np.column_stack([d["wbref__b1_de.x"],d["wbref__b1_de.y"]])
    m=np.where((t>r0)&(t<r1))[0][::10]; tt=t[m]
    red_i=np.column_stack([np.interp(tt,tr,red[:,k]) for k in range(3)])
    ee=red_i+D[m,24:27]+(D[m,48:51]-D[m,45:48])       # absolute EE = planned + task error + base CoM error
    xc=D[m,48:51]
    to=d["odom__recv"]-t0; Po=np.column_stack([d[f"odom__pose.pose.position.{k}"] for k in "xyz"]); base=np.column_stack([np.interp(tt,to,Po[:,k]) for k in range(3)])
    # lap index from the accumulated reference EE heading
    h=np.unwrap(np.arctan2(np.interp(tt,tr,b1e[:,1]),np.interp(tt,tr,b1e[:,0]))); acc=np.abs(h-h[0]); lap=(acc>=2*np.pi-1e-6).astype(int)
    mr=(tr>r0)&(tr<rend); planned_ee=red[mr][::10]; planned_com=xcd[mr][::10]
    err=np.linalg.norm(ee-red_i,axis=1)*1000
    r=lambda A: np.round(A,4).tolist()
    laps=float(acc[-1]/(2*np.pi))
    out[key]=dict(t=np.round(tt-r0,2).tolist(),ee=r(ee),ee_ref=r(planned_ee),com=r(xc),com_ref=r(planned_com),base=r(base),lap=lap.tolist(),err=np.round(err,1).tolist(),
                  laps_flown=round(laps,2),label=label or ("Flight 1 — two-lap plan, stopped at %.2f laps (arm 30° ± 10°, 48 s)"%laps))
    print(key,len(tt),"pts; laps",out[key]["laps_flown"],"; lap-2 samples",int(lap.sum()),"; EE err mean/max mm",err.mean().round(0),err.max().round(0),"; ee z",ee[:,2].min().round(3),ee[:,2].max().round(3))
json.dump(out,open("ee3d.json","w"),separators=(",",":")); import os; print("json bytes",os.path.getsize("ee3d.json"))

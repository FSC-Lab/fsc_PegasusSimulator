"""EE error by channel, all four flights -> ee_err.json for the report's interactive figure 2.
Same quantities as ee_figure.py (base CoM error, EE absolute = task + base, EE task error, EE heading error),
binned to 0.04 s means (25 Hz) so the embedded page stays small."""
import sys, json; sys.path.insert(0,"<scratch>")
from common import *
BIN=0.04
FL=[("a1","Flight 1 · 30° ± 10°, 48 s · stopped at 1.57 laps",24.8,78.05,75.5,[("go-to-start",24.81,30.84,"goto"),("",30.84,35.9,"hold"),("circle run",35.9,75.51,"run"),("mocap frozen",75.51,78.05,"frozen")]),
    ("a2","Flight 2 · 30° ± 10°, 48 s · one lap = half a cycle",16.2,57.7,57.7,[("go-to-start",16.17,20.27,"goto"),("",20.27,22.94,"hold"),("circle run",22.94,50.94,"run"),("",50.94,57.7,"hold")]),
    ("a3","Flight 3 · 25° ± 15°, 12 s · one lap",12.2,52.1,52.1,[("go-to-start",12.24,18.60,"goto"),("",18.60,20.62,"hold"),("circle run",20.62,48.62,"run"),("",48.62,52.1,"hold")]),
    ("a4","Flight 4 · 25° ± 15°, 6 s · one lap",12.7,55.5,55.5,[("go-to-start",12.74,17.41,"goto"),("",17.41,20.09,"hold"),("circle run",20.09,48.08,"run"),("",48.08,55.5,"hold")])]
out=[]
for nm,title,w0,w1,wfk,ph in FL:
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]; m=(t>w0)&(t<w1)
    task=D[:,24:27]*1000; base=(D[:,48:51]-D[:,45:48])*1000; absd=task+base
    te=d["pl_current_ee__recv"]-t0; Qe=np.column_stack([d[f"pl_current_ee__pose.orientation.{k}"] for k in "wxyz"]); w,x,y,z=Qe.T
    tr=d["wbref__recv"]-t0; hr=np.arctan2(np.interp(te,tr,d["wbref__b1_de.y"]),np.interp(te,tr,d["wbref__b1_de.x"]))
    hfk=np.degrees(np.angle(np.exp(1j*(np.arctan2(2*(x*y+w*z),1-2*(y*y+z*z))-hr-np.pi/2))))
    hlaw=np.degrees(np.arcsin(np.clip(D[:,27],-1,1))); me=(te>w0)&(te<wfk)
    hdg=np.sign(np.corrcoef(hfk[me],np.interp(te[me],t,hlaw))[0,1])*hlaw
    tt=t[m]; k=np.floor((tt-w0)/BIN).astype(int); n=k.max()+1; cnt=np.bincount(k,minlength=n); ok=cnt>0
    def bm(v): return (np.bincount(k,weights=v,minlength=n)[ok]/cnt[ok])
    T=bm(tt)
    r1=lambda v: [round(float(x),1) for x in v]
    rec=dict(title=title,t=[round(float(x),2) for x in T],phases=[[l,a_,b_,kind] for l,a_,b_,kind in ph],
             base=[r1(bm(base[m,j])) for j in range(3)],abs=[r1(bm(absd[m,j])) for j in range(3)],
             task=[r1(bm(task[m,j])) for j in range(3)],hdg=[round(float(x),2) for x in bm(hdg[m])])
    out.append(rec)
    print(nm,len(T),"bins; abs max",[max(abs(v) for v in c) for c in rec["abs"]],"task max",[max(abs(v) for v in c) for c in rec["task"]],"hdg max",max(abs(v) for v in rec["hdg"]))
json.dump(out,open("ee_err.json","w"),separators=(",",":"),ensure_ascii=False); import os; print("json bytes",os.path.getsize("ee_err.json"))

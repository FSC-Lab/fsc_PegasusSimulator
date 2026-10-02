"""Side-by-side arm / EE / base metrics for EE-circle runs, in PHYSICS time.
   PYTHONNOUSERSITE=1 /usr/bin/python3 compare_q2.py"""
import sys; sys.path.insert(0,"<scratch>")
from common import *
Q_MAX=np.array([35.0,50.0,50.0,120.0]); Q_MIN=np.array([-35.0,-80.0,-40.0,-120.0])
RUNS=[("s_new1","SIM new: fold 55, q2 25±15, 12 s",None),("s_old1","SIM old: fold 60, q2 30±10, 48 s",None),
      ("s_new","SIM new at 2x pace (stress)",None),("a1","HW flight 1: old design, 48 s",75.5),("a2","HW flight 2: old, 1 lap, 24 s",None)]
rows=[]
for nm,lab,cut in RUNS:
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]
    Ts=[float(s.split("T=")[1].rstrip("s")) for x,s in ex]; k=int(np.argmax(Ts)); r0=ex[k][0]; r1=min(r0+Ts[k], b-0.05, cut or 1e9)
    ts=d["sc__recv"]-t0; ms=(ts>r0)&(ts<r1); tsc=d["sc__timestamp"][ms]*1e-6; rtf=(tsc[-1]-tsc[0])/(ts[ms][-1]-ts[ms][0])
    m=(t>r0)&(t<r1); q=np.degrees(D[:,5:9]); qd=np.degrees(D[:,9:13])
    # q2_d period from zero crossings (plant time)
    x=qd[m,1]-(qd[m,1].max()+qd[m,1].min())/2; zc=t[m][np.where(np.diff(np.sign(x))!=0)[0]]; per=2*np.median(np.diff(zc))*rtf if len(zc)>2 else float("nan")
    res={}
    for j in (1,2):
        e=q[m,j]-qd[m,j]; xx=q[m,j]-q[m,j].mean(); yy=qd[m,j]-qd[m,j].mean(); best=(0,-2)
        for L in range(0,700,2):
            c=np.corrcoef(xx[L:],yy[:len(yy)-L])[0,1]
            if c>best[1]: best=(L,c)
        res[j]=dict(span=100*(q[m,j].max()-q[m,j].min())/(qd[m,j].max()-qd[m,j].min()),rms=np.sqrt((e**2).mean()),mx=np.abs(e).max(),lag=best[0]*0.004*rtf*1000,
                    marg=min(Q_MAX[j]-q[m,j].max(),q[m,j].min()-Q_MIN[j]),qmax=q[m,j].max(),qmin=q[m,j].min(),qdmin=qd[m,j].min(),qdmax=qd[m,j].max())
    ep=(D[m,48:51]-D[m,45:48])*1000; ey=D[m,24:27]*1000; ab=ep+ey; hd=np.degrees(np.arcsin(np.clip(D[m,27],-1,1)))
    tr=d["wbref__recv"]-t0; red=np.column_stack([d[f"wbref__r_ed.{kk}"] for kk in "xyz"]); mr=(tr>r0+4/max(rtf,0.2))&(tr<r1-4/max(rtf,0.2))
    L=np.linalg.norm(np.diff(red[mr],axis=0),axis=1).sum(); v=L/((tr[mr][-1]-tr[mr][0])*rtf)
    tau=np.abs(D[m,13:17]).max(0)
    rows.append((lab,rtf,v,per,res,np.sqrt((ep[:,:2]**2).sum(1).mean()),np.sqrt((ey**2).sum(1).mean()),np.sqrt((ab**2).sum(1).mean()),np.sqrt((hd**2).mean()),np.linalg.norm(D[m,28:31],axis=1).mean(),tau,(D[m,51]>0).sum(),(np.abs(D[m,13:17])>2.99).sum()))
print(f"{'run':36s} {'RTF':>5s} {'v_EE m/s':>8s} {'q2 per s':>8s} | {'q2 d-range':>11s} {'q2 meas':>11s} {'span%':>5s} {'rms':>5s} {'max':>5s} {'lag ms':>6s} | {'q3 d-range':>11s} {'q3 meas':>11s} {'span%':>5s} {'rms':>5s} {'max':>5s} {'lag ms':>6s} {'q3 margin':>9s} | base xy rms | EE task | EE abs | yaw rms | |e_R| | tau2/3 max | sat clamp")
for lab,rtf,v,per,res,eb,et,ea,hy,er,tau,ns,nc in rows:
    r2,r3=res[1],res[2]
    print(f"{lab:36s} {rtf:5.2f} {v:8.3f} {per:8.1f} | {r2['qdmin']:5.1f}..{r2['qdmax']:4.1f} {r2['qmin']:5.1f}..{r2['qmax']:4.1f} {r2['span']:5.0f} {r2['rms']:5.2f} {r2['mx']:5.2f} {r2['lag']:6.0f} | {r3['qdmin']:5.1f}..{r3['qdmax']:4.1f} {r3['qmin']:5.1f}..{r3['qmax']:4.1f} {r3['span']:5.0f} {r3['rms']:5.2f} {r3['mx']:5.2f} {r3['lag']:6.0f} {r3['marg']:9.1f} | {eb:6.1f} mm | {et:5.1f} mm | {ea:6.1f} mm | {hy:5.2f} | {er:.3f} | {tau[1]:.2f}/{tau[2]:.2f} | {ns} {nc}")

import sys; sys.path.insert(0,"<scratch>")
from common import *
JS_ORDER=[2,3,1,4]  # joint_states columns are [j2,j3,j1,j4]
for nm in ["a1","a2"]:
    d,t0,a,b,ex,hold=load(nm)
    t=d["wb__recv"]-t0; D=d["wb__data"]; q=np.degrees(D[:,5:9]); qd=np.degrees(D[:,9:13]); tau=D[:,13:17]
    r0=ex[1][0]; r1=75.5 if nm=="a1" else r0+28.0
    tj=d["js__recv"]-t0; jsq=np.degrees(d["js__position"][:,:4]); jsv=np.degrees(d["js__velocity"][:,:4]); jse=d["js__effort"][:,:4]
    # reorder js to j1..j4
    idx=[JS_ORDER.index(j) for j in [1,2,3,4]]; jsq=jsq[:,idx]; jsv=jsv[:,idx]; jse=jse[:,idx]
    tc=d["tcmd__recv"]-t0; tce=d["tcmd__effort"][:,:4]
    law=d["law__data"]; tl=d["law__recv"]-t0
    print(f"\n################ {nm}: run {r0:.1f}-{r1:.1f} s")
    m=(t>r0)&(t<r1)
    # reference sinusoid actually streamed: q_d per joint range, period via zero crossings of q2_d - mean
    for j in range(4):
        x=qd[m,j]; print(f"  q{j+1}_d range {x.min():6.1f}..{x.max():6.1f} deg (span {x.max()-x.min():5.1f}); q{j+1} meas range {q[m,j].min():6.1f}..{q[m,j].max():6.1f}")
    # q2_d period
    x=qd[m,1]-qd[m,1].mean(); zc=np.where(np.diff(np.sign(x))!=0)[0]; 
    if len(zc)>1: print(f"  q2_d zero-crossings at {t[m][zc]} -> half-period {np.diff(t[m][zc])}")
    qdot_d=np.gradient(qd[m],t[m],axis=0); print(f"  q_d peak rate (deg/s) {np.abs(qdot_d).max(0)}  ; measured qdot (velocity_observer) peak later")
    # tracking error per joint with lag estimate
    print("  joint | err mean | err rms | err max | lag (ms) via xcorr of detrended q vs q_d | tau cmd mean/rms/max | tau applied(js effort) mean | applied-cmd mean/rms")
    for j in range(4):
        e=q[m,j]-qd[m,j]
        x=q[m,j]-q[m,j].mean(); y=qd[m,j]-qd[m,j].mean()
        best=(0,-2)
        if y.std()>0.5:
            for L in range(0,400,2):
                c=np.corrcoef(x[L:],y[:len(y)-L])[0,1]
                if c>best[1]: best=(L,c)
        ecmd=np.interp(t[m],tc,tce[:,j]); eapp=np.interp(t[m],tj,jse[:,j])
        print(f"  j{j+1}   | {e.mean():+6.2f} | {np.sqrt((e**2).mean()):5.2f} | {np.abs(e).max():5.2f} | {best[0]*4:4d} ms (corr {best[1]:.3f}) | {tau[m,j].mean():+.3f}/{np.sqrt((tau[m,j]**2).mean()):.3f}/{np.abs(tau[m,j]).max():.3f} | {eapp.mean():+.3f} | {(eapp-ecmd).mean():+.3f}/{np.sqrt(((eapp-ecmd)**2).mean()):.3f}")
    # stiction signature: error vs reference velocity sign for j2/j3: when |qdot_d| small the error persists?
    for j in [1,2]:
        e=q[m,j]-qd[m,j]; v=qdot_d[:,j]
        for lo,hi in [(0,0.3),(0.3,1.0),(1.0,5)]:
            mm=(np.abs(v)>=lo)&(np.abs(v)<hi)
            if mm.sum()>100: print(f"  j{j+1} |qdot_d| in [{lo},{hi}) deg/s: {mm.sum()/250:5.1f} s, err mean {e[mm].mean():+.2f} rms {np.sqrt((e[mm]**2).mean()):.2f} deg, sign(err)*sign(qdot_d) mean {np.mean(np.sign(e[mm])*np.sign(v[mm])):+.2f}")
    # time table 2 s during the run: q2_d q2 q3_d q3 tau2 tau3
    print("   t   | q1_d q1 | q2_d  q2   err | q3_d  q3   err | q4_d q4 | tau1..4 cmd | dxl current-based effort j2 j3")
    for tt in np.arange(r0, r1, 2.0):
        mm=(t>tt-0.1)&(t<tt+0.1); mj=(tj>tt-0.1)&(tj<tt+0.1)
        if not mm.any(): continue
        print(f" {tt:5.1f} | {qd[mm,0].mean():5.1f} {q[mm,0].mean():5.1f} | {qd[mm,1].mean():5.1f} {q[mm,1].mean():5.1f} {q[mm,1].mean()-qd[mm,1].mean():+5.2f} | {qd[mm,2].mean():5.1f} {q[mm,2].mean():5.1f} {q[mm,2].mean()-qd[mm,2].mean():+5.2f} | {qd[mm,3].mean():5.1f} {q[mm,3].mean():5.1f} | {tau[mm].mean(0)} | {jse[mj,1].mean():+.3f} {jse[mj,2].mean():+.3f}")
    # velocity observer peaks, and law_debug written duty vs max effort
    tv=d["velobs__recv"]-t0; vv=np.degrees(d["velobs__velocity"][:,:4]); mv=(tv>r0)&(tv<r1); print(f"  velocity_observer |qdot| p99 (deg/s) in run: {np.percentile(np.abs(vv[mv]),99,axis=0)}  (order as published: {d['velobs__name'][0][:4]})")
    ml=(tl>r0)&(tl<r1); print(f"  law_debug[13..16] written duty |max| {np.abs(law[ml,13:17]).max(0)} ; law_debug[0] {np.unique(law[ml,0])[:5]} [26],[27] flags {np.unique(law[ml,26])} {np.unique(law[ml,27])}")
    # gravity-only vs dynamic: how much torque headroom: hardware max_effort N.m [0.34, 2.44, 1.42, 0.39]
    print(f"  torque |max| vs hardware caps [0.34 2.44 1.42 0.39]: {np.abs(tau[m]).max(0)/np.array([0.34,2.44,1.42,0.39])} of cap")
    # the DIRECT hold (arm static) joint error for comparison
    mh=(t>a+0.5)&(t<ex[0][0]-0.2); print(f"  hold arm err mean {np.mean(q[mh]-qd[mh],axis=0)} rms {np.sqrt(((q[mh]-qd[mh])**2).mean(0))} deg")

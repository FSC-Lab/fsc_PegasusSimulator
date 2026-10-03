import sys; sys.path.insert(0,"<scratch>")
from common import *
for nm in ["a1","a2"]:
    d,t0,a,b,ex,hold=load(nm)
    tm=d["mocap__hdr"]-t0; P=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); Q=np.column_stack([d[f"mocap__pose.orientation.{k}"] for k in "wxyz"]); E=eul(Q)
    new=np.r_[True,~np.all(np.diff(P,axis=0)==0,axis=1)]
    idx=np.where(new)[0]; tn=tm[idx]; Pn=P[idx]; En=E[idx]; Qn=Q[idx]; stp=qstep(Qn)
    dtn=np.diff(tn); gaps=np.where(dtn>0.020)[0]
    # armed/in-air window from vehicle_status + height
    print(f"\n######## {nm}: fresh mocap samples {len(idx)}, median spacing {np.median(dtn)*1000:.2f} ms; DIRECT {a:.1f}-{b:.1f}")
    print("  gap start |  dur ms | missing frames | z m   | yaw deg | pos x,y | first-after: dist to last-before / to linear extrapolation (mm) | orient steps >3deg within 0.5 s before/after")
    for g in gaps:
        i0,i1=g,g+1; t_a,t_b=tn[i0],tn[i1]
        # velocity from the 60 ms before the gap
        m=(tn>t_a-0.06)&(tn<=t_a)
        v=np.polyfit(tn[m]-t_a,Pn[m],1)[0] if m.sum()>=3 else np.zeros(3)
        extrap=Pn[i0]+v*(t_b-t_a)
        dl=np.linalg.norm(Pn[i1]-Pn[i0])*1000; de=np.linalg.norm(Pn[i1]-extrap)*1000
        pre=((tn[1:]>t_a-0.5)&(tn[1:]<=t_a)&(stp>3)).sum(); post=((tn[1:]>t_b)&(tn[1:]<t_b+0.5)&(stp>3)).sum()
        print(f"  {t_a:8.3f} | {1000*(t_b-t_a):7.0f} | {round((t_b-t_a)/0.00833)-1:5d} | {Pn[i0,2]:5.2f} | {En[i0,2]:7.1f} | {Pn[i0,0]:+.2f},{Pn[i0,1]:+.2f} | {dl:7.0f} / {de:7.0f} | {pre} / {post}")
    # heading-sector statistics of single missing frames during DIRECT (in-flight)
    mD=(tn[:-1]>a)&(tn[:-1]<b)
    print("  in DIRECT: frame-skip (>12 ms spacing) rate per heading sector:")
    for lo in range(-180,180,45):
        s=mD&(En[:-1,2]>=lo)&(En[:-1,2]<lo+45)
        if s.sum()>100: print(f"     [{lo:4d},{lo+45:4d}) samples {s.sum():5d}  skips {(dtn[s]>0.012).sum():4d} ({100*(dtn[s]>0.012).mean():.2f}%)  gaps>20ms {(dtn[s]>0.020).sum()}  orient steps>3deg {(stp[s]>3).sum()}")

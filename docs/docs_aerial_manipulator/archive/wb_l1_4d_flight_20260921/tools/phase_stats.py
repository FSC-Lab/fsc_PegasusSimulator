import numpy as np
np.set_printoptions(precision=4, suppress=True, linewidth=200)
d=np.load("/tmp/claude-1000/-home-shiqi-fsc-PegasusSimulator/439adc13-0e06-4d06-ac66-d9a5a6af280f/scratchpad/c3.npz",allow_pickle=True)
t0=d["wb__recv"][0]
def eul(q):  # q wxyz -> roll pitch yaw deg
    w,x,y,z=q.T
    r=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)); p=np.arcsin(np.clip(2*(w*y-z*x),-1,1)); yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return np.degrees(np.column_stack([r,p,yaw]))
def qstep(Q):
    dq=np.abs(np.sum(Q[1:]*Q[:-1],axis=1)); return 2*np.degrees(np.arccos(np.clip(dq,-1,1)))
phases={"SAFETY pre":(0.3,4.0),"DIRECT hold":(4.5,13.4),"DIRECT exec":(13.6,25.0),"SAFETY post":(26,40)}
# PX4 vehicle_attitude
tv=d["vatt__recv"]-t0; Qv=d["vatt__q"]  # px4 q is wxyz (q[0]=w)
Ev=eul(Qv); sv=qstep(Qv)
tm=d["mocap__hdr"]-t0; Qm=np.column_stack([d[f"mocap__pose.orientation.{a}"] for a in "wxyz"]); sm=qstep(Qm); Em=eul(Qm)
ti=d["imu__hdr"]-t0; G=np.column_stack([d[f"imu__angular_velocity.{a}"] for a in "xyz"]); gn=np.linalg.norm(G,axis=1)
tsc=d["sc__recv"]-t0; Gs=d["sc__gyro_rad"]; gsn=np.linalg.norm(Gs,axis=1)
print("=== per-sample orientation step (deg) and gyro, per phase ===")
print(f"{'phase':12s} | vatt step med/p99/max | implied rate p99 (deg/s) | mocap step med/p99/max | gyro |w| p50/p99 (deg/s) | sc gyro p99")
for nm,(a,b) in phases.items():
    mv=(tv[1:]>a)&(tv[1:]<b); mm=(tm[1:]>a)&(tm[1:]<b); mi=(ti>a)&(ti<b); ms=(tsc>a)&(tsc<b)
    print(f"{nm:12s} | {np.median(sv[mv]):.3f}/{np.percentile(sv[mv],99):.2f}/{sv[mv].max():.2f} | {np.percentile(sv[mv],99)*200:.0f} | {np.median(sm[mm]):.3f}/{np.percentile(sm[mm],99):.2f}/{sm[mm].max():.1f} | {np.degrees(np.percentile(gn[mi],50)):.1f}/{np.degrees(np.percentile(gn[mi],99)):.1f} | {np.degrees(np.percentile(gsn[ms],99)):.1f}")
print("\n=== attitude (PX4) per phase: roll/pitch/yaw mean, std, p-p ===")
for nm,(a,b) in phases.items():
    m=(tv>a)&(tv<b); E=Ev[m]
    print(f"{nm:12s} mean {E.mean(0)}  std {E.std(0)}  pp {E.max(0)-E.min(0)}")
print("\n=== PX4 yaw vs mocap yaw (deg): diff stats per phase (mocap yaw interp) ===")
for nm,(a,b) in phases.items():
    m=(tv>a)&(tv<b); ym=np.interp(tv[m],tm,np.unwrap(np.radians(Em[:,2]))); dy=np.degrees(np.angle(np.exp(1j*(np.radians(Ev[m,2])-ym))))
    rm=np.interp(tv[m],tm,Em[:,0]); pm=np.interp(tv[m],tm,Em[:,1])
    print(f"{nm:12s} yaw diff mean {dy.mean():+.2f} std {dy.std():.2f} | roll diff std {np.std(Ev[m,0]-rm):.2f} pitch diff std {np.std(Ev[m,1]-pm):.2f}")
print("quat reset counter unique:",np.unique(d["vatt__quat_reset_counter"]))
# e_R at full rate
t=d["wb__recv"]-t0; D=d["wb__data"]
def stats(nm,a,b):
    m=(t>a)&(t<b); eR=D[m,28:31]; n=np.linalg.norm(eR,axis=1)
    sc=np.mean(np.diff(np.sign(eR[:,0]))!=0)
    ep=D[m,48:51]-D[m,45:48]; u1=D[m,17]; mot=D[m,41:45]; tau=D[m,13:17]; ey=D[m,24:27]
    print(f"{nm:12s} |e_R| mean {n.mean():.3f} p99 {np.percentile(n,99):.3f} max {n.max():.3f} | e_R_x sign-change frac {sc:.2f} | e_p rms {np.sqrt((ep**2).mean(0))} max {np.abs(ep).max(0)} | u1 mean {u1.mean():.2f} std {u1.std():.2f} | motor std {mot.std(0)} min {mot.min():.2f} max {mot.max():.2f} | tau_j std {tau.std(0)} max| {np.abs(tau).max(0)} | e_y pos rms {np.sqrt((ey**2).mean(0))}")
print("\n=== law outputs per phase (DIRECT only meaningful) ===")
stats("DIRECT hold",4.5,13.4); stats("DIRECT exec",13.6,25.0); stats("exec 13.6-20",13.6,20.2); stats("exec 20.4-24.2",20.4,24.2)
# PSD of e_R_x, motors, u1 in hold
from numpy.fft import rfft, rfftfreq
def psdpeak(x,fs,nm):
    x=x-x.mean(); f=rfftfreq(len(x),1/fs); X=np.abs(rfft(x*np.hanning(len(x))))**2
    top=np.argsort(X[1:])[-4:][::-1]+1
    print(f"  {nm}: top freqs {f[top]} Hz ; power frac above 5 Hz {X[f>5].sum()/X[1:].sum():.2f}, above 15 Hz {X[f>15].sum()/X[1:].sum():.2f}")
for nm,(a,b) in [("DIRECT hold",(4.5,13.4)),("DIRECT exec",(13.6,20.2))]:
    m=(t>a)&(t<b); print(f"\nPSD {nm} (250 Hz):")
    psdpeak(D[m,28],250,"e_R_x"); psdpeak(D[m,29],250,"e_R_y"); psdpeak(D[m,17],250,"u1"); psdpeak(D[m,41],250,"motor0"); psdpeak(D[m,14],250,"tau_j2"); psdpeak(D[m,31],250,"d_hat_x"); psdpeak(D[m,21],250,"tau_body_x")
    mv=(tv>a)&(tv<b); psdpeak(Ev[mv,0],200,"PX4 roll (200Hz)"); psdpeak(Ev[mv,1],200,"PX4 pitch"); psdpeak(Ev[mv,2],200,"PX4 yaw")
    mm=(tm>a)&(tm<b); psdpeak(Em[mm,0],120,"mocap roll (120Hz)"); psdpeak(Em[mm,2],120,"mocap yaw")
    mi=(ti>a)&(ti<b); psdpeak(np.degrees(G[mi,0]),200,"gyro x (200Hz)"); psdpeak(np.degrees(G[mi,1]),200,"gyro y")
# tilt from PX4 attitude
R22=1-2*(Qv[:,1]**2+Qv[:,2]**2); tilt=np.degrees(np.arccos(np.clip(R22,-1,1)))
print("\n=== tilt (PX4) per phase ===")
for nm,(a,b) in phases.items():
    m=(tv>a)&(tv<b); print(f"{nm:12s} tilt mean {tilt[m].mean():.2f} p99 {np.percentile(tilt[m],99):.2f} max {tilt[m].max():.2f}")
# e_R as attitude error angle vs PX4-vs-reference: compare |e_R| with mocap-vs-px4 attitude difference sample-by-sample

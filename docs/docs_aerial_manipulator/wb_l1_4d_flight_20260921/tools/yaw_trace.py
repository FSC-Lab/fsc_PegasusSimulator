import numpy as np
np.set_printoptions(precision=3, suppress=True, linewidth=200)
d=np.load("/tmp/claude-1000/-home-shiqi-fsc-PegasusSimulator/439adc13-0e06-4d06-ac66-d9a5a6af280f/scratchpad/c3.npz",allow_pickle=True)
t0=d["wb__recv"][0]
def eul(q):
    w,x,y,z=q.T
    r=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)); p=np.arcsin(np.clip(2*(w*y-z*x),-1,1)); yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return np.degrees(np.column_stack([r,p,yaw]))
tv=d["vatt__recv"]-t0; Ev=eul(d["vatt__q"])
tm=d["mocap__hdr"]-t0; Em=eul(np.column_stack([d[f"mocap__pose.orientation.{a}"] for a in "wxyz"]))
tpe=d["px_eul__recv"]-t0; pe=np.column_stack([d["px_eul__x"],d["px_eul__y"],d["px_eul__z"]])
toe=d["opti_eul__recv"]-t0; oe=np.column_stack([d["opti_eul__x"],d["opti_eul__y"],d["opti_eul__z"]])
ti=d["imu__hdr"]-t0; gz=d["imu__angular_velocity.z"]
yaw_g=np.degrees(np.cumsum(gz[1:]*np.diff(ti)))  # integrated body z rate (approx yaw when level)
t=d["wb__recv"]-t0; D=d["wb__data"]
tr=d["wbref__hdr"]+0.048-t0; b1=np.column_stack([d["wbref__b1_d.x"],d["wbref__b1_d.y"]]); yaw_ref=np.degrees(np.arctan2(b1[:,1],b1[:,0]))
print("time  PX4yaw(NED q->?)  px_eul_z  opti_eul_z  mocap_yaw  gyroz_int  ref_yaw(model b1_d)  e_R_z  tau_body_z(FLU)  e_R(x,y)")
for tc in np.arange(3,27,0.5):
    iv=np.searchsorted(tv,tc); im=np.searchsorted(tm,tc); ip=np.searchsorted(tpe,tc); io=np.searchsorted(toe,tc); ii=np.searchsorted(ti,tc); iw=np.searchsorted(t,tc); ir=min(np.searchsorted(tr,tc),len(tr)-1)
    print(f"{tc:5.1f}  {Ev[iv,2]:8.1f}  {pe[ip,2]:8.1f}  {oe[io,2]:8.1f}  {Em[im,2]:8.1f}  {yaw_g[min(ii,len(yaw_g)-1)]:8.1f}  {yaw_ref[ir]:8.1f}  {D[iw,30]:+.3f}  {D[iw,23]:+.3f}  {D[iw,28:30]}")
print("\nvvo q (what PX4 receives) yaw vs mocap yaw at a few times:")
tvv=d["vvo__recv"]-t0; Evv=eul(d["vvo__q"])
for tc in [3,10,15,18,20,22,24]:
    i=np.searchsorted(tvv,tc); j=np.searchsorted(tm,tc); print(f"  t={tc}: vvo rpy {Evv[i]}  mocap rpy {Em[j]}  PX4 rpy {Ev[np.searchsorted(tv,tc)]}")
# velocity feedback noise
to=d["odom__hdr"]-t0; Vo=np.column_stack([d[f"odom__twist.twist.linear.{a}"] for a in "xyz"]); P=np.column_stack([d[f"mocap__pose.position.{a}"] for a in "xyz"])
from numpy.fft import rfft, rfftfreq
def psd(x,fs):
    x=x-x.mean(); f=rfftfreq(len(x),1/fs); X=np.abs(rfft(x*np.hanning(len(x))))**2; return f,X
print("\n=== odom velocity noise, DIRECT hold 4.5-13.4 ===")
m=(to>4.5)&(to<13.4)
for k,a in enumerate("xyz"):
    f,X=psd(Vo[m,k],120); top=np.argsort(X[1:])[-3:][::-1]+1
    print(f"  v{a}: std {Vo[m,k].std():.4f} m/s  top freqs {f[top]}  frac>5Hz {X[f>5].sum()/X[1:].sum():.2f}  frac>15Hz {X[f>15].sum()/X[1:].sum():.2f}")
mp=(tm>4.5)&(tm<13.4)
for k,a in enumerate("xyz"):
    f,X=psd(P[mp,k],120); top=np.argsort(X[1:])[-3:][::-1]+1
    print(f"  mocap p{a}: std {P[mp,k].std()*1000:.1f} mm  top freqs {f[top]}  frac>5Hz {X[f>5].sum()/X[1:].sum():.3f}; per-sample |dp| median {np.median(np.abs(np.diff(P[mp,k])))*1000:.2f} mm")
# is odom vel = FD of mocap position?
Vfd=np.diff(P,axis=0)/np.diff(tm)[:,None]
Vo_m=np.column_stack([np.interp(tm[1:],to,Vo[:,k]) for k in range(3)])
print("corr(odom v, FD mocap p) per axis in hold:", [np.corrcoef(Vo_m[mp[1:],k],Vfd[mp[1:],k])[0,1] for k in range(3)], " rms diff", np.sqrt(np.mean((Vo_m[mp[1:]]-Vfd[mp[1:]])**2,axis=0)))
# lagged correlation
for lag in [0,1,2,3]:
    c=[np.corrcoef(Vo_m[mp[1:]][lag:,k],Vfd[mp[1:]][:len(Vfd[mp[1:]])-lag,k])[0,1] for k in range(3)]; print("  lag",lag,c)

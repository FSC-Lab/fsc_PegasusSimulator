import sys; sys.path.insert(0,"<scratch>")
from common import *
d,t0,a,b,ex,hold=load("a1")
# IMU: sensor_combined accel (FRD, specific force), attitude from vehicle_attitude (FRD->NED); both on the PX4 clock
ts=d["sc__timestamp"]*1e-6; A=d["sc__accelerometer_m_s2"]; tq=d["vatt__timestamp_sample"]*1e-6; Qv=d["vatt__q"]
# map PX4 clock to bag time via receive times of sensor_combined
tsr=d["sc__recv"]-t0; off=np.median(tsr-ts)
def R_of(q):
    w,x,y,z=q.T
    return np.stack([np.stack([1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],-1),np.stack([2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],-1),np.stack([2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)],-1)],-2)
qi=np.column_stack([np.interp(ts,tq,Qv[:,k]) for k in range(4)]); qi/=np.linalg.norm(qi,axis=1)[:,None]
aw=np.einsum("nij,nj->ni",R_of(qi),A)+np.array([0,0,9.80665])   # NED kinematic acceleration
aw_enu=np.column_stack([aw[:,1],aw[:,0],-aw[:,2]])
tb=ts+off
tm=d["mocap__hdr"]-t0; P=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); new=np.r_[True,~np.all(np.diff(P,axis=0)==0,axis=1)]
tmn,Pn=tm[new],P[new]
def mocap_v(t,w=0.05):
    m=(tmn>t-w)&(tmn<t+w); c=np.polyfit(tmn[m]-t,Pn[m],1); return c[0],c[1]
def integrate(t_start,t_end,bias):
    m=(tb>=t_start)&(tb<=t_end); tt=tb[m]; aa=aw_enu[m]-bias
    v0,p0=mocap_v(t_start)
    dt=np.diff(tt,prepend=tt[0]); v=v0+np.cumsum(aa*dt[:,None],axis=0); p=p0+np.cumsum(v*dt[:,None],axis=0)
    return tt,p,v
# accel bias from a window with good mocap: mean IMU accel minus mocap-derived accel
mb=(tb>62)&(tb<75.4); mm=(tmn>62)&(tmn<75.4)
vfit=np.polyfit(tmn[mm],Pn[mm],2); acc_mocap=2*vfit[0]
bias=aw_enu[mb].mean(0)-acc_mocap
print("accel bias estimate ENU (m/s^2):",bias.round(3))
# validation: dead-reckon a 2.85 s window where mocap is valid, several windows
for s0 in [60.0,64.0,68.0,72.0]:
    tt,p,v=integrate(s0,s0+2.85,bias); j=np.argmin(np.abs(tmn-(s0+2.85)))
    print(f"validation {s0:.0f}-{s0+2.85:.2f} s: IMU end pos {p[-1].round(3)} vs mocap {Pn[j].round(3)} -> error {np.linalg.norm(p[-1][:2]-Pn[j][:2])*1000:.0f} mm")
tt,p,v=integrate(75.5095,78.47,bias)
for tc in [75.64,76.5,77.5,78.0,78.3585,78.4685]:
    k=np.argmin(np.abs(tt-tc)); print(f"IMU dead-reckoning t={tc:8.4f}: pos {p[k].round(3)}  vel {v[k].round(2)}")
print("mocap sample at 78.3585 (the 'stale' one): (0.510, 0.230, 1.065); first live sample at 78.4685: (-0.081, 1.308, 0.984)")

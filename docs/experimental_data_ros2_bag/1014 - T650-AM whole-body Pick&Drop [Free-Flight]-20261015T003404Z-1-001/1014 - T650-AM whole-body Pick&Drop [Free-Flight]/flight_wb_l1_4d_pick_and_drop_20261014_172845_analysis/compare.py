import numpy as np
np.set_printoptions(precision=3, suppress=True, linewidth=220)
d=np.load('hi.npz')
W=d['uav_0__fsc_autopilot_ros2__whole_body_direct_actuation__wb_control_debug']; wt=d['uav_0__fsc_autopilot_ros2__whole_body_direct_actuation__wb_control_debug__t']
m=W[:,0]==1; W=W[m]; wt=wt[m]
P=d['vrpn_mocap__uav_0__pose']; pt=d['vrpn_mocap__uav_0__pose__t']
I=d['uav_0__state_estimator__enu__imu__data']; it=d['uav_0__state_estimator__enu__imu__data__t']
B=d['uav_0__fmu__out__battery_status']; bt=d['uav_0__fmu__out__battery_status__t']
def rpy(q):
    x,y,z,w=q.T
    return np.degrees(np.stack([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)), np.arcsin(np.clip(2*(w*y-z*x),-1,1))],1))
att=rpy(P[:,3:7])
V=np.interp(wt,bt,B[:,0])
# battery regression of dFz using windows away from box
free=((wt>25)&(wt<76))|((wt>128.6)&(wt<131.7))
A=np.vstack([V[free],np.ones(free.sum())]).T
coef,*_=np.linalg.lstsq(A,W[free,33],rcond=None)
dFz_res=W[:,33]-(coef[0]*V+coef[1])
print('dFz battery fit: slope %.2f N/V'%coef[0])
wins=[('start hold z1.01 (open)',33.6,36.3),('pick hover z1.15',46.0,49.2),('place-start hold z0.70',70.5,76.2),
      ('OVER BOX approach z1.35',86.0,89.88),('OVER BOX hold z1.35',89.88,92.27),('over box z1.65 (abort hold)',103,115),('open hover (0,0) z1.68',128.6,131.7)]
print(f"{'window':30s} {'dur':>4} | {'pos std x/y/z mm':>20} | {'pitch sd':>8} {'roll sd':>7} | {'u1 sd N':>7} | {'tau_cmd pitch mean/sd':>22} {'tau_cmd roll mean/sd':>21} | {'dFz_res':>7} | {'az sd':>5}")
for name,a,b in wins:
    s=(wt>=a)&(wt<b); sp=(pt>=a)&(pt<b); si=(it>=a)&(it<b)
    xc=W[s,48:51]
    sd=xc.std(0)*1000
    tau=W[s,21:24]  # FLU commanded body torque
    print(f"{name:30s} {b-a:4.1f} | {sd[0]:6.1f} {sd[1]:6.1f} {sd[2]:6.1f} | {att[sp,1].std():8.2f} {att[sp,0].std():7.2f} | {W[s,17].std():7.2f} | {tau[:,1].mean():+9.3f} / {tau[:,1].std():.3f}    {tau[:,0].mean():+9.3f} / {tau[:,0].std():.3f} | {dFz_res[s].mean():+7.2f} | {I[si,2].std():5.2f}")
# spectrum of horizontal position at box hold vs abort hold
from numpy.fft import rfft, rfftfreq
for name,a,b in [('box approach+hold',86.0,92.27),('abort hold z1.65',103,115),('place start hold',70.5,76.2)]:
    s=(wt>=a)&(wt<b); x=W[s,48]-W[s,48].mean(); n=len(x)
    f=rfftfreq(n,1/250); X=np.abs(rfft(x*np.hanning(n)))
    k=np.argmax(X[1:])+1
    print(name,'dominant x freq %.2f Hz'%f[k], ' amp(pk-pk est) %.0f mm'%(np.ptp(x)*1000))

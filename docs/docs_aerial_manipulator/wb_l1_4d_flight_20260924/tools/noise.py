import sys; sys.path.insert(0,"<scratch>")
from common import *
def psd(x,fs):
    x=x-x.mean(); f=rfftfreq(len(x),1/fs); X=np.abs(rfft(x*np.hanning(len(x))))**2; return f,X
def bands(x,fs):
    f,X=psd(x,fs); tot=X[1:].sum(); top=f[np.argsort(X[1:])[-3:][::-1]+1]
    return f"top {np.round(top,1)} Hz | <1Hz {X[(f>0)&(f<1)].sum()/tot:.2f} 1-5 {X[(f>=1)&(f<5)].sum()/tot:.2f} 5-15 {X[(f>=5)&(f<15)].sum()/tot:.2f} >15 {X[f>=15].sum()/tot:.2f}"
for nm,lab in [("f18","0918 raw60"),("c3","0921#3 raw120"),("a1","0924#1 fused"),("a2","0924#2 fused")]:
    d,t0,a,b,ex,hold=load(nm); ah=a+0.5; bh=ex[0][0]-0.2
    print(f"\n######## {nm} {lab}: hold {ah:.1f}-{bh:.1f}")
    t=d["wb__recv"]-t0; D=d["wb__data"]; m=(t>ah)&(t<bh)
    to=d["odom__recv"]-t0; Vo=np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"]); mo=(to>ah)&(to<bh); fo=1/np.median(np.diff(to[mo]))
    print(f"  odom vx ({fo:.0f} Hz): {bands(Vo[mo,0],fo)}")
    print(f"  odom vz: {bands(Vo[mo,2],fo)}")
    print(f"  e_R_x (250): {bands(D[m,28],250)}")
    print(f"  e_R_y: {bands(D[m,29],250)}")
    print(f"  u1: {bands(D[m,17],250)}   u1 std {D[m,17].std():.3f} N, std of tick-to-tick diff {np.diff(D[m,17]).std():.3f} N")
    print(f"  motor0: {bands(D[m,41],250)}   motor std {D[m,41:45].std(0).mean():.4f}, tick-diff std {np.diff(D[m,41:45],axis=0).std(0).mean():.4f}")
    print(f"  tau_body_x FLU: {bands(D[m,21],250)}   std {D[m,21:24].std(0)}")
    print(f"  tau_j2: {bands(D[m,14],250)}  tick-diff std {np.diff(D[m,13:17],axis=0).std(0)}")
    print(f"  d_hat^c z deadbeat: {bands(D[m,64],250)}")
    ti=d["imu__hdr"]-t0; G=np.column_stack([d[f"imu__angular_velocity.{k}"] for k in "xyz"]); mi=(ti>ah)&(ti<bh); fi=1/np.median(np.diff(ti[mi]))
    print(f"  gyro x ({fi:.0f} Hz): {bands(np.degrees(G[mi,0]),fi)}")
    A=np.column_stack([d[f"imu__linear_acceleration.{k}"] for k in "xyz"]); print(f"  accel x: {bands(A[mi,0],fi)}  accel z: {bands(A[mi,2],fi)}")
    tv=d["vatt__recv"]-t0; Ev=eul(d["vatt__q"]); mv=(tv>ah)&(tv<bh); fv=1/np.median(np.diff(tv[mv])); print(f"  PX4 roll ({fv:.0f} Hz): {bands(Ev[mv,0],fv)}")
    # law-side actuator roughness = high-frequency motor content in absolute units: rms of motor cmd above 5 Hz
    f,X=psd(D[m,41],250); hf=np.sqrt(X[f>=5].sum()/X[1:].sum())*D[m,41].std(); print(f"  motor0 rms above 5 Hz: {hf:.4f} (of std {D[m,41].std():.4f})")
    f,X=psd(D[m,17],250); print(f"  u1 rms above 5 Hz: {np.sqrt(X[f>=5].sum()/X[1:].sum())*D[m,17].std():.3f} N")

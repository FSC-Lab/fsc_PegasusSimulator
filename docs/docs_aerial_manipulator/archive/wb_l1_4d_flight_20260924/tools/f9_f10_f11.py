import sys; sys.path.insert(0,"<scratch>"); sys.path.insert(0,"/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation")
from common import *
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
C={"blue":"#2a78d6","orange":"#eb6834","aqua":"#1baf7a","red":"#e34948","ink":"#0b0b0b","ink2":"#52514e","grid":"#e6e5e0","mute":"#a9a8a2","violet":"#4a3aa7"}
plt.rcParams.update({"font.size":9,"axes.edgecolor":C["ink2"],"axes.labelcolor":C["ink"],"xtick.color":C["ink2"],"ytick.color":C["ink2"],"axes.grid":True,"grid.color":C["grid"],"grid.linewidth":0.6,"axes.spines.top":False,"axes.spines.right":False,"figure.facecolor":"#fcfcfb","axes.facecolor":"#fcfcfb","legend.frameon":False})
R="/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/wb_l1_4d_flight_20260924/figures"
KT=np.array([162.4,154.0,150.5,153.4]); KPWM=np.array([169.47,149.70,135.25,148.51]); JS=[2,3,1,4]; idx=[JS.index(j) for j in (1,2,3,4)]
def lp(x,k=25): return np.convolve(x,np.ones(k)/k,'same')
# ---- F9: F3 and F4 arm tracking + torque
fig,axs=plt.subplots(3,2,figsize=(13,9),sharex="col")
for c,(nm,lab,(r0,r1)) in enumerate([("a3","Flight 3 (16:56) — fold 55°, q2 = 25° ± 15°, 12 s period",(20.62,48.62)),("a4","Flight 4 (17:21) — same angles, 6 s period",(20.09,48.08))]):
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]; m=(t>r0-2)&(t<r1+2); q=np.degrees(D[:,5:9]); qd=np.degrees(D[:,9:13])
    ax=axs[0,c]; ax.axhline(50,color=C["red"],lw=1); ax.text(r0-1.5,50.6,"+50° limit",color=C["red"],fontsize=8)
    ax.plot(t[m],qd[m,1],color=C["blue"],ls="--",lw=1.1,label="q2 reference"); ax.plot(t[m],q[m,1],color=C["blue"],lw=1.6,label="q2 measured"); ax.plot(t[m],qd[m,2],color=C["orange"],ls="--",lw=1.1,label="q3 reference"); ax.plot(t[m],q[m,2],color=C["orange"],lw=1.6,label="q3 measured")
    ax.set_title(lab,loc="left",fontweight="bold"); ax.set_ylabel("joint angle [deg]"); ax.set_ylim(0,55); ax.legend(fontsize=7.5,ncol=4,loc="lower left")
    ax=axs[1,c]; ax.plot(t[m],q[m,1]-qd[m,1],color=C["blue"],label="q2 error"); ax.plot(t[m],q[m,2]-qd[m,2],color=C["orange"],label="q3 error"); ax.axhline(0,color=C["ink2"],lw=0.6); ax.set_ylabel("q − q_d [deg]"); ax.legend(fontsize=7.5,loc="upper left")
    tl=d["law__recv"]-t0; L=d["law__data"]; ml=(tl>r0-2)&(tl<r1+2); tj=d["js__recv"]-t0; app=d["js__effort"][:,:4][:,idx]/KT; tc=d["tcmd__recv"]-t0; cmd=d["tcmd__effort"][:,:4]
    app_l=np.column_stack([np.interp(tl[ml],tj,app[:,k]) for k in range(4)]); cmd_l=np.column_stack([np.interp(tl[ml],tc,cmd[:,k]) for k in range(4)]); intd=cmd_l+(L[ml,18:22]+L[ml,22:26])/KPWM
    ax=axs[2,c]
    for j,col in [(1,C["blue"]),(2,C["orange"])]:
        ax.plot(tl[ml],lp(cmd_l[:,j]),color=col,ls="--",lw=1.1,label=f"τ{j+1} commanded by the law"); ax.plot(tl[ml],lp(intd[:,j]),color=col,ls=":",lw=1.3,label=f"τ{j+1} + arm-side friction FF and gravity correction"); ax.plot(tl[ml],lp(app_l[:,j]),color=col,lw=1.5,alpha=0.85,label=f"τ{j+1} applied (Kt·I)")
    ax.set_ylabel("joint torque, 10 Hz low-pass [N·m]"); ax.set_xlabel("time since bag start [s]"); ax.legend(fontsize=7,ncol=2,loc="upper left")
fig.tight_layout(); fig.savefig(f"{R}/f9_fast_arm_tracking.png",dpi=150,bbox_inches="tight")
# ---- F10: friction identification scatter (F3, F4) j2/j3
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as CT
P=TP.make_params_t650(); n=P["n"]; S_G=np.array([1.0,0.897,0.969,1.0]); FC=np.array([0.01711,0.03143,0.05751,0.05237]); MU=np.array([0,0.246,0.161,0]); W=0.015
R_NED_ENU=np.array([[0,1,0],[1,0,0],[0,0,-1.0]]); R_FRD_FLU=np.diag([1,-1,-1.0]); RZm=np.array([[0,1,0],[-1,0,0],[0,0,1.0]])
def R_of(q):
    w,x,y,z=q; return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
def gjoint(q,Rm):
    X=np.zeros(18+2*n); X[3:12]=Rm.reshape(9,order="F"); X[12:12+n]=q; return CT.dynamics(X,P)["g"][6:6+n]
fig,axs=plt.subplots(2,2,figsize=(12,7.6))
for c,(nm,lab,(r0,r1)) in enumerate([("a3","Flight 3, 12 s period",(20.62,48.62)),("a4","Flight 4, 6 s period",(20.09,48.08))]):
    d,t0,a,b,ex,hold=load(nm); tj=d["js__recv"]-t0; app=d["js__effort"][:,:4][:,idx]/KT; qj=d["js__position"][:,:4][:,idx]; tv=d["velobs__recv"]-t0; vob=d["velobs__velocity"][:,:4]; tq=d["vatt__recv"]-t0; Qv=d["vatt__q"]
    tt=np.arange(r0,r1,0.02); q=np.column_stack([np.interp(tt,tj,qj[:,k]) for k in range(4)]); ap=np.column_stack([np.interp(tt,tj,app[:,k]) for k in range(4)]); v=np.column_stack([np.interp(tt,tv,vob[:,k]) for k in range(4)]); acc=np.gradient(v,tt,axis=0)
    apf=np.column_stack([np.convolve(ap[:,j],np.ones(5)/5,'same') for j in range(4)])
    g=np.array([gjoint(q[i],R_NED_ENU@R_of(Qv[np.searchsorted(tq,ti)-1])@R_FRD_FLU@RZm) for i,ti in enumerate(tt)])
    res=apf-S_G*g-0.02*acc
    for r,(j,col) in enumerate([(1,C["blue"]),(2,C["orange"])]):
        ax=axs[r,c]; vd=np.degrees(v[:,j]); ax.scatter(vd,res[:,j],s=4,color=col,alpha=0.35,label="applied − S·g(q) − I·q̈, 50 Hz samples")
        vv=np.linspace(vd.min(),vd.max(),200); ff=(FC[j]+MU[j]*np.abs(apf[:,j]).mean())*np.tanh(np.radians(vv)/W); ax.plot(vv,ff,color=C["ink"],lw=1.6,label="ground-calibrated friction FF (fc + µ|τ|)·tanh(q̇/w)")
        mv=np.abs(v[:,j])>np.radians(1); A=np.column_stack([np.sign(v[mv,j]),v[mv,j],np.ones(mv.sum())]); cc=np.linalg.lstsq(A,res[mv,j],rcond=None)[0]
        ax.plot(vv,cc[0]*np.sign(vv)+cc[1]*np.radians(vv)+cc[2],color=C["red"],lw=1.4,ls="--",label=f"flight fit: Coulomb {cc[0]:.3f} N·m, viscous {cc[1]:.3f} N·m·s/rad")
        ax.axhline(0,color=C["ink2"],lw=0.6); ax.set_xlabel("measured joint velocity [deg/s]"); ax.set_ylabel(f"j{j+1} torque beyond gravity [N·m]"); ax.set_title(f"{lab} — joint {j+1}",loc="left",fontweight="bold",fontsize=9.5); ax.legend(fontsize=7,loc="upper left")
fig.tight_layout(); fig.savefig(f"{R}/f10_friction_id.png",dpi=150,bbox_inches="tight")
# ---- F11: VRPN timing F1 vs F3 vs F4
fig,axs=plt.subplots(1,3,figsize=(13,3.8))
for ax,(nm,lab,key) in zip(axs,[("a1","Flight 1 — /uav_0/mocap (raw VRPN not recorded)","mocap"),("a3","Flight 3 — /vrpn_mocap/uav_0/pose","vrpn"),("a4","Flight 4 — /vrpn_mocap/uav_0/pose","vrpn")]):
    d,t0,a,b,ex,hold=load(nm); th=d[f"{key}__hdr"]-t0
    if key=="mocap":
        P_=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); new=np.r_[True,~np.all(np.diff(P_,axis=0)==0,axis=1)]; th=th[new]
    dt=np.diff(th)*1000; ax.plot(th[1:],dt,lw=0.7,color=C["blue"]); ax.set_ylim(0,max(30,min(dt.max()*1.05,3000))); ax.set_yscale("log"); ax.set_ylim(5,3500)
    ax.axvspan(a,b,color=C["grid"],alpha=0.6); ax.set_title(lab,loc="left",fontweight="bold",fontsize=9); ax.set_xlabel("time [s]"); ax.set_ylabel("interval between fresh frames [ms]")
    ax.text(0.02,0.92,f"max {dt.max():.0f} ms, frames > 20 ms: {(dt>20).sum()}",transform=ax.transAxes,fontsize=8,color=C["ink"])
fig.tight_layout(); fig.savefig(f"{R}/f11_vrpn_timing.png",dpi=150,bbox_inches="tight"); print("ok")

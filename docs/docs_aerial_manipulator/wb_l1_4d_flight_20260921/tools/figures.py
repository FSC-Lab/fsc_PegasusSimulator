#!/usr/bin/env python3
"""Figures for the 2026-09-21 whole-body-L1-4D circle attempt (flight 3, 123637).

    PYTHONNOUSERSITE=1 /usr/bin/python3 figures.py <scratch dir with c1/c2/c3/f18.npz> figures/
"""
import sys, os
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from numpy.fft import rfft, rfftfreq
SURF="#fcfcfb"; INK="#0b0b0b"; INK2="#52514e"; INK3="#8a8985"; GRID="#e3e2dd"
S1,S2,S3,S4="#2a78d6","#eb6834","#1baf7a","#eda100"; REF="#8a8985"; BAND="#ecebe6"; CRIT="#e34948"
plt.rcParams.update({"figure.facecolor":SURF,"axes.facecolor":SURF,"savefig.facecolor":SURF,"font.family":"DejaVu Sans","font.size":9,
 "axes.edgecolor":GRID,"axes.labelcolor":INK2,"axes.titlesize":9.5,"axes.titleweight":"semibold","axes.titlecolor":INK,
 "xtick.color":INK3,"ytick.color":INK3,"xtick.labelcolor":INK2,"ytick.labelcolor":INK2,"grid.color":GRID,"grid.linewidth":0.7,
 "legend.frameon":False,"legend.fontsize":8,"axes.spines.top":False,"axes.spines.right":False,"lines.linewidth":1.2,"lines.solid_capstyle":"round"})
SRC, OUT = sys.argv[1], sys.argv[2]; os.makedirs(OUT, exist_ok=True)
T_DIR, T_EXEC, T_REV = 4.13, 13.51, 25.10; SPIKES=[20.28, 24.27]
def eul(q):
    w,x,y,z=q.T
    return np.degrees(np.column_stack([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),np.arcsin(np.clip(2*(w*y-z*x),-1,1)),np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))]))
def load(fn):
    d=np.load(fn,allow_pickle=True); t0=d["wb__recv"][0]; return d,t0
d,t0=load(f"{SRC}/c3.npz")
t=d["wb__recv"]-t0; D=d["wb__data"]
tm=d["mocap__hdr"]-t0; P=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); Qm=np.column_stack([d[f"mocap__pose.orientation.{k}"] for k in "wxyz"]); Em=eul(Qm)
to=d["odom__hdr"]-t0; Vo=np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"])
tv=d["vatt__recv"]-t0; Qv=d["vatt__q"]; Ev=eul(Qv); tilt=np.degrees(np.arccos(np.clip(1-2*(Qv[:,1]**2+Qv[:,2]**2),-1,1)))
tr=d["wbref__hdr"]+0.048-t0; xr=np.column_stack([d[f"wbref__x_cd.{k}"] for k in "xyz"]); yr=np.degrees(np.unwrap(np.arctan2(d["wbref__b1_d.y"],d["wbref__b1_d.x"])))+90  # model->actual
def frame(ax,ylab,last=False,x0=0,x1=41):
    ax.grid(True); ax.set_axisbelow(True); ax.set_xlim(x0,x1); ax.set_ylabel(ylab,fontsize=8.5); ax.yaxis.set_major_locator(MaxNLocator(4))
    ax.axvspan(T_DIR,T_REV,color=BAND,zorder=0,lw=0); ax.axvspan(T_EXEC,T_REV,color="#dcdad2",zorder=0,lw=0)
    for s in SPIKES: ax.axvline(s,color=CRIT,lw=0.8,ls=":",zorder=1)
    if last: ax.set_xlabel("time from record start  [s]",fontsize=8.5)
    else: ax.tick_params(labelbottom=False)
# ---------- f1 overview ----------
fig,ax=plt.subplots(5,1,figsize=(10,10),sharex=True)
for k,(lab,c) in enumerate(zip("xyz",(S1,S2,S3))):
    ax[0].plot(t,D[:,48+k],color=c,label=f"CoM {lab} (law)"); ax[0].plot(tr,xr[:,k],color=c,ls="--",lw=0.9,alpha=0.8)
ax[0].plot([],[],color=REF,ls="--",label="reference (dashed)"); frame(ax[0],"base CoM [m]"); ax[0].legend(ncol=4,loc="upper left")
ax[0].set_title("Flight 3 (12:36:37) overview - grey = DIRECT, darker grey = go-to-start executing, red dotted = mocap rigid-body flips")
yaw_px=np.degrees(np.unwrap(np.radians(90-Ev[:,2]))); ax[1].plot(tv,yaw_px,color=S1,label="PX4 yaw, ENU (law's R)")
keep=np.r_[True,np.abs(np.diff(Em[:,2]))<60]; ax[1].plot(tm[keep],np.degrees(np.unwrap(np.radians(Em[keep,2]))),color=S3,lw=0.8,label="mocap yaw (flip samples removed)")
m=(tr>T_DIR)&(tr<T_REV); ax[1].plot(tr[m],yr[m],color=REF,ls="--",label="heading reference"); frame(ax[1],"heading [deg]"); ax[1].legend(ncol=3,loc="upper left")
ax[2].plot(tv,tilt,color=S1,label="tilt (PX4)"); ax[2].plot(t,np.linalg.norm(D[:,28:31],axis=1)*57.3,color=S2,lw=0.7,label="|e_R| x 57.3 (law attitude error)"); frame(ax[2],"[deg]"); ax[2].legend(ncol=2,loc="upper left"); ax[2].set_ylim(0,40)
ax[3].plot(t,D[:,17],color=S1,lw=0.8,label="u1 collective (law)"); ax[3].axhline(3.746170*9.80665,color=REF,ls="--",lw=0.8); frame(ax[3],"thrust [N]"); ax[3].set_ylim(-90,180); ax[3].legend(loc="upper left")
ax[4].plot(to,np.linalg.norm(Vo,axis=1),color=S1,lw=0.8,label="|v| odometry (mocap-derived, law's v)"); ax[4].plot(d["vodom_px__recv"]-t0,np.linalg.norm(d["vodom_px__velocity"],axis=1),color=S3,lw=0.8,label="|v| PX4 EKF (not used by the law)")
frame(ax[4],"speed [m/s]",last=True); ax[4].set_ylim(0,10); ax[4].legend(ncol=2,loc="upper left")
fig.tight_layout(); fig.savefig(f"{OUT}/f1_overview.png",dpi=130); plt.close(fig)
# ---------- f2 spike anatomy at 24.27 ----------
a,b=24.05,24.95
fig,ax=plt.subplots(5,1,figsize=(10,9),sharex=True)
def fr2(ax,ylab,last=False):
    ax.grid(True); ax.set_axisbelow(True); ax.set_xlim(a,b); ax.set_ylabel(ylab,fontsize=8.5); ax.yaxis.set_major_locator(MaxNLocator(4))
    for s in [24.266,24.274,24.283,24.290,24.307,24.315]: ax.axvline(s,color=CRIT,lw=0.6,ls=":")
    if last: ax.set_xlabel("time [s]",fontsize=8.5)
    else: ax.tick_params(labelbottom=False)
mm=(tm>a)&(tm<b)
for k,(lab,c) in enumerate(zip("xyz",(S1,S2,S3))): ax[0].plot(tm[mm],P[mm,k]-P[mm,k][0],color=c,marker="o",ms=2.5,label=f"mocap {lab} - first")
fr2(ax[0],"position step [m]"); ax[0].legend(ncol=3,loc="upper left"); ax[0].set_title("Mocap rigid-body flip at t = 24.27 s: six alternating one-sample jumps of 0.18 m and ~110 deg, and what the law did with them")
ax[1].plot(tm[mm],Em[mm,2],color=S1,marker="o",ms=2.5,label="mocap yaw"); fr2(ax[1],"mocap yaw [deg]"); ax[1].legend(loc="upper left")
mo=(to>a)&(to<b); ax[2].plot(to[mo],Vo[mo,0],color=S1,marker="o",ms=2.5,label="odom vx (mocap FD)"); ax[2].plot(to[mo],Vo[mo,2],color=S3,marker="o",ms=2.5,label="odom vz"); fr2(ax[2],"velocity [m/s]"); ax[2].legend(ncol=2,loc="upper left")
mw=(t>a)&(t<b); ax[3].plot(t[mw],D[mw,17],color=S1,label="u1"); ax3b=ax[3]; fr2(ax[3],"u1 [N]"); ax[3].legend(loc="upper left")
for k,c in enumerate((S1,S2,S3,S4)): ax[4].plot(t[mw],D[mw,41+k],color=c,lw=0.9,label=f"motor {k}")
fr2(ax[4],"motor cmd [0-1]",last=True); ax[4].legend(ncol=4,loc="upper left"); ax[4].set_ylim(-0.05,1.05)
fig.tight_layout(); fig.savefig(f"{OUT}/f2_spike_anatomy.png",dpi=130); plt.close(fig)
# ---------- f3 feedback quality across flights ----------
runs=[("0918 (60 Hz)",f"{SRC}/f18.npz",(10.2,27.8)),("0921 #1 (60 Hz)",f"{SRC}/c1.npz",(6,50)),("0921 #2 (60 Hz)",f"{SRC}/c2.npz",(19.1,60)),("0921 #3 (120 Hz)",f"{SRC}/c3.npz",(4.6,13.3))]
cols=[S3,S1,S4,S2]
fig,ax=plt.subplots(1,3,figsize=(12,3.6))
for (nm,fn,(a_,b_)),c in zip(runs,cols):
    dd,tt0=load(fn); tmm=dd["mocap__hdr"]-tt0; mmm=(tmm>a_)&(tmm<b_); dtt=np.diff(tmm[mmm])*1000
    h,e=np.histogram(dtt,bins=np.arange(3,20,0.5)); ax[0].step(e[:-1],100*h/h.sum(),where="post",color=c,label=nm)
    too=dd["odom__hdr"]-tt0; vv=dd["odom__twist.twist.linear.x"]; mo_=(too>a_)&(too<b_); x=vv[mo_]-vv[mo_].mean(); fs=1/np.median(np.diff(too[mo_]))
    N=256; segs=[x[i:i+N]*np.hanning(N) for i in range(0,len(x)-N,N//2)]; f=rfftfreq(N,1/fs); X=np.mean([np.abs(rfft(sg))**2 for sg in segs],axis=0)/N; ax[1].semilogy(f[1:],X[1:],color=c,lw=1.1,label=nm)
ax[0].set_title("mocap timestamp interval, DIRECT hold"); ax[0].set_xlabel("dt [ms]"); ax[0].set_ylabel("samples [%]"); ax[0].legend(); ax[0].grid(True); ax[0].set_axisbelow(True)
ax[1].set_title("odometry vx Welch spectrum (the law's velocity feedback)"); ax[1].set_xlabel("frequency [Hz]"); ax[1].set_ylabel("power [a.u.]"); ax[1].legend(); ax[1].grid(True); ax[1].set_xlim(0,60)
# bars: odom v std x, d_hat^c std x, |e_R| mean, u1 std
labs=["odom vx std\n[cm/s]","deadbeat d_hat_x\nstd [N]","|e_R| mean\n[x100]","u1 std\n[N]"]; vals=[]
for nm,fn,(a_,b_) in runs:
    dd,tt0=load(fn); too=dd["odom__hdr"]-tt0; mo_=(too>a_)&(too<b_); tt=dd["wb__recv"]-tt0; DD=dd["wb__data"]; mw_=(tt>a_)&(tt<b_)
    vals.append([dd["odom__twist.twist.linear.x"][mo_].std()*100, DD[mw_,62].std(), np.linalg.norm(DD[mw_,28:31],axis=1).mean()*100, DD[mw_,17].std()])
vals=np.array(vals); w=0.2
for i,((nm,_,_),c) in enumerate(zip(runs,cols)):
    ax[2].bar(np.arange(4)+(i-1.5)*w,vals[i],w,color=c,label=nm)
    for j in range(4): ax[2].text(j+(i-1.5)*w,vals[i,j],f"{vals[i,j]:.1f}",ha="center",va="bottom",fontsize=6.5,color=INK2)
ax[2].set_xticks(range(4)); ax[2].set_xticklabels(labs,fontsize=7.5); ax[2].set_title("hold-phase noise budget"); ax[2].grid(True,axis="y"); ax[2].set_axisbelow(True)
fig.tight_layout(); fig.savefig(f"{OUT}/f3_feedback_quality.png",dpi=130); plt.close(fig)
# ---------- f4 orientation jitter vs heading ----------
ang=2*np.degrees(np.arccos(np.clip(np.abs(np.sum(Qm[1:]*Qm[:-1],axis=1)),-1,1)))
bins=np.arange(-180,181,20); cen=(bins[:-1]+bins[1:])/2; frac=[]; med=[]
for lo,hi in zip(bins[:-1],bins[1:]):
    m=(Em[1:,2]>=lo)&(Em[1:,2]<hi); frac.append(100*np.mean(ang[m]>3) if m.sum()>50 else np.nan); med.append(np.median(ang[m]) if m.sum()>50 else np.nan)
fig,ax=plt.subplots(1,2,figsize=(10,3.4))
ax[0].bar(cen,frac,18,color=S1); ax[0].set_xlabel("mocap heading [deg]"); ax[0].set_ylabel("samples with >3 deg step  [%]"); ax[0].set_title("orientation jitter vs heading (flight 3)"); ax[0].grid(True,axis="y"); ax[0].set_axisbelow(True)
sp=np.where(np.linalg.norm(np.diff(P,axis=0),axis=1)>0.05)[0]
for i in sp[::2]: ax[0].axvline(Em[i,2],color=CRIT,lw=0.8,ls=":")
ax[0].plot([],[],color=CRIT,ls=":",label="heading at each rigid-body flip"); ax[0].legend(loc="upper right")
ax[1].plot(tm[1:],ang,color=S1,lw=0.6); ax[1].set_ylim(0,15); ax[1].set_xlim(0,41); ax[1].axvspan(T_DIR,T_REV,color=BAND,zorder=0,lw=0); ax[1].axvspan(T_EXEC,T_REV,color="#dcdad2",zorder=0,lw=0)
ax[1].set_xlabel("time [s]"); ax[1].set_ylabel("orientation step per sample [deg]"); ax[1].set_title("orientation step per sample (clipped; flips reach 110 deg)"); ax[1].grid(True); ax[1].set_axisbelow(True)
fig.tight_layout(); fig.savefig(f"{OUT}/f4_orientation_jitter.png",dpi=130); plt.close(fig)
# ---------- f5 transition tracking ----------
a,b=13.0,25.2; mw=(t>a)&(t<b)
fig,ax=plt.subplots(4,1,figsize=(10,8),sharex=True)
def fr5(ax,ylab,last=False):
    ax.grid(True); ax.set_axisbelow(True); ax.set_xlim(a,b); ax.set_ylabel(ylab,fontsize=8.5); ax.yaxis.set_major_locator(MaxNLocator(4))
    for s in SPIKES: ax.axvline(s,color=CRIT,lw=0.8,ls=":")
    if last: ax.set_xlabel("time [s]",fontsize=8.5)
    else: ax.tick_params(labelbottom=False)
for k,(lab,c) in enumerate(zip("xyz",(S1,S2,S3))): ax[0].plot(t[mw],(D[mw,48+k]-D[mw,45+k])*1000,color=c,lw=0.8,label=f"CoM err {lab}")
fr5(ax[0],"CoM error [mm]"); ax[0].legend(ncol=3,loc="upper left"); ax[0].set_ylim(-150,150); ax[0].set_title("Go-to-start transition (13.5 s onward): tracking was clean until the mocap flips")
for k,(lab,c) in enumerate(zip("xyz",(S1,S2,S3))): ax[1].plot(t[mw],D[mw,24+k]*1000,color=c,lw=0.8,label=f"EE err {lab}")
ax[1].plot(t[mw],np.degrees(np.arcsin(np.clip(D[mw,27],-1,1))),color=S4,lw=0.8,label="EE heading err [deg]"); fr5(ax[1],"EE error [mm | deg]"); ax[1].legend(ncol=4,loc="upper left"); ax[1].set_ylim(-40,40)
for k,c in enumerate((S1,S2,S3,S4)):
    ax[2].plot(t[mw],np.degrees(D[mw,5+k]),color=c,lw=0.9,label=f"q{k+1}"); ax[2].plot(t[mw],np.degrees(D[mw,9+k]),color=c,ls="--",lw=0.7)
fr5(ax[2],"arm joints [deg]"); ax[2].legend(ncol=4,loc="lower left")
for k,c in enumerate((S1,S2,S3,S4)): ax[3].plot(t[mw],D[mw,13+k],color=c,lw=0.8,label=f"tau j{k+1}")
fr5(ax[3],"arm torque [N.m]",last=True); ax[3].legend(ncol=4,loc="upper left")
fig.tight_layout(); fig.savefig(f"{OUT}/f5_transition.png",dpi=130); plt.close(fig)
print("done")

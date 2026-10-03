import sys; sys.path.insert(0,"<scratch>")
from common import *
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
C={"blue":"#2a78d6","orange":"#eb6834","aqua":"#1baf7a","yellow":"#eda100","magenta":"#e87ba4","green":"#008300","violet":"#4a3aa7","red":"#e34948","ink":"#0b0b0b","ink2":"#52514e","grid":"#e6e5e0"}
plt.rcParams.update({"font.size":9,"axes.edgecolor":C["ink2"],"axes.labelcolor":C["ink"],"xtick.color":C["ink2"],"ytick.color":C["ink2"],"axes.grid":True,"grid.color":C["grid"],"grid.linewidth":0.6,"axes.spines.top":False,"axes.spines.right":False,"figure.facecolor":"#fcfcfb","axes.facecolor":"#fcfcfb","lines.linewidth":1.6,"legend.frameon":False})
R="/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/wb_l1_4d_flight_20260924/figures"
def ld(nm):
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]
    tm=d["mocap__hdr"]-t0; P=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); Q=np.column_stack([d[f"mocap__pose.orientation.{k}"] for k in "wxyz"])
    to=d["odom__recv"]-t0; Po=np.column_stack([d[f"odom__pose.pose.position.{k}"] for k in "xyz"]); Vo=np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"])
    tr=d["wbref__recv"]-t0; xr=np.column_stack([d[f"wbref__x_cd.{k}"] for k in "xyz"]); red=np.column_stack([d[f"wbref__r_ed.{k}"] for k in "xyz"])
    return dict(d=d,t0=t0,a=a,b=b,ex=ex,hold=hold,t=t,D=D,tm=tm,P=P,Q=Q,to=to,Po=Po,Vo=Vo,tr=tr,xr=xr,red=red)
A=ld("a1"); B_=ld("a2")
# ---------------- F1: flight 1 overview (xy + timeline)
fig=plt.figure(figsize=(12,8.2)); gs=fig.add_gridspec(3,2,width_ratios=[1.05,1.3],height_ratios=[1,1,1],hspace=0.35,wspace=0.22)
ax=fig.add_subplot(gs[:,0]); d=A
run=(d["tr"]>35.9)&(d["tr"]<78.1); ax.plot(d["xr"][run,0],d["xr"][run,1],color=C["ink2"],lw=1.2,ls="--",label="CoM reference x_cd (circle run)")
fresh=~np.r_[False,np.all(np.diff(d["P"],axis=0)==0,axis=1)]
mr=(d["tm"]>35.9)&(d["tm"]<75.51)&fresh; ax.plot(d["P"][mr,0],d["P"][mr,1],color=C["blue"],lw=1.4,label="mocap (fresh samples), run to freeze")
mo=(d["to"]>75.5)&(d["to"]<78.1); ax.plot(d["Po"][mo,0],d["Po"][mo,1],color=C["orange"],lw=2.2,label="fused estimate during freeze 75.5–78.1 s")
ma=(d["tm"]>78.55)&(d["tm"]<80.7)&fresh; ax.plot(d["P"][ma,0],d["P"][ma,1],color=C["red"],lw=2.0,label="mocap after re-acquisition 78.6–80.7 s (SAFETY→STAB)")
i=np.searchsorted(d["tm"],75.51); ax.plot(d["P"][i,0],d["P"][i,1],"o",ms=9,mfc="none",mec=C["red"],mew=2); ax.annotate("mocap lost 75.5 s\n(yaw 135°)",(d["P"][i,0],d["P"][i,1]),xytext=(0.75,0.65),color=C["ink"],arrowprops=dict(arrowstyle="-",color=C["ink2"]))
j=np.searchsorted(d["tm"],78.6); ax.annotate("re-acquired 78.6 s\n1.35 m away",(d["P"][j,0],d["P"][j,1]),xytext=(-1.6,1.1),color=C["ink"],arrowprops=dict(arrowstyle="-",color=C["ink2"]))
k=np.searchsorted(d["tm"],80.7); ax.annotate("touchdown 80.8 s\n3.7 m from the freeze point",(d["P"][k,0],d["P"][k,1]),xytext=(-1.75,2.4),color=C["ink"],arrowprops=dict(arrowstyle="-",color=C["ink2"]))
ax.set_aspect("equal"); ax.set_xlabel("world x [m]"); ax.set_ylabel("world y [m]"); ax.set_title("Flight 1 — base path (bird view)",loc="left",fontweight="bold"); ax.legend(loc="lower left",fontsize=8)
t=d["t"]; D=d["D"]; ep=np.linalg.norm(D[:,48:51]-D[:,45:48],axis=1)*1000; m=(t>11.8)&(t<78.05)
ax1=fig.add_subplot(gs[0,1]); ax1.plot(t[m],ep[m],color=C["blue"]); ax1.set_ylabel("|x_c − x_cd| [mm]"); ax1.set_title("Flight 1 — timeline",loc="left",fontweight="bold")
ax2=fig.add_subplot(gs[1,1],sharex=ax1); ax2.plot(t[m],np.linalg.norm(D[m,28:31],axis=1),color=C["orange"]); ax2.set_ylabel("|e_R| [–]")
ax3=fig.add_subplot(gs[2,1],sharex=ax1); ax3.plot(t[m],D[m,17],color=C["aqua"]); ax3.set_ylabel("u1 [N]"); ax3.set_xlabel("time since bag start [s]")
for axx in (ax1,ax2,ax3):
    axx.axvspan(24.8,30.8,color=C["grid"],alpha=0.8); axx.axvspan(35.9,78.09,color=C["grid"],alpha=0.4); axx.axvspan(75.51,78.35,color=C["red"],alpha=0.18)
    axx.axvline(55.07,color=C["red"],lw=0.8,ls=":")
ax1.text(25.2,ax1.get_ylim()[1]*0.85,"go-to-\nstart",fontsize=8,color=C["ink2"]); ax1.text(37,ax1.get_ylim()[1]*0.85,"circle run ×2 (52 s planned)",fontsize=8,color=C["ink2"]); ax1.text(62,ax1.get_ylim()[1]*0.85,"mocap\nfrozen",fontsize=8,color=C["red"]); ax1.text(50.5,ax1.get_ylim()[1]*0.55,"108 ms\npause",fontsize=7,color=C["red"])
fig.savefig(f"{R}/f1_flight1_overview.png",dpi=150,bbox_inches="tight"); plt.close(fig)
# ---------------- F2: freeze anatomy
fig,axs=plt.subplots(4,1,figsize=(11,9),sharex=True); d=A; w=(74.0,81.0)
mm=(d["tm"]>w[0])&(d["tm"]<w[1]); mo=(d["to"]>w[0])&(d["to"]<w[1]); mr=(d["tr"]>w[0])&(d["tr"]<w[1]); mt=(d["t"]>w[0])&(d["t"]<w[1])
ax=axs[0]; ax.plot(d["tm"][mm],d["P"][mm,0],color=C["blue"],label="mocap x (as published)"); ax.plot(d["to"][mo],d["Po"][mo,0],color=C["orange"],label="fused estimate x (what the law reads)"); ax.plot(d["tr"][mr],d["xr"][mr,0],color=C["ink2"],ls="--",label="reference x_cd"); ax.set_ylabel("x [m]"); ax.legend(loc="upper left",fontsize=8,ncol=3); ax.set_title("Flight 1 — anatomy of the mocap freeze",loc="left",fontweight="bold")
ax=axs[1]; ax.plot(d["tm"][mm],d["P"][mm,1],color=C["blue"]); ax.plot(d["to"][mo],d["Po"][mo,1],color=C["orange"]); ax.plot(d["tr"][mr],d["xr"][mr,1],color=C["ink2"],ls="--"); ax.set_ylabel("y [m]")
ax=axs[2]; e=(d["D"][:,48:51]-d["D"][:,45:48])*1000; ax.plot(d["t"][mt],e[mt,0],color=C["blue"],label="e_x"); ax.plot(d["t"][mt],e[mt,1],color=C["orange"],label="e_y"); ax.set_ylabel("x_c − x_cd [mm]"); ax.legend(loc="upper left",fontsize=8,ncol=2)
ax=axs[3]; ax.plot(d["t"][mt],d["D"][mt,17],color=C["aqua"],label="u1 [N]"); ax.plot(d["t"][mt],np.linalg.norm(d["D"][mt,28:31],axis=1)*100,color=C["violet"],label="|e_R| ×100"); ax.set_ylabel("u1 [N], |e_R|×100"); ax.set_xlabel("time since bag start [s]"); ax.legend(loc="upper left",fontsize=8,ncol=2)
for ax in axs:
    ax.axvspan(75.51,78.35,color=C["red"],alpha=0.15); ax.axvline(78.09,color=C["ink"],lw=1,ls=":"); ax.axvline(79.67,color=C["ink"],lw=1,ls=":")
axs[0].text(75.6,axs[0].get_ylim()[0]+0.02,"rigid body lost — pose republished frozen",color=C["red"],fontsize=8); axs[0].text(78.15,axs[0].get_ylim()[1]-0.1,"revert→SAFETY",fontsize=8,rotation=90,va="top"); axs[0].text(79.72,axs[0].get_ylim()[1]-0.1,"pilot→STAB, EKF reset",fontsize=8,rotation=90,va="top")
fig.savefig(f"{R}/f2_freeze_anatomy.png",dpi=150,bbox_inches="tight"); plt.close(fig)
# ---------------- F3: arm tracking
fig,axs=plt.subplots(3,2,figsize=(12,8),sharex="col")
for c,(d,lab,r0,r1) in enumerate([(A,"Flight 1 (2 laps, q2 period 48 s)",35.9,78.05),(B_,"Flight 2 (1 lap = half a q2 cycle, period 48 s)",22.94,51.0)]):
    t=d["t"]; D=d["D"]; m=(t>r0-3)&(t<r1); q=np.degrees(D[:,5:9]); qd=np.degrees(D[:,9:13])
    ax=axs[0,c]; ax.plot(t[m],qd[m,1],color=C["ink2"],ls="--",label="q2 reference"); ax.plot(t[m],q[m,1],color=C["blue"],label="q2 measured"); ax.plot(t[m],qd[m,2],color=C["ink2"],ls=":",label="q3 reference"); ax.plot(t[m],q[m,2],color=C["orange"],label="q3 measured"); ax.set_ylabel("joint angle [deg]"); ax.set_title(lab,loc="left",fontweight="bold"); ax.legend(fontsize=8,ncol=2,loc="upper right")
    ax=axs[1,c]; ax.plot(t[m],q[m,1]-qd[m,1],color=C["blue"],label="q2 error"); ax.plot(t[m],q[m,2]-qd[m,2],color=C["orange"],label="q3 error"); ax.axhline(0,color=C["ink2"],lw=0.6); ax.set_ylabel("q − q_d [deg]"); ax.legend(fontsize=8,loc="upper right")
    ax=axs[2,c]; ax.plot(t[m],D[m,14],color=C["blue"],label="τ2 commanded"); ax.plot(t[m],D[m,15],color=C["orange"],label="τ3 commanded"); ax.set_ylabel("joint torque [N·m]"); ax.set_xlabel("time since bag start [s]"); ax.legend(fontsize=8,loc="upper right")
    if c==0:
        for ax in axs[:,0]: ax.axvspan(75.51,78.05,color=C["red"],alpha=0.15)
fig.savefig(f"{R}/f3_arm_tracking.png",dpi=150,bbox_inches="tight"); plt.close(fig)
# ---------------- F4: feedback / configuration comparison (bars, one measure per panel)
rows=[("0918\nraw 60 Hz\nUSB",{}),("0921 #3\nraw 120 Hz\nUSB",{}),("0924 #1\nfused\nEth+400 Hz",{}),("0924 #2\nfused\nEth+400 Hz",{})]
vals={"law velocity noise\n(odom v_x std, hold) [cm/s]":[2.60,10.84,1.38,2.19],"deadbeat d̂_x std (hold) [N]":[13.7,91.5,9.8,9.2],"collective u1 rms > 5 Hz [N]":[0.42,1.09,0.06,0.07],"|e_R| mean, DIRECT hold [–]":[0.033,0.074,0.021,0.040],"timesync round trip, median [ms]":[3.67,3.71,0.48,0.45],"law tick dt, p99 [ms]":[5.33,5.81,5.16,5.27],"motor cmd rms > 5 Hz [–]":[0.031,0.033,0.047,0.041],"gyro power > 15 Hz, x axis [fraction]":[0.76,0.32,0.86,0.89]}
fig,axs=plt.subplots(2,4,figsize=(13,6.2)); cols=[C["ink2"],C["ink2"],C["blue"],C["blue"]]
for ax,(k,v) in zip(axs.flat,vals.items()):
    ax.bar(range(4),v,color=cols,width=0.62); ax.set_xticks(range(4)); ax.set_xticklabels([r[0] for r in rows],fontsize=7); ax.set_title(k,fontsize=8.5,loc="left"); ax.grid(axis="x",visible=False)
    for i,x in enumerate(v): ax.text(i,x,f"{x:g}",ha="center",va="bottom",fontsize=7.5,color=C["ink"])
fig.suptitle("Hardware-configuration comparison, DIRECT hold window of each flight (grey = before, blue = 2026-09-24)",x=0.01,ha="left",fontsize=10,fontweight="bold"); fig.tight_layout()
fig.savefig(f"{R}/f4_config_compare.png",dpi=150,bbox_inches="tight"); plt.close(fig)
# ---------------- F5: circle tracking, EE (FK) vs reference, and base error vs heading
fig,axs=plt.subplots(1,3,figsize=(13,4.4))
for ax,(d,lab,r0,r1) in zip(axs[:2],[(A,"Flight 1: EE, 1.57 laps to the freeze",35.9,75.5),(B_,"Flight 2: EE, 1 lap",22.94,50.94)]):
    dd=d["d"]; te=dd["pl_current_ee__recv"]-d["t0"]; Pe=np.column_stack([dd[f"pl_current_ee__pose.position.{k}"] for k in "xyz"]); me=(te>r0)&(te<r1)
    mr=(d["tr"]>r0)&(d["tr"]<r1); ax.plot(d["red"][mr,0],d["red"][mr,1],color=C["ink2"],ls="--",label="EE reference (r = 0.50 m)"); ax.plot(Pe[me,0],Pe[me,1],color=C["blue"],label="EE, FK of measured base+joints")
    ax.set_aspect("equal"); ax.set_title(lab,loc="left",fontweight="bold",fontsize=9); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend(fontsize=7.5,loc="lower left")
ax=axs[2]; d=A; t=d["t"]; D=d["D"]; m=(t>35.9)&(t<75.5); Em=eul(d["Q"]); yaw=np.interp(t[m],d["tm"],np.unwrap(np.radians(Em[:,2])))
ep=D[m,48:51]-D[m,45:48]; c,s=np.cos(yaw),np.sin(yaw); ebx=c*ep[:,0]+s*ep[:,1]; eby=-s*ep[:,0]+c*ep[:,1]
ax.plot(t[m],ep[:,0]*1000,color=C["blue"],label="e_x world"); ax.plot(t[m],ep[:,1]*1000,color=C["orange"],label="e_y world"); ax.plot(t[m],np.linalg.norm(D[m,28:31],axis=1)*1000,color=C["violet"],lw=1,label="|e_R| ×1000")
ax.set_title("Flight 1: base error is lap-periodic\n(one lap = 24 s), tied to |e_R|",loc="left",fontweight="bold",fontsize=9); ax.set_xlabel("time [s]"); ax.set_ylabel("x_c − x_cd [mm]"); ax.legend(fontsize=7.5,loc="lower left",ncol=3)
fig.tight_layout(); fig.savefig(f"{R}/f5_circle_tracking.png",dpi=150,bbox_inches="tight"); plt.close(fig)
print("figures written")

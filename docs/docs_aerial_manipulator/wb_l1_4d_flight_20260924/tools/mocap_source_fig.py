import sys; sys.path.insert(0,"<scratch>")
from common import *
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
exec(open("imu_deadreckon.py").read().split("tt,p,v=integrate(75.5095")[0].split("print(\"accel bias")[0])
C={"blue":"#2a78d6","orange":"#eb6834","aqua":"#1baf7a","red":"#e34948","ink":"#0b0b0b","ink2":"#52514e","grid":"#e6e5e0","mute":"#a9a8a2"}
plt.rcParams.update({"font.size":9,"axes.edgecolor":C["ink2"],"axes.labelcolor":C["ink"],"xtick.color":C["ink2"],"ytick.color":C["ink2"],"axes.grid":True,"grid.color":C["grid"],"grid.linewidth":0.6,"axes.spines.top":False,"axes.spines.right":False,"figure.facecolor":"#fcfcfb","axes.facecolor":"#fcfcfb","legend.frameon":False})
R="/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/wb_l1_4d_flight_20260924/figures"
d,t0,a,b,ex,hold=load("a1")
tm=d["mocap__hdr"]-t0; P=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); new=np.r_[True,~np.all(np.diff(P,axis=0)==0,axis=1)]
fig,axs=plt.subplots(2,2,figsize=(13,8.4))
# (a) the long freeze vs IMU
ax=axs[0,0]; tt,p,v=integrate(75.5095,78.60,bias)
w=(tm>74.6)&(tm<78.8)
ax.plot(tm[w&new&(tm<75.52)],P[w&new&(tm<75.52),1],".",ms=3,color=C["blue"],label="mocap, fresh frames")
ax.plot(tm[w&~new],P[w&~new,1],".",ms=3,color=C["mute"],label="mocap, republished frozen frame (60 Hz)")
ax.plot(tm[w&new&(tm>78.46)],P[w&new&(tm>78.46),1],".",ms=3,color=C["blue"])
i=np.argmin(np.abs(tm-78.3585)); ax.plot(tm[i],P[i,1],"o",ms=8,mfc="none",mec=C["red"],mew=2,label="the one frame that arrived at 78.36 s")
ax.plot(tt,p[:,1],color=C["orange"],lw=1.8,label="drone's own IMU, dead-reckoned from 75.51 s")
k=np.argmin(np.abs(tt-75.64)); ax.plot([tt[k],tm[i]],[p[k,1],P[i,1]],color=C["red"],lw=1,ls=":"); ax.annotate("same pose as the IMU at 75.64 s:\nmeasured then, delivered 2.72 s late",(tm[i],P[i,1]),xytext=(76.3,0.07),fontsize=8,color=C["ink"],arrowprops=dict(arrowstyle="-",color=C["ink2"]))
ax.set_title("(a) The 2.85 s freeze: the first frame back was 2.7 s old",loc="left",fontweight="bold"); ax.set_xlabel("time [s]"); ax.set_ylabel("world y [m]"); ax.legend(fontsize=7.5,loc="upper left")
# (b) short gap replay at 55.07
ax=axs[0,1]; w=(tm>54.98)&(tm<55.30)&new; t_=tm[w]; y_=P[w,1]*1000; x_=P[w,0]*1000
s=np.cumsum(np.r_[0,np.linalg.norm(np.diff(P[w][:,:2],axis=0),axis=1)])*1000
pre=t_<55.071; ax.plot(t_[pre],s[pre],"o",ms=4,color=C["blue"],label="frames before the pause")
rep=(t_>55.1)&(t_<55.225); ax.plot(t_[rep],s[rep],"o",ms=4,color=C["red"],label="frames after the pause: ~100 ms late, normal 8 ms cadence")
post=t_>55.225; ax.plot(t_[post],s[post],"o",ms=4,color=C["aqua"],label="caught up: live again")
m=(t_<55.071); c=np.polyfit(t_[m][-6:],s[m][-6:],1); tl=np.linspace(55.0,55.3,50); ax.plot(tl,np.polyval(c,tl),color=C["ink2"],lw=1,ls="--",label="where live frames would be")
ax.axvspan(55.070,55.178,color=C["grid"]); ax.text(55.08,s.max()*0.9,"108 ms\nno frames",fontsize=8,color=C["ink2"])
ax.set_title("(b) A 108 ms pause at 55.07 s: late frames replayed, then a skip",loc="left",fontweight="bold"); ax.set_xlabel("time [s]"); ax.set_ylabel("distance travelled [mm]"); ax.legend(fontsize=7.5,loc="lower right")
# (c) drone-WiFi traffic did not pause
ax=axs[1,0]
for k,lab,col in [("wb","whole-body law debug, 250 Hz (drone → ground station)",C["blue"]),("vvo","mocap forwarded to PX4 (ground station → drone → ground station)",C["orange"])]:
    tr=d[f"{k}__recv"]-t0; m=(tr>73.5)&(tr<80.0); ax.plot(tr[m][1:],np.diff(tr[m])*1000,lw=1,color=col,label=lab)
ax.axvspan(75.51,78.36,color=C["red"],alpha=0.12); ax.text(75.6,30,"mocap input to the\nground station silent",fontsize=8,color=C["red"])
ax.set_ylim(0,60); ax.set_title("(c) The drone's WiFi kept flowing both ways during the freeze",loc="left",fontweight="bold"); ax.set_xlabel("time [s]"); ax.set_ylabel("interval between arrivals [ms]"); ax.legend(fontsize=7.5,loc="upper left")
# (d) heading of the in-flight gaps
ax=axs[1,1]
bins=np.arange(-180,181,15); tot=np.zeros(len(bins)-1)
for nm in ["a1","a2"]:
    dd,tt0,aa,bb,_,_=load(nm); tmm=dd["mocap__hdr"]-tt0; E=eul(np.column_stack([dd[f"mocap__pose.orientation.{k}"] for k in "wxyz"])); m=(tmm>aa)&(tmm<bb)
    h,_=np.histogram(E[m,2],bins); tot+=h/120.0
ax.bar(bins[:-1]+7.5,tot,width=13,color=C["mute"],label="time spent at each heading in DIRECT, both flights [s]")
for yaw in (-171.9,135.2,139.1): ax.axvline(yaw,color=C["red"],lw=1.6)
ax.text(-168,tot.max()*0.62,"F1 55.07 s",rotation=90,ha="left",va="top",fontsize=8,color=C["red"])
ax.text(131,tot.max()*0.62,"F1 75.51 s (freeze)\nF2 37.19 s",rotation=90,ha="right",va="top",fontsize=8,color=C["red"])
ax.set_xlim(-180,180); ax.set_xticks(range(-180,181,45)); ax.set_xlabel("drone heading (ENU yaw) [deg]"); ax.set_ylabel("seconds")
ax.set_title("(d) All three in-flight pauses at headings the drone held 7% of the time",loc="left",fontweight="bold"); ax.legend(fontsize=7.5,loc="upper right",bbox_to_anchor=(0.97,1.0))
fig.tight_layout(); fig.savefig(f"{R}/f8_mocap_source.png",dpi=150,bbox_inches="tight"); print("ok")

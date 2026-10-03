import sys; sys.path.insert(0,"<scratch>")
from common import *
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
C={"blue":"#2a78d6","orange":"#eb6834","red":"#e34948","ink":"#0b0b0b","ink2":"#52514e","grid":"#e6e5e0"}
plt.rcParams.update({"font.size":9,"axes.edgecolor":C["ink2"],"axes.labelcolor":C["ink"],"xtick.color":C["ink2"],"ytick.color":C["ink2"],"axes.grid":True,"grid.color":C["grid"],"grid.linewidth":0.6,"axes.spines.top":False,"axes.spines.right":False,"figure.facecolor":"#fcfcfb","axes.facecolor":"#fcfcfb","legend.frameon":False})
R="/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/wb_l1_4d_flight_20260924/figures"
RUNS=[("s_new1","Simulation — new design: fold 55°, q2 = 25° ± 15°, 12 s period",None),
      ("s_old1","Simulation — old design: fold 60°, q2 = 30° ± 10°, 48 s period",None),
      ("a1","Hardware flight 1 — old design (for reference)",75.5)]
fig,axs=plt.subplots(3,1,figsize=(12,9.2),sharex=True)
for ax,(nm,title,cut) in zip(axs,RUNS):
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]
    Ts=[float(s.split("T=")[1].rstrip("s")) for x,s in ex]; k=int(np.argmax(Ts)); r0=ex[k][0]; r1=min(r0+Ts[k],b-0.05,cut or 1e9)
    ts=d["sc__recv"]-t0; ms=(ts>r0)&(ts<r1); tsc=d["sc__timestamp"][ms]*1e-6; rtf=(tsc[-1]-tsc[0])/(ts[ms][-1]-ts[ms][0])
    m=(t>r0)&(t<r1); tp=(t[m]-r0)*rtf; q=np.degrees(D[m,5:9]); qd=np.degrees(D[m,9:13])
    ax.axhline(50,color=C["red"],lw=1.0); ax.text(0.5,50.6,"q2 / q3 limit, +50°",color=C["red"],fontsize=8)
    ax.plot(tp,qd[:,1],color=C["blue"],ls="--",lw=1.2,label="q2 reference"); ax.plot(tp,q[:,1],color=C["blue"],lw=1.7,label="q2 measured")
    ax.plot(tp,qd[:,2],color=C["orange"],ls="--",lw=1.2,label="q3 reference"); ax.plot(tp,q[:,2],color=C["orange"],lw=1.7,label="q3 measured")
    i=np.argmax(q[:,2]); ax.plot(tp[i],q[i,2],"o",ms=5,mfc="none",mec=C["ink"]); ax.annotate(f"q3 peak {q[i,2]:.1f}°",(tp[i],q[i,2]),xytext=(8,-14),textcoords="offset points",fontsize=8,color=C["ink"])
    ax.set_title(title,loc="left",fontweight="bold"); ax.set_ylabel("joint angle [deg]"); ax.set_ylim(5,54)
axs[0].legend(loc="lower right",ncol=4,fontsize=8); axs[-1].set_xlabel("time since the circle run started, physics time [s] (hardware flight 1 ends at the mocap freeze)")
fig.tight_layout(); fig.savefig(f"{R}/f7_q2_design_sim.png",dpi=150,bbox_inches="tight"); print("ok")

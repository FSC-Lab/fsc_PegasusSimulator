import sys; sys.path.insert(0,"<scratch>")
from common import *
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
C={"blue":"#2a78d6","orange":"#eb6834","red":"#e34948","ink":"#0b0b0b","ink2":"#52514e","grid":"#e6e5e0","base":"#8a8984"}
plt.rcParams.update({"font.size":11.5,"xtick.labelsize":10.5,"ytick.labelsize":10.5,"axes.edgecolor":C["ink2"],"axes.labelcolor":C["ink"],"xtick.color":C["ink2"],"ytick.color":C["ink2"],"axes.grid":True,"grid.color":C["grid"],"grid.linewidth":0.6,"axes.spines.top":False,"axes.spines.right":False,"figure.facecolor":"#fcfcfb","axes.facecolor":"#fcfcfb","legend.frameon":False})
R="/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/wb_l1_4d_flight_20260924/figures"
# (npz, title, window, fk-check end, phases: (label, t0, t1, shade alpha or None = frozen feedback))
FL=[("a1","Flight 1 · 30° ± 10°, 48 s\nstopped at 1.57 laps",24.8,78.05,75.5,[("go-to-\nstart",24.81,30.84,0.55),("",30.84,35.9,0.25),("circle run",35.9,75.51,0),("mocap\nfrozen",75.51,78.05,None)]),
    ("a2","Flight 2 · 30° ± 10°, 48 s\none lap = half a cycle",16.2,57.7,57.7,[("go-to-\nstart",16.17,20.27,0.55),("",20.27,22.94,0.25),("circle run",22.94,50.94,0),("",50.94,57.7,0.25)]),
    ("a3","Flight 3 · 25° ± 15°, 12 s\none lap",12.2,52.1,52.1,[("go-to-\nstart",12.24,18.60,0.55),("",18.60,20.62,0.25),("circle run",20.62,48.62,0),("",48.62,52.1,0.25)]),
    ("a4","Flight 4 · 25° ± 15°, 6 s\none lap",12.7,55.5,55.5,[("go-to-\nstart",12.74,17.41,0.55),("",17.41,20.09,0.25),("circle run",20.09,48.08,0),("",48.08,55.5,0.25)])]
fig,axs=plt.subplots(4,4,figsize=(15,11),sharex="col",sharey="row")
stats=[]
for c,(nm,title,w0,w1,wfk,ph) in enumerate(FL):
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]; m=(t>w0)&(t<w1)
    arm=D[:,24:27]*1000; base=(D[:,48:51]-D[:,45:48])*1000; absd=arm+base
    # heading error: the law's asin(e_y[3]), sign fixed against the planner's independent FK heading
    te=d["pl_current_ee__recv"]-t0; Qe=np.column_stack([d[f"pl_current_ee__pose.orientation.{k}"] for k in "wxyz"]); w,x,y,z=Qe.T
    tr=d["wbref__recv"]-t0; hr=np.arctan2(np.interp(te,tr,d["wbref__b1_de.y"]),np.interp(te,tr,d["wbref__b1_de.x"]))
    hfk=np.degrees(np.angle(np.exp(1j*(np.arctan2(2*(x*y+w*z),1-2*(y*y+z*z))-hr-np.pi/2))))
    hlaw=np.degrees(np.arcsin(np.clip(D[:,27],-1,1)))
    me=(te>w0)&(te<wfk)
    sgn=np.sign(np.corrcoef(hfk[me],np.interp(te[me],t,hlaw))[0,1]); hdg=sgn*hlaw
    print(nm,"heading sign vs FK:",sgn,"; rms diff law vs FK heading err",np.sqrt(((hfk[me]-np.interp(te[me],t,hdg))**2).mean()).round(2),"deg")
    for r,(k,lab) in enumerate([(0,"x"),(1,"y"),(2,"z")]):
        ax=axs[r,c]
        ax.plot(t[m],base[m,k],color=C["base"],lw=1.0,ls="--",label="base CoM error (x_c − x_cd)")
        ax.plot(t[m],absd[m,k],color=C["blue"],lw=1.3,label="EE absolute error, world (measured − planned)")
        ax.plot(t[m],arm[m,k],color=C["orange"],lw=1.2,label="EE task error the law tracks (CoM-anchored)")
        ax.axhline(0,color=C["ink2"],lw=0.6)
        if c==0: ax.set_ylabel(f"EE error {lab} [mm]")
    ax=axs[3,c]; ax.plot(t[m],hdg[m],color=C["blue"],lw=1.2,label="EE heading error (measured − reference)"); ax.axhline(0,color=C["ink2"],lw=0.6)
    if c==0: ax.set_ylabel("EE yaw error [deg]")
    ax.set_xlabel("time since bag start [s]")
    axs[0,c].set_title(title,loc="left",fontweight="bold",fontsize=11)
    for ax in axs[:,c]:
        for lab,p0,p1,al in ph:
            if al is None: ax.axvspan(p0,p1,color=C["red"],alpha=0.16)
            elif al>0: ax.axvspan(p0,p1,color=C["grid"],alpha=al*1.6)
    for lab,p0,p1,al in ph:
        if lab:
            axs[0,c].text((p0+p1)/2,0.97,lab,transform=axs[0,c].get_xaxis_transform(),ha="center",va="top",fontsize=9.5,color=C["red"] if al is None else C["ink2"])
    for lab,p0,p1,al in ph:
        if al is None: continue
        mm=(t>p0+0.1)&(t<p1-0.1)
        name={"go-to-\nstart":"go-to-start","circle run":"circle run"}.get(lab,"hold")
        stats.append((nm,name,np.sqrt((absd[mm]**2).mean(0)),np.abs(absd[mm]).max(0),np.sqrt((arm[mm]**2).mean(0)),np.abs(arm[mm]).max(0),np.sqrt((hdg[mm]**2).mean()),np.abs(hdg[mm]).max(),hdg[mm].mean()))
h,l=axs[0,0].get_legend_handles_labels(); h2,l2=axs[3,0].get_legend_handles_labels()
fig.legend(h+h2,l+l2,loc="upper center",ncol=2,bbox_to_anchor=(0.5,1.035),fontsize=11)
fig.tight_layout(rect=(0,0,1,0.975)); fig.subplots_adjust(wspace=0.07); fig.savefig(f"{R}/f_ee_errors.png",dpi=130,bbox_inches="tight"); plt.close(fig)
print("\nflight | phase | ABS rms x/y/z | ABS max x/y/z | TASK rms x/y/z | TASK max x/y/z | yaw rms / max / mean")
for s in stats: print(f"{s[0]} | {s[1]:12s} | {s[2].round(1)} | {s[3].round(0)} | {s[4].round(1)} | {s[5].round(1)} | {s[6]:.2f} / {s[7]:.2f} / {s[8]:+.2f}")

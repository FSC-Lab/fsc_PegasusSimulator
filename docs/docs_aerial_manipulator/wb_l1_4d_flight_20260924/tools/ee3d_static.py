import json, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
C={"blue":"#2a78d6","orange":"#eb6834","aqua":"#1baf7a","ink":"#0b0b0b","ink2":"#52514e","mute":"#b9b8b2"}
plt.rcParams.update({"font.size":9,"figure.facecolor":"#fcfcfb","axes.facecolor":"#fcfcfb","legend.frameon":False})
D=json.load(open("<scratch>/ee3d.json"))
fig=plt.figure(figsize=(13,10.2))
for i,k in enumerate(["f1","f2","f3","f4"]):
    f=D[k]; ax=fig.add_subplot(2,2,i+1,projection="3d")
    A=lambda n: np.array(f[n]); ee,er,com,cr,base,lap=A("ee"),A("ee_ref"),A("com"),A("com_ref"),A("base"),A("lap")
    two=lap.sum()>=20
    if not two: lap[:]=0
    ax.plot(*er.T,color=C["ink2"],ls="--",lw=1.1,label="EE planned (r = 0.50 m)")
    ax.plot(*cr.T,color=C["ink2"],ls=":",lw=1.1,label="CoM planned")
    ax.plot(*com.T,color=C["aqua"],lw=1.4,label="CoM measured")
    ax.plot(*ee[lap==0].T,color=C["blue"],lw=1.8,label="EE measured, lap 1" if two else "EE measured")
    if two: ax.plot(*ee[lap==1].T,color=C["orange"],lw=1.8,label="EE measured, lap 2")
    for j in range(0,len(ee),50): ax.plot([base[j,0],ee[j,0]],[base[j,1],ee[j,1]],[base[j,2],ee[j,2]],color=C["mute"],lw=0.9)
    ax.scatter(*ee[0],color=C["ink"],s=18); ax.text(*(ee[0]+[0,0,0.03]),"start",fontsize=8,color=C["ink"])
    P=np.vstack([ee,er,com,cr]); rng=P.max(0)-P.min(0); ax.set_box_aspect(rng/rng.max())
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]",labelpad=6); ax.view_init(elev=30,azim=-58)
    ax.set_title(f["label"],loc="left",fontweight="bold",fontsize=9.5)
    ax.set_zticks(np.round([P[:,2].min(),P[:,2].max()],2)); ax.tick_params(axis="z",pad=2)
    ax.legend(loc="upper left",fontsize=7.5,bbox_to_anchor=(0.0,0.98))
fig.text(0.01,0.01,"Grey whiskers join the measured base to the measured EE every 2 s. True vertical scale.",fontsize=8,color=C["ink2"])
fig.tight_layout(); fig.savefig("/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/wb_l1_4d_flight_20260924/figures/f_circle_3d.png",dpi=130,bbox_inches="tight")
print("ok")

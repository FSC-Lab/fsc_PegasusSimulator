import sys; sys.path.insert(0,"<scratch>")
from common import *
for nm,(g0,r1) in [("a1",(24.8,75.5)),("a2",(16.2,57.7))]:
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]; m=(t>g0)&(t<r1)
    ey=D[:,24:27]; ex_=D[:,48:51]-D[:,45:48]; absd=ey+ex_
    te=d["pl_current_ee__recv"]-t0; Pe=np.column_stack([d[f"pl_current_ee__pose.position.{k}"] for k in "xyz"]); Qe=np.column_stack([d[f"pl_current_ee__pose.orientation.{k}"] for k in "wxyz"])
    tr=d["wbref__recv"]-t0; red=np.column_stack([d[f"wbref__r_ed.{k}"] for k in "xyz"]); b1e=np.column_stack([d[f"wbref__b1_de.{k}"] for k in "xyz"])
    me=(te>g0)&(te<r1); redi=np.column_stack([np.interp(te[me],tr,red[:,k]) for k in range(3)]); fk=Pe[me]-redi
    ab_i=np.column_stack([np.interp(te[me],t,absd[:,k]) for k in range(3)]); ey_i=np.column_stack([np.interp(te[me],t,ey[:,k]) for k in range(3)])
    print(f"\n### {nm}: FK-r_ed vs (e_y+e_x): rms diff {np.sqrt(((fk-ab_i)**2).mean(0))*1000} mm ; vs (e_y-e_x) {np.sqrt(((fk-(ey_i-np.column_stack([np.interp(te[me],t,ex_[:,k]) for k in range(3)])))**2).mean(0))*1000}; vs -(e_y+e_x) {np.sqrt(((fk+ab_i)**2).mean(0))*1000}")
    print("   rms FK-r_ed", np.sqrt((fk**2).mean(0))*1000, " rms e_y", np.sqrt((ey[m]**2).mean(0))*1000, " rms e_x", np.sqrt((ex_[m]**2).mean(0))*1000)
    # yaw: current_ee x axis heading vs b1_de heading
    w,x,y,z=Qe[me].T; R00=1-2*(y*y+z*z); R10=2*(x*y+w*z); yaw_ee=np.arctan2(R10,R00)
    R02=2*(x*z+w*y); R12=2*(y*z-w*x); R01=2*(x*y-w*z); R11=1-2*(x*x+z*z)
    for lab,(cx,cy) in {"x-axis":(R00,R10),"y-axis":(R01,R11),"z-axis":(R02,R12)}.items():
        h=np.arctan2(cy,cx); hr=np.arctan2(np.interp(te[me],tr,b1e[:,1]),np.interp(te[me],tr,b1e[:,0]))
        dd=np.degrees(np.angle(np.exp(1j*(h-hr)))); print(f"   heading of current_ee {lab} - b1_de: median {np.median(dd):+.1f} std {dd.std():.2f} deg")
    hdg=np.degrees(np.arcsin(np.clip(D[:,27],-1,1))); print(f"   asin(e_y[3]) rms {np.sqrt((hdg[m]**2).mean()):.2f} max {np.abs(hdg[m]).max():.2f} mean {hdg[m].mean():+.2f}; b1_de z comp max {np.abs(b1e[:,2]).max():.3f}")
    print("   b1_d vs b1_de heading diff median:", np.degrees(np.median(np.angle(np.exp(1j*(np.arctan2(d['wbref__b1_de.y'],d['wbref__b1_de.x'])-np.arctan2(d['wbref__b1_d.y'],d['wbref__b1_d.x'])))))))

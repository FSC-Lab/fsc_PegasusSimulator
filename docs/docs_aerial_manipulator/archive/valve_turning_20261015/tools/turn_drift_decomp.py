"""(2026-10-18, the airframe-drift analysis; run with PYTHONNOUSERSITE=1 /usr/bin/python3 from the repo root, args = run tags)
Decompose the valve-turn drift: law CoM error (dbg [45..50]) in the body frame
(longitudinal = nose/tangent, lateral = radial, vertical), e_R, d_hat_t in body frame,
contact truth, per phase and binned by plan angle."""
import sys, numpy as np
R="docs/docs_aerial_manipulator/archive/valve_turning_20261015/runs/"
def q2R(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
def at(tq,ts,X): return np.column_stack([np.interp(tq,ts,X[:,k]) for k in range(X.shape[1])])
def yaw_of(q): return np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2]**2+q[:,3]**2))
np.set_printoptions(precision=1,suppress=True,linewidth=200)
tags=sys.argv[1:] or ["base_n1"]
for tag in tags:
    d=np.load(R+tag+".npz",allow_pickle=True)
    marks=dict(zip(d["marks_name"].tolist(),d["marks_t"].tolist()))
    dbg=d["dbg"]; od=d["odom"]; ref=d["ref"]; con=d["contact"]; box=d["box"]
    t=dbg[:,0]; D=dbg[:,2:]
    e_x=D[:,48:51]-D[:,45:48]            # CoM actual - ref (world)
    eR=D[:,28:31]; dt_w=D[:,31:34]; u1=D[:,17]; ey=D[:,24:28]; q=D[:,5:9]; qd=D[:,9:13]
    we=D[:,72:78]; Fraw=D[:,97:101]; chi=D[:,105]
    Q=at(t,od[:,0],od[:,7:11])
    psi=yaw_of(Q)
    # body frame: x = nose (tangent), y = left (radial inward or outward?), z up
    ex_b=np.zeros_like(e_x); dt_b=np.zeros_like(dt_w)
    for i in range(len(t)):
        Rb=q2R(Q[i]); ex_b[i]=Rb.T@e_x[i]; dt_b[i]=Rb.T@dt_w[i]
    # sign of lateral: toward the valve axis = + ? axis = box[0,1:3]
    axis=box[0,1:3]
    pos=at(t,od[:,0],od[:,1:4])
    # radial unit from axis to base; "inward" component of e_x
    rad=pos[:,:2]-axis; rad/= np.linalg.norm(rad,axis=1)[:,None]
    ex_rad=np.einsum('ij,ij->i',e_x[:,:2],rad)          # + = outward
    tang=np.column_stack([-rad[:,1],rad[:,0]])
    ex_tan=np.einsum('ij,ij->i',e_x[:,:2],tang)         # + = counter-clockwise (turn direction?)
    # contact truth: t, ... find force cols: assume [1:4] force world
    tc=con[:,0]
    print(f"\n===== {tag}: {'ABORTED '+str(d['reason']) if bool(d['aborted']) else 'complete'}")
    print("contact row sample:", con[len(con)//2].round(3))
    def row(name,t0,t1):
        m=(t>=t0)&(t<=t1)
        if m.sum()<10: return
        f=at(t[m],tc,con[:,1:4]) if len(con) else np.zeros((m.sum(),3))
        print(f"  {name:22s} n={m.sum():5d} | e_x body lon/lat/z mm: mean {1e3*ex_b[m].mean(0)} std {1e3*ex_b[m].std(0)}"
              f" | e_x rad/tan mm: mean {1e3*ex_rad[m].mean():6.1f}/{1e3*ex_tan[m].mean():6.1f} std {1e3*ex_rad[m].std():5.1f}/{1e3*ex_tan[m].std():5.1f}"
              f" | e_R mean {eR[m].mean(0)*1e3} mrad std {eR[m].std(0)*1e3}"
              f" | d_t body N mean {dt_b[m].mean(0)} std {dt_b[m].std(0)} | u1 {u1[m].mean():.2f}"
              f" | |Fcontact| mean {np.linalg.norm(f,axis=1).mean():.2f} N | e_y mm {1e3*ey[m,:3].mean(0)} | chi_free {chi[m].mean():.0f}"
              f" | q-qd deg mean {np.degrees((q[m]-qd[m]).mean(0))}")
    for name in ["go_to_start","ready","exit","go_to_land_start"]:
        if f"{name}:start" in marks: row(name,marks[f"{name}:start"],marks.get(f"{name}:end",marks[f"{name}:start"]+5))
    if "close:start" in marks: row("close(grip)",marks["close:start"],marks["push:start"])
    t0=marks["push:start"]; t1=marks.get("push:end",t0+36)
    row("push settle 0-1.5s",t0,t0+1.5)
    # bin the turn by plan angle
    rb=np.degrees(np.unwrap(np.arctan2(ref[:,2]-axis[1],ref[:,1]-axis[0]))); rb-=np.interp(t0,ref[:,0],rb)
    ang=np.interp(t,ref[:,0],rb)
    rate=np.gradient(np.unwrap(np.interp(t,ref[:,0],np.unwrap(np.radians(rb)))),t)
    for a0 in range(0,360,45):
        m=(t>=t0)&(t<=t1)&(ang>=a0)&(ang<a0+45)
        if m.sum()<10: continue
        f=at(t[m],tc,con[:,1:4])
        print(f"  turn {a0:3d}-{a0+45:3d} deg rate {np.degrees(rate[m]).mean():5.1f}/s | e_x lon/lat/z mm {1e3*ex_b[m].mean(0)} | rad/tan {1e3*ex_rad[m].mean():6.1f}/{1e3*ex_tan[m].mean():6.1f}"
              f" | e_R mrad {eR[m].mean(0)*1e3} | d_t body {dt_b[m].mean(0)} | |F| {np.linalg.norm(f,axis=1).mean():.2f} | q2/q3 {np.degrees(q[m,1].mean()):5.1f}/{np.degrees(q[m,2].mean()):5.1f} | q1 {np.degrees(q[m,0].mean()):5.1f} | tilt-from-eR {np.degrees(np.hypot(eR[m,0],eR[m,1]).mean()):.2f}")
    # correlations over the whole turn
    m=(t>=t0+2)&(t<=t1-2)
    T=u1[m].mean()
    print(f"  corr(e_x lat, e_R roll(x))={np.corrcoef(ex_b[m,1],eR[m,0])[0,1]:.2f}; corr(e_x lon, e_R pitch(y))={np.corrcoef(ex_b[m,0],eR[m,1])[0,1]:.2f}")
    print(f"  T*e_R/k_x prediction (k_x 50): lat from roll {1e3*(T*eR[m,0]/50).mean():.1f} mm vs e_x lat {1e3*ex_b[m,1].mean():.1f}; lon from pitch {1e3*(T*eR[m,1]/50).mean():.1f} vs {1e3*ex_b[m,0].mean():.1f}")

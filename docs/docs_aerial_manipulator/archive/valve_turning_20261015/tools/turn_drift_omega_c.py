"""Exact-law reconstruction of omega_0c (the law's analytic chain, model acceleration e_ax)
and a 1-s table through the turn: e_R[0], the law's e_w, the fold, the yaw rate, d_r and
the deadbeat residual, so the standing pitch error can be attributed."""
import sys, numpy as np
R="docs/docs_aerial_manipulator/archive/valve_turning_20261015/runs/"
kx,kv,kR,kw,m,gg=50.03,12.58,1.6,1.2,3.746170,9.80665
def q2R(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
Rz=lambda a: np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
def vee(A): return np.array([A[2,1],A[0,2],A[1,0]])
def hat(v): return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
def at(tq,ts,X): return np.column_stack([np.interp(tq,ts,X[:,k]) for k in range(X.shape[1])])
def smooth(X,n=9):
    k=np.ones(n)/n; return np.column_stack([np.convolve(X[:,i],k,'same') for i in range(X.shape[1])])
np.set_printoptions(precision=1,suppress=True,linewidth=220)
e3=np.array([0,0,1.])
for tag in sys.argv[1:]:
    d=np.load(R+tag+".npz",allow_pickle=True)
    marks=dict(zip(d["marks_name"].tolist(),d["marks_t"].tolist()))
    dbg=d["dbg"]; od=d["odom"]; ref=d["ref"]; box=d["box"]
    t=dbg[:,0]; D=dbg[:,2:]
    t0=marks["push:start"]; t1=marks.get("push:end",t0+36)
    tg=np.arange(t0-2,t1+2,0.01); Dg=at(tg,t,D)
    ex=Dg[:,48:51]-Dg[:,45:48]; dt_=Dg[:,31:34]; u1=Dg[:,17]; eR_log=Dg[:,28:31]; q=Dg[:,5:9]; dr=Dg[:,34:37]; dc_r=Dg[:,65:68]
    xcd=at(tg,ref[:,0],ref[:,4:7]); xcdd=smooth(at(tg,ref[:,0],ref[:,13:16])); b1=at(tg,ref[:,0],ref[:,11:13]); b1=np.column_stack([b1,0*b1[:,0]])
    xcd_dot=smooth(np.gradient(xcd,tg,axis=0)); xcd_d3=smooth(np.gradient(xcdd,tg,axis=0)); b1_dot=smooth(np.gradient(b1,tg,axis=0))
    xc=Dg[:,48:51]; xc_dot=smooth(np.gradient(xc,tg,axis=0)); ev=xc_dot-xcd_dot
    Qa=at(tg,od[:,0],od[:,7:11]); n=len(tg)
    Rm=[q2R(Qa[i]/np.linalg.norm(Qa[i]))@Rz(-np.pi/2) for i in range(n)]
    om0=np.zeros((n,3))
    for i in range(1,n-1):
        om0[i]=vee(Rm[i].T@((Rm[i+1]-Rm[i-1])/0.02))
    om0=smooth(om0,5)
    ew=np.zeros((n,3)); eR_rec=np.zeros((n,3)); omc=np.zeros((n,3))
    for i in range(n):
        R_0=Rm[i]
        fd=-kx*ex[i]-kv*ev[i]+m*xcdd[i]+m*gg*e3-dt_[i]
        u1i=fd@(R_0@e3)
        xcdd_model=(u1i/m)*(R_0@e3)-gg*e3+dt_[i]/m
        eax=xcdd_model-xcdd[i]
        fd_dot=-kx*ev[i]-kv*eax+m*xcd_d3[i]
        nf=np.linalg.norm(fd); b3=fd/nf; P1=np.eye(3)-np.outer(b3,b3)
        s=P1@b1[i]; b3_dot=P1@fd_dot/nf; ns=np.linalg.norm(s); b1c=s/ns
        P1_dot=-(np.outer(b3_dot,b3)+np.outer(b3,b3_dot)); s_dot=P1_dot@b1[i]+P1@b1_dot[i]
        b1c_dot=(np.eye(3)-np.outer(b1c,b1c))@s_dot/ns
        b2=np.cross(b3,b1c); b2_dot=np.cross(b3_dot,b1c)+np.cross(b3,b1c_dot)
        R0c=np.column_stack([b1c,b2,b3]); R0c_dot=np.column_stack([b1c_dot,b2_dot,b3_dot])
        omc[i]=vee(R0c.T@R0c_dot)
        eR_rec[i]=0.5*vee(R0c.T@R_0-R_0.T@R0c)
        ew[i]=om0[i]-R_0.T@R0c@omc[i]
    axis=box[0,1:3]
    rb=np.unwrap(np.arctan2(ref[:,2]-axis[1],ref[:,1]-axis[0])); rate=np.interp(tg,ref[:,0],np.gradient(rb,ref[:,0]))
    sel=(tg>=t0+6)&(tg<=t1-6)
    print(f"\n== {tag}: fast part: e_R logged {1e3*eR_log[sel].mean(0)} rec {1e3*eR_rec[sel].mean(0)} mrad | e_w(law) {1e3*ew[sel].mean(0)} mrad/s -> predicted e_R {1e3*(-(kw/kR)*ew[sel].mean(0))} | k_R e_R + k_w e_w (x) = {1e3*(kR*eR_log[sel,0]+kw*ew[sel,0]).mean():.1f} mN.m | d_c-d_f (x) {1e3*(dc_r[sel,0]-dr[sel,0]).mean():.1f} mN.m")
    print("   t-t0 | rate deg/s | e_R0 mrad | e_w0 mrad/s | e_w2 | q2 q3 deg | d_r0 | d_c0 mN.m | e_x lon mm")
    Rb=[q2R(Qa[i]/np.linalg.norm(Qa[i])) for i in range(n)]
    exb=np.array([Rb[i].T@ex[i] for i in range(n)])
    for tt in np.arange(0,t1-t0,2.0):
        mm=(tg>=t0+tt)&(tg<t0+tt+2.0)
        print(f"   {tt:5.1f} | {np.degrees(rate[mm]).mean():6.1f} | {1e3*eR_log[mm,0].mean():6.1f} | {1e3*ew[mm,0].mean():6.1f} | {1e3*ew[mm,2].mean():5.1f} | {np.degrees(q[mm,1].mean()):5.1f} {np.degrees(q[mm,2].mean()):5.1f} | {1e3*dr[mm,0].mean():6.1f} | {1e3*dc_r[mm,0].mean():6.1f} | {1e3*exb[mm,0].mean():6.1f}")

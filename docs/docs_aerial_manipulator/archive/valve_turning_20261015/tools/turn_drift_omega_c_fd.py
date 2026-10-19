"""Reconstruct the law's desired frame R0c from the logged quantities and test the
09-26 mechanism on the valve turn: the law's omega_0c is built from f_d_dot WITHOUT the
derivative of the rotating translational estimate d_hat_t, so e_w carries a spurious
component and the attitude parks at e_R = -(k_w/k_R) e_w. Compare the prediction with
the logged e_R [28..30] during the turn."""
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
def at(tq,ts,X): return np.column_stack([np.interp(tq,ts,X[:,k]) for k in range(X.shape[1])])
def R0c_of(fd,b1):
    b3=fd/np.linalg.norm(fd); b1c=b1-b3*(b3@b1); b1c/=np.linalg.norm(b1c); b2=np.cross(b3,b1c)
    return np.column_stack([b1c,b2,b3])
np.set_printoptions(precision=1,suppress=True,linewidth=200)
for tag in sys.argv[1:]:
    d=np.load(R+tag+".npz",allow_pickle=True)
    marks=dict(zip(d["marks_name"].tolist(),d["marks_t"].tolist()))
    dbg=d["dbg"]; od=d["odom"]; ref=d["ref"]; bt=d["base_truth"]
    t=dbg[:,0]; D=dbg[:,2:]
    t0=marks["push:start"]; t1=marks.get("push:end",t0+36)
    m_=(t>=t0-3)&(t<=t1+3); t=t[m_]; D=D[m_]
    # resample everything on a uniform 100 Hz grid
    tg=np.arange(t[0],t[-1],0.01)
    Dg=at(tg,t,D)
    ex=Dg[:,48:51]-Dg[:,45:48]; dt_=Dg[:,31:34]; u1=Dg[:,17]; eR_log=Dg[:,28:31]
    xcd=at(tg,ref[:,0],ref[:,4:7]); xcdd=at(tg,ref[:,0],ref[:,13:16]); b1=at(tg,ref[:,0],ref[:,11:13]); b1=np.column_stack([b1,0*b1[:,0]])
    xcd_dot=np.gradient(xcd,tg,axis=0)
    # CoM velocity ~ law's x_c differentiated (law CoM from dbg 48..50), smooth a little
    xc=Dg[:,48:51]; xc_dot=np.gradient(xc,tg,axis=0)
    k=np.ones(9)/9; xc_dot=np.column_stack([np.convolve(xc_dot[:,i],k,'same') for i in range(3)])
    ev=xc_dot-xcd_dot
    Qa=at(tg,od[:,0],od[:,7:11])        # actual attitude (odom == truth here)
    e3=np.array([0,0,1.])
    n=len(tg); eR_rec=np.zeros((n,3)); om0=np.zeros((n,3)); omc_true=np.zeros((n,3)); omc_law=np.zeros((n,3)); R0cs=[]; fds=[]
    Rm=[q2R(Qa[i]/np.linalg.norm(Qa[i]))@Rz(-np.pi/2) for i in range(n)]   # MODEL-frame R_0
    for i in range(n):
        fd=-kx*ex[i]-kv*ev[i]+m*xcdd[i]+m*gg*e3-dt_[i]
        fds.append(fd); R0c=R0c_of(fd,b1[i]); R0cs.append(R0c)
        eR_rec[i]=0.5*vee(R0c.T@Rm[i]-Rm[i].T@R0c)
    h=0.01
    for i in range(1,n-1):
        # true rotation rate of the desired frame
        dR=(R0cs[i+1]-R0cs[i-1])/(2*h); omc_true[i]=vee(R0cs[i].T@dR)
        # the law's: same but with d_hat_t FROZEN at tick i (f_d_dot has no d(d_hat)/dt term)
        fdp=-kx*ex[i+1]-kv*ev[i+1]+m*xcdd[i+1]+m*gg*e3-dt_[i]
        fdm=-kx*ex[i-1]-kv*ev[i-1]+m*xcdd[i-1]+m*gg*e3-dt_[i]
        dRl=(R0c_of(fdp,b1[i+1])-R0c_of(fdm,b1[i-1]))/(2*h); omc_law[i]=vee(R0cs[i].T@dRl)
        # actual body rate (model frame) from the attitude derivative
        dRm=(Rm[i+1]-Rm[i-1])/(2*h); om0[i]=vee(Rm[i].T@dRm)
    ew_true=np.array([om0[i]-Rm[i].T@R0cs[i]@omc_true[i] for i in range(n)])
    ew_law =np.array([om0[i]-Rm[i].T@R0cs[i]@omc_law[i]  for i in range(n)])
    sel=(tg>=t0+6)&(tg<=t1-6)
    print(f"\n== {tag}  (fast part of the turn, {sel.sum()/100:.0f} s)")
    print(f"  e_R logged      mean {1e3*eR_log[sel].mean(0)} mrad")
    print(f"  e_R reconstructed mean {1e3*eR_rec[sel].mean(0)} mrad   (check of the reconstruction)")
    print(f"  omega_0 (body, model frame) mean {om0[sel].mean(0)} rad/s")
    print(f"  omega_0c true   mean {omc_true[sel].mean(0)} ; omega_0c law mean {omc_law[sel].mean(0)} ; diff {1e3*(omc_true[sel]-omc_law[sel]).mean(0)} mrad/s")
    print(f"  e_w true mean {1e3*ew_true[sel].mean(0)} mrad/s ; e_w law mean {1e3*ew_law[sel].mean(0)} mrad/s")
    print(f"  PREDICTED e_R = -(k_w/k_R) e_w_law = {1e3*(-(kw/kR)*ew_law[sel].mean(0))} mrad  vs logged {1e3*eR_log[sel].mean(0)}")
    print(f"  |d_hat_t horizontal| mean {np.linalg.norm(dt_[sel,:2],axis=1).mean():.2f} N; yaw rate mean {abs(om0[sel,2].mean()):.3f} rad/s -> |d|psi_dot/T = {1e3*np.linalg.norm(dt_[sel,:2],axis=1).mean()*abs(om0[sel,2].mean())/u1[sel].mean():.1f} mrad/s")

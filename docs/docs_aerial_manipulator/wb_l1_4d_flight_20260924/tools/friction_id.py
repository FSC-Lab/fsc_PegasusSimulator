"""Identify the gearbox friction the arm saw in flight: tau_applied - S*g(q,R0) vs measured joint velocity."""
import sys; sys.path.insert(0,"<scratch>"); sys.path.insert(0,"/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation")
from common import *
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as CT
P=TP.make_params_t650(); n=P["n"]
KT=np.array([162.4,154.0,150.5,153.4]); KPWM=np.array([169.47,149.70,135.25,148.51]); JS=[2,3,1,4]; idx=[JS.index(j) for j in (1,2,3,4)]
S_G=np.array([1.0,0.897,0.969,1.0]); FC=np.array([0.01711,0.03143,0.05751,0.05237]); MU=np.array([0,0.246,0.161,0]); IARM=0.0200
R_NED_ENU=np.array([[0,1,0],[1,0,0],[0,0,-1.0]]); R_FRD_FLU=np.diag([1,-1,-1.0]); RZm=np.array([[0,1,0],[-1,0,0],[0,0,1.0]])  # Rz(-90)
def R_of(q):
    w,x,y,z=q; return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
def gjoint(q,Rm):
    X=np.zeros(18+2*n); X[3:12]=Rm.reshape(9,order="F"); X[12:12+n]=q
    return CT.dynamics(X,P)["g"][6:6+n]
RUNS=[("a1",(35.9,75.5)),("a2",(22.94,50.94)),("a3",(20.62,48.62)),("a4",(20.09,48.08))]
out={}
for nm,(r0,r1) in RUNS:
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]
    tj=d["js__recv"]-t0; app=d["js__effort"][:,:4][:,idx]/KT; qj=d["js__position"][:,:4][:,idx]
    tv=d["velobs__recv"]-t0; vob=d["velobs__velocity"][:,:4]
    tq=d["vatt__recv"]-t0; Qv=d["vatt__q"]
    # 50 Hz grid over the circle phase
    tt=np.arange(r0,r1,0.02); q=np.column_stack([np.interp(tt,tj,qj[:,k]) for k in range(4)]); ap=np.column_stack([np.interp(tt,tj,app[:,k]) for k in range(4)])
    v=np.column_stack([np.interp(tt,tv,vob[:,k]) for k in range(4)]); acc=np.gradient(v,tt,axis=0)
    # low-pass applied torque 10 Hz (moving avg 0.1 s) to remove dither/noise
    k=5; apf=np.column_stack([np.convolve(ap[:,j],np.ones(k)/k,'same') for j in range(4)])
    g=np.zeros((len(tt),4))
    for i,ti in enumerate(tt):
        qq=Qv[np.searchsorted(tq,ti)-1]; R=R_NED_ENU@R_of(qq)@R_FRD_FLU; Rm=R@RZm; g[i]=gjoint(q[i],Rm)
    res=apf-S_G*g-IARM*acc   # what is left after gravity (scaled) and inertia = friction + coupling + model error
    out[nm]={}
    print(f"\n#### {nm} circle {r0:.1f}-{r1:.1f}: gravity torque g (model) mean j2/j3 {g[:,1].mean():+.3f}/{g[:,2].mean():+.3f} N.m; S*g {S_G[1]*g[:,1].mean():+.3f}/{S_G[2]*g[:,2].mean():+.3f}; applied(LP) mean {apf[:,1].mean():+.3f}/{apf[:,2].mean():+.3f}")
    for j in (1,2):
        r=res[:,j]; vv=v[:,j]; mv=np.abs(vv)>np.radians(1.0)
        pos=mv&(vv>0); neg=mv&(vv<0)
        cpos=r[pos].mean() if pos.any() else np.nan; cneg=r[neg].mean() if neg.any() else np.nan
        coul=(cpos-cneg)/2; bias=(cpos+cneg)/2
        # viscous: slope of r vs v within each sign
        A=np.column_stack([np.sign(vv[mv]),vv[mv],np.ones(mv.sum())]); c,_,_,_=np.linalg.lstsq(A,r[mv],rcond=None)
        ff=FC[j]+MU[j]*np.abs(apf[:,j]).mean()
        out[nm][j]=dict(coul=coul,bias=bias,fit_coul=c[0],fit_visc=c[1],fit_bias=c[2],ff=ff,frac_moving=mv.mean())
        print(f"  j{j+1}: residual mean when moving + {cpos:+.4f} / - {cneg:+.4f} N.m -> Coulomb-like {coul:+.4f}, bias {bias:+.4f} | lstsq: Coulomb {c[0]:+.4f}, viscous {c[1]:+.4f} N.m/(rad/s), bias {c[2]:+.4f} | yaml FF at this load fc+mu|tau| = {ff:.4f} (fc {FC[j]:.4f}, mu|tau| {MU[j]*np.abs(apf[:,j]).mean():.4f}) | moving {100*mv.mean():.0f}% | |v| mean {np.degrees(np.abs(vv[mv]).mean()):.1f} deg/s")
    # residual in stationary (hold) parts of DIRECT before the run for comparison
    th=(a+0.5, ex[0][0]-0.2); tt2=np.arange(th[0],th[1],0.02); q2=np.column_stack([np.interp(tt2,tj,qj[:,k]) for k in range(4)]); ap2=np.column_stack([np.interp(tt2,tj,app[:,k]) for k in range(4)])
    g2=np.array([gjoint(q2[i],R_NED_ENU@R_of(Qv[np.searchsorted(tq,ti)-1])@R_FRD_FLU@RZm) for i,ti in enumerate(tt2)])
    print(f"  hold {th[0]:.1f}-{th[1]:.1f}: applied - S*g mean j2/j3 {(ap2-S_G*g2)[:,1].mean():+.4f}/{(ap2-S_G*g2)[:,2].mean():+.4f} N.m (static: friction holds whatever is left)")
import json; json.dump({k:{str(j):v for j,v in d_.items()} for k,d_ in out.items()},open("friction_id.json","w"),indent=1)

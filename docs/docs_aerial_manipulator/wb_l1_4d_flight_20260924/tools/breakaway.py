import sys, json; sys.path.insert(0,"/tmp/claude-1000/-home-shiqi-fsc-PegasusSimulator/7dd2b67f-37e8-4043-a785-1a763d80f217/scratchpad"); sys.path.insert(0,"/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation")
from common import *
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as CT
P=TP.make_params_t650(); n=P["n"]; KT=np.array([162.4,154.0,150.5,153.4]); JS=[2,3,1,4]; idx=[JS.index(j) for j in (1,2,3,4)]; S_G=np.array([1.0,0.897,0.969,1.0])
R_NED_ENU=np.array([[0,1,0],[1,0,0],[0,0,-1.0]]); R_FRD_FLU=np.diag([1,-1,-1.0]); RZm=np.array([[0,1,0],[-1,0,0],[0,0,1.0]])
def R_of(q):
    w,x,y,z=q; return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
def gjoint(q,Rm):
    X=np.zeros(18+2*n); X[3:12]=Rm.reshape(9,order="F"); X[12:12+n]=q; return CT.dynamics(X,P)["g"][6:6+n]
RUNS=[("a1","F1",(35.9,75.5)),("a2","F2",(22.94,50.94)),("a3","F3",(20.62,48.62)),("a4","F4",(20.09,48.08))]
out={}
for nm,lab,(r0,r1) in RUNS:
    d,t0,a,b,ex,hold=load(nm); tj=d["js__recv"]-t0; app=d["js__effort"][:,:4][:,idx]/KT; qj=d["js__position"][:,:4][:,idx]; tv=d["velobs__recv"]-t0; vob=d["velobs__velocity"][:,:4]; tq=d["vatt__recv"]-t0; Qv=d["vatt__q"]
    tt=np.arange(r0,r1,0.02); q=np.column_stack([np.interp(tt,tj,qj[:,k]) for k in range(4)]); ap=np.column_stack([np.interp(tt,tj,app[:,k]) for k in range(4)]); v=np.degrees(np.column_stack([np.interp(tt,tv,vob[:,k]) for k in range(4)]))
    apf=np.column_stack([np.convolve(ap[:,j],np.ones(5)/5,'same') for j in range(4)])
    g=np.array([gjoint(q[i],R_NED_ENU@R_of(Qv[np.searchsorted(tq,ti)-1])@R_FRD_FLU@RZm) for i,ti in enumerate(tt)])
    res=apf-S_G*g; out[lab]={}
    for j in (1,2):
        still=np.abs(v[:,j])<1.0; onsets=[]
        i=15
        while i<len(tt)-5:
            if still[i-15:i].all() and (np.abs(v[i:i+5,j])>3.0).any():
                onsets.append(i); i+=25
            else: i+=1
        br=np.array([res[i-2,j]*np.sign(v[i+4,j]) for i in onsets])  # residual just before slipping, signed along the slip direction
        kin=res[np.abs(v[:,j])>5.0,j]*np.sign(v[np.abs(v[:,j])>5.0,j])
        out[lab][f"j{j+1}"]=dict(n_slips=len(onsets),breakaway_med=float(np.median(br)) if len(br) else None,breakaway_p75=float(np.percentile(br,75)) if len(br) else None,kinetic_med=float(np.median(kin)) if len(kin) else None)
        print(f"{lab} j{j+1}: slip onsets {len(onsets)}; torque beyond gravity just before slipping (along the slip): median {np.median(br) if len(br) else float('nan'):+.3f}, p25/p75 {np.percentile(br,25) if len(br) else float('nan'):+.3f}/{np.percentile(br,75) if len(br) else float('nan'):+.3f} N.m | kinetic (|v|>5 deg/s) median {np.median(kin) if len(kin) else float('nan'):+.3f} N.m (n={len(kin)})")
json.dump(out,open("breakaway.json","w"),indent=1)

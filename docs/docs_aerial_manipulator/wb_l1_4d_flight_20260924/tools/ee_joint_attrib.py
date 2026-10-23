import sys; sys.path.insert(0,"<scratch>")
sys.path.insert(0,"/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation")
from common import *
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
p=TP.make_params_t650()
def off(q):
    r0c,r0e,_=TP.arm_fk_model(q,p); return r0e-r0c
for nm,(r0,r1) in [("a1",(35.9,75.5)),("a2",(22.94,50.94)),("a3",(20.62,48.62)),("a4",(20.09,48.08))]:
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]; m=np.where((t>r0)&(t<r1))[0][::5]
    tm=d["mocap__hdr"]-t0; Q=np.column_stack([d[f"mocap__pose.orientation.{k}"] for k in "wxyz"]); yaw=np.unwrap(np.radians(eul(Q)[:,2]))
    pred=[]
    for i in m:
        ps=np.interp(t[i],tm,yaw)-np.pi/2; Rz=np.array([[np.cos(ps),-np.sin(ps),0],[np.sin(ps),np.cos(ps),0],[0,0,1]])
        pred.append(Rz@(off(D[i,5:9])-off(D[i,9:13])))
    pred=np.array(pred)*1000; ey=D[m,24:27]*1000
    print(f"{nm}: predicted EE-CoM offset change from (q - q_d), world, rms {np.sqrt((pred**2).mean(0)).round(1)} mm, max {np.abs(pred).max(0).round(1)}")
    print(f"     law EE task error e_y rms {np.sqrt((ey**2).mean(0)).round(1)} max {np.abs(ey).max(0).round(1)}; corr per axis {[round(np.corrcoef(pred[:,k],ey[:,k])[0,1],2) for k in range(3)]}; residual rms {np.sqrt(((ey-pred)**2).mean(0)).round(1)}")
    # sensitivity: EE-CoM offset per degree of split change at the start pose
    q0=np.radians([0,30,30,0]); dq=np.radians([0,1,-1,0]); print("     |d(offset)| per +1/-1 deg q2/q3 split at [0,30,30,0]:", np.round((off(q0+dq)-off(q0))*1000,2), "mm")

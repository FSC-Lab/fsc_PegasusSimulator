import sys, json; sys.path.insert(0,"<scratch>"); sys.path.insert(0,"/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation")
from common import *
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
P=TP.make_params_t650()
# effective joint stiffness of the task impedance at the circle's mean pose
for lab,q0 in [("home [0,40,40,0]",np.radians([0,40,40,0])),("F3/F4 centre [0,25,30,0]",np.radians([0,25,30,0]))]:
    J=np.zeros((3,4)); r0=TP.arm_fk_model(q0,P)[1]
    for k in range(4):
        dq=np.zeros(4); dq[k]=1e-6; J[:,k]=(TP.arm_fk_model(q0+dq,P)[1]-r0)/1e-6
    K=J.T@(20.0*np.eye(3))@J; Dm=J.T@(12.0*np.eye(3))@J
    print(f"{lab}: joint stiffness from K_y=20 N/m, diag(J^T K J) = {np.round(np.diag(K),3)} N.m/rad = {np.round(np.diag(K)*np.pi/180,4)} N.m/deg ; damping from D_y=12: {np.round(np.diag(Dm),3)} N.m.s/rad")
RUNS=[("a1","F1",(35.9,75.5)),("a2","F2",(22.94,50.94)),("a3","F3",(20.62,48.62)),("a4","F4",(20.09,48.08))]
out={}
for nm,lab,(r0,r1) in RUNS:
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]; m=(t>r0)&(t<r1); q=np.degrees(D[m,5:9]); qd=np.degrees(D[m,9:13]); tt=t[m]
    tv=d["velobs__recv"]-t0; vob=np.degrees(d["velobs__velocity"][:,:4]); v=np.column_stack([np.interp(tt,tv,vob[:,k]) for k in range(4)]); vd=np.gradient(qd,tt,axis=0)
    out[lab]={}
    for j in (1,2):
        refmov=np.abs(vd[:,j])>2.0; stuck=refmov&(np.abs(v[:,j])<1.0)
        pk_ref=np.abs(vd[:,j]).max(); pk=np.percentile(np.abs(v[:,j]),99)
        hi=(q[:,j]>=49.5).mean()*100; lo=(q[:,j]<=np.degrees(TP.Q_MIN[j])+0.5).mean()*100
        out[lab][f"j{j+1}"]=dict(stuck_pct=float(100*stuck.mean()/max(refmov.mean(),1e-9)),pk_ref=float(pk_ref),pk_meas_p99=float(pk),at_upper_stop_pct=float(hi),at_lower_stop_pct=float(lo),q_min=float(q[:,j].min()),q_max=float(q[:,j].max()))
        print(f"{lab} j{j+1}: ref peak |qdot| {pk_ref:.1f} deg/s, measured p99 |qdot| {pk:.1f} deg/s ({pk/pk_ref:.1f}x); stuck (|qdot|<1 while ref moves >2 deg/s) {100*stuck.mean()/max(refmov.mean(),1e-9):.0f}% of moving-reference time; q range {q[:,j].min():.1f}..{q[:,j].max():.1f} (limit {np.degrees(TP.Q_MIN[j]):.0f}..{np.degrees(TP.Q_MAX[j]):.0f}); at upper stop {hi:.1f}% of run")
json.dump(out,open("burst_rows.json","w"),indent=1)

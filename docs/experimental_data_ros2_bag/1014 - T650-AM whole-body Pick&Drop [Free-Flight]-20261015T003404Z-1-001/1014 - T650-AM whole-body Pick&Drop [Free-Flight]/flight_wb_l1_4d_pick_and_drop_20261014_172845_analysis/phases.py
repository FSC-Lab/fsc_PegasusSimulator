import numpy as np
np.set_printoptions(precision=3, suppress=True, linewidth=220)
d=np.load('hi.npz')
W=d['uav_0__fsc_autopilot_ros2__whole_body_direct_actuation__wb_control_debug']; wt=d['uav_0__fsc_autopilot_ros2__whole_body_direct_actuation__wb_control_debug__t']
P=d['vrpn_mocap__uav_0__pose']; pt=d['vrpn_mocap__uav_0__pose__t']
EE=d['uav_0__whole_body_planner__current_ee']; et=d['uav_0__whole_body_planner__current_ee__t']
def interp(t, ts, v): return np.stack([np.interp(t, ts, v[:,i]) for i in range(v.shape[1])],1)
pos = interp(wt, pt, P[:,:3])
# attitude from vrpn quaternion
def rpy(q):
    x,y,z,w=q.T
    roll=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)); pitch=np.arcsin(np.clip(2*(w*y-z*x),-1,1)); yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return np.degrees(np.stack([roll,pitch,yaw],1))
att = interp(wt, pt, rpy(P[:,3:7]))
phases = [
 ('hold@start(1.0m)', 25,30.4),
 ('go_to_start',30.5,33.5),
 ('hold start',33.5,36.4),
 ('pick approach',36.5,45.7),
 ('pick hover/desc',45.8,50.3),
 ('exit_pick',50.3,53.5),
 ('go_to_place_start',54.2,70.4),
 ('hold @ place start z0.70',70.5,76.3),
 ('drop leg (execute_place)',76.4,89.9),
 ('hold @ drop pt z1.35',89.9,92.27),
 ('abort climb',92.3,102.5),
 ('hold abort z1.65',102.6,115.5),
 ('wait',115.5,118.8),
 ('transit to (0,0,1.68)',118.9,128.6),
 ('hold (0,0,1.68)',128.6,131.7),
 ('descend to land',131.7,139.3),
]
m = W[:,0]==1
print(f"{'phase':28s} {'z':>6s} {'u1':>6s} {'u1sd':>5s} | {'dF_x':>6s} {'dF_y':>6s} {'dF_z':>6s} {'dFz_sd':>6s} | {'dTx':>6s} {'dTy':>6s} {'dTz':>6s} | {'|e_xy|mm':>8s} {'e_z mm':>7s} {'ezsd':>5s} | {'roll':>5s} {'pitch':>5s} {'r_sd':>5s} {'p_sd':>5s} | motors")
for name,a,b in phases:
    s=m&(wt>=a)&(wt<b)
    if not s.any(): continue
    w=W[s]
    exy=np.hypot(w[:,24],w[:,25])*1000
    print(f"{name:28s} {pos[s,2].mean():6.3f} {w[:,17].mean():6.2f} {w[:,17].std():5.2f} | {w[:,31].mean():6.2f} {w[:,32].mean():6.2f} {w[:,33].mean():6.2f} {w[:,33].std():6.2f} | {w[:,34].mean():6.3f} {w[:,35].mean():6.3f} {w[:,36].mean():6.3f} | {np.sqrt((exy**2).mean()):8.1f} {w[:,26].mean()*1000:7.1f} {w[:,26].std()*1000:5.1f} | {att[s,0].mean():5.2f} {att[s,1].mean():5.2f} {att[s,0].std():5.2f} {att[s,1].std():5.2f} | {w[:,41:45].mean(0)}")

import sys, pickle, numpy as np, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
out=sys.argv[1]
S1,S2='#2a78d6','#eb6834'; INK='#0b0b0b'; INK2='#52514e'; GRID='#e4e3df'; SURF='#fcfcfb'; SHADE='#f0efea'; BOX='#fbe3d6'
plt.rcParams.update({'font.size':9,'axes.edgecolor':INK2,'axes.labelcolor':INK2,'xtick.color':INK2,'ytick.color':INK2,
  'axes.facecolor':SURF,'figure.facecolor':SURF,'axes.grid':True,'grid.color':GRID,'grid.linewidth':0.6,'axes.spines.top':False,'axes.spines.right':False,'lines.linewidth':1.6})
d=np.load('hi.npz')
W=d['uav_0__fsc_autopilot_ros2__whole_body_direct_actuation__wb_control_debug']; wt=d['uav_0__fsc_autopilot_ros2__whole_body_direct_actuation__wb_control_debug__t']
m=W[:,0]==1; W=W[m]; wt=wt[m]
P=d['vrpn_mocap__uav_0__pose']; pt=d['vrpn_mocap__uav_0__pose__t']
x,y,z,w=P[:,3:7].T
roll=np.degrees(np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y))); pitch=np.degrees(np.arcsin(np.clip(2*(w*y-z*x),-1,1)))
A=pickle.load(open('arr.pkl','rb'))['/uav_0/whole_body_planner/pick_place/arrival_error']
T0=1792013326048378036
at=np.array([(ts-T0)/1e9 for ts,_ in A]); ae=np.array([mm.data[2] for _,mm in A])
t0,t1=66,116
def lp(v,n=25): return np.convolve(v,np.ones(n)/n,mode='same')
s=(wt>t0)&(wt<t1); sp=(pt>t0)&(pt<t1); sa=(at>85.5)&(at<92.3)
fig,ax=plt.subplots(5,1,figsize=(10,10.5),sharex=True,gridspec_kw={'height_ratios':[1,1.1,1.1,1,1]})
spans=[(70.4,76.3,'hold at\nDrop Start'),(76.4,89.9,'drop leg'),(89.9,92.27,'hold on\ndrop point'),(92.27,102.5,'abort:\nclimb +0.30 m'),(102.5,115.5,'hold 0.30 m higher,\nsame x-y')]
for a in ax:
    a.axvspan(85.5,92.27,color=BOX,lw=0,zorder=0)
    for b in [70.4,76.4,89.9,92.27,102.5]: a.axvline(b,color=INK2,lw=0.6,ls=':',zorder=1)
for a0,a1,lab in spans: ax[0].text((a0+a1)/2,1.02,lab,transform=ax[0].get_xaxis_transform(),ha='center',va='bottom',color=INK2,fontsize=8)
ax[0].plot(wt[s],W[s,50],color=S1); ax[0].set_ylabel('CoM height z [m]')
ax[0].text(86,0.80,'front rotors\nover the box',color='#a6461c',fontsize=8)
ex=(W[s,48]-W[s,45])*1000; ey=(W[s,49]-W[s,46])*1000
ax[1].plot(wt[s],ex,color=S1,label='x (fore-aft, toward box)'); ax[1].plot(wt[s],ey,color=S2,label='y (lateral)')
ax[1].set_ylabel('CoM position\nerror [mm]'); ax[1].legend(loc='upper left',frameon=False,fontsize=8,labelcolor=INK)
ax[2].plot(pt[sp],pitch[sp],color=S1,label='pitch'); ax[2].plot(pt[sp],roll[sp],color=S2,label='roll')
ax[2].set_ylabel('attitude [deg]'); ax[2].legend(loc='upper left',frameon=False,fontsize=8,labelcolor=INK)
ax[3].plot(wt[s],lp(W[s,22]),color=S1); ax[3].set_ylabel('commanded pitch\ntorque [N·m]')
ax[4].plot(at[sa],ae[sa]*1000,color=S1); ax[4].axhline(50,color=INK2,lw=1,ls='--'); ax[4].text(t1-0.5,53,'50 mm claw tolerance',ha='right',color=INK2,fontsize=8)
ax[4].set_ylabel('claw error to\ntarget [mm]'); ax[4].set_ylim(0,160); ax[4].set_xlabel('time since bag start [s]')
ax[4].text(92.5,120,'gripper opens\n(abort)',color=INK2,fontsize=8)
ax[-1].set_xlim(t0,t1)
fig.suptitle('Pick & Drop free flight 2026-10-14 17:28 — over-box hold vs. 0.30 m higher',x=0.01,ha='left',color=INK,fontsize=11)
fig.tight_layout(); fig.savefig(out+'/drop_timeseries.png',dpi=150)

# bar chart of fore-aft position std per hover
wins=[('start\n(open)\n1.01 m',33.6,36.3),('pick\nhover\n1.15 m',46.0,49.2),('Drop\nStart\n0.70 m',70.5,76.2),('over box\narriving\n1.35 m',86.0,89.88),('over box\nhold\n1.35 m',89.88,92.27),('over box\n(abort)\n1.65 m',103,115),('open\nhover\n1.68 m',128.6,131.7)]
vals=[]; tvals=[]
for _,a,b in wins:
    q=(wt>=a)&(wt<b); vals.append(W[q,48].std()*1000)
    qp=(pt>=a)&(pt<b); tvals.append(pitch[qp].std())
fig,ax=plt.subplots(1,2,figsize=(11,4.2))
cols=[S2 if 'over box' in n and '1.35' in n else S1 for n,_,_ in wins]
for k,(v,lab) in enumerate([(vals,'fore-aft (x) position std [mm]'),(tvals,'pitch std [deg]')]):
    a=ax[k]; bars=a.bar(range(len(wins)),v,color=cols,width=0.62,edgecolor=SURF,linewidth=2)
    for i,vv in enumerate(v): a.text(i,vv,f'{vv:.0f}' if k==0 else f'{vv:.1f}',ha='center',va='bottom',color=INK,fontsize=8)
    a.set_xticks(range(len(wins))); a.set_xticklabels([n for n,_,_ in wins],fontsize=7); a.set_title(lab,loc='left',color=INK,fontsize=10); a.grid(axis='x',visible=False)
fig.text(0.01,0.01,'orange = front rotors over the box at the release altitude (base 1.35 m)',color=INK2,fontsize=8)
fig.tight_layout(rect=(0,0.04,1,1)); fig.savefig(out+'/hover_comparison.png',dpi=150)
print('ok')

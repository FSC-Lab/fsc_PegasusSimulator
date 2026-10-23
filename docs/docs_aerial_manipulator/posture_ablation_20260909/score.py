#!/usr/bin/env python3
"""Score the posture-ablation runs: entry transient, arm excursion, steady state.
    /usr/bin/python3 score.py *.npz
"""
import sys, numpy as np
def score(path):
    z=np.load(path, allow_pickle=True)
    dbg=z['dbg']; log=z['log']; t0=float(z['t_direct'])
    t=dbg[:,0]-t0; d=dbg[:,1:]; tl=log[:,0]-t0
    ab=bool(z['aborted'])
    tend = (tl[log[:,17]>0.5].max() if ab else 80.0)
    sel=(t>0)&(t<=tend); ql=(tl>0)&(tl<=tend)
    ex=np.linalg.norm(d[sel,48:51]-d[sel,45:48],axis=1); ey=np.linalg.norm(d[sel,24:27],axis=1)
    q=np.degrees(log[ql,13:17])[:, [2,0,1,3]]   # broadcaster order [q2,q3,q1,q4] -> q1..q4
    last=(t>tend-20)&(t<=tend)
    r=dict(file=path.split('/')[-1].replace('.npz',''), aborted=ab, direct_s=float(tend),
           ex_peak=ex.max()*1e3, ey_peak=ey.max()*1e3, tilt_peak=float(log[ql,7].max()),
           clamp=100*np.mean(d[sel,51]>0), tau_peak=float(np.abs(d[sel,13:17]).max()),
           ex_last=np.linalg.norm(d[last,48:51]-d[last,45:48],axis=1).mean()*1e3 if not ab else np.nan,
           ey_last=np.linalg.norm(d[last,24:27],axis=1).mean()*1e3 if not ab else np.nan,
           dz_last=d[last,33].mean() if not ab else np.nan,
           q_min=q.min(axis=0), q_max=q.max(axis=0), q_end=q[-1])
    return r
rows=[score(p) for p in sys.argv[1:]]
print(f"{'run':16s} {'result':10s} {'peak e_x':>8s} {'peak e_y':>8s} {'tilt':>5s} {'clamp':>6s} {'tau':>5s} {'e_x set':>7s} {'e_y set':>7s} {'dhat_z':>6s}  q min / max / end [deg]")
for r in rows:
    res = f"ABORT {r['direct_s']:.1f}s" if r['aborted'] else "completed"
    print(f"{r['file']:16s} {res:10s} {r['ex_peak']:8.0f} {r['ey_peak']:8.0f} {r['tilt_peak']:5.1f} {r['clamp']:5.1f}% {r['tau_peak']:5.2f} {r['ex_last']:7.1f} {r['ey_last']:7.1f} {r['dz_last']:6.2f}  {np.round(r['q_min']).astype(int)} {np.round(r['q_max']).astype(int)} {np.round(r['q_end']).astype(int)}")

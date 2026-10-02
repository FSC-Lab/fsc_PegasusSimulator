"""Commanded vs applied arm torque in the circle phase, and the sign of the friction feed-forward vs actual motion."""
import sys; sys.path.insert(0,"<scratch>")
from common import *
KT=np.array([162.4,154.0,150.5,153.4]); KPWM=np.array([169.47,149.70,135.25,148.51]); MAXE=np.array([57.6117,364.8743,192.0391,57.6117])
JS=[2,3,1,4]; idx=[JS.index(j) for j in (1,2,3,4)]
RUNS=[("a1",(35.9,75.5)),("a2",(22.94,50.94)),("a3",(20.62,48.62)),("a4",(20.09,48.08))]
for nm,(r0,r1) in RUNS:
    d,t0,a,b,ex,hold=load(nm); t=d["wb__recv"]-t0; D=d["wb__data"]
    tl=d["law__recv"]-t0; L=d["law__data"]; ml=(tl>r0)&(tl<r1); L=L[ml]; tl=tl[ml]
    tj=d["js__recv"]-t0; app=d["js__effort"][:,:4][:,idx]/KT; jsv=d["js__velocity"][:,:4][:,idx]
    tc=d["tcmd__recv"]-t0; cmd=d["tcmd__effort"][:,:4]
    cmd_l=np.column_stack([np.interp(tl,tc,cmd[:,k]) for k in range(4)]); app_l=np.column_stack([np.interp(tl,tj,app[:,k]) for k in range(4)])
    acc=L[:,9:13]; duty=L[:,13:17]/KPWM; aux=L[:,18:22]/KPWM; gc=L[:,22:26]/KPWM
    tv=d["velobs__recv"]-t0 if "velobs__recv" in d else tj; vob=d["velobs__velocity"][:,:4] if "velobs__velocity" in d else jsv
    v_l=np.column_stack([np.interp(tl,tv,vob[:,k]) for k in range(4)])
    qd=np.column_stack([np.interp(tl,t,D[:,9+k]) for k in range(4)]); qd_dot=np.gradient(qd,tl,axis=0)
    print(f"\n#### {nm} circle {r0:.1f}-{r1:.1f}: law[0] passthrough {L[:,0].mean():.2f}, [1] fresh {L[:,1].mean():.2f}, [26] aux {L[:,26].mean():.0f}, [27] grav-corr flag {L[:,27].mean():.0f}")
    print("  joint | cmd(node) mean/rms N.m | accepted-cmd rms | duty(after corr) mean | aux(fric+dither) mean/rms | gravcorr mean | applied mean | applied-cmd mean/rms | applied-duty mean/rms | corr(app,duty) | duty clamp %")
    for j in range(4):
        e1=app_l[:,j]-cmd_l[:,j]; e2=app_l[:,j]-duty[:,j]; c=np.corrcoef(app_l[:,j],duty[:,j])[0,1]
        print(f"  j{j+1}   | {cmd_l[:,j].mean():+.3f}/{np.sqrt((cmd_l[:,j]**2).mean()):.3f} | {np.sqrt(((acc[:,j]-cmd_l[:,j])**2).mean()):.4f} | {duty[:,j].mean():+.3f} | {aux[:,j].mean():+.3f}/{np.sqrt((aux[:,j]**2).mean()):.3f} | {gc[:,j].mean():+.3f} | {app_l[:,j].mean():+.3f} | {e1.mean():+.3f}/{np.sqrt((e1**2).mean()):.3f} | {e2.mean():+.3f}/{np.sqrt((e2**2).mean()):.3f} | {c:.3f} | {100*np.mean(np.abs(L[:,13+j])>=MAXE[j]-0.5):.1f}")
    # friction FF direction vs actual motion (j2, j3): FF is driven by the REFERENCE velocity
    for j in (1,2):
        s_ref=np.sign(qd_dot[:,j]); s_act=np.sign(v_l[:,j]); mv=(np.abs(qd_dot[:,j])>0.005)|(np.abs(v_l[:,j])>0.005)
        wrong=np.mean((s_ref*s_act<0)&mv)
        # friction FF component only (aux minus dither): low-pass aux at 20 Hz? dither is 100 Hz -> average over 10 samples
        k=10; auxf=np.convolve(aux[:,j],np.ones(k)/k,'same')
        print(f"  j{j+1}: |qd_ref| mean {np.degrees(np.abs(qd_dot[:,j]).mean()):.2f} deg/s, |q_dot meas| mean {np.degrees(np.abs(v_l[:,j]).mean()):.2f}; ref and actual velocity OPPOSITE sign {100*wrong:.0f}% of moving time; friction FF (aux, dither averaged) rms {np.sqrt((auxf**2).mean()):.3f} N.m, |max| {np.abs(auxf).max():.3f}; FF against actual motion (sign(auxf)!=sign(v)) {100*np.mean((np.sign(auxf)*s_act<0)&(np.abs(v_l[:,j])>0.005)):.0f}% of moving time")

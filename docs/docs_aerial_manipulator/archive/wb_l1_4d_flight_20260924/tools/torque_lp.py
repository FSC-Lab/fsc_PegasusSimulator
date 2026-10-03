"""Commanded vs applied arm torque with the 100 Hz dither and current noise removed (10 Hz low-pass), circle phase."""
import sys, json; sys.path.insert(0,"<scratch>")
from common import *
KT=np.array([162.4,154.0,150.5,153.4]); KPWM=np.array([169.47,149.70,135.25,148.51]); JS=[2,3,1,4]; idx=[JS.index(j) for j in (1,2,3,4)]
RUNS=[("a1","F1",(35.9,75.5)),("a2","F2",(22.94,50.94)),("a3","F3",(20.62,48.62)),("a4","F4",(20.09,48.08))]
def lp(x,k=25): return np.convolve(x,np.ones(k)/k,'same')
rows={}
for nm,lab,(r0,r1) in RUNS:
    d,t0,a,b,ex,hold=load(nm)
    tl=d["law__recv"]-t0; L=d["law__data"]; ml=(tl>r0)&(tl<r1); L=L[ml]; tl=tl[ml]
    tj=d["js__recv"]-t0; app=d["js__effort"][:,:4][:,idx]/KT; tc=d["tcmd__recv"]-t0; cmd=d["tcmd__effort"][:,:4]
    cmd_l=np.column_stack([np.interp(tl,tc,cmd[:,k]) for k in range(4)]); app_l=np.column_stack([np.interp(tl,tj,app[:,k]) for k in range(4)])
    duty=L[:,13:17]/KPWM; aux=L[:,18:22]/KPWM; gc=L[:,22:26]/KPWM
    fric=duty-cmd_l-gc  # aux incl dither, reconstructed = duty - cmd - gravcorr (they add up by construction; check)
    rows[lab]={}
    print(f"\n#### {lab} [{nm}] circle: check duty == cmd+aux+gc: rms {np.sqrt(((duty-(cmd_l+aux+gc))**2).mean(0)).round(4)}")
    print("  joint | cmd rms | after 10 Hz LP: applied-cmd mean/rms | applied-(cmd+gravcorr) mean/rms | applied-INTENDED(cmd+gc+fricFF) mean/rms | corr(app,intended) | raw applied-duty rms (dither+noise) | back-EMF/trim duty-equivalent rms | dither amp")
    for j in range(4):
        c=lp(cmd_l[:,j]); ap=lp(app_l[:,j]); du=lp(duty[:,j]); cg=lp(cmd_l[:,j]+gc[:,j])
        it=lp(cmd_l[:,j]+gc[:,j]+aux[:,j]); e1=ap-c; e2=ap-cg; e3=ap-it; raw=app_l[:,j]-duty[:,j]; bemf=lp(duty[:,j]-(cmd_l[:,j]+gc[:,j]+aux[:,j]))
        rows[lab][f"j{j+1}"]=dict(cmd_rms=float(np.sqrt((c**2).mean())),app_cmd_mean=float(e1.mean()),app_cmd_rms=float(np.sqrt((e1**2).mean())),app_cg_mean=float(e2.mean()),app_cg_rms=float(np.sqrt((e2**2).mean())),app_duty_mean=float(e3.mean()),app_duty_rms=float(np.sqrt((e3**2).mean())),corr=float(np.corrcoef(ap,it)[0,1]),raw_rms=float(np.sqrt((raw**2).mean())),bemf_rms=float(np.sqrt((bemf**2).mean())),app_rms=float(np.sqrt((ap**2).mean())))
        r=rows[lab][f"j{j+1}"]
        print(f"  j{j+1}   | {r['cmd_rms']:.3f} | {r['app_cmd_mean']:+.3f}/{r['app_cmd_rms']:.3f} | {r['app_cg_mean']:+.3f}/{r['app_cg_rms']:.3f} | {r['app_duty_mean']:+.3f}/{r['app_duty_rms']:.3f} | {r['corr']:.3f} | {r['raw_rms']:.3f} | {r['bemf_rms']:.3f} | {[0,48.0098/149.70,19.2039/135.25,14.4029/148.51][j]:.3f}")
json.dump(rows,open("torque_rows.json","w"),indent=1)

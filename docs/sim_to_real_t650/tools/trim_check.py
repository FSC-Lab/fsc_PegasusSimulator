import numpy as np, sys
sys.path.insert(0, sys.argv[1] if len(sys.argv)>1 else ".")
from steps import euler_deg, horizontal_accel_ned, ref_onsets, detect_steps
from plot_steps import load_side

t_ref, ref, _ = ref_onsets("exp_A.npz", "ref_A.npz")
steps = detect_steps(t_ref, ref)
R = load_side("exp_A.npz", "real"); S_ = load_side("sim_stack_A.npz", "sim")

# hover windows: last 2.5 s before each step onset (vehicle settled at previous waypoint)
def trim(side, label):
    rolls, pitches, an, ae, thr = [], [], [], [], []
    for st in steps:
        m = (side["t_att"] >= st["t"]-2.5) & (side["t_att"] < st["t"]-0.2)
        if m.sum() < 10: continue
        e = side["att_eul"][m]; a = side["att_ah"][m]
        rolls.append(e[:,0].mean()); pitches.append(e[:,1].mean())
        an.append(a[:,0].mean()); ae.append(a[:,1].mean())
        mt = (side["t_sp"] >= st["t"]-2.5) & (side["t_sp"] < st["t"]-0.2)
        if mt.sum(): thr.append(side["sp_thr"][mt].mean())
    print(f"{label:5s} pre-step hover (n={len(rolls)} windows):")
    print(f"   roll  {np.mean(rolls):+7.3f} +- {np.std(rolls):.3f} deg    "
          f"pitch {np.mean(pitches):+7.3f} +- {np.std(pitches):.3f} deg")
    print(f"   a_north {np.mean(an):+7.3f}  a_east {np.mean(ae):+7.3f} m/s^2   "
          f"|a_h| {np.hypot(np.mean(an),np.mean(ae)):.3f}")
    print(f"   thrust cmd {np.mean(thr):.4f} +- {np.std(thr):.4f}")
    return np.array([np.mean(an), np.mean(ae)])

tr = trim(R, "real"); ts = trim(S_, "sim")
print(f"\n   trim difference (sim-real): {ts-tr}  |.| {np.linalg.norm(ts-tr):.3f} m/s^2")
print(f"   angle between trim vectors: "
      f"{np.degrees(np.arccos(np.dot(tr,ts)/np.linalg.norm(tr)/np.linalg.norm(ts))):.1f} deg")

# thrust over the whole run (battery sag visible on real only)
for side,label in ((R,"real"),(S_,"sim")):
    t, y = side["t_sp"], side["sp_thr"]
    m = (t>10)&(t<105)
    tt, yy = t[m], y[m]
    A = np.stack([tt, np.ones_like(tt)],1)
    sl = np.linalg.lstsq(A, yy, rcond=None)[0][0]
    print(f"{label:5s} thrust cmd 10..105s: mean {yy.mean():.4f}  drift {sl*95*100:+.2f} %-pts over 95 s")

#!/usr/bin/env python3
import numpy as np, sys
np.set_printoptions(precision=4, suppress=True, linewidth=160)
d = np.load(sys.argv[1], allow_pickle=True)
t0 = d["wb__recv"][0]
OFF = float(np.median(d["js__recv"] - d["js__hdr"]))
def T(key, hdr=True):
    if hdr and f"{key}__hdr" in d: return d[f"{key}__hdr"] + OFF - t0
    return d[f"{key}__recv"] - t0
tw = T("wb", hdr=False); W = d["wb__data"]; D = W[:,0] > 0.5; td, wd = tw[D], W[D]
to = d["odom__recv"] - t0
pos = np.column_stack([d[f"odom__pose.pose.position.{k}"] for k in "xyz"])
vel = np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"])
qw,qx,qy,qz = [d[f"odom__pose.pose.orientation.{k}"] for k in "wxyz"]
tm = d["mocap__recv"] - t0
print("=== A. the |e_R|~1 spike and the mocap dropout ===")
eRn = np.linalg.norm(wd[:,28:31],axis=1)
i = np.argmax(eRn); print(f"max |e_R| {eRn[i]:.4f} at t={td[i]:.3f}; samples with |e_R|>0.3: {np.sum(eRn>0.3)} ({np.sum(eRn>0.3)*4} ms); >0.25: {np.sum(eRn>0.25)}")
m = (td>td[i]-0.2)&(td<td[i]+0.3)
print("t, eR(3), x_c(3), x_cd(3), u1, tau_body(3), motors(4), e_y(4) around the spike:")
for k in np.where(m)[0][::4]:
    print(f"  {td[k]:8.3f} eR={wd[k,28:31]} xc={wd[k,48:51]} xcd={wd[k,45:48]} u1={wd[k,17]:.2f} tb={wd[k,21:24]} mot={wd[k,41:45]} ey={wd[k,24:28]}")
print("\nodometry gaps > 25 ms (t, gap ms):")
g = np.diff(to); 
for k in np.where(g>0.025)[0]: print(f"  {to[k]:8.3f} -> {to[k+1]:8.3f}  gap {g[k]*1e3:.0f} ms")
g2 = np.diff(tm); print("mocap topic gaps > 25 ms:"); 
for k in np.where(g2>0.025)[0]: print(f"  {tm[k]:8.3f} -> {tm[k+1]:8.3f}  gap {g2[k]*1e3:.0f} ms")
# odom sample-to-sample position jump
dp = np.linalg.norm(np.diff(pos,axis=0),axis=1)
print("largest odom position jumps (t, mm):", [(round(to[k],3), round(dp[k]*1e3,1)) for k in np.argsort(dp)[-6:]])
# is pose repeated (stale) around the dropout?
m = (to>44)&(to<47.5)
rep = np.sum(np.all(np.diff(pos[m],axis=0)==0,axis=1)); print(f"odom samples with identical consecutive position in 44-47.5 s: {rep} of {m.sum()}")
# attitude around the spike from odom
R22 = 1-2*(qx**2+qy**2); tilt = np.degrees(np.arccos(np.clip(R22,-1,1)))
m = (to>td[i]-0.3)&(to<td[i]+0.3); print("odom tilt around spike:", tilt[m].round(2))
# wb x_c around the dropout window
m = (td>44.4)&(td<47.2)
print("x_c during 44.4-47.2: min", wd[m,48:51].min(axis=0), "max", wd[m,48:51].max(axis=0), " ; e_R max", eRn[m].max())
print("\n=== B. yaw trim / body torque means ===")
tb = wd[:,21:24]; dh = wd[:,31:41]
for n,(a,b) in [("hover 20-27.9",(20,27.9)),("hold 60-69",(60,69)),("hold 76-84",(76,84)),("DIRECT all",(td[0],td[-1]))]:
    m=(td>=a)&(td<=b); print(f"  {n:14s} tau_body mean {tb[m].mean(axis=0)} | d_r mean {dh[m,3:6].mean(axis=0)} | motors mean {wd[m,41:45].mean(axis=0)}  CCW(M1,M2)-CW(M3,M4) = {wd[m,41:43].mean()-wd[m,43:45].mean():+.4f}")
print("\n=== C. battery ===")
tbat = d["batt__recv"]-t0; V = d["batt__voltage_v"]; I = d["batt__current_a"]
for a,b in [(0,9),(10,20),(40,50),(80,92),(95,104)]:
    m=(tbat>=a)&(tbat<=b); print(f"  {a:3d}-{b:3d} s  V {V[m].mean():.2f}  I {I[m].mean():.1f} A")
print("\n=== D. arm torque execution ===")
tj = T("js"); names = list(d["js__name"][0]); print("  js names", names)
order = [names.index(f"joint{k}") for k in (1,2,3,4)]
eff = d["js__effort"][:,order]; jsq = np.degrees(d["js__position"][:,order]); jsv = d["js__velocity"][:,order]
tc = T("tcmd"); tcmd = d["tcmd__effort"]; print("  tcmd names", list(d["tcmd__name"][0]))
tl = T("law"); law = d["law__data"]
KT = np.array([162.4,154.0,150.5,153.4]); KPWM = np.array([169.47,149.70,135.25,148.51]); MAXE = np.array([57.6117,364.8743,192.0391,57.6117])
print("  effort column ranges:", eff.min(axis=0), eff.max(axis=0))
tau_app = eff / KT
# the arm's own view: law [9..12] N.m accepted, [13..16] duty counts written
m_l = (tl>=td[0]+0.5)&(tl<=td[-1]-0.5)
tau_ext = law[m_l,9:13]; duty = law[m_l,13:17]; aux = law[m_l,18:22]; gcorr = law[m_l,22:26]
print("  law[0] passthrough mean in DIRECT:", law[m_l,0].mean(), " [1] fresh:", law[m_l,1].mean(), " [3] aux:", law[m_l,3].mean(), " [26]:", law[m_l,26].mean(), " [27]:", law[m_l,27].mean())
print("  duty clamp fraction per joint (|duty|>=max_effort-0.5):", (np.abs(duty) >= MAXE-0.5).mean(axis=0), "  max |duty|/max_effort:", (np.abs(duty).max(axis=0)/MAXE).round(3))
print("  aux term counts rms:", np.sqrt((aux**2).mean(axis=0)).round(2), " mean:", aux.mean(axis=0).round(2), " | gravity corr counts mean:", gcorr.mean(axis=0).round(2))
print("  commanded tau_ext N.m: mean", tau_ext.mean(axis=0), " max|.|", np.abs(tau_ext).max(axis=0), " (law clamp 3.0; arm-side clamp N.m =", (MAXE/KPWM).round(3), ")")
# nominal duty-implied torque vs applied (measured current * Kt)
tau_duty = duty / KPWM
app_l = np.column_stack([np.interp(tl[m_l], tj, tau_app[:,j]) for j in range(4)])
vel_l = np.column_stack([np.interp(tl[m_l], tj, jsv[:,j]) for j in range(4)])
ext_l = tau_ext
def report(mask, label):
    e1 = app_l[mask]-ext_l[mask]; e2 = app_l[mask]-tau_duty[mask]
    c = [np.corrcoef(app_l[mask,j], ext_l[mask,j])[0,1] if ext_l[mask,j].std()>1e-6 else float('nan') for j in range(4)]
    print(f"  {label:26s} applied-vs-streamed: mean {e1.mean(axis=0).round(4)} rms {np.sqrt((e1**2).mean(axis=0)).round(4)} | applied-vs-duty(incl. corrections): mean {e2.mean(axis=0).round(4)} rms {np.sqrt((e2**2).mean(axis=0)).round(4)} | corr {np.round(c,3)} | |cmd| mean {np.abs(ext_l[mask]).mean(axis=0).round(3)}")
tt = tl[m_l]
report(np.ones_like(tt,bool), "DIRECT all")
report((tt>=20)&(tt<=27.9), "hover 20-27.9")
legs = [(27.97,30.98),(39.7,42.71),(52.52,58.16),(71.9,74.91),(85.99,89.0)]
mv = np.zeros_like(tt,bool)
for a,b in legs: mv |= (tt>=a)&(tt<=b)
report(mv, "arm/base moving legs")
report((tt>=60)&(tt<=69), "hold 60-69")
report((tt>=76)&(tt<=84), "hold 76-84")
# velocity-dependence of the residual per joint
for j in range(4):
    r = app_l[:,j]-ext_l[:,j]; v = vel_l[:,j]
    if v.std()>1e-4:
        b = np.polyfit(v, r, 1); print(f"  joint {j+1}: residual vs qdot slope {b[0]:+.3f} N.m per rad/s, intercept {b[1]:+.4f}; |qdot| max {np.abs(v).max():.3f}")
# streamed command ripple: rms of high-passed command
print("  streamed cmd std in hold 60-69:", ext_l[(tt>=60)&(tt<=69)].std(axis=0).round(4), " applied std:", app_l[(tt>=60)&(tt<=69)].std(axis=0).round(4))
# joint tracking from js vs armref (hardware frame)
ta = T("armref"); qa = np.degrees(d["armref__points[0].positions"])
m = (tj>=td[0])&(tj<=td[-1])
qa_i = np.column_stack([np.interp(tj[m], ta, qa[:,j]) for j in range(4)])
err = jsq[m]-qa_i
print("  joint tracking (hardware frame, js - reference_joint_trajectory): rms", np.sqrt((err**2).mean(axis=0)).round(3), " max", np.abs(err).max(axis=0).round(2))
for a,b,l in [(20,27.9,"hover"),(60,69,"hold 60-69"),(76,84,"hold 76-84"),(89,92.6,"hold 89-92.6")]:
    mm=(tj[m]>=a)&(tj[m]<=b); print(f"    {l:12s} mean err {err[mm].mean(axis=0).round(3)}  q meas mean {jsq[m][mm].mean(axis=0).round(2)}  ref {qa_i[mm].mean(axis=0).round(2)}")

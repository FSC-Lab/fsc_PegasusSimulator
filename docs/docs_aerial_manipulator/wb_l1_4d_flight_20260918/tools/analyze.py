#!/usr/bin/env python3
import numpy as np, sys
np.set_printoptions(precision=4, suppress=True, linewidth=150)
d = np.load(sys.argv[1], allow_pickle=True)
t0 = d["wb__recv"][0]
OFF = float(np.median(d["js__recv"] - d["js__hdr"]))   # vehicle hdr -> recorder clock
def T(key, hdr=True):
    if hdr and f"{key}__hdr" in d: return d[f"{key}__hdr"] + OFF - t0
    return d[f"{key}__recv"] - t0
tw = T("wb", hdr=False); W = d["wb__data"]
L = np.sum(~np.isnan(W), axis=1)
print("wb array length by mode:", {int(m): sorted(set(L[W[:,0]==m])) for m in (0,1)})
D = W[:,0] > 0.5
td, wd = tw[D], W[D]
print(f"DIRECT {td[0]:.2f} .. {td[-1]:.2f} s  ({td[-1]-td[0]:.1f} s, {D.sum()} ticks, median dt {np.median(np.diff(td))*1e3:.2f} ms, p99 {np.percentile(np.diff(td),99)*1e3:.1f} ms, max {np.diff(td).max()*1e3:.1f})")
# flags
print("flags over DIRECT: hold[1] mean %.3f  gmo[2] %.3f  armref_fresh[3] %.4f  armstate_fresh[4] %.4f  stream_fresh[56] %.4f  l1_active[57] %.4f  bound[88] max %.0f mean %.4f  chi[105] min %.0f mean %.4f  n_sat[51] max %.0f  unalloc_thrust[55] max %.3f  unalloc_tau max %.4f"
      % (wd[:,1].mean(), wd[:,2].mean(), wd[:,3].mean(), wd[:,4].mean(), wd[:,56].mean(), wd[:,57].mean(), wd[:,88].max(), wd[:,88].mean(), wd[:,105].min(), wd[:,105].mean(), wd[:,51].max(), np.abs(wd[:,55]).max(), np.abs(wd[:,52:55]).max()))
print("F_hat_y consumed [58..61] max abs:", np.abs(wd[:,58:62]).max(axis=0))
# legs
ts = d["pl_status__recv"] - t0; sv = d["pl_status__data"]
legs = []
for i in range(len(sv)):
    if str(sv[i]).startswith("EXECUTING"):
        j = i+1
        while j < len(sv) and not str(sv[j]).startswith("HOLD"): j += 1
        legs.append((ts[i], ts[j]))
names = ["arm out (EE +0.02 x, -0.07 z)", "arm home", "base step +0.61 m x", "arm out #2", "arm home #2"]
print("legs:", [(round(a,2), round(b,2)) for a,b in legs])
# ---- CoM / EE / attitude ----
xcd, xc = wd[:,45:48], wd[:,48:51]; e = xc - xcd; en = np.linalg.norm(e[:,:2],axis=1)
ey = wd[:,24:28]; eR = wd[:,28:31]; eRn = np.linalg.norm(eR,axis=1)
q = np.degrees(wd[:,5:9]); qd = np.degrees(wd[:,9:13]); eq = q - qd
# odom based tilt/attitude
to = d["odom__recv"] - t0
qw,qx,qy,qz = [d[f"odom__pose.pose.orientation.{k}"] for k in "wxyz"]
R22 = 1 - 2*(qx**2 + qy**2)
tilt = np.degrees(np.arccos(np.clip(R22,-1,1)))
roll = np.degrees(np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx**2+qy**2)))
pitch = np.degrees(np.arcsin(np.clip(2*(qw*qy-qz*qx),-1,1)))
yaw = np.degrees(np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy**2+qz**2)))
pos = np.column_stack([d["odom__pose.pose.position.x"], d["odom__pose.pose.position.y"], d["odom__pose.pose.position.z"]])
vel = np.column_stack([d["odom__twist.twist.linear.x"], d["odom__twist.twist.linear.y"], d["odom__twist.twist.linear.z"]])
# check debug x_c vs odom
xo = np.column_stack([np.interp(td, to, pos[:,i]) for i in range(3)])
print("x_c[48..50] - odom position (mean, DIRECT):", (xc - xo).mean(axis=0), " std:", (xc-xo).std(axis=0))
def win(a, b): return (td>=a)&(td<=b)
def wo(a, b): return (to>=a)&(to<=b)
def stats(name, m, mo):
    if m.sum() < 10 or mo.sum() < 5:
        print(f"{name:32s} (window outside DIRECT)"); return
    print(f"{name:32s} CoMxy pk {en[m].max()*1e3:6.1f} rms {np.sqrt((en[m]**2).mean())*1e3:6.1f} | z pk {np.abs(e[m,2]).max()*1e3:5.1f} | EEpos pk {np.linalg.norm(ey[m,:3],axis=1).max()*1e3:6.1f} rms {np.sqrt((np.linalg.norm(ey[m,:3],axis=1)**2).mean())*1e3:5.1f} mm | head pk {np.degrees(np.arcsin(np.abs(ey[m,3]).max())):5.2f} rms {np.degrees(np.arcsin(np.sqrt((ey[m,3]**2).mean()))):5.2f} deg | |eR| pk {eRn[m].max():.4f} rms {np.sqrt((eRn[m]**2).mean()):.4f} | tilt pk {tilt[mo].max():5.2f} pp {tilt[mo].max()-tilt[mo].min():5.2f} | q err pk {np.abs(eq[m]).max(axis=0)} rms {np.sqrt((eq[m]**2).mean(axis=0))}")
print("\n=== per-window tracking (model-frame CoM error; EE task error; heading; attitude) ===")
stats("DIRECT all", win(td[0], td[-1]), wo(td[0], td[-1]))
stats("entry 0-5 s", win(td[0], td[0]+5), wo(td[0], td[0]+5))
stats("entry 5-15 s", win(td[0]+5, td[0]+15), wo(td[0]+5, td[0]+15))
stats("hover 20-27.9 s", win(20, 27.9), wo(20,27.9))
for (a,b),n in zip(legs,names):
    stats(f"LEG {n} move", win(a,b), wo(a,b))
    stats(f"    hold +0..5", win(b,b+5), wo(b,b+5))
    stats(f"    hold +5..10", win(b+5,b+10), wo(b+5,b+10))
    # settled = last 2 s before the next leg or 10 s after
    nxt = min([x[0] for x in legs if x[0] > b] + [td[-1]])
    stats(f"    settled (2 s before next @{nxt:.1f})", win(nxt-2, nxt), wo(nxt-2,nxt))
# mean position offsets in the holds (structural offset)
print("\n=== hold mean CoM error (x,y,z mm) per hold window (settled 2 s) ===")
for (a,b),n in zip(legs,names):
    nxt = min([x[0] for x in legs if x[0] > b] + [td[-1]]); m = win(nxt-2,nxt)
    print(f"  after {n:28s}: {e[m].mean(axis=0)*1e3}  EEpos mean {ey[m,:3].mean(axis=0)*1e3}  head mean {np.degrees(np.arcsin(ey[m,3].mean())):.2f} deg  q err mean {eq[m].mean(axis=0)}")
m = win(20,27.9); print(f"  pre-leg hover 20-27.9      : {e[m].mean(axis=0)*1e3}  EEpos mean {ey[m,:3].mean(axis=0)*1e3}  head mean {np.degrees(np.arcsin(ey[m,3].mean())):.2f} deg  q err mean {eq[m].mean(axis=0)}")
# ---- inputs ----
MG = 3.746170*9.80665
u1 = wd[:,17]; tb = wd[:,21:24]; mot = wd[:,41:45]
print(f"\n=== inputs ===  mg model {MG:.2f} N")
for n,(a,b) in [("DIRECT all",(td[0],td[-1])),("hover 20-27.9",(20,27.9)),("hold 60-69",(60,69)),("hold 76-84",(76,84))]:
    m = win(a,b)
    print(f"  {n:14s} u1 mean {u1[m].mean():6.2f} std {u1[m].std():5.2f} min {u1[m].min():6.2f} max {u1[m].max():6.2f} | tau_body rms {np.sqrt((tb[m]**2).mean(axis=0))} max {np.abs(tb[m]).max(axis=0)} | motors mean {mot[m].mean(axis=0)} min {mot[m].min():.3f} max {mot[m].max():.3f}")
# ---- observer ----
dh = wd[:,31:41]; dc = wd[:,62:72]; wh = wd[:,78:88]; we = wd[:,72:78]; Fr = wd[:,97:101]; wq = wd[:,101:105]
print("\n=== L1 observer (filtered d_hat [31..40]) ===")
for n,(a,b) in [("entry 0-2",(td[0],td[0]+2)),("entry 2-5",(td[0]+2,td[0]+5)),("hover 20-27.9",(20,27.9)),("hold 33-39",(33,39.6)),("hold 60-69",(60,69)),("hold 76-84",(76,84)),("hold 89-92.6",(89,92.6)),("DIRECT all",(td[0],td[-1]))]:
    m = win(a,b)
    print(f"  {n:14s} d_t mean {dh[m,0:3].mean(axis=0)} std {dh[m,0:3].std(axis=0)} | d_r mean {dh[m,3:6].mean(axis=0)} std {dh[m,3:6].std(axis=0)} | d_q mean {dh[m,6:10].mean(axis=0)} std {dh[m,6:10].std(axis=0)}")
m = win(20, 92.6)
print("  deadbeat d^c std (DIRECT 20s+):", dc[m].std(axis=0))
print("  filtered d_hat std          :", dh[m].std(axis=0))
print("  w_hat internal [78..87] mean (hold 60-69):", wh[win(60,69)].mean(axis=0))
print("  w_hat_e [72..77] mean/std (hold 60-69):", we[win(60,69)].mean(axis=0), we[win(60,69)].std(axis=0))
print("  u1 + d_hat_z (hold 60-69): %.3f N  vs mg %.3f" % ((u1+dh[:,2])[win(60,69)].mean(), MG))
# rise time of d_hat_z after entry
z = dh[:,2]; zf = z[win(20,27.9)].mean()
r90 = td[np.argmax(np.abs(z - z[0]) >= 0.9*abs(zf - z[0]))] - td[0] if abs(zf-z[0])>0.1 else float('nan')
print(f"  d_hat_z: at entry {z[0]:.2f} -> hover {zf:.2f} N, 90% rise {r90:.2f} s")
# bounds usage
print("  |d_hat| max: t %.2f N (bound 20)  r %.3f Nm (bound 2)  q %.3f Nm (bound 1.5)" % (np.abs(dh[:,0:3]).max(), np.abs(dh[:,3:6]).max(), np.abs(dh[:,6:10]).max()))
print("\n=== 4-D attribution ===")
print("  chi [105]: fraction free =", (wd[:,105]>0.5).mean())
print("  F_hat_y consumed [58..61]: max |.| =", np.abs(wd[:,58:62]).max())
Fn = np.linalg.norm(Fr[:,:3],axis=1)
def lp(x, tt, wc):
    y = np.zeros_like(x); a = 0.0
    for i in range(1,len(x)):
        dt = tt[i]-tt[i-1]; al = 1-np.exp(-wc*dt); y[i] = y[i-1] + al*(x[i]-y[i-1])
    return y
Frf = np.column_stack([lp(Fr[:,i], td, 0.25) for i in range(4)])
Frf1 = np.column_stack([lp(Fr[:,i], td, 2*np.pi*1.0) for i in range(4)])
for n,(a,b) in [("hover 20-27.9",(20,27.9)),("hold 33-39",(33,39.6)),("hold 60-69",(60,69)),("hold 76-84",(76,84)),("legs moving",(0,0)),("DIRECT all",(td[0],td[-1]))]:
    if n=="legs moving":
        pass
    if n=="legs moving":
        m = np.zeros_like(td,bool)
        for a2,b2 in legs: m |= win(a2,b2)
    else: m = win(a,b)
    print(f"  {n:14s} raw F mean {Fr[m,:3].mean(axis=0)} std {Fr[m,:3].std(axis=0)} |F| mean {Fn[m].mean():.2f} p95 {np.percentile(Fn[m],95):.2f} max {Fn[m].max():.2f} | raw Mz mean {Fr[m,3].mean():+.3f} std {Fr[m,3].std():.3f} | LP0.25 |F| mean {np.linalg.norm(Frf[m,:3],axis=1).mean():.3f} max {np.linalg.norm(Frf[m,:3],axis=1).max():.3f} | LP1Hz |F| mean {np.linalg.norm(Frf1[m,:3],axis=1).mean():.2f} max {np.linalg.norm(Frf1[m,:3],axis=1).max():.2f} | w_q mean {wq[m].mean(axis=0)}")
print("  w_hat_q [101..104] at end of DIRECT:", wq[-1], " max |.| over DIRECT:", np.abs(wq).max(axis=0))
# ---- arm torque execution ----
print("\n=== arm ===")
print("  js names:", d["js__name"][0], " tcmd names:", d["tcmd__name"][0], " armref:", d["armref__joint_names"][0])
tj = T("js"); js_eff = d["js__effort"]; js_pos = np.degrees(d["js__position"]); js_vel = d["js__velocity"]
tc = T("tcmd"); tcmd = d["tcmd__effort"]
tl = T("law"); law = d["law__data"]
print("  js effort range per column:", js_eff.min(axis=0), js_eff.max(axis=0))
print("  tcmd effort range:", tcmd.min(axis=0), tcmd.max(axis=0))
print("  law [9..12] tau_ext range:", law[:,9:13].min(axis=0), law[:,9:13].max(axis=0), " [13..16] cmd counts range:", law[:,13:17].min(axis=0), law[:,13:17].max(axis=0))
print("  law [0] passthrough frac in DIRECT window:", law[(tl>=td[0])&(tl<=td[-1]),0].mean(), " [26] grav corr", law[:,26].max(), " [27] integral", law[:,27].max(), " [3] aux", law[(tl>=td[0])&(tl<=td[-1]),3].mean())
print("  dxl comm_state unique:", np.unique(d["dxl__comm_state"]), " hw_state max:", np.nanmax(d["dxl__dxl_hw_state"]) if d["dxl__dxl_hw_state"].size else None)

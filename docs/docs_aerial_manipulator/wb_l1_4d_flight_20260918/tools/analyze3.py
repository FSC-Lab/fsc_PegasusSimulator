#!/usr/bin/env python3
import numpy as np, sys
np.set_printoptions(precision=4, suppress=True, linewidth=160)
d = np.load(sys.argv[1], allow_pickle=True)
t0 = d["wb__recv"][0]
OFF = float(np.median(d["js__recv"] - d["js__hdr"]))
tw = d["wb__recv"] - t0; W = d["wb__data"]; D = W[:,0] > 0.5; td, wd = tw[D], W[D]
to = d["odom__recv"] - t0
qw,qx,qy,qz = [d[f"odom__pose.pose.orientation.{k}"] for k in "wxyz"]
roll = np.degrees(np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx**2+qy**2)))
pitch = np.degrees(np.arcsin(np.clip(2*(qw*qy-qz*qx),-1,1)))
om = np.column_stack([d[f"odom__twist.twist.angular.{k}"] for k in "xyz"])
def psd_peak(x, fs, fmin=0.05, fmax=10):
    x = x - x.mean(); n = len(x)
    f = np.fft.rfftfreq(n, 1/fs); X = np.abs(np.fft.rfft(x*np.hanning(n)))**2
    m = (f>=fmin)&(f<=fmax); k = np.argmax(X[m]); 
    tot = X[m].sum(); 
    # fraction of power within +-0.15 Hz of the peak
    fp = f[m][k]; band = (f>=fp-0.15)&(f<=fp+0.15)
    return fp, X[band].sum()/tot
def rs(a,b,tt): return (tt>=a)&(tt<=b)
tj = d["js__hdr"] + OFF - t0; tc = d["tcmd__hdr"] + OFF - t0; tcmd = d["tcmd__effort"]
print("=== oscillation content in quiet holds (dominant freq, fraction of power in +-0.15 Hz) ===")
for n,(a,b) in [("hover 15-27.9",(15,27.9)),("hold 33-39.6",(33,39.6)),("hold 60-71.8",(60,71.8)),("hold 76-85.9",(76,85.9))]:
    m = rs(a,b,td); mo = rs(a,b,to); mc = rs(a,b,tc)
    fs = 1/np.median(np.diff(td[m])); fso = 1/np.median(np.diff(to[mo])); fsc = 1/np.median(np.diff(tc[mc]))
    ex = wd[m,48]-wd[m,45]; ey_ = wd[m,49]-wd[m,46]
    out = [f"CoMx {psd_peak(ex,fs)[0]:.2f}Hz({psd_peak(ex,fs)[1]:.2f})", f"CoMy {psd_peak(ey_,fs)[0]:.2f}Hz({psd_peak(ey_,fs)[1]:.2f})",
           f"eR_x {psd_peak(wd[m,28],fs)[0]:.2f}Hz", f"eR_y {psd_peak(wd[m,29],fs)[0]:.2f}Hz",
           f"roll {psd_peak(roll[mo],fso)[0]:.2f}Hz", f"pitch {psd_peak(pitch[mo],fso)[0]:.2f}Hz",
           f"tau_j2 {psd_peak(tcmd[mc,1],fsc)[0]:.2f}Hz", f"tau_j4 {psd_peak(tcmd[mc,3],fsc)[0]:.2f}Hz",
           f"u1 {psd_peak(wd[m,17],fs)[0]:.2f}Hz", f"tb_y {psd_peak(wd[m,22],fs)[0]:.2f}Hz"]
    print(f"  {n:14s} " + "  ".join(out))
    print(f"     std: CoMx {ex.std()*1e3:.1f} mm CoMy {ey_.std()*1e3:.1f} mm  roll {roll[mo].std():.3f} pitch {pitch[mo].std():.3f} deg  gyro std {om[mo].std(axis=0)} rad/s  tau_j2 std {tcmd[mc,1].std()*1e3:.1f} mN.m p-p {np.ptp(tcmd[mc,1])*1e3:.0f}  tau_j4 std {tcmd[mc,3].std()*1e3:.1f} p-p {np.ptp(tcmd[mc,3])*1e3:.0f}  u1 std {wd[m,17].std():.2f} N  tb std {wd[m,21:24].std(axis=0)}")
print("\n=== post-leg decay: CoM xy error envelope (peak |e| in 2 s bins after each leg end) ===")
legs = [(27.97,30.98,"arm out"),(39.7,42.71,"arm home"),(52.52,58.16,"base step"),(71.9,74.91,"arm out #2"),(85.99,89.0,"arm home #2")]
en = np.linalg.norm(wd[:,48:50]-wd[:,45:47],axis=1)*1e3
eRn = np.linalg.norm(wd[:,28:31],axis=1)
for a,b,n in legs:
    bins = []
    for k in range(6):
        m = rs(b+2*k, b+2*k+2, td)
        if m.sum()>10: bins.append(f"{en[m].max():5.0f}")
    mE = []
    for k in range(6):
        m = rs(b+2*k, b+2*k+2, td)
        if m.sum()>10: mE.append(f"{eRn[m].max():.3f}")
    print(f"  {n:12s} |e_xy| peak per 2 s bin: {' '.join(bins)} mm   |e_R| peak: {' '.join(mE)}")
print("\n=== entry: first 12 s in 1 s bins (|e_xy| peak mm, |e_R| peak, u1 mean) ===")
for k in range(12):
    m = rs(td[0]+k, td[0]+k+1, td); print(f"  {k:2d}-{k+1:2d} s  {en[m].max():5.0f} mm  eR {eRn[m].max():.3f}  u1 {wd[m,17].mean():.1f} N  d_z {wd[m,33].mean():+.2f} N")
print("\n=== the blind window 44.65-46.86 s and its recovery ===")
for a,b in [(43,44.6),(44.65,46.86),(46.9,48),(48,50),(50,52.5),(52.5,58.2),(58.2,62),(62,66),(66,71.8)]:
    m = rs(a,b,td); print(f"  {a:5.1f}-{b:5.1f}  |e_xy| mean {en[m].mean():5.0f} max {en[m].max():5.0f} mm  e_x mean {(wd[m,48]-wd[m,45]).mean()*1e3:+6.0f} e_y {(wd[m,49]-wd[m,46]).mean()*1e3:+6.0f}  |e_R| max {eRn[m].max():.3f}  u1 min {wd[m,17].min():.1f}")
# velocity spike from odom at the jump
v = np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"]); m = rs(46.8,47.0,to)
print("  odom velocity around 46.87 s (|v| max):", np.linalg.norm(v[m],axis=1).max(), " normal |v| in 60-69:", np.linalg.norm(v[rs(60,69,to)],axis=1).mean())

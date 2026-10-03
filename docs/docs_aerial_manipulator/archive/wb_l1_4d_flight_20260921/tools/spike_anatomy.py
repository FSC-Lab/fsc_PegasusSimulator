import numpy as np
np.set_printoptions(precision=3, suppress=True, linewidth=200)
d=np.load("/tmp/claude-1000/-home-shiqi-fsc-PegasusSimulator/439adc13-0e06-4d06-ac66-d9a5a6af280f/scratchpad/c3.npz",allow_pickle=True)
t0=d["wb__recv"][0]
th=d["mocap__hdr"]-t0
P=np.column_stack([d[f"mocap__pose.position.{a}"] for a in "xyz"])
Q=np.column_stack([d[f"mocap__pose.orientation.{a}"] for a in "wxyz"])
V=np.column_stack([d[f"mocap__twist.linear.{a}"] for a in "xyz"])
W=np.column_stack([d[f"mocap__twist.angular.{a}"] for a in "xyz"])
dP=np.diff(P,axis=0); n=np.linalg.norm(dP,axis=1)
print("=== ALL mocap steps > 0.03 m ===")
idx=np.where(n>0.03)[0]
for i in idx: print(f"t={th[i+1]:7.3f}  dp={dP[i]}  |dp|={n[i]:.3f}  P_before={P[i]}  P_after={P[i+1]}  v_after={V[i+1]}")
# quaternion angle steps
dq=np.abs(np.sum(Q[1:]*Q[:-1],axis=1)); ang=2*np.degrees(np.arccos(np.clip(dq,-1,1)))
print("\n=== mocap orientation steps > 3 deg ===")
for i in np.where(ang>3)[0]: print(f"t={th[i+1]:7.3f}  dang={ang[i]:.2f} deg")
print("\norientation step median %.3f p99 %.3f max %.2f deg"%(np.median(ang),np.percentile(ang,99),ang.max()))
print("\n=== window prints around glitches (mocap pos/vel) ===")
for tc in [20.28,24.27,25.81,39.89]:
    m=(th>tc-0.05)&(th<tc+0.08)
    print(f"-- around t={tc}")
    for ti,pi,vi in zip(th[m],P[m],V[m]): print(f"   {ti:7.3f}  P={pi}  V={vi}")
# structure: is it a single-sample spike (returns) or a step?
print("\n=== spike classification ===")
for i in idx:
    if i+2<len(P):
        back=np.linalg.norm(P[i+2]-P[i]); print(f"t={th[i+1]:7.3f}  |P[i+2]-P[i]|={back:.3f} (small => one-sample spike)")
# controller response windows
t=d["wb__recv"]-t0; D=d["wb__data"]
def win(tc,pre=0.15,post=0.6):
    m=(t>tc-pre)&(t<tc+post)
    print(f"\n=== wb debug around {tc} ===")
    print(" t      u1     |tau_body|   e_R          e_p(x_c-x_cd)          d_hat_t         motors            unalloc thr  Fraw          tau_j")
    for k in np.where(m)[0][::6]:
        r=D[k]; print(f"{t[k]:6.3f} {r[17]:6.2f}  {np.linalg.norm(r[21:24]):5.2f}  {r[28:31]}  {r[48:51]-r[45:48]}  {r[31:34]}  {r[41:45]}  {r[55]:7.2f}  {r[97:100]}  {r[13:17]}")
win(20.28); win(24.27)

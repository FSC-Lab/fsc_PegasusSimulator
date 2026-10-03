import numpy as np
np.set_printoptions(precision=3, suppress=True, linewidth=200)
S="/tmp/claude-1000/-home-shiqi-fsc-PegasusSimulator/439adc13-0e06-4d06-ac66-d9a5a6af280f/scratchpad"
for nm,fn,(a,b) in [("0921 #1 (60Hz)",f"{S}/c1.npz",(6,50)),("0921 #3 (120Hz)",f"{S}/c3.npz",(4.6,13.3)),("0918 (60Hz)",f"{S}/f18.npz",(10.2,27.8))]:
    d=np.load(fn,allow_pickle=True); t0=d["wb__recv"][0]
    tm=d["mocap__hdr"]-t0; P=np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"]); V=np.column_stack([d[f"mocap__twist.linear.{k}"] for k in "xyz"])
    m=(tm>a)&(tm<b); dt=np.diff(tm[m]); 
    print(f"\n### {nm}: mocap dt (ms) percentiles 1/25/50/75/99: {np.percentile(dt*1000,[1,25,50,75,99])}; std {dt.std()*1000:.2f} ms")
    x=V[m,0]-V[m,0].mean(); ac=np.correlate(x,x,'full')[len(x)-1:len(x)+8]/np.dot(x,x); print("  published vx autocorr lags0..8:", ac)
    # reconstruct: FD with stamps, FD with nominal dt, central difference
    Pm=P[m]; tmm=tm[m]
    fd_st=np.diff(Pm,axis=0)/np.diff(tmm)[:,None]; fd_nom=np.diff(Pm,axis=0)/np.median(dt); cd=(Pm[2:]-Pm[:-2])/(tmm[2:]-tmm[:-2])[:,None]
    Vp=V[m][1:]
    for lab,cand in [("FD/stamp dt",fd_st),("FD/nominal dt",fd_nom)]:
        print(f"  corr(published v, {lab}) x/y/z:", [round(np.corrcoef(Vp[:,k],cand[:,k])[0,1],3) for k in range(3)], " rms diff", np.sqrt(((Vp-cand)**2).mean(0)))
    Vp2=V[m][1:-1]; print("  corr(published v, central diff):", [round(np.corrcoef(Vp2[:,k],cd[:,k])[0,1],3) for k in range(3)], " rms diff", np.sqrt(((Vp2-cd)**2).mean(0)))
    # shifted: published v[k] vs FD ending at k+1 etc
    for sh in [-1,1,2]:
        c=[round(np.corrcoef(V[m][1:][max(0,sh):len(fd_st)+min(0,sh)][:,k], fd_st[max(0,-sh):len(fd_st)-max(0,sh)][:,k])[0,1],3) for k in range(3)]; print(f"  corr(published v shifted {sh}, FD/stamp):", c)
    print("  std published v", V[m].std(0), " std FD/stamp", fd_st.std(0), " std FD/nominal", fd_nom.std(0), " std central", cd.std(0))
    # PX4 EKF velocity as an alternative
    tp=d["vodom_px__recv"]-t0; Vpx=d["vodom_px__velocity"]; mp=(tp>a)&(tp<b); print("  PX4 EKF velocity (NED) std in same window:", Vpx[mp].std(0), " rate", 1/np.median(np.diff(tp)))
    # smoothed alternative: 5-sample moving average of published v
    k=5; sm=np.convolve(V[m,0],np.ones(k)/k,'valid'); print(f"  5-sample MA of published vx std {sm.std():.4f}")

import numpy as np
np.set_printoptions(precision=3, suppress=True, linewidth=220)
from numpy.fft import rfft, rfftfreq
def eul(q):  # wxyz -> roll pitch yaw deg
    w,x,y,z=q.T
    return np.degrees(np.column_stack([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),np.arcsin(np.clip(2*(w*y-z*x),-1,1)),np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))]))
def tilt_deg(q):
    R22=1-2*(q[:,1]**2+q[:,2]**2); return np.degrees(np.arccos(np.clip(R22,-1,1)))
def qstep(Q):
    dq=np.abs(np.sum(Q[1:]*Q[:-1],axis=1)); return 2*np.degrees(np.arccos(np.clip(dq,-1,1)))
def peakf(x,fs):
    x=x-x.mean(); f=rfftfreq(len(x),1/fs); X=np.abs(rfft(x*np.hanning(len(x))))**2; return f[np.argmax(X[1:])+1], X[f>5].sum()/X[1:].sum()
def load(nm):
    d=np.load(f"<scratch>/{nm}.npz",allow_pickle=True)
    t0=d["wb__recv"][0]
    tmo=d["wbmode__recv"]-t0; mo=d["wbmode__data"]; last=None; ed=[]
    for ti,vi in zip(tmo,mo):
        if vi!=last: ed.append((ti,vi)); last=vi
    st=[b for a,b in ed]
    if "DIRECT" in st:
        i=st.index("DIRECT"); a=ed[i][0]; b=(ed[i+1][0] if i+1<len(ed) else tmo[-1])
    else: a,b=None,None
    ps=d["pl_status__data"]; tps=d["pl_status__recv"]-t0
    ex=[(ti,str(s)) for ti,s in zip(tps,ps) if str(s).startswith("EXECUTING")]
    hold=[(ti,str(s)) for ti,s in zip(tps,ps) if str(s).startswith("HOLD")]
    return d,t0,a,b,ex,hold

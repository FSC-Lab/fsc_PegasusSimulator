import numpy as np
def smooth(t, x, w=0.5):
    ok = ~np.isnan(x).any(1); t, x = t[ok], x[ok]
    cs = np.cumsum(np.vstack([np.zeros((1, x.shape[1])), x]), 0)
    lo = np.searchsorted(t, t - w / 2); hi = np.searchsorted(t, t + w / 2)
    return t, (cs[hi] - cs[lo]) / np.maximum(hi - lo, 1)[:, None]

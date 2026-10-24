"""Find the free coast-downs: a speed peak (|w| >= 200 deg/s, the hand's flick ending), then a
monotone decay to rest with no re-acceleration. Returns (t_start, t_end, sign) per segment."""
import numpy as np

def find(t, w, peak_min=np.radians(200), start_after=0.12, stop_w=np.radians(15),
         reaccel=np.radians(40)):
    W = np.abs(w); segs = []; i = 1; n = len(t)
    while i < n - 1:
        if W[i] >= peak_min and W[i] >= np.nanmax(W[max(0, i - 40):i + 40]):
            s = np.sign(w[i]); j = i; run_min = W[i]; ok = True
            while j < n - 1 and not (np.sign(w[j]) == s and W[j] < stop_w) and np.sign(w[j]) == s:
                run_min = min(run_min, W[j])
                if W[j] > run_min + reaccel:     # pushed again before stopping
                    ok = False; break
                j += 1
            t0 = t[i] + start_after; t1 = t[j] - 0.03
            if ok and t1 - t0 > 0.5:
                segs.append((t0, t1, s, t[i]))
            i = j + 1
        else:
            i += 1
    return segs

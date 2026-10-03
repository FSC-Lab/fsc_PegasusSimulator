"""Where the whole-body CoM error lives in frequency: circle run, x/y combined, Welch PSD (8 s Hann, 50 % overlap).
Bands: lap-periodic (< 0.1 Hz), 0.1-0.6 Hz (the position/attitude cascade), > 0.6 Hz. Also the damping of the
position loop implied by the gains, with the total mass 3.746 kg: wn = sqrt(k_x/m), zeta = k_v / (2 sqrt(k_x m)).
Writes ../analysis/com_spectrum.json."""
import json
from common import *
from scipy.signal import welch
import metrics as M
m_tot = 3.746170
res = {}
for nm in ("a3", "a4", "w1", "w2", "d1", "d2"):
    o, S = M.analyse(nm); e = S["xc"] - S["xcd"]
    f, Px = welch(e[:, 0] - e[:, 0].mean(), fs=100, nperseg=800); _, Py = welch(e[:, 1] - e[:, 1].mean(), fs=100, nperseg=800)
    P = Px + Py; df = f[1] - f[0]; tot = P.sum() * df
    band = lambda a, b: float(np.sqrt(P[(f >= a) & (f < b)].sum() * df) * 1e3)
    i = np.argmax(P[1:]) + 1
    res[nm] = dict(peak_hz=float(f[i]), rms_total=float(1e3 * np.sqrt(tot)), lt01=band(0, 0.1), b01_06=band(0.1, 0.6), gt06=band(0.6, 50),
                   mean_xy=(1e3 * e[:, :2].mean(0)).tolist())
    r = res[nm]
    print(f"{FLIGHTS[nm]:14s} CoM xy (detrended) rms {r['rms_total']:5.1f} mm | <0.1 Hz {r['lt01']:5.1f} | 0.1-0.6 Hz {r['b01_06']:5.1f} | >0.6 Hz {r['gt06']:5.1f} | peak {r['peak_hz']:.3f} Hz | mean xy {np.round(r['mean_xy'],1)}")
for lab, kx, kv in (("0924 gains", 32.0, 20.0), ("H1b (today)", 50.03, 12.58)):
    print(f"{lab}: wn {np.sqrt(kx/m_tot):.2f} rad/s ({np.sqrt(kx/m_tot)/2/np.pi:.2f} Hz), zeta {kv/(2*np.sqrt(kx*m_tot)):.2f}")
json.dump(res, open("../analysis/com_spectrum.json", "w"), indent=1)

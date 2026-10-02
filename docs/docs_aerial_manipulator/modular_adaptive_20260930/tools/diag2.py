import numpy as np, modular_bench as MB, mapped as MP, circle_bench as CB
from fsc_aerial_manipulation.robotic_arm.utils_controller import modular_adaptive as MA
p = dict(MP.TABLE2_MAPPED, a_kp=300.0, a_kd=35.0)
import sys
if len(sys.argv) > 1: p.update(eval(sys.argv[1]))
# capture the adapter's last outputs
trace = []
orig = MB.Adapter.__call__
def call(self, Xm, dyn, ref, dt):
    o = orig(self, Xm, dyn, ref, dt)
    out, rc = self.last
    trace.append((out["e_p"].copy(), out["e_q"].copy(), out["e_a"].copy(), out["tau_q"].copy(), out["u1"], out["rho"].copy(), out["nr"].copy(), out["chi_n"], out["xi_n"], rc["p_d"].copy(), out["tau_p"].copy()))
    return o
MB.Adapter.__call__ = call
r = MB.run(MP.make(p), fb=CB.FB_IDEAL if "ideal" in sys.argv else None)
print(r["verdict"], r["t_end"])
for k in range(0, len(trace), max(1, len(trace)//15)):
    e_p, e_q, e_a, tq, u1, rho, nr, chi, xi, pd, tp = trace[k]
    print(f"k={k:5d} t={k*0.004:5.2f} e_p={np.round(e_p*1e3,0)} e_q={np.round(np.degrees(e_q),2)} e_a={np.round(np.degrees(e_a),1)} tau_q={np.round(tq,3)} u1={u1:.1f} tp={np.round(tp,2)} rho={np.round(rho,3)} |r|={np.round(nr,3)} chi={chi:.2f} xi={xi:.2f}")

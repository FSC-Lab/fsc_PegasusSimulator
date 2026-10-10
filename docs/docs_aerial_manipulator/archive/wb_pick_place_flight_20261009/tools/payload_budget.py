"""What the vehicle and the arm carried: the vertical-force budget around the lift and the removal.

Three independent readings of the load, per flight, on one 10 Hz grid over DIRECT:
  thrust   u1 = the law's collective in "allocator newtons" (alloc_thrust_coeff * sum(omega_cmd^2)); the same motor
           commands turned into delivered thrust with the 0820 stepped-payload fit of this airframe's motors,
           kf_eff = exp(-16.767 + 2.149 ln V - 0.264 u)  (battery voltage V, mean motor command u)
  power    battery V * I (hover power goes as thrust^1.5)
  arm      applied joint torque (Present Current / kappa_tau) minus the arm's own gravity torque S*g(q, R0);
           what is left on j2 / j3 is J^T F for a force F at the hook point -> F
Usage: payload_budget.py [table <run> [t0 t1 [step]]]   (no argument: the budget, written to ../analysis/payload_budget.json)
"""
import json, sys
from pp_common import *
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as CT

KF_ALLOC = 4.260431e-05
OM_IDLE, OM_MAX = 64.0603, 730.0507
S_G = np.array([1.0, 0.897, 0.969, 1.0])          # the arm controller's gravity_scale (measured 2026-09-11)
NJ = C.PARAMS["n"]


def kf_fit(V, u):
    return np.exp(-16.767 + 2.149 * np.log(V) - 0.264 * u)


def gjoint(q, Rm):
    X = np.zeros(18 + 2 * NJ); X[3:12] = Rm.reshape(9, order="F"); X[12:12 + NJ] = q
    return CT.dynamics(X, C.PARAMS)["g"][6:6 + NJ]


def hook_jac(q, Rm, eps=1e-6):
    """d(claw position, world)/dq with the base fixed: 3 x 4."""
    J = np.zeros((3, NJ))
    for k in range(NJ):
        dq = np.zeros(NJ); dq[k] = eps
        J[:, k] = (Rm @ TP.arm_fk_model(q + dq, C.PARAMS)[1] - Rm @ TP.arm_fk_model(q - dq, C.PARAMS)[1]) / (2 * eps)
    return J


def lp(x, n):
    k = np.ones(n) / n
    if x.ndim == 1:
        return np.convolve(x, k, "same")
    return np.column_stack([np.convolve(x[:, j], k, "same") for j in range(x.shape[1])])


def build(nm, dt=0.1):
    d, t0 = load(nm); a, b = direct_span(d, t0)
    t = np.arange(a + 0.2, b - 0.1, dt)
    tw, D = wbdebug(d, t0)
    # average the 250 Hz debug stream into each grid cell (not point samples)
    def cell(tsrc, X):
        X = np.asarray(X, float); X2 = X if X.ndim > 1 else X[:, None]
        idx = np.clip(((tsrc - t[0]) / dt + 0.5).astype(int), -1, len(t))
        out = np.full((len(t), X2.shape[1]), np.nan)
        ok = (idx >= 0) & (idx < len(t))
        cnt = np.bincount(idx[ok], minlength=len(t)).astype(float)
        for j in range(X2.shape[1]):
            s = np.bincount(idx[ok], weights=X2[ok, j], minlength=len(t))
            out[:, j] = np.where(cnt > 0, s / np.maximum(cnt, 1), np.nan)
        for j in range(X2.shape[1]):                      # fill empty cells
            m = np.isfinite(out[:, j]); out[:, j] = np.interp(t, t[m], out[m, j])
        return out if X.ndim > 1 else out[:, 0]
    u1 = cell(tw, D[:, WB["u1"]]); mot = cell(tw, D[:, WB["motors"]]); dh = cell(tw, D[:, WB["d_hat"]])
    tau_law = cell(tw, D[:, WB["tau"]]); wq = cell(tw, D[:, WB["w_q"]]); Fraw = cell(tw, D[:, WB["F_raw"]])
    om = OM_IDLE + (OM_MAX - OM_IDLE) * mot
    S2 = (om ** 2).sum(1)
    tb = d["batt__recv"] - t0
    V = np.interp(t, tb, d["batt__voltage_v"]); I = np.interp(t, tb, d["batt__current_a"])
    to, P, Q, Vel, W = odom(d, t0)
    Pu = interp(t, to, P); Vu = cell(to, Vel); Qu = quat_cont(interp(t, to, Q)); Qu /= np.linalg.norm(Qu, axis=1)[:, None]
    acc = np.gradient(lp(Vu, 5), t, axis=0)
    tj = d["js__recv"] - t0
    app = cell(tj, d["js__effort"][:, :4][:, JS_IDX] / KT * HW_SIGN)
    q = cell(tj, d["js__position"][:, :4][:, JS_IDX] * HW_SIGN)
    qd = cell(tj, d["js__velocity"][:, :4][:, JS_IDX] * HW_SIGN)
    g = np.zeros((len(t), 4)); lev = np.zeros((len(t), 4)); Fz = np.zeros((len(t), 2))
    for k in range(len(t)):
        Rm = quat_R(*Qu[k]) @ RZM90
        g[k] = gjoint(q[k], Rm)
        J = hook_jac(q[k], Rm)
        lev[k] = J[2]                                   # d z_claw / dq_j: a downward force F gives tau_j = +F * (-J_z)... sign below
    res = app - S_G * g                                 # joint torque not explained by the arm's own weight
    # a force (0, 0, -F) at the claw needs holding torque tau = -J^T (0,0,-F) = +F * J_z row
    with np.errstate(divide="ignore", invalid="ignore"):
        Fz = res[:, 1:3] / lev[:, 1:3]
    tg, G = gripper(d, t0); grip = np.interp(t, tg, G[:, 0])
    return dict(d=d, t0=t0, t=t, a=a, b=b, u1=u1, mot=mot, S2=S2, V=V, I=I, P=Pu, vel=Vu, acc=acc, Q=Qu, dh=dh,
                tau_law=tau_law, app=app, q=q, qd=qd, g=g, lev=lev, res=res, Fz=Fz, wq=wq, Fraw=Fraw, grip=grip,
                T_alloc=KF_ALLOC * S2, T_fit=kf_fit(V, mot.mean(1)) * S2, Pw=V * I, yaw=np.degrees(yaw_of(Qu)))


def table(nm, ta=None, tb_=None, step=2.0):
    B = build(nm); t = B["t"]; d, t0 = B["d"], B["t0"]
    ph = pp_phases(d, t0)
    def lab(ts):
        for a, b, l in ph:
            if a <= ts < b:
                return l
        return ""
    ta = t[0] if ta is None else ta; tb_ = t[-1] if tb_ is None else tb_
    print(f"== {RUNS[nm][0]}  DIRECT {B['a']:.1f}-{B['b']:.1f} s")
    print("    t      u1   T_fit   mot    V      I     P[W]    z    |v|   yaw   dz^    tau2a tau3a  res2  res3   Fz2   Fz3   grip  phase")
    for ts in np.arange(ta, tb_, step):
        m = (t >= ts) & (t < ts + step)
        if not m.any():
            continue
        f = lambda x: float(np.nanmean(x[m]))
        print(f"{ts:7.1f} {f(B['u1']):6.2f} {f(B['T_fit']):6.2f} {f(B['mot'].mean(1)):.3f} {f(B['V']):6.2f} {f(B['I']):5.1f} {f(B['Pw']):6.0f}"
              f" {f(B['P'][:, 2]):5.2f} {f(np.linalg.norm(B['vel'], axis=1)):5.2f} {f(B['yaw']):6.0f} {f(B['dh'][:, 2]):6.2f}"
              f"  {f(B['app'][:, 1]):5.2f} {f(B['app'][:, 2]):5.2f} {f(B['res'][:, 1]):5.2f} {f(B['res'][:, 2]):5.2f}"
              f" {f(B['Fz'][:, 0]):5.1f} {f(B['Fz'][:, 1]):5.1f} {f(B['grip']):6.3f}  {lab(ts + step / 2)}")


def budget():
    """The vertical-force budget of PP-2 and the unloaded references of all three flights."""
    out = {"unloaded": {}}
    B = {nm: build(nm) for nm in RUNS}
    hov = {"p1": [(30, 175)], "p3": [(14, 74)], "p2": [(16, 45.4), (121, 144)]}
    for nm, X in B.items():
        t = X["t"]; sp = np.linalg.norm(X["vel"], axis=1)
        m = (sp < 0.1) & np.any([(t > a) & (t < b) for a, b in hov[nm]], axis=0)
        out["unloaded"][nm] = dict(T_fit=float(X["T_fit"][m].mean()), T_fit_std=float(X["T_fit"][m].std()), P=float(X["Pw"][m].mean()), P_std=float(X["Pw"][m].std()),
                                   u1=[float(X["u1"][m].min()), float(X["u1"][m].max())], V=[float(X["V"][m].max()), float(X["V"][m].min())],
                                   dz=[float(X["dh"][m][:, 2].max()), float(X["dh"][m][:, 2].min())], seconds=float(m.sum() * 0.1))
    X = B["p2"]; t = X["t"]
    W = dict(pre=(38.5, 45.4), post=(49.6, 53.4), carry=(53.4, 73.0), ready_place=(79.0, 84.0), on_fingers=(98.0, 116.0), after=(120.0, 144.0))
    win = {}
    for k, (a, b) in W.items():
        m = (t >= a) & (t < b); f = lambda x: float(np.nanmean(x[m]))
        win[k] = dict(t=[a, b], u1=f(X["u1"]), T_fit=f(X["T_fit"]), P=f(X["Pw"]), mot=f(X["mot"].mean(1)), V=f(X["V"]), I=f(X["I"]), dz=f(X["dh"][:, 2]),
                      app=X["app"][m].mean(0).round(3).tolist(), res=X["res"][m].mean(0).round(3).tolist(), lev=X["lev"][m].mean(0).round(3).tolist())
        win[k]["ratio"] = win[k]["u1"] / win[k]["T_fit"]
    out["windows"] = win

    def split(k0, k1):
        a, b = win[k0], win[k1]
        d_u1 = b["u1"] - a["u1"]; d_T = b["T_fit"] - a["T_fit"]
        load_alloc = d_T * a["ratio"]                                   # the load, in the allocator's newtons at the earlier ratio
        batt = b["T_fit"] * a["ratio"] * 2.149 * np.log(a["V"] / b["V"])     # battery sag between the two windows
        thr = b["T_fit"] * a["ratio"] * 0.264 * (b["mot"] - a["mot"])        # thrust per command falls at the higher throttle
        return dict(d_u1=d_u1, d_T_fit=d_T, load_in_alloc_N=load_alloc, unit=load_alloc - d_T, battery=batt, throttle=thr,
                    check=load_alloc + batt + thr, d_P_ratio=b["P"] / a["P"])
    out["lift"] = split("pre", "post"); out["to_place"] = split("pre", "ready_place"); out["removal"] = split("after", "on_fingers")
    # the load by the three gauges (newtons)
    W0 = 3.74617 * 9.80665
    T0 = win["pre"]["T_fit"]; P0 = win["pre"]["P"]
    g = {}
    for k in ("post", "carry", "ready_place", "on_fingers"):
        base = "pre" if k != "on_fingers" else "after"
        dT = win[k]["T_fit"] - win[base]["T_fit"]; pr = win[k]["P"] / win[base]["P"]
        g[k] = dict(fit_N=dT, fit_frac=dT / win[base]["T_fit"], power_frac=pr ** (2 / 3) - 1,
                    power_N=[(pr ** (2 / 3) - 1) * win[base]["T_fit"], (pr ** (2 / 3) - 1) * W0], fit_N_scaled=dT / win[base]["T_fit"] * W0)
    out["gauges"] = g
    # arm: loaded carry split by the direction the joint last moved (velocity observer), friction-free midpoint
    d, t0 = X["d"], X["t0"]; tv = d["velobs__recv"] - t0; vob = d["velobs__velocity"][:, :4]
    arm = {}
    for j, nmj in ((1, "j2"), (2, "j3")):
        v = np.interp(t, tv, vob[:, j]); m = (t > 49.6) & (t < 88.5)
        up = m & (v > np.radians(0.7)); dn = m & (v < -np.radians(0.7))
        arm[nmj] = dict(up=float(X["res"][up, j].mean()), down=float(X["res"][dn, j].mean()), lever=float(X["lev"][m, j].mean()))
        arm[nmj]["mid"] = 0.5 * (arm[nmj]["up"] + arm[nmj]["down"]); arm[nmj]["F"] = arm[nmj]["mid"] / arm[nmj]["lever"]
    l2, l3 = arm["j2"]["lever"], arm["j3"]["lever"]
    arm["F_lsq"] = (arm["j2"]["mid"] * l2 + arm["j3"]["mid"] * l3) / (l2 * l2 + l3 * l3)
    out["arm"] = arm
    # the pitch moment the load puts on the vehicle: rotational disturbance estimate (model x = pitch axis, allocator
    # units -> delivered with the same ratio as the thrust), lever = claw ahead of the modelled system CoM
    mom = {}
    base_r = 0.5 * (np.nanmean(X["dh"][(t >= 33) & (t < 45.4), 3]) + np.nanmean(X["dh"][(t >= 121) & (t < 144), 3]))
    for k, (a, b) in dict(post=(49.6, 53.4), place_start=(68.3, 73.5), on_fingers=(98.0, 116.0)).items():
        m = (t >= a) & (t < b); q = X["q"][m].mean(0)
        r0c, r0e, _ = TP.arm_fk_model(q, C.PARAMS); lever = float((r0e - r0c)[1])
        ratio = float(np.nanmean(X["u1"][m]) / np.nanmean(X["T_fit"][m]))
        dM = float(np.nanmean(X["dh"][m, 3]) - base_r)
        mom[k] = dict(d_hat_r_x=float(np.nanmean(X["dh"][m, 3])), baseline=float(base_r), dM_alloc=dM, dM_delivered=dM / ratio, lever=lever,
                      F=-dM / ratio / lever, F_alloc_units=-dM / lever)
    out["moment"] = mom
    json.dump(out, open(os.path.join(OUT, "payload_budget.json"), "w"), indent=1)
    # figure data: 1 s means
    fig = {}
    for nm in ("p1", "p2"):
        Y = B[nm]; tt = Y["t"]; tq = np.arange(tt[0], tt[-1] - 1.0, 1.0)
        mean = lambda x: [round(float(np.nanmean(x[(tt >= a) & (tt < a + 1.0)])), 3) for a in tq]
        fig[nm] = dict(t=np.round(tq + 0.5, 1).tolist(), u1=mean(Y["u1"]), T_fit=mean(Y["T_fit"]), P=mean(Y["Pw"]), V=mean(Y["V"]),
                       dz=mean(Y["dh"][:, 2]), tau2=mean(Y["app"][:, 1]), tau3=mean(Y["app"][:, 2]))
    fig["p2"]["load"] = [47.4, 118.3]
    json.dump(fig, open(os.path.join(OUT, "fig_load.json"), "w"))
    return out


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "table":
        a = [float(x) for x in sys.argv[3:]]
        table(sys.argv[2], *(a[:2] if len(a) >= 2 else []), **({"step": a[2]} if len(a) >= 3 else {}))
    else:
        print(json.dumps(budget(), indent=1))

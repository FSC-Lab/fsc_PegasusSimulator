import json, sys, numpy as np
R = json.load(open(sys.argv[1]))
D = {"radial": (1, 1), "-radial": (1, -1), "lateral": (0, 1), "-lateral": (0, -1), "up": (2, 1), "down": (2, -1)}
def rd(r): v = r["kw"].get("rd"); return "0.21" if v is None else f"{v:g}"
print("PAYLOAD")
print(f"{'chi':18} {'rd':>4} {'m':>5} {'hw':>3} bff unl verdict  placeN mean/max/end  carryEE rms/z  tiltCarry exPk [lift/place/release]  eePk  util  eeAfter  chi_on / off")
for r in R:
    if r["scn"] != "payload": continue
    k = r["kw"]
    v = "ok " if r["verdict"] == "completed" else f"ABT{r['t_end']:.0f}"
    print(f"{r['chi']:18} {rd(r):>4} {k.get('m',0.1):5.2f} {str(k.get('hw',True))[0]:>3} {str(k.get('base_ff',False))[0]:>3} {str(k.get('unload',False))[0]:>3} {v:7} "
          f"{r.get('place_force',np.nan):5.2f}/{r.get('place_force_max',np.nan):5.2f}/{r.get('place_force_end',np.nan):5.2f}   "
          f"{r.get('ee_carry_rms',np.nan)*1e3:5.1f}/{r.get('ee_carry_z',np.nan)*1e3:5.1f}   {r.get('tilt_carry',np.nan):5.1f}  "
          f"{r.get('ex_pk',np.nan)*1e3:4.0f} [{r.get('ex_lift_pk',np.nan)*1e3:4.0f}/{r.get('ex_place_pk',np.nan)*1e3:4.0f}/{r.get('ex_release_pk',np.nan)*1e3:4.0f}] {r.get('ee_pk',np.nan)*1e3:4.0f}  {r.get('tau_util',np.nan):4.2f}  {r.get('ee_after',np.nan)*1e3:5.1f}  "
          f"{[round(x,1) for x in r.get('chi_on_t',[])]} / {[round(x,1) for x in r.get('chi_off_t',[])]}")
print("FORCE")
print(f"{'chi':18} {'rd':>4} {'dir':8} {'F':>5} {'hw':>3} bff verdict  def_along(pred)mm  Fhat_along  tiltPk exPk eePk util cap%  eeAfter  chi_on/off  noise99")
for r in R:
    if r["scn"] != "force": continue
    k = r["kw"]; ax, sg = D[k["dir"]]
    v = "ok " if r["verdict"] == "completed" else f"ABT{r['t_end']:.0f}"
    de = r.get("ee_def", [np.nan]*3)[ax]*sg*1e3; pr = r.get("ee_def_pred_contact",[np.nan]*3)[ax]*sg*1e3
    fh = r.get("F_hat",[np.nan]*3)[ax]*sg
    print(f"{r['chi']:18} {rd(r):>4} {k['dir']:8} {k['F']:5.1f} {str(k.get('hw',True))[0]:>3} {str(k.get('base_ff',False))[0]:>3} {v:7} {de:6.1f}({pr:5.1f})   {fh:6.2f}    "
          f"{r.get('tilt_pk',np.nan):5.1f} {r.get('ex_pk',np.nan)*1e3:4.0f} {r.get('ee_pk',np.nan)*1e3:4.0f} {r.get('tau_util',np.nan):4.2f} {r.get('cap_pct',np.nan):4.1f} "
          f"{r.get('ee_after',np.nan)*1e3:5.1f}  {[round(x,1) for x in r.get('chi_on_t',[])]}/{[round(x,1) for x in r.get('chi_off_t',[])]} {r.get('fhat_noise',np.nan):.2f}")
print("BOX")
for r in R:
    if r["scn"] != "box": continue
    k = r["kw"]
    v = "ok " if r["verdict"] == "completed" else f"ABT{r['t_end']:.0f}"
    print(f"{r['chi']:18} {rd(r):>4} {k['dir']:8} F_kin {k['F']:5.1f} {str(k.get('hw',True))[0]} bff={str(k.get('base_ff',False))[0]} {v:7} moved {r.get('box_moved',np.nan)*1e3:5.0f} mm  Fc mean {r.get('fc_push_mean',np.nan):5.2f} pk {r.get('fc_pk',np.nan):5.2f}  "
          f"EE rms {r.get('ee_push_rms',np.nan)*1e3:5.1f}  CoM pk {r.get('ex_push_pk',np.nan)*1e3:4.0f}  tilt {r.get('tilt_pk',np.nan):4.1f}  util {r.get('tau_util',np.nan):4.2f} cap {r.get('cap_pct',np.nan):4.1f}%  after {r.get('ee_after',np.nan)*1e3:5.1f}  chi {[round(x,1) for x in r.get('chi_on_t',[])]}/{[round(x,1) for x in r.get('chi_off_t',[])]}")

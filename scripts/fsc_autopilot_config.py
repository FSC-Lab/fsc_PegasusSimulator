#!/usr/bin/env python3
"""The C++ FSC autopilot's configuration, DERIVED from the airframe file (2026-10-28).

    FSC_AIRFRAME_ID=<cfg> VTOL_TANK_KG=0.8 VTOL_TANK_DZ=-0.076 python3 scripts/fsc_autopilot_config.py --out DIR

The C++ autopilot (`autopilot_sv_baseline_node`, scripts/start_vtol_fsc_hil.sh) is a multicopter offboard
controller: its position loop and its own attitude loop ("apm") send BODY-RATE setpoints with a normalized
thrust, and PX4 closes the rates and allocates through ITS control allocation (CA) -- the one controller
here whose airframe model lives in PX4 parameters, not in airframe_id.cfg. Until today those were the
4101 airframe's literals (CAD hub arms, KM 0.12 = the sim-only cQ of before 2026-09-17) and the
autopilot's YAML literals (mass 3.4, thrust_scaling 0.04 + idle 0.225, fitted to an 18 N rotor), so the
path could not follow an airframe change. Written:

  DIR/px4_params.txt     `NAME VALUE` per line, applied on the PX4 console by the launcher:
      CA_ROTORn_PX/PY/PZ  the file's hubs about the WET CG (the water at VTOL_TANK_DX/DZ, as so3 folds it),
                          FLU -> FRD
      CA_ROTORn_CT        the pair's static thrust at full signal [N] (PX4: Thrust = CT u^2, u the signal)
      CA_ROTORn_KM        spin x cq: the sim's reaction torque is -spin cq T about FLU z, PX4's KM is
                          positive for a CCW rotor -- the 4101 signs, with the measured magnitude
      THR_MDL_FAC 1       the sim's static thrust IS Tmax u^2: PX4 then outputs sqrt(rel. thrust), and the
                          allocation's thrust is linear in newtons
      CA_SV_TLn_*         PX4's tilt convention, unchanged: control -1 = MINA = upright (its allocation
                          assumes exactly that when it rebuilds), +1 = MAXA; yaw by differential tilt
      MPC_THR_HOVER       the hover's normalized thrust (the land detector's reference)
  DIR/autopilot_params.yaml  the base YAML with vehicle_mass = the wet mass, and thrust_scaling /
                          idle_thrust such that the autopilot's normalized thrust t = 4 T / (4 K):
                          K = the newtons PX4's normalized allocation makes per unit of collective
                          thrust, computed by REPLICATING its pseudo-inverse and normalization
                          (src/lib/control_allocation/.../ControlAllocationPseudoInverse.cpp, v1.16)
  DIR/env.sh             VTOL_TILT_PX4CA=MINA,MAXA for the Isaac pane: the sim maps PX4's tilt output
                          onto its own convention (angle = cmd x tilt_max) -- the sim's servo
                          calibration, as a real servo's PWM range would be. Without it PX4's upright
                          (-1) is the sim's -tilt_max, clipped to the aft stop.
"""
import argparse
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from controller_path import load_airframe     # noqa: E402

BASE_YAML = os.path.join(HERE, "config", "params_fsc_cad_vtol.yaml")
TILT_MINA, TILT_MAXA = 0.0, 90.0          # PX4's convention for the front pair (4101's values, kept)


def px4_mix(rpos_frd, ct, km, tilt_rotor_y, yaw_by_tilt=True):
    """PX4 v1.16's normalized pseudo-inverse for 4 rotors (axis up = FRD -z) + the yaw-tilts.
    Returns (mix [n_act x 6], effectiveness [6 x n_act]); columns of mix: roll, pitch, yaw, Fx, Fy, Fz."""
    n_r = len(ct); n_t = len(tilt_rotor_y)
    B = np.zeros((6, n_r + n_t))
    axis = np.array([0.0, 0.0, -1.0])
    for i in range(n_r):
        B[0:3, i] = ct[i] * np.cross(rpos_frd[i], axis) - ct[i] * km[i] * axis
        B[3:6, i] = ct[i] * axis
        if yaw_by_tilt:                      # enableYawByDifferentialThrust(!tilts.hasYawControl())
            B[2, i] = 0.0
    for j, y in enumerate(tilt_rotor_y):     # updateTorqueSign: rotated_pos(1) < 0 -> +1, > 0 -> -1
        B[2, n_r + j] = 1.0 if y < -0.01 else (-1.0 if y > 0.01 else 0.0)
    mix = np.linalg.pinv(B)
    n_a = mix.shape[0]
    nr = sum(abs(mix[i, 0]) > 1e-3 for i in range(n_a)); npi = sum(abs(mix[i, 1]) > 1e-3 for i in range(n_a))
    sr = math.sqrt(np.sum(mix[:, 0] ** 2) / (nr / 2.0)) if nr else 1.0
    sp = math.sqrt(np.sum(mix[:, 1] ** 2) / (npi / 2.0)) if npi else 1.0
    scale = np.ones(6); scale[0] = scale[1] = max(sr, sp); scale[2] = mix[:, 2].max()
    scale[5] = 1.0
    for ax in (2, 1, 0):                     # Fz first, then Fy, Fx fall back to it
        col = np.abs(mix[:, 3 + ax]); nz = int(np.sum(col > np.finfo(np.float32).eps))
        scale[3 + ax] = col.sum() / nz if nz else scale[5]
    for c in range(3):
        if scale[c] > 1e-7:
            mix[:, c] /= scale[c]
    mix[:, 3:6] /= scale[3:6]
    mix[np.abs(mix) < 1e-3] = 0.0
    return mix, B


def derive(af, water, dx, dz):
    r = af.rotors
    m_dry = float(af.mass.airframe_kg); m = m_dry + water
    cg = np.array([float(af.mass.cg_x) + water * dx / m, 0.0, water * dz / m])   # wet CG in the hubs' frame
    hubs = np.array(r.hubs, float)
    rel = hubs - cg
    rpos_frd = rel * np.array([1.0, -1.0, -1.0])
    ct = r.per_rotor("max_thrust"); cq = r.per_rotor("cq"); spin = np.array(r.spin, float)
    km = spin * cq
    tilting = [i for i in range(4) if r.tiltable[i] > 0.5]
    mix, B = px4_mix(rpos_frd, ct, km, [rpos_frd[i][1] for i in tilting])
    u1 = mix @ np.array([0, 0, 0, 0, 0, -1.0])          # one unit of collective thrust
    K = float(np.sum(ct * u1[:4]))                      # N per unit (THR_MDL_FAC 1: thrust = CT u)
    W = m * 9.81
    t_hover = W / K
    u_h = mix @ np.array([0, 0, 0, 0, 0, -t_hover])
    res = B @ u_h                                       # what the plant gets: moments and force at hover
    return dict(m=m, cg=cg, rpos_frd=rpos_frd, ct=ct, km=km, spin=spin, tilting=tilting, K=K,
                t_hover=t_hover, u_hover=u_h[:4], hover_force=-res[5], hover_moment=res[0:3],
                tilt_max=float(af.tilt.max_deg), mix=mix)


PARAM_SHOW = re.compile(r"\b([A-Z][A-Z0-9_]+) \[\d+,\d+\] : (-?[\d.]+)")


def verify(d):
    """The launcher's read-back: every `NAME VALUE` in DIR/px4_params.applied must appear, last-shown, in
    DIR/px4_console.txt (PX4's `param show` lines: `x + NAME [i,j] : VALUE`) within 1e-3."""
    want = []
    for ln in open(os.path.join(d, "px4_params.applied")):
        ln = ln.split("#")[0].split()
        if len(ln) == 2:
            want.append((ln[0], float(ln[1])))
    got = {}
    for m in PARAM_SHOW.finditer(open(os.path.join(d, "px4_console.txt"), errors="replace").read()):
        got[m.group(1)] = float(m.group(2))
    bad = [(n, v, got.get(n)) for n, v in want if n not in got or abs(got[n] - v) > 1e-3]
    for n, v, g in bad:
        print(f"[fsc-config] !! {n}: applied {v:g}, PX4 shows {g}")
    print(f"[fsc-config] PX4 read-back: {len(want) - len(bad)}/{len(want)} parameters as applied"
          + ("" if not bad else " -- NOT VERIFIED (no px4_params.verified: fly_fsc_cpp.py will refuse)"))
    return not bad


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--verify":
        sys.exit(0 if verify(sys.argv[2]) else 1)
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--airframe", default=os.environ.get("FSC_AIRFRAME_ID"))
    ap.add_argument("--tank-kg", type=float, default=float(os.environ.get("VTOL_TANK_KG") or 0.0))
    ap.add_argument("--tank-dx", type=float, default=float(os.environ.get("VTOL_TANK_DX") or 0.0))
    ap.add_argument("--tank-dz", type=float, default=float(os.environ.get("VTOL_TANK_DZ") or 0.0))
    ap.add_argument("--base-yaml", default=BASE_YAML)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    af = load_airframe(a.airframe)
    d = derive(af, a.tank_kg, a.tank_dx, a.tank_dz)
    os.makedirs(a.out, exist_ok=True)
    name = os.path.basename(af.path).rsplit(".", 1)[0]

    lines = [f"CA_ROTOR_COUNT 4"]
    for i in range(4):
        px, py, pz = d["rpos_frd"][i]
        lines += [f"CA_ROTOR{i}_PX {px:.4f}", f"CA_ROTOR{i}_PY {py:.4f}", f"CA_ROTOR{i}_PZ {pz:.4f}",
                  f"CA_ROTOR{i}_CT {d['ct'][i]:.2f}", f"CA_ROTOR{i}_KM {d['km'][i]:.4f}"]
    for j, i in enumerate(d["tilting"]):
        lines += [f"CA_ROTOR{i}_TILT {j + 1}", f"CA_SV_TL{j}_MINA {TILT_MINA:g}", f"CA_SV_TL{j}_MAXA {TILT_MAXA:g}",
                  f"CA_SV_TL{j}_CT 1", f"CA_SV_TL{j}_TD 0"]
    # ★ THE OUTPUT ROUTING. PX4's tiltrotor allocation numbers its servos CONTROL SURFACES FIRST, then the
    # tilts (ActuatorEffectivenessTiltrotorVTOL: _control_surfaces.addActuators, then _tilts) -- with 4101's
    # three surfaces the tilts are Servo4/5. 4101 routes Servo1/2 (201/202) to HIL controls[4,5] = the
    # plant's tilt_left/right, which is right for the direct-actuator nodes (their ActuatorServos put the
    # tilts first) and WRONG under PX4's own allocation: the first flight of this path (2026-10-28) sent
    # the aileron and elevator (0 = 45 deg through the tilt map) to the nacelles and PX4's tilt (-1) to the
    # elevator -- read off `listener actuator_servos` / `actuator_outputs_sim` on the ground. This path's
    # routing: controls[4,5] <- Servo4/5 (tilt0/1), controls[6,7,8] <- Servo1/2/3 (aileron/elevator/rudder).
    n_cs = 3
    lines += [f"CA_SV_CS_COUNT {n_cs}"]
    lines += [f"PWM_MAIN_FUNC{5 + j} {200 + n_cs + 1 + j}" for j in range(len(d["tilting"]))]
    lines += [f"PWM_MAIN_FUNC{5 + len(d['tilting']) + k} {201 + k}" for k in range(n_cs)]
    lines += ["VT_TILT_MC 0.0", "THR_MDL_FAC 1.0", f"MPC_THR_HOVER {d['t_hover']:.3f}"]
    # PX4's tilt-yaw RATE loop: 4101's values, pinned -- SITL saves whatever was last set (rootfs_fsc0/parameters.bson),
    # and a stray stiffer MC_YAWRATE_P (0.62, 2026-10-28) made yaw-by-tilt pump roll on this airframe until it fell
    lines += ["MC_YAWRATE_P 0.25", "MC_YAWRATE_I 0.2", "MC_YAWRATE_K 1.0"]
    with open(os.path.join(a.out, "px4_params.txt"), "w") as f:
        f.write(f"# {af.path} af={af.digest()} water {a.tank_kg:g} kg at ({a.tank_dx:g}, 0, {a.tank_dz:g}) -- "
                f"scripts/fsc_autopilot_config.py\n" + "\n".join(lines) + "\n")

    scaling = 4.0 / d["K"]
    y = open(a.base_yaml).read()
    subs = {"vehicle_name": f'"{name}"', "vehicle_thrust_scaling": f"{scaling:.6f}",
            "vehicle_idle_thrust": "0.0", "vehicle_mass": f"{d['m']:.4f}"}
    for k, v in subs.items():
        y, n = re.subn(rf"^(\s*{k}:\s*).*$", lambda mm, v=v: mm.group(1) + v + "   # derived from the airframe file",
                       y, flags=re.M)
        assert n == 1, f"{k} not found once in {a.base_yaml}"
    # the airframe file's [tuning] for this controller: fsc_<key> sets the YAML's <key> (absent = the base value)
    for tk in sorted(k for k in af.tuning if k.startswith("fsc_")):
        k, v = tk[len("fsc_"):], af.tuning[tk]
        y, n = re.subn(rf"^(\s*{k}:\s*).*$", lambda mm, v=v: mm.group(1) + f"{v:g}   # the airframe file's [tuning]",
                       y, flags=re.M)
        assert n == 1, f"[tuning] {tk}: {k} not found once in {a.base_yaml}"
    # FSC_YAML_SET="key=value,key=value": an explicit, recorded override of a base gain (an experiment)
    overrides = [kv.split("=", 1) for kv in os.environ.get("FSC_YAML_SET", "").split(",") if kv.strip()]
    for k, v in overrides:
        k = k.strip(); v = v.strip()
        y, n = re.subn(rf"^(\s*{k}:\s*).*$", lambda mm, v=v: mm.group(1) + v + "   # FSC_YAML_SET (override)", y, flags=re.M)
        assert n == 1, f"FSC_YAML_SET: {k} not found once in {a.base_yaml}"
    hdr = (f"# GENERATED by scripts/fsc_autopilot_config.py from {af.path} (af={af.digest()}) + water "
           f"{a.tank_kg:g} kg -- do not edit; the gains below are the base file's ({a.base_yaml})\n"
           f"# thrust: t = {scaling:.6f} T / 4 (idle 0) = T / {d['K']:.2f} N -- PX4's normalized allocation makes "
           f"{d['K']:.2f} N per unit of collective (THR_MDL_FAC 1); hover t = {d['t_hover']:.3f}\n"
           + (f"# OVERRIDES (FSC_YAML_SET): {', '.join(f'{k.strip()}={v.strip()}' for k, v in overrides)}\n" if overrides else ""))
    with open(os.path.join(a.out, "autopilot_params.yaml"), "w") as f:
        f.write(hdr + y)
    with open(os.path.join(a.out, "env.sh"), "w") as f:
        f.write(f"export VTOL_TILT_PX4CA={TILT_MINA:g},{TILT_MAXA:g}\n")

    print(f"{af.path}  af={af.digest()}  M {d['m']:.4f} kg (water {a.tank_kg:g})  wet CG {np.round(d['cg'], 4)}")
    for i, nm in enumerate(("FL", "FR", "RL", "RR")):
        print(f"  {nm}: FRD ({', '.join(f'{v:+.4f}' for v in d['rpos_frd'][i])})  CT {d['ct'][i]:.2f} N  KM {d['km'][i]:+.4f}")
    print(f"  PX4 allocation: {d['K']:.2f} N per unit collective; hover t {d['t_hover']:.3f} -> signals^2 "
          f"{np.round(d['u_hover'], 3)} = thrusts {np.round(d['ct'] * d['u_hover'], 2)} N "
          f"(sum {d['hover_force']:.2f} = W {d['m'] * 9.81:.2f}; residual moment {np.round(d['hover_moment'], 4)} N.m)")
    print(f"  autopilot: vehicle_mass {d['m']:.4f}, thrust_scaling {scaling:.6f}, idle 0")
    print(f"  sim tilt map: VTOL_TILT_PX4CA={TILT_MINA:g},{TILT_MAXA:g} on tilt_max {d['tilt_max']:g} deg -> "
          f"scale = offset = {(TILT_MAXA - TILT_MINA) / (2 * d['tilt_max']):.4f}")
    print(f"  wrote {a.out}/px4_params.txt, autopilot_params.yaml, env.sh")


if __name__ == "__main__":
    main()

"""The C++ FSC autopilot's configuration is derived from the airframe file, and the plant agrees with it.

    ~/envs/isaac_env/bin/python3 tests/test_fsc_autopilot_config.py

2026-10-28 (scripts/fsc_autopilot_config.py, scripts/start_vtol_fsc_hil.sh, vtol_utils.px4ca_tilt_map):
  1. the replica of PX4 v1.16's normalized pseudo-inverse: a symmetric quad of equal CT makes 4 CT newtons
     per unit of collective (PX4's normalization puts every motor at 1 for thrust 1); on the plan file the
     hover allocation makes exactly the weight with zero moment, every motor inside (0, 1);
  2. the generated PX4 set on the plan file: CT = the pair's static thrust, KM = spin x cq, the hubs about
     the WET CG in FRD, THR_MDL_FAC 1, PX4's own tilt convention (MINA 0 / MAXA 90); the YAML's mass is the
     wet mass and its thrust map is T / K;
  3. the plant's tilt map: with VTOL_TILT_PX4CA="0,90" PX4's -1 is upright and +1 is 90 deg on THIS
     airframe's travel; unset, the map is the identity every direct-actuator flight uses;
  4. the read-back refuses a parameter PX4 does not show as applied;
  5. the launcher: its own PX4 working dir, the config generated from the same file Isaac gets, the tilt map
     and the tank forwarded to Isaac, the read-back gate.
"""
import importlib.util
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
from controller_path import inner_dir, load_airframe     # noqa: E402

PLAN = os.path.join(inner_dir(), "airframe_plan_A.cfg")
bad = []


def check(ok, msg):
    print(("  ok    " if ok else "  FAIL  ") + msg)
    if not ok:
        bad.append(msg)


def gen():
    spec = importlib.util.spec_from_file_location("fac", os.path.join(ROOT, "scripts", "fsc_autopilot_config.py"))
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    return m


def main():
    g = gen()
    print("1. the PX4 allocation replica")
    sq = np.array([[0.25, -0.25, 0], [0.25, 0.25, 0], [-0.25, -0.25, 0], [-0.25, 0.25, 0]])
    mix, B = g.px4_mix(sq, np.full(4, 10.0), np.array([0.05, -0.05, -0.05, 0.05]), [-0.25, 0.25])
    u1 = mix @ np.array([0, 0, 0, 0, 0, -1.0])
    check(np.allclose(u1[:4], 1.0, atol=1e-6), f"symmetric quad: thrust 1 puts every motor at 1 ({np.round(u1[:4], 4)})")
    af = load_airframe(PLAN)
    d = g.derive(af, 0.8, 0.0, -0.076)
    W = d["m"] * 9.81
    check(abs(d["hover_force"] - W) < 1e-6 and np.allclose(d["hover_moment"], 0.0, atol=1e-6),
          f"plan hover: {d['hover_force']:.3f} N = W {W:.3f}, moment {np.round(d['hover_moment'], 6)}")
    check(np.all((d["u_hover"] > 0) & (d["u_hover"] < 1)), f"plan hover signals^2 inside (0, 1): {np.round(d['u_hover'], 3)}")
    T = d["ct"] * d["u_hover"]
    check(np.allclose(T, W / 4, atol=1e-6), f"the CG on the hubs' centroid: four equal thrusts {np.round(T, 3)}")

    print("2. the generated sets on the plan file")
    with tempfile.TemporaryDirectory() as td:
        env = dict(os.environ, FSC_AIRFRAME_ID=PLAN, VTOL_TANK_KG="0.8", VTOL_TANK_DX="0", VTOL_TANK_DZ="-0.076")
        r = subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "fsc_autopilot_config.py"), "--out", td],
                           env=env, capture_output=True, text=True)
        check(r.returncode == 0, f"the generator runs ({r.stderr.strip()[-200:]})")
        p = {}
        for ln in open(os.path.join(td, "px4_params.txt")):
            if not ln.startswith("#") and ln.strip():
                k, v = ln.split(); p[k] = float(v)
        r_ = af.rotors
        cg = np.array([float(af.mass.cg_x), 0.0, 0.8 * -0.076 / (float(af.mass.airframe_kg) + 0.8)])
        rel = (np.array(r_.hubs, float) - cg) * np.array([1.0, -1.0, -1.0])
        ok_geo = all(abs(p[f"CA_ROTOR{i}_{ax}"] - rel[i][j]) < 1e-4 for i in range(4) for j, ax in enumerate(("PX", "PY", "PZ")))
        check(ok_geo, "CA_ROTORn_PX/PY/PZ = the file's hubs about the wet CG, FLU -> FRD")
        ok_ct = all(abs(p[f"CA_ROTOR{i}_CT"] - r_.per_rotor("max_thrust")[i]) < 0.01 for i in range(4))
        ok_km = all(abs(p[f"CA_ROTOR{i}_KM"] - r_.spin[i] * r_.per_rotor("cq")[i]) < 1e-4 for i in range(4))
        check(ok_ct and ok_km, "CA_ROTORn_CT = the pair's static thrust, KM = spin x cq")
        check(p["THR_MDL_FAC"] == 1.0 and p["CA_SV_TL0_MINA"] == 0 and p["CA_SV_TL0_MAXA"] == 90 and p["VT_TILT_MC"] == 0,
              "THR_MDL_FAC 1; PX4's tilt convention MINA 0 / MAXA 90, hover collective 0")
        y = open(os.path.join(td, "autopilot_params.yaml")).read()
        check(f"vehicle_mass: {d['m']:.4f}" in y and f"vehicle_thrust_scaling: {4.0 / d['K']:.6f}" in y
              and "vehicle_idle_thrust: 0.0" in y, f"YAML: mass {d['m']:.4f}, thrust map T / {d['K']:.2f}")
        check(open(os.path.join(td, "env.sh")).read().strip() == "export VTOL_TILT_PX4CA=0,90", "env.sh: VTOL_TILT_PX4CA=0,90")
        # the routing under PX4's own allocation: surfaces are Servo1-3, tilts Servo4/5 (the first flight's 45 deg)
        check([p[f"PWM_MAIN_FUNC{i}"] for i in range(5, 10)] == [204, 205, 201, 202, 203] and p["CA_SV_CS_COUNT"] == 3,
              "outputs: controls[4,5] <- Servo4/5 (tilt0/1), controls[6..8] <- Servo1..3 (surfaces)")
        check((p["MC_YAWRATE_P"], p["MC_YAWRATE_I"], p["MC_YAWRATE_K"]) == (0.25, 0.2, 1.0),
              "PX4's yaw rate loop pinned to 4101's values (SITL saves the last set)")
        kpz = af.tuning.get("fsc_attctl_kp_angle_z")
        check(kpz is not None and f"attctl_kp_angle_z: {kpz:g}   # the airframe file's [tuning]" in y,
              f"the autopilot's yaw angle gain from the file's [tuning] ({kpz})")
    with tempfile.TemporaryDirectory() as td:
        env = {k: v for k, v in os.environ.items() if k != "FSC_AIRFRAME_ID"}
        subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "fsc_autopilot_config.py"), "--tank-kg", "0.502",
                        "--out", td], env=env, capture_output=True, text=True)
        y0 = open(os.path.join(td, "autopilot_params.yaml")).read()
        check("attctl_kp_angle_z: 6.0" in y0, "airframe_id.cfg (no [tuning]): the base YAML's 6.0 stands")
    with tempfile.TemporaryDirectory() as td:

        print("4. the read-back")
        open(os.path.join(td, "px4_params.applied"), "w").write("THR_MDL_FAC 1.0\nCA_ROTOR0_KM 0.0197\n")
        open(os.path.join(td, "px4_console.txt"), "w").write("x + THR_MDL_FAC [1,2] : 1.0000\nx + CA_ROTOR0_KM [3,4] : 0.0197\n")
        ok1 = g.verify(td)
        open(os.path.join(td, "px4_console.txt"), "w").write("x + THR_MDL_FAC [1,2] : 0.0000\nx + CA_ROTOR0_KM [3,4] : 0.0197\n")
        ok2 = g.verify(td)
        check(ok1 and not ok2, "verify passes on a match and refuses THR_MDL_FAC 0 where 1 was applied")

    print("3. the plant's tilt map")
    spec = importlib.util.spec_from_file_location("tpa", os.path.join(HERE, "test_plan_airframe.py"))
    tpa = importlib.util.module_from_spec(spec); spec.loader.exec_module(tpa)
    _, _, vu = tpa.plant_modules()
    # default_quad_tiltrotor_channel_map() (vtol_mavlink_backend.py imports pymavlink + Isaac): its entries
    quad = {"motor_front_L": (0, 1.0, 0.0), "motor_front_R": (1, 1.0, 0.0), "motor_rear_L": (2, 1.0, 0.0),
            "motor_rear_R": (3, 1.0, 0.0), "tilt_left": (4, 1.0, 0.0), "tilt_right": (5, 1.0, 0.0),
            "aileron": (6, 1.0, 0.0), "elevator": (7, 1.0, 0.0), "rudder": (8, 1.0, 0.0)}
    bsrc = open(os.path.join(ROOT, "extensions", "pegasus.simulator", "pegasus", "simulator", "logic", "backends",
                             "vtol_mavlink_backend.py")).read()
    check(all(f'"{k}":' in bsrc and f"({v[0]}, 1.0, 0.0)" in bsrc for k, v in quad.items()),
          "the quad channel map in vtol_mavlink_backend.py is the one used here")
    for path in (PLAN, os.path.join(inner_dir(), "airframe_id.cfg")):
        cfg = vu.airframe_vtol_config(path, cad_meshes=False)[0] if load_airframe(path).has_layout else vu.fsc_cad_vtol_config()
        base = dict(quad)
        same = vu.px4ca_tilt_map(base, cfg, spec="")
        check(same == base, f"{os.path.basename(path)}: unset = the identity map")
        mp = vu.px4ca_tilt_map(base, cfg, spec="0,90")
        tilt_rc = [rc for rc in cfg.rotors if rc.tiltable]
        tmax = tilt_rc[0].tilt_max_deg
        ang = lambda c, ch: (c * mp[ch][1] + mp[ch][2]) * tmax   # noqa: E731 -- vtol.py: angle = cmd x tilt_max
        check(all(abs(ang(-1, rc.tilt_channel)) < 1e-9 and abs(ang(1, rc.tilt_channel) - 90.0) < 1e-9
                  and abs(ang(0, rc.tilt_channel) - 45.0) < 1e-9 for rc in tilt_rc),
              f"{os.path.basename(path)} (travel {tmax:g}): PX4 -1 -> 0 deg, 0 -> 45, +1 -> 90")
        others = {k: v for k, v in mp.items() if k not in [rc.tilt_channel for rc in tilt_rc]}
        check(all(others[k] == base[k] for k in others), "only the tilt channels change")
        check(all(len(mp[rc.tilt_channel]) == 4 and abs(mp[rc.tilt_channel][3] * tmax) < 1e-12 for rc in tilt_rc),
              "a DISARMED (unwritten, 0.0) tilt output -> upright, PX4's hover tilt (the spool-up)")
    # the backend's _map_controls itself (vtol_mavlink_backend imports pymavlink + Isaac: exec the method alone)
    import ast, math as _m, types
    tree = ast.parse(bsrc)
    cls = next(nd for nd in ast.walk(tree) if isinstance(nd, ast.ClassDef) and nd.name == "PX4VTOLMavlinkBackend")
    fn = next(nd for nd in cls.body if isinstance(nd, ast.FunctionDef) and nd.name == "_map_controls")
    win = next(nd.value.value for nd in cls.body if isinstance(nd, ast.Assign) and nd.targets[0].id == "SPOOLUP_WINDOW_S")
    ns = {"math": _m}; exec(compile(ast.Module([fn], []), "mc", "exec"), ns)
    plan_cfg = vu.airframe_vtol_config(PLAN, cad_meshes=False)[0]
    fake = types.SimpleNamespace(_channel_map=vu.px4ca_tilt_map(dict(quad), plan_cfg, spec="0,90"), SPOOLUP_WINDOW_S=win)
    ctl = [0.7, 0.7, 0.7, 0.7, 0.0, -1.0, 0.0, 0.0, 0.0]
    early = ns["_map_controls"](fake, ctl, 0.5); late = ns["_map_controls"](fake, ctl, win + 1.0)
    check(abs(early["tilt_left"]) < 1e-12 and abs(late["tilt_left"] * 93.0 - 45.0) < 1e-9 and abs(late["tilt_right"]) < 1e-12,
          f"0.0 is the DISARMED servo only in the spool-up window ({win:g} s: upright), after it a COMMAND "
          f"(45 deg -- PX4 caps the yaw tilt at exactly 0)")
    ident = ns["_map_controls"](types.SimpleNamespace(_channel_map=dict(quad), SPOOLUP_WINDOW_S=win), ctl, 0.5)
    check(ident["tilt_left"] == 0.0 and ident["tilt_right"] == -1.0 and ident["motor_front_L"] == 0.7,
          "3-tuple entries (direct actuator) map exactly as before, inside the window too")

    print("5. the launcher")
    s = open(os.path.join(ROOT, "scripts", "start_vtol_fsc_hil.sh")).read()
    check('rootfs_fsc0' in s and 'rootfs_vtol0"' not in s.split("PX4_WORKDIR=")[1].split("\n")[0],
          "its own PX4 working dir (rootfs_fsc0), not the direct-actuator launchers' rootfs_vtol0")
    isaac = s.split('-n "2-isaac"')[1].split('tmux new-window')[0]
    check(all(k in isaac for k in ("FSC_AIRFRAME_ID=", "VTOL_TILT_PX4CA=", "VTOL_SLOSH=", "VTOL_TANK_KG=", "VTOL_TANK_DZ=")),
          "Isaac gets the airframe file, the tilt map and the tank")
    check('fsc_autopilot_config.py" \\\n    --airframe "$AF_FILE"' in s and "AUTOPILOT_PARAMS:-$FSC_CFG_DIR/autopilot_params.yaml" in s,
          "the config is generated from the same file (AF_FILE) and the autopilot flies its YAML")
    check("--verify" in s and "px4_params.verified" in s, "the PX4 read-back gate")

    print(f"\n{'PASS' if not bad else 'FAIL'}" + ("" if not bad else f" -- {len(bad)}: " + "; ".join(bad)))
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())

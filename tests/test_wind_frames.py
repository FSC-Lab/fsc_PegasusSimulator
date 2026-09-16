#!/usr/bin/env python3
"""Airspeed and wind (hardware-transfer piece 4, 2026-09-15): the frames, the signs, the pitot, the
wind estimator, and the gate's identity without wind.

  1. the sim's aero model with `wind_body`: a headwind on a vehicle at rest makes DRAG (aft) and
     lift (up) exactly as the same airspeed flown through still air does; a tailwind of the same
     size as the ground speed makes nothing (v_air = 0);
  2. the gate plant (so3.build_ode(wind=True)): w = 0 is bit-identical to the two-input Function;
     a headwind at cruise adds drag and lift; a tailwind removes them;
  3. the synthesised pitot (vtol_mavlink_backend): reads |v - w| along body x -- a 5 m/s headwind on
     a vehicle at rest is 5 m/s of airspeed; yawed 90 deg, a world +x wind is a crosswind and the
     pitot reads zero; no wind on the state = the flown behaviour;
  4. WindEstimator (the controller): no reading -> the inertial quantities bit for bit; fed TAS =
     |v - w| it converges to the along-course wind (a headwind is negative w_s), V_air = |v - w|;
     below v_min it is FROZEN (a stop in wind keeps the estimate); stale readings are ignored;
  5. the gate: `--wind 0,0,0` == flags absent (trace byte-identical); `--wind -5,0,0` changes it.
    ~/envs/isaac_env/bin/python3 tests/test_wind_frames.py      (~2 min for section 5)
"""
import hashlib
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..")
sys.path.insert(0, HERE); sys.path.insert(0, ROOT)
from test_slosh_import import stub_isaac, load          # noqa: E402
from controller_path import controller_dir, inner_dir   # noqa: E402

PY = os.path.expanduser("~/envs/isaac_env/bin/python3")
fails = 0


def check(cond, msg):
    global fails
    print(("PASS " if cond else "FAIL ") + msg); fails += 0 if cond else 1


def main():
    stub_isaac()
    aero_mod = load("pegasus.simulator.logic.dynamics.vtol_aerodynamics", "dynamics/vtol_aerodynamics.py")
    load("pegasus.simulator.logic.rotations", "rotations.py")
    state_mod = load("pegasus.simulator.logic.state", "state.py")

    # ---- 1. the sim's aero model with wind_body ---------------------------------------------
    print("\n1. vtol_aerodynamics.update(wind_body=)")
    aero = aero_mod.VTOLAerodynamics(aero_mod.VTOLAerodynamicsConfig())
    st = state_mod.State()
    st.linear_body_velocity = np.zeros(3); st.angular_velocity = np.zeros(3)
    F0, M0 = aero.update(st, None)
    Fh, Mh = aero.update(st, None, wind_body=np.array([-12.0, 0.0, 0.0]))      # air comes from the front
    st2 = state_mod.State(); st2.linear_body_velocity = np.array([12.0, 0.0, 0.0]); st2.angular_velocity = np.zeros(3)
    Ff, Mf = aero.update(st2, None)                                              # the same airspeed, flown
    check(np.allclose(F0, 0.0) and np.allclose(M0, 0.0), "at rest in still air: no aero force")
    check(np.allclose(Fh, Ff, atol=1e-9) and np.allclose(Mh, Mf, atol=1e-9),
          f"12 m/s headwind at rest == 12 m/s flown through still air: F=({Fh[0]:+.2f},{Fh[1]:+.2f},{Fh[2]:+.2f}) N")
    check(Fh[0] < -0.5 and Fh[2] > 0.5, "headwind: drag aft (Fx < 0) and lift up (Fz > 0) in FLU")
    Ft, _ = aero.update(st2, None, wind_body=np.array([12.0, 0.0, 0.0]))       # tailwind equal to the ground speed
    check(np.allclose(Ft, 0.0), "12 m/s ground speed in a 12 m/s tailwind: v_air = 0, no aero force")

    # ---- 2. the gate plant with the wind input ------------------------------------------------
    print("\n2. so3.build_ode(wind=True)")
    sys.path.insert(0, inner_dir())
    import nmpc_so3_transition_node as so3
    F = so3.build_ode(); Fw = so3.build_ode(wind=True)
    x = np.zeros(19); x[3:12] = np.eye(3).reshape(-1); x[2] = 30.0; x[13] = 16.0
    u = np.array([3.0, 3.0, 1.0, 1.0, math.pi / 2, 0.0, 0.0, 0.0])
    a = np.array(F(x, u)).ravel(); b = np.array(Fw(x, u, np.zeros(3))).ravel()
    check(np.array_equal(a, b), "wind=True at w=0 is bit-identical to the two-input plant")
    check(F.serialize() == so3.build_ode().serialize(), "build_ode() without wind serialises as before")
    h = np.array(Fw(x, u, np.array([-5.0, 0.0, 0.0]))).ravel(); t = np.array(Fw(x, u, np.array([5.0, 0.0, 0.0]))).ravel()
    check(h[13] < a[13] and h[15] > a[15], f"5 m/s headwind at 16 m/s: more drag (vsdot {a[13]:+.2f} -> {h[13]:+.2f}) and more lift (vzdot {a[15]:+.2f} -> {h[15]:+.2f})")
    check(t[13] > a[13] and t[15] < a[15], f"5 m/s tailwind: less drag ({t[13]:+.2f}) and less lift ({t[15]:+.2f})")

    # ---- 3. the synthesised pitot -------------------------------------------------------------
    print("\n3. the mavlink backend's pitot reads the air")
    src = open(os.path.join(ROOT, "extensions", "pegasus.simulator", "pegasus", "simulator", "logic", "backends", "vtol_mavlink_backend.py")).read()
    check('getattr(state, "wind_world", None)' in src and "v_fwd -= float(w_body_flu[0])" in src,
          "update_state subtracts the state's world wind (body x) from the pitot's forward speed")
    # replicate its arithmetic against the State's own conversions (the backend needs pymavlink to import)
    from scipy.spatial.transform import Rotation
    stp = state_mod.State(); stp.linear_velocity = np.zeros(3); stp.linear_body_velocity = np.zeros(3)
    stp.attitude = np.array([0.0, 0.0, 0.0, 1.0])

    def pitot(state, wind):
        v_fwd = float(state.get_linear_body_velocity_ned_frd()[0])
        if wind is not None and np.any(np.asarray(wind) != 0.0):
            v_fwd -= float(Rotation.from_quat(state.attitude).inv().apply(np.asarray(wind, float))[0])
        return v_fwd
    check(abs(pitot(stp, None)) < 1e-12 and abs(pitot(stp, np.zeros(3))) < 1e-12, "at rest, no wind: 0 m/s (the flown behaviour)")
    stv = state_mod.State(); stv.attitude = np.array([0.0, 0.0, 0.0, 1.0])
    stv.linear_velocity = np.array([12.0, 0.0, 0.0]); stv.linear_body_velocity = np.array([12.0, 0.0, 0.0]); stv.linear_acceleration = np.array([0.5, 0.0, 0.0])
    check(abs(pitot(stv, None) - 12.0) < 1e-9, "FLYING at 12 m/s in still air: the pitot reads the body VELOCITY, not the acceleration (the upstream State bug that read 0.0 at 16 m/s on every flight)")
    check(abs(pitot(stp, np.array([-5.0, 0.0, 0.0])) - 5.0) < 1e-9, "at rest in a 5 m/s headwind (wind -x, nose +x): 5 m/s")
    stp.attitude = Rotation.from_euler("z", 90, degrees=True).as_quat()        # nose along world +y
    check(abs(pitot(stp, np.array([-5.0, 0.0, 0.0]))) < 1e-9, "yawed 90 deg the same wind is a crosswind: 0 m/s")
    check(abs(pitot(stp, np.array([0.0, -5.0, 0.0])) - 5.0) < 1e-9, "and a world -y wind is its headwind: 5 m/s")

    # ---- 4. the controller's wind estimator ---------------------------------------------------
    print("\n4. WindEstimator")
    import nmpc_cascade_node as base
    est = base.WindEstimator(tau=2.0, tas_min=2.0)
    v = np.array([16.0, 0.0, 0.0]); course = np.array([1.0, 0.0, 0.0])
    V, va, ws = est.update(v, course, 0.005, 0.0)
    check(V == 16.0 and va is v and ws == 0.0, "no reading ever: returns the inertial velocity object itself and w_s = 0")
    w_true = np.array([-5.0, 0.0, 0.0])                                        # headwind
    t = 0.0
    for k in range(4000):                                                      # 20 s at 200 Hz, pitot at 20 Hz
        if k % 10 == 0:
            est.observe(float(np.linalg.norm(v - w_true)), t)
        V, va, ws = est.update(v, course, 0.005, t); t += 0.005
    check(abs(ws + 5.0) < 0.02 and abs(V - 21.0) < 0.02 and np.allclose(va, v - w_true, atol=0.02),
          f"5 m/s headwind: w_s {ws:+.2f} (tailwind sign), V_air {V:.2f}, v_air ({va[0]:.2f},{va[1]:.2f},{va[2]:.2f})")
    w_keep = est.w.copy(); n_keep = est.n
    est.observe(0.4, t)                                                        # a pitot in hover: noise
    V, va, ws = est.update(np.array([0.0, 0.0, 0.0]), course, 0.005, t)
    check(np.array_equal(est.w, w_keep) and est.n == n_keep and abs(ws + 5.0) < 0.02 and abs(V - 5.0) < 0.02,
          f"below tas_min the estimate is frozen: a stop in the wind still reads V_air {V:.2f} on 0 m/s of ground speed")
    esth = base.WindEstimator(tau=2.0, tas_min=2.0); t2 = 0.0
    for k in range(4000):                                                      # hovering INTO a 5 m/s wind from the start
        if k % 10 == 0:
            esth.observe(5.0, t2)
        V, va, ws = esth.update(np.zeros(3), course, 0.005, t2); t2 += 0.005
    check(abs(ws + 5.0) < 0.02 and abs(V - 5.0) < 0.02, f"a hover in a 5 m/s headwind: the pitot sees it (w_s {ws:+.2f}, V_air {V:.2f})")
    estc = base.WindEstimator(tau=2.0, tas_min=2.0); t2 = 0.0
    for k in range(4000):                                                      # a pure crosswind: the probe cannot see it
        if k % 10 == 0:
            estc.observe(16.0, t2)
        V, va, ws = estc.update(v, course, 0.005, t2); t2 += 0.005
    check(abs(ws) < 0.02 and abs(V - 16.0) < 0.02, "a pure crosswind reads as no wind along the probe (unobservable, by design)")
    est2 = base.WindEstimator(tau=2.0, tas_min=2.0)
    est2.observe(21.0, 0.0)
    V, va, ws = est2.update(v, course, 0.005, 1.0)
    check(est2.n == 0 and ws == 0.0, "a reading older than stale_s is ignored")
    est3 = base.WindEstimator(tau=2.0, tas_min=2.0)
    est3.observe(float("nan"), 0.0)
    check(est3.tas is None, "a NaN reading (PX4's 'invalid') is not a reading")

    # ---- 5. the gate: neutral == absent, wind moves it --------------------------------------
    print("\n5. the gate")
    node = os.path.join(inner_dir(), "nmpc_cascade_steso_node.py")
    base_args = ["--test", "--est", "mdob", "--ttr", "10", "--lat", "0.03", "--comp", "0.049", "--outer_lag", "0.03",
                 "--slosh", "--noise_att", "0.1", "--noise_rate", "1.0", "--l1", "--dur", "6", "--every", "1.0", "--seed", "3"]
    tmp = os.environ.get("TMPDIR", "/tmp")

    def run(tag, extra):
        log = os.path.join(tmp, f"wind_{tag}.csv")
        r = subprocess.run([PY, node] + extra + base_args + ["--log", log], capture_output=True, text=True, timeout=900,
                           cwd=os.path.dirname(inner_dir()))
        if r.returncode != 0:
            print(r.stdout[-600:], r.stderr[-600:]); return None, None
        rows = open(log).read().split("\n"); h = rows[0].split(","); keep = [i for i, c in enumerate(h) if "ms" not in c]
        body = [[x.split(",")[i] for i in keep] for x in rows[1:] if x]
        return hashlib.md5("\n".join(",".join(r_) for r_ in body).encode()).hexdigest(), np.array(body, float)
    h0, a0 = run("plain", []); h1, _ = run("zero", ["--wind", "0,0,0"]); h2, _ = run("head", ["--wind", "-5,0,0"]); h3, a3 = run("pitot", ["--airspeed", "--noise_aspd", "0"])
    check(h0 is not None and h0 == h1, "--wind 0,0,0 == flags absent (trace byte-identical)")
    check(h0 is not None and h2 is not None and h2 != h0, "--wind -5,0,0 changes the trace")
    # a noiseless pitot in still air reads the PLANT's body-x velocity while the controller projects
    # its NOISY attitude (0.1 deg) -- a phantom wind of a few mm/s, low-passed: the aircraft-level
    # trace (roll, pitch, u_ad) must agree to the noise floor (0.1 deg / 0.01 N.m), not to the bit
    ok5 = a0 is not None and a3 is not None and a0.shape == a3.shape
    if ok5:
        h = open(os.path.join(tmp, "wind_plain.csv")).readline().strip().split(",")
        col = {c: [i for i, cc in enumerate(h) if "ms" not in cc].index(h.index(c)) for c in ("roll_deg", "pitch_deg", "u_x", "u_y", "u_z")}
        dev = {c: float(np.abs(a0[:, i] - a3[:, i]).max()) for c, i in col.items()}
        ok5 = dev["roll_deg"] < 0.1 and dev["pitch_deg"] < 0.1 and max(dev["u_x"], dev["u_y"], dev["u_z"]) < 0.01
        print(f"     pitot in still air vs none: roll {dev['roll_deg']:.3f} deg, pitch {dev['pitch_deg']:.3f} deg, u_ad {max(dev['u_x'], dev['u_y'], dev['u_z']):.4f} N.m")
    check(ok5, "--airspeed with a noiseless pitot in still air == flags absent at the aircraft level (roll/pitch < 0.1 deg, u_ad < 0.01 N.m)")

    print()
    print("ALL PASS" if fails == 0 else f"{fails} FAILED")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

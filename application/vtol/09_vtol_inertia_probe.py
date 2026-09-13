"""Measure what PhysX actually flies: the airframe's rotational inertia about each body axis,
and the yaw torque a rotor-speed change produces beyond the modelled reaction torque.

    PYTHONEXE=... $ISAAC_PY application/vtol/09_vtol_inertia_probe.py [--slosh]

WHY. Fitting the 2026-09-12 HIL hover bags (yaw rate against the controller's own moment
model) gave an effective yaw inertia of 0.07-0.11 kg m^2 against the controller's assembled
0.49, and a pitch inertia of 0.40-0.50 against 0.303. The offline gate integrates the
controller's numbers as "the plant", so it cannot see either. This probe asks PhysX directly:

  1. props NOT spinning, gravity off, forces off: apply a pure torque about each axis for
     `PULSE_S` seconds and read the angular-velocity change -> I_axis = M * dt / d_omega.
  2. props spinning at hover, rotor_moment_ratio = 0 (no modelled reaction torque): step the
     yaw diagonal (FL+RR up, FR+RL down) and read the yaw-rate impulse -> the DRIVE reaction
     torque of the spinning prop links, which no controller models.

  3. a front/rear throttle step from a balanced open-loop hover -> pitch acceleration vs the
     model. SANITY CHECK ONLY: nothing holds attitude, so the aircraft drifts to 2-4 m/s within
     the spin-up and the wing's static stability eats ~30% of the step (three earlier versions
     of this experiment measured the aeroplane at 11-14 m/s and reported a 3x torque deficit
     that did not exist). 1.0 N up at x = +1.0 / +0.459 / -0.112 m (`--force_pos`) gives the
     assembled-CG arms to the millimetre -- the force->torque path is exact.

RESULT (2026-09-13): inertia 0.265/0.303/0.490 -- ratio 1.00 on every axis; the yaw step with
the modelled reaction OFF yawed at 17 rad/s^2 (8.3 N.m) until the prop links' PhysX angular-
velocity cap (100 rad/s default) was lifted, after which it is a -5 deg/s impulse.
Headless; ~1 min including startup. `--slosh`, `--instant`, `--no_spin`, `--g_off`.
"""
import math
import os
import sys
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np                                   # noqa: E402
import omni.timeline                                 # noqa: E402
from omni.isaac.core.world import World              # noqa: E402

from pegasus.simulator.logic.backends.backend import Backend            # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS            # noqa: E402
from fsc_aerial_manipulation.rotorcraft import fsc_cad_vtol_config      # noqa: E402

OUT = os.environ.get("PROBE_OUT", "/tmp/vtol_inertia_probe.txt")


def say(msg):
    """stdout is swallowed by the kit app on close; write to stderr AND a file."""
    sys.stderr.write(msg + "\n"); sys.stderr.flush()
    with open(OUT, "a") as f:
        f.write(msg + "\n")


PULSE_S = 0.10
TORQUE = 1.0            # N m
CTRL = {"Ixx": 0.265, "Iyy": 0.303, "Izz": 0.490}   # the controllers' assembled inertia
SLOSH = "--slosh" in sys.argv


class CmdBackend(Backend):
    """Feeds whatever command dict the probe sets; no ROS, no PX4."""

    def __init__(self):
        super().__init__(None)
        self.cmd = {}

    def initialize(self, vehicle):
        self._vehicle = vehicle

    def update_state(self, state):
        pass

    def update_sensor(self, sensor_type, data):
        pass

    def update_graphical_sensor(self, sensor_type, data):
        pass

    def input_reference(self):
        return self.cmd

    def update(self, dt):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def reset(self):
        pass


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    timeline = omni.timeline.get_timeline_interface()
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    simulation_app.update()

    cfg = fsc_cad_vtol_config()
    cfg.enable_forces = False
    cfg.disable_gravity = False           # hover experiments run FIRST from the level spawn (a true
                                          # hover: a climb into the prop's own inflow zeroes a momentum
                                          # prop, and a residual attitude turns a hover into a cruise --
                                          # both fooled earlier versions of this probe); gravity is
                                          # switched OFF afterwards for the rigid-body torque pulses
    cfg.rotor_moment_ratio = 0.0          # experiment 2 wants ONLY the drive reaction
    if "--instant" in sys.argv:
        cfg.thrust_model = "instant"      # A/B for experiment 3: flat, lag-free thrust
    if "--no_spin" in sys.argv:
        cfg.spin_drive_damping = 0.0      # A/B: thrust applied, prop LINKS not spun
    NO_AERO = "--no_aero" in sys.argv
    KEEP_G_OFF = "--g_off" in sys.argv   # A/B: hover experiments with gravity off (instant model only)
    be = CmdBackend()
    cfg.backends = [be]
    if SLOSH:
        from pegasus.simulator.logic.vehicles.vtol_slosh import VTOLSlosh, VTOLSloshConfig
        from pegasus.simulator.logic.dynamics.slosh import TankConfig
        from dataclasses import fields
        scfg = VTOLSloshConfig(**{f.name: getattr(cfg, f.name) for f in fields(cfg)}, tank=TankConfig())
        veh = VTOLSlosh(vehicle_id=0, init_pos=(0.0, 0.0, 40.0), config=scfg)
    else:
        from pegasus.simulator.logic.vehicles.vtol import VTOL
        veh = VTOL(vehicle_id=0, init_pos=(0.0, 0.0, 40.0), config=cfg)
    world.reset()
    timeline.play()
    dt = pg._world_settings["physics_dt"]
    say(f"[probe] physics_dt={dt:.4f}  thrust_model={getattr(veh._cfg, 'thrust_model', '?')}  "
          f"omega_max={veh._cfg.rotor_omega_max}  tau_spin={veh._cfg.rotor_tau_spin}  "
          f"config inertia={veh._cfg.inertia}  mass={veh._cfg.mass}  total_mass~{veh.total_mass:.3f}")

    def settle(n):
        for _ in range(n):
            world.step(render=False)

    settle(60)

    if NO_AERO:
        veh._apply_aerodynamics = lambda command: None
        say("[probe] aero DISABLED for the hover experiments")
    results = {}
    # ---- 2. drive reaction torque of the spinning props (no modelled reaction) ------------
    veh._cfg.enable_forces = True
    # exact hover throttle for THIS mass on the quadratic map: T = Tmax*thr^2 per rotor
    hover = (veh.total_mass * 9.81 / (4.0 * 18.0)) if getattr(veh._cfg, "thrust_model", "momentum") == "instant" \
        else math.sqrt(veh.total_mass * 9.81 / (4.0 * 18.0))
    # ★ BALANCE THE HOVER. Equal throttles leave 2T(0.283 - 0.288) = -0.08 N.m about the
    # assembled CG, and with nothing holding attitude that tips an open-loop hover 30 deg
    # nose-down in 3 s and sends it off at 14 m/s -- which turned every earlier version of the
    # pitch experiment into a measurement of the WING's static stability. Split front/rear
    # so the thrust moment is zero, and keep the spin-up short.
    xf, xr = 0.459 - 0.1764, -0.112 - 0.1764
    ratio = -xr / xf                            # T_front / T_rear for zero moment
    quad = getattr(veh._cfg, "thrust_model", "momentum") != "instant"
    kf = (ratio ** 0.5 if quad else ratio)
    hf = hover * (2.0 / (1.0 + 1.0 / kf)) ** (0.5 if quad else 1.0) if False else hover
    # solve: front^p * xf + rear^p * xr = 0 and front^p + rear^p = 2*hover^p  (p = 2 quad, 1 flat)
    pw = 2.0 if quad else 1.0
    rear_p = 2.0 * hover ** pw / (1.0 + ratio); front_p = ratio * rear_p
    hf, hr = front_p ** (1.0 / pw), rear_p ** (1.0 / pw)
    be.cmd = {"motor_front_L": hf, "motor_front_R": hf, "motor_rear_L": hr, "motor_rear_R": hr,
              "tilt_left": 0.0, "tilt_right": 0.0, "aileron": 0.0, "elevator": 0.0, "rudder": 0.0}
    say(f"[probe] balanced hover throttles front {hf:.4f} rear {hr:.4f}")
    settle(int(1.5 / dt))                       # spin up (tau 0.12 s -> 12 tau) and settle briefly
    say(f"[probe] after spin-up: z {veh.state.position[2]:.1f} m, attitude q {np.round(veh.state.attitude, 3)}")
    # what the props ACTUALLY spin at vs what the model thinks they spin at
    dc = veh.get_dc_interface(); art = dc.get_articulation(veh._stage_prefix)
    for rotor in veh._rotors:
        dof = dc.find_articulation_dof(art, rotor.spin_joint)
        actual = dc.get_dof_velocity(dof) if dof else float("nan")
        say(f"[probe] {rotor.name}: modelled omega {veh._omega[rotor.name]:7.1f} rad/s, "
            f"PhysX prop DOF velocity {actual:+8.1f} rad/s")
    say(f"[probe] hover throttle {hover:.3f}; vertical speed at the yaw step {veh.state.linear_body_velocity[2]:+.2f} m/s (must be ~0)")
    w0 = np.array(veh.state.angular_velocity, float)
    d = 0.20
    be.cmd.update({"motor_front_L": hf + d, "motor_rear_R": hr + d, "motor_front_R": hf - d, "motor_rear_L": hr - d})
    n = int(round(0.30 / dt))
    peak = 0.0; trace = []
    for k in range(n):
        world.step(render=False)
        w = np.array(veh.state.angular_velocity, float)
        trace.append((k * dt, float(w[2] - w0[2])))
        peak = max(peak, abs(float(w[2] - w0[2])))
    say(f"[probe] yaw-diagonal step +-{d:.2f} throttle at hover {hover:.2f}, rotor_moment_ratio=0: "
          f"yaw rate change over 0.30 s: " + " ".join(f"{t:.2f}s:{np.degrees(v):+.1f}" for t, v in trace[::int(0.05 / dt)]) + " deg/s")
    # impulse of the drive reaction: I_zz * d_omega = integral(M dt); a first-order omega step of
    # d_omega_prop = d*omega_max per prop (4 props, diagonal adds) gives J_prop*4*d*omega_max total
    Izz = CTRL["Izz"]
    w_end = trace[-1][1]
    J_est = Izz * w_end / (4.0 * d * veh._cfg.rotor_omega_max) if veh._cfg.rotor_omega_max else float("nan")
    say(f"[probe] implied per-prop spin inertia from the yaw impulse: {J_est:.2e} kg m^2 "
          f"(config link_inertia {veh._cfg.link_inertia})  [uses measured Izz {Izz:.3f}]")

    # ---- 3. pitch response to a front/rear differential vs the model ---------------------
    be.cmd.update({"motor_front_L": hf, "motor_front_R": hf, "motor_rear_L": hr, "motor_rear_R": hr})
    settle(int(1.0 / dt))
    w0 = np.array(veh.state.angular_velocity, float)
    vb0 = np.array(veh.state.linear_body_velocity, float)
    say(f"[probe] at the pitch step: v_body ({vb0[0]:+.2f},{vb0[1]:+.2f},{vb0[2]:+.2f}) m/s, attitude q {np.round(veh.state.attitude, 3)}, rates {np.degrees(w0).round(1)} deg/s")
    dp = 0.10
    be.cmd.update({"motor_front_L": hf + dp, "motor_front_R": hf + dp, "motor_rear_L": hr - dp, "motor_rear_R": hr - dp})
    n = int(round(0.30 / dt)); tr = []
    for k in range(n):
        world.step(render=False)
        w = np.array(veh.state.angular_velocity, float)
        tr.append((k * dt, float(w[1] - w0[1])))
        if k % int(0.05 / dt) == 0:
            st = veh._aero.last_state; vb = np.array(veh.state.linear_body_velocity, float)
            say(f"[probe]   t={k*dt:.2f} q={np.degrees(w[1]):+6.1f} deg/s  v_body=({vb[0]:+.2f},{vb[1]:+.2f},{vb[2]:+.2f})  "
                f"aero: V={getattr(st,'airspeed',float('nan')):.2f} alpha={np.degrees(getattr(st,'alpha',float('nan'))):+.1f} "
                f"moment={np.round(np.array(getattr(st,'moment',[0,0,0]),float),3)}  force={np.round(np.array(getattr(st,'force',[0,0,0]),float),2)}")
    cg = 0.1764                                   # the assembled CG the controllers use
    arms = {r.name: float(r.position[0]) - cg for r in veh._rotors}
    Iyy = CTRL["Iyy"]
    say(f"[probe] pitch: front +{dp:.2f} / rear -{dp:.2f} throttle step at hover {hover:.2f}: "
        f"pitch rate change over 0.30 s: " + " ".join(f"{t:.2f}s:{np.degrees(v):+.1f}" for t, v in tr[::int(0.05 / dt)]) + " deg/s")
    if getattr(veh._cfg, "thrust_model", "momentum") == "instant":
        Tf = 18.0 * (hf + dp); Trr = 18.0 * (hr - dp)
    else:
        Tf = 18.0 * (hf + dp) ** 2; Trr = 18.0 * (hr - dp) ** 2
    My = 2 * Tf * arms["front_L"] + 2 * Trr * arms["rear_L"]
    a_meas = (tr[-1][1] - tr[-int(0.1 / dt)][1]) / 0.1
    say(f"[probe] pitch: modelled steady torque {abs(My):.2f} N m (arms {arms['front_L']:+.3f}/{arms['rear_L']:+.3f} m) -> "
        f"expected {abs(My) / Iyy:.2f} rad/s^2 at Iyy {Iyy:.3f}; measured {abs(a_meas):.2f} rad/s^2 "
        f"-> I_eff {abs(My) / max(abs(a_meas), 1e-9):.3f}  (ratio to model {abs(My) / max(abs(a_meas), 1e-9) / Iyy:.2f})")
    # ---- gravity OFF, thrust off, props stopped: the rigid-body pulses -------------------
    veh._cfg.enable_forces = False
    be.cmd = {}
    from pxr import PhysxSchema, Usd
    stage = veh._current_stage
    for prim in Usd.PrimRange(stage.GetPrimAtPath(veh._stage_prefix)):
        if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI(prim).GetDisableGravityAttr().Set(True)
    settle(int(2.0 / dt))
    # ---- 1. rigid-body inertia about each axis (props still) ----------------------------
    results = {}
    for axis, name in ((0, "Ixx"), (1, "Iyy"), (2, "Izz")):
        settle(30)
        w0 = np.array(veh.state.angular_velocity, float)
        n = int(round(PULSE_S / dt))
        tq = [0.0, 0.0, 0.0]; tq[axis] = TORQUE
        for _ in range(n):
            veh.apply_torque(tq, body_part="/body")
            world.step(render=False)
        w1 = np.array(veh.state.angular_velocity, float)
        dw = w1 - w0
        I = TORQUE * n * dt / dw[axis] if abs(dw[axis]) > 1e-9 else float("nan")
        results[name] = I
        say(f"[probe] {name}: {TORQUE:.1f} N m x {n*dt:.3f} s -> d_omega {np.degrees(dw).round(2)} deg/s "
              f"-> I_eff = {I:.4f} kg m^2   (controller {CTRL[name]:.3f}, ratio {I/CTRL[name]:.2f})")
        # cancel the rate again
        for _ in range(n):
            veh.apply_torque([-v for v in tq], body_part="/body")
            world.step(render=False)

    # ---- 1b. force-at-position: does apply_force(pos=) make the torque r x F? --------------
    # 1 N up at 1 m ahead of the BODY ORIGIN for 0.1 s -> about the assembled CG that is
    # (1 - 0.1764) N m of nose-up pitch torque -> d_omega_y = M*dt/Iyy.
    for px in ((1.0, 0.459, -0.112) if "--force_pos" in sys.argv else ()):
        settle(30)
        w0 = np.array(veh.state.angular_velocity, float); n = int(round(PULSE_S / dt))
        for _ in range(n):
            veh.apply_force([0.0, 0.0, 1.0], pos=[px, 0.0, 0.0], body_part="/body")
            world.step(render=False)
        w1 = np.array(veh.state.angular_velocity, float); dw = w1 - w0
        arm_cg = px - 0.1764
        expect = 1.0 * arm_cg * n * dt / results["Iyy"]
        say(f"[probe] 1 N up at x={px:+.3f} m (body origin frame): d_omega_y {np.degrees(dw[1]):+.3f} deg/s; "
            f"expected {np.degrees(-expect):+.3f} about the assembled CG (arm {arm_cg:+.3f}) or "
            f"{np.degrees(-(px - 0.195) * n * dt / results['Iyy']):+.3f} about the body COM (arm {px - 0.195:+.3f}); "
            f"implied arm {-dw[1] * results['Iyy'] / (n * dt):+.3f} m")
        for _ in range(n):
            veh.apply_force([0.0, 0.0, -1.0], pos=[px, 0.0, 0.0], body_part="/body")
            world.step(render=False)

    # ---- 1c. a SUSTAINED pure torque: does the rate keep growing linearly? -------------------
    # (props still, no thrust, gravity off, command dict empty vs populated)
    for label, cmd in (("empty command", {}), ("hover command (drives targeted)", {
            "motor_front_L": 0.0, "motor_front_R": 0.0, "motor_rear_L": 0.0, "motor_rear_R": 0.0,
            "tilt_left": 0.0, "tilt_right": 0.0, "aileron": 0.0, "elevator": 0.0, "rudder": 0.0})):
        be.cmd = cmd
        settle(30)
        w0 = np.array(veh.state.angular_velocity, float); n = int(round(0.30 / dt)); tr = []
        for k in range(n):
            veh.apply_torque([0.0, 2.0, 0.0], body_part="/body")
            world.step(render=False)
            tr.append((k * dt, float(np.array(veh.state.angular_velocity, float)[1] - w0[1])))
        say(f"[probe] sustained 2 N m pitch torque, {label}: rate " +
            " ".join(f"{t:.2f}s:{np.degrees(v):+.1f}" for t, v in tr[::int(0.05 / dt)]) +
            f" deg/s  (constant 2/{results['Iyy']:.3f} = {2 / results['Iyy']:.2f} rad/s^2 would give {np.degrees(2 / results['Iyy'] * 0.3):+.1f} at 0.30 s)")
        for k in range(n):
            veh.apply_torque([0.0, -2.0, 0.0], body_part="/body")
            world.step(render=False)
    be.cmd = {}


    say("[probe] VERDICT: " + ", ".join(f"{k} {v:.3f}/{CTRL[k]:.3f}" for k, v in results.items()))
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()

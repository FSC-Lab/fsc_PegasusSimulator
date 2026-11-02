#!/usr/bin/env python
"""
| File: jgcd_x650_winch_direct.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2026, Longhao Qian. All rights reserved.
| Description: Isaac Sim plant for testing the JGCD paper's controller ("Robust Neural
|   Contraction Slung Load Manipulation With Active Cable Length Control",
|   ~/source/JGCD-paper-longhao, controller node in ~/source/JGCD2025-sim/ros/) over ROS 2.
|   No PX4: the external controller commands the four rotor speeds and the winch torque
|   directly through JGCDBridgeBackend (utils/jgcd_bridge_backend_utils.py).

Plant, matched to the paper's nominal model (JGCD2025-sim/jgcd_sim/params.py):
  * X650 airframe (x650_new.usd, MN4014 + 15x5" thrust curve, bench rotor lag 10.51 1/s) with
    0.3 kg of winch added to the body, so the drone side totals 3.8 kg (including rod_a);
  * point-mass-like payload: 0.58 kg brick + 0.02 kg rod_b = 0.6 kg on the payload side;
  * cable: two 0.2 m rods joined by a prismatic joint, length l = 0.4 + e, e in [0, 1.6] m, so
    l covers the planners' [L_MIN, L_MAX] = [0.4, 2.0] m; spherical joints at the drone body
    origin and at the payload centre;
  * AK40-10 drum emulated in the bridge (reflected mass 0.97 kg, friction, torque limits) behind
    an elastic cable (k = 1000 N/m);
  * Pegasus' default linear body drag stays on (an unmodelled disturbance for the controller).

Timing:
  * physics at JGCD_PHYSICS_HZ (default 250 Hz), no rendering when headless;
  * the loop is paced to real time (JGCD_RTF, default 1.0) and the achieved real-time factor is
    logged per second of simulation and summarised at exit;
  * JGCD_LOCKSTEP=1 instead waits after every step for the controller's reply to that step's
    state (deterministic, not real time) -- the reference the real-time runs are compared with.

Environment: PEGASUS_HEADLESS (default 1), JGCD_DURATION [s] (default 20), JGCD_L0 [m]
(default 1.0), JGCD_LOG (npz path for the truth log, default /tmp/jgcd_isaac.npz),
JGCD_WAIT_CONTROLLER [s] (default 120: wait for a subscriber on jgcd/state before playing).
"""
import os
import time

import carb
from isaacsim import SimulationApp

_headless = os.environ.get("PEGASUS_HEADLESS", "1").lower() in ("1", "true", "yes")
simulation_app = SimulationApp({"headless": _headless})

import numpy as np
import omni.timeline
import omni.usd
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from fsc_aerial_manipulation.rotorcraft.lagged_thrust_curve import LaggedQuadraticThrustCurve
from fsc_aerial_manipulation.rotorcraft import x650_params
from fsc_aerial_manipulation.rotorcraft.x650_bare_frame_utils import (
    X650_USD, X650_BODY_MASS, apply_x650_mass_properties)
from fsc_aerial_manipulation.utils import add_dome_lighting
from fsc_aerial_manipulation.utils.jgcd_bridge_backend_utils import JGCDBridgeBackend, STATE_FIELDS
import fsc_aerial_manipulation.slung_load as sl
import fsc_aerial_manipulation.constraints as con

G = 9.81
M_DRONE, M_PAYLOAD = 3.8, 0.6
ROD_MASS, ROD_LEN = 0.02, 0.2           # each rod; l = 2 ROD_LEN + e
L_MIN, L_MAX = 0.4, 2.0
WINCH_MASS = M_DRONE - x650_params.MASS - ROD_MASS     # added to the body (0.28 kg)


class _LeanMultirotor(Multirotor):
    """Stock Multirotor with a cheaper per-step state read and no propeller animation when headless.

    Vehicle.update_state queries the USD world transform for the attitude and converts carb.Float3
    values with np.array (sequence protocol, ~60 us each): ~0.7 ms of the 4 ms real-time budget at
    250 Hz. This reads the same quantities from dynamic control, so the State it fills is the same
    (the rigid-body pose is the body's world pose)."""

    def update_state(self, dt: float):
        dc = self.get_dc_interface()
        body = dc.get_rigid_body(self._stage_prefix + "/body")
        pose = dc.get_rigid_body_pose(body)
        p, r = pose.p, pose.r
        lv = dc.get_rigid_body_linear_velocity(body)
        av = dc.get_rigid_body_angular_velocity(body)
        v = np.array((lv.x, lv.y, lv.z))
        st = self._state
        if dt > 0.0:
            acc = (v - st.linear_velocity) / dt
            jerk = (acc - st.linear_acceleration) / dt
        else:
            acc, jerk = np.zeros(3), np.zeros(3)
        st.position = np.array((p.x, p.y, p.z))
        st.attitude = np.array((r.x, r.y, r.z, r.w))
        st.linear_velocity = v
        R_BI = Rotation.from_quat(st.attitude).inv()
        st.linear_body_velocity = R_BI.apply(v)
        st.angular_velocity = R_BI.apply(np.array((av.x, av.y, av.z)))
        st.linear_acceleration, st.linear_jerk = acc, jerk

    def handle_propeller_visual(self, rotor_number, force, articulation):
        if not _headless:
            super().handle_propeller_visual(rotor_number, force, articulation)


class JGCDSim:
    def __init__(self, l0=1.0, payload_pos=(0.0, 0.0, 1.0), physics_hz=250.0):
        self.timeline = omni.timeline.get_timeline_interface()
        stage = omni.usd.get_context().get_stage()
        self.stage = stage
        add_dome_lighting(stage=stage, dome_path="/World/DomeLight", intensity=2500.0, exposure=0.0,
                          color=(1.0, 1.0, 1.0))
        self.pg = PegasusInterface()
        settings = dict(self.pg._world_settings)
        settings["physics_dt"] = 1.0 / physics_hz
        self.pg._world = World(**settings)
        self.world = self.pg.world
        self.physics_dt = settings["physics_dt"]
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        x_p0 = np.array(payload_pos, float)
        x_q0 = x_p0 + np.array([0.0, 0.0, l0])
        k_rot, w_max = x650_params.ROTOR_CONSTANT, x650_params.MAX_ROTOR_VEL
        self.omega_hover = float(np.sqrt((M_DRONE + M_PAYLOAD) * G / (4 * k_rot)))

        # ---- vehicle: bridge is backends[0] (rotor speed references), no PX4
        config = MultirotorConfig()
        config.thrust_curve = LaggedQuadraticThrustCurve(config={
            "rotor_constant": [k_rot] * 4,
            "rolling_moment_coefficient": [x650_params.ROLLING_MOMENT_COEFFICIENT] * 4,
            "min_rotor_velocity": [x650_params.MIN_ROTOR_VEL] * 4,
            "max_rotor_velocity": [w_max] * 4,
            "rotor_lambda": [x650_params.ROTOR_LAMBDA] * 4,
            "rot_dir": [-1, -1, 1, 1],
        })
        config.backends = []       # filled once the bridge exists (it needs the rod/payload prims)
        config.sensors = []        # no IMU/GPS/baro/mag: the controller gets the state from the bridge
        # The asset's rigid-body origin, where the cable attaches, is not its root (3.3 cm below it
        # for x650_new.usd): spawn the root so that the body origin is at x_q0.
        asset = Usd.Stage.Open(X650_USD)
        root = asset.GetDefaultPrim()
        tf = lambda prim: np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation())
        body_off = tf(root.GetChild("body")) - tf(root)
        self.vehicle = _LeanMultirotor(stage_prefix="/World/quadrotor_0", usd_file=X650_USD, vehicle_id=0,
                                  init_pos=tuple(x_q0 - body_off), init_orientation=[0.0, 0.0, 0.0, 1.0],
                                  config=config)
        apply_x650_mass_properties(self.vehicle, body_mass=X650_BODY_MASS + WINCH_MASS)
        self.drone_path = self.vehicle._stage_prefix
        # rotors start at hover speed (the lagged thrust curve integrates from _velocity)
        config.thrust_curve._velocity = [self.omega_hover] * 4

        # ---- payload and cable (rod axis +X points down, from the drone to the payload)
        payload_path = "/World/payload_link"
        sl.create_brick_with_xform_root(stage=stage, root_path=payload_path, length_x=0.08, width_y=0.08,
                                        height_z=0.08, mass=M_PAYLOAD - ROD_MASS, world_pos=tuple(x_p0),
                                        world_quat_wxyz=(1.0, 0.0, 0.0, 0.0), enable_collision=True)
        quat_down = Rotation.from_euler("y", 90, degrees=True).as_quat()       # x, y, z, w: X -> -Z
        q_wxyz = (quat_down[3], quat_down[0], quat_down[1], quat_down[2])
        down = np.array([0.0, 0.0, -1.0])
        self.rod_a_path, self.rod_b_path = "/World/cable_rod_a", "/World/cable_rod_b"
        for path, centre in ((self.rod_a_path, x_q0 + 0.5 * ROD_LEN * down),
                             (self.rod_b_path, x_p0 - 0.5 * ROD_LEN * down)):
            sl.create_cylinder_with_xform_root(stage=stage, root_path=path, length=ROD_LEN, radius=0.006,
                                               mass=ROD_MASS, world_pos=tuple(centre), world_quat_wxyz=q_wxyz,
                                               enable_collision=False, axis="X")

        if self._wait_for_prim(self.drone_path) is None:
            raise RuntimeError(f"Drone prim did not load: {self.drone_path}")
        self.world.reset()
        apply_x650_mass_properties(self.vehicle, body_mass=X650_BODY_MASS + WINCH_MASS)
        body_now = np.array(omni.usd.get_world_transform_matrix(
            stage.GetPrimAtPath(self.drone_path + "/body")).ExtractTranslation())
        print(f"[JGCD] body origin placement error {np.linalg.norm(body_now - x_q0) * 1e3:.2f} mm", flush=True)

        body = con.find_rigidbody_prim(stage, self.drone_path)
        uav_path = body.GetPath().pathString
        h = 0.5 * ROD_LEN
        con.create_spherical_joint(stage=stage, joint_path="/World/joint_uav_rod_a", body0_path=uav_path,
                                   body1_path=self.rod_a_path, local_pos0=(0.0, 0.0, 0.0), local_pos1=(-h, 0.0, 0.0))
        con.create_prismatic_joint(stage=stage, joint_path="/World/joint_rod_a_rod_b", body0_path=self.rod_a_path,
                                   body1_path=self.rod_b_path, local_pos0=(h, 0.0, 0.0), local_pos1=(-h, 0.0, 0.0),
                                   axis="X", lower_limit=0.0, upper_limit=L_MAX - L_MIN)
        con.create_spherical_joint(stage=stage, joint_path="/World/joint_rod_b_payload", body0_path=self.rod_b_path,
                                   body1_path=payload_path, local_pos0=(h, 0.0, 0.0), local_pos1=(0.0, 0.0, 0.0))

        self.bridge = JGCDBridgeBackend(
            world=self.world, drone_body_path=uav_path, payload_path=payload_path,
            rod_a_path=self.rod_a_path, rod_b_path=self.rod_b_path, rod_offset=ROD_LEN,
            omega_hover=self.omega_hover, omega_max=w_max, s0_tension=M_PAYLOAD * G,
            config={"topic_prefix": "jgcd/"})
        self.vehicle._backends = [self.bridge]
        self.bridge.initialize(self.vehicle)
        print(f"[JGCD] drone {M_DRONE} kg (winch +{WINCH_MASS:.3f} kg), payload {M_PAYLOAD} kg, l0 {l0} m, "
              f"physics {1 / self.physics_dt:.0f} Hz, hover {self.omega_hover:.1f} rad/s", flush=True)

    def _wait_for_prim(self, prim_path, max_frames=300):
        for _ in range(max_frames):
            prim = self.stage.GetPrimAtPath(prim_path)
            if prim and prim.IsValid():
                return prim
            self.world.step(render=False)
        return None

    def run(self, duration, rtf_target=1.0, lockstep=False, wait_controller=120.0, log_path=None):
        node = self.bridge.node
        t_wait = time.time()
        # (never spin the bridge node here: it belongs to the bridge's executor thread)
        while node.count_subscribers("jgcd/state") == 0 and time.time() - t_wait < wait_controller:
            time.sleep(0.1)
        print(f"[JGCD] controller subscribed: {node.count_subscribers('jgcd/state') > 0}; "
              f"mode {'LOCKSTEP' if lockstep else f'real time x{rtf_target}'}", flush=True)
        self.timeline.play()
        render = not _headless
        dt = self.physics_dt
        n_steps = int(round(duration / dt))
        log, rtf_windows, step_ms = [], [], []
        late_steps, waits = 0, []
        win_sim, win_wall = 0.0, time.perf_counter()
        wall0 = time.perf_counter()
        prof = None
        if os.environ.get("JGCD_PROFILE", "0") == "1":
            import cProfile
            prof = cProfile.Profile()
        for k in range(n_steps):
            if prof is not None and k == int(1.0 / dt):
                prof.enable()
            if prof is not None and k == int(4.0 / dt):
                prof.disable()
                import pstats
                pstats.Stats(prof).sort_stats("tottime").print_stats(25)
            if not simulation_app.is_running():
                break
            if k % 250 == 0 and k > 0 and node.count_subscribers("jgcd/state") == 0:
                print(f"[JGCD] controller gone at t={k * dt:.2f}s: ending the run", flush=True)
                break
            c0 = time.perf_counter()
            self.world.step(render=render)
            step_ms.append((time.perf_counter() - c0) * 1e3)
            if self.bridge.last_state is not None:
                log.append(self.bridge.last_state.copy())
            sim_t = (k + 1) * dt
            if lockstep:
                t_state = self.bridge.last_state[0] if self.bridge.last_state is not None else -1.0
                w0 = time.perf_counter()
                while self.bridge.t_cmd < t_state - 1e-9 and time.perf_counter() - w0 < 2.0:
                    time.sleep(0.0002)
                waits.append((time.perf_counter() - w0) * 1e3)
                if self.bridge.t_cmd < t_state - 1e-9:
                    print(f"[JGCD] lockstep: no reply for 2 s at t={t_state:.3f}s: ending the run", flush=True)
                    break
            else:
                target = wall0 + sim_t / rtf_target
                now = time.perf_counter()
                if now < target:
                    time.sleep(target - now)
                elif now - target > 0.010:
                    late_steps += 1
            win_sim += dt
            if win_sim >= 1.0 - 1e-9:
                now = time.perf_counter()
                rtf_windows.append(win_sim / (now - win_wall))
                print(f"[JGCD] t={sim_t:6.2f}s RTF(1s)={rtf_windows[-1]:.3f} step avg {np.mean(step_ms[-int(1/dt):]):.2f} ms "
                      f"max {np.max(step_ms[-int(1/dt):]):.2f} ms cmds {self.bridge.n_cmd_rx}", flush=True)
                win_sim, win_wall = 0.0, now
        wall = time.perf_counter() - wall0
        rtf = np.array(rtf_windows)
        summary = dict(sim_time=len(step_ms) * dt, wall_time=wall, rtf_overall=len(step_ms) * dt / wall,
                       rtf_min=float(rtf.min()) if rtf.size else np.nan, rtf_mean=float(rtf.mean()) if rtf.size else np.nan,
                       windows_below_0p98=int(np.sum(rtf < 0.98)), late_steps_10ms=late_steps,
                       step_ms_mean=float(np.mean(step_ms)), step_ms_p99=float(np.percentile(step_ms, 99)),
                       step_ms_max=float(np.max(step_ms)), cmds=self.bridge.n_cmd_rx, lockstep=lockstep,
                       physics_hz=1.0 / dt, rtf_target=rtf_target)
        print("[JGCD] SUMMARY " + " ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}"
                                           for k, v in summary.items()), flush=True)
        if log_path:
            np.savez(log_path, state=np.array(log), fields=np.array(STATE_FIELDS), rtf_windows=rtf,
                     step_ms=np.array(step_ms), waits_ms=np.array(waits), **{f"summary_{k}": v for k, v in summary.items()})
            print(f"[JGCD] truth log -> {log_path}", flush=True)
        carb.log_warn("JGCDSim is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    env = os.environ.get
    sim = JGCDSim(l0=float(env("JGCD_L0", "1.0")), physics_hz=float(env("JGCD_PHYSICS_HZ", "250")))
    sim.run(duration=float(env("JGCD_DURATION", "20")), rtf_target=float(env("JGCD_RTF", "1.0")),
            lockstep=env("JGCD_LOCKSTEP", "0") == "1", wait_controller=float(env("JGCD_WAIT_CONTROLLER", "120")),
            log_path=env("JGCD_LOG", "/tmp/jgcd_isaac.npz"))


if __name__ == "__main__":
    main()

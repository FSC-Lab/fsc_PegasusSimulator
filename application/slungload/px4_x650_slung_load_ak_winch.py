#!/usr/bin/env python
"""
| File: px4_x650_slung_load_ak_winch.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2026, Longhao Qian. All rights reserved.
| Description: Isaac Sim plant for the X650 slung-load LAB STACK rehearsal (2026-11-21):
|   PX4 SITL is primary (attitude setpoints in SAFETY, actuator_motors in SLUNG, exactly as
|   on the vehicle), the winch is the AK40-10 motor behind the LAB cable node, and the mocap
|   is published under the lab's Motive names by fsc_autopilot_ros2 scripts/isaacsim/slung/sim_mocap.py.

Plant (matched to the JGCD paper plant, application/slungload/jgcd_x650_winch_direct.py):
  * X650 (x650_new.usd, MN4014 + 15x5" bench curve and rotor lag) via spawn_x650_with_mavlink,
    body mass raised by the winch so the drone side totals SLUNG_SIM_DRONE_MASS (3.8 kg);
  * payload: an 8 cm brick + rod_b = SLUNG_SIM_PAYLOAD_MASS (0.6 kg), cable at its centre;
  * cable: two SLUNG_SIM_ROD_LEN (0.1 m) rods joined by a prismatic joint, l = 2 ROD_LEN + e,
    e in [0, L_MAX - 2 ROD_LEN]; e = 0 (rods flush) IS THE CABLE STOPPER the lab auto zero homes on;
  * AK40-10 drum (JGCD WINCH constants: inertia, viscous + Coulomb friction, back-EMF torque
    limit) with drum radius SLUNG_SIM_DRUM_RADIUS (default 0.0188 m, see below),
    elastic cable k = 1000 N/m, integrated with sub-steps inside each physics step. Drum radius 0.0188 m:
    the EFFECTIVE radius found in the 11-20 flight bag (cable length changes 0.523 m per 1 m of encoder length
    at the deployed 0.036), set identically in the sim cable node, cable_torque_ctrl_node and the slung yaml;
  * the motor runs the AK MIT law tau = kp (p_des - theta) + kd (v_des - omega) + t_ff at
    physics rate on the command the lab node sends over the simulated CAN bus
    (/ak_sim/mit_cmd from AK40-10-ROS2-Bridge's ak_motor_cable_control_node_sim); a disabled motor is limp.
    theta = -s / R (motor frame: + = retract), s = cable paid out by the drum.

Sim-only controls (not on the vehicle):
  /sim/payload_attached  std_msgs/Bool   false = "no payload" (the brick's weight is carried,
                                         as if unhooked), for the lab's no-payload auto zero.
                                         Starts as SLUNG_SIM_ATTACHED (default 0).
  /sim/disturbance       Float64MultiArray [F_q(3), F_p(3), M_q(3)] world frame [N, N m],
                                         held until changed (publish zeros to clear).
Published every physics step:
  /ak_sim/motor_state    [t, theta, omega, tau]  (the simulated CAN feedback source)
  /sim/truth             [t, x_q(3), q_q xyzw(4), v_q(3), w_q world(3), x_p(3), q_p xyzw(4),
                          v_p(3), e, e_dot, s, s_dot, tension, tau_motor, attached]
plus the stock /uav_0/state/* from the spawn helper's ROS2Backend.

Environment: PEGASUS_HEADLESS, PEGASUS_PX4_LOCKSTEP (0 for the wall-clock DDS stack),
PEGASUS_REALTIME (1: hold sim time to the wall clock), PEGASUS_RENDER_EVERY, SLUNG_SIM_L0
(initial cable length, payload lying beside the drone, default 0.35 m), SLUNG_SIM_ATTACHED,
SLUNG_SIM_DRONE_MASS, SLUNG_SIM_PAYLOAD_MASS, SLUNG_SIM_DRUM_RADIUS, SLUNG_SIM_ROD_LEN.
"""
import os
import threading
import time

import carb
from isaacsim import SimulationApp

HEADLESS = os.environ.get("PEGASUS_HEADLESS", "0") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np
import omni.timeline
import omni.usd
from omni.isaac.core.world import World
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from fsc_aerial_manipulation.rotorcraft import x650_params
from fsc_aerial_manipulation.rotorcraft.x650_bare_frame_utils import (
    X650_BODY_MASS, X650_USD, apply_x650_mass_properties, spawn_x650_with_mavlink)
from fsc_aerial_manipulation.utils import add_dome_lighting
from fsc_aerial_manipulation.utils.jgcd_bridge_backend_utils import WINCH as JGCD_WINCH
import fsc_aerial_manipulation.slung_load as sl
import fsc_aerial_manipulation.constraints as con

# rclpy only after the Pegasus imports (they enable the ROS 2 bridge that provides it)
import rclpy
import rclpy.executors
from std_msgs.msg import Bool, Float64MultiArray

ENV = os.environ.get
PX4_LOCKSTEP = ENV("PEGASUS_PX4_LOCKSTEP", "0") == "1"
REALTIME = ENV("PEGASUS_REALTIME", "1") == "1"
RENDER_EVERY = int(ENV("PEGASUS_RENDER_EVERY", "") or 8)

G = 9.81
M_DRONE = float(ENV("SLUNG_SIM_DRONE_MASS", "3.8"))
M_PAYLOAD = float(ENV("SLUNG_SIM_PAYLOAD_MASS", "0.6"))
R_DRUM = float(ENV("SLUNG_SIM_DRUM_RADIUS", "0.0188"))   # effective radius from the 11-20 flight bag
ROD_MASS = 0.02
ROD_LEN = float(ENV("SLUNG_SIM_ROD_LEN", "0.1"))
L0 = float(ENV("SLUNG_SIM_L0", "0.35"))
L_MAX = 2.0
ATTACHED0 = ENV("SLUNG_SIM_ATTACHED", "0") == "1"
WINCH_MASS = M_DRONE - x650_params.MASS - ROD_MASS
SPAWN_ROOT = np.array([0.0, 0.0, 0.07])
BRICK = 0.08


def _v3(v):
    return np.array((v.x, v.y, v.z))


class AkWinchPlant:
    """AK40-10 drum + motor MIT law + cable tension + sim-only payload/disturbance controls."""

    T_CODEC = 5.0          # the MIT frame's torque range (lab yaml t_min/t_max)

    def __init__(self, world, paths, rod_offset, brick_mass):
        self._world = world
        self.paths = paths
        self.rod_offset = rod_offset
        self.brick_mass = brick_mass
        w = dict(JGCD_WINCH, R_D=R_DRUM, TAU_SAT=1.9)   # 1.9 N m = AK40-10 peak (lab yaml note)
        self.R, self.TAU_SAT, self.W_NL, self.TAU_BEMF0 = w["R_D"], w["TAU_SAT"], w["W_NL"], w["TAU_BEMF0"]
        self.M_W, self.B_L = w["J_W"] / self.R ** 2, w["B_W"] / self.R ** 2
        self.F_C, self.EPS_L = w["TAU_C"] / self.R, w["EPS_W"] * self.R
        self.k_c = 1000.0
        m_eff = 0.5
        m_red = self.M_W * m_eff / (self.M_W + m_eff)
        self.c_c = 2.0 * 0.7 * np.sqrt(self.k_c * m_red)
        self.n_sub = 10
        self.lock = threading.Lock()
        self.mit = np.zeros(6)          # enabled, p_des, v_des, kp, kd, t_ff
        self.attached = ATTACHED0
        self.dist = np.zeros(9)
        self.s = None
        self.sd = 0.0
        self.tau = 0.0
        self.T = 0.0
        self.t = 0.0
        self._dc = None
        self._rb = None
        try:
            rclpy.init()
        except Exception:
            pass
        self.node = rclpy.create_node("slung_sim_plant")
        qos = rclpy.qos.qos_profile_sensor_data
        self.motor_pub = self.node.create_publisher(Float64MultiArray, "/ak_sim/motor_state", qos)
        self.truth_pub = self.node.create_publisher(Float64MultiArray, "/sim/truth", qos)
        self.node.create_subscription(Float64MultiArray, "/ak_sim/mit_cmd", self._mit_cb, qos)
        self.node.create_subscription(Bool, "/sim/payload_attached", self._att_cb, 10)
        self.node.create_subscription(Float64MultiArray, "/sim/disturbance", self._dist_cb, 10)
        self._exec = rclpy.executors.SingleThreadedExecutor()
        self._exec.add_node(self.node)
        threading.Thread(target=self._exec.spin, daemon=True).start()
        world.add_physics_callback("slung_sim_ak_winch", self.step)
        print(f"[SLUNG-SIM] drum R {self.R} m, reflected mass {self.M_W:.3f} kg, Coulomb {self.F_C:.2f} N, "
              f"payload {'ATTACHED' if self.attached else 'DETACHED (no-payload auto zero)'}", flush=True)

    def _mit_cb(self, msg):
        if len(msg.data) >= 6:
            with self.lock:
                self.mit[:] = msg.data[:6]

    def _att_cb(self, msg):
        with self.lock:
            if self.attached != bool(msg.data):
                print(f"[SLUNG-SIM] payload {'ATTACHED' if msg.data else 'DETACHED'} at t={self.t:.2f}s", flush=True)
            self.attached = bool(msg.data)

    def _dist_cb(self, msg):
        d = np.zeros(9)
        n = min(9, len(msg.data))
        d[:n] = msg.data[:n]
        with self.lock:
            self.dist = d
        print(f"[SLUNG-SIM] disturbance F_q {d[0:3]} F_p {d[3:6]} M_q {d[6:9]} at t={self.t:.2f}s", flush=True)

    def _torque_limit(self, tau, sd):
        tau = float(np.clip(tau, -self.TAU_SAT, self.TAU_SAT))
        w = -sd / self.R                       # motor speed, + = retract
        if tau * w > 0:                        # motoring
            lim = max(self.TAU_BEMF0 * (1.0 - abs(w) / self.W_NL), 0.0)
            tau = float(np.clip(tau, -lim, lim))
        return tau

    def step(self, dt):
        if self._dc is None:
            self._dc = _dynamic_control.acquire_dynamic_control_interface()
        dc = self._dc
        if self._rb is None:
            self._rb = {k: dc.get_rigid_body(p) for k, p in self.paths.items()}
        rb = self._rb
        pose = {k: dc.get_rigid_body_pose(h) for k, h in rb.items()}
        pos = {k: _v3(P.p) for k, P in pose.items()}
        vel = {k: _v3(dc.get_rigid_body_linear_velocity(h)) for k, h in rb.items()}
        r_a = pose["a"].r
        x, y, z, wq = r_a.x, r_a.y, r_a.z, r_a.w
        axis = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + z * wq), 2 * (x * z - y * wq)])
        e = float(np.dot(pos["b"] - pos["a"], axis)) - self.rod_offset
        ed = float(np.dot(vel["b"] - vel["a"], axis))
        with self.lock:
            mit = self.mit.copy()
            attached = self.attached
            dist = self.dist.copy()
        if self.s is None:
            self.s, self.sd = e, ed

        # drum + motor, sub-stepped (explicit cable coupling, joint motion extrapolated linearly)
        h = dt / self.n_sub
        T_sum, tau_sum = 0.0, 0.0
        enabled, p_des, v_des, kp, kd, t_ff = mit
        for i in range(self.n_sub):
            e_i = e + ed * i * h
            T = max(0.0, self.k_c * (e_i - self.s) + self.c_c * (ed - self.sd))
            if enabled > 0.5:
                th, om = -self.s / self.R, -self.sd / self.R
                tau = kp * (p_des - th) + kd * (v_des - om) + t_ff
                tau = self._torque_limit(float(np.clip(tau, -self.T_CODEC, self.T_CODEC)), self.sd)
            else:
                tau = 0.0
            sdd = (T - tau / self.R - self.B_L * self.sd - self.F_C * np.tanh(self.sd / self.EPS_L)) / self.M_W
            self.sd += h * sdd
            self.s += h * self.sd
            T_sum += T
            tau_sum += tau
        self.T, self.tau = T_sum / self.n_sub, tau_sum / self.n_sub

        f = self.T * np.array([1.0, 0.0, 0.0])
        dc.apply_body_force(rb["b"], carb._carb.Float3((-f).tolist()), carb._carb.Float3([0.0, 0.0, 0.0]), False)
        dc.apply_body_force(rb["a"], carb._carb.Float3(f.tolist()), carb._carb.Float3([0.0, 0.0, 0.0]), False)
        F_p = dist[3:6].copy()
        if not attached:
            F_p[2] += self.brick_mass * G            # "unhooked": its weight is carried
        if np.any(F_p):
            dc.apply_body_force(rb["p"], carb._carb.Float3(F_p.tolist()), carb._carb.Float3(pos["p"].tolist()), True)
        if np.any(dist[0:3]):
            dc.apply_body_force(rb["q"], carb._carb.Float3(dist[0:3].tolist()), carb._carb.Float3(pos["q"].tolist()), True)
        if np.any(dist[6:9]):
            dc.apply_body_torque(rb["q"], carb._carb.Float3(dist[6:9].tolist()), True)

        self.t += dt
        m = Float64MultiArray()
        m.data = [self.t, -self.s / self.R, -self.sd / self.R, self.tau]
        self.motor_pub.publish(m)
        r_q, r_p = pose["q"].r, pose["p"].r
        tr = Float64MultiArray()
        tr.data = ([self.t] + pos["q"].tolist() + [r_q.x, r_q.y, r_q.z, r_q.w] + vel["q"].tolist()
                   + _v3(dc.get_rigid_body_angular_velocity(rb["q"])).tolist()
                   + pos["p"].tolist() + [r_p.x, r_p.y, r_p.z, r_p.w] + vel["p"].tolist()
                   + [e, ed, self.s, self.sd, self.T, self.tau, 1.0 if attached else 0.0])
        self.truth_pub.publish(tr)


class SlungSim:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        stage = omni.usd.get_context().get_stage()
        self.stage = stage
        add_dome_lighting(stage=stage, dome_path="/World/DomeLight", intensity=2500.0, exposure=0.0,
                          color=(1.0, 1.0, 1.0))
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        body_mass = X650_BODY_MASS + WINCH_MASS
        self.drone_path = spawn_x650_with_mavlink(
            px4_path=self.pg.px4_path, px4_default_airframe=self.pg.px4_default_airframe, vehicle_id=0,
            spawn_pos=tuple(SPAWN_ROOT), spawn_euler=(0.0, 0.0, 0.0), enable_lockstep=PX4_LOCKSTEP,
            body_mass=body_mass)

        # body origin (the cable attachment) in the world, from the asset's root->body offset
        asset = Usd.Stage.Open(X650_USD)
        root = asset.GetDefaultPrim()
        tf = lambda prim: np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation())
        x_q0 = SPAWN_ROOT + (tf(root.GetChild("body")) - tf(root))
        # payload lying on the floor beside the drone, cable straight at length L0
        z_p = 0.5 * BRICK + 0.002
        dz = z_p - x_q0[2]
        dx = float(np.sqrt(max(L0 ** 2 - dz ** 2, 1e-6)))
        x_p0 = np.array([x_q0[0] + dx, x_q0[1], z_p])
        d = (x_p0 - x_q0) / np.linalg.norm(x_p0 - x_q0)
        rot, _ = Rotation.align_vectors([d], [[1.0, 0.0, 0.0]])
        qx = rot.as_quat()
        q_wxyz = (qx[3], qx[0], qx[1], qx[2])

        payload_path = "/World/payload_link"
        brick_mass = M_PAYLOAD - ROD_MASS
        sl.create_brick_with_xform_root(stage=stage, root_path=payload_path, length_x=BRICK, width_y=BRICK,
                                        height_z=BRICK, mass=brick_mass, world_pos=tuple(x_p0),
                                        world_quat_wxyz=(1.0, 0.0, 0.0, 0.0), enable_collision=True)
        self.rod_a_path, self.rod_b_path = "/World/cable_rod_a", "/World/cable_rod_b"
        hh = 0.5 * ROD_LEN
        for path, centre in ((self.rod_a_path, x_q0 + hh * d), (self.rod_b_path, x_p0 - hh * d)):
            sl.create_cylinder_with_xform_root(stage=stage, root_path=path, length=ROD_LEN, radius=0.006,
                                               mass=ROD_MASS, world_pos=tuple(centre), world_quat_wxyz=q_wxyz,
                                               enable_collision=False, axis="X")
        for _ in range(300):
            prim = stage.GetPrimAtPath(self.drone_path)
            if prim and prim.IsValid():
                break
            self.world.step(render=False)
        self.world.reset()
        apply_x650_mass_properties(_VehicleShim(self.drone_path, self.world), body_mass=body_mass)

        body = con.find_rigidbody_prim(stage, self.drone_path)
        uav_path = body.GetPath().pathString
        con.create_spherical_joint(stage=stage, joint_path="/World/joint_uav_rod_a", body0_path=uav_path,
                                   body1_path=self.rod_a_path, local_pos0=(0.0, 0.0, 0.0), local_pos1=(-hh, 0.0, 0.0))
        con.create_prismatic_joint(stage=stage, joint_path="/World/joint_rod_a_rod_b", body0_path=self.rod_a_path,
                                   body1_path=self.rod_b_path, local_pos0=(hh, 0.0, 0.0), local_pos1=(-hh, 0.0, 0.0),
                                   axis="X", lower_limit=0.0, upper_limit=L_MAX - 2 * ROD_LEN)
        con.create_spherical_joint(stage=stage, joint_path="/World/joint_rod_b_payload", body0_path=self.rod_b_path,
                                   body1_path=payload_path, local_pos0=(hh, 0.0, 0.0), local_pos1=(0.0, 0.0, 0.0))
        self.plant = AkWinchPlant(self.world, dict(q=uav_path, p=payload_path, a=self.rod_a_path, b=self.rod_b_path),
                                  rod_offset=ROD_LEN, brick_mass=brick_mass)
        print(f"[SLUNG-SIM] drone {M_DRONE} kg (winch +{WINCH_MASS:.3f} kg on the body), payload {M_PAYLOAD} kg, "
              f"cable l = {2 * ROD_LEN:.2f} + e (stopper at e = 0, i.e. l = {2 * ROD_LEN:.2f} m), l0 {L0} m; "
              f"PX4 lockstep {PX4_LOCKSTEP}, real-time pacer {REALTIME}", flush=True)

    def run(self):
        self.timeline.play()
        steps = 0
        pacer = _RealTimePacer(self) if REALTIME else None
        while simulation_app.is_running():
            if HEADLESS or RENDER_EVERY > 0:
                self.world.step(render=False)
                if not HEADLESS and steps % RENDER_EVERY == 0:
                    self.world.render()
            else:
                self.world.step(render=True)
            steps += 1
            if pacer is not None:
                pacer.after_step()
        carb.log_warn("SlungSim is closing.")
        self.timeline.stop()
        simulation_app.close()


class _VehicleShim:
    """apply_x650_mass_properties only needs the stage prefix and the world."""

    def __init__(self, prefix, world):
        self._stage_prefix = prefix
        self._world = world


class _RealTimePacer:
    """Hold simulated time to the wall clock (same as 05/06's PEGASUS_REALTIME pacer)."""

    MAX_LAG_S = 0.05
    REPORT_S = 10.0

    def __init__(self, sim):
        self.sim = sim
        self.w0 = time.perf_counter()
        self.s0 = float(sim.world.current_time)
        self.rep_w, self.rep_s = self.w0, self.s0
        self.worst_lag, self.reanchors = 0.0, 0

    def after_step(self):
        now = time.perf_counter()
        st = float(self.sim.world.current_time)
        ahead = (st - self.s0) - (now - self.w0)
        if ahead > 0.0:
            time.sleep(ahead)
            now = time.perf_counter()
        elif -ahead > self.MAX_LAG_S:
            self.worst_lag = max(self.worst_lag, -ahead)
            self.reanchors += 1
            self.w0, self.s0 = now, st
        else:
            self.worst_lag = max(self.worst_lag, -ahead)
        if now - self.rep_w >= self.REPORT_S:
            rtf = (st - self.rep_s) / (now - self.rep_w)
            print(f"[SLUNG-SIM] RTF {rtf:.3f} over the last {now - self.rep_w:.1f} s (worst lag "
                  f"{1e3 * self.worst_lag:.1f} ms, {self.reanchors} re-anchor(s))", flush=True)
            self.rep_w, self.rep_s = now, st
            self.worst_lag, self.reanchors = 0.0, 0


def main():
    SlungSim().run()


if __name__ == "__main__":
    main()

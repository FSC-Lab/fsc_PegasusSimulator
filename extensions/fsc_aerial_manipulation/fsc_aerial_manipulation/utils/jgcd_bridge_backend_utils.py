"""
| File: jgcd_bridge_backend_utils.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2026, Longhao Qian. All rights reserved.
| Description: ROS 2 bridge for testing the JGCD paper's controller
|   (~/source/JGCD-paper-longhao/main.tex, companion code ~/source/JGCD2025-sim) on the X650
|   variable-length slung-load scenario, with no PX4 in the loop.

One object plays three roles:

1. Primary vehicle backend (``config.backends[0]``): ``input_reference()`` returns the rotor speed
   references [rad/s] last received on ``<prefix>cmd``. Pegasus' LaggedQuadraticThrustCurve
   applies the bench rotor lag to them. (The stock ROS2Backend cannot be used for this: its four
   rotor subscriptions are created in a loop with ``lambda x: self.rotor_callback(x, i)``, so
   every one of them writes rotor 3.)
2. AK40-10 winch drum. The prismatic rod pair carries only the cable; the drum is a separate 1-D
   state s (cable paid out) with the reflected mass, viscous and Coulomb friction and the torque
   limits of JGCD2025-sim/jgcd_sim/params.py + plant.winch_torque_limit:
       M_W s'' = T - F_m - B_L s' - F_C tanh(s'/EPS_L),     F_m = tau_lim(tau_m, s') / R_D,
   coupled to the joint extension e by an elastic cable, T = max(0, k (e - s) + c (e' - s')).
   T is applied to the rods as an equal-and-opposite local-frame force pair (the
   ROS2CableWinchBackend convention). The drum is integrated with sub-steps inside the physics
   step with the joint motion extrapolated linearly (explicit coupling; k and c are sized so the
   coupled mode is well resolved at 250 Hz).
3. State publisher: one Float64MultiArray per physics step on ``<prefix>state`` (layout in
   STATE_FIELDS), plus ``/clock``. External disturbance forces [N, world frame] on the drone and
   the payload are taken from ``<prefix>dist`` = [F_q(3), F_p(3)].
"""

import threading
import time

import numpy as np

import carb
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Gf

import rclpy
import rclpy.executors
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray

from pegasus.simulator.logic.backends.backend import Backend

# index -> name of the published state vector
STATE_FIELDS = (
    ["t"] + [f"x_q{i}" for i in range(3)] + [f"v_q{i}" for i in range(3)] + ["qx", "qy", "qz", "qw"]
    + [f"w{i}" for i in range(3)] + [f"x_p{i}" for i in range(3)] + [f"v_p{i}" for i in range(3)]
    + ["e", "e_dot", "tension", "s", "s_dot", "F_m", "t_cmd", "wall"] + [f"omega_ref{i}" for i in range(4)]
)

# AK40-10 winch, JGCD2025-sim/jgcd_sim/params.py (drum radius from the user, 2026-10-22; inertia
# and friction regressed on the real winch; torque limits from the deployed controller/datasheet)
WINCH = dict(R_D=0.032, J_W=0.000995, B_W=0.00038, TAU_C=0.0559, EPS_W=0.5, TAU_SAT=1.5,
             W_NL=435.0 * 2 * np.pi / 60.0, TAU_BEMF0=1.3 / (1.0 - 370.0 / 435.0))


def _v3(v):
    """carb.Float3 -> ndarray (np.array(v) goes through the sequence protocol: ~60 us)."""
    return np.array((v.x, v.y, v.z))


class JGCDBridgeBackend(Backend):

    def __init__(self, world, drone_body_path: str, payload_path: str, rod_a_path: str, rod_b_path: str,
                 rod_offset: float, omega_hover: float, omega_max: float, s0_tension: float,
                 config: dict = {}):
        """
        Args:
            world: the Isaac Sim World.
            drone_body_path / payload_path: rigid-body prims of the drone and the payload.
            rod_a_path / rod_b_path: drone-side / payload-side rods of the prismatic cable.
            rod_offset (float): distance between the rod centres at zero joint extension [m].
            omega_hover (float): rotor speed reference before the first command [rad/s].
            omega_max (float): rotor speed clamp [rad/s].
            s0_tension (float): cable tension at start [N]; the drum starts pre-stretched by it.
            config (dict): {"topic_prefix": "jgcd/", "cable_k": 1000.0 N/m, "cable_zeta": 0.7,
                "drum_substeps": 10, "winch": {...overrides of WINCH}}
        """
        self._world = world
        self.paths = dict(q=drone_body_path, p=payload_path, a=rod_a_path, b=rod_b_path)
        self.rod_offset = float(rod_offset)
        self.omega_hover, self.omega_max = float(omega_hover), float(omega_max)
        self._prefix = config.get("topic_prefix", "jgcd/")
        w = dict(WINCH, **config.get("winch", {}))
        self.R_D, self.TAU_SAT, self.W_NL, self.TAU_BEMF0 = w["R_D"], w["TAU_SAT"], w["W_NL"], w["TAU_BEMF0"]
        self.M_W, self.B_L = w["J_W"] / w["R_D"] ** 2, w["B_W"] / w["R_D"] ** 2
        self.F_C, self.EPS_L = w["TAU_C"] / w["R_D"], w["EPS_W"] * w["R_D"]
        self.k_c = float(config.get("cable_k", 1000.0))
        m_eff = 0.5     # payload-side effective mass of the relative motion (~ m_p m_q / (m_p + m_q))
        m_red = self.M_W * m_eff / (self.M_W + m_eff)
        self.c_c = 2.0 * float(config.get("cable_zeta", 0.7)) * np.sqrt(self.k_c * m_red)
        self.n_sub = int(config.get("drum_substeps", 10))
        self.s0_tension = float(s0_tension)

        try:
            rclpy.init()
        except Exception:
            pass
        self.node = rclpy.create_node("simulator_jgcd_bridge")
        qos = rclpy.qos.qos_profile_sensor_data
        self.state_pub = self.node.create_publisher(Float64MultiArray, self._prefix + "state", qos)
        self.clock_pub = self.node.create_publisher(Clock, "/clock", 10)
        self.node.create_subscription(Float64MultiArray, self._prefix + "cmd", self._cmd_cb, qos)
        self.node.create_subscription(Float64MultiArray, self._prefix + "dist", self._dist_cb, 10)
        # Commands are taken on a dedicated executor thread: a spin_once inside the physics step
        # handles one message per call and costs ~0.2 ms; the callbacks only swap small arrays.
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._dc = None
        self._rb = None
        self._axis_local = Gf.Vec3d(1.0, 0.0, 0.0)
        self.reset()
        self._world.add_physics_callback("jgcd_bridge", self.update_sim_state)

    # ------------------------------------------------------------------ ROS callbacks
    def _cmd_cb(self, msg):
        d = np.asarray(msg.data, float)
        if d.size != 6 or not np.all(np.isfinite(d)):
            carb.log_warn(f"[JGCD] rejected command {d}")
            return
        self.t_cmd = d[0]
        self.omega_ref = np.clip(d[1:5], 0.0, self.omega_max)
        self.tau_m = d[5]
        self.n_cmd_rx += 1

    def _dist_cb(self, msg):
        d = np.asarray(msg.data, float)
        if d.size == 6 and np.all(np.isfinite(d)):
            self.F_q, self.F_p = d[0:3].copy(), d[3:6].copy()

    # ------------------------------------------------------------------ Backend interface
    def input_reference(self):
        return [float(x) for x in self.omega_ref]

    def update_state(self, state):
        pass

    def update_sensor(self, sensor_type: str, data):
        pass

    def update_graphical_sensor(self, sensor_type: str, data):
        pass

    def update(self, dt: float):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def reset(self):
        self.t = 0.0
        self.t_cmd = -1.0
        self.n_cmd_rx = 0
        self.omega_ref = np.full(4, self.omega_hover)
        self.tau_m = self.s0_tension * WINCH["R_D"]      # holds the payload until the first command
        self.F_q, self.F_p = np.zeros(3), np.zeros(3)
        self.s = None            # drum state, initialised from the joint on the first step
        self.sd = 0.0
        self.tension = self.s0_tension
        self.F_m = 0.0
        self.last_state = None

    # ------------------------------------------------------------------ winch model
    def _torque_limit(self, tau, sd):
        tau = float(np.clip(tau, -self.TAU_SAT, self.TAU_SAT))
        w = sd / self.R_D
        if tau * (-w) > 0:      # motoring (retract-positive torque, shaft turning to retract)
            lim = max(self.TAU_BEMF0 * (1.0 - abs(w) / self.W_NL), 0.0)
            tau = float(np.clip(tau, -lim, lim))
        return tau

    def _drum(self, e, ed, dt):
        h = dt / self.n_sub
        T_sum = 0.0
        for i in range(self.n_sub):
            e_i = e + ed * i * h
            T = max(0.0, self.k_c * (e_i - self.s) + self.c_c * (ed - self.sd))
            F_m = self._torque_limit(self.tau_m, self.sd) / self.R_D
            sdd = (T - F_m - self.B_L * self.sd - self.F_C * np.tanh(self.sd / self.EPS_L)) / self.M_W
            self.sd += h * sdd                  # semi-implicit Euler
            self.s += h * self.sd
            T_sum += T
        self.F_m = F_m
        return T_sum / self.n_sub

    # ------------------------------------------------------------------ physics step
    def update_sim_state(self, dt: float):
        if self._dc is None:
            self._dc = _dynamic_control.acquire_dynamic_control_interface()
        dc = self._dc
        if self._rb is None:
            self._rb = {k: dc.get_rigid_body(p) for k, p in self.paths.items()}
        rb = self._rb
        pose = {k: dc.get_rigid_body_pose(h) for k, h in rb.items()}
        pos = {k: _v3(P.p) for k, P in pose.items()}
        vel = {k: _v3(dc.get_rigid_body_linear_velocity(h)) for k, h in rb.items()}

        # cable: joint extension along rod_a's local x axis (first column of its rotation; from the
        # dynamic-control pose -- a USD transform query here costs ~1 ms per step)
        r_a = pose["a"].r
        x, y, z, wq = r_a.x, r_a.y, r_a.z, r_a.w
        axis = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + z * wq), 2 * (x * z - y * wq)])
        e = float(np.dot(pos["b"] - pos["a"], axis)) - self.rod_offset
        ed = float(np.dot(vel["b"] - vel["a"], axis))
        if self.s is None:
            self.s = e - self.s0_tension / self.k_c
            self.sd = ed

        # state at the start of this step (published before the step's forces act)
        r_q = pose["q"].r
        q = np.array((r_q.x, r_q.y, r_q.z, r_q.w))
        w_world = _v3(dc.get_rigid_body_angular_velocity(rb["q"]))
        x, y, z, wq = q
        Rm = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * wq), 2 * (x * z + y * wq)],
                       [2 * (x * y + z * wq), 1 - 2 * (x * x + z * z), 2 * (y * z - x * wq)],
                       [2 * (x * z - y * wq), 2 * (y * z + x * wq), 1 - 2 * (x * x + y * y)]])
        w_body = Rm.T @ w_world
        st = np.concatenate([[self.t], pos["q"], vel["q"], q, w_body, pos["p"], vel["p"],
                             [e, ed, self.tension, self.s, self.sd, self.F_m, self.t_cmd, time.time()],
                             self.omega_ref])
        self.last_state = st
        msg = Float64MultiArray()
        msg.data = st.tolist()
        self.state_pub.publish(msg)
        clk = Clock()
        clk.clock.sec, clk.clock.nanosec = int(self.t), int((self.t % 1.0) * 1e9)
        self.clock_pub.publish(clk)

        # forces over this step: cable tension (rods), disturbances (drone, payload)
        self.tension = self._drum(e, ed, dt)
        f = self.tension * np.array(self._axis_local)
        dc.apply_body_force(rb["b"], carb._carb.Float3((-f).tolist()), carb._carb.Float3([0.0, 0.0, 0.0]), False)
        dc.apply_body_force(rb["a"], carb._carb.Float3(f.tolist()), carb._carb.Float3([0.0, 0.0, 0.0]), False)
        if np.any(self.F_q):
            dc.apply_body_force(rb["q"], carb._carb.Float3(self.F_q.tolist()), carb._carb.Float3(pos["q"].tolist()), True)
        if np.any(self.F_p):
            dc.apply_body_force(rb["p"], carb._carb.Float3(self.F_p.tolist()), carb._carb.Float3(pos["p"].tolist()), True)
        self.t += dt

"""RaceEnv — a Gymnasium env flying a drone through a gate course on Isaac Sim.

The simplest, standalone form of the isaacrace MWE: one ``RaceEnv`` owns the drone prim
and drives PhysX directly via ``dynamic_control`` — no quadsim ``Quadrotor``/``State``
abstraction. Each step it applies the OQCRL analytic body force/torque, so the policy
flies the exact analytic model it was trained on, on real PhysX.

Frames: ENU-FLU native (Isaac); the lone NED is inside the observation (the policy is
forever-NED) — ``_ned_obs_state`` converts the drone state to the OQCRL NED-FRD
16-vector and feeds ``course.build_obs``.
"""

import carb
import gymnasium as gym
import numpy as np
from gymnasium import spaces
from isaacsim.core.utils.prims import define_prim
from omni.isaac.dynamic_control import _dynamic_control  # noqa: PLC2701
from scipy.spatial.transform import Rotation

import race.dynamics as dyn
from race.conversions import quat_aero_isaac, vec_enu_ned, vec_flu_frd
from race.course import RaceCourse


class RaceEnv(gym.Env):
    """obs: 20-dim gate-relative state; action: 4 motor cmds in [-1, 1]."""

    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        world,
        course: RaceCourse,
        usd_path: str,
        params=None,
        stage_prefix: str = "/World/drone",
        render_on_step: bool = False,
        max_steps: int = 1200,
        seed: int = 0,
        randomize_reset: bool = True,
        dynamics_mode: str = "kinematic",
    ):
        """Spawn the drone prim and build the env over an existing Isaac ``world``.

        The drone prim is referenced here (a stage-structure change), so this must run
        BEFORE the caller's ``world.reset()``. Physics must be live (world reset + the
        timeline playing) before the first ``reset()``.

        Args:
            world: the Isaac ``World`` (physics_dt should be ``dyn.DT`` = 100 Hz so one
                env step == one physics step, as the OQCRL model assumes).
            course: a ``RaceCourse`` (gate geometry + obs + rules).
            usd_path: the drone USD to reference at ``stage_prefix``.
            params: OQCRL ``QuadParams`` (defaults to ``dyn.PARAMS_5INCH``).
            stage_prefix: USD path for the drone.
            render_on_step: render each ``world.step`` (GUI).
            max_steps: truncation horizon.
            seed: RNG seed for reset randomization.
            randomize_reset: random gate/pose each episode (else the deterministic start).
            dynamics_mode: "kinematic" (write body rates -> the exact model the policy
                trained on) or "native" (apply a torque through PhysX's own inertia).
        """
        self.world = world
        self.course = course
        self.params = np.asarray(dyn.PARAMS_5INCH if params is None else params)
        self.stage_prefix = stage_prefix
        self.render_on_step = render_on_step
        self.max_steps = max_steps
        self.randomize_reset = randomize_reset
        self.dynamics_mode = dynamics_mode
        self.dt = dyn.DT
        self.rng = np.random.default_rng(seed)

        # Spawn the drone (stage-structure change -> before the caller's world.reset()).
        define_prim(stage_prefix, "Xform").GetReferences().AddReference(usd_path)

        self.action_space = spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(
            -np.inf, np.inf, shape=(20,), dtype=np.float32
        )
        self.motor_w = np.zeros(4)
        self.target_gate = 0
        self.step_count = 0
        self._obs = np.zeros(20, dtype=np.float32)

        # lazily-cached PhysX handles; ENU-FLU state filled by _update_state().
        self._dc = None
        self._body = None
        self._mass = None
        self._inertia = None
        self._pos = np.zeros(3)
        self._vel = np.zeros(3)
        self._quat = np.array([0.0, 0.0, 0.0, 1.0])  # xyzw, FLU->ENU
        self._omega = np.zeros(3)  # body FLU rates

    # ------------------------------------------------------------ PhysX handles
    def _dci(self):
        if self._dc is None:
            self._dc = _dynamic_control.acquire_dynamic_control_interface()
        return self._dc

    def _body_handle(self):
        """Cache the /body rigid-body handle + its mass and diagonal inertia."""
        if self._body is None:
            dc = self._dci()
            self._body = dc.get_rigid_body(self.stage_prefix + "/body")
            props = dc.get_rigid_body_properties(self._body)
            self._mass = float(props.mass)
            m = props.moment
            self._inertia = np.array([m.x, m.y, m.z])
        return self._body

    def _update_state(self):
        """Read the ENU-FLU drone state from PhysX into the cached fields."""
        dc, body = self._dci(), self._body_handle()
        pose = dc.get_rigid_body_pose(body)
        lin = np.array(dc.get_rigid_body_linear_velocity(body))  # ENU world
        ang_w = np.array(dc.get_rigid_body_angular_velocity(body))  # ENU world
        rot = Rotation.from_quat([pose.r.x, pose.r.y, pose.r.z, pose.r.w])  # FLU->ENU
        self._pos = np.array(pose.p)
        self._quat = np.array([pose.r.x, pose.r.y, pose.r.z, pose.r.w])
        self._vel = lin
        self._omega = rot.inv().apply(ang_w)  # body FLU rates

    # ------------------------------------------------------------ obs assembly
    def _ned_obs_state(self) -> np.ndarray:
        """ENU-FLU drone state -> the OQCRL NED-FRD 16-vector build_obs expects."""
        out = np.empty(16)
        out[0:3] = vec_enu_ned(self._pos)
        out[3:6] = vec_enu_ned(self._vel)
        out[6:9] = Rotation.from_quat(quat_aero_isaac(self._quat)).as_euler("xyz")
        out[9:12] = vec_flu_frd(self._omega)
        out[12:16] = self.motor_w
        return out

    # ------------------------------------------------------- faithful OQCRL step
    def _apply_oqcrl_control(self, actions: np.ndarray):
        """Advance the rotor state and apply the OQCRL body force/torque for one step."""
        body = self._body_handle()
        d_w, _ = dyn.motor_derivative(self.motor_w, actions, self.params)
        self.motor_w = np.clip(self.motor_w + self.dt * d_w, -1.0, 1.0)

        body2world = Rotation.from_quat(self._quat)
        vb_frd = vec_flu_frd(body2world.inv().apply(self._vel))
        accel_frd, torque_frd = dyn.body_force_torque_accel(
            self.motor_w, actions, vb_frd, self.params
        )
        dc = self._dci()
        force_flu = (self._mass * vec_flu_frd(accel_frd)).tolist()
        dc.apply_body_force(
            body, carb.Float3(force_flu), carb.Float3([0.0, 0.0, 0.0]), False
        )
        alpha_flu = vec_flu_frd(torque_frd)  # commanded angular accel (FLU)
        if self.dynamics_mode == "native":
            torque = (self._inertia * alpha_flu).tolist()
            dc.apply_body_torque(body, carb.Float3(torque), False)
        else:  # "kinematic": write the body rates directly (exact OQCRL model)
            rates_world = body2world.apply(self._omega + self.dt * alpha_flu).tolist()
            dc.set_rigid_body_angular_velocity(body, carb.Float3(rates_world))

    # ----------------------------------------------------------------- gym API
    def step(self, action):
        """Advance one control step; return the gym 5-tuple."""
        actions = np.asarray(action, dtype=np.float32).reshape(4)
        pos_old = self._pos.copy()

        self._apply_oqcrl_control(actions)
        self.world.step(render=self.render_on_step)
        self.step_count += 1
        self._update_state()

        ned = self._ned_obs_state()
        self._obs = self.course.build_obs(ned, self.target_gate)
        pos_new = self._pos

        reward = float(self.course.progress(pos_old, pos_new, self.target_gate))
        reward -= self.course.RATE_PENALTY * float(np.linalg.norm(self._omega))

        passed, collided = self.course.gate_passed(pos_old, pos_new, self.target_gate)
        self.target_gate = int(self.course.advance_gate(self.target_gate, passed))
        crashed = bool(self.course.out_of_bounds(pos_new)) or collided
        if crashed:
            reward = self.course.CRASH_REWARD
        terminated = crashed
        truncated = bool(self.step_count >= self.max_steps)
        info = {
            "gate_passed": passed,
            "target_gate": int(self.target_gate),
            "collision": crashed,
        }
        return self._obs, np.float32(reward), terminated, truncated, info

    def reset(self, seed=None, options=None):
        """Reset to a (optionally randomized) spawn; return (obs, info).

        Assumes physics is already live (caller did world.reset() + timeline.play()).
        """
        super().reset(seed=seed, options=options)
        self.step_count = 0

        g, ned = self.course.sample_spawn(self.rng, randomize=self.randomize_reset)
        self.target_gate = g
        self.motor_w = ned[12:16].copy()
        # NED-FRD spawn 16-vec -> ENU-FLU PhysX pose + velocities.
        q_isaac = quat_aero_isaac(Rotation.from_euler("xyz", ned[6:9]).as_quat())
        R = Rotation.from_quat(q_isaac)
        dc, body = self._dci(), self._body_handle()
        dc.set_rigid_body_pose(
            body,
            _dynamic_control.Transform(
                vec_enu_ned(ned[0:3]).tolist(), [float(v) for v in q_isaac]
            ),
        )
        dc.set_rigid_body_linear_velocity(
            body, carb.Float3(vec_enu_ned(ned[3:6]).tolist())
        )
        dc.set_rigid_body_angular_velocity(
            body, carb.Float3(R.apply(vec_flu_frd(ned[9:12])).tolist())
        )

        self.world.step(render=self.render_on_step)
        self._update_state()
        self._obs = self.course.build_obs(self._ned_obs_state(), self.target_gate)
        return self._obs, {}

    def render(self):
        """Render one frame."""
        self.world.step(render=True)

    def close(self):
        """No resources to release."""

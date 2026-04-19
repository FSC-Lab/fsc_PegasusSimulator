#!/usr/bin/env python
"""
| File: hover_env.py
| Description: Gymnasium environment for precision hover PPO task.
"""

import gymnasium as gym
import numpy as np
from gymnasium import spaces
from scipy.spatial.transform import Rotation

from pegasus.simulator.logic.state import State


class HoverEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 20}

    def __init__(
        self,
        world,
        vehicle,
        backend,
        physics_dt=1 / 120.0,
        control_dt=1 / 20.0,
        render_on_step=False,
    ):
        self.world = world
        self.vehicle = vehicle
        self.backend = backend
        self.physics_dt = physics_dt
        self.control_dt = control_dt
        self.decimation = int(round(self.control_dt / self.physics_dt))
        self.render_on_step = render_on_step

        # action = [collective thrust, roll, pitch, yaw]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32
        )

        # obs = pos_err(3) + vel(3) + gravity_vec(3) + body_rates(3) + yaw_sin_cos(2) + prev_action(4)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(18,), dtype=np.float32
        )

        self.target_pos = np.array([0.0, 0.0, 1.5], dtype=np.float32)

        # Hover baseline rotor speed
        self.omega_hover = 656.0

        # Action scales
        self.thrust_scale = 0.08
        self.attitude_scale = 0.04

        self.prev_action = np.zeros(4, dtype=np.float32)
        self.prev_motor_speed = np.ones(4, dtype=np.float32) * self.omega_hover

        self.steps = 0
        self.max_steps = int(12.0 / self.control_dt)

        # Success definition
        self.success_xy_thresh = 0.10
        self.success_z_thresh = 0.05
        self.success_vel_thresh = 0.15
        self.success_tilt_thresh_deg = 8.0
        self.success_hold_steps = 40  # 2 seconds at 20 Hz
        self.success_counter = 0

    def _mixer(self, action: np.ndarray):
        """
        Convert higher-level action [thrust, roll, pitch, yaw]
        into 4 motor commands for an X quad.
        """
        thrust_cmd = self.thrust_scale * action[0]
        roll_cmd = self.attitude_scale * action[1]
        pitch_cmd = self.attitude_scale * action[2]
        yaw_cmd = self.attitude_scale * action[3]

        delta = np.array([
            thrust_cmd - roll_cmd + pitch_cmd - yaw_cmd,
            thrust_cmd + roll_cmd - pitch_cmd - yaw_cmd,
            thrust_cmd + roll_cmd + pitch_cmd + yaw_cmd,
            thrust_cmd - roll_cmd - pitch_cmd + yaw_cmd,
        ], dtype=np.float32)

        target_motor_speed = np.clip(
            self.omega_hover * (1.0 + delta),
            0.0,
            2000.0,
        )
        return target_motor_speed

    def _in_success_region(self, state: State):
        pos = state.position
        vel = state.linear_velocity

        xy_err = np.linalg.norm(pos[:2] - self.target_pos[:2])
        z_err = abs(pos[2] - self.target_pos[2])
        vel_err = np.linalg.norm(vel)

        rot = Rotation.from_quat(state.attitude)
        body_z_in_world = rot.apply([0.0, 0.0, 1.0])
        tilt_deg = np.rad2deg(np.arccos(np.clip(body_z_in_world[2], -1.0, 1.0)))

        return (
            xy_err < self.success_xy_thresh
            and z_err < self.success_z_thresh
            and vel_err < self.success_vel_thresh
            and tilt_deg < self.success_tilt_thresh_deg
        )

    def step(self, action):
        action = np.asarray(action, dtype=np.float32).reshape(4)
        prev_action = self.prev_action.copy()

        target_motor_speed = self._mixer(action)

        # Smooth the motor command to avoid violent jumps
        motor_speed = 0.9 * self.prev_motor_speed + 0.1 * target_motor_speed
        self.prev_motor_speed = motor_speed.copy()
        self.backend.receive_action(motor_speed)

        for _ in range(self.decimation):
            self.world.step(render=self.render_on_step)

        state = self.vehicle.state

        in_success = self._in_success_region(state)
        if in_success:
            self.success_counter += 1
        else:
            self.success_counter = 0

        reward = self._get_reward(state, action, prev_action, in_success)

        terminated = self._is_terminated(state)
        success_done = self.success_counter >= self.success_hold_steps

        self.steps += 1
        truncated = self.steps >= self.max_steps

        self.prev_action = action.copy()
        observation = self._get_obs(state, self.prev_action)

        info = {}
        done_flag = terminated or truncated or success_done
        if done_flag:
            pos = state.position
            vel = state.linear_velocity
            rot = Rotation.from_quat(state.attitude)
            body_z_in_world = rot.apply([0.0, 0.0, 1.0])
            tilt_deg = np.rad2deg(np.arccos(np.clip(body_z_in_world[2], -1.0, 1.0)))

            info = {
                "is_success": bool(success_done),
                "final_dist": float(np.linalg.norm(pos - self.target_pos)),
                "final_xy_err": float(np.linalg.norm(pos[:2] - self.target_pos[:2])),
                "final_z_err": float(abs(pos[2] - self.target_pos[2])),
                "final_speed": float(np.linalg.norm(vel)),
                "final_tilt_deg": float(tilt_deg),
                "success_counter": int(self.success_counter),
            }

        return observation, reward, (terminated or success_done), truncated, info

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        self.success_counter = 0
        self.prev_action[:] = 0.0
        self.prev_motor_speed[:] = self.omega_hover
        self.backend.reset()

        self.world.reset()

        for _ in range(3):
            self.world.step(render=self.render_on_step)

        state = self.vehicle.state
        obs = self._get_obs(state, self.prev_action)
        return obs, {}

    def _get_obs(self, state: State, prev_action: np.ndarray):
        e_p = state.position - self.target_pos
        v = state.linear_velocity

        rot = Rotation.from_quat(state.attitude)
        gravity_vec = rot.inv().apply([0.0, 0.0, -1.0])
        omega_body = state.angular_velocity

        yaw = rot.as_euler("xyz", degrees=False)[2]
        target_yaw = 0.0
        yaw_error_sin = np.sin(yaw - target_yaw)
        yaw_error_cos = np.cos(yaw - target_yaw)

        return np.concatenate(
            [e_p, v, gravity_vec, omega_body, [yaw_error_sin, yaw_error_cos], prev_action]
        ).astype(np.float32)

    def _get_reward(self, state: State, action: np.ndarray, prev_action: np.ndarray, in_success: bool):
        pos = state.position
        vel = state.linear_velocity
        omega = state.angular_velocity

        xy_err = np.linalg.norm(pos[:2] - self.target_pos[:2])
        z_err = abs(pos[2] - self.target_pos[2])
        vel_err = np.linalg.norm(vel)
        ang_err = np.linalg.norm(omega)

        rot = Rotation.from_quat(state.attitude)
        body_z_in_world = rot.apply([0.0, 0.0, 1.0])

        # Dense shaping rewards
        xy_reward = 1.5 * np.exp(-3.0 * xy_err)
        z_reward = 3.0 * np.exp(-6.0 * z_err)
        vel_reward = 1.2 * np.exp(-1.0 * vel_err)
        upright_reward = 1.0 * max(body_z_in_world[2], 0.0)

        # Small penalties
        ang_penalty = 0.03 * ang_err
        act_penalty = 0.0015 * np.linalg.norm(action) ** 2
        smooth_penalty = 0.01 * np.linalg.norm(action - prev_action) ** 2

        reward = xy_reward + z_reward + vel_reward + upright_reward
        reward -= ang_penalty + act_penalty + smooth_penalty

        # Encourage staying inside the target region
        if in_success:
            reward += 2.0

        # Big bonus when success hold is achieved
        if self.success_counter >= self.success_hold_steps:
            reward += 20.0

        return float(reward)

    def _is_terminated(self, state: State):
        pos_err = np.linalg.norm(state.position - self.target_pos)

        rot = Rotation.from_quat(state.attitude)
        body_z_in_world = rot.apply([0.0, 0.0, 1.0])
        tilt_angle = np.rad2deg(np.arccos(np.clip(body_z_in_world[2], -1.0, 1.0)))

        is_crashed = state.position[2] < 0.05
        is_too_far = pos_err > 4.0
        is_tilted = tilt_angle > 85.0

        return bool(is_crashed or is_too_far or is_tilted)

    def render(self):
        self.world.step(render=True)

    def close(self):
        pass
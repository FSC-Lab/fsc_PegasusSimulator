#!/usr/bin/env python
"""
| File: train_hover_ppo.py
| Description: PPO training script for precision hover task.
"""

import os
import sys
from pathlib import Path
from collections import deque

import numpy as np

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.append(str(REPO_ROOT))

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import carb
import omni.timeline
import omni.usd
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback, CallbackList

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS

from fsc_aerial_manipulation.utils import add_dome_lighting

from application.rl.rl_backend import RLBackend
from application.rl.hover_env import HoverEnv


class SuccessRateCallback(BaseCallback):
    def __init__(self, window_size=100, verbose=0):
        super().__init__(verbose)
        self.success_buffer = deque(maxlen=window_size)

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", [])

        for done, info in zip(dones, infos):
            if done and "is_success" in info:
                self.success_buffer.append(float(info["is_success"]))

        if len(self.success_buffer) > 0:
            self.logger.record("rollout/success_rate", float(np.mean(self.success_buffer)))

        return True


def main():
    checkpoints_dir = REPO_ROOT / "checkpoints"
    tb_dir = REPO_ROOT / "ppo_hover_tensorboard"
    checkpoints_dir.mkdir(parents=True, exist_ok=True)
    tb_dir.mkdir(parents=True, exist_ok=True)

    pg = PegasusInterface()
    pg._world_settings["physics_dt"] = 1.0 / 120.0
    pg._world_settings["stage_units_in_meters"] = 1.0
    pg._world_settings["rendering_dt"] = 1.0 / 20.0

    pg._world = World(**pg._world_settings)
    world = pg.world
    timeline = omni.timeline.get_timeline_interface()

    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    stage = omni.usd.get_context().get_stage()

    add_dome_lighting(
        stage=stage,
        dome_path="/World/DomeLight",
        intensity=2500.0,
        exposure=0.0,
        color=(1.0, 1.0, 1.0),
    )

    backend = RLBackend()

    config = MultirotorConfig()
    config.backends = [backend]

    vehicle = Multirotor(
        "/World/quadrotor",
        ROBOTS["Iris"],
        0,
        [0.0, 0.0, 1.5],
        Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        config=config,
    )

    world.reset()
    timeline.play()

    env = HoverEnv(
        world=world,
        vehicle=vehicle,
        backend=backend,
        physics_dt=pg._world_settings["physics_dt"],
        control_dt=pg._world_settings["rendering_dt"],
        render_on_step=False,
    )

    env = Monitor(
        env,
        info_keywords=(
            "is_success",
            "final_dist",
            "final_xy_err",
            "final_z_err",
            "final_speed",
            "final_tilt_deg",
        ),
    )

    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=1e-4,
        gamma=0.995,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        n_steps=2048,
        batch_size=512,
        n_epochs=10,
        verbose=1,
        tensorboard_log=str(tb_dir),
        device="cpu",
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=100000,
        save_path=str(checkpoints_dir),
        name_prefix="ppo_hover",
        save_replay_buffer=False,
        save_vecnormalize=False,
    )

    success_callback = SuccessRateCallback(window_size=100)
    callback = CallbackList([checkpoint_callback, success_callback])

    # 先短测
    # model.learn(total_timesteps=10000, callback=callback)

    model.learn(total_timesteps=10000000, callback=callback)

    model.save(str(REPO_ROOT / "ppo_hover_policy_final"))

    carb.log_warn("Training finished, closing Simulation App.")
    env.close()
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
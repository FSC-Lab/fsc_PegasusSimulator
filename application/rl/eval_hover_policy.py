#!/usr/bin/env python
"""
| File: eval_hover_policy.py
| Description: Render a trained PPO hover policy in Isaac Sim.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.append(str(REPO_ROOT))

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import omni.timeline
import omni.usd
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation
from stable_baselines3 import PPO

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS

from fsc_aerial_manipulation.utils import add_dome_lighting

from application.rl.rl_backend import RLBackend
from application.rl.hover_env import HoverEnv


def main():
    model_path = REPO_ROOT / "ppo_hover_policy_final.zip"
    if not model_path.exists():
        raise FileNotFoundError(f"Model not found: {model_path}")

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
        render_on_step=True,
    )

    model = PPO.load(str(model_path), device="cpu")

    obs, _ = env.reset()

    max_eval_steps = 3000
    for step in range(max_eval_steps):
        if not simulation_app.is_running():
            break

        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)

        if terminated or truncated:
            print(f"[Eval] Episode ended at step {step}, info={info}")
            obs, _ = env.reset()

    carb.log_warn("Evaluation finished, closing Simulation App.")
    env.close()
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
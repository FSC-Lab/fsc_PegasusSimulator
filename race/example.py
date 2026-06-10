"""Standalone race demo: fly a policy through a figure-8 gate course (no quadsim).

Boots a SimulationApp, builds a bare World (ground + dome light), spawns the drone, and
flies the packaged policy through the course with ``race.RaceEnv``.

    HEADLESS=1 python -m race.example                 # packaged models/race_ppo.zip
    HEADLESS=1 python -m race.example /path/model.zip  # a different policy
"""

# ruff: noqa: E402 — imports are deferred until after the SimulationApp boots.
import contextlib
import os
import sys
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": os.environ.get("HEADLESS") == "1"})

import omni.timeline
from isaacsim.core.api.world import World
from pxr import Gf, UsdGeom, UsdLux

import race.dynamics as dyn
from race.config import RaceTrackConfig
from race.course import RaceCourse
from race.env import RaceEnv

_HERE = Path(__file__).parent

# Figure-8 (r = 1.5) in the OQCRL NED frame (z down; gates at z=-1.5 => 1.5 m up).
FIGURE8 = RaceTrackConfig(
    gate_pos=[
        [1.5, -1.5, -1.5],
        [0.0, 0.0, -1.5],
        [-1.5, 1.5, -1.5],
        [0.0, 3.0, -1.5],
        [1.5, 1.5, -1.5],
        [0.0, 0.0, -1.5],
        [-1.5, -1.5, -1.5],
        [0.0, -3.0, -1.5],
    ],
    gate_yaw=[1, 2, 1, 0, -1, -2, -1, 0],
    gate_yaw_unit="multiples_pi_2",
    start_pos=[1.5, -2.5, -1.5],  # 1 m behind gate 0
    gate_size=1.5,
)


def _load_policy(path):
    """Load an sb3 PPO policy (numpy 2.x -> 1.x unpickle shim for the foreign model)."""
    import numpy as _np

    sys.modules["numpy._core"] = _np.core
    for _sub in ["multiarray", "umath", "numeric", "_multiarray_umath", "overrides"]:
        with contextlib.suppress(Exception):
            sys.modules["numpy._core." + _sub] = __import__(
                "numpy.core." + _sub, fromlist=[_sub]
            )
    from stable_baselines3 import PPO

    return PPO.load(path, device="cpu")


def main():
    """Roll out the policy and report gates cleared."""
    model_path = (
        sys.argv[1] if len(sys.argv) > 1 else str(_HERE / "models" / "race_ppo.zip")
    )
    model = _load_policy(model_path)
    course = RaceCourse(FIGURE8)

    headless = os.environ.get("HEADLESS") == "1"
    world = World(physics_dt=dyn.DT, rendering_dt=dyn.DT, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = world.stage
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(1000.0)
    dome.CreateColorAttr(Gf.Vec3f(0.80, 0.85, 0.92))
    key = UsdLux.DistantLight.Define(stage, "/World/KeyLight")  # directional shading
    key.CreateIntensityAttr(3000.0)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-55.0, 0.0, 35.0))

    env = RaceEnv(
        world,
        course,
        usd_path=str(_HERE / "assets" / "iris.usd"),
        randomize_reset=False,
        render_on_step=not headless,  # render each step in windowed mode (else black)
    )
    world.reset()
    omni.timeline.get_timeline_interface().play()

    obs, _ = env.reset()
    gates = 0
    for _t in range(env.max_steps):
        if not simulation_app.is_running():
            break
        action, _ = model.predict(obs, deterministic=True)
        obs, _reward, terminated, truncated, info = env.step(action)
        gates += int(info["gate_passed"])
        if terminated or truncated:
            break

    laps = gates / course.num_gates
    print(f"gates_passed={gates} laps={laps:.2f}")
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()

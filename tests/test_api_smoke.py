#!/usr/bin/env python
"""
| File: tests/test_api_smoke.py
| Description: Lightweight import / API regression smoke tests for the RL stack.
|
| These tests boot a single headless Isaac Sim app and verify that the modern
| ``isaacsim.*`` packaging resolves the APIs the RL application depends on, that
| the Pegasus / FSC / RL modules import cleanly, and that the public RL surface
| (RLBackend, HoverEnv spaces) is intact. They also act as a regression guard
| against reintroducing the deprecated ``omni.isaac.*`` namespace.
|
| Run with:  uv run pytest tests/test_api_smoke.py -v
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# A SimulationApp must exist before any isaacsim.core.* / pegasus module is
# imported. Boot one headless app for the whole module.
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})


def teardown_module(module):
    # Closing immediately can abort with "Destroying busy TaskGroup!"; stop the
    # timeline and pump one update first so kit tears down cleanly.
    try:
        import omni.timeline
        omni.timeline.get_timeline_interface().stop()
        simulation_app.update()
    except Exception:
        pass
    simulation_app.close()


def test_world_resolves_from_modern_namespace():
    """The RL scripts now import World from isaacsim.core.api.world."""
    from isaacsim.core.api.world import World

    assert World is not None


def test_pegasus_modules_import():
    from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
    from pegasus.simulator.logic.vehicles.multirotor import (
        Multirotor,
        MultirotorConfig,
    )
    from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS

    assert PegasusInterface is not None
    assert Multirotor is not None and MultirotorConfig is not None
    assert "Iris" in ROBOTS
    assert len(SIMULATION_ENVIRONMENTS) > 0


def test_fsc_utils_import():
    from fsc_aerial_manipulation.utils import add_dome_lighting

    assert callable(add_dome_lighting)


def test_rl_backend_api():
    from application.rl.rl_backend import RLBackend

    backend = RLBackend()
    backend.receive_action([1.0, 2.0, 3.0, 4.0])
    ref = backend.input_reference()
    assert len(ref) == 4
    backend.reset()
    assert all(v == 0.0 for v in backend.input_reference())


def test_hover_env_spaces():
    from application.rl.hover_env import HoverEnv

    # __init__ only stores world/vehicle/backend and builds the spaces; it never
    # touches them, so None placeholders are sufficient to assert the contract.
    env = HoverEnv(world=None, vehicle=None, backend=None)
    assert env.action_space.shape == (4,)
    assert env.observation_space.shape == (18,)
    assert env.decimation == 6  # control_dt / physics_dt = (1/20) / (1/120)


def test_no_deprecated_omni_isaac_imports_in_rl_scripts():
    """Regression guard: keep the RL scripts on the modern namespace."""
    rl_dir = REPO_ROOT / "application" / "rl"
    offenders = []
    for script in rl_dir.glob("*.py"):
        text = script.read_text()
        if "omni.isaac" in text:
            offenders.append(script.name)
    assert not offenders, f"Deprecated omni.isaac.* import found in: {offenders}"

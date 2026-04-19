#!/usr/bin/env python
"""
| File: rl_backend.py
| Description: RL backend for Pegasus multirotor control.
"""

import numpy as np
from pegasus.simulator.logic.backends.backend import Backend
from pegasus.simulator.logic.state import State


class RLBackend(Backend):
    def __init__(self, config=None):
        super().__init__(config)
        self._action = np.zeros(4, dtype=np.float32)
        self._latest_state = None

    def update(self, dt: float):
        pass

    def update_state(self, state: State):
        self._latest_state = state

    def update_sensor(self, sensor_type, data):
        pass

    def update_graphical_sensor(self, sensor_type, data):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def reset(self):
        self._action[:] = 0.0
        self._latest_state = None

    def input_reference(self):
        return self._action.tolist()

    def receive_action(self, action):
        action = np.asarray(action, dtype=np.float32).reshape(4)
        self._action = action.copy()
"""FSC VTOL flight controllers.

Geometric hover (the safe controller), an SMC outer loop, and the NMPC family
(cascade, 6-state longitudinal, 19-state full-state), driving PX4 through
direct-actuator offboard control.

Simulator-independent: nothing here imports Isaac Sim or Pegasus. The
controllers are validated in HIL against that simulator, and the plant model
they carry is kept honest by tests/test_controller_aero_match.py on the
simulator side, but the flight code has no build or runtime dependency on it.

Only the NMPC controllers need casadi; the geometric hover path, the SMC outer
loop and the PX4/mocap bridges are numpy-only.
"""

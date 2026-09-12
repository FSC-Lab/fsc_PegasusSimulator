# NMPC quad controller (`nmpc_quad_node.py`)

Self-contained CasADi **NMPC** that flies the Isaac 4-rotor quadplane
(`fsc_cad_vtol`, ~3.4 kg, 2 front tilting + 2 rear static rotors). It reuses the
SO(3) path-following NMPC *design* from `AVTOL_Work` (multiple-shooting NLP, RK4,
ipopt, geodesic attitude cost, second-order tilt-actuator states) but with a
**fresh dynamics model of our quadplane** — it does **not** import the AVTOL_Work
package. Only external deps: `casadi`, `numpy`, `rclpy`, `px4_msgs`,
`fsc_autopilot_ros2_msgs` (all already available; `casadi` was pip-installed into
the ROS venv).

It is a standalone script (not wired into `setup.py`), so nothing existing was
modified — run it directly with `python3`.

## Model
```
state  x(22) = [ pos(3), R_bn row-major(9), beta_L, beta_R,
                 vel_world(3), omega_body(3), betad_L, betad_R ]
control u(6)  = [ T_FL, T_FR, T_RL, T_RR (N),  betacmd_L, betacmd_R (rad) ]
```
4 rotor forces (front two tilt `(sinβ,0,cosβ)`, rear two vertical), moments
`Σ rᵢ×Fᵢ` (CG-relative) + reaction torque `-spinᵢ·cQ·Tᵢ`, first/second-order tilt
servo. Params match Isaac: rotor hubs, spins FL+ FR− RL− RR+, cQ=0.12,
I=(0.20,0.23,0.36), max_thrust=18 N. Verify against `vtol.py`
`quad_tiltrotor_config()` if you retune the airframe.

## Behaviour
- **Multicopter mode**: tilt is heavily penalized (`w_beta`) so it stays ≈0 and
  the vehicle translates by pitching (like the tuned PX4 path), holding altitude.
  Tilt is a transition DOF for later.
- **Reference governor**: the tracked set-point ramps toward the target at a
  bounded velocity/yaw-rate (`sp_max_vel`, `sp_max_climb`, `sp_max_yawrate_deg`),
  fed forward as `xref` velocity + yaw rate. This is the "how fast to move" knob
  and what gives clean, non-bang-bang step responses.
- Closed-loop sim (model==truth) step responses: z 1.1 s rise / 0 % overshoot,
  x translate holding altitude (tilt ≤0.5°), yaw 90° in ~1.3 s.

## I/O
- **Sub** `/<ns>/mocap` (`fsc_autopilot_ros2_msgs/Mocap`): world pose, world
  velocity (`twist.linear`), body rates (`twist.angular`).
- **Sub** `/<ns>/fsc_autopilot_ros2/position_controller/reference`
  (`PositionControllerReference`): target position + yaw. Same topic the ground
  station / `step_response.py` already publish — so those tools drive the NMPC
  unchanged.
- **Pub** `/<ns>/fmu/in/{offboard_control_mode(direct_actuator),actuator_motors,
  actuator_servos,vehicle_command}` — flies Isaac through PX4 HIL passthrough.
- Auto offboard+arm once state is flowing (`auto_arm`, default on in the launcher).

## Run
Full stack (DDS + PX4 + Isaac + mocap + NMPC), one command:
```
scripts/start_vtol_nmpc_hil.sh pieper_machine
# knobs:  AUTO_ARM=true NMPC_RATE=50 SP_MAX_VEL=1.0 SP_MAX_YAWRATE=60 ...
```
Then drive the reference from the ground station GUI or:
```
ros2 topic pub -1 /uav_0/fsc_autopilot_ros2/position_controller/reference \
  fsc_autopilot_ros2_msgs/msg/PositionControllerReference \
  '{position: {x: 1.0, y: 0.0, z: 1.5}, yaw: 45.0, yaw_unit: 0}'
```
Or run the node alone against an already-running mocap+PX4:
```
python3 ros2/px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_quad_node.py \
  --ros-args -p px4_ns:=uav_0 -p auto_arm:=true
```

## Tuning (live via `ros2 param` only for governor/rate; weights are edit+restart)
- Sluggish / slow moves → raise `sp_max_vel`, `sp_max_climb`, `sp_max_yawrate_deg`.
- Overshoot on far steps → lower those (governor) first; then raise `w_vel`.
- Weights (`w_pos, w_vel, w_att, w_beta, w_du, ...`) live in `NMPCConfig` at the
  top of the node — edit and restart.

# 90° wing-borne cruise — 6-state pitch-moment NMPC (`nmpc6_cruise`)

The controller that cracks the rotor→wing handoff: a 6-state longitudinal NMPC
`X=[s,z,vs,vz,theta,q]`, `U=[Tf,Tr,beta,de]` that **co-optimizes the pitch moment**
(`qdot = My/Iyy`, elevator in the loop) — every prior controller failed the corridor because it
planned pitch as if directly commandable.

**Offline-validated** against the *true* plant (`build_ode`) WITH 100 ms latency + delay
compensation: hover → clean transition → **sustained tilt=90°, V=16 m/s, AoA=+6.4°, altitude
dead-flat, rear rotors OFF, front rotors 3.8 N (drag only), wing carries the weight.**

File: `ros2/px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc6_cruise_node.py`.

## Verify offline (no stack needed)
```bash
cd ros2/px4_vtol_direct_actuator/px4_vtol_direct_actuator
~/envs/isaac_env/bin/python3 nmpc6_cruise_node.py --validate     # model == true plant (err 0.0000)
FSC_TILT_CAP_DEG=90 ~/envs/isaac_env/bin/python3 nmpc6_cruise_node.py --test --vc 16 --cap 90 --ttr 18 --lat 0.10
# -> DONE z=20.0 V=16.0 tilt=90 AoA=6.4 rotors=3.8N
```

## Fly in HIL
1. Launch the stack (the node starts STANDBY on window `6f-nmpc6`):
   ```bash
   NMPC6_VC=16.0 NMPC6_CAP=90.0 NMPC6_TTR=18.0 bash scripts/start_vtol_smc_hil.sh --here pieper_machine
   ```
   Env: `NMPC6_VC` (cruise speed), `NMPC6_CAP` (tilt cap, 90 = full pusher), `NMPC6_TTR` (transition
   time s), `NMPC6_LAT` (delay-comp, match the loop latency ~0.10).
   **First launch JIT-compiles the solver (~2 min) then CACHES it** (`~/.cache/nmpc6_cruise_*.casadi`,
   Function.load 0.1 s after) — the node logs `building…` then `NMPC built.` **Wait for `NMPC built.`
   before engaging.** Solve time is then ~9 ms (well inside the 25 Hz budget).
2. **Take off on the safe controller first** (Safe Hover / `geometric`) to the cruise altitude.
3. Engage the wing-borne NMPC — either the GS button **"✈ 6-state Wing-borne (90°)"** (Trajectory
   tab, Controller row), or:
   ```bash
   ros2 topic pub -1 /uav_0/ground_station/active_controller std_msgs/msg/String "{data: nmpc6_cruise}"
   ```
   On engage it snapshots the heading + altitude, settles briefly, then transitions to the 90°
   wing-borne cruise and flies forward along the heading holding altitude.
4. To recover / abort: switch back to **Safe Hover (geometric)** — but note a hard STOP from full
   wing-borne cruise is not survivable (see the main runbook); decelerate first or hand off gently.

Node log: `NMPC6 fly s=.. z=.. V=.. tilt=.. AoA=.. th=.. ok=.. ..ms | T=[Tf,Tr] de=..`.

## Structure / tuning (live `ros2 param set /nmpc6_cruise_control ...`)
- Longitudinal NMPC drives `Tf,Tr,beta,de`; a lightweight lateral stabilizer holds roll=0 + yaw=heading
  via differential thrust + aileron/rudder (`kR_roll,kW_roll,kR_yaw,kW_yaw,k_ail,k_rud`).
- The 5 keys are baked into the NMPC (see the code header + [[nmpc6-wing-borne-solved]] memory):
  pitch dynamics, state bounds on `theta,q`, reference-feedforward fallback on solve failure, SOFT
  tilt guide (`w_beta=12`, self-limits the corridor), delay compensation.

## Gotchas
- Aero constants import from `nmpc_so3_transition_node` so the model can't drift from the sim plant.
- The node survives teardowns — add `nmpc6_cruise` to `/tmp/kill_hil.sh`'s `pgrep -f` pattern before a
  relaunch (like `nmpc_so3_cascade`).
- Longitudinal only: large lateral disturbances are held by the simple SE(2) stabilizer, not a full
  lateral NMPC — keep the reference roughly straight-line.
- Take off on `geometric` first; the wing-borne NMPC engages from a hover/low-speed forward state.

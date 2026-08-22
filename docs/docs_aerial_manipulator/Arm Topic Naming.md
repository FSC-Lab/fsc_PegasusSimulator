# Arm ROS 2 Topic Naming

Convention for every ROS 2 topic, service and parameter belonging to the
OpenMANIPULATOR-X arm, across all backends: Isaac Sim, Gazebo, and real
hardware.

Three repos share this contract:

| repo | role |
|---|---|
| `fsc_PegasusSimulator` | the Isaac plant (servo emulation) |
| `fsc_open_manipulator` | the ros2_control stack (controllers, GUI, hardware) |
| `fsc_autopilot_ros2` | consumer (the geometric node's arm feedforward) |

The scheme exists so that **one arm stack runs unchanged on the bench and on
the drone, in simulation and on hardware**. Every rule below serves that.

---

## 1. The rule: the prefix names the OWNER

Every topic sits under a namespace naming the **package that owns it**, the
same way the flight stack owns `/uav_0/fsc_autopilot_ros2/...`. Reading a
topic name tells you which subsystem produced it, with no lookup:

| prefix | owner | on real hardware |
|---|---|---|
| `<veh>/fsc_open_manipulator/…` | the real arm stack (ros2_control) | **identical** |
| `<veh>/isaacsim_manipulator/…` | the simulator's servo emulation | **absent** |
| `<veh>/fsc_autopilot_ros2/…` | the flight controller | identical |

The split between the first two is decided by one question:

> **Does this topic still exist, unchanged, when the real arm is plugged in?**

Yes → `fsc_open_manipulator/`. No, because a physical bus replaces it →
`isaacsim_manipulator/`. The second group stands in for the Dynamixel/U2D2
serial link, so on hardware it is simply *absent* — that absence is the whole
sim/hardware distinction, expressed structurally.

**An application must never subscribe to `isaacsim_manipulator/`.** It works
in sim and silently stops working on the real arm. This is the one rule worth
enforcing in review.

### Corollaries

1. **One physical quantity → one topic name.** Two topics meaning the same
   thing is a defect even when both work.
2. **Namespace, don't prefix.** `fsc_open_manipulator/arm_position_controller`,
   not a flat underscore name — namespaces remap as a group.
3. **Direction is from the plant's view.** `*_commands` in, `*_states` out.
   Never bare `cmd`/`state`.
4. **Standard message → standard name.** `joint_states` for
   `sensor_msgs/JointState`; RViz, rqt and ros2_control work for free.
5. **Names must agree in UNITS, not just in spelling** — see §4.

### Known tradeoff

`isaacsim_manipulator` names the **backend**, which the owner-prefix rule
otherwise avoids. It is safe only because those topics are sim-only by
definition. If a Gazebo bridge is ever added it needs its own
`gazebo_manipulator/` prefix and its own value for the bridge's
`joint_states_topic` parameter — a backend-neutral name would not have.
Accepted deliberately: the prefix is instantly readable, and there is exactly
one simulator today.

---

## 2. Layout

```
<veh>/fsc_open_manipulator/     ← the real arm stack; same names on hardware
    joint_states                broadcaster — THE measured state
    joint_desired_states        commanded state (GUI plots it vs measured)
    dynamic_joint_states        per-interface view, same data
    arm_position_controller/    target_joint_positions, sine_command, go_home
    gripper_controller/         hardware & Gazebo only; absent on the AM plant
    controller_manager

<veh>/isaacsim_manipulator/     ← EXISTS ONLY IN SIMULATION
    joint_states                plant → IsaacTopicSystem::read()
    position_commands           IsaacTopicSystem::write() → plant
    effort_commands             IsaacTopicEffortSystem::write() -> plant (torque mode, 2026-08-22)
```

`ros2 topic list | grep fsc_open_manipulator` is the arm's real interface.
`grep isaacsim_manipulator` is exactly what vanishes on hardware.

The whole first group comes from **one launch argument** —
`namespace:=uav_0/fsc_open_manipulator` — because the config yaml uses `/**`
wildcard keys (§3). On the bench, `namespace:=fsc_open_manipulator` gives the
same tree without a vehicle.

---

## 3. Namespaces: `/**` wildcard, launch owns the value

Config yamls **must** use wildcard node keys so one file serves every
namespace. The launch `namespace` argument is the single source of truth.

```yaml
/**/controller_manager:
  ros__parameters:
    update_rate: 100

/**/arm_position_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4]
```

Bench: `namespace:=fsc_open_manipulator` → `/fsc_open_manipulator/joint_states`.
Flight: `namespace:=uav_0/fsc_open_manipulator` →
`/uav_0/fsc_open_manipulator/joint_states`.
Same file, no edit.

### The trap

The wildcard must be **flat**. The nested shape parses as valid YAML and then
matches nothing — parameters silently stop applying, `joints` comes back
empty, and the controllers fail to configure:

| form | result |
|---|---|
| `/**/controller_manager:` → `ros__parameters:` | ✅ works |
| `/**:` → `controller_manager:` → `ros__parameters:` | ❌ *Parameter not set* |

Verified empirically 2026-08-15 against a live node at both `/uav_0/arm` and
`/arm`; only the flat form resolves. This matters because the nested shape is
what a hardcoded `/uav_0:` key looks like, so the natural edit
(`/uav_0:` → `/**:`) produces the broken form.

---

## 4. Units: the one thing a shared name cannot guarantee

Tier A promises "identical across backends". For `effort` on `joint_states`,
that promise is currently **false**:

| backend | `joint_states.effort` |
|---|---|
| Isaac (05 servo emulation) | N·m, applied joint torque |
| Hardware (Dynamixel) | **raw counts** (XM430-W350: 1 count ≈ 2.69 mA) |
| Gazebo | always 0.0 (position commands applied kinematically) |

Documented at `open_manipulator_x_custom_controller/config/
position_controller_hardware.yaml`.

Nothing reads `effort` today — the geometric node's `armff` uses `position`
only — so this is latent. It stops being latent with torque mode, since the
real arm runs its Dynamixels in **current mode**.

> **Rule.** Where a backend cannot produce the canonical unit, convert it at
> the hardware interface, or give the topic a different name. A shared name
> with different units is worse than two names, because it defeats the one
> check a reader can make.

The fix is conversion inside the Dynamixel `SystemInterface` (counts → N·m).
Until then, treat `effort` as backend-specific.

---

## 5. Ordering: match by name, never by index

The two `joint_states` publishers do not agree on array order:

| publisher | `name` order |
|---|---|
| `joint_state_broadcaster` | `[joint2, joint3, joint1, joint4]` |
| Isaac servo bus (05) | `[joint1, joint2, joint3, joint4]` |

Every current consumer matches by name — Isaac 05, `IsaacTopicSystem`, and
the geometric node (`std::find` over `msg->name`) — so today's code is
correct. A position-indexing consumer would silently scramble the arm model
with no error. Match by name.

---

## 6. Migration status

| step | scope | status |
|---|---|---|
| 1 | Servo-bus rename → `isaacsim_manipulator/*` (Isaac 05, bridge URDF/plugin/launch, launcher gate, docs) | **done** 2026-08-15 |
| 2 | `armff_joint_topic` → `fsc_open_manipulator/joint_states` + corrected guidance | **done** 2026-08-15 |
| 3 | `/**` flatten, `position_controller_isaac_aerial.yaml` | **done** 2026-08-15 |
| 4 | Move the Isaac stack under `fsc_open_manipulator/` (launch default, launcher `ARM_NS`, ground station `__ns`) | **done** 2026-08-15 |
| 5 | `/**` + `namespace` arg for the other 9 configs and their launches (Gazebo + hardware) | todo |
| 6 | Effort unit conversion in the Dynamixel `SystemInterface` | todo, before HARDWARE torque mode (the SIM torque chain shipped 2026-08-22 is all-N·m end to end and does not need it) |

Steps 1–4 are behaviour-preserving *in structure*: step 3 was diffed
parameter-by-parameter against the previous file (only difference: the
separately-made `measured_states_topic` change), and steps 1, 2 and 4 move
producer and consumer together. **Step 4 does change every arm topic's path**,
so it is the one to re-verify on the next flight.

### Notes for step 5

- The 9 non-Isaac configs are root-level with **absolute** logging topics
  (`/joint_desired_states`); the hardware launches have no `namespace`
  argument at all and hardcode `/controller_manager` in their spawners. Adding
  the wildcard and the argument is one edit per pair.
- The bench then runs `namespace:=fsc_open_manipulator`, giving the same
  topic tree as the drone minus the vehicle prefix — the point of the whole
  scheme.
- The geometric autopilot node needs **no code change** at any step;
  `armff_joint_topic` is a parameter.
- Step 4 invalidates the topic names recorded in
  `open_manipulator_x_custom_controller/results/bags/`. Decide whether those
  get replayed before renaming.

---

## 7. Scope

Three launchers have an arm ROS interface: the geometric and geometric+L1
pair (`start_t650_aerial_manipulator_geometric_direct_actuator_sitl.sh` /
`start_t650_aerial_manipulator_geometric_L1_adaptive_sitl.sh`, both running
`05_…_ros2_arm_hold.py` with the POSITION-mode stack) and the whole-body
launcher (`start_t650_aerial_manipulator_whole_body_direct_actuation_sitl.sh`,
running `06_…_ros2_arm_torque.py` with the TORQUE-mode stack —
`IsaacTopicEffortSystem` + `ExternalTorqueController`, effort on
`isaacsim_manipulator/effort_commands`, all N·m; Command.md §7.14).

`start_t650_aerial_manipulator_direct_actuator_sitl.sh` and
`start_t650_aerial_manipulator_baseline_sitl.sh` both run
`04_px4_direct_t650_aerial_manipulator_hold.py`, which holds the arm entirely
in-process and publishes **no arm topics**. Nothing here affects them.

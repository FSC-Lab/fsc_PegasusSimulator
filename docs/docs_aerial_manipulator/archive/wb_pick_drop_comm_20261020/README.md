# 2026-10-20 pick-and-drop: does the PX4 <-> Orin link still drop?

Bag: `docs/experimental_data_ros2_bag/1020 - T650-AM whole-body Pick&Drop [Success]-*/…/flight_wb_l1_4d_pick_and_drop_20261020_182234`
(108 s, one full pick-and-drop mission). Hardware changed since 10-19: new antennas, new PX4 ↔ Orin
Ethernet cable, reinforced power distribution board. Follow-up to `../wb_pick_drop_wifi_20261019/`.

## Verdict

**No communication loss anywhere in this flight.** One clean flight does not prove the fault is gone:
the cut was intermittent (2 in-flight occurrences in ~15 hardware bags since 10-09).

| link | 10-20 result |
|---|---|
| PX4 → Agent → Orin (`/fmu/out/sensor_combined`, PX4 clock) | 43 221 samples at 400.0 Hz, **0 missing**, largest step 2.6 ms |
| same, laptop receive time | largest gap 12 ms; `vehicle_attitude` / `vehicle_odometry` 12 / 18 ms |
| timesync RTT | median 0.76 ms, max 1.5 ms; offset span 2.8 ms (no session re-establishment) |
| Orin → laptop WiFi (joint_states, wb debug, actuator_motors) | full rate, largest gap 14 ms; delay p99 2.0 ms, max 9.2 ms (10-19: 44.7) |
| laptop → Orin WiFi (mocap → `fmu/in/vehicle_visual_odometry`) | 120 Hz, largest gap 22 ms |
| flight node | no feedback-lost / stale warning; PX4 never reported offboard loss or failsafe |

**The 10-19 cut moment was exercised and held:** the cut came as the basket's weight came on at the
lift. Here the lift (exit_pick 41.8 s) drew the same electrical load — battery peak 25.0 A / 592 W
against 24.7 A / 584 W at the 10-19 cut — with no gap.

**Small stalls are gone too.** `tools/eth_loss.py` over every bag 10-09 … 10-20: earlier flights all had
1–7 stalls of 20–75 ms (one ~38 ms is PX4 closing its ulog after disarm), 10-19 PD 8 183 missing
samples. 10-20: 0 stalls, 0 missing — but this bag ends 0.2 s after disarm, so the ulog-close stall is
outside it. Earlier flights without any in-flight stall exist as well (10-10, 10-14, 10-15).

## Other things seen

- Mission completed. Pilot took STAB at 105.87 s during the landing (z 0.39 m). The node reverted to SAFETY
  in the same tick and streamed **0** `actuator_motors` after it (the 10-14 motor-stop fix working).
  Touchdown at ≤ 6° tilt, disarmed 108.03 s.
- Allocator thrust-saturation warnings (−0.2 … −5.8 N, roll/pitch ≤ 0.16 N·m) during the lift and the
  carry: these are actuator limits, not communications.

## Tools

```bash
set +u; source /opt/ros/humble/setup.bash; source ~/ros2_ws/rosdeps/local_setup.bash; source ~/ros2_ws/install/setup.bash
/usr/bin/python3 ../wb_pick_drop_wifi_20261019/tools/extract_timing.py "<bag dir>" npz/pd.npz
/usr/bin/python3 tools/link_check.py npz/pd.npz          # gaps per link, PX4 status, node mode, rosout
/usr/bin/python3 tools/load_and_latency.py               # lift load + WiFi delay, 10-19 vs 10-20
(cd ../../../experimental_data_ros2_bag && /usr/bin/python3 ../docs_aerial_manipulator/archive/wb_pick_drop_comm_20261020/tools/eth_loss.py)
```

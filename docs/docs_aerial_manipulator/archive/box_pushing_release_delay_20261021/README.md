# Release sooner after the push: the arm GS's automatic exit 1.0 → 0.2 s (2026-10-21)

Fix 1 of the 10-21 hardware report (`../wb_box_pushing_flight_20261021/`). In that flight the q3
guard tripped **0.78 s after the push ended**. The jaws were still closed and the claw still
held in the room, because the arm GS starts Exit To Push only `push_pull_auto_exit_delay_s`
(1.0 s) after the push. The arm's swing has a ~0.9 s half-period, so a 1.0 s wait leaves a
full swing extreme with the box clamped.

## The change

- **`utils_custom_ground_station` (fsc_open_manipulator):** `push_pull_auto_exit_delay_s`
  default **1.0 → 0.2 s**, in `push_pull_panel.cpp` (the `declareOr` default and the button
  tooltip) and `push_pull_panel.hpp` (the member initialiser and the header comment). The
  hardware station passes no value for it, so the code default is what flies. A launch
  `-p push_pull_auto_exit_delay_s:=…` still overrides it, and 0 still means "by hand only".
- **Laptop: rebuild `utils_custom_ground_station`.** Nothing on the Orin changes.
- **Workflow page** (`~/Documents/box_pushing_workflow`): the text says 0.2 s and the lamp
  picture was re-rendered (old copies in `previous/`).

## How it was tested

- **The real station panel, offscreen** (`../box_pushing_hardware_20261019/tools/gs_grasp_check.sh`):
  - **new case X7**: the panel with the parameter undeclared, i.e. its own default. Nothing at
    0.12 s, the lamp names 0.2 s, and release → open → exit all done by 0.62 s;
  - every earlier case still passes.
  - The fake planner's step times now match the 0.10 m plan, so the workflow picture shows
    real numbers.
- **Isaac, with the station itself running the exit, as on hardware.**
  - New driver mode `pl_mission.py --gs-exit`. `run_pl.sh` now takes `ARM_GS_AUTO_EXIT_DELAY`
    from the environment (default 0.0, so every other campaign is unchanged).
  - Task = the 10-21 hardware flight: 240 g, 0.10 m at 21 mm/s, raw mocap, guard tilt 10°,
    abort release hold 1.5 s.
  - Six flights, interleaved: `tools/chain.sh`, `tools/fly.sh`; scored by
    `tools/release_metrics.py` → `release_metrics.txt`.
- **A harness bug found on the way, fixed:** after the close the driver kept republishing the
  close target on the gripper bus, which fought the station's open. Isaac's gripper server
  then called the open "stalled" and the station correctly refused to climb. The run is kept
  as `runs/x_d02_1_driverheld.*`. With `--gs-exit` the driver now stops publishing at the push
  end.

## Results

| | station delay 0.2 s (3 flights) | 1.0 s (3 flights) | hardware 10-21, 1.0 s |
|---|---|---|---|
| ended with the normal Exit | **3/3** | 3/3 | no: q3-guard abort |
| jaws start opening after the push | **0.22–0.24 s** | 1.02–1.05 s | 0.81 s (the abort's open) |
| lowest q3 while still clamped | 23.4–28.2° | **16.8**–25.3° | 4.5° |
| highest q2 while still clamped | 33.6–37.6° | 35.9–**44.5**° (limit 45) | 45.1° |
| lowest q3, push end → exit end | 23.4–28.2° | 16.8–29.1° | −7.3° |
| arm swing (q2−q3)/2 p-p, same window | 9.2–12.7° | 12.4–20.2° | 38.8° |
| box moved after the push | +0.9 to +4.7 mm | +3.2 to **+22.7** mm | −17 mm (pulled back) |

- **0.2 s works.** The station's timer, release and open ran as on hardware, every exit was
  normal, and nothing got worse.
- **The 1.0 s wait showed the hardware mechanism once in Isaac too.** In `d10_3`, inside the
  clamped second, the swing took q2 to 44.5° (0.5° from the real limit) and q3 to 16.8°, and
  it pushed the box 22.7 mm further. With 0.2 s, no run went below 23° or above 38°.

## What it cannot show

- **Isaac's swing is half the hardware's**, so this does not prove the 10-21 trip would have
  been avoided.
- **On hardware the jaws take ~0.5 s to open** (both opens on 10-21), and the release blends
  the claw's room-hold out over 1.0 s. With 0.2 s the jaws are fully open about 0.75 s after
  the push, and the climb starts on that report. The 10-21 trip came at 0.78 s.
- So 0.2 s closes most of the window, but not with a wide margin. **Fix 5 of the report**
  (damping the swing) is still what removes the cause.

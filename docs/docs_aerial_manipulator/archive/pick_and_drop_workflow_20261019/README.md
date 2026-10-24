# 2026-10-19 Pick-and-Drop workflow page — arm-GS snapshots

The pictures of `~/Documents/pick_and_drop_workflow/pick_and_drop_workflow.html` (the section the user
pastes into the lab's Indoor Experiment Workflow Google Doc), parallel to
`~/Documents/box_pushing_workflow/`. That folder's `build_section.py` builds the page from the PNGs beside it.

The arm-GS pictures are the REAL Pick & Drop tab, not mock-ups: `tools/snapshots.sh` starts the real
`whole_body_trajectory_planner` on the HARDWARE `..._t650_pick_and_drop.yaml` (hardware launch arguments),
the real `PickPlacePanel(node, true)` offscreen (`tools/gs_harness/gs_pnd_snapshot`), and a loopback rig
(`tools/pnd_rig.py`) on ROS domain 77 under `/uav_pndshot` — no Isaac, no PX4. The rig follows the
operator's workflow and presses the tab's own buttons: Drop row Get with the basket on the cover, Pick row
Get + the hand edit (x + 0.01, yaw 0), Adjust with the vehicle on its start mark (+0.11, −0.07), DIRECT,
Plan, then every leg. The basket hangs from the claw after the grasp (10-15 D-1's offset) so the drop aim
and the automatic drop run as in flight; only the gripper is emulated (answered 0.7 s after each request).
The tab's automation ran end to end on 2026-10-19: Exit To Pick after the hook grasp, Ready To Drop
"aimed at the basket", the planner's automatic release 2.6 s after the leg, Exit To Drop on the OPEN
report, gripper closed after the exit, COMPLETE.

    # build the harness once (after building utils_custom_ground_station)
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    cmake -S tools/gs_harness -B /tmp/pnd_build && cmake --build /tmp/pnd_build -j
    bash tools/snapshots.sh /tmp/pnd_build/gs_pnd_snapshot /tmp/pnd_shots
    PYTHONNOUSERSITE=1 /usr/bin/python3 -I tools/crop.py /tmp/pnd_shots ~/Documents/pick_and_drop_workflow <isaac frames dir>
    cd ~/Documents/pick_and_drop_workflow && python3 build_section.py

`crop.py` places every red box from the widget rectangles saved beside each grab, so a station layout
change moves the boxes with it. The Isaac frames (`isaac_1.png`, `isaac_75.png`) are the left half of
`~/Videos/Aerial Manipulator - Isaac Sim - Whole-Body Pick and Drop.mp4` at 1 s and 75 s (the scene labels
there still say "Place Start" / "Place"; the tab says Drop).

Page text vs the Google Doc's draft (which was copied from the Pick-and-Place section): Drop row kept as
measured (the doc's x − 0.02 predates the 10-16 basket aim), arch → fin (Box_Assembly holder), "Safety
Margin" → Vertical / Side / Drop / Back Margin, carry pose = pick pose (the arm no longer moves at Drop
Start), hover ≥ 6 s at Drop Start for the aim, automatic drop and Exit To Drop, landing at 0.35 m.

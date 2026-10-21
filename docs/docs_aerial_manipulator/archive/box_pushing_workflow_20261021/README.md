# Box-pushing workflow page: pictures refreshed (2026-10-21)

The page lives in `~/Documents/box_pushing_workflow/` (`build_section.py` writes the HTML, headless
Chrome prints the PDF). On 2026-10-21 its workflow section got the latest pictures:

| picture | from | what changed |
|---|---|---|
| `isaac_overview.png` | live windowed Isaac flight, frame 14:14:01 (`shots/f_141401.png`) | the table at its new place, (0.60, 0.50) corner; after Plan, drone at the 1.30 m takeoff hover, all six waypoint labels |
| `isaac_push.png` | same flight, frame 14:14:13 (`shots/f_141413.png`) | the claw on the fin before the push, new layout |
| `gs_auto_exit.png` | `tools/gs_wf_snapshot.sh` (real Box Pushing panel, fake planner) | today's step times: Push 6.3 s (0.10 m at 21 mm/s + 1.5 s hold), Land 7.4 s (was 25.5 / 5.1 s) |
| `gs_panel.png`, `gs_rows.png` | unchanged: the 13:43 real-planner renders (latest GS build, Mocap block included) | — |

The overview caption in `build_section.py` now describes the new picture (Land Start / Land read
(−0.50, −0.07) because the simulated vehicle spawns off its start mark and Adjust shifts them).
The old pictures, HTML, PDF and builder are in `shots/previous/`.

## How

- **Isaac frames:** `tools/isaac_grab.sh` grabbed the simulator window every 2 s with
  `gst-launch-1.0 ximagesrc xid=<window>` (no ROS, no Isaac API) while the user flew the mission
  windowed; the pictures are the viewport crop `(52, 30, 1000, 585)` = 948 × 555, the old size.
  The window id is hard-coded: find it with `xwininfo -root -tree` on `DISPLAY=:1`.
- **GS crop:** `tools/gs_wf_snapshot.sh` builds `gs_wf_harness` outside the repo (links the station's
  `libjoint_plot_core.a`) and runs it offscreen on ROS domain 81: a fake planner publishes the
  push-and-pull parameters, `push_pull/info` (77 values, today's step times and goals) and the
  FLYING push → DONE push edge, and a healthy raw-mocap guard message; the panel's own
  auto-exit countdown is grabbed and the Mission row cropped. No planner node runs, so nothing
  matches the stacks' `pgrep -f` guards.

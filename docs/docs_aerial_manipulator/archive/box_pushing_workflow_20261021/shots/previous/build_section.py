"""Build the Box-Pushing Workflow section as one HTML page to copy into the Google Doc.
Images are embedded (data URIs) so a select-all / copy / paste carries them along."""
import base64, json, os

D = os.path.dirname(os.path.abspath(__file__))


def img(name, width):
    b = base64.b64encode(open(os.path.join(D, name), "rb").read()).decode()   # the pictures sit next to this script
    return f'<p><img src="data:image/png;base64,{b}" width="{width}"></p>'


INTRO = ("Start everything as in the runbook's <b>Whole-body - T650-AM - L1-Adaptive-4D-Impedance</b> section, "
         "with MOCAP <b>uav_0 obj_0</b> and the flight stack <b>[Box-Pushing]</b> (raw mocap; the _fused stack "
         "refuses box pushing). Unless a step says otherwise, every button is on the arm GS's "
         "<b>Box Pushing</b> tab.")

# (title, body or None, [sub-bullets], picture or None)
STEPS = [
    ("Table and box placement.", None, [
        "Table: length along y, <b>1.20 m</b> ahead of the start mark. Put the reference leg (red circle: front-left, "
        "seen from the start) with its outer corner at <b>(0.60, 0.50) m</b> in the mocap frame; the top is "
        "<b>0.80 m</b>. Check both with a marker: the planner keeps the drone clear of the table at "
        "x 0.60–0.75, y −0.50–0.50, top 0.80.",
        "Box: on the bare table (no mat: the push expects about 1.2 N of friction), centred across the table, "
        "its back (+y) face <b>0.10 m</b> from the table's +y end, the fin along y. Its centre is at "
        "<b>(0.675, 0.28) m</b>. Mass <b>240 g</b> for the first test (the box emptied; <b>500 g</b> full), "
        "with the bracket (weigh it: within ±50 g).",
     ], ("table_leg.png", 300)),
    ("Box rigid body in Motive.", None, [
        "Place the box as above, square to the room (yaw ≈ 0), and create the rigid body <b>obj_0</b> from its "
        "markers. Use an asymmetric marker layout: Get refuses a flipping pose.",
        "Move the pivot to the <b>box's centre</b>: centred in x and y, <b>50 mm above the table top</b>.",
        "Axes: x across the fin (the jaws close along x), y along the fin and the box's long side, towards the "
        "table's +y end, z up.",
        "If the pivot cannot sit at the box's centre, set EE Offset z = 0.233 − the pivot's height above the "
        "table top.",
     ], None),
    ("Check the table with the box (Get).", None, [
        "Put the box near the table's <b>+y end</b>, square to the table and centred across it (about 5 mm over "
        "each edge: the box is 160 mm wide, the table 150 mm), and press <b>Get</b>. Then move it near the "
        "<b>−y end</b> the same way and press Get again.",
        "<b>x</b> must read the same at both ends, about <b>0.675</b>. 1 cm of difference over the 0.76 m between "
        "the two places is 0.75° of table rotation; more than about 2 cm: turn the table.",
        "<b>yaw</b> close to 0 at both ends (within ±1°): the box sits square to the room as well as to the table. "
        "The push follows the box's yaw, so this is what keeps it along the table.",
        "<b>z − 0.050</b> is the table-top height (0.800 expected). The same at both ends means a level table; a "
        "different height must go into <i>push_pull_table</i>'s last number.",
        "Put the box back at its start place, centre (0.675, 0.28), square to the table. The Get in step 5 is the "
        "one the planner uses.",
     ], None),
    ("Bench check of the grip (first flight; vehicle on the ground, disarmed).", None, [
        "Arm GS, <b>Gripper</b> column: <b>OPEN</b>. Hold the box so the fin sits between the pads, about 25 mm "
        "below the fin top, then <b>CLOSE</b>.",
        "<b>Opening (m)</b> must settle between <b>−0.010 and −0.005</b> (the 3 mm pick-and-drop fin closed to "
        "−0.0097). Wider than −0.005 means the pads are on something else: the tab would call it a wrong grip "
        "and not push.",
        "Pull the box gently along the fin: the pads must hold it (the push needs about 1.2 N). Then <b>OPEN</b>.",
     ], None),
    ("Box pose → Get.",
     "With the box back in place, press <b>Get</b>: the Box (obj_0) row fills with x, y, z, yaw (editable; "
     "press Enter to send). Get refuses a frozen or flipping pose, or a moving box: fix the rigid body and press "
     "again. Get again whenever the box is moved.", [], ("gs_rows.png", 452)),
    ("Drone thrust test.", None, [], None),
    ("Start → Adjust.",
     "Vehicle on its start mark, <b>(−0.60, 0) m</b>, facing along +x (yaw ≈ 0). Adjust shifts the Start, Land Start and Land rows by the "
     "measured offset; the box is measured, never shifted.", [], None),
    ("Check the settings.",
     "Press a button only if you change its value.", [
        "EE Offset <b>[0, 0, 0.183]</b>. From the box centre (obj_0's origin) to where the claw grips the fin, "
        "in the box's frame: 25 mm below the fin top, 233 mm above the table.",
        "Vertical Margin <b>0.15</b>. How far above the grasp point the claw waits at Go To Start.",
        "Side Margin <b>0.12</b>. How far behind the grasp point the claw waits; Ready To Push slides it in "
        "from there.",
        "Push Distance <b>0.10</b> for the first test (<b>0.50</b> the full push). How far the box slides "
        "along the table (−y). The <b>Monitor</b> under the points shows the push time, the contact time and "
        "the mean speed this gives (amber until you press Push Distance): the time is the distance at the "
        "planner's mean speed, <b>21 mm/s</b> (<i>push_pull_push_mean_speed</i>, the one setting, in the yaml), "
        "so 0.10 m takes <b>4.8 s</b> and the jaws hold the box <b>6.3 s</b>. The numbers are <b>green</b> inside "
        "the tested range (0.10–0.50 m, 8–30 mm/s) and <b>red</b> outside it, with a note saying what to retype.",
     ], None),
    ("Take off.",
     "In SAFETY to <b>1.3 m</b> from the drone GS and wait for a steady hover. (Not 1.0 m: from there Plan "
     "refuses Go To Start.)", [], None),
    ("Switch to DIRECT.",
     "From the drone GS Controller tab. Wait until the tab's status reads <i>NOT PLANNED: press Plan</i>. "
     "Never switch to DIRECT on the ground with props on.", [], None),
    ("Plan.",
     "This dry-runs all six steps and draws the path. Wait for <i>READY next=go_to_start</i>. If it says "
     "INFEASIBLE, read the reason before going on (a box-on-table refusal usually means obj_0's yaw is wrong).",
     ["The <b>Monitor</b> under the points, <i>now</i> against <i>plan</i>, top to bottom: <b>Push Time</b>, "
      "<b>Contact Time</b> (the 1.5 s hold + the push), <b>Mean Speed</b>, <b>Box Speed</b>, <b>Box Travelled</b> "
      "(the box from mocap obj_0) -- live during the push, then the push's numbers until the next push or Reset -- "
      "and last <b>Table Offset</b>: the landing gear's height above the table top, <i>plan</i> the lowest anywhere "
      "the plan takes it over the table (about 0.12 m at the grasp), <i>now</i> the live value; green ≥ 0.05 m "
      "(what the planner keeps), amber below, red under the top, grey while the gear is not over the table.",
      "Right of the Monitor rows, the <b>Mocap</b> lamp (the raw-mocap guard on the Orin): green <b>MOCAP</b> "
      "is the only state to fly in. <b>EKF2 BRIDGE</b> (amber) means the drone's mocap paused, froze or jumped "
      "and the odometry is EKF2's for the moment; with the claw on the box a fault longer than 0.15 s aborts the "
      "push by itself (jaws open, climb, arm home) and every step waits. <b>LOST -- LAND WITH THE RC</b> (red, "
      "flashing, beeping): take over in STAB within about 2 s and land -- EKF2 alone drifts tens of centimetres "
      "in a few seconds. <b>NO LINK</b>: the Orin's guard status stopped arriving."], None),
    ("1 Go To Start.",
     "The drone flies to the table's left (+y) end and turns right to <b>−90°</b>; the claw stops one Side "
     "Margin behind and one Vertical Margin above the grasp point. Wait for <i>DONE go_to_start</i>.", [], None),
    ("2 Ready To Push.",
     "The gripper opens; once it reports open, the claw slides down-forward onto the fin. Wait for "
     "<i>DONE ready</i>.", [], None),
    ("3 Push (press once).",
     "The gripper closes once the claw is within 2 cm of the grasp point and centred across the fin to 3 mm. "
     "Once it grips (on hardware: the close completes, no wider than −0.005 m) the push starts by itself: the "
     "contact line turns purple, the drone holds 1.5 s, then slides the box the Push Distance at <b>21 mm/s</b> "
     "(0.10 m: <b>4.8 s</b>); the Monitor shows the box's travel and speed against the plan. If the "
     "lamp says wrong grip, nothing in the jaws or no result, nothing moves: press 4 Exit To Push (or ABORT); "
     "after a failed grasp the exit never starts by itself.",
     [], ("isaac_push.png", 452)),
    ("4 Exit To Push (starts by itself).",
     "At <i>DONE push</i> the lamp turns purple and the exit starts by itself <b>1 s</b> later: the claw's "
     "reference is released first, then the gripper opens; when it reports open, the claw climbs 0.20 m and the "
     "arm folds home. To go sooner press 4 Exit To Push; to stop it press ABORT.", [
        "If the lamp says the gripper did not report OPEN, or push_pull/release was not available, nothing "
        "climbs: check the jaws, then press 4 Exit To Push again, or ABORT.",
     ], ("gs_auto_exit.png", 600)),
    ("5 Go To Land Start.",
     "One transition back to Land Start, <b>1.3 m</b> above the start mark, clear of the table, turning clockwise to <b>−360°</b> "
     "(about <b>24 s</b>).", [], None),
    ("6 Land.",
     "Straight down from Land Start (1.3 m) to the <b>0.3 m</b> hover above the start mark, (−0.60, 0) m: the vehicle lands where it took off. "
     "The status reads <i>COMPLETE</i>.", [], None),
    ("Touch down.", "Back to SAFETY, land and disarm from the drone GS.", [], None),
]

# ---------------------------------------------------------------- key parameters (2026-10-21)
# The waypoints come from the planner itself (extract_waypoints.py -> layout_waypoints.json), the
# same file make_layout_figures.py draws from, so the table and the pictures cannot disagree.
WP = json.load(open(os.path.join(D, "layout_waypoints.json")))
_G = WP["goals"]


def _v(p, n=3):
    return "(" + ", ".join(f"{x:.{n}f}".replace("-0.000", "0.000").replace("-", "−") for x in p) + ")"


TABLE_STYLE = ("border-collapse:collapse;font-size:10pt;margin:6px 0 14px 0")
CELL = "border:1px solid #c9c8c3;padding:3px 6px;vertical-align:top"
HEAD = CELL + ";background:#f1f0ec;font-weight:bold"


def table(header, rows):
    out = [f'<table style="{TABLE_STYLE}"><tr>' + "".join(f'<td style="{HEAD}">{h}</td>' for h in header) + "</tr>"]
    for r in rows:
        out.append("<tr>" + "".join(f'<td style="{CELL}">{c}</td>' for c in r) + "</tr>")
    return "".join(out) + "</table>"


FRAME = ("All coordinates are in the <b>mocap (world) frame, in metres</b>: origin at the field centre, "
         "<b>x</b> toward the table, <b>y</b> to the left as seen from the start mark, <b>z</b> up; yaw 0 = +x, "
         "positive anticlockwise seen from above. The numbers are the nominal layout: the vehicle exactly on its "
         "start mark and the box exactly at its place.")

PLACEMENT = [
    ("Field", "4.5 m (x) × 4.2 m (y), centred on the origin (edges x ±2.25, y ±2.10)",
     "As in the simulation scenes; confirm against the lab."),
    ("Drone start mark", "<b>(−0.60, 0)</b>, facing +x (yaw 0)",
     "Adjust measures the vehicle's offset from it. Isaac spawns at (−0.50, −0.07): the mark plus a deliberate "
     "placement error, so Adjust has something to measure."),
    ("Takeoff hover", "(−0.60, 0, 1.30)", "SAFETY, from the drone GS."),
    ("Landing spot", "the start mark, (−0.60, 0)",
     "Land Start (z 1.30) and Land (z 0.30, the skids ~5 cm above the floor) are typed at the start mark, so the vehicle lands where it took off "
     "(Adjust shifts them with the start mark)."),
    ("Table", "HOOBRO BB52UXG01G1, 1.000 m long (along y) × 0.150 m deep (x)", "Length along y."),
    ("Table reference leg", "outer corner <b>(0.600, 0.500)</b>",
     "The +y end, on the side facing the start (front-left, seen from the start mark)."),
    ("Table footprint", "x 0.600 – 0.750, y −0.500 – 0.500",
     "The planner's keep-out (<i>push_pull_table</i>); it keeps the landing gear and the arm 0.05 m clear."),
    ("Table top height", "<b>0.800 m</b> in the simulation",
     "On hardware measure it. The 10-20 Get read the box at z 0.858 (simulation 0.850): if obj_0's pivot is at "
     "the box centre, the lab top is about 0.808, and <i>push_pull_table</i>'s last number should match."),
    ("Box", "0.240 m (y) × 0.160 m (x) × 0.100 m (z), 240 g (first test; 500 g full)",
     "The CAD box with its clamp bracket and fin. Box–table friction 0.25 in the simulation (the real box, "
     "measured)."),
    ("Box initial pose (obj_0)", "centre <b>(0.675, 0.280, 0.850)</b>, yaw 0",
     "Centred across the table; its back (+y) face at y 0.400, 0.10 m from the table's +y end; its front face "
     "at y 0.160."),
    ("Fin", "5 mm (x) × 60 mm (y), top at z 1.058 (0.258 m above the table)",
     "On the box centre; the jaws close across it, along x."),
    ("Grasp point", "box centre + (0, 0, 0.183) = <b>(0.675, 0.280, 1.033)</b>",
     "EE Offset. 25 mm under the fin top, 0.233 m above the table."),
    ("Box after the push", "centre (0.675, 0.180) after 0.10 m; (0.675, −0.220) after 0.50 m",
     "After 0.50 m its front face is 0.16 m from the table's −y end."),
]

SETTINGS = [
    ("EE Offset", "[0, 0, 0.183]", "From obj_0's origin (the box centre) to the grasp point, in the box frame."),
    ("Vertical Margin", "0.15 m", "At Go To Start the claw waits this far above the grasp point."),
    ("Side Margin", "0.12 m", "... and this far behind it (+y); Ready To Push slides it in from there."),
    ("Push", "the Push Distance along −y at 21 mm/s mean, after a 1.5 s hold: 0.10 m in 4.8 s, 0.50 m in 23.8 s",
     "The drone and the claw slide together. One setting: the planner's push_pull_push_mean_speed."),
    ("Arm pose for the push", "[0, 32, 28, 0] deg", "The claw 0.267 m ahead of and 0.132 m below the body. "
     "Home (folded) is [0, 40, 40, 0] deg."),
    ("Exit", "the claw climbs 0.20 m, then the arm folds home", "Starts by itself 1 s after the push."),
    ("Heading", "−90° from Go To Start to Exit (the nose along −y); −360° at Land Start and Land",
     "−360° = +x again, after a 270° clockwise turn on the way to Land Start."),
    ("Landing gear", "skids 0.2425 m (simulation) / 0.248 m (hardware, measured) under the body origin, "
     "0.28 m apart", "At the grasp the skids are at z 0.922: 22 mm above the box top, the box between them."),
]

STEP_ROWS = [
    ("0", "Takeoff hover", WP["hover"], 0.0, None, "claw at the folded home pose", "—"),
    ("1", "Go To Start", _G["go_to_start"]["base"], _G["go_to_start"]["yaw_deg"], _G["go_to_start"]["claw"],
     "Side Margin behind, Vertical Margin above the grasp", WP["step_T"][0]),
    ("2", "Ready To Push", _G["ready"]["base"], _G["ready"]["yaw_deg"], _G["ready"]["claw"], "the grasp point",
     WP["step_T"][1]),
    ("3", "Push", _G["push"]["base"], _G["push"]["yaw_deg"], _G["push"]["claw"], "the grasp point after 0.50 m",
     WP["step_T"][2]),
    ("4", "Exit To Push", _G["exit"]["base"], _G["exit"]["yaw_deg"], _G["exit"]["claw"],
     "0.20 m up, then the arm home", WP["step_T"][3]),
    ("5", "Go To Land Start", _G["go_to_land_start"]["base"], _G["go_to_land_start"]["yaw_deg"],
     _G["go_to_land_start"]["claw"], "arm home", WP["step_T"][4]),
    ("6", "Land", _G["land"]["base"], _G["land"]["yaw_deg"], _G["land"]["claw"],
     "arm home; then touch down in SAFETY", WP["step_T"][5]),
]
CLAW0 = WP["claw_path"][1:4]


def waypoint_rows():
    rows = []
    for k, name, base, yaw, claw, note, T in STEP_ROWS:
        claw = CLAW0 if claw is None else claw
        nw = '<span style="white-space:nowrap">{}</span>'
        rows.append((k, name, nw.format(_v(base)), f"{yaw:.0f}°".replace("-", "−"), nw.format(_v(claw)), note,
                     T if isinstance(T, str) else f"{T:.1f} s"))
    return rows


WAYPOINT_NOTE = ("These are the planner's own goals (Plan with the vehicle on its start mark at the 1.30 m takeoff "
                 "hover and the box at its place). Steps 1–4 are computed from the box (Get), so they move with "
                 "the box. The start mark, Land Start and Land are typed and shifted by Adjust. The Go To Start time "
                 "depends on where the vehicle hovers when Plan is pressed. Total planned flight: about "
                 f"{sum(WP['step_T']):.0f} s.")

FIGURES = [
    ("layout_3d.png", "The scene and the planned task. Blue: the drone body's path and goals; orange: the claw's."),
    ("layout_plan.png", "Top view of the whole field: where the start mark, the table and the landing spot are, "
     "and how far each is from the field edge. The shaded circles are the propeller tips (0.52 m)."),
    ("layout_detail.png", "Top view of the table and the box, with every dimension and the coordinates of the "
     "steps on the table."),
    ("layout_side.png", "Side view: every height in the task."),
    ("layout_front.png", "Front view, as seen from the start mark: the approach (Side and Vertical Margins), the "
     "grasp and the push."),
]


def key_section():
    out = ["<h4>Key parameters</h4>", f"<p>{FRAME}</p>",
           "<p><b>Placement</b></p>", table(("Item", "Value", "Note"), PLACEMENT),
           "<p><b>Task settings</b></p>", table(("Setting", "Value", "Note"), SETTINGS),
           "<p><b>Waypoints</b> (drone body = the mocap body's origin; claw = the grasp point)</p>",
           table(("#", "Step", "Drone body (x, y, z)", "Yaw", "Claw (x, y, z)", "Where the claw is", "Planned time"),
                 waypoint_rows()),
           f"<p>{WAYPOINT_NOTE}</p>",
           "<h4>Layout</h4>"]
    for name, cap in FIGURES:
        out += [img(name, 720), f"<p><i>{cap}</i></p>"]
    out.append("<h4>Workflow</h4>")
    return out

HINT = ("ABORT (red) works at any time: the gripper opens and contact ends; with the claw on the box the drone "
        "holds where it is for 1.5 s, backs 3 cm off against the push, then climbs 0.30 m with the arm folding "
        "home. <b>Afterwards press 5 Go To Land Start and 6 Land; never land in place beside the table.</b> The "
        "planner aborts by itself if the tilt stays above <b>10°</b>, the force on the claw stays above <b>5 N</b> "
        "(from the grasp to the exit), the box falls behind or runs ahead of its plan by more than <b>40 mm</b>, "
        "the box tilts more than <b>2°</b>, the box's mocap goes silent or freezes, the arm's joint states go "
        "silent, or q3 drops below <b>10°</b>. A guard trip names itself in the status line. If DIRECT is lost with the claw on the box (watchdog, pilot STAB), the arm GS opens the "
        "gripper at once and the arm folds home 2 s later. If the box was moved before Ready (the downwash, a "
        "bump), press Get and Plan again, then Go To Start.")


def li(title, body, subs, pic):
    out = f"<li><b>{title}</b>"
    if body:
        out += f"<br>{body}"
    if subs:
        out += "<ul>" + "".join(f"<li>{s}</li>" for s in subs) + "</ul>"
    if pic:
        out += img(*pic)
    return out + "</li>"


html = ["<!doctype html><html><head><meta charset='utf-8'><title>Box-Pushing Workflow</title>",
        "<style>body{font-family:Arial,sans-serif;font-size:11pt;max-width:760px;margin:24px auto;"
        "padding:0 16px;color:#202124;background:#fff} li{margin:4px 0} ul li{margin:2px 0}</style></head><body>",
        "<h3>Box-Pushing Workflow</h3>"] + key_section() + [
        img("isaac_overview.png", 454),
        "<p><i>Isaac snapshot taken before the 2026-10-21 move, when the table stood 0.60 m further forward "
        "(reference corner (1.20, 0.50)); the start-to-table distance is the same.</i></p>",
        img("gs_panel.png", 600),
        f"<p>{INTRO}</p>", "<ol>"]
html += [li(*s) for s in STEPS]
html += ["</ol>", f"<p><b>Hint:</b> {HINT}</p>", "</body></html>"]
open(os.path.join(D, "box_pushing_workflow.html"), "w").write("\n".join(html))
print("wrote", os.path.join(D, "box_pushing_workflow.html"))

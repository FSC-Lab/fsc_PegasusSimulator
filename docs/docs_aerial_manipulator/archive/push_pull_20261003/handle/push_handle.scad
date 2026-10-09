// Push-and-pull handle: a vertical post glued or screwed to the TOP CENTRE of
// the 240 x 160 x 95 mm box (2026-10-06). Matches the Isaac scene that flew 6/6
// (07 push-and-pull, PEGASUS_PUSH_HANDLE=post, PEGASUS_PUSH_POST_TOP=0.298):
// the gripper grasps the post 25 mm below its top, 0.273 m above the table.
//
// Orientation on the box: the post's THICKNESS (post_t, 20 mm) runs ACROSS the
// table (the box's 160 mm width) -- the direction the jaws close; its DEPTH
// (post_d, 30 mm) runs ALONG the push (the box's 240 mm length). The two
// gripped faces are therefore the 30 mm-wide faces.
//
// Units: mm. Print upright (base plate on the bed), 4 perimeters, 20-30 %
// infill; the base fillet ribs keep the root stiff. Bending at the root from a
// ~1.2 N push 0.18 m up is ~0.2 N.m (stress ~0.1 MPa), far below PLA's layer
// strength. Weigh the printed handle: box + handle is the mass the sim needs.

box_h      = 95;    // box height (only used for the reported heights)
post_top   = 298;   // post top above the TABLE (sim: 0.298 m)
grasp_down = 25;    // grasp point below the post top (sim)
post_t     = 20;    // thickness ACROSS the jaws (check: inside your gripper's closing range)
post_d     = 30;    // depth ALONG the push
plate_x    = 70;    // base plate across the box (box is 160)
plate_y    = 90;    // base plate along the box (box is 240)
plate_h    = 4;     // base plate thickness
rib_h      = 35;    // fillet rib height
rib_l      = 25;    // fillet rib reach from the post
rib_t      = 4;     // fillet rib thickness
hole_d     = 4.4;   // M4 clearance (or leave the holes empty and glue / tape)
hole_inset = 9;     // hole centre from the plate edge

post_h = post_top - box_h;          // above the box top, plate included (203 mm)
echo(str("part height ", post_h, " mm; grasp at ", post_h - grasp_down,
         " mm above the box top = ", post_top - grasp_down, " mm above the table"));

module rib(len, ht, th) {           // right-angle triangle in the x-z plane, extruded th along y
  rotate([90, 0, 0]) linear_extrude(height = th, center = true)
    polygon([[0, 0], [len, 0], [0, ht]]);
}

difference() {
  union() {
    // base plate, centred on the origin = the box's top centre
    translate([-plate_x / 2, -plate_y / 2, 0]) cube([plate_x, plate_y, plate_h]);
    // the post
    translate([-post_t / 2, -post_d / 2, 0]) cube([post_t, post_d, post_h]);
    // four fillet ribs at the root (two across, two along)
    for (s = [-1, 1]) {
      translate([s * post_t / 2, 0, plate_h]) scale([s, 1, 1]) rib(rib_l, rib_h, rib_t);
      translate([0, s * post_d / 2, plate_h]) rotate([0, 0, 90]) scale([s, 1, 1]) rib(rib_l, rib_h, rib_t);
    }
  }
  // mounting holes
  for (sx = [-1, 1], sy = [-1, 1])
    translate([sx * (plate_x / 2 - hole_inset), sy * (plate_y / 2 - hole_inset), -1])
      cylinder(d = hole_d, h = plate_h + 2, $fn = 32);
  // centring notches on the plate edges: line them up with the box's centre lines
  for (s = [-1, 1]) {
    translate([s * plate_x / 2, 0, plate_h / 2]) cube([3, 2, plate_h + 2], center = true);
    translate([0, s * plate_y / 2, plate_h / 2]) cube([2, 3, plate_h + 2], center = true);
  }
  // shallow groove marking the grasp height on the two gripped (30 mm) faces
  for (s = [-1, 1])
    translate([s * post_t / 2, 0, post_h - grasp_down]) cube([1.2, post_d + 2, 1.0], center = true);
}

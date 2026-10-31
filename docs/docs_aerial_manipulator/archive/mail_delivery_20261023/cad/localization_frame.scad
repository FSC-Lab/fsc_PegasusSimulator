// Mail-delivery localization frame, rev B (2026-10-29), units mm. Generated
// by make_localization_frame.py; the same dimensions as localization_frame.stl.
// Origin O = centre of the UNDERSIDE (the face it rests on); +x = the drone's
// travel at that step; +y left; z up. The five marker holders (holders/*.stl)
// glue into the pockets; obj_0 = their markers, Motive's default pivot.
HAT_R = 100.0; BOX_D = 125.0; CLR = 0.3;           // what it fits
ENV_L = 150.0; ENV_W = 90.0; OVERHANG = 70.0;      // the envelope it places
R_IN = HAT_R + CLR;          // skirt arc (drops over the hat's rim)
XF = BOX_D / 2 + CLR;        // skirt flats (close on the mailbox's front and back faces)
XO = R_IN + 4.0; HY = 100.0;
PLATE_T = 4.0; SKIRT_D = 3.0;
SLOT_HY = ENV_W / 2 + CLR;                 // U-slot, open to the -x edge
SLOT_STOP = -(HAT_R + OVERHANG) + ENV_L;   // the envelope's far end stop
POCKET_D = 2.0;
POCKETS = [[10.0, 35.0, 30.5, 20.5] /* A */, [25.0, 80.0, 20.5, 30.5] /* B */, [-25.0, -75.0, 30.5, 15.5] /* C */, [-35.0, 80.0, 30.5, 20.5] /* D */, [40.0, -65.0, 20.5, 30.5] /* D */];   // centre x, y, size x, size y (holder design in the comment)

difference() {
    union() {
        translate([-XO, -HY, 0]) cube([2 * XO, 2 * HY, PLATE_T]);              // plate
        difference() {                                                        // the two skirts
            translate([-XO, -HY, -SKIRT_D]) cube([2 * XO, 2 * HY, SKIRT_D]);
            translate([0, 0, -SKIRT_D - 1]) cylinder(r = R_IN, h = SKIRT_D + 2, $fn = 720);
            translate([-XF, -HY - 1, -SKIRT_D - 1]) cube([2 * XF, 2 * HY + 2, SKIRT_D + 2]);
        }
    }
    translate([-XO - 1, -SLOT_HY, -SKIRT_D - 1]) cube([SLOT_STOP + XO + 1, 2 * SLOT_HY, PLATE_T + SKIRT_D + 2]);
    for (p = POCKETS) translate([p[0] - p[2] / 2, p[1] - p[3] / 2, PLATE_T - POCKET_D])
        cube([p[2], p[3], POCKET_D + 1]);
}

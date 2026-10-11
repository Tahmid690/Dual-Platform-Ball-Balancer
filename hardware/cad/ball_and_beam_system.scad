/* =====================================================================
   Ball-and-Beam PID System Chassis  (v3 — Bolt-Free Joints)
   =====================================================================
   - Sliding dovetail beam joints (no bolts, self-locking)
   - Tongue-and-groove baseplate joints (friction fit, no bolts)
   - MG995R open-front snap-fit box enclosure
   - VL53L0X ToF sensor bracket
   - 120° V-groove for smooth ball rolling
   - Gusseted pivot pillar with wider bearings
   - Only 8× M3 bolts total (structural base connections)
   ===================================================================== */
$fn = 100;
eps = 0.05;

// === EXPORT MENU ===
// 0  = Full Assembly Preview
// 2  = Left Pivot Pillar
// 3  = MG995R Servo Box (with support pillar)
// 5  = Linkage Arms (crank + vertical link)
// 7  = Printable Pin Pack (Axle + Linkage Hinge Pins)
// 11 = Baseplate Part 1 (Left:   0–190mm)
// 12 = Baseplate Part 2 (Middle: 190–340mm)
// 13 = Baseplate Part 3 (Right:  340–550mm with Sensor Platform)
// 41 = Upper Beam Part 1 (Left:   0–190mm)
// 42 = Upper Beam Part 2 (Middle: 190–340mm)
// 43 = Upper Beam Part 3 (Right:  340–540mm)
part_to_render = 0;

// === Hardware ===
clr_M3   = 3.4;
tap_M3   = 2.8;
clr_axle = 5.4;

// === System Geometry ===
pivot_z  = 150;
servo_z  = 90;
beam_w   = 25;
link_y   = 92;

// === MG995R Servo (mounted: shaft along -Y toward beam) ===
sv_body_x = 40.7;    // body length (X)
sv_body_y = 22;       // body depth behind tabs (Y)
sv_body_z = 19.7;    // body thickness (Z)
sv_tabs   = 54;       // total length with tabs (X)
sv_clr    = 0.15;     // per-side clearance (tight snap fit)

// === Split Positions ===
sp1 = 190;
sp2 = 340;

// === Tongue-and-Groove (baseplate, friction fit) ===
tg_len = 20;
tg_tol = 0.15;

// === Sliding Dovetail (beam joints) ===
dt_depth = 12;        // protrusion length in X
dt_h     = 6;         // dovetail height (Z)
dt_bot_hw = 4;        // half-width at bottom (narrow neck)
dt_top_hw = 6;        // half-width at top (wide cap, locks in Z)
dt_z_base = 5;        // Z offset from beam bottom
dt_tol   = 0.15;      // clearance per side

// ===============================================================
// HELPER MODULES
// ===============================================================
module r_cube(dim, r=3) {
    hull() for (x=[r,dim[0]-r], y=[r,dim[1]-r])
        translate([x, y, 0]) cylinder(r=r, h=dim[2]);
}

module cut_right(x) {
    translate([x, -500, -500]) cube([2000, 3000, 3000]);
}

// ---------------------------------------------------------------
// Baseplate Tongue-and-Groove (friction fit, no bolts)
// 80mm wide, 6mm tall, positioned at Y=45 to Y=125 (clear of lower track at Y=15..35)
// ---------------------------------------------------------------
module bp_tongue(x, tol) {
    translate([x, 45+tol, 2+tol])
        cube([tg_len-tol, 80-2*tol, 6-2*tol]);
}
module bp_groove(x, tol) {
    translate([x-eps, 45-tol, 2-tol])
        cube([tg_len+eps+tol, 80+2*tol, 6+2*tol]);
}

// ---------------------------------------------------------------
// Sliding Dovetail (beam joints)
// Trapezoidal prism: wider at top (Z+), narrower at bottom (Z-)
// Pieces slide together in Y direction, locked in X and Z
// Top surface stays flush (dovetail is below V-groove)
// ---------------------------------------------------------------
module dt_rail(x_split, tol=0) {
    // Dovetail rail protruding in +X from split face
    xc = x_split + dt_depth/2;
    hull() {
        // Bottom face (narrow neck)
        translate([xc-(dt_bot_hw-tol), 57.5+tol, dt_z_base+tol])
            cube([2*(dt_bot_hw-tol), beam_w-2*tol, eps]);
        // Top face (wide cap — locks in Z)
        translate([xc-(dt_top_hw-tol), 57.5+tol, dt_z_base+dt_h-tol])
            cube([2*(dt_top_hw-tol), beam_w-2*tol, eps]);
    }
}
module dt_socket(x_split, tol=0) {
    // Matching groove cut into receiving piece
    xc = x_split + dt_depth/2;
    hull() {
        translate([xc-(dt_bot_hw+tol), 57.5-tol-eps, dt_z_base-tol])
            cube([2*(dt_bot_hw+tol), beam_w+2*tol+2*eps, eps]);
        translate([xc-(dt_top_hw+tol), 57.5-tol-eps, dt_z_base+dt_h+tol])
            cube([2*(dt_top_hw+tol), beam_w+2*tol+2*eps, eps]);
    }
}

// ---------------------------------------------------------------
// Prominent Base Attachment Dovetail Key Joints (Direct Base Lock)
// Male dovetail keys protrude downward on bottom of Pillar & Servo Box (Z=2..10)
// Female dovetail sockets cut cleanly into Baseplate surface (Z=2..10)
// Slides laterally into position — zero screws needed, rock solid!
// ---------------------------------------------------------------
module base_dovetail_key_male(x_center, y_center, size_x=24, size_y=16, h=8, tol=0) {
    // Male dovetail key post (wider at bottom Z=2, narrower at top Z=10)
    translate([x_center, y_center, 10-h])
    linear_extrude(height=h)
    polygon([
        [-(size_x/2)+tol, -(size_y/2)+tol],
        [-(size_x/2 - 3)+tol, (size_y/2)-tol],
        [(size_x/2 - 3)-tol, (size_y/2)-tol],
        [(size_x/2)-tol, -(size_y/2)+tol]
    ]);
}

module base_dovetail_key_female(x_center, y_center, size_x=24, size_y=16, h=8.5, tol=0.15) {
    // Female dovetail keyway socket cut into baseplate
    translate([x_center, y_center, 10-h-eps])
    linear_extrude(height=h+2*eps)
    polygon([
        [-(size_x/2)-tol, -(size_y/2)-tol],
        [-(size_x/2 - 3)-tol, (size_y/2)+tol],
        [(size_x/2 - 3)+tol, (size_y/2)+tol],
        [(size_x/2)+tol, -(size_y/2)-tol]
    ]);
}

// ===============================================================
// PRINTABLE HARDWARE PIN PACK (100% 3D-Printed Snap-Fit Lock Pins)
// Self-locking barb tips with flex-slots — push until they CLICK!
// Zero screws, zero bolts, zero glue needed!
// ===============================================================
module snap_pivot_axle() {
    difference() {
        union() {
            // Head flange (11mm diameter)
            cylinder(d=11, h=3.5);
            // Main axle shaft (5.0mm diameter x 64mm length)
            translate([0,0,3.5]) cylinder(d=5.0, h=64);
            // Snap-fit barb tip (5.7mm max diameter)
            translate([0,0,67.5])
                cylinder(d1=5.0, d2=5.7, h=3);
            translate([0,0,70.5])
                cylinder(d1=5.7, d2=4.8, h=1.5);
        }
        // Flex slot (1.2mm wide, 18mm long down the tip)
        translate([0, 0, 58])
            cube([1.2, 12, 20], center=true);
    }
}

module snap_link_pin() {
    difference() {
        union() {
            // Head flange (7mm diameter)
            cylinder(d=7, h=2.2);
            // Shaft (3.0mm diameter x 11.5mm length)
            translate([0,0,2.2]) cylinder(d=3.0, h=11.5);
            // Snap-fit barb tip (3.6mm max diameter)
            translate([0,0,13.7])
                cylinder(d1=3.0, d2=3.6, h=2);
            translate([0,0,15.7])
                cylinder(d1=3.6, d2=2.8, h=1);
        }
        // Flex slot (0.8mm wide, 10mm long down tip)
        translate([0, 0, 11])
            cube([0.8, 8, 12], center=true);
    }
}

module printable_pins() {
    // 1x Main Pivot Snap Axle Pin
    color("LightSteelBlue")
        translate([0, 0, 0]) snap_pivot_axle();

    // 2x Linkage Snap Hinge Pins
    color("LightSteelBlue") {
        translate([20, 0, 0]) snap_link_pin();
        translate([35, 0, 0]) snap_link_pin();
    }
}

// ===============================================================
// BOTTOM TRACK SENSOR BRACKET (At right end of base channel, X=500–505)
// Straight vertical plate perpendicular to the lower base track,
// holding the bottom VL53L0X sensor facing down the channel (-X).
// ===============================================================
module bottom_track_sensor_bracket() {
    difference() {
        // Straight vertical bracket plate at right end of lower channel
        hull() {
            translate([500, 15, 10]) r_cube([5, 20, 2], r=1);
            translate([500, 15, 30]) r_cube([5, 20, 2], r=1);
        }
        // Sensor laser optical window aperture (centered on bottom channel at Y=25, Z=20mm)
        translate([499-eps, 25, 20]) rotate([0, 90, 0]) cylinder(d=8, h=10);

        // VL53L0X sensor PCB mounting holes (2x M2 tap holes, 15mm Y spacing)
        for (dy = [-7.5, 7.5])
            translate([499-eps, 25+dy, 20]) rotate([0, 90, 0]) cylinder(d=2.2, h=10);
    }
}

// ===============================================================
// BASEPLATE  (550 × 140 × 10 mm)
// ===============================================================
module baseplate() {
    difference() {
        union() {
            r_cube([550, 140, 10], r=8);
            // Straight vertical bracket for the bottom track
            bottom_track_sensor_bracket();
        }

        // Cable / lower sensor channel (X=50..500, Y=15..35)
        translate([50, 15, 5]) r_cube([450, 20, 6], r=2);

        // Left pillar dovetail keyway sockets (2x female sockets cut in baseplate)
        base_dovetail_key_female(20, 52, size_x=24, size_y=16);
        base_dovetail_key_female(20, 88, size_x=24, size_y=16);

        // Servo box support pillar dovetail keyway sockets (2x female sockets cut in baseplate)
        base_dovetail_key_female(390, link_y+12, size_x=20, size_y=20);
        base_dovetail_key_female(430, link_y+12, size_x=20, size_y=20);

        // Rubber feet holes (corners)
        for (p=[[15,15],[535,15],[15,125],[535,125]])
            translate([p[0],p[1],-eps]) cylinder(d=clr_M3, h=15);

        // ID text engraving
        translate([520, 90, -eps])
            linear_extrude(1.5) mirror([1,0,0]) {
                text("2205009", size=7);
                translate([0,12,0]) text("2205011", size=7);
                translate([0,24,0]) text("2205013", size=7);
            }
    }
}

// ===============================================================
// LEFT PIVOT PILLAR  (gusseted, wider bearings, 2x male dovetail base keys)
// ===============================================================
module left_pivot_pillar() {
    // Base mounting pad + 2x male dovetail keys underneath (Z=2..10)
    union() {
        translate([0, 45, 10]) r_cube([40, 50, 10], r=4);
        base_dovetail_key_male(20, 52, size_x=24, size_y=16);
        base_dovetail_key_male(20, 88, size_x=24, size_y=16);
    }
    difference() {
        union() {
            hull() {
                translate([5, 45, 20])  r_cube([30, 10, 2], r=2);
                translate([20, 50, pivot_z])
                    rotate([-90,0,0]) cylinder(d=24, h=10, center=true);
            }
            hull() {
                translate([5, 85, 20])  r_cube([30, 10, 2], r=2);
                translate([20, 90, pivot_z])
                    rotate([-90,0,0]) cylinder(d=24, h=10, center=true);
            }
            // Gusset ribs
            hull() { translate([5,45,20]) cube([30,10,1]); translate([10,45,70]) cube([20,10,1]); }
            hull() { translate([5,85,20]) cube([30,10,1]); translate([10,85,70]) cube([20,10,1]); }
        }
        translate([20, 30, pivot_z]) rotate([-90,0,0]) cylinder(d=clr_axle, h=80);
    }
}

// ===============================================================
// MG995R SERVO BOX  (open-front, tab-channel snap-fit, 2x male dovetail base keys)
// ===============================================================
module mg995r_servo_box() {
    wall = 4;
    tab_ext = (sv_tabs - sv_body_x)/2;  // ~6.65mm tab overhang per side
    tab_h = 2.5;  // tab thickness

    // Inner cavity dimensions
    in_x = sv_body_x + 2*sv_clr;        // ~41.0mm (body cavity, narrow)
    in_y = sv_body_y + 2*sv_clr;        // ~22.3mm
    in_z = sv_body_z + 2*sv_clr;        // ~20.0mm

    // Outer box (wide enough for tab channels in side walls)
    out_x = sv_tabs + 2*sv_clr + 2*wall;  // ~62.3mm
    out_y = in_y + wall;                    // ~26.3mm (back wall, front open)
    out_z = in_z + 2*wall;                  // ~28.0mm

    // Tab channel position: at mid-height of servo body
    tab_z = wall + in_z/2 - tab_h/2;       // ~12.75mm from box bottom

    // World position
    bx = 410 - out_x/2;
    by = link_y;                            // front (open) at y=92
    bz = servo_z - out_z/2;                // centered on servo_z

    // --- SUPPORT PILLAR (with 2x male dovetail base keys + vertical cable conduit) ---
    union() {
        base_dovetail_key_male(390, link_y+12, size_x=20, size_y=20);
        base_dovetail_key_male(430, link_y+12, size_x=20, size_y=20);
        difference() {
            hull() {
                translate([380, by, 10])  r_cube([60, out_y+5, 5], r=4);
                translate([bx, by, bz])   r_cube([out_x, out_y, 2], r=3);
            }
            // Vertical cable conduit (routes 3-wire servo cable down from box to base channel)
            translate([410-6, by+out_y-wall-10, 2])
                cube([12, 12, bz-2+eps]);
        }
    }

    // --- BOX ENCLOSURE (open front for shaft/crank) ---
    translate([bx, by, bz])
    difference() {
        union() {
            // Bottom plate
            r_cube([out_x, out_y, wall], r=2);
            // Top plate
            translate([0, 0, out_z-wall]) r_cube([out_x, out_y, wall], r=2);
            // Left wall
            cube([wall, out_y, out_z]);
            // Right wall
            translate([out_x-wall, 0, 0]) cube([wall, out_y, out_z]);
            // Back wall
            translate([0, out_y-wall, 0]) r_cube([out_x, wall, out_z], r=2);

            // Retention lips at front opening (prevent servo pullout)
            // Small bumps servo body clicks past
            for (sx = [wall+1, out_x-wall-3])
                translate([sx, 0, wall+2])
                    cube([2, 1.5, in_z-4]);
        }

        // Body cavity (narrow, body-width only)
        translate([(out_x-in_x)/2, -eps, wall])
            cube([in_x, in_y+eps, in_z]);

        // Tab channels (wider than body, at tab height)
        // Servo tabs slide into these channels when inserted
        translate([wall, -eps, tab_z])
            cube([out_x-2*wall, in_y+eps, tab_h+0.5]);

        // Wire exit slot (back wall, opening down into cable conduit)
        translate([out_x/2-6, out_y-wall-eps, 0])
            cube([12, wall+2*eps, 12]);
    }
}

// Dummy servo body for assembly visualization
module mg995r_ghost() {
    color("DimGray", 0.5) {
        // Main body
        translate([410-sv_body_x/2, link_y, servo_z-sv_body_z/2])
            cube([sv_body_x, sv_body_y, sv_body_z]);
        // Tabs
        translate([410-sv_tabs/2, link_y, servo_z-1.25])
            cube([sv_tabs, sv_body_y, 2.5]);
        // Shaft
        translate([400, link_y-8, servo_z])
            rotate([-90,0,0]) cylinder(d=5, h=8);
    }
}

// === System Kinematics & Tilt Testing ===
// Change beam_tilt_angle to test tilt (e.g. 10, -10, 0), or enable OpenSCAD animation ($t)
beam_tilt_angle = 12 * sin($t * 360);

// ===============================================================
// UPPER BEAM TRACK  (540mm, 120° V-groove, Straight End Bracket)
// ===============================================================
module raw_beam() {
    gd  = 6;
    ghw = gd * tan(60);

    difference() {
        union() {
            // Main beam body
            hull() {
                translate([5, 57.5, 0])  r_cube([530, beam_w, 2], r=1);
                translate([0, 57.5, 5])  r_cube([540, beam_w, 15], r=1);
            }
            // Left pivot ear
            hull() {
                translate([0, 57.5, 20]) cube([20, beam_w, 1]);
                translate([0, 57.5, 20]) cube([5, beam_w, 15]);
            }
            // Right-end straight vertical sensor bracket (at end of track, X=535-540)
            hull() {
                translate([535, 57.5, 15]) r_cube([5, beam_w, 2], r=1);
                translate([535, 57.5, 45]) r_cube([5, beam_w, 2], r=1);
            }
            // Linkage connection boss
            hull() {
                translate([394, 85, 4]) cube([12, 14, 12]);
                translate([400, 95, 10])
                    rotate([-90,0,0]) cylinder(d=12, h=4);
            }
        }

        // 120° V-groove
        translate([270, 70, 20])
        rotate([0, 90, 0])
        linear_extrude(height=560, center=true)
        polygon([[-1, -(ghw+1)], [gd, 0], [-1, ghw+1]]);

        // Sensor laser optical window aperture (centered on V-groove at Y=70, Z=26mm)
        translate([534-eps, 70, 26]) rotate([0, 90, 0]) cylinder(d=8, h=10);

        // VL53L0X sensor PCB mounting holes (2x M2 tap holes, 15mm Y spacing)
        for (dy = [-7.5, 7.5])
            translate([534-eps, 70+dy, 26]) rotate([0, 90, 0]) cylinder(d=2.2, h=10);

        // Pivot axle hole
        translate([20, 40, 10])
            rotate([-90,0,0]) cylinder(d=clr_axle, h=80);

        // Linkage articulation slot (8mm wide for 6mm link arm, 24mm wide along X for angular clearance)
        translate([388, 88, -5]) cube([24, 8, 30]);
        // Linkage pin hole (centered at Y=92)
        translate([400, 80, 10])
            rotate([-90,0,0]) cylinder(d=clr_M3, h=25);
    }
}

// ===============================================================
// SERVO CRANK  (solid)
// ===============================================================
module servo_crank() {
    difference() {
        hull() {
            cylinder(d=14, h=14);
            translate([32, 0, 0]) cylinder(d=12, h=14);
        }
        translate([0,0,-eps]) cylinder(d=6, h=5);
        translate([0,0,-eps]) cylinder(d=clr_M3, h=20);
        translate([32,0,-eps]) cylinder(d=clr_M3, h=20);
    }
}

// ===============================================================
// VERTICAL LINK  (6mm thick)
// ===============================================================
module vertical_link() {
    difference() {
        hull() {
            cylinder(d=10, h=6);
            translate([68,0,0]) cylinder(d=10, h=6);
        }
        translate([0,0,-eps])  cylinder(d=clr_M3, h=8);
        translate([68,0,-eps]) cylinder(d=clr_M3, h=8);
    }
}

// ===============================================================
// RENDER SELECTOR
// ===============================================================

// --- Static Parts ---
if (part_to_render == 2) color("DarkSlateGray") left_pivot_pillar();
else if (part_to_render == 3) color("DarkSlateGray") mg995r_servo_box();
else if (part_to_render == 5) {
    color("Gold")    servo_crank();
    color("Crimson") translate([0, 25, 5]) vertical_link();
}
else if (part_to_render == 7) printable_pins();

// --- Baseplate Splits (tongue-and-groove, no bolts) ---
else if (part_to_render == 11) {
    union() {
        difference() { baseplate(); cut_right(sp1); }
        bp_tongue(sp1, tg_tol);
    }
}
else if (part_to_render == 12) {
    difference() {
        union() {
            difference() {
                intersection() { baseplate(); cut_right(sp1); }
                cut_right(sp2);
            }
            bp_tongue(sp2, tg_tol);
        }
        bp_groove(sp1, tg_tol);
    }
}
else if (part_to_render == 13) {
    difference() {
        intersection() { baseplate(); cut_right(sp2); }
        bp_groove(sp2, tg_tol);
    }
}

// --- Beam Splits (sliding dovetail, no bolts) ---
else if (part_to_render == 41) {
    // Left piece + dovetail rail protruding right
    union() {
        difference() { raw_beam(); cut_right(sp1); }
        dt_rail(sp1, dt_tol);
    }
}
else if (part_to_render == 42) {
    // Middle piece: socket on left, rail on right
    difference() {
        union() {
            difference() {
                intersection() { raw_beam(); cut_right(sp1); }
                cut_right(sp2);
            }
            dt_rail(sp2, dt_tol);
        }
        dt_socket(sp1, dt_tol);
    }
}
else if (part_to_render == 43) {
    // Right piece: socket on left
    difference() {
        intersection() { raw_beam(); cut_right(sp2); }
        dt_socket(sp2, dt_tol);
    }
}

// --- Full Kinematic Assembly View (with Dynamic Tilt Animation) ---
else if (part_to_render == 0) {
    color("White")          baseplate();
    color("DarkSlateGray")  left_pivot_pillar();
    color("DarkSlateGray")  mg995r_servo_box();

    // Kinematic positions based on beam_tilt_angle
    theta = beam_tilt_angle;
    // Beam pivot point at (X=20, Z=150)
    color("DodgerBlue")
        translate([20, 0, pivot_z])
        rotate([0, -theta, 0])
        translate([-20, 0, -10])
        raw_beam();

    // Beam link pin position in world space
    bx_pin = 20 + 380 * cos(theta);
    bz_pin = pivot_z + 380 * sin(theta);

    // Servo crank rotation angle calculation (approximated for smooth kinematic display)
    phi_servo = 6.2 * theta;

    // Ghost servo
    mg995r_ghost();

    // Servo crank arm (rotates at servo center X=400, Z=90, Y=link_y)
    color("Gold")
        translate([400, link_y, servo_z])
        rotate([0, -phi_servo, 0])
        rotate([90,0,0]) translate([0,0,-7])
        servo_crank();

    // Crank end pin location
    cx_pin = 400 + 32 * cos(phi_servo);
    cz_pin = servo_z + 32 * sin(phi_servo);

    // Link arm angle relative to crank end
    dx_link = bx_pin - cx_pin;
    dz_link = bz_pin - cz_pin;
    link_ang = atan2(dz_link, dx_link);

    // Vertical linkage arm connecting crank pin to beam boss
    color("Crimson")
        translate([cx_pin, link_y, cz_pin])
        rotate([0, -link_ang, 0])
        rotate([90,0,0]) translate([0,0,-3])
        vertical_link();
}
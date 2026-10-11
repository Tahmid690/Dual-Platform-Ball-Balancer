/* =====================================================================
   Ball-and-Beam PID System — Upper Beam Track (36cm Total Length)
   =====================================================================
   - Total Length: 360mm (36cm)
   - Part 1: Main Track (0–340mm / 34cm Hard Print Limit) with 120° V-groove & linkage boss
   - Part 2: Sensor Attachment Cap (340–360mm / 2cm) with Vertical Ultrasound Holder
   - Locking Mechanism: Cylindrical Rod joint (10mm d) with cross-pin lock
   ===================================================================== */
$fn = 100;
eps = 0.05;

// === EXPORT MENU ===
// 0  = Full Assembly Preview (Main Track + Sensor Part + Lock Pin)
// 40 = Unified Single-Piece Beam (Full 360mm / 36cm, Horizontal)
// 41 = Part 1: Main Track (0–340mm / 34cm HARD PRINT LIMIT)
// 42 = Part 2: Compact Sensor Cap (340–360mm / 2cm with Cylindrical Rod)
// 43 = Part 3: Printable Cylindrical Lock Pin (25mm)
// 45 = Unified Single-Piece Beam (Pre-Oriented 45° Diagonal for 32x32cm Print Bed)
part_to_render = 45;

// === Hardware & Clearances ===
clr_M3   = 3.4;
tap_M3   = 2.8;
clr_axle = 5.4;
joint_tol = 0.15; // Clearance per side for 3D printed fit

// === System Geometry ===
beam_w    = 25;
pivot_x   = 10;   // X-position of pivot axle hole (Trimmed redundant material behind pivot!)
track_len = 340;  // 34cm HARD PRINT LIMIT for Part 1!
total_len = 360;  // 36cm total beam length

// === Interlocking Dovetail Key Joint Geometry ===
dt_len  = 14.0;   // Dovetail insertion length into main track
dt_w1   = 14.0;   // Top wide width (locks vertical & torsional twist)
dt_w2   = 8.0;    // Bottom narrow neck width
dt_h    = 8.0;    // Dovetail height
dt_z    = 4.0;    // Z-offset from beam bottom
lock_x  = 333.0;  // X-position of transverse locking pin hole (centered in 14mm stem)

// ===============================================================
// HELPER MODULES
// ===============================================================
module r_cube(dim, r=3) {
    hull() for (x=[r,dim[0]-r], y=[r,dim[1]-r])
        translate([x, y, 0]) cylinder(r=r, h=dim[2]);
}

// Interlocking Male Dovetail Key (Protrudes -X from sensor part)
module male_dovetail_key(tol=0) {
    translate([track_len - dt_len - eps, 70, dt_z])
    rotate([0, 90, 0])
    linear_extrude(height = dt_len + eps)
    polygon([
        [-(dt_h - tol), -(dt_w2/2 - tol)],
        [-(dt_h - tol),  (dt_w2/2 - tol)],
        [  tol,          (dt_w1/2 - tol)],
        [  tol,         -(dt_w1/2 - tol)]
    ]);
}

// Female Dovetail Socket (Cut into main track)
module female_dovetail_socket(tol=joint_tol) {
    translate([track_len - dt_len - eps, 70, dt_z - tol])
    rotate([0, 90, 0])
    linear_extrude(height = dt_len + 2*eps)
    polygon([
        [-(dt_h + tol), -(dt_w2/2 + tol)],
        [-(dt_h + tol),  (dt_w2/2 + tol)],
        [  tol,          (dt_w1/2 + tol)],
        [  tol,         -(dt_w1/2 + tol)]
    ]);
}

// 320mm x 320mm Print Bed Visual Outline
module print_bed_32x32() {
    %color("SlateGray", 0.15)
        translate([0, 0, -1]) cube([320, 320, 1]);
    %color("White", 0.4)
        translate([0, 0, -0.5])
        difference() {
            cube([320, 320, 0.5]);
            translate([3, 3, -1]) cube([314, 314, 3]);
        }
}

// ===============================================================
// RAW BEAM BODY (Base 360mm length)
// ===============================================================
module raw_beam_base() {
    gd  = 6;
    ghw = gd * tan(60);

    difference() {
        union() {
            // Main beam body (0 to total_len smooth track)
            hull() {
                translate([0, 57.5, 0])  r_cube([total_len, beam_w, 2], r=1);
                translate([0, 57.5, 5])  r_cube([total_len, beam_w, 15], r=1);
            }

            // Left pivot ear (X = 0 to pivot_x + 5mm)
            hull() {
                translate([0, 57.5, 20]) cube([pivot_x + 5, beam_w, 1]);
                translate([0, 57.5, 20]) cube([5, beam_w, 15]);
            }

            // Right-end vertical ultrasound sensor holder plate (flush at X = total_len-5 to total_len)
            translate([total_len - 5, 57.5, 0]) cube([5, beam_w, 68]);
        }

        // 120° V-groove for smooth ball rolling (X = 0 to total_len-5mm)
        translate([total_len/2, 70, 20])
        rotate([0, 90, 0])
        linear_extrude(height=total_len + 20, center=true)
        polygon([[-1, -(ghw+1)], [gd, 0], [-1, ghw+1]]);

        // Vertical Ultrasound Sensor (HC-SR04 / US-015) Cutouts (Centered at Y=70, X=total_len-6)
        // Lower transducer (raised to Z=28mm, sits above track)
        translate([total_len - 6 - eps, 70, 28]) rotate([0, 90, 0]) cylinder(d=16.3, h=10);
        // Upper transducer (stacked vertically 25.8mm above at Z=53.8mm)
        translate([total_len - 6 - eps, 70, 53.8]) rotate([0, 90, 0]) cylinder(d=16.3, h=10);

        // Ultrasound sensor PCB mounting holes (4x M3 tap/clearance holes)
        for (dz = [18, 60]) {
            for (dy = [-7.5, 7.5]) {
                translate([total_len - 6 - eps, 70+dy, dz]) rotate([0, 90, 0]) cylinder(d=clr_M3, h=10);
            }
        }

        // Pivot axle hole (at X = pivot_x = 10mm)
        translate([pivot_x, 40, 10])
            rotate([-90,0,0]) cylinder(d=clr_axle, h=80);

        // Linkage articulation slot (8mm wide, at X=280mm)
        translate([268, 88, -5]) cube([24, 8, 30]);
        // Linkage pin hole (centered at Y=92)
        translate([280, 80, 10])
            rotate([-90,0,0]) cylinder(d=clr_M3, h=25);
    }
}

// ===============================================================
// PART 1: MAIN TRACK (0–340mm / 34cm HARD PRINT LIMIT)
// ===============================================================
module beam_part1_main_track() {
    difference() {
        // Cut raw beam at 340mm
        intersection() {
            raw_beam_base();
            translate([-10, -500, -500]) cube([track_len + 10, 1000, 1000]);
        }

        // Female dovetail keyway socket for sensor part
        female_dovetail_socket(joint_tol);

        // Transverse cylindrical locking pin hole
        translate([lock_x, 40, dt_z + dt_h/2])
            rotate([-90, 0, 0])
            cylinder(d=clr_M3, h=80);
    }
}

// ===============================================================
// PART 2: SENSOR ATTACHMENT CAP (340–360mm / 2cm)
// ===============================================================
module beam_part2_sensor_attachment() {
    union() {
        // Sensor attachment body (340 to 360mm)
        difference() {
            intersection() {
                raw_beam_base();
                translate([track_len, -500, -500]) cube([total_len - track_len + 20, 1000, 1000]);
            }
        }

        // Male dovetail key (protrudes -X from 340mm into main track)
        difference() {
            male_dovetail_key(joint_tol);

            // Transverse locking pin hole through male dovetail key
            translate([lock_x, 40, dt_z + dt_h/2])
                rotate([-90, 0, 0])
                cylinder(d=clr_M3, h=80);
        }
    }
}

// ===============================================================
// PART 3: PRINTABLE CYLINDRICAL LOCK PIN
// ===============================================================
module cylindrical_lock_pin() {
    difference() {
        union() {
            // Head flange
            cylinder(d=7.0, h=2.5);
            // Main cylindrical pin shaft (3.0mm diameter x 25mm length)
            translate([0, 0, 2.5]) cylinder(d=3.0, h=25);
            // Snap retention tip
            translate([0, 0, 27.5]) cylinder(d1=3.0, d2=3.6, h=1.5);
            translate([0, 0, 29.0]) cylinder(d1=3.6, d2=2.8, h=1.0);
        }
        // Flex slot down the tip
        translate([0, 0, 24]) cube([0.8, 8, 12], center=true);
    }
}

// ===============================================================
// RENDER SELECTOR
// ===============================================================
if (part_to_render == 41) {
    // Part 1: Main Beam Track (33cm)
    color("DodgerBlue") beam_part1_main_track();
}
else if (part_to_render == 42) {
    // Part 2: Sensor Attachment Part (5cm) with Cylindrical Lock Rod
    color("DodgerBlue") beam_part2_sensor_attachment();
}
else if (part_to_render == 43) {
    // Part 3: Printable Cylindrical Lock Pin
    color("LightSteelBlue") cylindrical_lock_pin();
}
else if (part_to_render == 40) {
    // Unified Single-Piece Beam (Full 38cm, Horizontal)
    color("DodgerBlue") raw_beam_base();
}
else if (part_to_render == 45) {
    // Pre-Oriented 45° Diagonal Layout on 320x320mm Print Bed
    print_bed_32x32();
    color("DodgerBlue")
        translate([160, 160, 0])
        rotate([0, 0, 45])
        translate([-190, -70, 0])
        raw_beam_base();
}
else {
    // Default (0): Full Assembly Preview (Main Track + Sensor Part + Lock Pin)
    color("DodgerBlue") beam_part1_main_track();
    color("DeepSkyBlue") beam_part2_sensor_attachment();
    color("Gold") translate([lock_x, 57.5 - 2, 8]) rotate([-90, 0, 0]) cylindrical_lock_pin();
}
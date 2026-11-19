"""Generate the long straight tracks for Straight_Wall_Project.

Every track runs along +x. It is a chain of sections; each section has its own width and centre
offset, and the end of one section is the start of the next (a short perpendicular step wall joins
them). Walls are built from 0.5-1.0 m segments, and each segment can carry a "wall error":
  slant  - one end pushed outward (up to 10 cm) or slightly inward (up to 2 cm)
  shift  - whole segment moved in/out by up to 1.5 cm
  gap    - segment shortened, leaving a 3-6 cm hole (smaller than the 8 cm chassis)
plus 0.5 cm jitter on every endpoint. Inward errors are clamped so the free corridor never
gets narrower than MIN_FREE. The trainer's StraightGuide ignores these errors (global axis +
median centreline), so the robot is rewarded for tracking the NOMINAL centre/direction.

Run from the project root:  python tools/gen_straight_tracks.py
"""
import math
import os
import random

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tracks")
BACK_CAP = -0.25     # wall behind the spawn
FINISH_BEFORE_END = 0.30
MIN_FREE = 0.22      # narrowest allowed wall-to-wall distance after inward errors


def build(name, comment, sections, spawn_heading, spawn_lat=0.0, errors=1.0, seed=0, min_free=None):
    """sections: list of (length_m, width_m, centre_offset_m). min_free overrides MIN_FREE for this track."""
    mf = MIN_FREE if min_free is None else min_free
    rng = random.Random(seed)
    walls = []

    # Section boundaries in x.
    xs = [BACK_CAP]
    x = 0.0
    for i, (length, _, _) in enumerate(sections):
        x += length
        xs.append(x)
    end_x = xs[-1]

    def side_walls(sign):
        """sign=+1 left wall (y>0), -1 right wall."""
        prev_y = None
        for i, (_, width, off) in enumerate(sections):
            y_nom = off + sign * width / 2.0
            x0, x1 = xs[i], xs[i + 1]
            if prev_y is not None and abs(prev_y - y_nom) > 1e-6:
                walls.append((x0, prev_y, x0, y_nom))  # step between sections
            prev_y = y_nom
            # inward limit for this section: keep at least MIN_FREE free width
            max_in = max(0.0, (width - mf) / 2.0)
            xa = x0
            while xa < x1 - 1e-6:
                seg = rng.uniform(0.5, 1.0)
                xb = min(x1, xa + seg)
                if x1 - xb < 0.25:
                    xb = x1
                ya = yb = y_nom
                xa_e, xb_e = xa, xb
                r = rng.random()
                if r < 0.18 * errors:            # slant
                    if rng.random() < 0.75:
                        d = rng.uniform(0.03, 0.10)               # outward
                    else:
                        d = -min(rng.uniform(0.01, 0.02), max_in)  # inward
                    if rng.random() < 0.5:
                        yb += sign * d
                    else:
                        ya += sign * d
                elif r < 0.30 * errors:          # shift
                    d = rng.uniform(-min(0.015, max_in), 0.015)
                    ya += sign * d
                    yb += sign * d
                elif r < 0.40 * errors:          # gap at the end of this segment
                    xb_e = xb - rng.uniform(0.03, 0.06)
                j = 0.005 * errors
                ya += rng.uniform(-j, j) if xa > x0 + 1e-6 else 0.0
                yb += rng.uniform(-j, j) if xb < x1 - 1e-6 else 0.0
                walls.append((xa_e, ya, xb_e, yb))
                xa = xb
        return prev_y

    first_w, first_off = sections[0][1], sections[0][2]
    last_w, last_off = sections[-1][1], sections[-1][2]
    side_walls(+1)
    side_walls(-1)
    # caps
    walls.append((BACK_CAP, first_off + first_w / 2, BACK_CAP, first_off - first_w / 2))
    walls.append((end_x, last_off + last_w / 2, end_x, last_off - last_w / 2))

    fx = end_x - FINISH_BEFORE_END
    # The finish runs wall to wall (+2 cm each side) measured on the ACTUAL walls at fx, so an
    # outward wall error there cannot leave a gap beside it. Falls back to the nominal wall
    # position if fx happens to land in a gap error.
    for i in range(len(sections)):
        if xs[i] <= fx <= xs[i + 1]:
            w, off = sections[i][1], sections[i][2]
    ys = [b + (d - b) * (fx - a) / (c - a)
          for (a, b, c, d) in walls if abs(c - a) > 1e-9 and min(a, c) <= fx <= max(a, c)]
    top = min((y for y in ys if y > off), default=off + w / 2)
    bottom = max((y for y in ys if y < off), default=off - w / 2)
    finish = (fx, top + 0.02, fx, bottom - 0.02)

    sy = first_off + spawn_lat
    lines = [f"# {name}: {comment}",
             f"# sections (length m, width m, centre offset m): {sections}",
             f"# spawn heading {spawn_heading:+.0f} deg from the track axis, lateral offset {spawn_lat:+.3f} m",
             f"spawn 0.000 {sy:.3f} {spawn_heading:.1f}", ""]
    for (a, b, c, d) in walls:
        lines.append(f"wall {a:.3f} {b:.3f} {c:.3f} {d:.3f}")
    lines += ["", "finish {:.3f} {:.3f} {:.3f} {:.3f}".format(*finish), ""]
    with open(os.path.join(OUT_DIR, name + ".track"), "w", newline="\n") as f:
        f.write("\n".join(lines))
    print(f"{name}: {end_x:.1f} m, {len(walls)} walls")


# The real robot (robot_hardware.txt) turns in place inside a 2 x 156.5 = 313 mm circle about its axle.
# FEASIBLE_W = that + 2 x 12.5 mm for endpoint jitter / wall errors, rounded up = 0.34 m; tracks the robot must be
# able to spin in keep at least FEASIBLE_FREE free after inward wall errors.
FEASIBLE_W = 0.34
FEASIBLE_FREE = 0.33
# A rotated spawn is FAIR when the outline clears the walls by >= 3 cm (3 sigma of the per-run wall jitter at 1x
# noise): otherwise the random wall offset, not the controller, decides whether the robot hits at t = 0.
# For spawns rotated 60-80 deg that needs a 0.38 m section (reach of the front corners ~0.156 m + 3 cm per side).
FAIR_SPAWN_W = 0.38

TRACKS = [
    # The original versions of the two tracks that were narrower than the turning circle, kept for comparison.
    ("straight_long_narrow_old", "20 m, 0.26 m wide, start rotated +70 deg (ORIGINAL, narrower than the real robot's turning circle)",
     [(20.0, 0.26, 0.0)], 70.0, 0.0, 1.0, 1),
    ("straight_mixed_widths_old", "20 m, five 4 m sections 0.30/0.50/0.26/0.60/0.36 m, start -60 deg (ORIGINAL)",
     [(4.0, 0.30, 0.0), (4.0, 0.50, 0.0), (4.0, 0.26, 0.0), (4.0, 0.60, 0.0), (4.0, 0.36, 0.0)],
     -60.0, 0.0, 1.0, 5),
    # Widened to the minimum feasible width (same seed, same errors otherwise).
    ("straight_long_narrow_034", "20 m, 0.34 m wide (minimum feasible), start rotated +70 deg (spawn clearance 1.6 cm: unfair)",
     [(20.0, FEASIBLE_W, 0.0)], 70.0, 0.0, 1.0, 1, FEASIBLE_FREE),
    ("straight_long_narrow", "20 m, 0.38 m wide (fair spawn for the real robot), start rotated +70 deg",
     [(20.0, FAIR_SPAWN_W, 0.0)], 70.0, 0.0, 1.0, 1, FEASIBLE_FREE),
    ("straight_long_wide", "22 m, 0.60 m wide, start rotated -80 deg and 12 cm off-centre",
     [(22.0, 0.60, 0.0)], -80.0, 0.12, 1.0, 2),
    ("straight_long_medium", "18 m, 0.40 m wide, start +45 deg, 8 cm off-centre, heavy wall errors",
     [(18.0, 0.40, 0.0)], 45.0, -0.08, 1.8, 3),
    ("straight_long_errors", "25 m, 0.34 m wide, start -30 deg, very heavy wall errors",
     [(25.0, 0.34, 0.0)], -30.0, 0.03, 2.4, 4),
    ("straight_mixed_widths_034", "20 m, five 4 m sections 0.34/0.50/0.34/0.60/0.36 m, start -60 deg (spawn clearance 1.4 cm: unfair)",
     [(4.0, FEASIBLE_W, 0.0), (4.0, 0.50, 0.0), (4.0, FEASIBLE_W, 0.0), (4.0, 0.60, 0.0), (4.0, 0.36, 0.0)],
     -60.0, 0.0, 1.0, 5, FEASIBLE_FREE),
    ("straight_mixed_widths", "20 m, five 4 m sections 0.38/0.50/0.34/0.60/0.36 m (spawn section fair, the rest feasible), start -60 deg",
     [(4.0, FAIR_SPAWN_W, 0.0), (4.0, 0.50, 0.0), (4.0, FEASIBLE_W, 0.0), (4.0, 0.60, 0.0), (4.0, 0.36, 0.0)],
     -60.0, 0.0, 1.0, 5, FEASIBLE_FREE),
    ("straight_mixed_offset_old", "21 m, sections of different width AND shifted centre, start +80 deg (0.30 / 0.28 m sections: no room for a scan turn)",
     [(4.0, 0.36, 0.0), (4.0, 0.50, 0.08), (4.0, 0.30, -0.05), (5.0, 0.56, 0.0), (4.0, 0.28, 0.04)],
     80.0, 0.0, 1.0, 6),
    ("straight_mixed_offset", "21 m, sections of different width AND shifted centre, start +80 deg (0.30/0.28 widened to 0.34, spawn section 0.38)",
     [(4.0, FAIR_SPAWN_W, 0.0), (4.0, 0.50, 0.08), (4.0, FEASIBLE_W, -0.05), (5.0, 0.56, 0.0), (4.0, FEASIBLE_W, 0.04)],
     80.0, 0.0, 1.0, 6, FEASIBLE_FREE),
    ("straight_width_ladder_old", "21 m, seven 3 m sections stepping 0.26->0.62->0.28 m, start +15 deg (0.26/0.28/0.32 m: no room for a scan turn)",
     [(3.0, 0.26, 0.0), (3.0, 0.32, 0.0), (3.0, 0.42, 0.0), (3.0, 0.52, 0.0), (3.0, 0.62, 0.0),
      (3.0, 0.40, 0.0), (3.0, 0.28, 0.0)], 15.0, 0.0, 1.2, 7),
    ("straight_width_ladder", "21 m, seven 3 m sections stepping 0.34->0.62->0.34 m (narrow ones widened to the minimum feasible), start +15 deg",
     [(3.0, FEASIBLE_W, 0.0), (3.0, FEASIBLE_W, 0.0), (3.0, 0.42, 0.0), (3.0, 0.52, 0.0), (3.0, 0.62, 0.0),
      (3.0, 0.40, 0.0), (3.0, FEASIBLE_W, 0.0)], 15.0, 0.0, 1.2, 7, FEASIBLE_FREE),
]

# Held-out tracks (tasks.txt C5): NEVER listed in train_tracks; only `benchmark --heldout` uses them.
# New seeds (101+); the spawn section is always >= FEASIBLE_W so the robot can turn in place there.
HELDOUT = [
    ("heldout_01", "18 m, 0.40 m wide, start +60 deg",
     [(18.0, 0.40, 0.0)], 60.0, 0.0, 1.2, 101, FEASIBLE_FREE),
    ("heldout_02_old", "20 m, 0.45 m wide, start -75 deg 5 cm off-centre, heavy wall errors (spawn clearance 1.8 cm: unfair)",
     [(20.0, 0.45, 0.0)], -75.0, -0.05, 2.0, 102, FEASIBLE_FREE),
    ("heldout_02", "20 m, 0.45 m wide, start -75 deg centred, heavy wall errors",
     [(20.0, 0.45, 0.0)], -75.0, 0.0, 2.0, 102, FEASIBLE_FREE),
    ("heldout_03", "19 m, sections 0.36/0.55/0.40/0.34 m, start +35 deg",
     [(5.0, 0.36, 0.0), (5.0, 0.55, 0.0), (5.0, 0.40, 0.0), (4.0, FEASIBLE_W, 0.0)], 35.0, 0.0, 1.0, 103, FEASIBLE_FREE),
    ("heldout_04", "21 m, shifted sections 0.42/0.36(+6 cm)/0.50(-8 cm)/0.38, start -45 deg",
     [(5.0, 0.42, 0.0), (5.0, 0.36, 0.06), (6.0, 0.50, -0.08), (5.0, 0.38, 0.0)], -45.0, 0.04, 1.3, 104, FEASIBLE_FREE),
    ("heldout_05", "22 m, 0.58 m wide, start +80 deg, 10 cm off-centre",
     [(22.0, 0.58, 0.0)], 80.0, 0.10, 1.0, 105, FEASIBLE_FREE),
    ("heldout_06", "20 m, 0.34 m wide (minimum feasible), start -20 deg, very heavy wall errors",
     [(20.0, FEASIBLE_W, 0.0)], -20.0, 0.0, 2.4, 106, FEASIBLE_FREE),
    ("heldout_07", "21 m, ladder 0.48/0.40/0.34/0.40/0.52/0.44/0.36 m, start +50 deg",
     [(3.0, 0.48, 0.0), (3.0, 0.40, 0.0), (3.0, FEASIBLE_W, 0.0), (3.0, 0.40, 0.0), (3.0, 0.52, 0.0),
      (3.0, 0.44, 0.0), (3.0, 0.36, 0.0)], 50.0, 0.0, 1.2, 107, FEASIBLE_FREE),
    ("heldout_08", "18 m, sections 0.40/0.34(-4 cm)/0.46(+5 cm), start -65 deg, 6 cm off-centre",
     [(6.0, 0.40, 0.0), (6.0, FEASIBLE_W, -0.04), (6.0, 0.46, 0.05)], -65.0, 0.06, 1.5, 108, FEASIBLE_FREE),
]

if __name__ == "__main__":
    # python tools/gen_straight_tracks.py [--only name1,name2]   (default: all training + held-out tracks)
    import sys
    only = None
    if "--only" in sys.argv:
        only = set(sys.argv[sys.argv.index("--only") + 1].split(","))
    for t in TRACKS + HELDOUT:
        if only is None or t[0] in only:
            build(*t)
            print("wrote tracks/" + t[0] + ".track")

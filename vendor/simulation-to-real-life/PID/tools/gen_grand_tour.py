"""Generate 'grand_tour': one big showcase circuit chaining every feature the five training
tracks exercise, for visualizing a trained robot on a long continuous run.

Usage (from the repo root): python tools/gen_grand_tour.py [out_dir]   (default: tracks)

The route runs as a large clockwise loop and finishes beside the spawn, so the whole circuit
fits one screen. Sections, in order:
   1  wide warm-up straight (0.50 m)
   2  sweeping arcs (0.42 m)
   3  45-degree zigzag (0.35 m)
   4  corridor widening 0.24 -> 0.55 m
   5  sharp 90-degree corner into a tight uniform section (0.24 m) with curves
   6  progressive narrowing 0.34 -> 0.28 -> 0.24 m
   7  two 90-degree dead-end corners (scan recovery)
   8  hairpins (180-degree, tight radius)
   9  chicane / S-curves at speed
  10  mixed sharp corners alternating wide (0.50 m) and tight (0.20 m)
  11  long wide return straight back to the finish beside the spawn

Walls carry a small jitter everywhere, so no two walls are exactly parallel -- the
"corridor with errors" case the robot meets on real hardware.
"""
import math, sys

from track_lib import *  # noqa: F403  (shared geometry + verification)

OUT_DIR = sys.argv[1] if len(sys.argv) > 1 else "../tracks"

SECTIONS = []  # (label, s_start, s_end) for the per-section report


def _s(path):
    return sum(math.hypot(path.pts[i][0] - path.pts[i - 1][0], path.pts[i][1] - path.pts[i - 1][1])
               for i in range(1, len(path.pts)))


def section(path, label, fn):
    a = _s(path)
    fn()
    SECTIONS.append((label, a, _s(path)))


# The route is a large clockwise rectangle: four +90 deg corners (360 deg total) bring it back
# alongside the spawn. Every feature sits inside a leg and has zero net turn, so the legs stay
# straight and the loop closes.
p = Path(-0.30, 0.0, 0.0, 0.50)

# ---- leg 1, east: wide warm-up -> sweepers -> zigzag
section(p, "1 wide warm-up",      lambda: p.straight(2.8, step=0.5))
section(p, "2 sweeping arcs",     lambda: p.straight(0.4, width=0.42, step=0.2).arc(50, 1.3).arc(-50, 1.3).straight(0.3))
section(p, "3 zigzag 45deg",      lambda: (p.straight(0.3, width=0.35, step=0.15)
                                           .turn(45).straight(0.65).turn(-90).straight(0.65)
                                           .turn(90).straight(0.65).turn(-45).straight(1.1, step=0.4)))
section(p, "4 corner + widening", lambda: (p.turn(90).straight(0.5, width=0.24, step=0.25)
                                           .straight(3.0, width=0.55, step=0.4)))
# ---- leg 2, south: tight uniform -> progressive narrowing
section(p, "5 tight uniform",     lambda: (p.straight(0.5, width=0.38, step=0.25)     # graded entry: 0.55 -> 0.20
                                           .straight(0.5, width=0.28, step=0.25)
                                           .straight(0.5, width=0.24, step=0.25)
                                           .straight(0.4, step=0.2).arc(70, 0.65).arc(-70, 0.65)
                                           .straight(0.5, step=0.25).arc(-60, 0.7).arc(60, 0.7).straight(0.3)))
section(p, "6 progressive",       lambda: (p.straight(0.3, width=0.34, step=0.15).straight(1.1, step=0.3)
                                           .straight(0.3, width=0.28, step=0.1)
                                           .straight(0.5, step=0.25).arc(45, 0.8).arc(-45, 0.8)
                                           .straight(0.3, width=0.24, step=0.1).straight(0.9, step=0.25)))
# ---- leg 3, west: dead ends -> hairpins
section(p, "7 corner + dead ends", lambda: (p.straight(0.3, width=0.30, step=0.15).turn(90)
                                            .straight(1.3, step=0.35).turn(90)
                                            .straight(1.2, step=0.4).turn(-90)
                                            .straight(1.3, step=0.4).turn(-90)
                                            .straight(1.4, step=0.45)))
section(p, "8 hairpins",          lambda: (p.straight(0.3, width=0.34, step=0.15)
                                           .arc(180, 0.55).straight(1.0, step=0.3).arc(-180, 0.55)
                                           .straight(1.6, step=0.4)))
# ---- leg 4, north: chicane -> mixed sharp corners -> return straight to the finish
section(p, "9 corner + chicane",  lambda: (p.straight(0.3, width=0.40, step=0.15).turn(90)
                                           .straight(0.6, step=0.3)
                                           .arc(40, 0.9).arc(-80, 0.9).arc(80, 0.9).arc(-40, 0.9)
                                           .straight(0.5, step=0.25)))
section(p, "10 mixed corners",    lambda: (p.straight(0.3, width=0.50, step=0.15).straight(0.9, step=0.45)
                                           .turn(90).straight(0.6).straight(0.3, width=0.20).straight(0.9, step=0.45)
                                           .turn(-90).straight(1.2, step=0.4).straight(0.6, width=0.50, step=0.3)
                                           .straight(0.6, step=0.3).turn(-90).straight(1.7, step=0.45).turn(90)))


def close_the_loop(path, target_x, target_y, width):
    """Steer the tail of the route back beside the spawn, so the circuit reads as a loop.
    Runs the long leg north FIRST, clear of everything built so far, then makes the short
    sideways jog to the finish -- going sideways first would run parallel to the west leg and
    collide with it. Computed from the geometry actually built, not hand-tuned leg lengths."""
    def turn_to(heading):
        d = (heading - path.h + 180) % 360 - 180
        if abs(d) > 0.01:
            path.turn(d)

    path.straight(1.3, step=0.45)  # realize any turn still pending from the previous section (so the
                                   # next turn applies to a real segment and never reverses) and clear
                                   # the previous section before the long return runs alongside it
    x, y = path.pts[-1]
    turn_to(270.0)                                   # north: the long return straight
    if y - target_y > 0.05:
        path.straight(y - target_y, width=width, step=0.5)
    dx = target_x - x
    turn_to(0.0 if dx >= 0 else 180.0)               # short jog across to the finish
    if abs(dx) > 0.05:
        path.straight(abs(dx), step=0.5)


section(p, "11 return straight",  lambda: close_the_loop(p, -0.30, 1.55, 0.44))

tr = build_track(p, spawn_s=0.30, finish_back=0.30, jitter=0.007, seed=11)

note = ("grand_tour: big showcase circuit -- wide start, sweepers, zigzag, widening corridor, tight 0.18 m "
        "section, progressive narrowing, dead ends, hairpins, chicane, mixed sharp corners, long return. "
        "Walls jittered 0.007 m so none are exactly parallel.")

ok = verify("grand_tour", tr)

xs = [w[0] for w in tr["walls"]] + [w[2] for w in tr["walls"]]
ys = [w[1] for w in tr["walls"]] + [w[3] for w in tr["walls"]]
print(f"    bounding box {max(xs) - min(xs):.2f} x {max(ys) - min(ys):.2f} m")
sx, sy, _ = tr["spawn"]
fx, fy = (tr["finish"][0] + tr["finish"][2]) / 2, (tr["finish"][1] + tr["finish"][3]) / 2
print(f"    spawn ({sx:.2f}, {sy:.2f}) -> finish ({fx:.2f}, {fy:.2f}); gap {math.hypot(fx - sx, fy - sy):.2f} m")

walls = tr["walls"]
print("    per-section width and pivot clearance:")
worst_all = 99.0
for label, a, b in SECTIONS:
    pts = tr["centerline"]
    cum, samples = 0.0, []
    for i in range(1, len(pts)):
        ax, ay = pts[i - 1]; bx, by = pts[i]
        L = math.hypot(bx - ax, by - ay); hd = math.degrees(math.atan2(by - ay, bx - ax))
        n = max(1, int(L / 0.05))
        for k in range(n):
            f = k / n
            s_here = cum + L * f
            if a <= s_here <= b and 0.3 < s_here < _s(p) - 0.3:
                samples.append((ax + (bx - ax) * f, ay + (by - ay) * f, hd))
        cum += L
    if not samples:
        continue
    widths = [ray(walls, x, y, h + 90) + ray(walls, x, y, h - 90) for x, y, h in samples]
    clear = min(min(point_seg_dist(x, y, w) for w in walls) for x, y, _ in samples)
    worst_all = min(worst_all, clear)
    print(f"      {label:20s} s {a:5.2f}-{b:5.2f} m  width {min(widths):.2f}-{max(widths):.2f} m  "
          f"min centre clearance {clear:.3f} m")
print(f"    tightest point on the whole circuit: {worst_all:.3f} m "
      f"({'pivot fits' if worst_all >= ROTATE_RADIUS else 'BELOW pivot radius'} {ROTATE_RADIUS:.3f} m)")

write_track("grand_tour", tr, note, OUT_DIR)
print("ALL OK" if ok else "PROBLEMS FOUND")

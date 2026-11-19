"""Generate the five training tracks (freeform format) and verify each one.

Coordinates: metres, x right, y down (screen), heading 0 = +x, heading in degrees.
Each track is built from a centerline polyline with a width per vertex; left/right walls are
mitred offsets of the centerline, closed with a back cap behind the spawn and an end cap after
the finish, so there is exactly one path from spawn to finish.
"""
import math, sys

from track_lib import *  # noqa: F403  (shared geometry + verification)

# Usage (from the repo root): python tools/gen_tracks.py [out_dir]   (default: tracks)
OUT_DIR = sys.argv[1] if len(sys.argv) > 1 else "../tracks"

def report_progressive(tr):
    """Width profile per section and pivot clearance (exact wall distance) along the whole route."""
    pts, walls = tr["centerline"], tr["walls"]
    sx, sy, _ = tr["spawn"]
    fx = (tr["finish"][0] + tr["finish"][2]) / 2
    fy = (tr["finish"][1] + tr["finish"][3]) / 2
    samples = []  # (s, x, y, heading)
    cum = 0.0
    for i in range(1, len(pts)):
        ax, ay = pts[i - 1]; bx, by = pts[i]
        L = math.hypot(bx - ax, by - ay); hd = math.degrees(math.atan2(by - ay, bx - ax))
        n = max(1, int(L / 0.02))
        for k in range(n):
            f = k / n
            samples.append((cum + L * f, ax + (bx - ax) * f, ay + (by - ay) * f, hd))
        cum += L
    s_spawn = 0.25
    s_finish = cum - 0.30
    route = [smp for smp in samples if s_spawn <= smp[0] <= s_finish]
    print("    width profile (perpendicular, exact) and pivot clearance (centre-to-nearest-wall) along the route:")
    for label, a, b, nominal in PROG_SECTIONS:
        sec = [smp for smp in route if a <= smp[0] <= b]
        if not sec:
            continue
        widths = [ray(walls, x, y, h + 90) + ray(walls, x, y, h - 90) for _, x, y, h in sec]
        clear = [min(point_seg_dist(x, y, w) for w in walls) for _, x, y, _ in sec]
        worst = min(clear)
        print(f"      {label:18s} s {a:5.2f}-{b:5.2f} m ({b - a:4.2f} m): width {min(widths):.3f}-{max(widths):.3f} m, "
              f"min centre clearance {worst:.3f} m -> pivot fits with up to {worst - ROTATE_RADIUS:+.3f} m off-centre")
    worst_all = min(min(point_seg_dist(x, y, w) for w in walls) for _, x, y, _ in route)
    print(f"      whole route spawn->finish: min centre clearance {worst_all:.3f} m "
          f"({'>=' if worst_all >= ROTATE_RADIUS else '<'} pivot radius {ROTATE_RADIUS:.3f} m)")
    return worst_all >= ROTATE_RADIUS


tracks = {}

# 1. corridor_long: straight, 0.30 m wide
p = Path(-0.25, 0.0, 0.0, 0.30).straight(7.0, step=1.0)
tracks["corridor_long"] = (build_track(p, spawn_s=0.25, finish_back=0.30),
                           "corridor_long: straight corridor, 0.30 m wide, 7 m long")

# 2. corridor_widening: straight-ish, width 0.25 -> 0.60, irregular wall segments
p = Path(-0.25, 0.0, 0.0, 0.25)
seg, total = 0.35, 7.5
n = int(total / seg)
for i in range(n):
    s_mid = (i + 1) * seg
    p.h = 3.0 * math.sin(2 * math.pi * s_mid / 3.0)       # gentle heading wobble (+-3 deg)
    p.straight(seg, width=0.25 + 0.35 * (i + 1) / n)
tracks["corridor_widening"] = (build_track(p, spawn_s=0.25, finish_back=0.30, jitter=0.012, seed=7),
                               "corridor_widening: width grows 0.25 -> 0.60 m, irregular non-parallel walls")

# 3. tight_uniform: 0.16 m throughout, smooth curves
p = (Path(-0.20, 0.0, 0.0, 0.16).straight(1.0, step=0.25)
     .arc(90, 0.6).straight(0.6, step=0.2)
     .arc(-90, 0.6).arc(-60, 0.7).straight(0.5, step=0.25)
     .arc(60, 0.7).arc(90, 0.6).straight(0.4, step=0.2)
     .arc(-45, 0.8).arc(45, 0.8).straight(1.0, step=0.25))
tracks["tight_uniform"] = (build_track(p, spawn_s=0.20, finish_back=0.30),
                           "tight_uniform: uniform 0.16 m corridor with smooth curves (radius 0.6-0.8 m)")

# 3b. tight_progressive: one corridor narrowing 0.32 -> 0.28 -> 0.24 -> 0.20 m in ~2 m sections,
#     0.3 m tapers between sections, gentle curves (radius 0.6-0.8 m) in sections 2, 3 and 4.
#     Section boundaries are recorded as centerline distances for the width-profile report.
PROG_SECTIONS = []  # (label, s_start, s_end, nominal_width)
def _s(path):
    return sum(math.hypot(path.pts[i][0] - path.pts[i - 1][0], path.pts[i][1] - path.pts[i - 1][1])
               for i in range(1, len(path.pts)))
p = Path(-0.25, 0.0, 0.0, 0.32)
a = _s(p); p.straight(2.25, step=0.25); PROG_SECTIONS.append(("section 1 (0.32)", a + 0.25, _s(p), 0.32))
a = _s(p); p.straight(0.3, width=0.28, step=0.1); PROG_SECTIONS.append(("taper 1->2", a, _s(p), None))
a = _s(p); p.straight(0.4, step=0.2).arc(60, 0.7).arc(-60, 0.7).straight(0.14); PROG_SECTIONS.append(("section 2 (0.28)", a, _s(p), 0.28))
a = _s(p); p.straight(0.3, width=0.24, step=0.1); PROG_SECTIONS.append(("taper 2->3", a, _s(p), None))
a = _s(p); p.straight(0.6, step=0.2).arc(-45, 0.8).arc(45, 0.8).straight(0.14); PROG_SECTIONS.append(("section 3 (0.24)", a, _s(p), 0.24))
a = _s(p); p.straight(0.3, width=0.20, step=0.1); PROG_SECTIONS.append(("taper 3->4", a, _s(p), None))
a = _s(p); p.straight(0.3, step=0.15).arc(60, 0.6).arc(-60, 0.6).straight(0.74, step=0.25); PROG_SECTIONS.append(("section 4 (0.20)", a, _s(p), 0.20))
tracks["tight_progressive"] = (build_track(p, spawn_s=0.25, finish_back=0.30),
                               "tight_progressive: one corridor narrowing 0.32 -> 0.28 -> 0.24 -> 0.20 m, 0.3 m tapers, curves r 0.6-0.8 m")

# 4. mixed_turns: sharp corners joining wide (0.50) and tight (0.20) sections
p = (Path(-0.30, 0.0, 0.0, 0.50).straight(1.6, step=0.4)
     .turn(90).straight(0.6).straight(0.3, width=0.20).straight(0.8, step=0.4)
     .turn(-90).straight(1.0, step=0.5).straight(0.3, width=0.50).straight(0.8, step=0.4)
     .turn(-120).straight(0.8).straight(0.3, width=0.20).straight(0.4)
     .turn(90).straight(0.6, step=0.3).straight(0.3, width=0.50).straight(1.0, step=0.5))
tracks["mixed_turns"] = (build_track(p, spawn_s=0.30, finish_back=0.30),
                         "mixed_turns: sharp 90/120 deg corners, sections alternate 0.50 m wide and 0.20 m tight")

# 5. maze_deadends: 0.30 m corridors, 90-degree dead-end corners (scan recovery) + 45-degree zigzags
MAZE_W = 0.30
p = Path(-0.25, 0.0, 0.0, MAZE_W)
dead_end_vertices = []
def dead_end(turn_deg):
    dead_end_vertices.append((len(p.pts) - 1, p.h, turn_deg))
    p.turn(turn_deg)
p.straight(1.25, step=0.4); dead_end(-90)
p.straight(1.0, step=0.5); dead_end(90)
p.straight(0.6, step=0.3)
p.turn(45).straight(0.6).turn(-90).straight(0.6).turn(90).straight(0.6).turn(-45)   # zigzag
p.straight(0.7, step=0.35); dead_end(90)
p.straight(1.2, step=0.4); dead_end(-90)
p.straight(0.6, step=0.3)
p.turn(-45).straight(0.6).turn(90).straight(0.6).turn(-45)                            # zigzag
p.straight(0.9, step=0.3)
tracks["maze_deadends"] = (build_track(p, spawn_s=0.25, finish_back=0.30),
                           "maze_deadends: 0.30 m corridors, four 90 deg dead-end corners (scan recovery), two zigzags")

all_ok = True
for name, (tr, note) in tracks.items():
    all_ok &= verify(name, tr)
    if name == "maze_deadends":
        walls = tr["walls"]
        pts = tr["centerline"]
        for k, (vi, h_in, turn) in enumerate(dead_end_vertices, 1):
            vx, vy = pts[vi]
            r = math.radians(h_in)
            # robot centre where the centre ToF (mounted at the centre) reads EMERGENCY_STOP straight ahead
            ahead_at_vertex = ray(walls, vx, vy, h_in)
            back = EMERGENCY_STOP - ahead_at_vertex
            cx, cy = vx - math.cos(r) * back, vy - math.sin(r) * back
            fwd = ray(walls, cx, cy, h_in)
            side_open = ray(walls, cx, cy, h_in + turn)
            side_closed = ray(walls, cx, cy, h_in - turn)
            print(f"    dead end {k}: ahead of corner {ahead_at_vertex:.3f} m -> scan triggers with centre "
                  f"{abs(back):.3f} m {'past' if back < 0 else 'before'} the corner; at trigger: ahead {fwd:.3f} m, "
                  f"opening side {side_open:.2f} m, closed side {side_closed:.3f} m")
    # True perpendicular width along the centerline (grid-free), skipping 0.25 m around sharp corners/caps
    pts = tr["centerline"]; walls = tr["walls"]; widths = []
    cum = 0.0; total = sum(math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]) for i in range(1, len(pts)))
    for i in range(1, len(pts)):
        ax, ay = pts[i-1]; bx, by = pts[i]; L = math.hypot(bx-ax, by-ay); hd = math.degrees(math.atan2(by-ay, bx-ax))
        for k in range(1, 4):
            f = k / 4; x, y = ax + (bx-ax)*f, ay + (by-ay)*f; s_here = cum + L*f
            if s_here < 0.3 or s_here > total - 0.3: continue
            near_corner = any(abs(hd - math.degrees(math.atan2(pts[j+1][1]-pts[j][1], pts[j+1][0]-pts[j][0]))) % 360 > 20
                              and math.hypot(x-pts[j][0], y-pts[j][1]) < 0.35 for j in range(max(0,i-4), min(len(pts)-1, i+3)))
            if near_corner: continue
            widths.append(ray(walls, x, y, hd + 90) + ray(walls, x, y, hd - 90))
        cum += L
    print(f"    perpendicular corridor width along centerline: min {min(widths):.3f} m, max {max(widths):.3f} m")
    if name == "tight_progressive":
        report_progressive(tr)
    write_track(name, tr, note, OUT_DIR)
print("ALL OK" if all_ok else "PROBLEMS FOUND")

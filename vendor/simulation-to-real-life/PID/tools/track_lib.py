"""Shared geometry, track-building and verification helpers for the track generators.

Coordinates: metres, x right, y down (screen), heading 0 = +x, heading in degrees.
A track is built from a centerline polyline with a width per vertex; left/right walls are
mitred offsets of the centerline, closed with a back cap behind the spawn and an end cap after
the finish, so there is exactly one path from spawn to finish.

Used by gen_tracks.py (the five training tracks) and gen_grand_tour.py (the big showcase track).
"""
import math, random
from collections import deque
import numpy as np

CHASSIS_HALF_W = 0.0675        # half the outer width: wheels 12 cm apart (centres) + 1.5 cm wheel width
ROTATE_RADIUS = math.hypot(0.062, 0.0675)  # farthest wheel corner from the origin (wheels stick out past the chassis)
EMERGENCY_STOP = 0.12


# ----------------------------------------------------------------- centerline builders
class Path:
    def __init__(self, x, y, heading_deg, width):
        self.pts = [(x, y)]
        self.w = [width]
        self.h = heading_deg

    def straight(self, length, width=None, step=None):
        """Advance; if width is given it tapers linearly to that width over this run."""
        w0 = self.w[-1]
        w1 = w0 if width is None else width
        n = max(1, int(math.ceil(length / step))) if step else 1
        x, y = self.pts[-1]
        r = math.radians(self.h)
        for i in range(1, n + 1):
            f = i / n
            self.pts.append((x + math.cos(r) * length * f, y + math.sin(r) * length * f))
            self.w.append(w0 + (w1 - w0) * f)
        return self

    def turn(self, deg):
        """Sharp corner: change heading in place (next straight starts from the corner)."""
        self.h += deg
        return self

    def arc(self, deg, radius, step_deg=4.0):
        n = max(1, int(math.ceil(abs(deg) / step_deg)))
        d = deg / n
        chord = 2 * radius * math.sin(math.radians(abs(d)) / 2)
        for _ in range(n):
            self.h += d / 2
            x, y = self.pts[-1]
            r = math.radians(self.h)
            self.pts.append((x + math.cos(r) * chord, y + math.sin(r) * chord))
            self.w.append(self.w[-1])
            self.h += d / 2
        return self


def offset_walls(pts, widths, jitter=0.0, rng=None):
    """Mitred left/right offsets of the centerline; returns (left_pts, right_pts)."""
    left, right = [], []
    n = len(pts)
    for i in range(n):
        if i == 0:
            dx, dy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
            L = math.hypot(dx, dy); nx, ny = -dy / L, dx / L; scale = 1.0
        elif i == n - 1:
            dx, dy = pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]
            L = math.hypot(dx, dy); nx, ny = -dy / L, dx / L; scale = 1.0
        else:
            ax, ay = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
            bx, by = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
            la, lb = math.hypot(ax, ay), math.hypot(bx, by)
            n1 = (-ay / la, ax / la); n2 = (-by / lb, bx / lb)
            mx, my = n1[0] + n2[0], n1[1] + n2[1]
            lm = math.hypot(mx, my)
            nx, ny = mx / lm, my / lm
            scale = 1.0 / max(0.2, nx * n2[0] + ny * n2[1])  # miter length factor
        h = widths[i] / 2 * scale
        jl = (rng.gauss(0, jitter), rng.gauss(0, jitter)) if jitter else (0, 0)
        jr = (rng.gauss(0, jitter), rng.gauss(0, jitter)) if jitter else (0, 0)
        left.append((pts[i][0] + nx * h + jl[0], pts[i][1] + ny * h + jl[1]))
        right.append((pts[i][0] - nx * h + jr[0], pts[i][1] - ny * h + jr[1]))
    return left, right


def polyline_walls(p):
    return [(p[i][0], p[i][1], p[i + 1][0], p[i + 1][1]) for i in range(len(p) - 1)]


def build_track(path, spawn_s, finish_back, jitter=0.0, seed=0):
    """spawn_s: distance along centerline for the spawn; finish_back: finish sits this far before the end cap."""
    rng = random.Random(seed)
    pts, widths = path.pts, path.w
    left, right = offset_walls(pts, widths, jitter, rng)
    walls = polyline_walls(left) + polyline_walls(right)
    walls.append((left[0][0], left[0][1], right[0][0], right[0][1]))      # back cap
    walls.append((left[-1][0], left[-1][1], right[-1][0], right[-1][1]))  # end cap

    cum = [0.0]
    for i in range(1, len(pts)):
        cum.append(cum[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
    total = cum[-1]

    def at(s):
        for i in range(1, len(pts)):
            if cum[i] >= s:
                f = (s - cum[i - 1]) / (cum[i] - cum[i - 1])
                x = pts[i - 1][0] + (pts[i][0] - pts[i - 1][0]) * f
                y = pts[i - 1][1] + (pts[i][1] - pts[i - 1][1]) * f
                hd = math.degrees(math.atan2(pts[i][1] - pts[i - 1][1], pts[i][0] - pts[i - 1][0]))
                w = widths[i - 1] + (widths[i] - widths[i - 1]) * f
                return x, y, hd, w
        return pts[-1][0], pts[-1][1], 0.0, widths[-1]

    sx, sy, sh, _ = at(spawn_s)
    fx, fy, fh, fw = at(total - finish_back)
    r = math.radians(fh)
    nx, ny = -math.sin(r), math.cos(r)
    half = fw / 2 + 0.02  # reach just past the jittered walls
    finish = (fx + nx * half, fy + ny * half, fx - nx * half, fy - ny * half)
    return dict(walls=walls, spawn=(sx, sy, sh), finish=finish, path_len=total - finish_back - spawn_s,
                centerline=pts, left=left, right=right)


# ----------------------------------------------------------------- verification
def seg_intersect(a, b, allow_shared=True):
    (x1, y1, x2, y2), (x3, y3, x4, y4) = a, b
    if allow_shared:
        for p in ((x1, y1), (x2, y2)):
            for q in ((x3, y3), (x4, y4)):
                if abs(p[0] - q[0]) < 1e-6 and abs(p[1] - q[1]) < 1e-6:
                    return False
    rx, ry, sx, sy = x2 - x1, y2 - y1, x4 - x3, y4 - y3
    cr = rx * sy - ry * sx
    if abs(cr) < 1e-12:
        return False
    qx, qy = x3 - x1, y3 - y1
    t = (qx * sy - qy * sx) / cr
    u = (qx * ry - qy * rx) / cr
    return 0 < t < 1 and 0 < u < 1


def wall_crossings(walls):
    bad = 0
    for i in range(len(walls)):
        for j in range(i + 1, len(walls)):
            if seg_intersect(walls[i], walls[j]):
                bad += 1
    return bad


def distance_field(walls, res=0.01, pad=0.3):
    xs = [w[0] for w in walls] + [w[2] for w in walls]
    ys = [w[1] for w in walls] + [w[3] for w in walls]
    x0, y0 = min(xs) - pad, min(ys) - pad
    nx = int((max(xs) + pad - x0) / res) + 1
    ny = int((max(ys) + pad - y0) / res) + 1
    gx = x0 + np.arange(nx) * res
    gy = y0 + np.arange(ny) * res
    X, Y = np.meshgrid(gx, gy)
    D = np.full(X.shape, np.inf)
    for (ax, ay, bx, by) in walls:
        vx, vy = bx - ax, by - ay
        l2 = vx * vx + vy * vy
        t = np.clip(((X - ax) * vx + (Y - ay) * vy) / l2, 0, 1) if l2 > 0 else 0
        d = np.hypot(X - (ax + t * vx), Y - (ay + t * vy))
        np.minimum(D, d, out=D)
    return D, x0, y0, res


def reachable(D, x0, y0, res, start, goal, clearance):
    free = D >= clearance
    si, sj = int(round((start[1] - y0) / res)), int(round((start[0] - x0) / res))
    gi, gj = int(round((goal[1] - y0) / res)), int(round((goal[0] - x0) / res))
    if not free[si, sj]:
        return False, False
    seen = np.zeros_like(free)
    seen[si, sj] = True
    q = deque([(si, sj)])
    H, W = free.shape
    leak = False
    while q:
        i, j = q.popleft()
        if i in (0, H - 1) or j in (0, W - 1):
            leak = True
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < H and 0 <= b < W and free[a, b] and not seen[a, b]:
                seen[a, b] = True
                q.append((a, b))
    return bool(seen[gi, gj]), leak


def bottleneck(D, x0, y0, res, start, goal):
    lo, hi = 0.0, 0.5
    for _ in range(12):
        mid = (lo + hi) / 2
        ok, _ = reachable(D, x0, y0, res, start, goal, mid)
        lo, hi = (mid, hi) if ok else (lo, mid)
    return lo


def ray(walls, x, y, heading_deg, max_r=4.0):
    r = math.radians(heading_deg)
    dx, dy = math.cos(r), math.sin(r)
    best = max_r
    for (ax, ay, bx, by) in walls:
        sx, sy = bx - ax, by - ay
        cr = dx * sy - dy * sx
        if abs(cr) < 1e-12:
            continue
        qx, qy = ax - x, ay - y
        t = (qx * sy - qy * sx) / cr
        u = (qx * dy - qy * dx) / cr
        if t >= 0 and 0 <= u <= 1:
            best = min(best, t)
    return best


def write_track(name, tr, note, out_dir):
    path = f"{out_dir}/{name}.track"
    with open(path, "w", newline="\n") as f:
        f.write(f"# {note}\n")
        sx, sy, sh = tr["spawn"]
        f.write(f"spawn {sx:.3f} {sy:.3f} {sh:.1f}\n\n")
        for w in tr["walls"]:
            f.write("wall {:.3f} {:.3f} {:.3f} {:.3f}\n".format(*w))
        f.write("\nfinish {:.3f} {:.3f} {:.3f} {:.3f}\n".format(*tr["finish"]))
    return path


def verify(name, tr):
    walls = tr["walls"]
    crossings = wall_crossings(walls)
    D, x0, y0, res = distance_field(walls)
    fx = (tr["finish"][0] + tr["finish"][2]) / 2
    fy = (tr["finish"][1] + tr["finish"][3]) / 2
    start = tr["spawn"][:2]
    ok_drive, leak = reachable(D, x0, y0, res, start, (fx, fy), CHASSIS_HALF_W + 0.01)
    ok_rot, _ = reachable(D, x0, y0, res, start, (fx, fy), ROTATE_RADIUS)
    neck = bottleneck(D, x0, y0, res, start, (fx, fy))
    print(f"{name}: walls={len(walls)} path~{tr['path_len']:.2f} m  wall crossings={crossings}  enclosed={not leak}")
    print(f"    spawn->finish connected at clearance {CHASSIS_HALF_W + 0.01:.3f} m (drive): {ok_drive}; "
          f"at {ROTATE_RADIUS:.3f} m (rotate in place anywhere): {ok_rot}; bottleneck clearance {neck:.3f} m "
          f"(= min corridor width {2 * neck:.3f} m)")
    return crossings == 0 and not leak and ok_drive


def point_seg_dist(px, py, w):
    ax, ay, bx, by = w
    vx, vy = bx - ax, by - ay
    l2 = vx * vx + vy * vy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / l2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))

"""60 x 31 m glasshouse with crop rows: the same aisle / cross-aisle topology as the warehouse (so the
UGV + UAV inspection mission runs on it unchanged), but crops instead of racks.

Layout (x forward, y left, origin = spawn point, in the middle of the y=0 main aisle):
  - 8 crop rows parallel to x at y = +-1.9, +-5.7, +-9.5, +-13.3 (0.8 m deep), leaving 3 m wide
    aisles between them (main aisle at y in [-1.5, 1.5]) and ~1.9 m strips along the side walls.
  - every row is cut by 3 m wide cross-aisles at x = -14, 0, 14 into 4 segments (2 m strips along the
    end walls).
  - crops (ROW_CROPS, outside -> main aisle): strawberries on raised gutters (1.1 m), cucumbers,
    bell peppers, high-wire tomatoes next to the main aisle. Tall crops grow from a gutter on legs
    (~0.55 m) up to the crop support wire (3.0 m); steel greenhouse posts at the segment ends / middle.
  - glass side walls (4.5 m), roof gutters above every row line at 4.4 m (no roof panes).
  - blue harvest crates in every aisle: *temporary* obstacles (model name temp_obstacle*), left out of
    the prior map used by the planners -- only the onboard sensors see them.

The plants are detailed meshes (stems, compound / broad leaves, tomato trusses, cucumbers, peppers,
strawberries), one merged OBJ per row segment, written next to the world into
models/greenhouse_crops/meshes (model://greenhouse_crops/...: the session links that model into PX4's
gz models folder). They are VISUAL only -- what the cameras and gpu_lidars see, gaps between leaves
included. Collision (physics + the planners' prior map) is a simple box per row segment around the
canopy, plus the gutters / legs / wire / posts as boxes.
"""
import math
import os
import random

from ..sdf import FOOTER, box, header, model
from .. import World

X_MIN, X_MAX = -30.0, 30.0
Y_MIN, Y_MAX = -15.6, 15.6
WALL_H, WALL_T = 4.5, 0.1

ROW_DEPTH = 0.8
ROW_YS = [-13.3, -9.5, -5.7, -1.9, 1.9, 5.7, 9.5, 13.3]
ROW_CROPS = ["strawberry", "cucumber", "pepper", "tomato", "tomato", "pepper", "cucumber", "strawberry"]
AISLE_YS = [-11.4, -7.6, -3.8, 0.0, 3.8, 7.6, 11.4]
CROSS_AISLES = [-14.0, 0.0, 14.0]  # x centres
CROSS_W = 3.0
GUTTER_Z, GUTTER_H, GUTTER_W = 0.50, 0.12, 0.30   # tall crops
SB_GUTTER_Z = 1.10                                # strawberries: raised gutter
WIRE_Z = 3.0
POST_H = 4.4

STEEL = (0.75, 0.76, 0.78)
GUTTER = (0.92, 0.92, 0.90)
SOIL = (0.30, 0.24, 0.18)
GLASS = (0.70, 0.80, 0.85)
CRATE_BLUE = (0.10, 0.30, 0.80)

TEMP_PREFIX = "temp_obstacle"
TMP_SIZE = (0.6, 0.4, 0.7)  # [m] stacked harvest crates
TMP_Y = 0.75                # [m] off the aisle centreline: occupies |y| 0.55..0.95, ~2 m free on the other side
TMP_PER_AISLE = 3

MESH_MODEL = "greenhouse_crops"
MATERIALS = {  # name: (r, g, b)
    "stem": (0.33, 0.52, 0.18),
    "leaf": (0.16, 0.45, 0.12),
    "leaf2": (0.22, 0.55, 0.16),
    "leaf_dark": (0.10, 0.32, 0.08),
    "tomato_red": (0.85, 0.08, 0.05),
    "tomato_orange": (0.95, 0.45, 0.08),
    "tomato_green": (0.45, 0.70, 0.20),
    "cucumber": (0.12, 0.38, 0.10),
    "flower": (0.98, 0.85, 0.10),
    "pepper_red": (0.80, 0.05, 0.05),
    "pepper_yellow": (0.98, 0.80, 0.05),
    "pepper_orange": (0.98, 0.50, 0.05),
    "pepper_green": (0.20, 0.50, 0.12),
    "strawberry": (0.85, 0.06, 0.10),
    "strawberry_unripe": (0.85, 0.90, 0.70),
    "slab": (0.90, 0.90, 0.86),
}


# ------------------------------------------------------------------------------------------------
# mesh building (OBJ, per-face normals, leaves two-sided)
# ------------------------------------------------------------------------------------------------
def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a):
    n = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
    return (a[0] / n, a[1] / n, a[2] / n) if n > 1e-12 else (0.0, 0.0, 1.0)


def _perp(axis):
    """Two unit vectors perpendicular to `axis` and to each other."""
    a = _norm(axis)
    ref = (0.0, 0.0, 1.0) if abs(a[2]) < 0.9 else (1.0, 0.0, 0.0)
    u = _norm(_cross(a, ref))
    return u, _cross(a, u)


class Mesh:
    def __init__(self):
        self.v = []
        self.n = []
        self.f = {}  # material -> [(v1, v2, v3, normal index)]

    def _vert(self, p):
        self.v.append(p)
        return len(self.v)

    def tri(self, mat, a, b, c, two_sided=False):
        ia, ib, ic = self._vert(a), self._vert(b), self._vert(c)
        nrm = _norm(_cross(_sub(b, a), _sub(c, a)))
        self.n.append(nrm)
        self.f.setdefault(mat, []).append((ia, ib, ic, len(self.n)))
        if two_sided:
            self.n.append(_mul(nrm, -1))
            self.f[mat].append((ia, ic, ib, len(self.n)))

    def quad(self, mat, a, b, c, d, two_sided=False):
        self.tri(mat, a, b, c, two_sided)
        self.tri(mat, a, c, d, two_sided)

    def cylinder(self, mat, p0, p1, r, sides=5):
        u, w = _perp(_sub(p1, p0))
        ring = [_add(_mul(u, r * math.cos(2 * math.pi * k / sides)), _mul(w, r * math.sin(2 * math.pi * k / sides)))
                for k in range(sides)]
        for k in range(sides):
            a, b = ring[k], ring[(k + 1) % sides]
            self.quad(mat, _add(p0, a), _add(p0, b), _add(p1, b), _add(p1, a))

    def ellipsoid(self, mat, c, rx, ry, rz, nu=6, nv=4):
        def pt(i, j):
            th = math.pi * j / nv
            ph = 2 * math.pi * i / nu
            return (c[0] + rx * math.sin(th) * math.cos(ph), c[1] + ry * math.sin(th) * math.sin(ph),
                    c[2] + rz * math.cos(th))
        for j in range(nv):
            for i in range(nu):
                a, b, cc, d = pt(i, j), pt(i + 1, j), pt(i + 1, j + 1), pt(i, j + 1)
                if j == 0:
                    self.tri(mat, a, cc, d)
                elif j == nv - 1:
                    self.tri(mat, a, b, cc)
                else:
                    self.quad(mat, a, b, cc, d)

    def box(self, mat, c, sx, sy, sz):
        x0, x1 = c[0] - sx / 2, c[0] + sx / 2
        y0, y1 = c[1] - sy / 2, c[1] + sy / 2
        z0, z1 = c[2] - sz / 2, c[2] + sz / 2
        P = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        for q in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
            self.quad(mat, *[P[i] for i in q])

    def leaf(self, mat, base, direction, length, width, droop=0.0):
        """Diamond-shaped leaf blade from `base` along `direction` (two-sided), tip drooping by `droop`."""
        d = _norm(direction)
        side = _norm(_cross(d, (0.0, 0.0, 1.0))) if abs(d[2]) < 0.95 else (1.0, 0.0, 0.0)
        mid = _add(base, _mul(d, length * 0.4))
        tip = _add(_add(base, _mul(d, length)), (0.0, 0.0, -droop))
        l = _add(_add(mid, _mul(side, width / 2)), (0.0, 0.0, -droop * 0.3))
        r = _add(_sub(mid, _mul(side, width / 2)), (0.0, 0.0, -droop * 0.3))
        self.quad(mat, base, l, tip, r, two_sided=True)

    def broad_leaf(self, mat, center, normal, radius, rng, lobes=5):
        """Palmate leaf (cucumber): lobed fan around `center`, facing `normal` (two-sided)."""
        u, w = _perp(normal)
        pts = []
        for k in range(lobes * 2):
            a = 2 * math.pi * k / (lobes * 2)
            rr = radius * (1.0 if k % 2 == 0 else 0.62) * rng.uniform(0.9, 1.1)
            pts.append(_add(center, _add(_mul(u, rr * math.cos(a)), _mul(w, rr * math.sin(a)))))
        for k in range(len(pts)):
            self.tri(mat, center, pts[k], pts[(k + 1) % len(pts)], two_sided=True)

    def flower(self, mat, center, radius):
        for k in range(5):
            a0, a1 = 2 * math.pi * k / 5, 2 * math.pi * (k + 0.5) / 5
            self.tri(mat, center, _add(center, (radius * math.cos(a0), radius * math.sin(a0), 0.0)),
                     _add(center, (radius * 0.5 * math.cos(a1), radius * 0.5 * math.sin(a1), 0.0)), two_sided=True)

    def write(self, path, mtl_name, name):
        with open(path, "w") as f:
            f.write(f"# generated by worldgen/worlds/greenhouse.py\nmtllib {mtl_name}\no {name}\n")
            for p in self.v:
                f.write(f"v {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            for n in self.n:
                f.write(f"vn {n[0]:.3f} {n[1]:.3f} {n[2]:.3f}\n")
            for mat, faces in self.f.items():
                f.write(f"usemtl {mat}\n")
                for a, b, c, n in faces:
                    f.write(f"f {a}//{n} {b}//{n} {c}//{n}\n")

    def triangles(self):
        return sum(len(v) for v in self.f.values())


def write_mtl(path):
    with open(path, "w") as f:
        for name, (r, g, b) in MATERIALS.items():
            f.write(f"newmtl {name}\nKa {r * 0.6:.3f} {g * 0.6:.3f} {b * 0.6:.3f}\nKd {r:.3f} {g:.3f} {b:.3f}\n"
                    f"Ks 0.05 0.05 0.05\nNs 10\nd 1.0\nillum 2\n\n")


# ------------------------------------------------------------------------------------------------
# crops (each draws one plant at x = px on a row at y = y; side = +1 / -1 picks the aisle it leans to)
# ------------------------------------------------------------------------------------------------
TOP_GUTTER = GUTTER_Z + GUTTER_H / 2


def _horiz(theta, down=0.0):
    return (math.cos(theta), math.sin(theta), -down)


def tomato(m, rng, px, y):
    h = rng.uniform(2.4, 2.85)
    lean = rng.uniform(-0.04, 0.04)
    top = (px + rng.uniform(-0.05, 0.05), y + lean, h)
    m.cylinder("stem", (px, y, TOP_GUTTER), top, 0.012)

    def at(z):  # point on the stem at height z
        t = (z - TOP_GUTTER) / (h - TOP_GUTTER)
        return (px + (top[0] - px) * t, y + (top[1] - y) * t, z)

    # compound leaves, alternating towards the two aisles
    z, k = 0.75, 0
    while z < h - 0.1:
        side = 1 if k % 2 == 0 else -1
        th = side * math.pi / 2 + rng.uniform(-0.6, 0.6)
        base = at(z)
        L = rng.uniform(0.34, 0.45)
        d = _norm(_horiz(th, 0.25))
        tip = _add(base, _mul(d, L))
        m.cylinder("stem", base, tip, 0.004, sides=3)
        for j in range(4):  # leaflet pairs along the rachis + terminal one
            p = _add(base, _mul(d, L * (0.25 + 0.2 * j)))
            for s in (-1, 1):
                ld = _norm(_add(_horiz(th + s * rng.uniform(0.9, 1.4)), (0.0, 0.0, 0.1)))
                m.leaf(rng.choice(["leaf", "leaf2", "leaf_dark"]), p, ld, rng.uniform(0.11, 0.16),
                       rng.uniform(0.06, 0.09), 0.03)
        m.leaf("leaf", tip, d, rng.uniform(0.12, 0.16), 0.07, 0.03)
        z += rng.uniform(0.13, 0.19)
        k += 1
    # trusses: lower = riper
    z, k = 0.75, 0
    while z < min(h - 0.5, 1.95):
        side = 1 if k % 2 == 0 else -1
        base = at(z)
        stalk = _add(base, (rng.uniform(-0.05, 0.05), side * rng.uniform(0.08, 0.14), -0.05))
        m.cylinder("stem", base, stalk, 0.004, sides=3)
        color = "tomato_red" if z < 1.35 else ("tomato_orange" if z < 1.7 and rng.random() < 0.6 else "tomato_green")
        if z < 1.35 and rng.random() < 0.15:
            color = "tomato_orange"
        n = rng.randint(4, 6)
        for i in range(n):
            r = rng.uniform(0.027, 0.036)
            c = _add(stalk, (0.07 * (i - (n - 1) / 2) * 0.8, side * 0.03 * (i % 2), -0.05 - 0.035 * (i % 3)))
            m.ellipsoid(color, c, r, r, r * 0.92)
        z += rng.uniform(0.35, 0.5)
        k += 1
    # growing point
    for i in range(4):
        m.leaf("leaf2", top, _norm(_horiz(rng.uniform(0, 2 * math.pi), -0.3)), 0.08, 0.04)
    return h


def cucumber(m, rng, px, y):
    h = rng.uniform(2.3, 2.75)
    top = (px + rng.uniform(-0.05, 0.05), y + rng.uniform(-0.04, 0.04), h)
    m.cylinder("stem", (px, y, TOP_GUTTER), top, 0.011)

    def at(z):
        t = (z - TOP_GUTTER) / (h - TOP_GUTTER)
        return (px + (top[0] - px) * t, y + (top[1] - y) * t, z)

    z, k = 0.8, 0
    while z < h - 0.1:
        side = 1 if k % 2 == 0 else -1
        th = side * math.pi / 2 + rng.uniform(-0.7, 0.7)
        base = at(z)
        d = _norm(_horiz(th, -0.2))
        pet = _add(base, _mul(d, rng.uniform(0.10, 0.15)))
        m.cylinder("stem", base, pet, 0.005, sides=3)
        r = rng.uniform(0.15, 0.21)
        center = _add(pet, _mul(_norm(_horiz(th, 0.1)), r * 0.8))
        normal = _norm(_add(_horiz(th), (0.0, 0.0, rng.uniform(0.6, 1.4))))
        m.broad_leaf(rng.choice(["leaf", "leaf_dark"]), center, normal, r, rng)
        if 0.9 < z < 2.1 and rng.random() < 0.7:  # cucumber hanging from the leaf axil
            cl = rng.uniform(0.10, 0.15)
            c = _add(base, (rng.uniform(-0.04, 0.04), -side * 0.06, -cl - 0.02))
            m.ellipsoid("cucumber", c, 0.022, 0.022, cl, nu=6, nv=5)
        elif rng.random() < 0.4:
            m.flower("flower", _add(base, (0.0, -side * 0.05, 0.02)), 0.03)
        z += rng.uniform(0.15, 0.22)
        k += 1
    return h


PEPPER_COLORS = ["pepper_red", "pepper_yellow", "pepper_orange"]


def pepper(m, rng, px, y):
    h = rng.uniform(1.6, 2.1)
    ripe = rng.choice(PEPPER_COLORS)
    for s in (-1, 1):  # V system: two stems
        top = (px + s * rng.uniform(0.08, 0.14), y + rng.uniform(-0.06, 0.06), h - rng.uniform(0, 0.15))
        base = (px, y, TOP_GUTTER)
        m.cylinder("stem", base, top, 0.010)
        z = TOP_GUTTER + 0.2
        while z < top[2] - 0.05:
            t = (z - TOP_GUTTER) / (top[2] - TOP_GUTTER)
            p = (px + (top[0] - px) * t, y + (top[1] - y) * t, z)
            for _ in range(3):
                m.leaf(rng.choice(["leaf", "leaf2", "leaf_dark"]), p, _norm(_horiz(rng.uniform(0, 2 * math.pi), 0.1)),
                       rng.uniform(0.12, 0.18), rng.uniform(0.06, 0.09), 0.03)
            if 0.8 < z < h - 0.25 and rng.random() < 0.3:
                th = rng.uniform(0, 2 * math.pi)
                c = _add(p, (0.07 * math.cos(th), 0.07 * math.sin(th), -0.07))
                color = ripe if rng.random() < 0.75 else "pepper_green"
                m.ellipsoid(color, c, 0.040, 0.040, 0.050, nu=7, nv=5)
            z += rng.uniform(0.07, 0.11)
    return h


def strawberry(m, rng, px, y):
    """One rosette on the raised gutter (top at SB_GUTTER_Z + GUTTER_H / 2), berries over both edges."""
    zc = SB_GUTTER_Z + GUTTER_H / 2
    crown = (px, y, zc + 0.02)
    for k in range(rng.randint(8, 11)):
        th = 2 * math.pi * k / 9 + rng.uniform(-0.3, 0.3)
        d = _norm((math.cos(th) * 0.6, math.sin(th), 0.7))
        tip = _add(crown, _mul(d, rng.uniform(0.10, 0.17)))
        m.cylinder("stem", crown, tip, 0.003, sides=3)
        for s in (-0.9, 0.0, 0.9):  # trifoliate
            m.leaf(rng.choice(["leaf", "leaf2", "leaf_dark"]), tip, _norm(_add(_horiz(th + s), (0.0, 0.0, 0.25))),
                   rng.uniform(0.07, 0.09), rng.uniform(0.055, 0.07), 0.01)
    for k in range(rng.randint(3, 5)):
        side = rng.choice([-1, 1])
        c = (px + rng.uniform(-0.1, 0.1), y + side * rng.uniform(0.16, 0.19), zc - rng.uniform(0.03, 0.09))
        m.cylinder("stem", _add(crown, (0, side * 0.05, 0)), _add(c, (0, 0, 0.02)), 0.002, sides=3)
        m.ellipsoid("strawberry" if rng.random() < 0.75 else "strawberry_unripe", c, 0.015, 0.015, 0.022, nu=6, nv=4)
    return zc + 0.2


CROPS = {"tomato": (tomato, 0.55), "cucumber": (cucumber, 0.6), "pepper": (pepper, 0.45),
         "strawberry": (strawberry, 0.25)}  # crop: (plant function, spacing along the row [m])


# ------------------------------------------------------------------------------------------------
# world
# ------------------------------------------------------------------------------------------------
def segments():
    """Row segments along x, split by the cross-aisles: list of (x_start, x_end)."""
    edges = [X_MIN + 2.0] + [v for c in CROSS_AISLES for v in (c - CROSS_W / 2, c + CROSS_W / 2)] + [X_MAX - 2.0]
    return [(edges[i], edges[i + 1]) for i in range(0, len(edges), 2)]


def collision_box(name, cx, cy, cz, sx, sy, sz):
    return f"""        <collision name="{name}_c">
          <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        </collision>
"""


def crop_row(rng, idx, x0, x1, y, crop, mesh_dir):
    length = x1 - x0
    cx = (x0 + x1) / 2
    plant, step = CROPS[crop]
    gz = SB_GUTTER_Z if crop == "strawberry" else GUTTER_Z
    body = box("soil", cx, y, 0.002, length, ROW_DEPTH + 0.2, 0.004, SOIL, collide=False)
    body += box("gutter", cx, y, gz, length, GUTTER_W, GUTTER_H, GUTTER)
    nlegs = max(2, round(length / 2.5) + 1)
    for i in range(nlegs):
        lx = x0 + 0.1 + i * (length - 0.2) / (nlegs - 1)
        body += box(f"leg{i}", lx, y, (gz - GUTTER_H / 2) / 2, 0.05, 0.25, gz - GUTTER_H / 2, STEEL)
    body += box("wire", cx, y, WIRE_Z, length, 0.02, 0.02, STEEL)
    for k, px in enumerate((x0 + 0.05, cx, x1 - 0.05)):
        body += box(f"post{k}", px, y, POST_H / 2, 0.08, 0.08, POST_H, STEEL)

    # plants (visual mesh) + rockwool slabs on the gutter
    m = Mesh()
    top = gz + GUTTER_H / 2
    nslabs = int(length / 1.0)
    for i in range(nslabs):
        m.box("slab", (x0 + (i + 0.5) * length / nslabs, y, top + 0.04), length / nslabs - 0.05, 0.2, 0.08)
    hmax = 0.0
    nplants = int(length / step)
    for i in range(nplants):
        px = x0 + (i + 0.5) * length / nplants
        if crop != "strawberry" and (abs(px - cx) < 0.12 or px - x0 < 0.12 or x1 - px < 0.12):
            continue  # posts
        hmax = max(hmax, plant(m, rng, px, y))
    name = f"row{idx}"
    m.write(os.path.join(mesh_dir, name + ".obj"), "crops.mtl", name)
    body += f"""        <visual name="plants_v">
          <geometry><mesh><uri>model://{MESH_MODEL}/meshes/{name}.obj</uri></mesh></geometry>
        </visual>
"""
    # collision of the canopy: one box per segment (collision only, the mesh is what sensors see)
    if crop == "strawberry":
        body += collision_box("canopy", cx, y, top + 0.12, length, 0.5, 0.24)
    else:
        z0 = 0.75
        body += collision_box("canopy", cx, y, (z0 + hmax) / 2, length, 0.6, hmax - z0)
    return model(f"croprow{idx}", body), m.triangles()


def roof_gutters():
    body = ""
    for i, y in enumerate(ROW_YS):
        body += box(f"g{i}", 0.0, y, POST_H + 0.05, X_MAX - X_MIN, 0.15, 0.1, STEEL)
    return model("roof_gutters", body)


def walls():
    xm, ym = (X_MIN + X_MAX) / 2, (Y_MIN + Y_MAX) / 2
    lx, ly = X_MAX - X_MIN + WALL_T, Y_MAX - Y_MIN + WALL_T
    body = box("n", xm, Y_MAX + WALL_T / 2, WALL_H / 2, lx, WALL_T, WALL_H, GLASS)
    body += box("s", xm, Y_MIN - WALL_T / 2, WALL_H / 2, lx, WALL_T, WALL_H, GLASS)
    body += box("e", X_MAX + WALL_T / 2, ym, WALL_H / 2, WALL_T, ly, WALL_H, GLASS)
    body += box("w", X_MIN - WALL_T / 2, ym, WALL_H / 2, WALL_T, ly, WALL_H, GLASS)
    return model("walls", body)


def temp_obstacles(seed):
    """Blue harvest crates, TMP_PER_AISLE per aisle, alternating sides of the centreline. Out of the
    cross-aisles and the spawn area, >= 2.5 m apart along x, and >= 2 m from the segment midpoints (the
    UGV stops of deploy.sh: the planners do not know these crates, a stop must not land on one)."""
    rng = random.Random(seed)
    mids = [(x0 + x1) / 2 for x0, x1 in segments()]
    out, taken, n = "", [], 0
    for ay in AISLE_YS:
        for k in range(TMP_PER_AISLE):
            for _try in range(200):
                x = rng.uniform(X_MIN + 4, X_MAX - 4)
                if abs(x) < 3.0 and abs(ay) < 3.0:
                    continue
                if any(abs(x - c) < CROSS_W / 2 + 0.6 for c in CROSS_AISLES):
                    continue
                if any(abs(x - m) < 2.0 for m in mids):
                    continue
                if any(abs(py - ay) < 1.5 and abs(x - px) < 2.5 for px, py in taken):
                    continue
                y = ay + (TMP_Y if (k + n) % 2 == 0 else -TMP_Y)
                taken.append((x, y))
                out += model(f"{TEMP_PREFIX}{n}", box("crate", x, y, TMP_SIZE[2] / 2, *TMP_SIZE, CRATE_BLUE))
                n += 1
                break
    return out


def build(rng, params):
    worlds_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    model_dir = os.path.join(worlds_dir, "models", MESH_MODEL)
    mesh_dir = os.path.join(model_dir, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    write_mtl(os.path.join(mesh_dir, "crops.mtl"))
    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{MESH_MODEL}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Crop meshes (one per row segment) of worlds/greenhouse.sdf, generated by
    worldgen/worlds/greenhouse.py -- not spawned on its own, the world references the meshes.</description>
</model>
""")
    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(f'<?xml version="1.0"?>\n<sdf version="1.9"><model name="{MESH_MODEL}"><static>true</static>'
                f'<link name="link"/></model></sdf>\n')

    out = header("greenhouse", ground=(0.62, 0.62, 0.60), ground_size=(100, 60), background=(0.75, 0.85, 0.95),
                 ambient=0.6)
    out += walls() + roof_gutters()
    i, tris = 0, 0
    for y, crop in zip(ROW_YS, ROW_CROPS):
        for x0, x1 in segments():
            xml, t = crop_row(rng, i, x0, x1, y, crop, mesh_dir)
            out += xml
            tris += t
            i += 1
    print(f"greenhouse: {i} crop meshes, {tris} triangles in {mesh_dir}", file=__import__("sys").stderr)
    out += temp_obstacles(rng.randint(0, 10**6))
    return out + FOOTER


# Spawn is in the y=0 main aisle, in the x=0 cross-aisle. The goal sits in the y=7.6 aisle near the
# far -x end: the UAV has to leave the main aisle through a cross-aisle and pass crop rows / crates.
WORLD = World(
    name="greenhouse",
    description="60 x 31 m glasshouse: tomato / cucumber / pepper / strawberry rows, aisles, cross-aisles, crates",
    goal=(-25, 7.6, 1.5, 0.0),
    build=build,
)

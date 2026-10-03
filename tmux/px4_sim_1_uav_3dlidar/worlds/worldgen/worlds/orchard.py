"""Fruit orchard, ~80 x 60 m: long tree rows with narrow aisles, the classic agricultural setting.

  - rows of trees parallel to x every 5 m in y (aisles at y = 0, +-5, +-10, ...; the UAV spawns in
    the y=0 aisle), trees every ~2.5 m in a row with crowns at flight altitude (0.8-3.5 m), so an
    aisle leaves only ~2-3 m of free width.
  - trellis: posts every few trees and thin wires between them (hard for a lidar to see).
  - a headland (cross-track) at x = -30 is the only way between aisles; the goal is in another
    aisle beyond it, so the UAV has to fly down its aisle, turn through the headland, and back in.
  - some trees missing (gaps), occasional bins and a tractor parked in the headland.
"""
import math

from .. import World
from ..placement import Placer
from ..sdf import FOOTER, box, cylinder, header, jitter, model, sphere

X0, X1 = -68.0, 14.0
ROW_SPACING = 5.0
HEADLAND = (-33.0, -27.0)
GOAL = (-60.0, 10.0, 2.0, 0.0)

BARK = (0.35, 0.26, 0.17)
LEAF = (0.25, 0.48, 0.18)
FRUIT = [(0.80, 0.12, 0.10), (0.85, 0.70, 0.15)]
POST = (0.55, 0.50, 0.42)
WIRE = (0.70, 0.70, 0.72)
BIN = (0.55, 0.40, 0.22)
TRACTOR = (0.15, 0.45, 0.15)


def tree(rng, name, x, y, p):
    h = rng.uniform(1.2, 1.8)
    body = cylinder("trunk", x, y, h / 2, rng.uniform(0.06, 0.11), h, jitter(BARK, rng))
    r = rng.uniform(*p["crown_radius"])
    for k in range(rng.randint(2, 4)):
        body += sphere(f"c{k}", x + rng.uniform(-0.4, 0.4), y + rng.uniform(-0.2, 0.2),
                       h + r * rng.uniform(0.4, 0.9) + k * 0.4, r * rng.uniform(0.7, 1.0), jitter(LEAF, rng, 0.06))
    if p["fruit"]:
        colour = rng.choice(FRUIT)
        for k in range(rng.randint(3, 8)):
            a = rng.uniform(0, 2 * math.pi)
            body += sphere(f"f{k}", x + r * 0.9 * math.cos(a), y + r * 0.9 * math.sin(a),
                           h + rng.uniform(0.2, 1.4), 0.06, colour, collide=False)
    return model(name, body)


def row_segment(rng, idx, y, x0, x1, p):
    """Trees + trellis between x0 and x1 at row y."""
    out = ""
    n = 0
    x = x0
    while x <= x1:
        if rng.random() > p["gap_share"]:
            out += tree(rng, f"tree{idx}_{n}", x + rng.uniform(-0.2, 0.2), y + rng.uniform(-0.15, 0.15), p)
        n += 1
        x += p["tree_spacing"]
    if p["trellis"]:
        body = ""
        posts = [x0 + k * 4 * p["tree_spacing"] for k in range(int((x1 - x0) / (4 * p["tree_spacing"])) + 1)]
        for k, px in enumerate(posts):
            body += cylinder(f"p{k}", px + p["tree_spacing"] / 2, y, 1.4, 0.05, 2.8, POST)
        length = posts[-1] - posts[0]
        for k, z in enumerate((0.9, 1.7, 2.5)):
            body += cylinder(f"w{k}", (posts[0] + posts[-1]) / 2 + p["tree_spacing"] / 2, y, z, 0.012, length,
                             WIRE, rpy=(0, math.pi / 2, 0))
        out += model(f"trellis{idx}", body)
    return out


def build(rng, p):
    out = header("orchard", ground=(0.36, 0.42, 0.22), ground_size=(220, 160), background=(0.62, 0.76, 0.90))
    rows = [ROW_SPACING / 2 + k * ROW_SPACING for k in range(p["rows_per_side"])]
    rows = sorted([-r for r in rows] + rows)
    idx = 0
    for y in rows:
        for x0, x1 in ((X0, HEADLAND[0]), (HEADLAND[1], X1)):
            out += row_segment(rng, idx, y, x0, x1, p)
            idx += 1
    # headland clutter: fruit bins and a tractor, leaving the track passable
    pl = Placer(rng, keepouts=[(0, 0, 3.0), (GOAL[0], GOAL[1], 3.0)])
    hx = (HEADLAND[0] + 0.8, HEADLAND[1] - 0.8)
    for k, (x, y, _) in enumerate(pl.scatter(p["bins"], hx, (rows[0], rows[-1]), radius=3.0)):
        out += model(f"bin{k}", box("b", x, y, 0.4, 1.2, 1.0, 0.8, jitter(BIN, rng)))
    ty = rng.choice(rows[1:-1])
    for x, y, _ in pl.scatter(1, (HEADLAND[0] + 2, HEADLAND[1] - 2), (ty - 1, ty + 1), radius=3.0):
        body = box("chassis", x, y, 1.0, 1.4, 3.0, 1.0, TRACTOR)
        body += box("cab", x, y - 0.4, 2.0, 1.2, 1.2, 1.2, (0.15, 0.18, 0.20))
        for k, (dx, dy, r) in enumerate([(0.85, -0.9, 0.75), (-0.85, -0.9, 0.75), (0.75, 1.1, 0.45), (-0.75, 1.1, 0.45)]):
            body += cylinder(f"wh{k}", x + dx, y + dy, r, r, 0.4, (0.06, 0.06, 0.06), rpy=(0, math.pi / 2, 0))
        out += model("tractor", body)
    return out + FOOTER


WORLD = World(
    name="orchard",
    description="fruit orchard: tree rows 5 m apart, trellis posts + thin wires, headland at x=-30, bins, tractor",
    goal=GOAL,
    build=build,
    params=dict(rows_per_side=6, tree_spacing=2.5, crown_radius=(0.7, 1.1), gap_share=0.08, trellis=True,
                fruit=True, bins=8),
)

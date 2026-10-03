"""Urban grid, ~210 x 140 m: city blocks with buildings of mixed height, streets with furniture.

Layout (origin = UAV spawn point, in the centre of an intersection):
  - streets every 36 m in x and y (8 m road + 3 m sidewalk on each side), dashed centre lines,
    zebra crossings.
  - each block is split into lots: buildings (some with a podium + set-back tower, dark window band
    on every floor), small parks, or open parking.
  - street furniture at flight altitude: street lights with arms over the road, street trees whose
    crowns overhang the road, traffic lights at the corners, parked cars along the kerb.
  - the goal is on another street, so the UAV has to follow streets and turn corners between
    buildings rather than fly straight.
"""
import math

from .. import World
from ..placement import Placer
from ..sdf import FOOTER, box, cylinder, header, jitter, model, sphere

STREETS_X = [-108.0, -72.0, -36.0, 0.0, 36.0]
STREETS_Y = [-36.0, 0.0, 36.0]
PITCH = 36.0
ROAD_HW = 4.0         # road half width
SIDEWALK = 3.0
GOAL = (-90.0, 36.0, 3.0, 0.0)

ASPHALT = (0.22, 0.22, 0.24)
PAVEMENT = (0.62, 0.61, 0.58)
GRASS = (0.30, 0.50, 0.22)
WHITE = (0.92, 0.92, 0.90)
GLASS = (0.12, 0.15, 0.20)
METAL = (0.30, 0.31, 0.33)
BARK = (0.33, 0.24, 0.16)
LEAF = (0.22, 0.42, 0.15)
FACADES = [(0.80, 0.76, 0.68), (0.70, 0.55, 0.45), (0.55, 0.57, 0.60), (0.85, 0.83, 0.80),
           (0.62, 0.42, 0.33), (0.45, 0.50, 0.55), (0.78, 0.70, 0.55)]
CAR_COLOURS = [(0.75, 0.10, 0.10), (0.10, 0.20, 0.55), (0.85, 0.85, 0.85), (0.08, 0.08, 0.08),
               (0.45, 0.45, 0.48), (0.10, 0.40, 0.20), (0.80, 0.65, 0.15)]


def lines(streets):
    """Block boundaries: the streets plus one extra (street-less) line on each side."""
    return [streets[0] - PITCH] + streets + [streets[-1] + PITCH]


def building(rng, name, x0, x1, y0, y1, p):
    cx, cy, sx, sy = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    colour = jitter(rng.choice(FACADES), rng, 0.03)
    parts = []  # (sx, sy, z0, h)
    if rng.random() < p["tower_share"] and min(sx, sy) > 12:
        podium = rng.uniform(7, 12)
        inset = rng.uniform(2, 4)
        parts.append((sx, sy, 0.0, podium))
        parts.append((sx - 2 * inset, sy - 2 * inset, podium, rng.uniform(20, p["max_height"])))
    else:
        parts.append((sx, sy, 0.0, rng.uniform(p["min_height"], p["max_height"] * 0.6)))
    body = ""
    for k, (bx, by, z0, h) in enumerate(parts):
        body += box(f"b{k}", cx, cy, z0 + h / 2, bx, by, h, colour)
        if p["windows"]:
            # one slightly larger dark box per floor reads as a ribbon of windows around the facade
            for f in range(int(h // 3.2)):
                z = z0 + 1.7 + f * 3.2
                if z + 0.6 < z0 + h:
                    body += box(f"w{k}_{f}", cx, cy, z, bx + 0.06, by + 0.06, 1.2, GLASS, collide=False)
        body += box(f"roof{k}", cx, cy, z0 + h + 0.4, bx, by, 0.8, jitter(colour, rng, 0.08))
    return model(name, body)


def street_tree(rng, name, x, y):
    h = rng.uniform(3.0, 4.0)
    crown = rng.uniform(1.5, 2.3)
    body = cylinder("trunk", x, y, h / 2, rng.uniform(0.1, 0.16), h, BARK)
    body += sphere("crown", x, y, h + crown * 0.6, crown, jitter(LEAF, rng, 0.06))
    return model(name, body)


def street_light(name, x, y, towards):
    """Pole on the sidewalk with an arm reaching `towards` (unit vector) over the road."""
    ux, uy = towards
    body = cylinder("pole", x, y, 3.5, 0.09, 7.0, METAL)
    body += box("arm", x + ux * 0.9, y + uy * 0.9, 6.9, 1.8 if ux else 0.1, 1.8 if uy else 0.1, 0.1, METAL)
    body += box("lamp", x + ux * 1.8, y + uy * 1.8, 6.8, 0.6, 0.6, 0.15, WHITE, emissive=(0.9, 0.85, 0.6))
    return model(name, body)


def traffic_light(name, x, y):
    body = cylinder("pole", x, y, 2.25, 0.07, 4.5, METAL)
    body += box("head", x, y, 4.3, 0.35, 0.35, 1.0, (0.1, 0.1, 0.1))
    return model(name, body)


def car(rng, name, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)

    def at(lx, ly):
        return x + lx * c - ly * s, y + lx * s + ly * c

    colour = rng.choice(CAR_COLOURS)
    body = box("body", x, y, 0.65, 4.4, 1.8, 0.7, colour, rpy=(0, 0, yaw))
    cx, cy = at(-0.2, 0)
    body += box("cabin", cx, cy, 1.27, 2.3, 1.6, 0.55, GLASS, rpy=(0, 0, yaw))
    for k, (lx, ly) in enumerate([(1.4, 0.8), (1.4, -0.8), (-1.4, 0.8), (-1.4, -0.8)]):
        wx, wy = at(lx, ly)
        body += cylinder(f"wh{k}", wx, wy, 0.33, 0.33, 0.22, (0.05, 0.05, 0.05), rpy=(math.pi / 2, 0, yaw),
                         collide=False)
    return model(name, body)


def streets(rng, p, pl):
    """Road markings, crossings, furniture and parked cars along every street."""
    xs, ys = lines(STREETS_X), lines(STREETS_Y)
    out = ""
    n = 0
    # (fixed coordinate, along-axis 'x' or 'y', span, crossing coordinates)
    specs = [(sx, "y", (ys[0], ys[-1]), STREETS_Y) for sx in STREETS_X] + \
            [(sy, "x", (xs[0], xs[-1]), STREETS_X) for sy in STREETS_Y]

    def xy(fixed, along, axis):
        return (fixed, along) if axis == "y" else (along, fixed)

    for fixed, axis, (a0, a1), crossings in specs:
        def near_crossing(a, margin):
            return any(abs(a - c) < ROAD_HW + margin for c in crossings)

        marks = ""
        a = a0 + 2
        while a < a1 - 2:
            if not near_crossing(a, 4):
                mx, my = xy(fixed, a + 1.5, axis)
                marks += box(f"m{n}", mx, my, 0.005, *((0.15, 3.0) if axis == "y" else (3.0, 0.15)), 0.01, WHITE,
                             collide=False)
                n += 1
            a += 6
        for c in crossings:  # zebra crossings on both sides of every intersection
            for side in (-1, 1):
                for k in range(-3, 4):
                    zx, zy = xy(fixed + k * 1.1, c + side * (ROAD_HW + 2.0), axis)
                    marks += box(f"m{n}", zx, zy, 0.005, *((0.5, 2.5) if axis == "y" else (2.5, 0.5)), 0.01,
                                 WHITE, collide=False)
                    n += 1
        out += model(f"markings_{axis}{int(fixed)}", marks)

        for side in (-1, 1):
            kerb = fixed + side * (ROAD_HW + 0.7)       # furniture line on the sidewalk
            park = fixed + side * (ROAD_HW - 1.1)       # parked-car line on the road
            towards = (-side, 0) if axis == "y" else (0, -side)
            a = a0 + rng.uniform(4, 10)
            while a < a1:
                if not near_crossing(a, 3):
                    x, y = xy(kerb, a, axis)
                    roll = rng.random()
                    if roll < p["tree_share"] and pl.try_place(x, y, 2.0):
                        out += street_tree(rng, f"stree{n}", x, y)
                    elif roll < p["tree_share"] + 0.3 and pl.try_place(x, y, 0.5):
                        out += street_light(f"slight{n}", x, y, towards)
                    n += 1
                a += rng.uniform(9, 14)
            a = a0 + rng.uniform(3, 8)
            while a < a1:
                if not near_crossing(a, 8) and rng.random() < p["parked_cars"]:
                    x, y = xy(park, a, axis)
                    if pl.try_place(x, y, 2.3):
                        out += car(rng, f"car{n}", x, y, (math.pi / 2 if axis == "y" else 0) +
                                   (math.pi if rng.random() < 0.5 else 0))
                        n += 1
                a += rng.uniform(5.5, 8.0)
    for sx in STREETS_X:
        for sy in STREETS_Y:
            for dx in (-1, 1):
                for dy in (-1, 1):
                    x, y = sx + dx * (ROAD_HW + 0.5), sy + dy * (ROAD_HW + 0.5)
                    if pl.try_place(x, y, 0.3):
                        out += traffic_light(f"tl{n}", x, y)
                        n += 1
    return out


def blocks(rng, p, pl):
    xs, ys = lines(STREETS_X), lines(STREETS_Y)
    out = ""
    n = 0
    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            # block footprint between road edges (an edge without a street is just open space)
            bx0 = xs[i] + (ROAD_HW if xs[i] in STREETS_X else 0)
            bx1 = xs[i + 1] - (ROAD_HW if xs[i + 1] in STREETS_X else 0)
            by0 = ys[j] + (ROAD_HW if ys[j] in STREETS_Y else 0)
            by1 = ys[j + 1] - (ROAD_HW if ys[j + 1] in STREETS_Y else 0)
            out += model(f"sidewalk{i}_{j}", box("s", (bx0 + bx1) / 2, (by0 + by1) / 2, 0.075,
                                                 bx1 - bx0, by1 - by0, 0.15, PAVEMENT))
            # lots: split the inner area (inside the sidewalk) into a 2x2 or 3x2 grid
            ix0, ix1, iy0, iy1 = bx0 + SIDEWALK, bx1 - SIDEWALK, by0 + SIDEWALK, by1 - SIDEWALK
            nx, ny = rng.choice([(2, 2), (3, 2), (2, 3), (1, 2), (2, 1)])
            cuts_x = [ix0] + sorted(rng.uniform(ix0 + 5, ix1 - 5) for _ in range(nx - 1)) + [ix1]
            cuts_y = [iy0] + sorted(rng.uniform(iy0 + 5, iy1 - 5) for _ in range(ny - 1)) + [iy1]
            for a in range(nx):
                for b in range(ny):
                    x0, x1, y0, y1 = cuts_x[a], cuts_x[a + 1], cuts_y[b], cuts_y[b + 1]
                    roll = rng.random()
                    if roll < p["park_share"]:
                        out += model(f"park{n}", box("g", (x0 + x1) / 2, (y0 + y1) / 2, 0.16, x1 - x0,
                                                     y1 - y0, 0.02, GRASS, collide=False))
                        for _k in range(rng.randint(2, 5)):
                            tx, ty = rng.uniform(x0 + 2, x1 - 2), rng.uniform(y0 + 2, y1 - 2)
                            if pl.try_place(tx, ty, 2.0):
                                out += street_tree(rng, f"ptree{n}_{_k}", tx, ty)
                    elif roll < p["park_share"] + 0.08:
                        for _k in range(rng.randint(2, 6)):
                            cx, cy = rng.uniform(x0 + 3, x1 - 3), rng.uniform(y0 + 2, y1 - 2)
                            if pl.try_place(cx, cy, 2.4):
                                out += car(rng, f"pcar{n}_{_k}", cx, cy, rng.choice([0, math.pi / 2]))
                    else:
                        gap = rng.uniform(0.0, 0.8)  # narrow alleys between neighbouring buildings
                        out += building(rng, f"bldg{n}", x0 + gap, x1 - gap, y0 + gap, y1 - gap, p)
                    n += 1
    return out


def build(rng, p):
    out = header("city", ground=ASPHALT, ground_size=(320, 220), background=(0.66, 0.76, 0.88))
    pl = Placer(rng, keepouts=[(0, 0, 3.0), (GOAL[0], GOAL[1], 3.0)])
    out += blocks(rng, p, pl)
    out += streets(rng, p, pl)
    return out + FOOTER


WORLD = World(
    name="city",
    description="urban grid: mixed-height buildings/towers, parks, street trees, lights, traffic lights, parked cars",
    goal=GOAL,
    build=build,
    params=dict(min_height=6.0, max_height=45.0, tower_share=0.25, park_share=0.12, tree_share=0.45,
                parked_cars=0.5, windows=True),
)

"""70 x 36 m warehouse (walls, no roof) full of pallet racks: a structured, indoor-like environment
(as opposed to the forests' scattered trunks).

Layout (x forward, y left, origin = UAV spawn point, in the middle of the y=0 main aisle):
  - 8 rows of racks parallel to x at y = +-2.1, +-6.3, +-10.5, +-14.7 (1.2 m deep, 4.5 m tall),
    leaving 3 m wide aisles between rows (main aisle at y in [-1.5, 1.5]).
  - each row is cut by 3 m wide cross-aisles at x = -16.5, 0, 16.5, giving 4 rack segments per row.
  - racks are open shelving (uprights + 4 shelf planks + random cartons on the planks), so the
    lidar sees through/around them like real racking rather than off a solid wall.
  - a few columns in the cross-aisles; loose pallets with box stacks in the aisles only with
    PALLETS = True (off: the aisles hold only the red crates), keeping >= 1.6 m of free width.
  - the main aisle (y = 0) gets small red crates instead of pallets (>= 1.9 m free width): these are
    *temporary* obstacles (model name temp_obstacle*), left out of the prior map used by the planners.
"""
from ..sdf import FOOTER, box, header, model
from .. import World

X_MIN, X_MAX = -35.0, 35.0
Y_MIN, Y_MAX = -18.0, 18.0
WALL_H, WALL_T = 6.0, 0.3

RACK_DEPTH, RACK_H = 1.2, 4.5
ROW_YS = [-14.7, -10.5, -6.3, -2.1, 2.1, 6.3, 10.5, 14.7]
CROSS_AISLES = [-16.5, 0.0, 16.5]  # x centers
CROSS_W = 3.0
SHELF_Z = [0.15, 1.35, 2.55, 3.75]
BAY = 2.7  # upright spacing

BLUE = (0.10, 0.25, 0.65)
ORANGE = (0.90, 0.45, 0.10)
CARTON = [(0.65, 0.48, 0.28), (0.72, 0.55, 0.33), (0.58, 0.42, 0.24)]
WOOD = (0.55, 0.40, 0.22)
CONCRETE = (0.62, 0.62, 0.64)
TEMP_RED = (0.85, 0.10, 0.08)

# pallets with carton stacks in the side aisles: off (the aisles hold only the red crates). Their
# random draws are still made, so the racks / crates / columns keep their layout either way.
PALLETS = False

# temporary obstacles (main aisle only): not part of the prior map, see temp_obstacle()
TEMP_PREFIX = "temp_obstacle"
TMP_SIZE = (0.45, 0.45, 0.7)  # [m] tall enough for the UGV's stereo camera / rbl band (z 0.2..1.0)
TMP_Y = 0.75                  # [m] crate centre off the centreline: occupies |y| 0.525..0.975


def segments():
    """Rack segments along x, split by the cross-aisles: list of (x_start, x_end)."""
    edges = [X_MIN + 2.0] + [v for c in CROSS_AISLES for v in (c - CROSS_W / 2, c + CROSS_W / 2)] + [X_MAX - 2.0]
    return [(edges[i], edges[i + 1]) for i in range(0, len(edges), 2)]


def rack(rng, idx, x0, x1, y):
    length = x1 - x0
    cx = (x0 + x1) / 2
    nbays = max(1, round(length / BAY))
    bay = length / nbays
    body = ""
    # uprights: a thin frame plane across the rack depth at every bay boundary
    for i in range(nbays + 1):
        body += box(f"up{i}", x0 + i * bay, y, RACK_H / 2, 0.08, RACK_DEPTH, RACK_H, BLUE)
    # shelf planks + horizontal beam at the top
    for k, z in enumerate(SHELF_Z):
        body += box(f"sh{k}", cx, y, z, length, RACK_DEPTH, 0.05, ORANGE)
    # cartons on the shelves
    n = 0
    for i in range(nbays):
        for k, z in enumerate(SHELF_Z):
            if rng.random() < 0.35:
                continue
            for _ in range(rng.choice([1, 2])):
                sx = rng.uniform(0.5, 1.1)
                sy = rng.uniform(0.6, RACK_DEPTH - 0.1)
                sz = rng.uniform(0.4, 0.9)
                px = x0 + (i + 0.5) * bay + rng.uniform(-(bay - sx) / 2 + 0.1, (bay - sx) / 2 - 0.1)
                pz = z + 0.025 + sz / 2
                body += box(f"c{n}", px, y + rng.uniform(-0.05, 0.05), pz, sx, sy, sz, rng.choice(CARTON))
                n += 1
    return model(f"rack{idx}", body)


def temp_obstacle(i, x, y):
    """Small red crate, standing off the aisle centreline: the UGV (0.45 m encumbrance) passes it
    with ~2 m of free width on the other side. Its model name starts with TEMP_PREFIX, so the prior
    map (rbl_mission loadWorldObstacles: global / coverage planners) ignores it -- only the onboard
    sensors see it -- and mission_viz draws it in red."""
    return model(f"{TEMP_PREFIX}{i}", box("crate", x, y, TMP_SIZE[2] / 2, *TMP_SIZE, TEMP_RED))


def clutter(rng):
    """Pallets with box stacks and columns in the aisles. Aisle centerlines are y = 0, +-4.2, +-8.4,
    +-12.6, +-16.65 (outer strip); items stay narrow enough to keep >= 1.6 m of free width."""
    out = ""
    aisle_ys = [0.0, 4.2, -4.2, 8.4, -8.4, 12.6, -12.6]
    placed = []
    n = n_tmp = 0
    for ay in aisle_ys:
        for _ in range(rng.randint(2, 3)):
            for _try in range(50):
                x = rng.uniform(X_MIN + 4, X_MAX - 4)
                y = ay + rng.uniform(-0.5, 0.5)
                # keep spawn area clear, and don't sit in a cross-aisle centre (that's where the
                # columns go, and it would block the turn)
                if abs(x) < 3.0 and abs(y) < 3.0:
                    continue
                if any(abs(x - c) < CROSS_W / 2 + 0.6 for c in CROSS_AISLES):
                    continue
                if any((x - px) ** 2 + (y - py) ** 2 < 4.0 for px, py in placed):
                    continue
                placed.append((x, y))
                break
            else:
                continue
            body = box("pallet", x, y, 0.075, 1.2, 0.8, 0.15, WOOD)
            z = 0.15
            for j in range(rng.randint(1, 3)):
                h = rng.uniform(0.35, 0.6)
                body += box(f"b{j}", x, y, z + h / 2, rng.uniform(0.9, 1.1), rng.uniform(0.6, 0.75), h,
                            rng.choice(CARTON))
                z += h
            if ay == 0.0:
                # main aisle: a small temporary obstacle instead of the pallet (the pallet's random
                # draws above are still made, so every other aisle keeps its layout)
                out += temp_obstacle(n_tmp, x, ay + (TMP_Y if y >= ay else -TMP_Y))
                n_tmp += 1
                continue
            if PALLETS:
                out += model(f"pallet{n}", body)
            n += 1
    # columns in the cross-aisles, off-centre so a 3 m cross-aisle keeps >= 2 m of free width
    for i, cxa in enumerate(CROSS_AISLES):
        for j, cy in enumerate([-8.4, 4.2, 12.6]):
            if abs(cxa) < 1e-6 and abs(cy) < 4.3 and cy != 4.2:
                continue
            out += model(f"column{i}_{j}", box("col", cxa + 0.8, cy, WALL_H / 2, 0.5, 0.5, WALL_H, CONCRETE))
    out += extra_temp_obstacles(aisle_ys, placed, n_tmp)
    return out


EXTRA_TMP_PER_AISLE = 3   # extra red crates in every aisle (on top of the main aisle's pallet swaps)
EXTRA_TMP_SEED = 4242     # own generator: the racks / pallets / columns keep their layout


def extra_temp_obstacles(aisle_ys, placed, n_tmp):
    """More temporary red crates (see temp_obstacle), EXTRA_TMP_PER_AISLE per aisle, alternating
    sides of the centreline. Kept >= 2.5 m (x) from the pallets and from each other, out of the
    cross-aisles and the spawn area, and >= 2 m from the segment midpoints (deploy.sh's UGV stops: the
    planners do not know these crates, so a stop must not land on one)."""
    import random
    rng = random.Random(EXTRA_TMP_SEED)
    mids = [(x0 + x1) / 2 for x0, x1 in segments()]
    taken = list(placed)
    out = ""
    for ay in aisle_ys:
        for k in range(EXTRA_TMP_PER_AISLE):
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
                y = ay + (TMP_Y if (k + int(ay)) % 2 == 0 else -TMP_Y)
                taken.append((x, y))
                out += temp_obstacle(n_tmp, x, y)
                n_tmp += 1
                break
    return out


def walls():
    xm, ym = (X_MIN + X_MAX) / 2, (Y_MIN + Y_MAX) / 2
    lx, ly = X_MAX - X_MIN + WALL_T, Y_MAX - Y_MIN + WALL_T
    body = box("n", xm, Y_MAX + WALL_T / 2, WALL_H / 2, lx, WALL_T, WALL_H, CONCRETE)
    body += box("s", xm, Y_MIN - WALL_T / 2, WALL_H / 2, lx, WALL_T, WALL_H, CONCRETE)
    body += box("e", X_MAX + WALL_T / 2, ym, WALL_H / 2, WALL_T, ly, WALL_H, CONCRETE)
    body += box("w", X_MIN - WALL_T / 2, ym, WALL_H / 2, WALL_T, ly, WALL_H, CONCRETE)
    return model("walls", body)


def build(rng, params):
    out = header("warehouse") + walls()
    i = 0
    for y in ROW_YS:
        for x0, x1 in segments():
            out += rack(rng, i, x0, x1, y)
            i += 1
    out += clutter(rng)
    return out + FOOTER


# Spawn is in the y=0 main aisle between rack rows, in the x=0 cross-aisle. The goal sits in the
# y=8.4 aisle near the far -x end, so the UAV has to leave the main aisle through a cross-aisle
# (x=-16.5), turn into a side aisle, and pass shelves/pallets/columns on the way.
WORLD = World(
    name="warehouse",
    description="70 x 36 m warehouse: pallet-rack rows, aisles, cross-aisles, columns, pallets",
    goal=(-28, 8.4, 2.5, 0.0),
    build=build,
)

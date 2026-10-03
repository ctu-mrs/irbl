"""Natural mixed forest, 80 x 46 m: a harder, more realistic cousin of cylinder_forest.

  - mixed species: conifers (straight trunk, tiered foliage starting 2-7 m up), deciduous trees
    (often leaning trunk, clustered crown), birches (thin, pale), dead snags (bare, leaning) and
    young saplings whose foliage sits right at flight altitude.
  - trunk radius / height vary per tree (no two trees alike, unlike the uniform cylinders).
  - understorey: bushes, fallen logs and rocks, so the lidar also sees low clutter.
  - spawn at the origin, goal at the far -x end; a 3 m disc around both is kept clear.
"""
import math

from .. import World
from ..placement import Placer, area
from ..sdf import FOOTER, cylinder, header, jitter, leaning_cylinder, lying_cylinder, model, sphere

X = (-68.0, 12.0)
Y = (-23.0, 23.0)
GOAL = (-58.0, 0.0, 2.5, 0.0)

BARK = (0.33, 0.24, 0.16)
BIRCH = (0.85, 0.83, 0.78)
DEAD = (0.42, 0.38, 0.33)
CONIFER = (0.10, 0.28, 0.14)
LEAF = (0.22, 0.42, 0.15)
BUSH = (0.20, 0.36, 0.14)
ROCK = (0.45, 0.44, 0.42)


def conifer(rng, i, x, y, r):
    h = rng.uniform(12, 22)
    body = cylinder("trunk", x, y, h / 2, r, h, jitter(BARK, rng))
    # foliage tiers: wide at the bottom, narrowing towards the top
    base = rng.uniform(2.0, 7.0)
    tiers = rng.randint(4, 6)
    tier_h = (h - base) / tiers
    bottom = rng.uniform(1.4, 2.6)
    for k in range(tiers):
        rad = bottom * (1 - k / tiers) + 0.3
        body += cylinder(f"t{k}", x, y, base + (k + 0.5) * tier_h, rad, tier_h * 0.8, jitter(CONIFER, rng, 0.03))
    return model(f"conifer{i}", body)


def deciduous(rng, i, x, y, r, young=False):
    h = rng.uniform(3, 5) if young else rng.uniform(6, 11)
    tilt = rng.uniform(0, 0.15)
    xml, top = leaning_cylinder("trunk", x, y, 0, r, h, tilt, rng.uniform(0, 2 * math.pi), jitter(BARK, rng))
    body = xml
    # crown: a cluster of overlapping spheres around the trunk tip
    crown = rng.uniform(1.0, 1.6) if young else rng.uniform(2.0, 3.5)
    for k in range(rng.randint(3, 5)):
        a = rng.uniform(0, 2 * math.pi)
        d = rng.uniform(0, crown * 0.6)
        body += sphere(f"c{k}", top[0] + d * math.cos(a), top[1] + d * math.sin(a),
                       top[2] + rng.uniform(-0.3, 0.6) * crown, crown * rng.uniform(0.6, 1.0),
                       jitter(LEAF, rng, 0.06))
    return model(f"{'sapling' if young else 'tree'}{i}", body)


def birch(rng, i, x, y, r):
    h = rng.uniform(8, 14)
    xml, top = leaning_cylinder("trunk", x, y, 0, r, h, rng.uniform(0, 0.1), rng.uniform(0, 2 * math.pi), BIRCH)
    body = xml
    for k in range(3):
        body += sphere(f"c{k}", top[0] + rng.uniform(-0.8, 0.8), top[1] + rng.uniform(-0.8, 0.8),
                       top[2] - k * 1.5, rng.uniform(1.0, 1.6), jitter(LEAF, rng, 0.08))
    return model(f"birch{i}", body)


def snag(rng, i, x, y, r):
    h = rng.uniform(4, 10)
    xml, _ = leaning_cylinder("trunk", x, y, 0, r, h, rng.uniform(0.05, 0.35), rng.uniform(0, 2 * math.pi),
                              jitter(DEAD, rng))
    return model(f"snag{i}", xml)


def build(rng, p):
    out = header("forest", ground=(0.30, 0.27, 0.18), ground_size=(250, 150), background=(0.62, 0.74, 0.86))
    pl = Placer(rng, keepouts=[(0, 0, 3.0), (GOAL[0], GOAL[1], 3.0)])
    n_trees = int(p["density"] * area(X, Y))
    kinds = [conifer, deciduous, birch, snag]
    weights = [p["conifer_share"], 1 - p["conifer_share"] - 0.2, 0.12, 0.08]
    # spacing radius > trunk radius: real trunks rarely stand closer than ~1.5 m
    for i, (x, y, _) in enumerate(pl.scatter(n_trees, X, Y, radius=lambda g: g.uniform(0.8, 1.6))):
        kind = rng.choices(kinds, weights)[0]
        r = rng.uniform(0.08, 0.14) if kind is birch else min(0.6, rng.lognormvariate(math.log(0.22), 0.35))
        out += kind(rng, i, x, y, r)
    for i, (x, y, _) in enumerate(pl.scatter(int(p["saplings"]), X, Y, radius=1.0)):
        out += deciduous(rng, i, x, y, rng.uniform(0.04, 0.08), young=True)
    for i, (x, y, r) in enumerate(pl.scatter(int(p["bushes"]), X, Y, radius=lambda g: g.uniform(0.4, 1.0))):
        out += model(f"bush{i}", sphere("b", x, y, r * 0.5, r, jitter(BUSH, rng, 0.06)))
    for i, (x, y, _) in enumerate(pl.scatter(int(p["logs"]), X, Y, radius=3.0)):
        r = rng.uniform(0.15, 0.35)
        out += model(f"log{i}", lying_cylinder("l", x, y, r, rng.uniform(3, 7), rng.uniform(0, math.pi),
                                               jitter(DEAD, rng)))
    for i, (x, y, r) in enumerate(pl.scatter(int(p["rocks"]), X, Y, radius=lambda g: g.uniform(0.3, 1.1))):
        out += model(f"rock{i}", sphere("r", x, y, r * 0.2, r, jitter(ROCK, rng)))
    return out + FOOTER


WORLD = World(
    name="forest",
    description="80 x 46 m mixed forest: conifers, leaning deciduous, birches, snags, saplings, bushes, logs, rocks",
    goal=GOAL,
    build=build,
    params=dict(density=0.07, conifer_share=0.45, saplings=40, bushes=90, logs=20, rocks=40),
)

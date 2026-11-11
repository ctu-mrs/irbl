#!/usr/bin/env python3
"""Random spawn / goal positions for the px4_sim_10_uav session.

  ./gen_positions.py spawn   -> config/spawn.env  (UAV<i>_SPAWN_X/Y/Z, read by session.yml)
  ./gen_positions.py goals   -> config/goals.env  (UAV<i>_GOAL_X/Y/Z/YAW, read by activation.sh)

All coordinates are in the shared control frame uav1/world_origin, i.e. relative to uav1's spawn,
so uav1 always spawns at the origin. Positions keep MIN_SEP apart from each other; goals also keep
MIN_TRAVEL away from the same UAV's spawn so every vehicle has to actually cross the group.
"""
import math
import os
import random
import sys

N = 20
HALF_SIZE = 17.0   # [m] positions sampled in [-HALF_SIZE, HALF_SIZE]^2 around uav1's spawn
MIN_SEP = 3.0      # [m] between any two spawns / any two goals
MIN_TRAVEL = 10.0  # [m] between a UAV's spawn and its goal
GOAL_Z = 2.5
HERE = os.path.dirname(os.path.abspath(__file__))


def sample(fixed_first=None, avoid=None):
    for _ in range(10000):
        pts = [fixed_first] if fixed_first else []
        ok = True
        while len(pts) < N and ok:
            for _ in range(1000):
                p = (random.uniform(-HALF_SIZE, HALF_SIZE), random.uniform(-HALF_SIZE, HALF_SIZE))
                if all(math.dist(p, q) >= MIN_SEP for q in pts) and (
                        avoid is None or math.dist(p, avoid[len(pts)]) >= MIN_TRAVEL):
                    pts.append(p)
                    break
            else:
                ok = False
        if ok:
            return pts
    sys.exit("could not place positions, relax HALF_SIZE/MIN_SEP/MIN_TRAVEL")


def read_spawns():
    env = {}
    with open(os.path.join(HERE, "config", "spawn.env")) as f:
        for stmt in f.read().replace("\n", ";").split(";"):
            if "=" in stmt:
                k, v = stmt.replace("export", "").strip().split("=")
                env[k] = float(v)
    return [(env[f"UAV{i}_SPAWN_X"], env[f"UAV{i}_SPAWN_Y"]) for i in range(1, N + 1)]


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else ""
    if what == "spawn":
        pts = sample(fixed_first=(0.0, 0.0))
        lines = [f"export UAV{i}_SPAWN_X={x:.2f}; export UAV{i}_SPAWN_Y={y:.2f}; export UAV{i}_SPAWN_Z=0.0"
                 for i, (x, y) in enumerate(pts, 1)]
        out = "spawn.env"
    elif what == "goals":
        pts = sample(avoid=read_spawns())
        lines = [f"export UAV{i}_GOAL_X={x:.2f}; export UAV{i}_GOAL_Y={y:.2f}; "
                 f"export UAV{i}_GOAL_Z={GOAL_Z}; export UAV{i}_GOAL_YAW=0.0"
                 for i, (x, y) in enumerate(pts, 1)]
        out = "goals.env"
    else:
        sys.exit("usage: gen_positions.py spawn|goals")
    with open(os.path.join(HERE, "config", out), "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"config/{out}:\n" + "\n".join(lines))


if __name__ == "__main__":
    main()

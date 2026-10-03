# Worlds

Procedurally generated Gazebo worlds for `px4_sim_1_uav_3dlidar`. Each world is one Python file in
`worldgen/worlds/`. `generate.py` turns it into `<name>.sdf` and records its rbl goal in
`world_goals.yaml`. On start, the session links every `*.sdf` here into PX4.

| world       | what it tests |
|-------------|---------------|
| `forest`    | mixed natural forest: varied trunks, leaning trees, conifer foliage and saplings at flight altitude, bushes, logs, rocks |
| `city`      | street grid: buildings and towers, corners, street trees overhanging the road, lights, traffic lights, parked cars |
| `orchard`   | narrow aisles between tree rows, thin trellis wires, a single headland to change aisle |
| `warehouse` | pallet-rack aisles, cross-aisles, columns, pallets |

```bash
./generate.py --list                          # worlds, goals and tunable params
./generate.py --all                           # regenerate everything
./generate.py forest --seed 7                 # another random layout (goal stays valid)
./generate.py city --set max_height=80 windows=False
./generate.py forest --name forest_dense --set density=0.11   # a variant next to the default
cd .. && ./start.sh forest                    # then ./activation.sh once 'Arming' shows up
```

## Adding a world

Create `worldgen/worlds/<name>.py`. It is discovered automatically:

```python
from .. import World
from ..placement import Placer
from ..sdf import FOOTER, box, cylinder, header, model

GOAL = (-40.0, 0.0, 2.5, 0.0)

def build(rng, p):
    out = header("<name>", ground=(0.4, 0.4, 0.3))         # <name> must equal WORLD.name
    pl = Placer(rng, keepouts=[(0, 0, 3.0), (GOAL[0], GOAL[1], 3.0)])  # spawn + goal stay clear
    for i, (x, y, r) in enumerate(pl.scatter(p["n"], (-50, 10), (-20, 20), radius=1.0)):
        out += model(f"pillar{i}", cylinder("c", x, y, 2.5, 0.4, 5.0, (0.5, 0.5, 0.5)))
    return out + FOOTER

WORLD = World(name="<name>", description="...", goal=GOAL, build=build, params=dict(n=100))
```

Then run `./generate.py <name>` and `./start.sh <name>`.

Rules of thumb:
- **Spawn at the origin, keep it clear.** The UAV always spawns at (0, 0) unless
  `PX4_GZ_MODEL_POSE` is set in `config/custom_config.yaml`. Keep at least 3 m clear around the
  spawn point and the goal. `Placer` does this for anything placed through it.
- **Use primitives (box / cylinder / sphere), static models.** The gpu_lidar sees the visuals and the
  UAV hits the collisions, so keep them identical. Use `collide=False` only for flat cosmetics
  (markings, window bands, fruit).
- **Watch the size.** A few thousand shapes is fine. Many more slows the lidar and pushes
  real-time factor below 1.
- **Fixed seeds.** Each world has a default seed, so the committed `.sdf` files are reproducible:
  rerunning `./generate.py --all` produces identical files.

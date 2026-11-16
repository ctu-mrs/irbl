# px4_sim_ugv_uav_greenhouse_dynamic

The UGV + UAV pair of the warehouse sessions, inspecting the crop rows of a greenhouse that **moves**, and counting
the fruits (see Fruit counting below).

**Vehicles and stack = the warehouse sessions'.** `session_gen.py` (called by `generate.py`) runs the generator of
`../px4_sim_5ugv_5uav_warehouse_light` for 1 pair, as `../px4_sim_ugv_uav_warehouse_dynamic` does:
- **Vehicles:** UGV `ugv1_carrier_gh` and UAV `uav1_x500_crop`, the pair-1 models of
  `../px4_sim_2ugv_2uav_warehouse_stereo_larger`. Both have a forward **stereo depth camera** (160×120, 10 Hz, as in
  the stereo sessions) for rbl_controller. The UAV also has a down camera (640×480, 20 Hz), and the UGV deck carries
  the **ArUco board** (5 markers) for PX4 precision landing. On top of that the UAV gets the crop cameras.
- **Configs (`config/`):** rbl, PX4 and sensor configs are links to the warehouse ones. The exception is
  `uav_rbl.yaml`, a copy of the warehouse UAV config with `z_max: 2.6` instead of 10 m, so rbl never routes over a
  crop row (plants up to 2.8 m, crop wires at 3.0 m).
- **Session:** `session.yml`, `env.sh`, `ugv_uav_pairs.rviz`, `deploy.sh`, `kill.sh` and the PX4 airframes come from
  the light generator. On top: this world, `config/greenhouse_map.yaml`, the wind plugin and the `scan` window.
- **Names:** ROS domain 47, tmux socket `ghdynugvuav`, headless by default (`HEADLESS=0 ./start.sh` for the gz GUI).

What moves:

- **Wind-blown crops.** The gz system plugin `gz_greenhouse_dynamics::PlantSway`
  (`ros_packages/gz_greenhouse_dynamics`) animates the plant visuals. Gust fronts sweep down the greenhouse,
  every ~2 m plant group sways about its stem base at its own frequency, and the leaves flutter.
  Strawberries only flutter. With the defaults the top of a 2.5 m tomato moves by up to ~8–10 cm.
  The wind settings live in `config/gz_server.config`.
- **Operators.** Three gz actors (walking / standing person) loop through the aisles (`OPERATORS` in `generate.py`):
  - `operator_1` circles the block between the main aisle and aisle 5 in segment 3, and crosses 0.8 m in front of the
    UGV's spawn point.
  - `operator_2` harvests along aisle 2 (y = −7.6).
  - `operator_3` walks the far end of aisle 6 and the x = −14 cross-aisle.

  They stop now and then, facing a crop row.

Both are **visual only**. The depth and crop cameras see them; physics, collisions and the planners' prior map don't.
The mission parses only the `<model>` collision boxes, which are the static ones of `greenhouse.py`. Like the blue
crates, rbl_controller has to deal with the operators and the moving foliage from its depth cloud. Actors have no
collision, so a vehicle that hits one passes through it.

```bash
colcon build --packages-select gz_greenhouse_dynamics rbl_mission   # once, from the workspace root
./start.sh                       # `deploy` window: arrow-up + Enter = tomato row 4 2 left (history: pepper 5 2, cucumber 6 2)
./deploy.sh row 5 column 3 left  # meets operator_1
./kill.sh
```

## Fruit counting (crop_scanner)

The UAV counts the fruits of every row segment it covers, by crop and ripeness, and writes a report when the segment is
done. It's the greenhouse version of the warehouse session's shelf scanner (`../px4_sim_ugv_uav_warehouse_dynamic`).

```bash
colcon build --packages-select gz_greenhouse_dynamics rbl_mission
./start.sh
./deploy.sh row 4 column 2 left row 5 column 2 left row 6 column 2 left row 7 column 2 left   # tomato, pepper, cucumber, strawberry
# `scan` window: REPORT A4-S2-L tomato: ripe 145/170, ... -> scans/<run>/uav1_A4-S2-L_<time>/report.html
```

| crop (rows from the main aisle outwards) | classes counted |
|---|---|
| tomato (rows at y = ±1.9) | ripe (red), turning (orange), unripe (green) |
| pepper (±5.7) | ripe (red, orange, yellow), unripe (green) |
| cucumber (±9.5) | cucumbers |
| strawberry (±13.3, raised gutters) | ripe (red), unripe (pale) |

- **Cameras:** the UAV (`models/uav1_x500_crop`, generated) carries left / right **colour + depth** cameras: 1280×960, 80° FOV, portrait, 5 Hz, same pose. gz Harmonic's
  `rgbd_camera` renders its colour image unlit, so it's two sensors. Like the shelf scanner, it reads them straight
  from gz and only the side facing the row, only while scanning.
- **Detection** (`rbl_mission/scripts/crop_detector.py`): colour segmentation in HSV per crop and ripeness class,
  calibrated on renders of this world. Each blob gets the median depth of its pixels; touching fruits (tomato trusses)
  are split by the area one fruit covers at that depth. Each detection is placed in the world with TF and kept if it's
  in the scanned row.
  In `greenhouse.py` the green fruits had the leaves' colour (cucumber and green pepper hue = leaf hue), so this world
  gives them their own realistic greens: cucumber bluish dark green, green pepper deep green, unripe tomato yellowish
  green (`MATERIAL_OVERRIDES` in `generate.py`). Shading only changes brightness here, so hue and saturation identify
  a fruit.
- **Counting:** per cell of 0.5 m along the row × 0.3 m height and per ripeness, the 70th percentile of the per-frame
  counts over the frames that saw the cell well inside the view. A fruit seen in 20 frames counts once, and a fruit
  swaying into the next cell for a moment isn't counted twice.
- **Ground truth:** `worlds/crop_truth.json`, written by `generate.py` while it builds the plants. It has every fruit
  (5891) with its rest position, size, ripeness, and the aisle sides it is visible from: ray-cast through leaves,
  stems, other fruits, the gutters, slabs and gutter legs from the coverage lane at the pass heights. The report
  scores against the fruits visible from the scanned side. 5119 of the 5891 fruits are visible from some aisle. Most of
  the rest hang behind a gutter: on the outer strawberry rows, half the berries hang on the wall side, and no aisle
  sees them.
- **Coverage:** two passes at 1.2 / 1.9 m (`config/greenhouse_map.yaml`, generated). Here the UAV's rbl keeps it
  between about 1.1 and 2.0 m, and the planned 0.9 / 2.2 m goals were never reached (each timed out after about 40 s).
  At 1.6 m from the row the portrait camera sees ±1.3 m of the plants, so two passes cover them. The height tolerance
  `mission.tolerance_z: 0.15` is the same as in the warehouse session.

Output in `scans/<start time>/` (git-ignored):

| file | content |
|---|---|
| `uav1_A<a>-S<s>-<L\|R>_<time>/report.html` | counts per ripeness found / expected, total, recall and precision per 1 m section and per cell, height × section grid, annotated frames |
| `.../report.md`, `.../report.json` | the same as text / data (JSON: every cell, ids of the missed fruits) |
| `.../detections.csv` | every detection of every frame: ripeness, colour, world position, camera position |
| `inventory.csv`, `index.md` | one line per cell × ripeness / per row segment, all segments of the run |

### Test with the warehouse vehicles (2026-11-15, `./deploy.sh row 4 column 2 left`)

Full mission (drive, takeoff, coverage 8/8 with no timeouts, rendezvous, precision landing on the ArUco board
4.9 cm off centre, home). Tomato: 283 counted / 259 visible; per 1 m section, recall 97% and precision 89%. Camera at
1.15–1.88 m.

### Test with the previous lidar UAV (2026-11-15, one deployment over all four crops in segment 2, headless)

`./deploy.sh row 4 column 2 left row 5 column 2 left row 6 column 2 left row 7 column 2 left`; counts with the
70th percentile (`count_percentile`, chosen by replaying the flights' `detections.csv` at 50–80):

| row | crop | counted / visible | recall / precision per 1 m section | per cell recall |
|---|---|---|---|---|
| A4-S2-L | tomato | 287 / 259 | 94% / 85% | 82% |
| A5-S2-L | pepper | 134 / 131 | 87% / 85% | 73% |
| A6-S2-L | cucumber | 79 / 75 | 93% / 89% | 79% |
| A7-S2-L | strawberry | 62 / 68 | 88% / 97% | 88% |

Detections are accurate: on the tomato row, 99.99% of the counted detections were within 25 cm of a real fruit, with a
mean offset of 4 cm and a spread of 4 cm. Ripeness was right whenever a detection was near a fruit. Most of the
remaining error is in counting touching fruits (trusses) and in fruits swaying between frames. A lower percentile
under-counts and a higher one over-counts. Tracking individual fruits across frames was tried and was worse,
because the wind moves them more than they are apart.

In rviz: **crop scan (lateral camera)** shows the annotated frame, upright. **fruits counted** shows a sphere per
counted fruit, coloured like the fruit; after the report the missed ones appear grey at their true position.

## Earlier tests with the lidar vehicles (2026-11-15, `./deploy.sh row 5 column 3 left`)

Before this session moved to the warehouse vehicles it used `../px4_sim_ugv_uav_greenhouse`'s (3D lidar UAV, single
ArUco marker):

| world | result |
|---|---|
| static greenhouse | DONE: coverage 12/12, precision landing, home |
| dynamic, wind only (`./generate.py --no-operators`) | DONE: coverage 12/12, landed 1.9 cm off the deck centre, home |
| dynamic, wind + operators | **FAILED in RENDEZVOUS**: `operator_1` walks through the pickup point (aisle 5, x ≈ 11.5) under the hovering UAV. The UAV stayed 0.1–0.3 m off the deck and the mission timed out. |

A person below the landing spot makes the mission give up instead of waiting for the spot to clear. That gap is still
open in ugv_uav_mission_node.

## Files

| | |
|---|---|
| `generate.py` | world (`worlds/greenhouse_dynamic.sdf`, crop meshes, operators), `worlds/crop_truth.json`, `config/greenhouse_map.yaml` (levels, tolerances) |
| `session_gen.py` | vehicles (`models/`), `config/uav_rbl.yaml`, `config/uav_sensors.yaml`, config links, `session.yml`, `env.sh`, `ugv_uav_pairs.rviz`, `deploy.sh`, `kill.sh`, `start.sh`, `set_px4_params.sh` |
| `config/gz_server.config` | gz systems + PlantSway (wind settings), hand-written |

Edit those and re-run `./generate.py`, not the outputs. Only one warehouse / greenhouse session can run at a time,
because they share the gz / PX4 / MicroXRCEAgent ports.

## World generation

`generate.py` writes `worlds/greenhouse_dynamic.sdf`, `worlds/models/greenhouse_crops_dynamic/` (~850 OBJs) and the
model files of `worlds/models/greenhouse_operator/`. Edit `generate.py` (CHUNK, LEAF_BANDS, OPERATORS,
WALK_SPEED) and re-run it, not its outputs. The row layout and plant models are imported from
`../px4_sim_1_uav_3dlidar/worlds/worldgen/worlds/greenhouse.py`.

The operator's skin and animations (`worlds/models/greenhouse_operator/meshes/{walk,stand}.dae`) are copied from the
`actor` model by Mingfei on fuel.gazebosim.org.

## Performance

Measured with the earlier lidar vehicles, as sim time / real time over 20 s, headless, both vehicles idle: the
static greenhouse session runs at RTF ≈ 0.73 on this machine and this one at RTF ≈ 0.58 (≈ 20 % slower). Both show the same periodic sensor-render stalls.
The extra cost is the number of visuals (~850 instead of 32 plant meshes). Larger `CHUNK` and fewer
`LEAF_BANDS` mean fewer visuals and less motion detail. `update_rate` in `config/gz_server.config` sets how
often the poses change.

## Next: Gaussian-splat cameras (not done yet)

This session is the "dynamics" half of a hybrid gz + 3D Gaussian splatting setup:

1. Capture a real greenhouse (2–3 min walkthrough video), then run COLMAP/GLOMAP and train with gsplat or splatfacto.
2. Align the splat to the gz world frame, e.g. ICP between a mesh extracted from the splat and these collision boxes, or
   rebuild the collision boxes from the splat.
3. Add a ROS 2 node that renders the splat at each camera pose (RGB + depth), adds the same wind field to the
   Gaussians in the crop regions, and merges in the gz render of the dynamic objects (operators, vehicles) by depth.
4. Depth on the splat: ray-cast the Gaussians (3DGUT-style) with the wind applied, for the depth cameras.

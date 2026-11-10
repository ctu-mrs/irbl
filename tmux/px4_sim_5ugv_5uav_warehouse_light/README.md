# px4_sim_5ugv_5uav_warehouse_light

Five UGV + UAV pairs inspect warehouse shelves at the same time. Each pair is a copy of the pairs in
`../px4_sim_2ugv_2uav_warehouse_stereo_larger`: rbl_controller on both vehicles, the UAV carried on the
UGV deck, PX4 precision landing on the UGV's ArUco board, and one mission node per pair. The pairs
don't coordinate with each other.

```bash
./start.sh            # headless gz (HEADLESS=0 ./start.sh for the gz GUI), rviz (WEB_VIZ=1: foxglove instead)
# wait for 'Arming' in the px4_bridge window, then in the `mission` window press arrow-up:
#   the newest history line deploys all 5 pairs; the older lines deploy one pair each
./deploy.sh pair 3 row 6 column 3 right
./deploy.sh abort     # every pair (./deploy.sh pair 2 abort: just one)
./kill.sh             # kills the whole process tree of every pane
```

| pair | UGV (PX4 inst.) | UAV (PX4 inst.) | spawn | ArUco ids | deploy in history |
|---|---|---|---|---|---|
| 1 | ugv1 (0) | uav1 (1) | (0, 0) | 0–4 | row 4 column 3 left |
| 2 | ugv2 (2) | uav2 (3) | (0, −4.2) | 5–9 | row 3 column 2 right |
| 3 | ugv3 (4) | uav3 (5) | (0, 8.4) | 10–14 | row 6 column 3 right |
| 4 | ugv4 (6) | uav4 (7) | (0, −12.6) | 15–19 | row 1 column 2 left |
| 5 | ugv5 (8) | uav5 (9) | (−0.7, 4.2) | 20–24 | row 5 column 2 left |

Pair 5 starts 0.7 m to the −x side of its cross-aisle, because a building column stands at (0.8, 4.2).
The ROS domain is 51, the tmux socket is `whugvuav5p`, and only one warehouse session can run at a time.

## Generated files

`generate.py` writes `session.yml`, `env.sh`, `ugv_uav_pairs.rviz`, `models/*` (vehicle models, deck
boards, merged warehouse meshes) and `worlds/warehouse_light.sdf`. Edit `generate.py` (PAIRS,
SPAWNS, DEPLOY, LIGHT) and re-run it; don't edit those outputs. `./generate.py --pairs 3` makes a smaller session.

## What makes it lighter than the 2-pair session

Measured on this laptop (i9-13900H, RTX 4090 laptop), with the real-time factor (RTF) averaged over 20 s:

| change | why |
|---|---|
| **static visuals merged** into 7 meshes (`worlds/warehouse_light.sdf`; collisions unchanged, they are the mission's prior map) | gz sends every visual's pose on `/world/<w>/pose/info`, and **every PX4 instance subscribes to it**: 1491 poses per message at about 50 Hz before, 401 now |
| **gz started by the session** with `config/gz_server.config` (PX4's config minus `GstCameraSystem` / `OpticalFlowSystem`); every PX4 runs `PX4_GZ_STANDALONE` | GstCameraSystem H.264-encodes **every** camera, so the 5 down cameras were rendered and encoded all the time |
| down cameras only while landing: `aruco_landing_target.py state_topic:=/ugv_uav_mission_<k>/state` + lazy `ros_gz_bridge` | gz-sensors skips rendering a camera that has no subscriber. The aruco node also drops its 100 Hz PX4 odometry subscription while idle (it was about 14% CPU per node) |
| headless by default, no shadows, 96×72 depth cameras (was 160×120), no depth-image bridge | rendering |
| `kill.sh` kills whole pane process trees | leftover nodes from an earlier run were enough to halve the RTF |

**Result with 5 pairs:** RTF is about 0.65 at idle, about 0.5 while one pair flies, and 0.3–0.47 while 4–5 pairs fly at once.
All 5 pairs completed their missions concurrently (drive, inspect, ArUco landing, home). Everything runs on
sim time, so a lower RTF only slows the run down; it doesn't change the results.

**What still limits it:** gz alone with the same 10 vehicles (no PX4) runs at RTF 1.0. With PX4 attached,
gz's single main thread does about 3 ms of extra work per 4 ms physics step: it publishes IMU, clock and pose
to the 10 PX4 instances (and the bridges) on every step. On top of that come about 11 cores of ROS
load, mostly rbl_controller (CIRI on about 50k points per UGV).
Things that did **not** help:
- An 8 ms physics step reached RTF 0.83, but a UGV with its UAV blew up (it was thrown 2.5 m into the air). Don't use it.
- Pinning gz to its own P-cores (`PIN_CPUS=1`) made it worse (0.45).

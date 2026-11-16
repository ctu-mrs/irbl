# px4_sim_3ugv_3uav_warehouse_light

Three UGV + UAV pairs inspect warehouse shelves at the same time. This is `../px4_sim_5ugv_5uav_warehouse_light`
with 3 pairs (same generator, same light world and models, same mission per pair), its own goals, tmux socket
and ROS domain. The README of the 5-pair session explains what makes it lighter.

```bash
./start.sh            # headless gz (HEADLESS=0 ./start.sh for the gz GUI), rviz (WEB_VIZ=1: foxglove instead)
# you land in the `deploy` window: one pane per pair, only its own lines in the history.
# Wait for 'Arming' in the px4_bridge window, then C-a s (synchronize-panes) and arrow-up + Enter: every
# pair goes to its first shelf at once (2x arrow-up: second shelf, 3x: third, 4x: abort that pair).
# The operator pane in the `mission` window has all lines in one history (newest: all pairs in one line).
./deploy.sh pair 2 1 3 right      # = ./deploy.sh pair 2 row 1 column 3 right
./deploy.sh abort     # every pair (./deploy.sh pair 2 abort: just one)
./kill.sh             # kills the whole process tree of every pane
```

| pair | UGV (PX4 inst.) | UAV (PX4 inst.) | spawn (row) | ArUco ids | shelves in its `deploy` pane (arrow-up 1x, 2x, 3x) |
|---|---|---|---|---|---|
| 1 | ugv1 (0) | uav1 (1) | (0, 0) row 4 | 0–4 | 3 3 left, 5 2 right, 4 2 right |
| 2 | ugv2 (2) | uav2 (3) | (0, −4.2) row 3 | 5–9 | 6 2 left, 1 3 right, 3 3 left |
| 3 | ugv3 (4) | uav3 (5) | (0, 8.4) row 6 | 10–14 | 4 2 left, 7 3 right, 6 3 left |

Each round puts the 3 pairs in 3 different rows. In every round at least one pair goes +y while another
goes −y, and they also head in opposite x directions, so when all pairs are deployed at once they meet
head-on in the cross-aisles:

- **round 1 (rotate):** 1: row 4 → 3 (+x), 2: row 3 → 6 (−x), 3: row 6 → 4 (−x). Pairs 2 (going up) and 3 (going down) both leave from x = 0 and pass each other in that cross-aisle.
- **round 2 (shuffle):** 1: row 3 → 5 (up, −x), 2: row 6 → 1 (down across the whole warehouse, +x), 3: row 4 → 7 (up, +x). Pair 2 meets pairs 1 and 3 head-on.
- **round 3 (home):** each pair goes back to its own row, in another column or on another side than in round 1. Pair 1 goes down, pair 2 up, pair 3 down.

The ROS domain is 53 (`UGV_UAV_3PAIRS_DOMAIN_ID` overrides it), and the tmux socket is `whugvuav3p`. Only one warehouse
session can run at a time, because they share the gz world name and the PX4 ports. The model names are the same as in the 5-pair
session (`ugv<k>_carrier_light`, ...), and each session re-links its own copy into PX4 when it starts.

## Generated files

`generate.py` writes `session.yml`, `env.sh`, `ugv_uav_pairs.rviz`, `models/*` and `worlds/warehouse_light.sdf`.
Edit `generate.py` (PAIRS, SPAWNS, DEPLOY, LIGHT) and re-run it. Don't edit those outputs.

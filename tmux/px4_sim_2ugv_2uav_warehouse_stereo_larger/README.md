# px4_sim_2ugv_2uav_warehouse_stereo

This session runs two copies of `../px4_sim_ugv_uav_warehouse_stereo` in the same warehouse at the
same time. Each copy is a UGV carrying a UAV, both with stereo depth cameras and rbl_controller,
plus ArUco precision landing on the UGV deck. Each pair is driven by its own mission node.

| | pair 1 | pair 2 |
|---|---|---|
| vehicles (PX4 instance) | ugv1 (0, hosts the gz world), uav1 (1) | ugv2 (2), uav2 (3) |
| spawn | (0, 0) | (0, −4.2): cross-aisle / row 3 |
| gz models | `ugv1_carrier_stereo`, `uav1_x500_stereo` | `ugv2_carrier_stereo`, `uav2_x500_stereo` |
| deck landing board | ArUco ids 0–4 | ArUco ids 5–9 |
| mission node | `/ugv_uav_mission_1` | `/ugv_uav_mission_2` |
| MAV_SYS_ID / onboard MAVLink port | 1, 2 / 14581 (uav1) | 3, 4 / 14583 (uav2) |

**UGV deck.** Both UGVs carry a 0.40 × 0.40 m cargo deck, the size of the r1_rover body (it was
0.80 m before; the x500's skids span 0.25 × 0.28 m).
- The deck top is at the r1_rover's top, so the UAV spawn and landed heights are unchanged.
- The stereo camera sits on the rover's front face just below the deck edge: 0.215 m ahead of
  base_link, 0.24 m above it.
- The footprint is the plain rover's again, so the defaults apply: rbl `encumbrance` 0.45
  (`config/ugv_rbl.yaml`, a copy here), `planner.ugv_clearance` / `mission.ugv_clearance` 0.6,
  `mission.deck_tolerance` 0.15.

**Landing board.** Each deck carries a 0.36 m ArUco board instead of a single marker,
made by `models/make_deck_boards.py` (`deck_board_ugv<k>.png` in each UGV model: unique names, gz
ogre2 caches textures by file basename, so two `deck_board.png` rendered ugv1's board on both decks).
It has four 0.11 m markers at (±0.095, ±0.095) m and a 0.05 m marker in the centre. The corner
markers are detected from the rendezvous / search height (~28 px at 1.5 m). The small one stays
fully in the down camera's view until touchdown (camera ~0.19 m above it when landed).
`aruco_landing_target.py` fits one pose to the corners of every board marker it sees
(`board_ids` / `board_sizes` / `board_centers` in session.yml, same layout as the script).

Each vehicle has its own model because a model hard-codes three things:
- its TF frame ids: `<vehicle>/stereo_link` and `<vehicle>/down_cam_link`
- which UAV its DetachableJoint holds, and the joint's `/ugv<k>/uav_{detach,attach}` topics
- the landing board texture on its deck

The models are otherwise the stereo session's models. Airframes 4043 and 4044 are reused from
there through `PX4_SYS_AUTOSTART`. The configs `uav_rbl.yaml`, `{ugv,uav}_px4.yaml` and
`warehouse_map.yaml` are symlinks to the stereo session's files, so tuning stays in one place.
`ugv_rbl.yaml` is a copy (it carried a different encumbrance for the old 0.80 m deck).
For ugv2, the rbl_controller pane writes a copy of `ugv_rbl.yaml` to `/tmp` with
`/uav2/odometry` in `filter_agents`.

The two pairs don't coordinate with each other. Every rbl_controller treats the other pair's
vehicles as obstacles that its camera sees, and each UGV removes only its own UAV from its point
cloud. The global planner of each mission plans on the static map only.

```bash
./start.sh                                     # HEADLESS=1 for no gz GUI
./deploy.sh pair 1 row 5 column 2 left         # once all four px4_bridge panes are up
./deploy.sh pair 2 row 3 column 3 right
./deploy.sh abort                              # both pairs (./deploy.sh pair 2 abort: one)
./kill.sh
```

Only one warehouse session can run at a time, because they all use the same gz/PX4 instances.
This one uses tmux socket `whugvuav2p` and ROS_DOMAIN_ID 50.

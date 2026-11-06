# px4_sim_ugv_uav_greenhouse

Same UGV + UAV mission as `../px4_sim_ugv_uav_warehouse` (everything below applies), in a **greenhouse**:
`worlds/greenhouse.sdf` (generator `../px4_sim_1_uav_3dlidar/worlds/worldgen/worlds/greenhouse.py`).

- 60 x 31 m glasshouse, 8 crop rows along x (0.8 m deep; from the main aisle outwards: high-wire
  tomatoes, bell peppers, cucumbers, strawberries on raised gutters at 1.1 m), crop wire at 3.0 m,
  steel posts, 7 aisles 3 m wide (y = -11.4 .. 11.4, main aisle y = 0), cross-aisles at x = -14, 0, 14
  -> 4 segments per aisle. Blue harvest crates in the aisles are temporary obstacles (not in the
  planners' prior map). `./deploy.sh map` lists rows / columns.
- The plants are generated meshes (stems, leaves, fruit; ~510k triangles, one OBJ per row segment) in
  `worlds/models/greenhouse_crops` (`model://greenhouse_crops`, linked into PX4's gz models folder by
  session.yml). Visual only: cameras and lidars see the real foliage with its gaps; collision (physics
  and the planners' map) is one box per row segment around the canopy.
- What differs from the warehouse session: `config/greenhouse_map.yaml` (map + coverage: levels
  0.9 / 1.6 / 2.2 m, z_max 2.25 m), `config/uav_rbl.yaml` (z_max 2.6: under the crop wires), tmux
  session `ugv_uav_greenhouse` / socket `ghugvuav`, `ROS_DOMAIN_ID` 49, `PX4_GZ_WORLD=greenhouse`.
- Shared with the warehouse session (symlinks, so fixes there apply here too): `models/`, `airframes/`,
  `config/{ugv,uav}_px4.yaml`, `config/{ugv,uav}_sensors.yaml`, `config/ugv_rbl.yaml`.
- Run one of the two sessions at a time (same Gazebo / PX4 SITL ports, same MicroXRCEAgent port):
  `../px4_sim_ugv_uav_warehouse/kill.sh` first, then `./start.sh`; `./kill.sh` stops this one.

  ./start.sh
  ./deploy.sh row 4 column 2 left      # once the px4_bridge window printed 'Arming' (UGV)

---
Original README of the warehouse session (file names: warehouse_map.yaml -> greenhouse_map.yaml here):

# px4_sim_ugv_uav_warehouse

The UGV drives to an aisle position and stops. The UAV takes off from it, scans the shelf and hovers
where it leaves the aisle; the UGV then drives below it (PICKUP) and the UAV does PX4's built-in precision
landing on the ArUco marker of its deck (down camera, see below) and is re-attached. Then the UGV carries
it back to its initial position. Both vehicles run the same, unchanged, 3D `rbl_controller`; PX4 SITL
x2 + Gazebo.

| | UGV (`ugv1`) | UAV (`uav1`) |
|---|---|---|
| PX4 | instance 0, differential rover (unicycle), airframe 4041, topics `/ugv_1/fmu/...` | instance 1, x500, airframe 4042, topics `/uav_1/fmu/...` |
| gz model | `r1_rover_carrier`: r1_rover + forward 3D lidar (as the UAV's) + ArUco marker on the deck + DetachableJoint holding the UAV | `x500_lidar_3d_cam_down` (3D lidar + down camera) |
| rbl | 3D cell as the UAV's, on its lidar cloud (`config/ugv_rbl.yaml`); planar control: reference z = its own (`planar_control`) | 3D, on the lidar cloud (`config/uav_rbl.yaml`) |
| on top | `ugv_uav_mission_node`: phases, and UGV routes along the aisle grid of the map | `warehouse_inspection_node`: shelf coverage |

Phases: `DRIVE -> TAKEOFF -> INSPECT -> PICKUP -> RENDEZVOUS -> LAND -> ATTACH -> HOME -> DONE`
(ATTACH disarms the UAV; with several shelves, ATTACH goes on to the next shelf's DRIVE, HOME only after the last).

Planner (`PLANNER` in `env.sh`, default `global`; `config/warehouse_map.yaml` `planner.*`):
- `global`: each UGV leg is planned **once**, start -> goal, by A* on the known map (the world SDF's
  obstacles, `rbl_mission/global_planner.hpp`) and sent whole to `rbl_controller ~/goto_path`
  (`rbl_msgs/srv/GotoPath`), which follows it **without** its local replanner (rbl's reactive avoidance
  stays on). The UAV's coverage legs, already planned on the map, go the same way as straight paths.
  rviz: `/ugv1/rbl_controller/path` (or `/ugv_uav_mission/ugv_global_path`).
- `local` (`PLANNER=local ./start.sh`): the previous behaviour, `~/goto` + rbl's local replanner.
  A plain `~/goto` always switches an rbl_controller back to its local replanner.

```bash
# once, in robogistic_ws (px4_msgs / px4_ros_com from an underlay, e.g. ~/ros_wss/irbl-ws)
source /opt/ros/jazzy/setup.bash && source ~/ros_wss/irbl-ws/install/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select \
  ciri rbl_msgs rbl_controller_core rbl_replanner rbl_controller_node rbl_px4_bridge rbl_mission

./start.sh                             # HEADLESS=1 ./start.sh without the gz GUI
./deploy.sh row 5 column 2 left        # ~30 s after 'Arming': aisle (row 1-7), segment (column 1-4), shelf side
./deploy.sh map                        # list the rows / columns and their coordinates
./deploy.sh row 3 column 3 right       # again once DONE (the UAV is back on the UGV, at home)
./deploy.sh row 5 column 2 left  row 3 column 3 right  row 1 column 4 left   # several shelves in sequence
./deploy.sh 5 2 left 3 3 right                                            # (short form)
./deploy.sh abort
./kill.sh
```

rviz (`ugv_uav.rviz`): simplified picture from `mission_viz_node`, i.e. the warehouse, the UGV (orange)
and the UAV (blue) labelled with their phase, the UAV coverage plan, both planned paths and the
UGV lidar cloud, plus each vehicle's rbl centroid and sensing cell (actively sensed part of
its cell A; the full cell A and the goal are there too, off). `ugv_uav_debug.rviz` shows everything (TF, rbl cells, raw clouds). The UGV's TF
frame sits at a fixed 0.6 m (`env.sh` explains why); its body is drawn on the floor.

Precision landing (LAND phase, `mission.landing: px4_precland`, the default): once the UAV is above the
parked UGV, `ugv_uav_mission_node` calls `px4_bridge ~/precland` (PX4 mode AUTO_PRECLAND) and PX4 does
the rest. The marker (ArUco `DICT_4X4_50` id 0, 0.30 m) is detected in the UAV's down camera by
`rbl_mission/scripts/aruco_landing_target.py`, which sends MAVLink `TARGET_RELATIVE` to PX4 (onboard
link, UDP 14581); PX4's `mavlink_receiver` turns each one into `landing_target_pose` for the precision
landing (`PLD_*`, `config/uav_px4.yaml`). The `vision_target_estimator` is off (`VTE_EN 0`, airframe
4042: it diverged in this sim). No PX4 rebuild: the SITL build already has the MAVLink development
dialect. Check on the PX4 shell (`px4_sitl` window, UAV pane): `listener landing_target_pose`.
`mission.landing: tracked` brings back the old descent tracked by the mission node.

Simulation notes (details in the files): magnetic declination fixed to the gz field in both airframes
(PX4's model was ~6 deg off); UAV range aiding off (it takes off from / lands on the deck);
`rbl_px4_bridge` gained `~/arm`, `~/land`, `~/precland`, `~/disarm`, `takeoff_at_current_position` and `ground_vehicle`
(defaults unchanged for the other sessions).

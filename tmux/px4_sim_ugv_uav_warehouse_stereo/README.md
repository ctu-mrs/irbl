# px4_sim_ugv_uav_warehouse_stereo

The same thing as `../px4_sim_ugv_uav_warehouse`: same mission (`deploy.sh`), same PX4 setup, same
ArUco precision landing on the UGV deck, and rbl_controller is unchanged. The difference is the
obstacle sensor. **Both vehicles use a forward stereo depth camera instead of the 3D lidar.** It is
the camera from `../px4_sim_warehouse_inspection_stereo`: OAK-D-Lite-like optics, 73° × 58°,
0.2–19.1 m, scaled down to 160×120 @ 10 Hz, with σ 2 cm noise.

| | lidar session | this session |
|---|---|---|
| UGV gz model / airframe | `r1_rover_carrier` / 4041 | `r1_rover_carrier_stereo` / 4044 |
| UAV gz model / airframe | `x500_lidar_3d_cam_down` / 4042 | `x500_stereo_cam_down` / 4043 (x500 + stereo + down cam, no LW20) |
| UGV cloud | `/ugv1/sensors/lidar_3d/points` | `/ugv1/sensors/stereo/points` (+ `/ugv1/sensors/stereo/depth`) |
| UAV cloud | `/uav1/sensors/lidar_3d/points` | `/uav1/sensors/stereo/points` (+ `/uav1/sensors/stereo/depth`) |
| sensor frames | `ugv_lidar_link`, `livox_link` | `ugv_stereo_link`, `uav_stereo_link` |
| rbl FOV | `lidar_tilt 90`, `lidar_fov 180` | `lidar_tilt 29`, `lidar_fov 58` |
| tmux socket / session | `whugvuav` / `ugv_uav_warehouse` | `whugvuavst` / `ugv_uav_warehouse_stereo` |
| ROS_DOMAIN_ID | 48 | 49 (`UGV_UAV_STEREO_DOMAIN_ID`) |

Camera mounts:
- UGV: 0.21 m ahead of / 0.25 m above base_link (~0.30 m above the floor), untilted. The floor
  shows up from ~0.55 m ahead. The carried UAV is out of view.
- UAV: the same mount as `x500_stereo` (0.12 m ahead, 0.242 m above base_link, above the
  propellers).

Gazebo's depth cloud is x-forward rather than optical, so `config/*_sensors.yaml` only adds the
mount offset.

`config/{ugv,uav}_px4.yaml` and `config/warehouse_map.yaml` are symlinks to the lidar session's
files. The other configs are copies.

The models get new names so that linking them into `$PX4_DIR/Tools/simulation/gz/models` doesn't
replace the lidar session's links. Run only one of the two sessions at a time, because both use
gz/PX4 instances 0 and 1.

```bash
./start.sh
./deploy.sh row 5 column 2 left    # once both px4_bridge panes printed 'Arming'/'px4_bridge started'
./kill.sh
```

Differences from the lidar to expect:
- Each vehicle only sees what is in front of it (±36° horizontally, ±29° vertically). rbl yaws
  towards where it is going (`limited_fov: true`), so moving sideways or backwards relies on the
  remembered static map.
- During a shelf pass the UAV looks along the aisle, not at the shelf.
- rbl_controller logs PCL's `Failed to find match for field 'intensity'` because the depth cloud
  has no intensity channel. This is harmless: `reflective_intensity_threshold` is < 0.

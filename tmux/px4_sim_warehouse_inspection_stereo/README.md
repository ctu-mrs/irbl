# px4_sim_warehouse_inspection_stereo

Same as `../px4_sim_warehouse_inspection` (GOTO aisle position → COVERAGE scan of the shelf →
RETURN, all via `rbl_mission`'s `warehouse_inspection_node`, rbl_controller unchanged), but the
UAV senses with a **forward stereo depth camera** instead of the 3D lidar. Its point cloud goes to
rbl_controller's `pcl_topic` exactly like the lidar cloud did.

| | lidar session | this session |
|---|---|---|
| vehicle | `gz_x500_lidar_3d` (PX4) | `gz_x500_stereo` (`models/x500_stereo`, `airframes/4040_gz_x500_stereo`) |
| sensor | 360° x 90°, 10.8 k pts @ 10 Hz | 73° x 58°, 0.2–19.1 m, 160x120 = 19.2 k pts @ 10 Hz, σ 2 cm noise |
| cloud | `/uav1/sensors/lidar_3d/points` | `/uav1/sensors/stereo/points` (+ `/uav1/sensors/stereo/depth` image) |
| rbl FOV | `lidar_tilt 90`, `lidar_fov 180` | `lidar_tilt 29`, `lidar_fov 58` (`config/rbl_controller.yaml`) |
| EKF range aid | on (LW20 down) | off, no rangefinder on this model |

The camera optics/mount follow PX4's `x500_depth` (OAK-D Lite), with resolution and rate reduced
from 640x480 @ 30 Hz: rbl's local static map ray-casts every cloud point, and the full stream
(~9 M points/s) would stall it. Raise `<width>/<height>/<update_rate>` in `models/x500_stereo/model.sdf`
if your machine keeps up. Gazebo's depth cloud is x-forward (not optical), frame `stereo_link`,
which `config/sensor_config.yaml` mounts on `uav1/fcu`.

The session links `models/x500_stereo` into `$PX4_DIR/Tools/simulation/gz/models` (PX4 only spawns
from there), the same way the worlds are linked. `config/warehouse_map.yaml` is a link to the lidar
session's, so both use one map.

```bash
./start.sh
./inspect.sh -28 8.4 left        # once px4_bridge printed 'Arming' and the UAV hovers
./kill.sh
```

Behaviour to expect vs. the lidar: rbl yaws towards where it is going (`limited_fov: true`), so
the camera sees what the UAV flies into but not the sides; during a shelf pass it looks along the
aisle, not at the shelf. Vertical moves at the pass ends rely on the remembered static map, since
the camera sees only ±29° in elevation. rbl_controller will log PCL's
`Failed to find match for field 'intensity'` for this cloud (it has no intensity channel); harmless,
`reflective_intensity_threshold` is < 0 so intensity is unused.

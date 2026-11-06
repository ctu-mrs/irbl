#!/bin/bash
# Publishes one static TF per entry in config/sensor_config.yaml, each parented on "$UAV_NAME/fcu"
# (the frame px4_bridge_node broadcasts live from vehicle odometry -- see px4_bridge_node.cpp).
# Chained together this gives tf2 a live path from any sensor's own frame (e.g. the 3D lidar's
# "livox_link", set via <gz_frame_id> in the gz sensor plugin and carried straight through into
# the bridged point cloud's header.frame_id) into control_frame, which is what lets
# rbl_controller_node's "~/pcl_in" transform a real sensor's point cloud automatically instead of
# assuming it already arrives in control_frame the way the old synthetic map_generator feed did.
#
# Independent of PX4 -- doesn't need to wait for "pxh>" like set_px4_params.sh/fix_arming_healthcheck.sh.

CONFIG="$(dirname "$0")/config/sensor_config.yaml"

pids=()

while read -r FRAME X Y Z ROLL PITCH YAW; do
  ros2 run tf2_ros static_transform_publisher \
    --x "$X" --y "$Y" --z "$Z" --roll "$ROLL" --pitch "$PITCH" --yaw "$YAW" \
    --frame-id "$UAV_NAME/fcu" --child-frame-id "$FRAME" \
    --ros-args -p use_sim_time:=true &
  pids+=("$!")
  echo "publishing static TF: $UAV_NAME/fcu -> $FRAME"
done < <(python3 - "$CONFIG" <<'PYEOF'
import sys, yaml
with open(sys.argv[1]) as f:
    cfg = yaml.safe_load(f)
for sensor in cfg["sensors"]:
    tx, ty, tz = sensor["translation"]
    roll, pitch, yaw = sensor["rotation"]
    print(sensor["frame_id"], tx, ty, tz, roll, pitch, yaw)
PYEOF
)

wait "${pids[@]}"

#!/bin/bash
# ./publish_sensor_tf.sh <vehicle name> <sensor yaml>
# One static TF "<vehicle>/fcu" -> "<vehicle>/<frame_id>" per entry of the yaml (see config/*_sensors.yaml).
# px4_bridge broadcasts control_frame -> "<vehicle>/fcu" live, so tf2 can bring every sensor's
# cloud into the shared control frame for that vehicle's rbl_controller.
VEHICLE=$1
CONFIG=$2
pids=()
while read -r FRAME X Y Z ROLL PITCH YAW; do
  ros2 run tf2_ros static_transform_publisher \
    --x "$X" --y "$Y" --z "$Z" --roll "$ROLL" --pitch "$PITCH" --yaw "$YAW" \
    --frame-id "$VEHICLE/fcu" --child-frame-id "$VEHICLE/$FRAME" \
    --ros-args -p use_sim_time:=true -r __node:=sensor_tf_${VEHICLE}_${FRAME} &
  pids+=("$!")
  echo "publishing static TF: $VEHICLE/fcu -> $VEHICLE/$FRAME"
done < <(python3 - "$CONFIG" <<'PYEOF'
import sys, yaml
for s in yaml.safe_load(open(sys.argv[1]))["sensors"]:
    print(s["frame_id"], *s["translation"], *s["rotation"])
PYEOF
)
wait "${pids[@]}"

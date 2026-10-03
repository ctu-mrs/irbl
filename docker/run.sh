#!/bin/bash
# Start the irbl container with X11 (Gazebo GUI + rviz2).
# Inside:  cd px4_sim_1_uav_3dlidar && ./start.sh      (or px4_sim_1_uav, px4_sim_2_uav, ...)

IMAGE=${IMAGE:-irbl:jazzy}

xhost +local:docker >/dev/null 2>&1

GPU_ARGS=()
if command -v nvidia-smi >/dev/null 2>&1 && docker info 2>/dev/null | grep -qi nvidia; then
  GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
elif [ -e /dev/dri ]; then
  # Mesa can't drive the proprietary NVIDIA driver's DRM nodes; on hybrid laptops passing them makes
  # EGL pick the NVIDIA card and gz's GPU lidar (Ogre2) segfault. Only pass the Mesa-capable nodes.
  for node in /dev/dri/card* /dev/dri/renderD*; do
    [ "$(basename "$(readlink -f "/sys/class/drm/$(basename "$node")/device/driver")")" = nvidia ] && continue
    GPU_ARGS+=(--device "$node")
  done
fi

docker run -it --rm \
  --name irbl \
  --net=host --ipc=host \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  "${GPU_ARGS[@]}" \
  "$@" \
  "$IMAGE"

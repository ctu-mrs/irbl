#!/bin/sh
# Sourced (not executed) by every pane via session.yml's pre_window. Exports the `launch:` section of
# config/custom_config.yaml as environment variables, then derives the values the individual
# windows need from them (PX4 instance -> gz model name, uXRCE-DDS namespace -> px4_bridge px4_ns,
# ...). POSIX sh so it works from both zsh and bash.
#
# Precedence: a non-null value in the yaml always wins, even over something your ~/.zshrc exports
# (e.g. ROS_DOMAIN_ID); a null/omitted value leaves whatever the shell already has. The one
# exception is the world: ./start.sh <world> (or PX4_GZ_WORLD=<world> ./start.sh) beats the yaml.

LAUNCH_ENV_CONFIG="${LAUNCH_ENV_CONFIG:-./config/custom_config.yaml}"

eval "$(python3 - "$LAUNCH_ENV_CONFIG" <<'PYEOF'
import sys, shlex, yaml
with open(sys.argv[1]) as f:
    launch = (yaml.safe_load(f) or {}).get("launch") or {}
for name, value in launch.items():
    if value is None:
        continue
    print(f"export {name}={shlex.quote(str(value))}")
PYEOF
)"

[ -n "$PX4_GZ_WORLD_CLI" ] && export PX4_GZ_WORLD="$PX4_GZ_WORLD_CLI"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-cylinder_forest}"
export HEADLESS="${HEADLESS:-0}"
export PX4_INSTANCE="${PX4_INSTANCE:-0}"
export PX4_UXRCE_DDS_PORT="${PX4_UXRCE_DDS_PORT:-8888}"

# Namespace the uXRCE-DDS client puts in front of every /fmu/... topic, resolved the same way PX4's
# own rcS + uxrce_dds_client do: explicit PX4_UXRCE_DDS_NS (even empty) > uav_<PX4_UXRCE_DDS_NS_IDX>
# (what the UXRCE_DDS_NS_IDX param would give -- that param is only read when the client starts, so
# it can't go through set_px4_params.sh) > px4_<instance> for instance > 0 > none.
if [ -z "${PX4_UXRCE_DDS_NS+x}" ] && [ -n "$PX4_UXRCE_DDS_NS_IDX" ] && [ "$PX4_UXRCE_DDS_NS_IDX" -ge 0 ]; then
  export PX4_UXRCE_DDS_NS="uav_$PX4_UXRCE_DDS_NS_IDX"
fi
if [ -n "${PX4_UXRCE_DDS_NS+x}" ]; then
  export PX4_FMU_NS="$PX4_UXRCE_DDS_NS"
elif [ "$PX4_INSTANCE" -ne 0 ]; then
  export PX4_FMU_NS="px4_$PX4_INSTANCE"
else
  export PX4_FMU_NS=""
fi

# rcS sets MAV_SYS_ID (= the VehicleCommand target_system) to instance+1, and px4-rc.gzsim spawns
# the vehicle as <model>_<instance>.
export PX4_TARGET_SYSTEM=$((PX4_INSTANCE + 1))
# NOT named PX4_GZ_MODEL_NAME: px4-rc.gzsim reads that as "attach to an already-spawned model" and
# then never spawns the vehicle (no sensors -> no odometry -> never arms).
export GZ_VEHICLE_MODEL_NAME="x500_lidar_3d_$PX4_INSTANCE"

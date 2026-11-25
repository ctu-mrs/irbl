#!/bin/bash
# Start the simulation stack of one scenario (compose.yaml).

cd "$(dirname "$(readlink -f "$0")")"

SCENARIOS=$(ls scenarios | sed 's/\.env$//' | paste -sd '|')
HELP="usage: ./up.sh <$SCENARIOS> [arguments]
Arguments:
-n                      Don't synchronize the workspace (keep the colcon_workspace volume; ../../tmux is copied anyway)
-r                      Also record a rosbag (service 'rosbag')
Starts the containers only: ../tmux.sh <scenario> runs the scenario's tmux session in them.
"

SCENARIO=1_uav
if [ -n "$1" ] && [ "${1#-}" = "$1" ]; then
  SCENARIO=$1
  shift
fi

COPY_WORKSPACE=true
PROFILES=()

OPTIND=1
while getopts "hH?nNrR" opt; do
  case "$opt" in
    h|H|\?) echo "$HELP"; exit 0 ;;
    n|N)    COPY_WORKSPACE=false ;;
    r|R)    PROFILES+=(--profile record) ;;
  esac
done

if [ ! -f "./scenarios/$SCENARIO.env" ]; then
  echo "$0: unknown scenario '$SCENARIO'"
  echo "$HELP"
  exit 1
fi

# settings for docker compose (exported: the shell environment is what compose interpolates)
set -a
source ./stack.env
source "./scenarios/$SCENARIO.env"
set +a

[ -n "$WEB_VIZ" ] && PROFILES+=(--profile web) && export DISPLAY=:99   # Xvfb in the sim container
# the host's tmux/ folder, linked in the sim container to the copy in the workspace volume (pane.sh, tmuxinator)
export HOST_TMUX=$(readlink -f ../../tmux)
# the scenario's tmux socket (session.yml socket_name), linked in the sim container to the host's
export TMUX_SOCKET=$(sed -n 's/^socket_name: *//p' "$HOST_TMUX/$TMUX_DIR/session.yml" | head -1)
export HOST_UID=$(id -u)
mkdir -p "$HOST_TMUX/$TMUX_DIR/scans"   # bind-mounted in the sim container (scan reports), else root-owned
mkdir -p -m 700 /tmp/tmux-$HOST_UID   # else docker creates it, owned by root

COLCON_BUILDER_PATH=./../colcon_workspace_builder
WORKSPACE_CACHE=${COLCON_BUILDER_PATH}/cache/etc/docker/colcon_workspace
COMPOSE=(docker compose -p "$SESSION_NAME" "${PROFILES[@]}")

if ! docker image inspect "$IMAGE" > /dev/null 2>&1; then
  echo "$0: image '$IMAGE' not found, build it with smart-deployment/docker/lazydocker/base_image/build.sh"
  exit 1
fi

if $COPY_WORKSPACE && [ ! -e ${WORKSPACE_CACHE}/install ]; then
  echo "$0: workspace not built, run ../colcon_workspace_builder/build.sh"
  exit 1
fi

# GPU=dri: only the /dev/dri nodes Mesa can drive. On hybrid laptops the proprietary NVIDIA driver's nodes
# make EGL pick the NVIDIA card and gz's GPU lidar (Ogre2) segfault.
if [ "${GPU:-dri}" = dri ]; then
  {
    echo "# written by up.sh (GPU=dri in ./stack.env): the host's Mesa-capable /dev/dri nodes"
    echo "x-dri: &dri"
    echo "  devices:"
    for node in /dev/dri/card* /dev/dri/renderD*; do
      [ -e "$node" ] || continue
      [ "$(basename "$(readlink -f "/sys/class/drm/$(basename "$node")/device/driver")")" = nvidia ] && continue
      echo "    - $node:$node"
    done
    echo "services:"
    echo "  sim: *dri"
    echo "  rosbag: *dri"
  } > compose.gpu-dri.yaml
fi

if $COPY_WORKSPACE; then
  echo "$0: removing old colcon_workspace volume"
  docker volume rm -f ${SESSION_NAME}_colcon_workspace > /dev/null
fi

"${COMPOSE[@]}" up -d init

if $COPY_WORKSPACE; then
  echo "$0: copying colcon workspace"
  # compress the install + src folders of the workspace and pass them to the "colcon_workspace" volume
  tar -C ${COLCON_BUILDER_PATH}/cache/etc/docker --exclude build --exclude log -czh ./colcon_workspace | "${COMPOSE[@]}" cp - init:/etc/docker
fi

# the tmux sessions, live from the repo (no rebuild needed after editing them). At src/irbl/tmux, so the
# sessions' `source ../../../../install/setup.zsh` finds the workspace. Symlinks are kept (all relative,
# inside tmux/).
echo "$0: copying ../../tmux"
tar -C ../.. --exclude ./tmux/old --exclude ./tmux/*/scans --exclude __pycache__ --exclude .tmuxinator.yml -cz ./tmux \
  | "${COMPOSE[@]}" cp - init:/etc/docker/colcon_workspace/src/irbl

# GUIs in the containers (root) on the host display
[ -n "$DISPLAY" ] && xhost +SI:localuser:root > /dev/null 2>&1

echo "$0: starting the containers of '$SCENARIO' (tmux/$TMUX_DIR)"
"${COMPOSE[@]}" up -d || exit 1
until docker exec "$SESSION_NAME-sim-1" test -e "$HOST_TMUX" 2> /dev/null; do sleep 0.2; done
[ -n "$WEB_VIZ" ] && echo "$0: open http://localhost:8080/?ds=foxglove-websocket&ds.url=ws://localhost:8765"
exit 0

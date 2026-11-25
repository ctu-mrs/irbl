#!/bin/bash
# The docker version of ../tmux/px4_sim_*/start.sh: the scenario's own tmux session (session.yml, unchanged),
# with tmux on the PC and every pane a shell in the `sim` container (sim/pane.sh), which runs the image
# smart-deployment:latest with the workspace built by colcon_workspace_builder/. One tmux, nothing nested.
# Two windows are added at the end, on the host: lazydocker and workspace (colcon_workspace_builder/).
#
# usage: ./tmux.sh [scenario] [-d] [-r] [-n]   (scenarios: sim/scenarios/*.env, default 1_uav)
#   -d: start detached (re-attach: tmux -L <socket_name of the scenario's session.yml> a)
#   -r: also record a rosbag, -n: keep the workspace volume (sim/up.sh)
#   START_ARGS=city ./tmux.sh 1_uav_3dlidar   the argument of the scenario's start.sh
# Stop with ./kill.sh

DOCKER_DIR=$(dirname "$(readlink -f "$0")")
cd "$DOCKER_DIR"

SCENARIO=1_uav
if [ -n "$1" ] && [ "${1#-}" = "$1" ]; then
  SCENARIO=$1
  shift
fi

DETACH=false
UP_ARGS=()
for a in "$@"; do
  case "$a" in
    -d) DETACH=true ;;
    -r|-n) UP_ARGS+=("$a") ;;
    *) echo "$0: unknown argument '$a'"; exit 1 ;;
  esac
done

if [ ! -f "./sim/scenarios/$SCENARIO.env" ]; then
  echo "usage: $0 [$(ls sim/scenarios | sed 's/\.env$//' | paste -sd '|')] [-d] [-r] [-n]"
  exit 1
fi

# settings of the scenario: for docker compose, and inherited by the tmux server (pane.sh reads them)
set -a
source ./sim/stack.env
source "./sim/scenarios/$SCENARIO.env"
set +a

SCENARIO_DIR=$(readlink -f "../tmux/$TMUX_DIR")
SESSION_YML=$SCENARIO_DIR/session.yml

# the scenario's own session / socket names: its scripts (set_px4_params.sh, ...) use them, from the container
TMUX_SESSION_NAME=$(sed -n 's/^name: *//p' "$SESSION_YML" | head -1)
TMUX_SOCKET_NAME=$(sed -n 's/^socket_name: *//p' "$SESSION_YML" | head -1)
export TMUX_SESSION_NAME TMUX_SOCKET_NAME

./kill.sh > /dev/null

# the same session on the host (../tmux/<dir>/start.sh) would share gz-transport, the uXRCE-DDS ports and
# the ROS domain with this one (and its tmux socket)
if tmux -L "$TMUX_SOCKET_NAME" has-session 2>/dev/null; then
  echo "$0: a host session runs on tmux socket '$TMUX_SOCKET_NAME' (../tmux/$TMUX_DIR/start.sh): stop it first"
  exit 1
fi
./sim/up.sh "$SCENARIO" "${UP_ARGS[@]}" || exit 1

# tmux config of the session: the scenario's own (tmux_options: -f ./tmux.conf), else ~/.tmux.conf, then
# pane.sh as the shell of every pane
GEN_CONF=$DOCKER_DIR/sim/.tmux.conf
SCENARIO_CONF=$(sed -n 's/^tmux_options: *-f *//p' "$SESSION_YML" | head -1)
{
  echo "# written by ../tmux.sh $SCENARIO"
  if [ -n "$SCENARIO_CONF" ]; then
    echo "source-file $SCENARIO_DIR/${SCENARIO_CONF#./}"
  elif [ -f ~/.tmux.conf ]; then
    echo "source-file ~/.tmux.conf"
  fi
  echo "set-option -g default-command $DOCKER_DIR/sim/pane.sh"
} > "$GEN_CONF"

# the scenario's session.yml with this config and an absolute root
GEN_YML=$DOCKER_DIR/sim/.session.yml
sed -e "s#^root:.*#root: $SCENARIO_DIR#" \
    -e "/^tmux_options:/d" \
    -e "s#^attach:.*#attach: false#" \
    "$SESSION_YML" > "$GEN_YML"
echo "tmux_options: -f $GEN_CONF" >> "$GEN_YML"

echo "$TMUX_SOCKET_NAME $TMUX_SESSION_NAME" > "$DOCKER_DIR/sim/.tmux_session"   # for kill.sh
tmuxinator start -p "$GEN_YML" || exit 1

# host windows (explicit commands: not pane.sh)
T=(tmux -L "$TMUX_SOCKET_NAME")
"${T[@]}" new-window -d -t "$TMUX_SESSION_NAME:" -n lazydocker -c "$DOCKER_DIR/sim" \
  "COMPOSE_PROJECT_NAME=$SESSION_NAME lazydocker; exec bash"
"${T[@]}" new-window -d -t "$TMUX_SESSION_NAME:" -n workspace -c "$DOCKER_DIR/colcon_workspace_builder" bash

$DETACH && exit 0

if [ -z "$TMUX" ]; then
  "${T[@]}" a -t "$TMUX_SESSION_NAME"
else
  tmux detach-client -E "tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME"
fi

#!/bin/bash
SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

export TMUX_SESSION_NAME=ugv_uav_warehouse
export TMUX_SOCKET_NAME=whugvuav

tmux -L $TMUX_SOCKET_NAME list-panes -s -t $TMUX_SESSION_NAME -F "#{pane_pid}" 2>/dev/null | while read -r pid; do
  pkill -TERM -P "$pid" 2>/dev/null
done
tmux -L $TMUX_SOCKET_NAME kill-session -t $TMUX_SESSION_NAME 2>/dev/null
sleep 1

pkill -f MicroXRCEAgent 2>/dev/null
pkill -f "bin/px4" 2>/dev/null
pkill -f "px4_sitl_default" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "gz-sim" 2>/dev/null
pkill -f "px4_bridge_node" 2>/dev/null
pkill -f "parameter_bridge" 2>/dev/null
pkill -f "static_transform_publisher" 2>/dev/null
pkill -f "rbl_controller_container" 2>/dev/null
pkill -f "warehouse_inspection_node" 2>/dev/null
pkill -f "ugv_uav_mission_node" 2>/dev/null
pkill -f "mission_viz_node" 2>/dev/null
pkill -f "rviz2" 2>/dev/null
pkill -f "foxglove_bridge" 2>/dev/null

# a hung gz server ignores SIGTERM and would be silently reused by the next start (no GUI, no spawn)
sleep 2
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "bin/px4" 2>/dev/null

echo "ugv_uav_warehouse stopped"

#!/bin/bash

SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

export TMUX_SESSION_NAME=px4_simulation_2uav
export TMUX_SOCKET_NAME=px4sim2

# terminate every process tree running inside the session's panes
tmux -L $TMUX_SOCKET_NAME list-panes -s -t $TMUX_SESSION_NAME -F "#{pane_pid}" 2>/dev/null | while read -r pid; do
  pkill -TERM -P "$pid" 2>/dev/null
done

tmux -L $TMUX_SOCKET_NAME kill-session -t $TMUX_SESSION_NAME 2>/dev/null

sleep 1

# PX4 SITL (both instances) / gz / the uXRCE-DDS agent / our own nodes sometimes linger after
# the session is gone
pkill -f MicroXRCEAgent 2>/dev/null
pkill -f "bin/px4" 2>/dev/null
pkill -f "px4_sitl_default" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "gz-sim" 2>/dev/null
pkill -f "px4_bridge_node" 2>/dev/null
pkill -f "group_state_relay_node" 2>/dev/null
pkill -f "random_forest" 2>/dev/null
pkill -f "rbl_controller_container" 2>/dev/null
pkill -f "rviz2" 2>/dev/null

echo "px4_simulation_2uav stopped"

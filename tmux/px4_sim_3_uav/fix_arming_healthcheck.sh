#!/bin/bash
# PX4's own gz_x500 airframe startup script sets `NAV_DLL_ACT` to a non-zero value by default
# (see ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500), which makes the commander refuse
# to arm without a GCS/MAVLink heartbeat. We don't have one here since this stack talks to PX4
# purely over uXRCE-DDS, so this resets NAV_DLL_ACT to 0 for the running SITL instance so
# px4_bridge_node's auto-arm sequence can succeed.
#
# Usage: ./fix_arming_healthcheck.sh <tmux-window-name>
#
# This is a runtime `param set` against the given instance, not a permanent edit of any file in
# this repository.

export TMUX_SOCKET_NAME=px4sim3
export TMUX_SESSION_NAME=px4_simulation_3uav

WINDOW="${1:?usage: $0 <tmux-window-name>}"

until tmux -L $TMUX_SOCKET_NAME capture-pane -p -t $TMUX_SESSION_NAME:$WINDOW 2>/dev/null | grep -q "pxh>"; do
  sleep 1
done

tmux -L $TMUX_SOCKET_NAME send-keys -t $TMUX_SESSION_NAME:$WINDOW "param set NAV_DLL_ACT 0" Enter
echo "[$WINDOW] NAV_DLL_ACT reset to 0 (PX4 default) so OFFBOARD arming does not require a GCS link."

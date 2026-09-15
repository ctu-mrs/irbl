#!/bin/bash
# Starts an extra MAVLink UDP link from one PX4 SITL instance straight to its own lazypx4 pane.
#
# This is deliberately separate from PX4's own default "GCS link" (which every instance already
# starts on boot, unicast to 127.0.0.1:14550 -- see ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink):
# with 3 UAVs all unicasting to that same port, a single listener there (QGroundControl, or one
# lazypx4) sees every vehicle mixed together on one socket, distinguished only by sysid. Instead,
# each lazypx4 pane gets its own dedicated link on its own port, so it only ever sees its own
# vehicle. -x enables MAVLink FTP, which lazypx4 needs for its log-download and parameter screens.
#
# Usage: ./start_lazypx4_link.sh <tmux-window.pane> <px4-local-port> <lazypx4-port>

export TMUX_SOCKET_NAME=px4sim3
export TMUX_SESSION_NAME=px4_simulation_3uav

TARGET="${1:?usage: $0 <tmux-window.pane> <px4-local-port> <lazypx4-port>}"
PX4_LOCAL_PORT="${2:?usage: $0 <tmux-window.pane> <px4-local-port> <lazypx4-port>}"
LAZYPX4_PORT="${3:?usage: $0 <tmux-window.pane> <px4-local-port> <lazypx4-port>}"

until tmux -L $TMUX_SOCKET_NAME capture-pane -p -t $TMUX_SESSION_NAME:$TARGET 2>/dev/null | grep -q "pxh>"; do
  sleep 1
done

tmux -L $TMUX_SOCKET_NAME send-keys -t $TMUX_SESSION_NAME:$TARGET \
  "mavlink start -x -u $PX4_LOCAL_PORT -r 4000000 -o $LAZYPX4_PORT -t 127.0.0.1" Enter
echo "[$TARGET] extra MAVLink link -> 127.0.0.1:$LAZYPX4_PORT for lazypx4"

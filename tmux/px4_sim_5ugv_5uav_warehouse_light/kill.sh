#!/bin/bash
# Stops the session: every process started from its panes (the WHOLE tree below each pane: SIGINT,
# then SIGTERM, then SIGKILL), then whatever is left by name. With 5 pairs a leftover set of nodes
# from an earlier run (e.g. 15 aruco nodes, 30 `ros2 topic echo`) is enough to halve gz's real-time factor.
SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"
. ./env.sh

tree() { local c; for c in $(pgrep -P "$1"); do tree "$c"; done; echo "$1"; }
PIDS=$(tmux -L $TMUX_SOCKET_NAME list-panes -s -t $TMUX_SESSION_NAME -F "#{pane_pid}" 2>/dev/null |
       while read -r p; do for c in $(pgrep -P "$p"); do tree "$c"; done; done)
if [ -n "$PIDS" ]; then
  kill -INT $PIDS 2>/dev/null; sleep 2
  kill -TERM $PIDS 2>/dev/null; sleep 1
  kill -KILL $PIDS 2>/dev/null
fi
tmux -L $TMUX_SOCKET_NAME kill-session -t $TMUX_SESSION_NAME 2>/dev/null

# by name (a pane closed by hand leaves its processes behind)
for P in MicroXRCEAgent "bin/px4" px4_sitl_default "gz sim" gz-sim px4_bridge_node parameter_bridge \
         static_transform_publisher rbl_controller_container warehouse_inspection_node ugv_uav_mission_node \
         mission_viz_node aruco_landing_target.py "ros2 topic echo /ugv_uav_mission_" "ros2 topic echo /uav" \
         publish_sensor_tf.sh rviz2 foxglove_bridge; do
  pkill -TERM -f "$P" 2>/dev/null
done
# a hung gz server ignores SIGTERM and would be silently reused by the next start
sleep 2
for P in "gz sim" "bin/px4" aruco_landing_target.py rbl_controller_container; do pkill -KILL -f "$P" 2>/dev/null; done

echo "$TMUX_SESSION_NAME stopped"

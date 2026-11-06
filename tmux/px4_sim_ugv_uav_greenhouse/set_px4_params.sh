#!/bin/bash
# ./set_px4_params.sh <tmux pane of that PX4 shell> <yaml>
# Waits for that PX4 instance's "pxh>" prompt, then `param set`s every entry of every group of the
# yaml into it (runtime only, nothing in PX4-Autopilot is edited). E.g.
#   ./set_px4_params.sh px4_sitl.0 config/ugv_px4.yaml
PANE=$1
CONFIG=$2
SOCKET=ghugvuav
SESSION=ugv_uav_greenhouse

until tmux -L $SOCKET capture-pane -p -t $SESSION:$PANE 2>/dev/null | grep -q "pxh>"; do
  sleep 1
done

python3 - "$CONFIG" <<'PYEOF' | while read -r NAME VALUE; do
import sys, yaml
for key, group in (yaml.safe_load(open(sys.argv[1])) or {}).items():
    for name, value in (group or {}).items():
        print(name, value)
PYEOF
  tmux -L $SOCKET send-keys -t $SESSION:$PANE "param set $NAME $VALUE" Enter
  sleep 0.05
done
echo "[$PANE] PX4 parameters applied from $CONFIG"

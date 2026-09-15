#!/bin/bash
# Pushes the PX4 firmware parameters listed in ../config/custom_config.yaml into the given running
# PX4 SITL instance via `param set`, the same runtime-only mechanism fix_arming_healthcheck.sh uses
# for NAV_DLL_ACT. This is a runtime `param set` against the given instance, not a permanent edit of
# any file in this repository or in PX4-Autopilot.
#
# Usage: ./set_px4_params.sh <tmux-window.pane>

export TMUX_SOCKET_NAME=px4sim2
export TMUX_SESSION_NAME=px4_simulation_2uav

WINDOW="${1:?usage: $0 <tmux-window.pane>}"
CONFIG="$(dirname "$0")/config/custom_config.yaml"

until tmux -L $TMUX_SOCKET_NAME capture-pane -p -t $TMUX_SESSION_NAME:$WINDOW 2>/dev/null | grep -q "pxh>"; do
  sleep 1
done

python3 - "$CONFIG" <<'PYEOF' | while read -r NAME VALUE; do
import sys, yaml
with open(sys.argv[1]) as f:
    groups = yaml.safe_load(f)
for group in groups.values():
    for name, value in group.items():
        print(name, value)
PYEOF
  tmux -L $TMUX_SOCKET_NAME send-keys -t $TMUX_SESSION_NAME:$WINDOW "param set $NAME $VALUE" Enter
  sleep 0.05
done

echo "[$WINDOW] custom PX4 parameters applied from $CONFIG"

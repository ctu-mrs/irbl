#!/bin/bash
# Pushes the PX4 firmware parameters listed in ../config/custom_config.yaml into the running PX4
# SITL instance via `param set`, the same runtime-only mechanism fix_arming_healthcheck.sh uses for
# NAV_DLL_ACT. This is a runtime `param set` against the running instance, not a permanent edit of
# any file in this repository or in PX4-Autopilot.

export TMUX_SOCKET_NAME=px4sim
export TMUX_SESSION_NAME=px4_simulation

CONFIG="$(dirname "$0")/config/custom_config.yaml"

until tmux -L $TMUX_SOCKET_NAME capture-pane -p -t $TMUX_SESSION_NAME:px4_sitl 2>/dev/null | grep -q "pxh>"; do
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
  tmux -L $TMUX_SOCKET_NAME send-keys -t $TMUX_SESSION_NAME:px4_sitl "param set $NAME $VALUE" Enter
  sleep 0.05
done

echo "[px4_sitl] custom PX4 parameters applied from $CONFIG"

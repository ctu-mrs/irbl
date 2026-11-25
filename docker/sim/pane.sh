#!/bin/bash
# default-command of the tmux session started by ../tmux.sh: every pane of the scenario's session is a zsh in
# the `sim` container, in the pane's directory (the host's tmux/ path, linked to the copy in the container).
# Variables named in FORWARD_ENV (./stack.env) are passed on when set on the host.

C=${SESSION_NAME:-irbl_sim}-sim-1

if [ "$(docker inspect -f '{{.State.Running}}' "$C" 2>/dev/null)" != true ]; then
  echo "pane.sh: container $C is not running (../tmux.sh <scenario> starts it); host shell:"
  exec bash
fi

ENV=(-e "TERM=${TERM:-xterm-256color}")
for v in TMUX_SOCKET_NAME TMUX_SESSION_NAME $FORWARD_ENV; do
  [ -n "${!v}" ] && ENV+=(-e "$v=${!v}")
done

exec docker exec -it "${ENV[@]}" -w "$PWD" "$C" /entrypoint.sh zsh

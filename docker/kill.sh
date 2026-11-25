#!/bin/bash
# Stop the docker simulation (any scenario) and its tmux session (the scenario's own socket, see tmux.sh).

cd "$(dirname "$(readlink -f "$0")")"

./sim/down.sh
if [ -f sim/.tmux_session ]; then
  read -r SOCKET SESSION < sim/.tmux_session
  tmux -L "$SOCKET" kill-session -t "$SESSION" 2>/dev/null
  rm -f sim/.tmux_session
fi
tmux -L irbl_sim kill-server 2>/dev/null   # session of an older tmux.sh
echo "irbl_sim stopped"

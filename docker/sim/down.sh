#!/bin/bash
# Stop the simulation stack (any scenario).

cd "$(dirname "$(readlink -f "$0")")"

set -a; source ./stack.env; set +a
export TMUX_DIR=none   # required by compose.yaml, unused here

docker compose -p $SESSION_NAME --profile record --profile web down --remove-orphans --timeout 2

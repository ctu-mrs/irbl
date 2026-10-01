#!/bin/bash

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

# optional first argument: world name (cylinder_forest [default] or warehouse)
# (beats launch.PX4_GZ_WORLD in config/custom_config.yaml, see launch_env.sh)
export PX4_GZ_WORLD_CLI=${1:-$PX4_GZ_WORLD}

export TMUX_SESSION_NAME=px4_simulation_3dlidar
export TMUX_SOCKET_NAME=px4sim3dlidar

# start tmuxinator
tmuxinator start -p ./session.yml

# if we are not in tmux
if [ -z $TMUX ]; then

  # just attach to the session
  tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME

# if we are in tmux
else

  # switch to the newly-started session
  tmux detach-client -E "tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME"

fi

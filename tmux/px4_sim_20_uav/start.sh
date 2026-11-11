#!/bin/bash

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

export TMUX_SESSION_NAME=px4_simulation_20uav
export TMUX_SOCKET_NAME=px4sim20

# fresh random spawn positions for this run (goals are drawn later by ./activation.sh)
./gen_positions.py spawn
rm -f config/goals.env

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

#!/bin/bash
SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

export TMUX_SESSION_NAME=ugv_uav_greenhouse
export TMUX_SOCKET_NAME=ghugvuav

tmuxinator start -p ./session.yml

if [ -z $TMUX ]; then
  tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME
else
  tmux detach-client -E "tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME"
fi

#!/bin/bash
SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

. ./env.sh

tmuxinator start -p ./session.yml

if [ -z $TMUX ]; then
  tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME
else
  tmux detach-client -E "tmux -L $TMUX_SOCKET_NAME a -t $TMUX_SESSION_NAME"
fi

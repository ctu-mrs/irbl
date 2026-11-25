#!/bin/bash
# The `sim` container: prepares it for the panes of the scenario's tmux session (../pane.sh docker-execs a zsh
# per pane), then idles. Settings from stack.env + scenarios/<scenario>.env (exported by up.sh).

# the host's tmux/ folder -> the copy in the workspace volume, so the host paths tmuxinator sends (cd <root>) work
if [ -n "$HOST_TMUX" ]; then
  mkdir -p "$(dirname "$HOST_TMUX")"
  ln -sfn /etc/docker/colcon_workspace/src/irbl/tmux "$HOST_TMUX"
fi

# tmux (root) wants its own /tmp/tmux-0; the session's socket in it links to the host's (/tmp/host-tmux)
mkdir -p -m 700 /tmp/tmux-0
ln -sfn "/tmp/host-tmux/$TMUX_SOCKET" "/tmp/tmux-0/$TMUX_SOCKET"

touch /root/.zshrc   # no zsh-newuser-install prompt in the panes

# WEB_VIZ: no host display, Gazebo's sensor rendering gets an Xvfb one (DISPLAY=:99 set by up.sh)
[ -n "$WEB_VIZ" ] && Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp > /dev/null 2>&1 &

exec sleep infinity

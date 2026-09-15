#!/bin/bash
# This machine's PX4-Autopilot checkout (and its SITL "instance 0" rootfs/eeprom) is shared
# with other projects. If one of them left NAV_DLL_ACT non-zero, PX4's commander refuses to
# arm without a GCS/MAVLink heartbeat -- which we don't have here, since this stack talks to
# PX4 purely over uXRCE-DDS. This resets NAV_DLL_ACT to PX4's own factory default (0) for the
# running SITL instance so px4_bridge_node's auto-arm sequence can succeed.
#
# This is a runtime `param set`, not a permanent edit of any file in this repository. Note
# that PX4 autosaves parameter changes, so this can persist into the shared eeprom -- if
# another project on this machine relies on NAV_DLL_ACT being non-zero, re-set it there.

export TMUX_SOCKET_NAME=px4sim
export TMUX_SESSION_NAME=px4_simulation

until tmux -L $TMUX_SOCKET_NAME capture-pane -p -t $TMUX_SESSION_NAME:px4_sitl 2>/dev/null | grep -q "pxh>"; do
  sleep 1
done

tmux -L $TMUX_SOCKET_NAME send-keys -t $TMUX_SESSION_NAME:px4_sitl "param set NAV_DLL_ACT 0" Enter
echo "NAV_DLL_ACT reset to 0 (PX4 default) so OFFBOARD arming does not require a GCS link."

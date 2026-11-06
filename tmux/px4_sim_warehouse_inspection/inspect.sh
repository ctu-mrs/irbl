#!/bin/bash
# Sends the UAV to an aisle position, scans the shelf there, then brings it back to where it was.
#
#   ./inspect.sh <x> <y> [left|right] [z]   aisle position in the control frame (uav1/world_origin)
#                                           + which rack face of that aisle segment to scan
#                                           (left = +y rack row, right = -y rack row; default left).
#                                           z = altitude of the aisle position (default: first
#                                           coverage level in config/warehouse_map.yaml)
#   ./inspect.sh abort                      park the UAV where it is
#
# Examples (aisle centrelines y = 0, +-4.2, +-8.4, +-12.6; segments x -33..-18, -15..-1.5,
# 1.5..15, 18..33 -- see config/warehouse_map.yaml):
#   ./inspect.sh -28 8.4 left       aisle y=8.4, far -x segment, shelf on its +y side
#   ./inspect.sh 10 -4.2 right      aisle y=-4.2, segment x 1.5..15, shelf on its -y side
cd "$(dirname "$(readlink -f "$0")")"
# same ROS_DOMAIN_ID etc. as the session, also from a shell that didn't come from session.yml
. ./launch_env.sh
UAV_NAME=${UAV_NAME:-uav1}
NODE=/$UAV_NAME/warehouse_inspection

case "$1" in
  ""|-h|--help)
    sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
    ;;
  abort)
    ros2 service call $NODE/abort std_srvs/srv/Trigger
    exit $?
    ;;
esac

X=$1
Y=${2:?usage: ./inspect.sh <x> <y> [left|right] [z]}
case "${3:-left}" in
  left|l)  SIDE=1.0 ;;
  right|r) SIDE=-1.0 ;;
  *) echo "side must be left or right" >&2; exit 1 ;;
esac
Z=${4:-0.0}

echo "inspect: aisle position [$X, $Y, $Z], ${3:-left} shelf"
ros2 service call $NODE/inspect rbl_msgs/srv/Vec4 "{goal: [$X, $Y, $Z, $SIDE]}"

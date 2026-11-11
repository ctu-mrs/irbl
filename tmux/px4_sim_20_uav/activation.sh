#!/bin/bash
# Sends every UAV its goal (config/goals.env) and activates all controllers, in parallel.
#   ./activation.sh          -> use the goals already in config/goals.env
#   ./activation.sh --new    -> draw a fresh random set of goals first, then send them
# Re-running it while flying just retargets everyone (e.g. ./activation.sh --new for another round).
cd "$(dirname "$(readlink -f "$0")")"
N=20

if [ "$1" == "--new" ] || [ ! -f config/goals.env ]; then
  ./gen_positions.py goals
fi
source config/goals.env

for i in $(seq 1 $N); do
  gx="UAV${i}_GOAL_X"; gy="UAV${i}_GOAL_Y"; gz="UAV${i}_GOAL_Z"; gyaw="UAV${i}_GOAL_YAW"
  ros2 service call /uav$i/rbl_controller/goto rbl_msgs/srv/Vec4 \
    "{goal: [${!gx}, ${!gy}, ${!gz}, ${!gyaw}]}" > /dev/null &
done
wait
echo "goals sent"

for i in $(seq 1 $N); do
  ros2 service call /uav$i/rbl_controller/activation std_srvs/srv/Trigger > /dev/null &
done
wait
echo "all $N controllers activated"

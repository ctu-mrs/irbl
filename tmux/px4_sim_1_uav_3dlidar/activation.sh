#!/bin/bash
# Goal depends on the world the session was started with (PX4_GZ_WORLD, see start.sh).
# Generated worlds (worlds/generate.py) record their goal in worlds/world_goals.yaml; any other world
# (e.g. PX4's own cylinder_forest) uses the default below.
cd "$(dirname "$(readlink -f "$0")")"
# Same ROS_DOMAIN_ID etc. as the session, also from a shell that didn't come from session.yml (a new
# tmux window, `docker exec`, ...) -- otherwise the service calls below wait forever on domain 0.
# A world already in the environment (./start.sh <world>) still beats the yaml's.
export PX4_GZ_WORLD_CLI="${PX4_GZ_WORLD_CLI:-$PX4_GZ_WORLD}"
. ./launch_env.sh
WORLD="${PX4_GZ_WORLD:-cylinder_forest}"

# The 'cylinder_forest' world spawns the vehicle at the origin with 150 randomly-placed tree
# obstacles densely packed across roughly x in [-40,40], y in [-15,15] (none within 2m of the
# spawn point, trunks as close as ~0.25m edge-to-edge elsewhere). This goal sends it most of the
# way down the x axis through that whole field, so the 3D lidar has to actually detect and
# repeatedly avoid obstacles over a long run, not just clip the odd one near the start.
GOAL="[-50, 0, 2.5, 0.0]"

FROM_YAML=$(python3 - "$WORLD" worlds/world_goals.yaml <<'PYEOF'
import sys, yaml
try:
    goal = (yaml.safe_load(open(sys.argv[2])) or {}).get(sys.argv[1])
except OSError:
    goal = None
if goal:
    print("[" + ", ".join(str(v) for v in goal) + "]")
PYEOF
)
[ -n "$FROM_YAML" ] && GOAL="$FROM_YAML"
echo "world '$WORLD' -> goal $GOAL"

ros2 service call /uav1/rbl_controller/goto rbl_msgs/srv/Vec4 "{goal: $GOAL}"

ros2 service call /uav1/rbl_controller/activation std_srvs/srv/Trigger

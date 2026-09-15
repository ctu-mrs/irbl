#!/bin/bash
# The 'cylinder_forest' world spawns the vehicle at the origin with 45 randomly-placed tree
# obstacles densely packed across roughly x in [-14,14], y in [-12.5,12.5] (none within 2m of the
# spawn point, trunks as close as ~0.25m edge-to-edge elsewhere). This goal sends it straight down
# the x axis through that whole field -- about 9 trees sit directly in that corridor -- so the 3D
# lidar has to actually detect and repeatedly avoid obstacles, not just clip the odd one at the edges.
ros2 service call /uav1/rbl_controller/goto rbl_msgs/srv/Vec4 "{goal: [-20, 1, 2.5, 0.0]}"

ros2 service call /uav1/rbl_controller/activation std_srvs/srv/Trigger

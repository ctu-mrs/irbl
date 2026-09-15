#!/bin/bash
# The 'cylinder_forest' world spawns the vehicle at the origin with 150 randomly-placed tree
# obstacles densely packed across roughly x in [-40,40], y in [-15,15] (none within 2m of the
# spawn point, trunks as close as ~0.25m edge-to-edge elsewhere). This goal sends it most of the
# way down the x axis through that whole field, so the 3D lidar has to actually detect and
# repeatedly avoid obstacles over a long run, not just clip the odd one near the start.
ros2 service call /uav1/rbl_controller/goto rbl_msgs/srv/Vec4 "{goal: [-45, 4, 2.5, 0.0]}"

ros2 service call /uav1/rbl_controller/activation std_srvs/srv/Trigger

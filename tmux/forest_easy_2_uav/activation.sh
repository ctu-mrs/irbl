#!/bin/bash
# assign goal
ros2 service call /$UAV_NAME/rbl_controller/goto mrs_msgs/srv/Vec4 "{goal: [10, -0.0, 2.5, 0.0]}"
# trigger activation
ros2 service call /$UAV_NAME/rbl_controller/activation std_srvs/srv/Trigger

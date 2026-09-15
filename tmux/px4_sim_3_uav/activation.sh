#!/bin/bash
# Runs against whichever UAV this pane belongs to ($UAV_NAME, $GOAL_X/Y/Z/YAW -- set per-pane in
# the "activation" window's setup lines in session.yml). With tmux's synchronize-panes on, typing
# ./activation.sh once in any of the three panes runs it in all three at the same keystroke, so
# every UAV's goto+activation fires together instead of one after another.
ros2 service call /$UAV_NAME/rbl_controller/goto rbl_msgs/srv/Vec4 "{goal: [$GOAL_X, $GOAL_Y, $GOAL_Z, $GOAL_YAW]}"
ros2 service call /$UAV_NAME/rbl_controller/activation std_srvs/srv/Trigger

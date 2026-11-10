#!/bin/sh
# Shared environment of px4_sim_2ugv_2uav_warehouse_stereo: TWO UGV + UAV pairs of
# ../px4_sim_ugv_uav_warehouse_stereo (stereo depth cameras) in one warehouse. Sourced by every pane
# (session.yml pre_window) and by deploy.sh. POSIX sh (bash or zsh panes).
#
#   PX4 instance:  0 ugv1 (hosts the gz world)   1 uav1 (on ugv1's deck)
#                  2 ugv2                         3 uav2 (on ugv2's deck)
#   MAV_SYS_ID = instance + 1, PX4 uXRCE-DDS namespace ugv_<k> / uav_<k>, gz model
#   <vehicle>_<model>_<instance>, onboard MAVLink port 14580 + instance (aruco_landing_target.py).
# Every vehicle's EKF is relative to its own spawn point: px4_bridge adds home_offset_x/y/z (= the
# spawn) so all four share the frame "world" (= gz world).

export ROS_DOMAIN_ID=${UGV_UAV_2PAIRS_DOMAIN_ID:-50}
export PX4_DIR=${PX4_DIR:-$HOME/git/PX4-Autopilot}
export PX4_GZ_WORLD=${PX4_GZ_WORLD:-warehouse}
# HEADLESS=1 ./start.sh: no gz GUI (PX4 starts it only when HEADLESS is EMPTY, so unset unless 1)
if [ "${HEADLESS:-0}" = "1" ]; then export HEADLESS=1; else unset HEADLESS; fi
export USE_SIM_TIME=true
export PLANNER=${PLANNER:-global}     # global | local, see ../px4_sim_ugv_uav_warehouse/env.sh
export CONTROL_FRAME=world

# heights as in ../px4_sim_ugv_uav_warehouse/env.sh (why the UGV's z is pinned at 0.6 m there)
export UGV_SPAWN_Z=0.05
export UGV_HOME_Z=0.6
export UAV_SPAWN_Z=0.32               # x500 skids on the deck
export UAV_HOME_Z=0.56

# pair 1: spawns at the world origin (cross-aisle x=0 / row 4, y=0)
export UGV1_X=0.0 UGV1_Y=0.0
# pair 2: next intersection, cross-aisle x=0 / row 3 (y=-4.2), nothing within 2 m
export UGV2_X=0.0 UGV2_Y=-4.2

#!/bin/sh
# Shared environment of px4_sim_ugv_uav_warehouse: sourced by every pane (session.yml pre_window)
# and by deploy.sh. POSIX sh (bash or zsh panes).

export ROS_DOMAIN_ID=${UGV_UAV_DOMAIN_ID:-48}  # ROS 2 domain of every node AND of both PX4 instances (rcS copies
                                      # it into UXRCE_DDS_DOM_ID)
export PX4_DIR=${PX4_DIR:-$HOME/git/PX4-Autopilot}
export PX4_GZ_WORLD=${PX4_GZ_WORLD:-warehouse}
# HEADLESS=1 ./start.sh: gz sim without its GUI window. PX4 (px4-rc.gzsim) starts the GUI only when
# HEADLESS is EMPTY -- any value, "0" included, suppresses it -- so it is unset unless it is 1.
if [ "${HEADLESS:-0}" = "1" ]; then export HEADLESS=1; else unset HEADLESS; fi
export USE_SIM_TIME=true
export PLANNER=${PLANNER:-global}     # global: UGV legs planned once on the known map (A*), rbl_controller follows
                                      # them without its local replanner; local: rbl's local replanner
                                      # (config/warehouse_map.yaml planner.mode explains)
export CONTROL_FRAME=world            # shared frame of both vehicles = gz world (UGV spawns at its origin)

# UGV: PX4 instance 0 (hosts the gz world), differential rover, PX4 topics /ugv_1/fmu/...
export UGV_NAME=ugv1
export UGV_INSTANCE=0
export UGV_FMU_NS=ugv_1               # PX4 uXRCE-DDS namespace (PX4_UXRCE_DDS_NS) = px4_bridge px4_ns
export UGV_TARGET_SYSTEM=1            # MAV_SYS_ID = instance + 1
export UGV_GZ_MODEL=r1_rover_carrier_0
export UGV_SPAWN="0,0,0.05,0,0,0"     # x,y,z,roll,pitch,yaw (gz world)
# The UGV runs the same 3D rbl_controller as the UAV; its reference z is simply ignored by PX4's
# rover. Its odometry z is pinned (px4_bridge ground_vehicle) at this fixed height, because rbl
# treats everything more than `encumbrance` (0.45 m) below the vehicle as floor and drops it: at the
# rover's real height (~0.05 m, + EKF drift) the floor itself would be an obstacle. At 0.6 m the cut
# is at 0.15 m. config/ugv_sensors.yaml mounts the lidar relative to this height, so the cloud keeps
# its real heights above the floor. In rviz the UGV's TF frame therefore sits 0.6 m above the floor
# (its body is drawn on the floor).
export UGV_HOME_Z=0.6

# UAV: PX4 instance 1 (joins the world standalone), x500 + 3D lidar + down camera (precision landing on
# the ArUco marker of the UGV deck), PX4 topics /uav_1/fmu/...
export UAV_NAME=uav1
export UAV_INSTANCE=1
export UAV_TARGET_SYSTEM=2
export UAV_FMU_NS=uav_1               # PX4 uXRCE-DDS namespace (PX4_UXRCE_DDS_NS) = px4_bridge px4_ns
export UAV_GZ_MODEL=x500_lidar_3d_cam_down_1  # must match <child_model> of the DetachableJoint in
                                      # models/r1_rover_carrier/model.sdf
# On the UGV deck (top at ~0.322 m): the x500 model frame is 0.24 m below its base_link and the
# skids 0.227 m below base_link -> model z 0.32 puts the skids on the deck.
export UAV_SPAWN="0,0,0.32,0,0,0"
export UAV_HOME_Z=0.56                # base_link height on the deck when PX4 boots
export UAV_MAVLINK_PORT=14581          # PX4 SITL onboard MAVLink link of instance 1 (14580 + instance):
                                      # aruco_landing_target.py sends TARGET_RELATIVE there

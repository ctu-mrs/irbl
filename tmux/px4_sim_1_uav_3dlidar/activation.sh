#!/bin/bash
# Goal depends on the world the session was started with (PX4_GZ_WORLD, see start.sh).
case "${PX4_GZ_WORLD:-cylinder_forest}" in
  warehouse)
    # Warehouse (worlds/generate_warehouse.py): spawn is in the y=0 main aisle between rack rows,
    # in the x=0 cross-aisle. This goal sits in the y=8.4 aisle near the far -x end, so the UAV
    # has to leave the main aisle through a cross-aisle (x=-16.5), turn into a side aisle, and
    # pass shelves/pallets/columns on the way, rather than just flying straight down one aisle.
    GOAL="[-28, 8.4, 2.5, 0.0]"
    ;;
  *)
    # The 'cylinder_forest' world spawns the vehicle at the origin with 150 randomly-placed tree
    # obstacles densely packed across roughly x in [-40,40], y in [-15,15] (none within 2m of the
    # spawn point, trunks as close as ~0.25m edge-to-edge elsewhere). This goal sends it most of the
    # way down the x axis through that whole field, so the 3D lidar has to actually detect and
    # repeatedly avoid obstacles over a long run, not just clip the odd one near the start.
    GOAL="[-50, 0, 2.5, 0.0]"
    ;;
esac

ros2 service call /uav1/rbl_controller/goto rbl_msgs/srv/Vec4 "{goal: $GOAL}"

ros2 service call /uav1/rbl_controller/activation std_srvs/srv/Trigger

# px4_sim_warehouse_inspection

Single UAV in `worlds/warehouse.sdf` (PX4 SITL + Gazebo + simulated 3D lidar, same stack as
`px4_sim_1_uav_3dlidar`), with an inspection mission on top of the **unchanged** rbl_controller:

1. **GOTO** – fly to the requested aisle position
2. **COVERAGE** – boustrophedon scan of the shelf (rack face) on the requested side of that aisle
   segment: one pass along the shelf per height in `coverage.levels`
3. **RETURN** – back through the aisle position to where the UAV was before the request

The mission is `ros_packages/rbl_mission` (`warehouse_inspection_node`, C++). It only sends
`rbl_controller/goto` goals one at a time and waits on `odometry` for arrival; it knows the
warehouse layout from `config/warehouse_map.yaml` (mirrors `worlds/worldgen/worlds/warehouse.py`).

```bash
colcon build --packages-select rbl_msgs rbl_mission   # once, from the workspace root
./start.sh                       # opens on the `mission` window
# wait for 'Arming' in the px4_bridge window (auto takeoff to 2 m), then:
./inspect.sh -28 8.4 left        # already in the shell history of the mission window
./inspect.sh abort               # park where it is
./kill.sh
```

Mission window panes: the node, its state (`/uav1/warehouse_inspection/state`), and the operator
shell. rviz (`inspection.rviz`) also shows the planned coverage path and the current goal.

## Coverage planner

`rbl_mission/include/rbl_mission/coverage_planner.hpp` (shared by the node and the checker):
each pass is sampled every 0.25 m against every collision shape of the world; blocked stretches
get one consistent detour (sideways inside the lane band, or over a pallet), legs that are still
not clear (approach, level changes, leaving) are routed with A* inside the aisle, and nothing goes
above `coverage.z_max` (3.8 m, below the 4.5 m racks; rbl_controller's `z_max` is 4.0 m here too),
so the UAV never flies over a shelf. Check every shelf offline (prints a table, exit 1 on failure):

```bash
ros2 run rbl_mission coverage_plan_check --ros-args --params-file config/warehouse_map.yaml \
  -p csv:=/tmp/plans.csv      # optional: every goal + sample, for plotting
```

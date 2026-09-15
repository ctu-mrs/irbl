## 🎥 Multimedia material

![Real-world Experiment (4 UAVs)](forest4.gif)

(speed >>1x)

- https://mrs.fel.cvut.cz/irbl


## 🚀 Prerequisites

  Ensure you have the following installed:

  - ** MRS System **  
    Follow setup instructions here for ROS2: [MRS Apptainer GitHub](https://github.com/ctu-mrs/mrs_apptainer)

  ---

## 🛠 Prepare the Workspace


  ```bash
  cd ~/git/mrs_apptainer/user_ros_workspace/src
  git clone git@github.com:ctu-mrs/irbl.git
  git clone git@github.com:MichalKamler/filter_reflective_uavs.git -b ros2
  cd ../../
  ./example_wrapper
  cd user_ros_workspace
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

 ## ▶️ run simulation 

remember to source the workspace

```bash  
cd src/irbl/tmux/forest_easy_2_uav
./start.sh
```

wait for takeoff and then run ./activation.sh


Notice that the results reported in the following paper were obtained from the ROS1 (branch irbl) version of the package 

- [Perception-Aware Communication-Free Multi-UAV Coordination in the Wild](https://arxiv.org/abs/2603.08379)

## 🛩️ PX4-Autopilot simulation (no MRS system/library)

`ros_packages/nodelet` and `ros_packages/rbl_controller_core` do not depend on `mrs_lib` or the
MRS flight stack (`mrs_uav_core`, `mrs_multirotor_simulator`, `mrs_uav_gazebo_simulator`,
`mrs_uav_autostart`/`status`, `hw_api`) at all; `mrs_msgs` is kept only as a plain
message-definition dependency (e.g. the `Reference`/`Vec4` types), not a runtime one. A new
`ros_packages/rbl_px4_bridge` package talks to PX4 natively over uXRCE-DDS (`px4_msgs`),
converting PX4's `vehicle_odometry` into the `nav_msgs/Odometry` the controller expects, and the
controller's reference output into PX4 `TrajectorySetpoint`/`OffboardControlMode`, handling
arm + OFFBOARD switching itself.

### Extra prerequisites

- A built [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) checkout with Gazebo (`gz`) SITL support (`make px4_sitl gz_x500` must work on its own first).
- [`px4_msgs`](https://github.com/PX4/px4_msgs) and [`px4_ros_com`](https://github.com/PX4/px4_ros_com), cloned next to `irbl` under `src/` (same convention as `filter_reflective_uavs`) so they build into the same workspace.
- The [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent) (`MicroXRCEAgent`) on `PATH`.

### Run it (single UAV)

```bash
cd src/irbl/tmux/px4_sim_1_uav
PX4_DIR=/path/to/PX4-Autopilot ./start.sh
```

This brings up the uXRCE-DDS agent, PX4 SITL (Gazebo, `x500`), the `rbl_px4_bridge` node, a live
random-forest obstacle cloud (`map_generator`), the refactored `rbl_controller_node`, and rviz.
The bridge auto-arms and climbs to a 2 m hover once PX4 is streaming odometry; once you see
`Arming` in the `px4_bridge` pane, run `./activation.sh` to set a goal and activate RBL.

Note: this PX4-Autopilot checkout may be shared with other projects on your machine. If PX4
refuses to arm with `Preflight Fail: No connection to the GCS`, that means `NAV_DLL_ACT` was left
non-zero by another project's SITL run; `fix_arming_healthcheck.sh` (wired into the tmux session)
resets it back to PX4's own factory default (`0`) automatically.

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

# UFEP_Released
Unlocking Full Exploration Potential of Ground Robots by Multi-Resolution Topological Mapping.

All the source code are here. However, there might be some third-libraries used but not listed here. Users can gradually configure based on reminders of compiler.

## Dependencies
These instructions assume that ROS desktop-full of the appropriate ROS distro is installed.

Install necessary libraries:

- [pplanner_simulator](https://github.com/YinghaoJia98/pplanner_simulator.git). The branch SimulatorForUFEP-Released. When compiling it, some libraries used by the controller of UnitreeA1 might need to be configured mannually and most of them could be find in my repositories, such as [OSQP](https://github.com/YinghaoJia98/OSQP0.6.2ForUnitreeMPC.git) and [OSQP_EIGEN](https://github.com/YinghaoJia98/OSQP_EIGEN0.6.3ForUnitreeMPC.git).
```bash
git clone git@github.com:YinghaoJia98/pplanner_simulator.git -b SimulatorForUFEP-Released
```

- [grid_map](https://github.com/ANYbotics/grid_map) (grid map library for mobile robots)
```bash
sudo apt-get install ros-noetic-grid-map-core ros-noetic-grid-map-msgs
```
- [elevation_mapping_cupy](https://github.com/YinghaoJia98/elevation_mapping_cupy.git) The EleForUFEP-Released branch is used in my experiment and the original [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy.git) also can be used. If the pybind installed by 'apt' do not work, please install it manually. The pybind11 and pybind11_catkin used by us could be found in my repositories.
```bash
git clone git@github.com:YinghaoJia98/elevation_mapping_cupy.git -b EleForUFEP-Released
```

- [vrmapping_msgs](https://github.com/YinghaoJia98/vrmapping_msgs.git) The msg and srv files which would be in cluded into this pack in the future.
```bash
git clone git@github.com:YinghaoJia98/vrmapping_msgs.git
```

- [vrmapping_ui](https://github.com/YinghaoJia98/vrmapping_ui.git) The ui files which would be in cluded into this pack in the future.
```bash
git clone git@github.com:YinghaoJia98/vrmapping_ui.git
```

## Build
```bash
mkdir -p mmapping_ws/src/
git clone git@github.com:YinghaoJia98/UFEP_Released.git
catkin build -DCMAKE_BUILD_TYPE=Release
```

## Usage with SuperMegaBot
```bash
source devel/setup.bash
roslaunch pplanner_simulator VRP_Smb.launch
roslaunch vrmapping vrmapping_nogdb.launch
```

## Usgae with UnitreeA1
```bash
source devel/setup.bash
roslaunch pplanner_simulator VRP_UnitreeA1.launch
roslaunch vrmapping vrmapping_UnitreeA1_nogdb.launch
```

## Experiments Videos
[![UFEP_video](img/VideoFront.png)](https://youtu.be/i0CMJkWDkV0)

## Thanks
The robots used in simulator is reorganized from [GBP](https://github.com/ntnu-arl/gbplanner_ros.git) and [UnitreeA1](https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller.git).
Benifiting from the robotics community and we choose to make our work public.

## Contact us
* [Yinghao Jia](mailto:yinghaojia@163.com)
* Wei Tang
## featureless_and_high_dynamic_navigation
This package provides localization by using different odometry sources, combined with the "ekf_localization_node" node from the "robot_localization" package.

# Dependencies
For the purpose of research, the "ekf_localization_node" is applied from the "robot_localization"

- robot_localization

```shell
$ cd your_workspace/src
$ git clone https://github.com/cra-ros-pkg/robot_localization
$ cd ..
$ catkin_make
```

- poyzpy
More info here: https://www.pozyx.io/Documentation/Datasheet/python
```shell
$ pip install pypozyx
```

- TinkerForge Brick 2.0
For daemon instructions: https://www.tinkerforge.com/en/doc/Software/Brickd_Install_Linux.html#brickd-install-linux
Python library:
```shell
$ pip install tinkerforge
```

- RTAB-Map ROS
More info here: https://github.com/introlab/rtabmap_ros
```shell
$ sudo apt-get install ros-kinetic-rtabmap-ros
```

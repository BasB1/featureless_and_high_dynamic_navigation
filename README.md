# featureless_and_high_dynamic_navigation
This package provides localization by using different odometry sources, combined with the "ekf_localization_node" node from the "robot_localization" package.

## Dependencies
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


## Trilateration
This package contains a node that does the filtered trilateration with the Pozyx system. Using this trilateration method it is possible to improve accuracy with 60% in combination with an EKF and IMU. More is elaborated in the research paper that is available under the folder 'paper'.

### Kalman Filter
The distance measurements are performed with an one dimensional KF to filter out the oresent gaussian noise. Normally, the distance measurements are feeded directly to an EKF model for filtered localization. By taking the filtering one step higher before feeding the information to the EKF, a better result can be achieved. 

# inekf
This repository contains a C++ library that implements an invariant extended Kalman filter (InEKF) for 3D aided inertial navigation. 

[![InEKF LiDAR Mapping](https://i.imgur.com/BwtIepo.jpg)](https://www.youtube.com/watch?v=pNyXsZ5zVZk)

This filter can be used to estimate a robot's 3D pose and velocity using an IMU motion model for propagation. The following measurements are currently supported:
* Prior landmark position measurements (localization)
* Estiamted landmark position measurements (SLAM)
* Kinematic and contact measurements

The core theory was developed by Barrau and Bonnabel and is presented in:
["The Invariant Extended Kalman filter as a Stable Observer"](https://arxiv.org/abs/1410.1465).

Inclusion of kinematic and contact measurements is presented in:
["Contact-aided Invariant Extended Kalman Filtering for Legged Robot State Estimation"](https://arxiv.org/pdf/1805.10410.pdf).

A ROS wrapper for the filter is available at [https://github.com/RossHartley/invariant-ekf-ros](https://github.com/RossHartley/invariant-ekf-ros).

## Setup
### Requirements
* CMake 2.8.3 or later
* g++ 5.4.0 or later
* [Eigen3 C++ Library](http://eigen.tuxfamily.org/index.php?title=Main_Page)

### Installation Using CMake
```
mkdir build
cd build 
cmake .. 
make
``` 
invariant-ekf can be easily included in your cmake project by adding the following to your CMakeLists.txt:
```
find_package(inekf) 
include_directories(${inekf_INCLUDE_DIRS})
```

## Examples
1. A landmark-aided inertial navigation example is provided at `src/examples/landmarks.cpp`
2. A contact-aided inertial navigation example is provided at `src/examples/kinematics.cpp`

## Citations
The contact-aided invariant extended Kalman filter is described in: 
* R. Hartley, M. G. Jadidi, J. Grizzle, and R. M. Eustice, “Contact-aided invariant extended kalman filtering for legged robot state estimation,” in Proceedings of Robotics: Science and Systems, Pittsburgh, Pennsylvania, June 2018.
```
@INPROCEEDINGS{Hartley-RSS-18, 
    AUTHOR    = {Ross Hartley AND Maani Ghaffari Jadidi AND Jessy Grizzle AND Ryan M Eustice}, 
    TITLE     = {Contact-Aided Invariant Extended Kalman Filtering for Legged Robot State Estimation}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2018}, 
    ADDRESS   = {Pittsburgh, Pennsylvania}, 
    MONTH     = {June}, 
    DOI       = {10.15607/RSS.2018.XIV.050} 
} 
```
The core theory of invariant extended Kalman filtering is presented in:
* Barrau, Axel, and Silvère Bonnabel. "The invariant extended Kalman filter as a stable observer." IEEE Transactions on Automatic Control 62.4 (2017): 1797-1812.
```
@article{barrau2017invariant,
  title={The invariant extended Kalman filter as a stable observer},
  author={Barrau, Axel and Bonnabel, Silv{\`e}re},
  journal={IEEE Transactions on Automatic Control},
  volume={62},
  number={4},
  pages={1797--1812},
  year={2017},
  publisher={IEEE}
}
```

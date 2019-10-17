# README
 The code is open-source (BSD License). Please note that this work is an on-going research and thus some parts are not fully developed yet. Furthermore, the code will be subject to changes in the future which could include greater re-factoring.

* nao_walk_naoqi: A C++ naoqi module that achieves real-time omni-directional walking. This module needs to be cross-compiles the cross naoqi sdk and uploaded to NAO.

* nao_walk_ros: A ROS/C++ wrapper that enables real-time communication with NAO without relying on naoqi-sdk.
<img src="http://users.ics.forth.gr/~spiperakis/hill_nao.gif?raw=true" width="200px">
<img src="http://users.ics.forth.gr/~spiperakis/grass_nao.gif?raw=true" width="200px">

## Prerequisites
* Ubuntu 16.04 and later
* ROS kinetic and later
* Eigen 3.2.0 and later

## Instalation
* git clone https://github.com/mrsp/nao_walk.git in a ROS workspace.

## naoqi walk module
* cd nao_walk/nao_walk_naoqi && qibuild init
* qibuild configure --release -c cross-naoqi-sdk
* qibuild make -c cross-naoqi-sdk
* This will generate a naoqi module named "libnao_walkF.so"  in:
  nao_walk/nao_walk_naoqi/build-cross-naoqi-sdk/sdk/lib/naoqi
* scp libnao_walkF.so nao@nao.local:/home/nao/modules to copy the module onto NAO.

## nao walk ROS wrapper
* catkin_make 
* roslaunch nao_walk gait_control.launch
* roslaunch teleop_twist_keyboard velocity_cmd.launch (To make the robot walk with the keyboard)

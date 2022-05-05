# Webots ROS2 Spot

This is a ROS 2 package to simulate the Boston Dynamics spot in [webots](https://cyberbotics.com/). The goal is to control the spot via a teleop.
Therefore, the joint angles for the leg motors must be calculated when a or none command velocity is received.

In this project we try to realize the developed kinematic of [SoftServe](https://www.softserveinc.com/en-us/blog/spot-simulation-tools) for Gazebo under webots.


## Prerequisites

    - Tested for ubuntu 20.04
    - ROS 2 foxy        
    - Webots R2022a
    - Webots ROS 2 interface

[Webots R2022a](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt) can be installed via apt package manager or downloaded [here](https://github.com/cyberbotics/webots/releases).

The webots ros2 interface can also be directly installed via [apt](https://github.com/cyberbotics/webots_ros2/wiki/Getting-Started) and should be working fine, but i would recommend to follow the [Build and Install](https://github.com/cyberbotics/webots_ros2/wiki/Build-and-Install) instructions and install it from sources.
Then you are able to checkout the newest version 1.2.2 of the webots ros2 interface.

## Install 

### webots_ros2_spot
```
# Source ROS 2
source /opt/ros/$ROS_DISTRO/local_setup.bash

cd /path/to/ros2_ws
git clone git@github.com:MASKOR/webots_ros2_spot.git src/webots_spot
```
### spot_teleop
```
git clone git@github.com:MASKOR/webots_spot_teleop.git src/spot_teleop
```
### spot_msgs
```
git clone git@github.com:MASKOR/webots_spot_msgs.git src/spot_msgs

# Build everything
colcon build

source install/local_setup.bash
```
### Python requirements
scipy
## Start
Starting the simulation:

    ros2 launch webots_spot spot_launch.py

Teleop keyboard:

    ros2 run spot_teleop spot_teleop_keyboard

## Demo

![spot_teleop](https://fh-aachen.sciebo.de/s/eJ69q6EPqclEHHB/download)

## Integration notes
The leg kinematic of the spot is ported from the SoftServeSAG [robotics_spot repo](https://github.com/SoftServeSAG/robotics_spot/tree/temp_robomaker).

## Inspiration:

[Spot in AWS Robomaker](https://github.com/SoftServeSAG/robotics_spot/tree/temp_robomaker)
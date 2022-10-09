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
git clone https://github.com/MASKOR/webots_ros2_spot src/webots_spot
```
### spot_teleop
```
git clone https://github.com/MASKOR/webots_spot_teleop src/spot_teleop
```
### spot_msgs
```
git clone https://github.com/MASKOR/webots_spot_msgs src/spot_msgs
```
# Build everything
```
colcon build

source install/local_setup.bash
```
### Install dependencies
```
sudo apt install ros-$ROS_DISTRO-webots* ros-$ROS_DISTRO-nav2* -y
sudo apt install ros-$ROS_DISTRO-pointcloud-to-laserscan -y
sudo apt install ros-$ROS_DISTRO-moveit* -y
sudo apt install python3-open3d -y # for Ubuntu 22.04
```
```
pip3 install scipy pupil-apriltags
pip3 install open3d # for Ubuntu 20.04
```
## Start
Starting the simulation:

    ros2 launch webots_spot spot_launch.py

To launch navigation with Rviz2:

    ros2 launch webots_spot nav_launch.py set_initial_pose:=true

To launch mapping with Slamtoolbox:

    ros2 launch webots_spot slam_launch.py

To launch mapping with RTABMAP: #https://github.com/introlab/rtabmap_ros

    ros2 launch webots_spot rtabmap_launch.py

Teleop keyboard:

    ros2 run spot_teleop spot_teleop_keyboard

## Demo

![spot_teleop](https://fh-aachen.sciebo.de/s/eJ69q6EPqclEHHB/download)

## Integration notes
The leg kinematic of the spot is ported from the SoftServeSAG [robotics_spot repo](https://github.com/SoftServeSAG/robotics_spot/tree/temp_robomaker).

## Inspiration:

[Spot in AWS Robomaker](https://github.com/SoftServeSAG/robotics_spot/tree/temp_robomaker)

# Webots ROS2 Spot

This is a ROS 2 package to simulate the Boston Dynamics spot in [webots](https://cyberbotics.com/). Spot is able to walk around, to sit, standup and lie down. We also attached some sensors on spot, like a kincet and a 3D laser.
The world contains apriltags, a red line to test lane follower and soon objects for manipulation tasks.

![webots_spot_sim](https://fh-aachen.sciebo.de/s/4N8dz67jsxARgdN/download)

## Prerequisites

    - Tested for ubuntu 22.04
    - ROS 2 humble
    - Webots R2022b
    - Webots ROS 2 interface

[Webots R2022b](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt) can be installed via apt package manager or downloaded [here](https://github.com/cyberbotics/webots/releases).

The webots ros2 interface has to be installed from [sources](https://github.com/cyberbotics/webots_ros2/wiki/Complete-Installation-Guide#install-webots_ros2-from-sources).

## Install

### Install dependencies
```
sudo apt install ros-$ROS_DISTRO-webots* ros-$ROS_DISTRO-nav2* -y
sudo apt install ros-$ROS_DISTRO-pointcloud-to-laserscan -y
pip3 install scipy pupil-apriltags
pip3 install open3d # for Ubuntu 20.04
sudo apt install python3-open3d # for Ubuntu 22.04
```

### webots_ros2_spot
```
# Boston Dynamic
pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core

# Source ROS 2
source /opt/ros/$ROS_DISTRO/local_setup.bash

cd /path/to/ros2_ws
git clone https://github.com/MASKOR/webots_ros2_spot src/webots_spot

# spot_msgs
git clone https://github.com/MASKOR/webots_spot_msgs src/spot_msgs

# Build everything
colcon build --symlink-install
source install/local_setup.bash
```

## Start
Starting the simulation:

    ros2 launch webots_spot spot_launch.py

Starting apriltag detection:

    ros2 run webots_spot apriltag_ros

To launch navigation with Rviz2:

    ros2 launch webots_spot nav_launch.py set_initial_pose:=true

To launch mapping with Slamtoolbox:

    ros2 launch webots_spot slam_launch.py

To launch mapping with RTABMAP: #https://github.com/introlab/rtabmap_ros

    ros2 launch webots_spot rtabmap_launch.py

Teleop keyboard:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

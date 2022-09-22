# Webots ROS2 Spot

This is a ROS 2 package to simulate the Boston Dynamics spot in [webots](https://cyberbotics.com/).

## Prerequisites

    - Tested for ubuntu 22.04
    - ROS 2 humble
    - Webots R2022a
    - Webots ROS 2 interface

[Webots R2022a](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt) can be installed via apt package manager or downloaded [here](https://github.com/cyberbotics/webots/releases).

The webots ros2 interface can also be directly installed via [apt](https://github.com/cyberbotics/webots_ros2/wiki/Getting-Started) or from [sources](https://github.com/cyberbotics/webots_ros2/wiki/Build-and-Install).

## Install

### webots_ros2_spot
```
# Source ROS 2
source /opt/ros/$ROS_DISTRO/local_setup.bash

cd /path/to/ros2_ws
git clone https://github.com/MASKOR/webots_ros2_spot src/webots_spot
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
pip3 install scipy pupil-apriltags
pip3 install open3d # for Ubuntu 20.04
sudo apt install python3-open3d # for Ubuntu 22.04
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

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

## Demo

![spot_teleop](https://fh-aachen.sciebo.de/s/eJ69q6EPqclEHHB/download)

## Inspiration:

[Spot in AWS Robomaker](https://github.com/SoftServeSAG/robotics_spot/tree/temp_robomaker)

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

```
# Source ROS 2
source /opt/ros/$ROS_DISTRO/local_setup.bash

cd /path/to/ros2_ws
git clone https://github.com/METEORITENMAX/webots_ros2_spot.git src/webots_spot

colcon build

source install/local_setup.bash
```

## Start
Starting the simulation:

    ros2 launch webots_spot spot_launch.py

Teleop keyboard:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

## Integration notes
The leg kinematic of the spot is ported from the SoftServeSAG [robotics_spot repo](https://github.com/SoftServeSAG/robotics_spot/tree/temp_robomaker). 

The python scripts for the invervse kinematic: `Bezier.py`, `LegKinematics.py`, `LieAlgebra.py` and `SpotKinematics.py` are untouched.
The `env_tester.py` and `gui_spot.py` are integreted in `spot_driver.py`.

I added touch sensors on the feet for all legs to detect contact with the ground used for `GenerateTrajectory()`.

Also, i changed the min/max position for the leg motors inside the `Spot.proto` resp. `SpotRightLeg.proto` and `SpotLeftLeg.proto` to the limitations from SoftServe found in e.g. `SpotKinematics.py`.

To calculate `YawRate_desired` the `self.yaw_inst` is set by reading the rotation field of the robot itself.

Finally the inputs for the Bezier gait control are set by default inside the constructor and not over a GUI.

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
git clone https://github.com/METEORITENMAX/webots_ros2_spot.git src/webots_spot
```
### spot_teleop
```
git clone https://github.com/METEORITENMAX/webots_spot_teleop.git src/spot_teleop
```
### spot_msgs
```
git clone https://github.com/METEORITENMAX/webots_spot_msgs.git src/spot_msgs

# Build everything
colcon build

source install/local_setup.bash
```
## Start
Starting the simulation:

    ros2 launch webots_spot spot_launch.py

Teleop keyboard:

    ros2 run spot_teleop spot_teleop_keyboard

## Integration notes
The leg kinematic of the spot is ported from the SoftServeSAG [robotics_spot repo](https://github.com/SoftServeSAG/robotics_spot/tree/temp_robomaker). 

The python scripts for the invervse kinematic: `Bezier.py`, `LegKinematics.py`, `LieAlgebra.py` and `SpotKinematics.py` are nearly untouched.
Only the height is set to `0.84` instead of `0.6`, according to the original dimensions found from Boston Dynamics [docs](https://dev.bostondynamics.com/docs/concepts/about_spot#dimensions).
At a height of 0.6, the spot stretches its legs and falls to the ground when the simulation starts.

The `env_tester.py` and `gui_spot.py` are integreted in `spot_driver.py`.

I added touch sensors on the feet for all legs to detect contact with the ground used for `GenerateTrajectory()`.

Also, i changed the min/max position for the leg motors inside the `Spot.proto` resp. `SpotRightLeg.proto` and `SpotLeftLeg.proto` to the limitations from SoftServe found in e.g. `SpotKinematics.py`.

To calculate `YawRate_desired` the `self.yaw_inst` is set by reading the rotation field of the robot itself.

The `spot_teleop_keyboard` sends `GaitInput.msg` recevied by the `spot_driver`.
Moving in x,y,z and roll, pitch yaw almost works. When moving in z, the spot does not go completely to its knees.

Also, the teleop allows the movement to overshoot, so the robot falls.
The reason for this could be mismatched motor limitations or parameters for hip, shoulder, elbow in the `SpotKinematics.py` and `LegKinematics.py` or `spot.proto`.

Moving forward is not working at all.

<img src="https://fh-aachen.sciebo.de/s/AcGtrbTN9mxG9OY/download" width="600">

## Inspiration:

[Spot in AWS Robomaker](https://github.com/SoftServeSAG/robotics_spot/tree/temp_robomaker)
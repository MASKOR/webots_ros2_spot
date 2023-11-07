# Webots ROS2 Spot

This is a ROS 2 package to simulate the Boston Dynamics spot in [webots](https://cyberbotics.com/). Spot is able to walk around, to sit, standup and lie down. We also attached some sensors on spot, like a kinect and a 3D laser.
The world contains apriltags, a red line to test lane follower and objects for manipulation tasks.

![spot_UR3](https://fh-aachen.sciebo.de/s/zDZLpVTjPWLzt7x/download)

## Prerequisites

    - Ubuntu 22.04
    - ROS2 Humble https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
    - Webots 2023b https://github.com/cyberbotics/webots/releases/tag/R2023b

## Install

1. Make sure that `colcon`, its extensions, and `vcs` are installed:
    ```
    sudo apt install python3-colcon-common-extensions python3-vcstool
    ```

    Also, rosdep is installed https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html#rosdep-installation.

2. Create a new ROS2 workspace:
    ```
    export COLCON_WS=~/ros2_ws
    mkdir -p $COLCON_WS/src
    ```

3. Pull relevant packages, install dependencies, compile, and source the workspace by using:
    ```
    cd $COLCON_WS
    git clone https://github.com/MASKOR/webots_ros2_spot src/webots_ros2_spot
    vcs import src --skip-existing --input src/webots_ros2_spot/webots_ros2_spot.repos
    rosdep install --ignore-src --from-paths src -y -r
    ```

4. Build packages and source the workspace
    ```
    colcon build --symlink-install
    source install/setup.bash
    ```

## Start
Starting the simulation:
```
ros2 launch webots_spot spot_launch.py
```

To launch navigation with Rviz2:
```
ros2 launch webots_spot nav_launch.py set_initial_pose:=true
```

To launch mapping with Slamtoolbox:
```
ros2 launch webots_spot slam_launch.py
```

Starting MoveIt:
```
ros2 launch webots_spot moveit_launch.py
```

Teleop keyboard:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# OR ros2 run spot_teleop spot_teleop_keyboard for body_pose control as well
```

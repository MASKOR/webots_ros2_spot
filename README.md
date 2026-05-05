# Webots ROS2 Spot

[![ROS2 Humble](https://github.com/MASKOR/webots_ros2_spot/actions/workflows/test_ros2_humble.yml/badge.svg?branch=main)](https://github.com/MASKOR/webots_ros2_spot/actions/workflows/test_ros2_humble.yml)

This is a ROS 2 package to simulate the Boston Dynamics spot in [webots](https://cyberbotics.com/). Spot is able to walk around, to sit, standup and lie down. We also attached some sensors on spot, like a kinect and a 3D laser.
The world contains apriltags, a red line to test lane follower and objects for manipulation tasks.

![Spot](https://github.com/MASKOR/webots_ros2_spot/blob/rescue_arena/worlds/.rescue_arena.jpg)

## Prerequisites

    - Ubuntu 22.04
    - ROS2 Humble https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
    - Webots 2025a https://github.com/cyberbotics/webots/releases/tag/R2025a
    - git lfs

## Install

1. Install ROS2 Development tools and initialise and update rosdep:
    ```bash
    sudo apt install -y ros-dev-tools
    ```
    ```bash
    source /opt/ros/humble/setup.bash
    sudo rosdep init
    rosdep update
    ```

2. Create a new ROS2 workspace:
    ```bash
    export COLCON_WS=~/ros2_ws
    mkdir -p $COLCON_WS/src
    ```

3. Pull the repository, initialize submodules, install dependencies, and build the workspace:

    ```bash
    cd $COLCON_WS

    # Clone main repository with submodules
    git clone --recurse-submodules git@github.com:MASKOR/webots_ros2_spot.git src/webots_ros2_spot

    # If already cloned without submodules
    # git submodule update --init --recursive

    # Install dependencies
    rosdep install --ignore-src --from-paths src -y -r

    # Import additional dependencies (if required by .repos file)
    vcs import --recursive src --skip-existing --input src/webots_ros2_spot/webots_ros2_spot.repos
    ```

4. Install Kortex description and robotiq description and gripper controller
    ```
    sudo apt install ros-humble-kortex-description ros-humble-robotiq-description ros-humble-gripper-controllers
    ```
5. Copy the 2f_140 folder in ```opt/ros/humble/share/robotiq_description/meshes/visual``` and ```opt/ros/humble/share/robotiq_description/meshes/collision```

6. Build packages and source the workspace
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

## Switch to Float Mode

Call the service `float_mode` with true:

```
$ ros2 service call /Spot/float_mode std_srvs/srv/SetBool "data: true"
```

# Realtime servoing 
```
ros2 launch webots_spot servo_launch.py

ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
```
Note: The commands can be published on topics ```sevo_node/delta_twist_cmds``` and ```sevo_node/delta_joint_cmds```. Play around with the `moveit_servo_config.yaml` if servo launch throws errors

For servoing using keyboard(teleop) (acting a bit weird, but works)
```
ros2 run webots_spot teleop_servo 
```

## ALeRT 3DNav

Sorry! It's private 
# Webots ROS2 Spot

[![ROS2 Humble](https://github.com/MASKOR/webots_ros2_spot/actions/workflows/test_ros2_humble.yml/badge.svg?branch=main)](https://github.com/MASKOR/webots_ros2_spot/actions/workflows/test_ros2_humble.yml)

This is a ROS 2 package to simulate the Boston Dynamics spot in [webots](https://cyberbotics.com/). Spot is able to walk around, to sit, standup and lie down. We also attached some sensors on spot, like a kinect and a 3D laser.
The world contains apriltags, a red line to test lane follower and objects for manipulation tasks.

![Spot](https://github.com/MASKOR/webots_ros2_spot/blob/main/spot.jpg)

## Prerequisites

    - Ubuntu 22.04
    - ROS2 Humble https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
    - Webots 2025a https://github.com/cyberbotics/webots/releases/tag/R2025a

## Install

1. Install ROS2 Development tools and initialise and update rosdep:
    ```
    sudo apt install -y ros-dev-tools
    ```
    ```
    source /opt/ros/humble/setup.bash
    sudo rosdep init
    rosdep update
    ```

2. Create a new ROS2 workspace:
    ```
    export COLCON_WS=~/ros2_ws
    mkdir -p $COLCON_WS/src
    ```

3. Pull relevant packages, install dependencies, compile, and source the workspace by using:
    ```
    cd $COLCON_WS
    git clone https://github.com/MASKOR/webots_ros2_spot src/webots_ros2_spot
    rosdep install --ignore-src --from-paths src -y -r
    vcs import --recursive src --skip-existing --input src/webots_ros2_spot/webots_ros2_spot.repos
    chmod +x src/webots_ros2/webots_ros2_driver/webots_ros2_driver/ros2_supervisor.py
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

## To switch Arenas

1) Change false to true in https://github.com/MASKOR/webots_ros2_spot/blob/main/resource/spot_control.urdf#L5

2) Change map.yaml to map_arena3.yaml https://github.com/MASKOR/webots_ros2_spot/blob/main/launch/nav_launch.py#L15 (map of arena 2 not created)

## Task: implement a path-planning algorithm

[webots_spot/nav/](webots_spot/nav/) contains a small, from-scratch alternative to nav2 for the maze world: [mapping_server.py](webots_spot/nav/mapping_server.py) republishes a pre-generated `.pgm`/`.yaml` map as an `OccupancyGrid`, and [global_planner.py](webots_spot/nav/global_planner.py) turns a `2D Goal Pose` into a `nav_msgs/Path` using one of the search algorithms in [webots_spot/nav/planner_algorithms/](webots_spot/nav/planner_algorithms/). BFS, DFS, Dijkstra and A* are stubbed out with pseudocode and `NotImplementedError` — the task is to fill in the `YOUR CODE HERE` sections.

1. Find your `webots_ros2_spot` package inside your ROS2 workspace (`$COLCON_WS/src/webots_ros2_spot`), go into that folder and check out this task's branch:
    ```
    cd $COLCON_WS/src/webots_ros2_spot
    git checkout maze-world
    ```

2. Switching branches changes the package's files, so rebuild it and source the workspace again from the root of your ROS2 workspace:
    ```
    cd $COLCON_WS
    colcon build --packages-select webots_spot
    source install/setup.bash
    ```

3. Open a terminal (<kbd>Ctrl</kbd>+<kbd>T</kbd> for a new tab, <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>T</kbd> for a new window) and make sure `~/.bashrc` sources both ROS2 and this workspace:
    ```
    source /opt/ros/humble/setup.bash
    source $COLCON_WS/install/setup.bash
    ```

4. Launch the simulation:
    ```
    ros2 launch webots_spot spot_launch.py
    ```

5. In a new terminal, start RViz2 and configure it:
    ```
    rviz2
    ```
    - Set **Fixed Frame** to `map`.
    - **Add** → `TF`.
    - **Add** → `RobotModel`.
    - **Add** → `Map`, set its **Topic** to `/map` (`nav_msgs/OccupancyGrid`).
    - Add the **2D Goal Pose** tool button (the arrow icon) to the toolbar if it isn't already there.

6. In two more terminals, start the map server and the global planner:
    ```
    ros2 run webots_spot mapping_server
    ```
    ```
    ros2 run webots_spot global_planner
    ```

7. In RViz2, click **2D Goal Pose** and click-drag on the map to send a goal. With the default `algorithm` (`astar`) unimplemented, check the `global_planner` terminal for the `NotImplementedError` it logs — that's the algorithm you're about to implement.

8. Open the workspace in VS Code and implement the search in [webots_spot/nav/planner_algorithms/bfs.py](webots_spot/nav/planner_algorithms/bfs.py) (each `STEP` comment maps to a line of the pseudocode in the module docstring). `dfs.py`, `dijkstra.py` and `astar.py` follow the same pattern. Rebuild after editing, from the root of your ROS2 workspace (not this package's directory):
    ```
    cd $COLCON_WS
    colcon build --packages-select webots_spot
    source install/setup.bash
    ```

9. In a new terminal, start `rqt` and open **Plugins → Configuration → Dynamic Reconfigure** to switch `/global_planner`'s `algorithm` parameter between `bfs`, `dfs`, `dijkstra` and `astar` at runtime, then send another **2D Goal Pose** to compare them:
    ```
    rqt
    ```
    (Equivalently: `ros2 param set /global_planner algorithm dfs`.)

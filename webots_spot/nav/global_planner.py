#!/usr/bin/env python3
"""Grid-based global planner.

Subscribes to the occupancy grid once (latched ``/map``), turns it into a
:class:`GridMap`, then on every ``/goal_pose`` computes a cell path from the
robot's current pose to the goal with BFS or DFS and publishes it as
``nav_msgs/Path``.

While searching it also streams the explored cells and the frontier as
``nav_msgs/GridCells`` so the BFS vs DFS expansion order can be watched in RViz.

The ``algorithm`` parameter is dynamic: ``ros2 param set /global_planner
algorithm dfs`` takes effect on the next goal, no restart needed.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from webots_spot.nav.grid_map import GridMap
from webots_spot.nav.planner_algorithms import astar, bfs, dfs, dijkstra

PLANNERS = {
    "astar": astar.plan,
    "dijkstra": dijkstra.plan,
    "bfs": bfs.plan,
    "dfs": dfs.plan,
}


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__("global_planner")

        self.declare_parameter("algorithm", "astar")   # astar|dijkstra|bfs|dfs
        # Weighted A* (f = g + weight*h): >1 is faster/greedier, path at most
        # `weight` x optimal. Only used when algorithm == "astar".
        self.declare_parameter("astar_weight", 1.0)
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("goal_topic", "goal_pose")
        self.declare_parameter("path_topic", "plan")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("allow_diagonal", False)
        self.declare_parameter("unknown_is_occupied", True)
        # Occupancy value at/above which a cell is a wall.
        self.declare_parameter("lethal_threshold", 90)
        # Extra traversal cost per unit occupancy for non-lethal cells (mud).
        # BFS/DFS ignore this; Dijkstra/A* use it to route around costly terrain.
        self.declare_parameter("cost_weight", 0.1)
        # Coarsen the map for planning: 0.0 = use it as-is, else target cell
        # size in metres (rounded to a whole multiple of the map resolution).
        self.declare_parameter("planning_resolution", 1.0)
        # Exploration visualisation: publish every Nth expansion, then sleep.
        self.declare_parameter("viz_every", 25)
        self.declare_parameter("viz_delay", 0.25)
        # Fallback start pose, used only when the TF lookup fails (NaN = unset).
        self.declare_parameter("start_x", float("nan"))
        self.declare_parameter("start_y", float("nan"))

        if self._algorithm() not in PLANNERS:
            raise ValueError(
                f"algorithm must be one of {list(PLANNERS)}, "
                f"got '{self._algorithm()}'"
            )
        self.add_on_set_parameters_callback(self._validate_params)

        self.grid = None
        self.map_frame = "map"
        self._step = 0
        self._discovered = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(
            Path, self.get_parameter("path_topic").value, 10
        )
        self.explored_pub = self.create_publisher(GridCells, "explored", 10)
        self.frontier_pub = self.create_publisher(GridCells, "frontier", 10)

        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self._map_cb,
            map_qos,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self._goal_cb,
            10,
        )

        self.get_logger().info(
            f"global_planner up (algorithm={self._algorithm()}); waiting for map..."
        )

    # ---------------------------------------------------------------- params
    def _algorithm(self):
        return self.get_parameter("algorithm").value

    def _validate_params(self, params):
        for p in params:
            if p.name == "algorithm" and p.value not in PLANNERS:
                return SetParametersResult(
                    successful=False,
                    reason=f"algorithm must be one of {list(PLANNERS)}",
                )
        return SetParametersResult(successful=True)

    # --------------------------------------------------------------- map (once)
    def _map_cb(self, msg):
        planning_res = self.get_parameter("planning_resolution").value
        downsample = 1
        if planning_res and planning_res > msg.info.resolution:
            downsample = max(1, round(planning_res / msg.info.resolution))

        self.grid = GridMap(
            msg,
            lethal_threshold=self.get_parameter("lethal_threshold").value,
            unknown_is_occupied=self.get_parameter("unknown_is_occupied").value,
            cost_weight=self.get_parameter("cost_weight").value,
            downsample=downsample,
        )
        self.map_frame = msg.header.frame_id or "map"
        self.get_logger().info(
            f"map received: {self.grid.width}x{self.grid.height} cells @ "
            f"{self.grid.resolution:.3f} m/cell (downsample x{downsample}) "
            f"in frame '{self.map_frame}'"
        )
        self.destroy_subscription(self.map_sub)
        self.map_sub = None

    # ------------------------------------------------------------------- goal
    def _goal_cb(self, msg):
        if self.grid is None:
            self.get_logger().warn("goal received but no map yet; ignoring")
            return

        algorithm = self._algorithm()
        allow_diagonal = bool(self.get_parameter("allow_diagonal").value)

        start_xy = self._robot_xy()
        if start_xy is None:
            return
        goal_xy = (msg.pose.position.x, msg.pose.position.y)

        start_cell = self.grid.world_to_grid(*start_xy)
        goal_cell = self.grid.world_to_grid(*goal_xy)

        if not self.grid.in_bounds(goal_cell):
            self.get_logger().warn(
                f"goal {goal_xy} -> cell {goal_cell} is outside the map"
            )
            return

        start_cell = self._snap(start_cell, "start")
        goal_cell = self._snap(goal_cell, "goal")
        if start_cell is None or goal_cell is None:
            return

        self._step = 0
        self._discovered = None
        kwargs = {"callback": self._on_search_step}
        if algorithm == "astar":
            kwargs["weight"] = self.get_parameter("astar_weight").value
        try:
            cells = PLANNERS[algorithm](
                self.grid, start_cell, goal_cell, allow_diagonal, **kwargs
            )
        except NotImplementedError as exc:
            self.get_logger().error(
                f"planner '{algorithm}' is not implemented yet: {exc}"
            )
            return
        self._flush_exploration()

        if not cells:
            self.get_logger().warn(
                f"no {algorithm} path from {start_cell} to {goal_cell}"
            )
            self.path_pub.publish(self._make_path([]))
            return

        path_cost = sum(self.grid.cost(c) for c in cells[1:])
        mud_cells = sum(1 for c in cells if self.grid.is_costly(c))
        self.get_logger().info(
            f"{algorithm}: {len(cells)} cells, cost {path_cost:.1f}, "
            f"{mud_cells} through mud; explored "
            f"{len(self._discovered or [])} cells"
        )
        self.path_pub.publish(self._make_path(cells, msg.pose.orientation))

    def _snap(self, cell, label):
        if self.grid.is_free(cell):
            return cell
        snapped = self.grid.nearest_free_cell(cell)
        if snapped is None:
            self.get_logger().warn(
                f"{label} cell {cell} is blocked and no free cell is nearby"
            )
        else:
            self.get_logger().info(
                f"{label} cell {cell} blocked -> snapped to {snapped}"
            )
        return snapped

    def _robot_xy(self):
        base_frame = self.get_parameter("base_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, base_frame, Time()
            )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException as exc:
            sx = self.get_parameter("start_x").value
            sy = self.get_parameter("start_y").value
            if not (math.isnan(sx) or math.isnan(sy)):
                self.get_logger().warn(
                    f"TF {self.map_frame}->{self.base_frame} failed ({exc}); "
                    f"using start_x/start_y fallback ({sx}, {sy})"
                )
                return (sx, sy)
            self.get_logger().warn(
                f"cannot get {self.map_frame}->{base_frame}: {exc}"
            )
            return None

    # ---------------------------------------------------------- exploration viz
    def _on_search_step(self, current, discovered, frontier):
        self._discovered = discovered
        self._step += 1

        every = self.get_parameter("viz_every").value
        if every <= 0 or current is None:
            return
        if self._step % every != 0:
            return

        self._publish_cells(self.explored_pub, discovered)
        self._publish_cells(self.frontier_pub, frontier)

        delay = self.get_parameter("viz_delay").value
        if delay > 0.0:
            time.sleep(delay)

    def _flush_exploration(self):
        if self._discovered is not None:
            self._publish_cells(self.explored_pub, self._discovered)
        self._publish_cells(self.frontier_pub, [])

    def _publish_cells(self, pub, cells):
        msg = GridCells()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.cell_width = self.grid.resolution
        msg.cell_height = self.grid.resolution
        for cell in cells:
            wx, wy = self.grid.grid_to_world(cell)
            msg.cells.append(Point(x=wx, y=wy, z=0.0))
        pub.publish(msg)

    # ------------------------------------------------------------------- path
    def _make_path(self, cells, goal_orientation=None):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        world = [self.grid.grid_to_world(c) for c in cells]
        for i, (wx, wy) in enumerate(world):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            if i + 1 < len(world):
                yaw = math.atan2(world[i + 1][1] - wy, world[i + 1][0] - wx)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            elif goal_orientation is not None:
                pose.pose.orientation = goal_orientation
            else:
                pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

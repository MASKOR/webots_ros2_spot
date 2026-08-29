#!/usr/bin/env python3
"""Path-following controller.

Subscribes to ``/plan`` (``nav_msgs/Path``) and drives the robot along it with
a pure-pursuit law, publishing ``geometry_msgs/Twist`` (``linear.x`` /
``angular.z``) on ``/cmd_vel``. The robot pose is taken from TF
(``<path frame> -> base_frame``).
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from webots_spot.nav import pure_pursuit


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class Controller(Node):
    def __init__(self):
        super().__init__("controller")

        self.declare_parameter("plan_topic", "plan")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("control_frequency", 20.0)

        self.declare_parameter("lookahead_distance", 0.5)
        self.declare_parameter("desired_linear_velocity", 0.3)
        # Multiplies the linear speed (and rotate-in-place speed). Dynamic.
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("max_angular_velocity", 1.5)
        self.declare_parameter("goal_tolerance", 0.15)
        # Above this heading error (rad) turn in place instead of driving.
        self.declare_parameter("rotate_to_heading_angle", 0.8)
        self.declare_parameter("rotate_gain", 1.5)
        # Start slowing down within this distance of the goal.
        self.declare_parameter("slowdown_distance", 0.6)

        self.base_frame = self.get_parameter("base_frame").value
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.create_subscription(
            Path, self.get_parameter("plan_topic").value, self._plan_cb, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_xy = []
        self.path_frame = "map"
        self.goal_reached = True
        self._idx = 0

        hz = self.get_parameter("control_frequency").value
        self.create_timer(1.0 / hz, self._control_step)
        self.get_logger().info("controller up; waiting for a plan...")

    # ------------------------------------------------------------------- plan
    def _plan_cb(self, msg):
        self.path_xy = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self.path_frame = msg.header.frame_id or "map"
        self._idx = 0

        if not self.path_xy:
            self.goal_reached = True
            self._stop()
            self.get_logger().warn("received empty plan; stopping")
            return

        self.goal_reached = False
        self.get_logger().info(f"new plan: {len(self.path_xy)} poses")

    # ---------------------------------------------------------------- control
    def _control_step(self):
        if self.goal_reached or not self.path_xy:
            return

        pose = self._robot_pose()
        if pose is None:
            self._stop()
            return
        rx, ry, ryaw = pose

        gx, gy = self.path_xy[-1]
        dist_to_goal = math.hypot(gx - rx, gy - ry)
        if dist_to_goal <= self.get_parameter("goal_tolerance").value:
            self._stop()
            self.goal_reached = True
            self.get_logger().info("goal reached")
            return

        lookahead = self.get_parameter("lookahead_distance").value
        scale = self.get_parameter("speed_scale").value
        v_max = self.get_parameter("desired_linear_velocity").value * scale
        w_max = self.get_parameter("max_angular_velocity").value

        self._idx = pure_pursuit.nearest_index(self.path_xy, rx, ry, self._idx)
        target, _ = pure_pursuit.lookahead_point(
            self.path_xy, rx, ry, lookahead, self._idx
        )

        cmd = Twist()
        head_err = pure_pursuit.heading_error(rx, ry, ryaw, target[0], target[1])
        if abs(head_err) > self.get_parameter("rotate_to_heading_angle").value:
            # Too far off: rotate in place toward the target.
            gain = self.get_parameter("rotate_gain").value * scale
            cmd.angular.z = max(-w_max, min(w_max, gain * head_err))
        else:
            v, w = pure_pursuit.command(
                rx, ry, ryaw, target[0], target[1], v_max, w_max
            )
            slowdown = self.get_parameter("slowdown_distance").value
            if slowdown > 0.0 and dist_to_goal < slowdown:
                v *= max(0.15, dist_to_goal / slowdown)
            cmd.linear.x = v
            cmd.angular.z = w

        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------- utils
    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.path_frame, self.base_frame, Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"cannot get {self.path_frame}->{self.base_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        t = tf.transform.translation
        return (t.x, t.y, yaw_from_quaternion(tf.transform.rotation))

    def _stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

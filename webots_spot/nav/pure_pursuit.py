#!/usr/bin/env python3
"""Pure-pursuit geometry helpers (no ROS)."""

import math


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def nearest_index(path_xy, x, y, start=0):
    """Index of the path point closest to (x, y), searching from ``start``."""
    best_i = start
    best_d2 = float("inf")
    for i in range(start, len(path_xy)):
        px, py = path_xy[i]
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
    return best_i


def lookahead_point(path_xy, x, y, lookahead, start=0):
    """First path point at least ``lookahead`` metres from (x, y).

    Scans forward from ``start``; returns ``(point, index)``. Falls back to the
    last path point when none is far enough (i.e. near the end of the path).
    """
    for i in range(start, len(path_xy)):
        px, py = path_xy[i]
        if math.hypot(px - x, py - y) >= lookahead:
            return (px, py), i
    return path_xy[-1], len(path_xy) - 1


def command(robot_x, robot_y, robot_yaw, target_x, target_y,
            linear_velocity, max_angular_velocity):
    """Pure-pursuit (v, omega) toward the target point.

    Curvature = 2 * y_r / L^2, with (x_r, y_r) the target in the robot frame
    and L the straight-line distance to it.
    """
    dx = target_x - robot_x
    dy = target_y - robot_y
    dist2 = dx * dx + dy * dy
    if dist2 < 1e-9:
        return 0.0, 0.0

    cos_y = math.cos(-robot_yaw)
    sin_y = math.sin(-robot_yaw)
    y_r = sin_y * dx + cos_y * dy

    curvature = 2.0 * y_r / dist2
    omega = linear_velocity * curvature
    omega = max(-max_angular_velocity, min(max_angular_velocity, omega))
    return linear_velocity, omega


def heading_error(robot_x, robot_y, robot_yaw, target_x, target_y):
    """Signed angle between the robot heading and the direction to the target."""
    desired = math.atan2(target_y - robot_y, target_x - robot_x)
    return normalize_angle(desired - robot_yaw)

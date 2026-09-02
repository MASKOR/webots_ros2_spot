#!/usr/bin/env python3
"""2D occupancy grid wrapper used by the global planner.

Builds a lightweight, query-friendly grid from a ``nav_msgs/OccupancyGrid``.
Cell coordinates are ``(col, row)`` with cell ``(0, 0)`` at the map's ``origin``
pose; storage is row-major, matching the OccupancyGrid ``data`` layout.
"""

import math


class GridMap:
    FREE = "free"
    OCCUPIED = "occupied"
    UNKNOWN = "unknown"

    def __init__(self, occupancy_grid, lethal_threshold=90,
                 unknown_is_occupied=True, cost_weight=0.1, downsample=1):
        """``lethal_threshold``: occupancy value at/above which a cell is a wall.
        Values below it are traversable; ``cost_weight`` turns their occupancy
        value into an extra per-cell traversal cost (mud etc.)."""
        info = occupancy_grid.info
        self.origin_x = info.origin.position.x
        self.origin_y = info.origin.position.y

        q = info.origin.orientation
        self.origin_yaw = 2.0 * math.atan2(q.z, q.w)
        self._cos = math.cos(self.origin_yaw)
        self._sin = math.sin(self.origin_yaw)

        self._lethal = lethal_threshold
        self._unknown_is_occupied = unknown_is_occupied
        self._cost_weight = cost_weight

        n = max(1, int(downsample))
        self.downsample = n
        if n == 1:
            self.resolution = info.resolution
            self.width = info.width
            self.height = info.height
            self._data = list(occupancy_grid.data)
        else:
            self.resolution = info.resolution * n
            self.width = info.width // n
            self.height = info.height // n
            self._data = self._pool(occupancy_grid.data, info.width, n)

    def _pool(self, src, src_width, n):
        """Reduce each n-by-n source block to one cell, worst-case:
        any wall -> wall; else any unknown -> unknown; else the max cost."""
        thr = self._lethal
        out = [0] * (self.width * self.height)
        for cy in range(self.height):
            for cx in range(self.width):
                block_max = -128
                has_unknown = False
                for dy in range(n):
                    base = (cy * n + dy) * src_width + cx * n
                    for dx in range(n):
                        v = src[base + dx]
                        if v < 0:
                            has_unknown = True
                        elif v > block_max:
                            block_max = v
                if block_max >= thr:
                    out[cy * self.width + cx] = block_max
                elif has_unknown:
                    out[cy * self.width + cx] = -1
                else:
                    out[cy * self.width + cx] = max(block_max, 0)
        return out

    # --------------------------------------------------------------- queries
    def in_bounds(self, cell):
        cx, cy = cell
        return 0 <= cx < self.width and 0 <= cy < self.height

    def value(self, cell):
        """Raw occupancy value (-1 unknown, 0..100)."""
        cx, cy = cell
        return self._data[cy * self.width + cx]

    def state(self, cell):
        v = self.value(cell)
        if v < 0:
            return self.UNKNOWN
        if v >= self._lethal:
            return self.OCCUPIED
        return self.FREE

    def is_occupied(self, cell):
        if not self.in_bounds(cell):
            return True
        s = self.state(cell)
        if s == self.OCCUPIED:
            return True
        if s == self.UNKNOWN:
            return self._unknown_is_occupied
        return False

    def is_free(self, cell):
        return self.in_bounds(cell) and not self.is_occupied(cell)

    def cost(self, cell):
        """Traversal cost of entering ``cell`` (>= 1.0). Higher over mud."""
        v = self.value(cell)
        if v < 0:
            v = 0
        return 1.0 + self._cost_weight * v

    def is_costly(self, cell):
        return self.is_free(cell) and self.value(cell) > 0

    def neighbors(self, cell, allow_diagonal=False):
        """Yield the free 4- (or 8-) connected neighbours of ``cell``."""
        cx, cy = cell
        steps = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if allow_diagonal:
            steps += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

        for dx, dy in steps:
            nxt = (cx + dx, cy + dy)
            if not self.is_free(nxt):
                continue
            # Don't allow a diagonal move that squeezes between two walls.
            if dx != 0 and dy != 0:
                if self.is_occupied((cx + dx, cy)) and \
                        self.is_occupied((cx, cy + dy)):
                    continue
            yield nxt

    def nearest_free_cell(self, cell, max_radius=30):
        """Closest free cell to ``cell`` (itself if already free), or None."""
        if self.is_free(cell):
            return cell
        cx, cy = cell
        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    cand = (cx + dx, cy + dy)
                    if self.is_free(cand):
                        return cand
        return None

    # ----------------------------------------------------------- conversions
    def world_to_grid(self, wx, wy):
        dx = wx - self.origin_x
        dy = wy - self.origin_y
        lx = self._cos * dx + self._sin * dy
        ly = -self._sin * dx + self._cos * dy
        return (int(math.floor(lx / self.resolution)),
                int(math.floor(ly / self.resolution)))

    def grid_to_world(self, cell):
        cx, cy = cell
        lx = (cx + 0.5) * self.resolution
        ly = (cy + 0.5) * self.resolution
        wx = self.origin_x + self._cos * lx - self._sin * ly
        wy = self.origin_y + self._sin * lx + self._cos * ly
        return (wx, wy)

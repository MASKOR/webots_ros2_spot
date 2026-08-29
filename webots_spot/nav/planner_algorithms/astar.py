#!/usr/bin/env python3
"""A* search over a GridMap.

Dijkstra guided by a heuristic ``h`` (Manhattan for 4-connected grids, octile
for 8-connected). It returns the same cheapest path but, pulled toward the goal,
expands far fewer cells. Move length is 1 orthogonally / sqrt(2) diagonally,
multiplied by the destination cell's ``grid.cost`` (so it also routes around
mud). ``h`` is scaled by the minimum cost (1.0) so it never overestimates and
the path stays optimal when ``weight == 1``.

``weight`` > 1 runs weighted A* (``f = g + weight * h``): greedier and faster,
with a path cost at most ``weight`` times optimal.

Pseudocode
----------
    g[start]   <- 0
    came_from  <- { start: none }
    open       <- priority queue keyed by f = g + weight*h, holding start
    closed     <- {}
    while open not empty:
        current <- open.pop_min()                 # smallest f
        if current in closed: continue
        closed.add(current)
        if current == goal:
            return reconstruct(came_from, goal)
        for next in free_neighbours(current):
            step      <- 1  (orthogonal)  or  sqrt(2)  (diagonal)
            tentative <- g[current] + step * cost(next)
            if tentative < g[next]:
                g[next]         <- tentative
                came_from[next] <- current
                f <- tentative + weight * h(next, goal)
                open.push(next, priority = f)
    return failure

Complexity: O(E log V) time, O(V) memory  (heuristic only cuts the constant).
"""

from __future__ import annotations

import heapq
import math
from typing import Optional

from webots_spot.nav.planner_algorithms._common import (
    Cell,
    GridMap,
    SearchCallback,
    reconstruct_path,
)

SQRT2: float = math.sqrt(2.0)


def plan(grid: GridMap,
         start: Cell,
         goal: Cell,
         allow_diagonal: bool = False,
         callback: Optional[SearchCallback] = None,
         weight: float = 1.0) -> Optional[list[Cell]]:
    if start == goal:
        return [start]
    if not grid.is_free(start) or not grid.is_free(goal):
        return None

    gx, gy = goal
    w = max(1.0, weight)

    def heuristic(cell: Cell) -> float:
        dx = abs(cell[0] - gx)
        dy = abs(cell[1] - gy)
        if allow_diagonal:
            base = (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)
        else:
            base = dx + dy
        return w * base

    open_heap: list[tuple[float, int, Cell]] = [(heuristic(start), 0, start)]
    counter: int = 1
    came_from: dict[Cell, Optional[Cell]] = {start: None}
    g_score: dict[Cell, float] = {start: 0.0}
    closed: set[Cell] = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        if callback is not None:
            callback(current, g_score, (item[2] for item in open_heap))

        if current == goal:
            return reconstruct_path(came_from, goal)

        cx, cy = current
        for nxt in grid.neighbors(current, allow_diagonal):
            step = SQRT2 if (nxt[0] != cx and nxt[1] != cy) else 1.0
            tentative = g_score[current] + step * grid.cost(nxt)
            if tentative < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative
                heapq.heappush(
                    open_heap, (tentative + heuristic(nxt), counter, nxt)
                )
                counter += 1

    if callback is not None:
        callback(None, g_score, iter(()))
    return None

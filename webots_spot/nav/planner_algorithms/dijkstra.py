#!/usr/bin/env python3
"""Dijkstra / uniform-cost search over a GridMap.

Unlike BFS it accounts for per-cell traversal cost (``grid.cost``), so it finds
the cheapest route rather than the one with the fewest cells - it detours around
mud that BFS would drive straight through. With no heuristic it expands cells in
order of cost-so-far, spreading out equally in every direction.

Pseudocode
----------
    g[start]   <- 0
    came_from  <- { start: none }
    open       <- priority queue keyed by g, holding start
    closed     <- {}
    while open not empty:
        current <- open.pop_min()                 # smallest g
        if current in closed: continue            # stale queue entry
        closed.add(current)
        if current == goal:
            return reconstruct(came_from, goal)
        for next in free_neighbours(current):
            step      <- 1  (orthogonal)  or  sqrt(2)  (diagonal)
            tentative <- g[current] + step * cost(next)
            if tentative < g[next]:
                g[next]         <- tentative
                came_from[next] <- current
                open.push(next, priority = tentative)
    return failure

Complexity: O(E log V) time, O(V) memory.
"""

from __future__ import annotations

import heapq
from typing import Optional

from webots_spot.nav.planner_algorithms._common import (
    Cell,
    GridMap,
    SearchCallback,
    reconstruct_path,
)

SQRT2: float = 2.0 ** 0.5


def plan(grid: GridMap,
         start: Cell,
         goal: Cell,
         allow_diagonal: bool = False,
         callback: Optional[SearchCallback] = None) -> Optional[list[Cell]]:
    if start == goal:
        return [start]
    if not grid.is_free(start) or not grid.is_free(goal):
        return None

    open_heap: list[tuple[float, int, Cell]] = [(0.0, 0, start)]
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
                heapq.heappush(open_heap, (tentative, counter, nxt))
                counter += 1

    if callback is not None:
        callback(None, g_score, iter(()))
    return None

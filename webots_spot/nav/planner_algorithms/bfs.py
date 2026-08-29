#!/usr/bin/env python3
"""Breadth-first search over a GridMap.

BFS explores cells in order of distance (in steps) from the start, so the first
time it reaches the goal it has found a path with the fewest cells. All edges
count as 1 - it ignores per-cell terrain cost (see dijkstra / astar for that).

Pseudocode
----------
    frontier   <- queue containing start          # FIFO
    came_from  <- { start: none }
    while frontier not empty:
        current <- frontier.dequeue()             # closest unexpanded cell
        if current == goal:
            return reconstruct(came_from, goal)
        for next in free_neighbours(current):
            if next not seen:
                came_from[next] <- current
                frontier.enqueue(next)
    return failure                                 # goal unreachable

Complexity: O(V + E) time, O(V) memory  (V = free cells, E = edges).
"""

from __future__ import annotations

from collections import deque
from typing import Optional

from webots_spot.nav.planner_algorithms._common import (
    Cell,
    GridMap,
    SearchCallback,
    reconstruct_path,
)


def plan(grid: GridMap,
         start: Cell,
         goal: Cell,
         allow_diagonal: bool = False,
         callback: Optional[SearchCallback] = None) -> Optional[list[Cell]]:
    if start == goal:
        return [start]
    if not grid.is_free(start) or not grid.is_free(goal):
        return None

    frontier: deque[Cell] = deque([start])
    came_from: dict[Cell, Optional[Cell]] = {start: None}

    while frontier:
        current = frontier.popleft()
        if callback is not None:
            callback(current, came_from, frontier)
        if current == goal:
            return reconstruct_path(came_from, goal)
        for nxt in grid.neighbors(current, allow_diagonal):
            if nxt not in came_from:
                came_from[nxt] = current
                frontier.append(nxt)

    if callback is not None:
        callback(None, came_from, frontier)
    return None

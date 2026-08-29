#!/usr/bin/env python3
"""Depth-first search over a GridMap.

DFS follows one branch as far as it can before backtracking. It finds *a* path
if one exists, with little memory, but - unlike BFS - the path is usually far
from the shortest. It also ignores terrain cost.

Pseudocode
----------
    stack      <- [ start ]                        # LIFO
    came_from  <- { start: none }
    while stack not empty:
        current <- stack.pop()                    # most recently discovered
        if current == goal:
            return reconstruct(came_from, goal)
        for next in free_neighbours(current):
            if next not seen:
                came_from[next] <- current
                stack.push(next)
    return failure                                 # goal unreachable

(Neighbours are marked seen when pushed, so each cell is expanded once and the
search always terminates.)

Complexity: O(V + E) time, O(V) memory.
"""

from __future__ import annotations

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

    stack: list[Cell] = [start]
    came_from: dict[Cell, Optional[Cell]] = {start: None}

    while stack:
        current = stack.pop()
        if callback is not None:
            callback(current, came_from, stack)
        if current == goal:
            return reconstruct_path(came_from, goal)
        for nxt in grid.neighbors(current, allow_diagonal):
            if nxt not in came_from:
                came_from[nxt] = current
                stack.append(nxt)

    if callback is not None:
        callback(None, came_from, stack)
    return None

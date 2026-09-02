#!/usr/bin/env python3
"""Depth-first search over a GridMap.

DFS follows one branch as far as it can before backtracking. It finds *a* path
if one exists, with little memory, but - unlike BFS - the path is usually far
from the shortest. It also ignores terrain cost.

The ONLY code difference from BFS is STEP 1: DFS pops the cell that was added
*last* (LIFO stack) instead of the one added first (FIFO queue).

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

Complexity: O(V + E) time, O(V) memory.

------------------------------------------------------------------------------
EXERCISE: fill in the three steps marked "YOUR CODE HERE" inside the loop.
Helpers you can use:
  * grid.neighbors(cell, allow_diagonal)  -> iterator of free neighbour cells
  * reconstruct_path(came_from, goal)     -> list[Cell] from start to goal
  * a cell counts as "seen" once it is a key in `came_from`
------------------------------------------------------------------------------
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

    # The frontier is a LIFO stack; `came_from` doubles as the "seen" set and
    # as the back-pointers used to rebuild the path.
    stack: list[Cell] = [start]
    came_from: dict[Cell, Optional[Cell]] = {start: None}

    while stack:
        raise NotImplementedError(
            "dfs.plan is not implemented yet - complete STEP 1-3 below "
            "and delete this raise"
        )

        # ---- STEP 1 : take the next cell to expand -----------------------
        # DFS treats `stack` as a LIFO stack -> remove the MOST RECENTLY
        # added entry and store it in `current`.
        #   pseudocode:  current <- stack.pop()
        #
        # >>>>>>>>>>>>>>>>>>>>>>>> YOUR CODE HERE <<<<<<<<<<<<<<<<<<<<<<<<<<<
        current = None  # TODO: replace with the popped cell

        # Keep this — it feeds the /explored and /frontier RViz visualisation.
        if callback is not None:
            callback(current, came_from, stack)

        # ---- STEP 2 : goal test -----------------------------------------
        # If `current` is the goal, return reconstruct_path(came_from, goal)
        #
        # >>>>>>>>>>>>>>>>>>>>>>>> YOUR CODE HERE <<<<<<<<<<<<<<<<<<<<<<<<<<<

        # ---- STEP 3 : expand `current` --------------------------------------
        # For every free neighbour that has NOT been seen yet:
        #   * set its back-pointer:  came_from[nxt] = current
        #   * push it onto the stack
        for nxt in grid.neighbors(current, allow_diagonal):
            # >>>>>>>>>>>>>>>>>>>> YOUR CODE HERE <<<<<<<<<<<<<<<<<<<<<<<<<<<
            pass

    # Stack drained without reaching the goal -> no path exists.
    if callback is not None:
        callback(None, came_from, stack)
    return None

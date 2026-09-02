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

------------------------------------------------------------------------------
EXERCISE: fill in the three steps marked "YOUR CODE HERE" inside the loop.
Helpers you can use:
  * grid.neighbors(cell, allow_diagonal)  -> iterator of free neighbour cells
  * reconstruct_path(came_from, goal)     -> list[Cell] from start to goal
  * a cell counts as "seen" once it is a key in `came_from`
------------------------------------------------------------------------------
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

    # The frontier is a FIFO queue; `came_from` doubles as the "seen" set and
    # as the back-pointers used to rebuild the path.
    frontier: deque[Cell] = deque([start])
    came_from: dict[Cell, Optional[Cell]] = {start: None}

    while frontier:
        raise NotImplementedError(
            "bfs.plan is not implemented yet - complete STEP 1-3 below "
            "and delete this raise"
        )

        # ---- STEP 1 : take the next cell to expand -----------------------
        # BFS treats `frontier` as a FIFO queue -> remove the OLDEST entry
        # and store it in a variable called `current`.
        #   pseudocode:  current <- frontier.dequeue()
        #
        # >>>>>>>>>>>>>>>>>>>>>>>> YOUR CODE HERE <<<<<<<<<<<<<<<<<<<<<<<<<<<
        current = None  # TODO: replace with the dequeued cell

        # Keep this — it feeds the /explored and /frontier RViz visualisation.
        if callback is not None:
            callback(current, came_from, frontier)

        # ---- STEP 2 : goal test -----------------------------------------
        # If `current` is the goal, return the finished path with
        #   reconstruct_path(came_from, goal)
        #
        # >>>>>>>>>>>>>>>>>>>>>>>> YOUR CODE HERE <<<<<<<<<<<<<<<<<<<<<<<<<<<

        # ---- STEP 3 : expand `current` --------------------------------------
        # For every free neighbour that has NOT been seen yet:
        #   * set its back-pointer:  came_from[nxt] = current
        #   * add it to the frontier
        #   pseudocode:  came_from[next] <- current;  frontier.enqueue(next)
        for nxt in grid.neighbors(current, allow_diagonal):
            # >>>>>>>>>>>>>>>>>>>> YOUR CODE HERE <<<<<<<<<<<<<<<<<<<<<<<<<<<
            pass

    # Frontier drained without reaching the goal -> no path exists.
    if callback is not None:
        callback(None, came_from, frontier)
    return None

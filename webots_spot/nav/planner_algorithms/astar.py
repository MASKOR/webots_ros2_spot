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

The ONLY code difference from Dijkstra is the priority pushed in STEP 3:
    Dijkstra:  priority = tentative                (g only)
    A*:        priority = tentative + heuristic(nxt)  (g + h, heuristic already
                                                       includes `weight`)

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

------------------------------------------------------------------------------
EXERCISE: fill in the steps marked "YOUR CODE HERE".
`heuristic(cell)` is already provided (it also multiplies in `weight`).
Same helpers as dijkstra.py.
------------------------------------------------------------------------------
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
        """Optimistic remaining cost from `cell` to the goal (ignores walls)."""
        dx = abs(cell[0] - gx)
        dy = abs(cell[1] - gy)
        if allow_diagonal:
            base = (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)   # octile distance
        else:
            base = dx + dy                                   # Manhattan distance
        return w * base

    open_heap: list[tuple[float, int, Cell]] = [(heuristic(start), 0, start)]
    counter: int = 1
    came_from: dict[Cell, Optional[Cell]] = {start: None}
    g_score: dict[Cell, float] = {start: 0.0}
    closed: set[Cell] = set()

    while open_heap:
        raise NotImplementedError(
            "astar.plan is not implemented yet - complete STEP 1-3 below "
            "and delete this raise"
        )

        # ---- STEP 1 : pop the cell with the smallest f ------------------
        # Pop the smallest heap entry, unpack the cell into `current`.
        # Skip it if it is already in `closed` (stale entry); otherwise add
        # `current` to `closed`.  (Identical to Dijkstra's STEP 1.)
        #
        # >>>>>>>>>>>>>>>>>>>>>>>> YOUR CODE HERE <<<<<<<<<<<<<<<<<<<<<<<<<<<
        current = None  # TODO

        # Keep this — it feeds the /explored and /frontier RViz visualisation.
        if callback is not None:
            callback(current, g_score, (item[2] for item in open_heap))

        # ---- STEP 2 : goal test ---------------------------------------
        #   if current == goal: return reconstruct_path(came_from, goal)
        #
        # >>>>>>>>>>>>>>>>>>>>>>>> YOUR CODE HERE <<<<<<<<<<<<<<<<<<<<<<<<<<<

        # ---- STEP 3 : relax the neighbours -------------------------------
        # For each free neighbour `nxt` of `current`:
        #   step      = SQRT2 for a diagonal move, else 1.0
        #   tentative = g_score[current] + step * grid.cost(nxt)
        #   if tentative is cheaper than the best cost known for `nxt`:
        #       update g_score[nxt] and came_from[nxt]
        #       priority = tentative + heuristic(nxt)      <-- the A* bit
        #       heappush (priority, counter, nxt) and bump `counter`
        cx, cy = current
        for nxt in grid.neighbors(current, allow_diagonal):
            # >>>>>>>>>>>>>>>>>>>> YOUR CODE HERE <<<<<<<<<<<<<<<<<<<<<<<<<<<
            pass

    if callback is not None:
        callback(None, g_score, iter(()))
    return None

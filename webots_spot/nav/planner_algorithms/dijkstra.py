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

------------------------------------------------------------------------------
EXERCISE: fill in the steps marked "YOUR CODE HERE".
Helpers / facts:
  * open_heap holds tuples (priority, tie_breaker, cell); use
      heapq.heappop(open_heap) / heapq.heappush(open_heap, (...))
  * `counter` is the tie-breaker: pass it, then do `counter += 1`
  * grid.cost(cell)  -> traversal cost of entering `cell` (>= 1.0, more on mud)
  * best-known cost of a cell:  g_score.get(cell, float("inf"))
  * reconstruct_path(came_from, goal) -> list[Cell]
------------------------------------------------------------------------------
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
        raise NotImplementedError(
            "dijkstra.plan is not implemented yet - complete STEP 1-3 below "
            "and delete this raise"
        )

        # ---- STEP 1 : pop the cheapest cell -----------------------------
        # Pop the heap entry with the smallest priority and unpack the cell
        # into `current` (entry layout: (priority, tie_breaker, cell)).
        # If `current` is already in `closed` it is a stale entry -> skip to
        # the next loop iteration. Otherwise add `current` to `closed`.
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
        #   step      = SQRT2 for a diagonal move (both coords change), else 1.0
        #   tentative = g_score[current] + step * grid.cost(nxt)
        #   if tentative is cheaper than the best cost known for `nxt`:
        #       update g_score[nxt] and came_from[nxt]
        #       heappush (tentative, counter, nxt) and bump `counter`
        cx, cy = current
        for nxt in grid.neighbors(current, allow_diagonal):
            # >>>>>>>>>>>>>>>>>>>> YOUR CODE HERE <<<<<<<<<<<<<<<<<<<<<<<<<<<
            pass

    if callback is not None:
        callback(None, g_score, iter(()))
    return None

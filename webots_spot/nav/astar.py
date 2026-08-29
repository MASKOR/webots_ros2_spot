#!/usr/bin/env python3
"""A* search over a GridMap.

Dijkstra guided by a heuristic (Manhattan for 4-connected grids, octile for
8-connected): it finds the same cheapest path but, pulled toward the goal,
expands far fewer cells. Move length is 1 orthogonally / sqrt(2) diagonally,
multiplied by the destination cell's ``grid.cost`` (so it also routes around
mud). The heuristic uses the minimum cost (1.0), so it never overestimates.

``weight`` > 1 runs weighted A* (``f = g + weight * h``): much faster / greedier,
with a path cost at most ``weight`` times optimal.
"""

import heapq
import math

SQRT2 = math.sqrt(2.0)


def plan(grid, start, goal, allow_diagonal=False, callback=None, weight=1.0):
    """Return a list of ``(col, row)`` cells from ``start`` to ``goal``.

    Includes both endpoints. Returns ``None`` if no path exists.

    ``callback(current, discovered, frontier)`` is invoked once per expanded
    cell (and once more when the search ends) for visualising the exploration.
    """
    if start == goal:
        return [start]
    if not grid.is_free(start) or not grid.is_free(goal):
        return None

    gx, gy = goal
    w = max(1.0, weight)

    def heuristic(cell):
        dx = abs(cell[0] - gx)
        dy = abs(cell[1] - gy)
        if allow_diagonal:
            base = (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)
        else:
            base = dx + dy
        return w * base

    open_heap = [(heuristic(start), 0, start)]
    counter = 1
    came_from = {start: None}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        if callback is not None:
            callback(current, g_score, (item[2] for item in open_heap))

        if current == goal:
            return _reconstruct(came_from, goal)

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


def _reconstruct(came_from, goal):
    path = [goal]
    while came_from[path[-1]] is not None:
        path.append(came_from[path[-1]])
    path.reverse()
    return path

#!/usr/bin/env python3
"""Dijkstra / uniform-cost search over a GridMap.

Unlike BFS it accounts for per-cell traversal cost (``grid.cost``), so it finds
the cheapest route, not the one with the fewest cells - it will detour around
mud that BFS would drive straight through. It expands cells in order of
cost-so-far and, with no heuristic, explores in every direction equally.
"""

import heapq

SQRT2 = 2.0 ** 0.5


def plan(grid, start, goal, allow_diagonal=False, callback=None):
    """Return a list of ``(col, row)`` cells from ``start`` to ``goal``.

    Includes both endpoints. Returns ``None`` if no path exists.

    ``callback(current, discovered, frontier)`` is invoked once per expanded
    cell (and once more when the search ends) for visualising the exploration.
    """
    if start == goal:
        return [start]
    if not grid.is_free(start) or not grid.is_free(goal):
        return None

    open_heap = [(0.0, 0, start)]
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
                heapq.heappush(open_heap, (tentative, counter, nxt))
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

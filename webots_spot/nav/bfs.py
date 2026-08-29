#!/usr/bin/env python3
"""Breadth-first search over a GridMap.

BFS explores cells in order of distance (in steps) from the start, so the first
time it reaches the goal it has found a path with the fewest cells.
"""

from collections import deque


def plan(grid, start, goal, allow_diagonal=False, callback=None):
    """Return a list of ``(col, row)`` cells from ``start`` to ``goal``.

    Includes both endpoints. Returns ``None`` if no path exists.

    ``callback(current, discovered, frontier)`` is invoked once per expanded
    cell (and once more when the search ends) for visualising the exploration.
    ``discovered`` is the dict of every seen cell, ``frontier`` the open queue.
    """
    if start == goal:
        return [start]
    if not grid.is_free(start) or not grid.is_free(goal):
        return None

    frontier = deque([start])
    came_from = {start: None}

    while frontier:
        current = frontier.popleft()
        if callback is not None:
            callback(current, came_from, frontier)
        if current == goal:
            return _reconstruct(came_from, goal)
        for nxt in grid.neighbors(current, allow_diagonal):
            if nxt not in came_from:
                came_from[nxt] = current
                frontier.append(nxt)

    if callback is not None:
        callback(None, came_from, frontier)
    return None


def _reconstruct(came_from, goal):
    path = [goal]
    while came_from[path[-1]] is not None:
        path.append(came_from[path[-1]])
    path.reverse()
    return path

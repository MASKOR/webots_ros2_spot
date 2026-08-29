#!/usr/bin/env python3
"""Depth-first search over a GridMap.

DFS follows one branch as far as it can before backtracking. It finds *a* path
if one exists, but - unlike BFS - it is not guaranteed to be the shortest.
"""


def plan(grid, start, goal, allow_diagonal=False, callback=None):
    """Return a list of ``(col, row)`` cells from ``start`` to ``goal``.

    Includes both endpoints. Returns ``None`` if no path exists.

    ``callback(current, discovered, frontier)`` is invoked once per expanded
    cell (and once more when the search ends) for visualising the exploration.
    ``discovered`` is the dict of every seen cell, ``frontier`` the open stack.
    """
    if start == goal:
        return [start]
    if not grid.is_free(start) or not grid.is_free(goal):
        return None

    stack = [start]
    came_from = {start: None}

    while stack:
        current = stack.pop()
        if callback is not None:
            callback(current, came_from, stack)
        if current == goal:
            return _reconstruct(came_from, goal)
        for nxt in grid.neighbors(current, allow_diagonal):
            if nxt not in came_from:
                came_from[nxt] = current
                stack.append(nxt)

    if callback is not None:
        callback(None, came_from, stack)
    return None


def _reconstruct(came_from, goal):
    path = [goal]
    while came_from[path[-1]] is not None:
        path.append(came_from[path[-1]])
    path.reverse()
    return path

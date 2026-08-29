#!/usr/bin/env python3
"""Shared types for the grid planners.

Every ``plan()`` in this package has the same signature:

    plan(grid, start, goal, allow_diagonal=False, callback=None) -> path | None

* ``grid``           : a :class:`~webots_spot.nav.grid_map.GridMap`
* ``start`` / ``goal``: ``Cell`` = ``(col, row)`` integer grid coordinates
* ``allow_diagonal`` : allow 8-connected moves (diagonal step costs sqrt(2))
* ``callback``       : optional progress hook for visualising the search
* returns            : list of ``Cell`` from start to goal inclusive, or ``None``
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from typing import Optional

from webots_spot.nav.grid_map import GridMap

Cell = tuple[int, int]

# callback(current, discovered, frontier):
#   current    - the cell just expanded, or None once the search has finished
#   discovered - mapping of every cell seen so far (came_from / g_score dict)
#   frontier   - the open set right now (queue / stack / heap contents)
SearchCallback = Callable[[Optional[Cell], object, Iterable[Cell]], None]


def reconstruct_path(came_from: dict[Cell, Optional[Cell]],
                     goal: Cell) -> list[Cell]:
    """Walk the ``came_from`` links back from ``goal`` to the start."""
    path = [goal]
    while came_from[path[-1]] is not None:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


__all__ = ["Cell", "GridMap", "SearchCallback", "reconstruct_path"]

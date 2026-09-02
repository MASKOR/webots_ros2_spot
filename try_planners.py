#!/usr/bin/env python3
"""Quick offline test bench for the planner exercises.

Run it from this directory (the package root):

    python3 try_planners.py                 # all four planners
    python3 try_planners.py bfs astar        # just some
    python3 try_planners.py --seed 3 astar   # a different maze

It builds a small maze (via maze_generator.py), runs each planner
start -> goal, prints cells / cost / explored, and draws the path over the
maze. No ROS / Webots needed.
"""

import sys

import maze_generator
from webots_spot.nav.planner_algorithms import astar, bfs, dfs, dijkstra

PLANNERS = {"bfs": bfs, "dfs": dfs, "dijkstra": dijkstra, "astar": astar}

MUD_COST = 6.0
SMALL_W, SMALL_H = 31, 21


class AsciiGrid:
    """Minimal GridMap stand-in: only the methods the planners call."""

    def __init__(self, rows):
        self.rows = rows
        self.h = len(rows)
        self.w = len(rows[0])

    def char(self, cell):
        x, y = cell
        return self.rows[y][x]

    def is_free(self, cell):
        x, y = cell
        return 0 <= x < self.w and 0 <= y < self.h and self.rows[y][x] != "#"

    def cost(self, cell):
        return MUD_COST if self.char(cell) == "~" else 1.0

    def neighbors(self, cell, allow_diagonal=False):
        x, y = cell
        steps = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if allow_diagonal:
            steps += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dx, dy in steps:
            nxt = (x + dx, y + dy)
            if self.is_free(nxt):
                yield nxt


def find(rows, ch):
    for y, row in enumerate(rows):
        for x, c in enumerate(row):
            if c == ch:
                return (x, y)
    raise ValueError(f"'{ch}' not in maze")


def draw(rows, path):
    on_path = set(path)
    for y, row in enumerate(rows):
        print("  " + "".join(
            "*" if (x, y) in on_path and c in ".~" else c
            for x, c in enumerate(row)
        ))


def main(argv):
    seed = maze_generator.SEED
    names = []
    it = iter(argv)
    for arg in it:
        if arg == "--seed":
            seed = int(next(it))
        elif arg in PLANNERS:
            names.append(arg)
        else:
            sys.exit(f"unknown arg: {arg!r}  (planners: {list(PLANNERS)})")
    names = names or list(PLANNERS)

    rows = maze_generator.generate_maze(SMALL_W, SMALL_H, seed)
    grid = AsciiGrid(rows)
    start, goal = find(rows, "S"), find(rows, "G")
    print(f"maze {grid.w}x{grid.h}, seed {seed}, S={start} G={goal}")

    for name in names:
        explored = [0]

        def tally(*_):
            explored[0] += 1

        try:
            path = PLANNERS[name].plan(grid, start, goal, callback=tally)
        except NotImplementedError as exc:
            print(f"\n{name}: NOT IMPLEMENTED - {exc}")
            continue

        print(f"\n{name}:")
        if not path:
            print("  no path found")
            continue
        cost = sum(grid.cost(c) for c in path[1:])
        mud = sum(1 for c in path if grid.char(c) == "~")
        print(f"  cells={len(path)}  cost={cost:.1f}  mud={mud}  "
              f"explored={explored[0]}")
        draw(rows, path)


if __name__ == "__main__":
    main(sys.argv[1:])

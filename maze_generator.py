#!/usr/bin/env python3
"""Generate a demo maze: a Webots PROTO + a nav2 occupancy/cost map.

The maze is built to contrast grid planners:

* loops (braiding) + open rooms  -> several routes of similar length, so BFS,
  Dijkstra and A* can disagree on which is "best";
* "mud" patches                  -> passable but expensive terrain. Cost-aware
  planners (Dijkstra, A*) detour around it; BFS / DFS plough straight through.

Grid legend:  '#' wall   '.' free   '~' mud (costly)   'S' start   'G' goal
"""

import random
from collections import deque
from pathlib import Path


# ============================================================
# Configuration
# ============================================================

WIDTH = 61
HEIGHT = 61

CELL_SIZE = 1.0
WALL_HEIGHT = 1.0
WALL_THICKNESS = CELL_SIZE

SEED = 7

# 0.0 -> perfect maze (one route). Higher -> more loops -> more route choice.
BRAID_PROB = 0.55
ROOM_COUNT = 6            # open rooms carved into the maze (space for A* to cut)
ROOM_MIN = 4
ROOM_MAX = 10
# A big open hall through the middle: BFS/Dijkstra flood it, A* cuts straight
# across it. Fraction of the map (width, height) it spans.
PLAZA_W = 0.42
PLAZA_H = 0.42

# "Mud": passable, but costly. Drawn as mid-grey in the PGM so cost-aware
# planners see a raised cost while BFS/DFS still treat it as plain free space.
MUD_BLOBS = 8
MUD_MIN = 4
MUD_MAX = 10
MUD_SHADE = 110          # PGM grey level (0 = wall/black ... 254 = free/white)

OUTPUT = Path("protos/Maze.proto")

MAP_OUTPUT = Path("map/maze.pgm")
# meters / pixel. CELL_SIZE / MAP_RESOLUTION must be a positive integer.
MAP_RESOLUTION = 0.5


# ============================================================
# Maze generation
# ============================================================

ORTHO = ((1, 0), (-1, 0), (0, 1), (0, -1))

WALL, FREE, MUD = "#", ".", "~"


def generate_maze(width, height, seed):
    grid = _recursive_backtracker(width, height, seed)
    _carve_rooms(grid, seed)
    _carve_plaza(grid)
    _braid(grid, seed, BRAID_PROB)
    _add_mud(grid, seed)
    _place_endpoints_diameter(grid)
    return grid


def _carve_plaza(grid):
    h, w = len(grid), len(grid[0])
    pw, ph = int(w * PLAZA_W), int(h * PLAZA_H)
    x0, y0 = (w - pw) // 2, (h - ph) // 2
    for y in range(y0, y0 + ph):
        for x in range(x0, x0 + pw):
            grid[y][x] = FREE


def _recursive_backtracker(width, height, seed):
    # Maze algorithms work best with odd dimensions.
    if width % 2 == 0:
        width -= 1
    if height % 2 == 0:
        height -= 1

    rng = random.Random(seed)
    maze = [[WALL] * width for _ in range(height)]

    maze[1][1] = FREE
    stack = [(1, 1)]
    jumps = ((2, 0), (-2, 0), (0, 2), (0, -2))

    while stack:
        x, y = stack[-1]
        options = [
            (x + dx, y + dy, dx, dy)
            for dx, dy in jumps
            if 1 <= x + dx < width - 1
            and 1 <= y + dy < height - 1
            and maze[y + dy][x + dx] == WALL
        ]
        if not options:
            stack.pop()
            continue

        nx, ny, dx, dy = rng.choice(options)
        maze[y + dy // 2][x + dx // 2] = FREE
        maze[ny][nx] = FREE
        stack.append((nx, ny))

    return maze


def _carve_rooms(grid, seed):
    rng = random.Random(seed + 101)
    h, w = len(grid), len(grid[0])
    for _ in range(ROOM_COUNT):
        rw = rng.randint(ROOM_MIN, ROOM_MAX)
        rh = rng.randint(ROOM_MIN, ROOM_MAX)
        rx = rng.randint(1, max(1, w - rw - 1))
        ry = rng.randint(1, max(1, h - rh - 1))
        for y in range(ry, min(h - 1, ry + rh)):
            for x in range(rx, min(w - 1, rx + rw)):
                grid[y][x] = FREE


def _braid(grid, seed, prob):
    """Open a fraction of dead ends, turning the tree into a graph with loops."""
    if prob <= 0.0:
        return
    rng = random.Random(seed + 202)
    h, w = len(grid), len(grid[0])
    for y in range(1, h - 1):
        for x in range(1, w - 1):
            if grid[y][x] != FREE:
                continue
            nbrs = [(dx, dy) for dx, dy in ORTHO if grid[y + dy][x + dx] != WALL]
            if len(nbrs) != 1 or rng.random() > prob:
                continue
            walls = [
                (dx, dy) for dx, dy in ORTHO
                if grid[y + dy][x + dx] == WALL
                and 1 <= x + 2 * dx < w - 1
                and 1 <= y + 2 * dy < h - 1
            ]
            if walls:
                dx, dy = rng.choice(walls)
                grid[y + dy][x + dx] = FREE


def _add_mud(grid, seed):
    rng = random.Random(seed + 303)
    h, w = len(grid), len(grid[0])
    for _ in range(MUD_BLOBS):
        bw = rng.randint(MUD_MIN, MUD_MAX)
        bh = rng.randint(MUD_MIN, MUD_MAX)
        bx = rng.randint(1, max(1, w - bw - 1))
        by = rng.randint(1, max(1, h - bh - 1))
        for y in range(by, min(h - 1, by + bh)):
            for x in range(bx, min(w - 1, bx + bw)):
                if grid[y][x] == FREE:
                    grid[y][x] = MUD


# ------------------------------------------------------------------- endpoints

def _farthest_free(grid, start):
    """Plain BFS (mud counts as free); return the farthest reachable cell."""
    h, w = len(grid), len(grid[0])
    seen = {start}
    q = deque([start])
    far = start
    while q:
        far = q.popleft()
        cx, cy = far
        for dx, dy in ORTHO:
            n = (cx + dx, cy + dy)
            if (0 <= n[0] < w and 0 <= n[1] < h
                    and grid[n[1]][n[0]] != WALL and n not in seen):
                seen.add(n)
                q.append(n)
    return far


def _place_endpoints_diameter(grid):
    """S and G at the two ends of the longest shortest-path (graph diameter)."""
    h, w = len(grid), len(grid[0])
    seed = next(
        (x, y) for y in range(h) for x in range(w) if grid[y][x] != WALL
    )
    a = _farthest_free(grid, seed)
    b = _farthest_free(grid, a)
    grid[a[1]][a[0]] = "S"
    grid[b[1]][b[0]] = "G"


# ============================================================
# Generate Webots PROTO
# ============================================================

def generate_proto(maze):
    height = len(maze)
    width = len(maze[0])

    maze_width = width * CELL_SIZE
    maze_depth = height * CELL_SIZE

    lines = [
        "#VRML_SIM R2025a utf8",
        "",
        "PROTO Maze [",
        "  field SFVec3f    translation 0 0 0",
        "  field SFRotation rotation    0 0 1 0",
        "]",
        "{",
        "  Solid {",
        "    translation IS translation",
        "    rotation IS rotation",
        "",
        "    children [",
        "      Shape {",
        "        appearance PBRAppearance {",
        "          baseColor 0.7 0.7 0.7",
        "          roughness 1",
        "        }",
        "        geometry Box {",
        f"          size {maze_width:.3f} {maze_depth:.3f} 0.1",
        "        }",
        "      }",
        "",
    ]

    # Mud: flat brown decals just above the floor, no collision. Merge each
    # row's run of '~' cells into one box to keep the node count down.
    for y, row in enumerate(maze):
        x = 0
        while x < width:
            if row[x] != MUD:
                x += 1
                continue
            x0 = x
            while x < width and row[x] == MUD:
                x += 1
            run = x - x0
            cx = (x0 + run / 2) * CELL_SIZE - maze_width / 2
            cy = maze_depth / 2 - (y + 0.5) * CELL_SIZE
            lines += [
                "      Pose {",
                f"        translation {cx:.3f} {cy:.3f} 0.06",
                "        children [",
                "          Shape {",
                "            appearance PBRAppearance {",
                "              baseColor 0.40 0.26 0.13",
                "              roughness 1",
                "              metalness 0",
                "            }",
                "            geometry Box {",
                f"              size {run * CELL_SIZE:.3f} {CELL_SIZE:.3f} 0.02",
                "            }",
                "          }",
                "        ]",
                "      }",
                "",
            ]

    for y, row in enumerate(maze):
        for x, cell in enumerate(row):
            if cell != WALL:
                continue

            # Skip fully-buried walls (all 8 neighbours are wall): no one can
            # ever touch them, and it keeps the proto small.
            if all(
                0 <= x + dx < width and 0 <= y + dy < height
                and maze[y + dy][x + dx] == WALL
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                               (1, 1), (1, -1), (-1, 1), (-1, -1))
            ):
                continue

            world_x = (x + 0.5) * CELL_SIZE - maze_width / 2
            # Webots R2025a is ENU (Z-up): maze lies in X/Y, walls rise in +Z.
            world_y = maze_depth / 2 - (y + 0.5) * CELL_SIZE

            lines += [
                "      Solid {",
                f"        translation {world_x:.3f} {world_y:.3f} "
                f"{WALL_HEIGHT / 2:.3f}",
                "        children [",
                "          Shape {",
                "            appearance PBRAppearance {",
                "              baseColor 0.1 0.1 0.1",
                "              roughness 1",
                "            }",
                "            geometry Box {",
                f"              size {WALL_THICKNESS:.3f} "
                f"{WALL_THICKNESS:.3f} {WALL_HEIGHT:.3f}",
                "            }",
                "          }",
                "        ]",
                "        boundingObject Box {",
                f"          size {WALL_THICKNESS:.3f} "
                f"{WALL_THICKNESS:.3f} {WALL_HEIGHT:.3f}",
                "        }",
                "      }",
                "",
            ]

    lines += [
        "    ]",
        "",
        "    boundingObject Box {",
        f"      size {maze_width:.3f} {maze_depth:.3f} 0.1",
        "    }",
        "  }",
        "}",
    ]
    return "\n".join(lines)


# ============================================================
# Generate 2D occupancy / cost map (PGM + YAML)
# ============================================================

def generate_costmap(maze):
    rows = len(maze)
    cols = len(maze[0])

    ppc = int(round(CELL_SIZE / MAP_RESOLUTION))
    img_w = cols * ppc
    img_h = rows * ppc

    shade = {WALL: 0, MUD: MUD_SHADE}   # everything else -> free (254)
    data = bytearray([254]) * (img_w * img_h)

    for y, row in enumerate(maze):
        for x, cell in enumerate(row):
            if cell not in shade:
                continue
            value = shade[cell]
            for py in range(y * ppc, (y + 1) * ppc):
                base = py * img_w + x * ppc
                data[base:base + ppc] = bytes([value]) * ppc

    header = f"P5\n{img_w} {img_h}\n255\n".encode("ascii")
    pgm = header + bytes(data)

    # Origin = world coordinate of the image's bottom-left corner. The maze is
    # centred on the world origin (matches Maze.proto).
    origin_x = -(cols * CELL_SIZE) / 2.0
    origin_y = -(rows * CELL_SIZE) / 2.0

    yaml = "\n".join([
        f"image: {MAP_OUTPUT.name}",
        f"resolution: {MAP_RESOLUTION}",
        f"origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0]",
        "negate: 0",
        "occupied_thresh: 0.65",
        "free_thresh: 0.15",
        "mode: scale",                 # keep the mid-grey mud as a graded cost
        "",
    ])
    return pgm, yaml


# ============================================================
# Main
# ============================================================

def main():
    maze = generate_maze(WIDTH, HEIGHT, SEED)

    proto = generate_proto(maze)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(proto)

    pgm, yaml = generate_costmap(maze)
    MAP_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    MAP_OUTPUT.write_bytes(pgm)
    MAP_OUTPUT.with_suffix(".yaml").write_text(yaml)

    rows, cols = len(maze), len(maze[0])
    mud = sum(r.count(MUD) for r in maze)
    print(f"Generated {OUTPUT}")
    print(f"Generated {MAP_OUTPUT}")
    print(f"Generated {MAP_OUTPUT.with_suffix('.yaml')}")
    print(f"{cols} x {rows} cells, {CELL_SIZE} m each, {mud} mud cells, seed {SEED}")
    print()
    for row in maze:
        print("".join(row))


if __name__ == "__main__":
    main()

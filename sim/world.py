"""Maze map: a grid of blocked/free cells, plus robot start and goal cell.

World frame: x to the right, y up, angles counter-clockwise from +x, meters.
Cell (i, j) covers x in [i*cell, (i+1)*cell), y in [j*cell, (j+1)*cell).
In the JSON file rows are stored top row first, as they look on screen.
Everything outside the board counts as a wall.
"""
import json
import math
from dataclasses import dataclass
from pathlib import Path

WALL = ord("#")
FREE = ord(".")


@dataclass
class Pose:
    x: float
    y: float
    heading: float  # radians


class World:
    def __init__(self, width: int, height: int, cell: float):
        self.width = width
        self.height = height
        self.cell = cell
        self.cells = [bytearray([FREE]) * width for _ in range(height)]
        self.start = Pose(cell * 1.5, cell * 1.5, math.pi / 2)
        self.goal = (width - 2, height - 2)
        self.version = 0  # bumped on every edit, lets the renderer cache walls

    @property
    def size_m(self):
        return self.width * self.cell, self.height * self.cell

    def in_bounds(self, i: int, j: int) -> bool:
        return 0 <= i < self.width and 0 <= j < self.height

    def is_wall(self, i: int, j: int) -> bool:
        if not self.in_bounds(i, j):
            return True
        return self.cells[j][i] == WALL

    def set_wall(self, i: int, j: int, wall: bool):
        if self.in_bounds(i, j):
            v = WALL if wall else FREE
            if self.cells[j][i] != v:
                self.cells[j][i] = v
                self.version += 1

    def clear(self):
        for row in self.cells:
            row[:] = bytearray([FREE]) * self.width
        self.version += 1

    def cell_at(self, x: float, y: float):
        return math.floor(x / self.cell), math.floor(y / self.cell)

    def raycast(self, ox, oy, dx, dy, max_dist):
        """Grid DDA. Returns (distance, face) where face is 'x' for a hit on a
        vertical wall side and 'y' for a horizontal one, or None."""
        cs = self.cell
        i, j = self.cell_at(ox, oy)
        if self.is_wall(i, j):
            return 0.0, "x"
        inf = float("inf")
        step_i = 1 if dx > 0 else -1
        step_j = 1 if dy > 0 else -1
        if dx != 0:
            t_dx = cs / abs(dx)
            t_x = (((i + 1) * cs - ox) if dx > 0 else (ox - i * cs)) / abs(dx)
        else:
            t_dx = t_x = inf
        if dy != 0:
            t_dy = cs / abs(dy)
            t_y = (((j + 1) * cs - oy) if dy > 0 else (oy - j * cs)) / abs(dy)
        else:
            t_dy = t_y = inf
        while True:
            if t_x < t_y:
                t, face = t_x, "x"
                i += step_i
                t_x += t_dx
            else:
                t, face = t_y, "y"
                j += step_j
                t_y += t_dy
            if t > max_dist:
                return None
            if self.is_wall(i, j):
                return t, face

    def box_hits_wall(self, cx, cy, ux, uy, half_len, half_wid) -> bool:
        """Oriented rectangle (center, forward unit vector, half sizes) against
        wall cells, separating axis test."""
        h = self.cell / 2
        ax, ay = abs(ux), abs(uy)
        ex = ax * half_len + ay * half_wid
        ey = ay * half_len + ax * half_wid
        i0, j0 = self.cell_at(cx - ex, cy - ey)
        i1, j1 = self.cell_at(cx + ex, cy + ey)
        box_on_axis = h * (ax + ay)
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                if not self.is_wall(i, j):
                    continue
                dx = (i + 0.5) * self.cell - cx
                dy = (j + 0.5) * self.cell - cy
                if abs(dx) >= ex + h or abs(dy) >= ey + h:
                    continue
                if abs(dx * ux + dy * uy) >= half_len + box_on_axis:
                    continue
                if abs(-dx * uy + dy * ux) >= half_wid + box_on_axis:
                    continue
                return True
        return False

    # ---------------------------------------------------------------- files

    def to_json(self) -> dict:
        return {
            "version": 1,
            "cell_size_m": self.cell,
            "width": self.width,
            "height": self.height,
            "start": {
                "x": round(self.start.x, 4),
                "y": round(self.start.y, 4),
                "heading_deg": round(math.degrees(self.start.heading), 2),
            },
            "goal": {"col": self.goal[0], "row": self.height - 1 - self.goal[1]},
            "grid": [self.cells[j].decode() for j in reversed(range(self.height))],
        }

    @classmethod
    def from_json(cls, data: dict) -> "World":
        w = cls(int(data["width"]), int(data["height"]), float(data["cell_size_m"]))
        rows = data["grid"]
        if len(rows) != w.height or any(len(r) != w.width for r in rows):
            raise ValueError("grid size does not match width/height")
        for r, line in enumerate(rows):
            j = w.height - 1 - r
            w.cells[j] = bytearray(WALL if ch == WALL else FREE for ch in line.encode())
        s = data.get("start")
        if s:
            w.start = Pose(float(s["x"]), float(s["y"]), math.radians(float(s["heading_deg"])))
        g = data.get("goal")
        if g:
            w.goal = (int(g["col"]), w.height - 1 - int(g["row"]))
        return w

    def save(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(json.dumps(self.to_json(), indent=1) + "\n")
        tmp.replace(path)

    @classmethod
    def load(cls, path: Path) -> "World":
        return cls.from_json(json.loads(path.read_text()))

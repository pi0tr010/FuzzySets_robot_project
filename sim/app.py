"""Top-down view and map editor (pygame)."""
import math
import subprocess
from pathlib import Path

import pygame

from .hardware import Hardware
from .simulation import Simulation
from .world import Pose, World

PANEL_W = 360
HELP_H = 26
SPEEDS = [0.1, 0.25, 0.5, 1, 2, 4, 8, 16]

BG = (24, 26, 30)
FLOOR = (236, 236, 228)
GRID = (214, 214, 206)
WALL_C = (52, 56, 66)
GOAL_C = (120, 200, 120)
START_C = (70, 120, 220)
TRAIL_C = (230, 120, 40)
BODY_C = (40, 140, 200)
BODY_HIT_C = (220, 60, 60)
WHEEL_C = (25, 25, 25)
SERVO_C = (30, 30, 120)
SONAR_C = (60, 170, 255)
TEXT_C = (225, 225, 225)
DIM_C = (150, 150, 150)
EDIT_C = (255, 200, 60)

HELP_RUN = ("SPACE pause  R reset  B rebuild+reset  +/- speed  M max speed  "
            "T clear trail  C cone  E editor  ESC quit")
HELP_EDIT = ("LMB wall  RMB erase  [ ] brush  S start  wheel rotate start  "
             "G goal  X clear  Ctrl+S save  E back")


class App:
    def __init__(self, world: World, map_path: Path, robot_cmd: list[str],
                 firmware_dir: Path, speed: float = 1.0, seed: int = 1,
                 size=(1280, 820), headless: bool = False):
        pygame.init()
        flags = 0 if headless else pygame.RESIZABLE
        self.screen = pygame.display.set_mode(size, flags)
        self.world = world
        self.map_path = map_path
        self.firmware_dir = firmware_dir
        self.hw = Hardware()
        self.sim = Simulation(world, self.hw, robot_cmd, seed)
        self.speed_idx = min(range(len(SPEEDS)), key=lambda k: abs(SPEEDS[k] - speed))
        self.max_speed = False
        self.paused = False
        self.editing = False
        self.show_cone = True
        self.brush = 1
        self.dirty = False
        self.clock = pygame.time.Clock()
        self.font = self._font(15)
        self.font_small = self._font(13)
        self._walls_cache = None
        self._update_title()

    @staticmethod
    def _font(size):
        for name in ("dejavusansmono", "liberationmono", "monospace"):
            path = pygame.font.match_font(name)
            if path:
                return pygame.font.Font(path, size)
        return pygame.font.Font(None, size + 4)

    # ------------------------------------------------------------- geometry

    def _layout(self):
        sw, sh = self.screen.get_size()
        aw, ah = max(100, sw - PANEL_W), max(100, sh - HELP_H)
        wm, hm = self.world.size_m
        scale = min((aw - 20) / wm, (ah - 20) / hm)
        ox = (aw - wm * scale) / 2
        oy = (ah - hm * scale) / 2
        return scale, ox, oy

    def to_screen(self, x, y):
        s, ox, oy = self._layout()
        return ox + x * s, oy + (self.world.size_m[1] - y) * s

    def to_world(self, px, py):
        s, ox, oy = self._layout()
        return (px - ox) / s, self.world.size_m[1] - (py - oy) / s

    # ------------------------------------------------------------- main loop

    def run(self, duration_s=None, screenshot=None):
        running = True
        while running:
            dt = min(self.clock.tick(60) / 1000.0, 0.1)
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    running = self._on_key(ev)
                elif ev.type == pygame.MOUSEWHEEL and self.editing:
                    self._rotate_start(15 * ev.y)
            if self.editing:
                self._edit_mouse()
            elif not self.paused:
                if self.max_speed:
                    self.sim.run(self.sim.t_us + 10_000_000, budget_s=0.014)
                else:
                    step = int(dt * SPEEDS[self.speed_idx] * 1e6)
                    self.sim.run(self.sim.t_us + step, budget_s=0.012)
            self._draw()
            pygame.display.flip()
            if duration_s is not None and self.sim.t_us >= duration_s * 1e6:
                running = False
        if screenshot:
            pygame.image.save(self.screen, str(screenshot))
        if self.dirty:
            print(f"warning: unsaved map changes ({self.map_path})", flush=True)
        self.sim.stop_process()
        pygame.quit()

    def _on_key(self, ev) -> bool:
        k, mods = ev.key, ev.mod
        ctrl = mods & pygame.KMOD_CTRL
        if k == pygame.K_ESCAPE:
            return False
        if k == pygame.K_e:
            self.editing = not self.editing
            return True
        if ctrl and k == pygame.K_s:
            self.world.save(self.map_path)
            self.dirty = False
            self.sim.log("sim", f"saved {self.map_path}")
            self._update_title()
            return True
        if self.editing:
            mx, my = pygame.mouse.get_pos()
            x, y = self.to_world(mx, my)
            if k == pygame.K_s:
                self.world.start = Pose(x, y, self.world.start.heading)
                self._mark_dirty()
            elif k == pygame.K_g:
                cell = self.world.cell_at(x, y)
                if self.world.in_bounds(*cell):
                    self.world.goal = cell
                    self._mark_dirty()
            elif k == pygame.K_x:
                self.world.clear()
                self._mark_dirty()
            elif k == pygame.K_LEFTBRACKET:
                self.brush = max(1, self.brush - 1)
            elif k == pygame.K_RIGHTBRACKET:
                self.brush = min(9, self.brush + 1)
            return True
        if k == pygame.K_SPACE:
            self.paused = not self.paused
        elif k == pygame.K_r:
            self.sim.reset()
        elif k == pygame.K_b:
            self._rebuild()
        elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self.speed_idx = min(len(SPEEDS) - 1, self.speed_idx + 1)
        elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.speed_idx = max(0, self.speed_idx - 1)
        elif k == pygame.K_m:
            self.max_speed = not self.max_speed
        elif k == pygame.K_t:
            self.sim.trail.clear()
            self.sim.trail.append((self.sim.robot.x, self.sim.robot.y))
        elif k == pygame.K_c:
            self.show_cone = not self.show_cone
        return True

    def _rebuild(self):
        self.sim.stop_process()
        res = subprocess.run(["make", "-C", str(self.firmware_dir), "sim"],
                             capture_output=True, text=True)
        out = (res.stdout + res.stderr).strip().splitlines()
        for line in out[-12:]:
            self.sim.log("make", line)
            print(f"[make] {line}", flush=True)
        if res.returncode != 0:
            self.sim.log("make", f"BUILD FAILED ({res.returncode})")
            self.sim.status = "build failed"
            return
        self.sim.reset()

    def _mark_dirty(self):
        self.dirty = True
        self._update_title()

    def _update_title(self):
        star = " *" if self.dirty else ""
        pygame.display.set_caption(f"Maze robot sim - {self.map_path}{star}")

    def _rotate_start(self, deg):
        s = self.world.start
        self.world.start = Pose(s.x, s.y, s.heading + math.radians(deg))
        self._mark_dirty()

    def _edit_mouse(self):
        buttons = pygame.mouse.get_pressed()
        if not (buttons[0] or buttons[2]):
            return
        mx, my = pygame.mouse.get_pos()
        if mx >= self.screen.get_width() - PANEL_W:
            return
        ci, cj = self.world.cell_at(*self.to_world(mx, my))
        r = self.brush // 2
        lo, hi = -r, self.brush - r
        before = self.world.version
        for dj in range(lo, hi):
            for di in range(lo, hi):
                self.world.set_wall(ci + di, cj + dj, bool(buttons[0]))
        if self.world.version != before:
            self._mark_dirty()

    # --------------------------------------------------------------- drawing

    def _draw(self):
        self.screen.fill(BG)
        self._draw_map()
        self._draw_trail()
        self._draw_pings()
        self._draw_robot()
        if self.editing:
            self._draw_start_marker()
            self._draw_brush()
        self._draw_panel()
        self._draw_help()

    def _draw_map(self):
        s, ox, oy = self._layout()
        key = (self.world.version, self.screen.get_size())
        if self._walls_cache is None or self._walls_cache[0] != key:
            wm, hm = self.world.size_m
            surf = pygame.Surface((math.ceil(wm * s) + 1, math.ceil(hm * s) + 1))
            surf.fill(FLOOR)
            cs = self.world.cell * s
            if cs >= 5:
                for i in range(self.world.width + 1):
                    pygame.draw.line(surf, GRID, (i * cs, 0), (i * cs, hm * s))
                for j in range(self.world.height + 1):
                    pygame.draw.line(surf, GRID, (0, j * cs), (wm * s, j * cs))
            for j in range(self.world.height):
                for i in range(self.world.width):
                    if self.world.is_wall(i, j):
                        r = pygame.Rect(round(i * cs), round((self.world.height - 1 - j) * cs),
                                        math.ceil(cs), math.ceil(cs))
                        surf.fill(WALL_C, r)
            self._walls_cache = (key, surf)
        self.screen.blit(self._walls_cache[1], (ox, oy))
        gi, gj = self.world.goal
        cs = self.world.cell * s
        gx, gy = self.to_screen(gi * self.world.cell, (gj + 1) * self.world.cell)
        g = pygame.Surface((math.ceil(cs), math.ceil(cs)), pygame.SRCALPHA)
        g.fill((*GOAL_C, 170))
        self.screen.blit(g, (gx, gy))
        wm, hm = self.world.size_m
        pygame.draw.rect(self.screen, WALL_C, (ox - 3, oy - 3, wm * s + 6, hm * s + 6), 3)
        scale_px = 0.5 * s
        y0 = oy + hm * s + 6
        pygame.draw.line(self.screen, DIM_C, (ox, y0), (ox + scale_px, y0), 2)
        self.screen.blit(self.font_small.render("50 cm", True, DIM_C), (ox + scale_px + 6, y0 - 8))

    def _draw_trail(self):
        if len(self.sim.trail) > 1:
            pts = [self.to_screen(x, y) for x, y in self.sim.trail]
            pygame.draw.lines(self.screen, TRAIL_C, False, pts, 2)

    def _draw_pings(self):
        now = self.sim.t_us
        sp = self.hw.sonar
        for p in self.sim.pings:
            age = (now - p.t_us) / 1e6
            if p.hit is None or age > 3:
                continue
            fade = max(0, 1 - age / 3)
            col = (int(255 * fade + 236 * (1 - fade)), int(50 * fade + 236 * (1 - fade)),
                   int(50 * fade + 228 * (1 - fade)))
            pygame.draw.circle(self.screen, col, self.to_screen(*p.hit), 3)
        if not self.show_cone or not self.sim.pings:
            return
        p = self.sim.pings[-1]
        if (now - p.t_us) > 300_000:
            return
        length = p.true_dist if p.true_dist is not None else sp.max_range
        half = math.radians(sp.beam_deg) / 2
        ox, oy = p.origin
        pts = [self.to_screen(ox, oy)]
        for k in range(9):
            a = p.heading - half + 2 * half * k / 8
            pts.append(self.to_screen(ox + math.cos(a) * length, oy + math.sin(a) * length))
        cone = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        pygame.draw.polygon(cone, (*SONAR_C, 70), pts)
        self.screen.blit(cone, (0, 0))
        if p.hit:
            pygame.draw.line(self.screen, SONAR_C, pts[0], self.to_screen(*p.hit), 1)

    def _rect_pts(self, cx, cy, ux, uy, half_len, half_wid):
        vx, vy = -uy, ux
        return [self.to_screen(cx + ux * a * half_len + vx * b * half_wid,
                               cy + uy * a * half_len + vy * b * half_wid)
                for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))]

    def _draw_robot_at(self, x, y, heading, servo_deg, color, alpha=255):
        r = self.sim.robot
        ch = self.hw.chassis
        layer = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        cx, cy, ux, uy = r.footprint(x, y, heading)
        vx, vy = -uy, ux
        pygame.draw.polygon(layer, (*color, alpha),
                            self._rect_pts(cx, cy, ux, uy, ch.length / 2, ch.width / 2))
        for side in (1, -1):
            wx = x + vx * side * ch.track / 2
            wy = y + vy * side * ch.track / 2
            pygame.draw.polygon(layer, (*WHEEL_C, alpha),
                                self._rect_pts(wx, wy, ux, uy, ch.wheel_diameter / 2, ch.wheel_width / 2))
        s = self.hw.servo
        d = ch.axle_from_front - s.axis_from_front
        ax, ay = x + ux * d, y + uy * d
        beam = heading + math.radians(servo_deg - 90)
        bx, by = math.cos(beam), math.sin(beam)
        pygame.draw.polygon(layer, (*SERVO_C, alpha),
                            self._rect_pts(ax + bx * s.sensor_offset, ay + by * s.sensor_offset,
                                           bx, by, 0.008, 0.023))
        pygame.draw.circle(layer, (255, 255, 255, alpha), self.to_screen(ax, ay), 2)
        self.screen.blit(layer, (0, 0))

    def _draw_robot(self):
        r = self.sim.robot
        self._draw_robot_at(r.x, r.y, r.heading, r.servo_deg,
                            BODY_HIT_C if r.touching else BODY_C)

    def _draw_start_marker(self):
        st = self.world.start
        r = self.sim.robot
        bad = r.collides_at(st.x, st.y, st.heading)
        self._draw_robot_at(st.x, st.y, st.heading, 90, BODY_HIT_C if bad else START_C, 120)

    def _draw_brush(self):
        mx, my = pygame.mouse.get_pos()
        if mx >= self.screen.get_width() - PANEL_W:
            return
        ci, cj = self.world.cell_at(*self.to_world(mx, my))
        r = self.brush // 2
        cs = self.world.cell
        x0, y0 = self.to_screen((ci - r) * cs, (cj - r + self.brush) * cs)
        s, _, _ = self._layout()
        pygame.draw.rect(self.screen, EDIT_C, (x0, y0, self.brush * cs * s, self.brush * cs * s), 1)

    def _draw_panel(self):
        sw, sh = self.screen.get_size()
        x0 = sw - PANEL_W
        pygame.draw.rect(self.screen, (34, 37, 43), (x0, 0, PANEL_W, sh))
        sim, r = self.sim, self.sim.robot
        last = sim.pings[-1] if sim.pings else None
        mode = "EDITOR" if self.editing else ("PAUSED" if self.paused else "RUN")
        speed = "max" if self.max_speed else f"{SPEEDS[self.speed_idx]}x"
        if last is None:
            sonar = "-"
        elif last.echo_us == 0:
            sonar = "no echo"
        else:
            sonar = f"{last.echo_us} us = {last.echo_us / 58:.1f} cm"
        goal = f"{sim.goal_time_us / 1e6:.2f} s" if sim.goal_time_us is not None else "not yet"
        lines = [
            (f"{mode}   speed {speed}", EDIT_C if self.editing else TEXT_C),
            (f"time     {sim.t_us / 1e6:9.3f} s", TEXT_C),
            (f"robot    {sim.status}", TEXT_C),
            ("", TEXT_C),
            (f"PWM L/R  {r.pwm_l:5d} {r.pwm_r:5d}", TEXT_C),
            (f"v L/R    {r.wheel_speed(r.pwm_l):5.2f} {r.wheel_speed(r.pwm_r):5.2f} m/s", TEXT_C),
            (f"servo    {r.servo_deg:6.1f} -> {r.servo_target:5.1f} deg", TEXT_C),
            (f"sonar    {sonar}", TEXT_C),
            (f"collis.  {r.collisions}" + ("  TOUCHING" if r.touching else ""),
             BODY_HIT_C if r.touching else TEXT_C),
            (f"goal     {goal}", GOAL_C if sim.goal_time_us is not None else TEXT_C),
            ("", TEXT_C),
            (f"pose     x {r.x:.3f} y {r.y:.3f} m", DIM_C),
            (f"         heading {math.degrees(r.heading):7.1f} deg", DIM_C),
            (f"map      {self.world.width}x{self.world.height} cells, {self.world.cell * 100:.0f} cm",
             DIM_C),
        ]
        if self.editing:
            lines.append((f"brush    {self.brush}x{self.brush}", EDIT_C))
        y = 10
        for text, col in lines:
            self.screen.blit(self.font.render(text, True, col), (x0 + 12, y))
            y += 20
        y += 6
        self.screen.blit(self.font.render("log", True, DIM_C), (x0 + 12, y))
        y += 20
        max_lines = max(0, (sh - HELP_H - y - 6) // 16)
        tail = list(sim.logs)[-max_lines:] if max_lines else []
        colors = {"robot": TEXT_C, "sim": (150, 200, 255), "stderr": (255, 170, 120),
                  "make": (200, 200, 120)}
        for ln in tail:
            text = f"{ln.t_us / 1e6:7.2f} {ln.text}"
            max_chars = (PANEL_W - 20) // 8
            self.screen.blit(self.font_small.render(text[:max_chars], True, colors.get(ln.source, DIM_C)),
                             (x0 + 12, y))
            y += 16

    def _draw_help(self):
        sw, sh = self.screen.get_size()
        pygame.draw.rect(self.screen, (34, 37, 43), (0, sh - HELP_H, sw, HELP_H))
        text = HELP_EDIT if self.editing else HELP_RUN
        self.screen.blit(self.font_small.render(text, True, DIM_C), (10, sh - HELP_H + 6))

"""Robot body: differential drive, servo head and HC-SR04 model."""
import math
import random
from dataclasses import dataclass

from .hardware import Hardware
from .world import Pose, World


@dataclass
class PingResult:
    t_us: int
    origin: tuple          # sensor position (x, y)
    heading: float         # beam direction, radians
    hit: tuple | None      # point that produced the echo
    true_dist: float | None
    echo_us: int           # what the robot gets, 0 = no echo


class Robot:
    def __init__(self, hw: Hardware, world: World, start: Pose):
        self.hw = hw
        self.world = world
        self.x, self.y, self.heading = start.x, start.y, start.heading
        self.pwm_l = 0
        self.pwm_r = 0
        self.servo_deg = hw.servo.initial_deg
        self.servo_target = hw.servo.initial_deg
        self.touching = False
        self.collisions = 0
        ch = hw.chassis
        self.half_len = ch.length / 2
        self.half_wid = max(ch.width, ch.track + ch.wheel_width) / 2
        self.center_offset = ch.axle_from_front - ch.length / 2

    # ------------------------------------------------------------- actuators

    def wheel_speed(self, pwm: int) -> float:
        m = self.hw.motor
        pwm = max(-m.pwm_max, min(m.pwm_max, pwm))
        if abs(pwm) <= m.deadband_pwm:
            return 0.0
        volts = (m.supply_voltage - m.driver_drop_v) * abs(pwm) / m.pwm_max
        rpm = m.rated_rpm * volts / m.rated_voltage
        v = rpm / 60.0 * math.pi * self.hw.chassis.wheel_diameter
        return math.copysign(v, pwm)

    def set_motors(self, left: int, right: int):
        self.pwm_l, self.pwm_r = left, right

    def set_servo(self, deg: float):
        s = self.hw.servo
        self.servo_target = max(s.min_deg, min(s.max_deg, deg))

    # --------------------------------------------------------------- physics

    def footprint(self, x, y, heading):
        ux, uy = math.cos(heading), math.sin(heading)
        cx = x + ux * self.center_offset
        cy = y + uy * self.center_offset
        return cx, cy, ux, uy

    def collides_at(self, x, y, heading) -> bool:
        cx, cy, ux, uy = self.footprint(x, y, heading)
        return self.world.box_hits_wall(cx, cy, ux, uy, self.half_len, self.half_wid)

    def step(self, dt: float):
        s = self.hw.servo
        diff = self.servo_target - self.servo_deg
        max_move = s.speed_deg_s * dt
        self.servo_deg += max(-max_move, min(max_move, diff))

        vl = self.wheel_speed(self.pwm_l)
        vr = self.wheel_speed(self.pwm_r)
        if vl == 0 and vr == 0:
            return
        v = (vl + vr) / 2
        w = (vr - vl) / self.hw.chassis.track
        th = self.heading
        if abs(w) < 1e-9:
            nx = self.x + v * math.cos(th) * dt
            ny = self.y + v * math.sin(th) * dt
            nth = th
        else:
            nth = th + w * dt
            nx = self.x + v / w * (math.sin(nth) - math.sin(th))
            ny = self.y - v / w * (math.cos(nth) - math.cos(th))
        if self.collides_at(nx, ny, nth):
            if not self.touching:
                self.collisions += 1
            self.touching = True
            return
        self.touching = False
        self.x, self.y = nx, ny
        self.heading = math.atan2(math.sin(nth), math.cos(nth))

    # ---------------------------------------------------------------- sensor

    def sensor_pose(self):
        s = self.hw.servo
        ux, uy = math.cos(self.heading), math.sin(self.heading)
        d = self.hw.chassis.axle_from_front - s.axis_from_front
        ax, ay = self.x + ux * d, self.y + uy * d
        beam = self.heading + math.radians(self.servo_deg - 90.0)
        sx = ax + math.cos(beam) * s.sensor_offset
        sy = ay + math.sin(beam) * s.sensor_offset
        return (ax, ay), (sx, sy), beam

    def ping(self, t_us: int, rng: random.Random) -> PingResult:
        sp = self.hw.sonar
        _, (sx, sy), beam = self.sensor_pose()
        half = math.radians(sp.beam_deg) / 2
        max_inc = math.radians(sp.max_incidence_deg)
        best = None
        n = max(1, sp.rays)
        for k in range(n):
            a = beam if n == 1 else beam - half + 2 * half * k / (n - 1)
            dx, dy = math.cos(a), math.sin(a)
            hit = self.world.raycast(sx, sy, dx, dy, sp.max_range * 1.2)
            if hit is None:
                continue
            dist, face = hit
            incidence = math.acos(min(1.0, abs(dx) if face == "x" else abs(dy)))
            if incidence > max_inc:
                continue
            if best is None or dist < best[0]:
                best = (dist, (sx + dx * dist, sy + dy * dist))

        if best is None:
            return PingResult(t_us, (sx, sy), beam, None, None, 0)
        dist, point = best
        measured = dist + rng.gauss(0.0, sp.noise_abs + sp.noise_rel * dist)
        measured = max(sp.min_range, measured)
        if measured > sp.max_range:
            return PingResult(t_us, (sx, sy), beam, point, dist, 0)
        echo_us = int(round(2 * measured / sp.speed_of_sound * 1e6))
        return PingResult(t_us, (sx, sy), beam, point, dist, echo_us)

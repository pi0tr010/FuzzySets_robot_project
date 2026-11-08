"""Lockstep simulation: the simulator owns the clock, the robot program only
sees time pass through HAL calls. Time the robot spends computing between
calls is not counted (except a fixed cost per call)."""
import random
import time
from collections import deque
from dataclasses import dataclass

from .bridge import EOF, RobotProcess
from .hardware import PHYSICS_STEP_US, Hardware
from .robot import PingResult, Robot
from .world import World

TRAIL_EVERY_US = 20_000


@dataclass
class LogLine:
    t_us: int
    source: str
    text: str


class Simulation:
    def __init__(self, world: World, hw: Hardware, robot_cmd: list[str], seed: int = 1):
        self.world = world
        self.hw = hw
        self.robot_cmd = robot_cmd
        self.seed = seed
        self.logs: deque[LogLine] = deque(maxlen=300)
        self.proc: RobotProcess | None = None
        self.reset()

    # ------------------------------------------------------------- lifecycle

    def reset(self):
        self.stop_process()
        self.rng = random.Random(self.seed)
        self.robot = Robot(self.hw, self.world, self.world.start)
        self.t_us = 0
        self.phys_us = 0
        self.pending = None           # (end_time_us, reply or None)
        self.pings: deque[PingResult] = deque(maxlen=60)
        self.last_trigger_us = None
        self.trail = deque([(self.robot.x, self.robot.y)], maxlen=20000)
        self.next_trail_us = TRAIL_EVERY_US
        self.goal_time_us = None
        self.status = "starting"
        self.warned_ping_rate = False
        self.log("sim", "reset")
        try:
            self.proc = RobotProcess(self.robot_cmd)
            self.status = "running"
        except OSError as e:
            self.proc = None
            self.status = f"cannot start robot: {e}"
            self.log("sim", self.status)

    def stop_process(self):
        if self.proc:
            self.proc.kill()
            self._drain_stderr()
            self.proc = None

    def log(self, source: str, text: str):
        self.logs.append(LogLine(self.t_us, source, text))

    def _drain_stderr(self):
        if self.proc:
            q = self.proc.stderr_lines
            while q:
                line = q.popleft()
                self.log("stderr", line)
                print(f"[robot] {line}", flush=True)

    # --------------------------------------------------------------- physics

    def _advance(self, target_us: int):
        r = self.robot
        while self.phys_us + PHYSICS_STEP_US <= target_us:
            r.step(PHYSICS_STEP_US / 1e6)
            self.phys_us += PHYSICS_STEP_US
            if self.phys_us >= self.next_trail_us:
                self.next_trail_us += TRAIL_EVERY_US
                last = self.trail[-1]
                if abs(last[0] - r.x) + abs(last[1] - r.y) > 0.002:
                    self.trail.append((r.x, r.y))
            if self.goal_time_us is None and self.world.cell_at(r.x, r.y) == self.world.goal:
                self.goal_time_us = self.phys_us
                self.log("sim", f"GOAL reached at {self.phys_us / 1e6:.2f} s")
        self.t_us = max(self.t_us, target_us)

    # ------------------------------------------------------------- main loop

    def run(self, until_us: int, budget_s: float):
        """Advance simulated time up to until_us, spending at most budget_s
        of real time waiting for the robot program."""
        deadline = time.perf_counter() + budget_s
        while self.t_us < until_us:
            if self.pending:
                end, reply = self.pending
                if end > until_us:
                    self._advance(until_us)
                    break
                self._advance(end)
                self.pending = None
                if reply is not None and self.proc:
                    self.proc.send(reply)
                continue
            if self.proc is None:
                self._advance(until_us)
                break
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                break
            line = self.proc.read_line(remaining)
            if line is None:
                break
            if line is EOF:
                code = self.proc.proc.wait()
                self._drain_stderr()
                self.status = f"robot exited (code {code})"
                self.log("sim", self.status)
                self.proc = None
                continue
            self._handle(line)
        self._drain_stderr()

    def _cost(self):
        self._advance(self.t_us + self.hw.mcu.call_cost_us)

    def _handle(self, line: str):
        cmd, _, arg = line.partition(" ")
        p = self.proc
        try:
            if cmd == "MS":
                self._cost()
                p.send(f"T {(self.t_us // 1000) & 0xFFFFFFFF}")
            elif cmd == "US":
                self._cost()
                p.send(f"T {self.t_us & 0xFFFFFFFF}")
            elif cmd == "DLY":
                self.pending = (self.t_us + int(arg) * 1000, "OK")
            elif cmd == "DLYU":
                self.pending = (self.t_us + int(arg), "OK")
            elif cmd == "MOT":
                left, right = (int(v) for v in arg.split())
                self._cost()
                self.robot.set_motors(left, right)
                p.send("OK")
            elif cmd == "SRV":
                self._cost()
                self.robot.set_servo(float(arg))
                p.send("OK")
            elif cmd == "PING":
                self._ping()
            elif cmd == "LOG":
                self.log("robot", arg)
            elif cmd == "HELLO":
                p.send("OK")
            else:
                raise ValueError("unknown command")
        except ValueError as e:
            self.log("sim", f"bad request '{line}': {e}")
            p.send("ERR")

    def _ping(self):
        sp = self.hw.sonar
        # Cycle is trigger to trigger; 1 ms slack for millis() truncation.
        if (self.last_trigger_us is not None and not self.warned_ping_rate
                and self.t_us - self.last_trigger_us < (sp.min_cycle_ms - 1) * 1000):
            self.warned_ping_rate = True
            self.log("sim", f"warning: pings closer than {sp.min_cycle_ms} ms")
        self.last_trigger_us = self.t_us
        start = self.t_us + sp.trigger_overhead_us
        self._advance(start)
        res = self.robot.ping(self.t_us, self.rng)
        self.pings.append(res)
        end = start + (res.echo_us if res.echo_us else sp.timeout_us)
        self.pending = (end, f"PONG {res.echo_us}")

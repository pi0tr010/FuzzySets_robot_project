"""Child process running the C robot program, line protocol on stdin/stdout."""
import os
import select
import subprocess
import threading
from collections import deque

EOF = object()


class RobotProcess:
    def __init__(self, cmd: list[str]):
        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self._fd = self.proc.stdout.fileno()
        self._buf = b""
        self.stderr_lines: deque[str] = deque()
        threading.Thread(target=self._pump_stderr, daemon=True).start()

    def _pump_stderr(self):
        for raw in iter(self.proc.stderr.readline, b""):
            self.stderr_lines.append(raw.decode(errors="replace").rstrip())

    def read_line(self, timeout: float):
        """Next request line, None if nothing arrived in time, EOF if the
        program ended."""
        while b"\n" not in self._buf:
            ready, _, _ = select.select([self._fd], [], [], max(0.0, timeout))
            if not ready:
                return None
            chunk = os.read(self._fd, 65536)
            if not chunk:
                return EOF
            self._buf += chunk
            timeout = 0.0
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode(errors="replace").rstrip("\r")

    def send(self, text: str):
        try:
            self.proc.stdin.write(text.encode() + b"\n")
        except (BrokenPipeError, OSError):
            pass

    def exit_code(self):
        return self.proc.poll()

    def kill(self):
        # Closing stdin makes the HAL exit on its own; kill only if it hangs
        # in robot code that never calls the HAL.
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        try:
            self.proc.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            try:
                self.proc.kill()
                self.proc.wait(timeout=1)
            except (OSError, subprocess.TimeoutExpired):
                pass
        for f in (self.proc.stdout, self.proc.stderr):
            try:
                f.close()
            except OSError:
                pass

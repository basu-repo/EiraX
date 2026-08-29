"""Read an entity pose from Gazebo without feeding it to flight control."""

from __future__ import annotations

import re
import subprocess
import threading
import time
from collections import deque


class GazeboPoseReader:
    """Continuously sample one model from a Gazebo Pose_V topic."""

    def __init__(self, world_name: str, model_name: str = "x500_0") -> None:
        self._model_name = model_name
        self._latest: tuple[float, float, float, float, float] | None = None
        self._history: deque[tuple[float, float, float, float]] = deque(maxlen=10_000)
        self._lock = threading.Lock()
        self._process = subprocess.Popen(
            ["gz", "topic", "-e", "-t", f"/world/{world_name}/pose/info"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self) -> None:
        assert self._process.stdout is not None
        block: list[str] = []
        depth = 0
        stamp_seconds = 0.0
        stamp_sec: int | None = None
        for line in self._process.stdout:
            stripped = line.strip()
            if not block:
                if stripped.startswith("sec:"):
                    stamp_sec = int(stripped.split(":", 1)[1])
                elif stripped.startswith("nsec:") and stamp_sec is not None:
                    stamp_seconds = stamp_sec + int(stripped.split(":", 1)[1]) / 1e9
                if stripped != "pose {":
                    continue
                block = [line]
                depth = 1
                continue
            block.append(line)
            depth += line.count("{") - line.count("}")
            if depth != 0:
                continue
            text = "".join(block)
            block = []
            if f'name: "{self._model_name}"' not in text:
                continue
            position_match = re.search(
                r"position\s*\{[^}]*?x:\s*([-+0-9.eE]+)[^}]*?y:\s*([-+0-9.eE]+)[^}]*?z:\s*([-+0-9.eE]+)",
                text,
                re.DOTALL,
            )
            if position_match:
                with self._lock:
                    self._latest = (
                        float(position_match.group(1)),
                        float(position_match.group(2)),
                        float(position_match.group(3)),
                        stamp_seconds,
                        time.monotonic(),
                    )
                    self._history.append(
                        (stamp_seconds, self._latest[0], self._latest[1], self._latest[2])
                    )

    def wait(self, timeout: float = 15.0) -> tuple[float, float, float, float, float]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if pose := self.latest():
                return pose
            time.sleep(0.05)
        raise TimeoutError(f"No Gazebo ground-truth pose received for {self._model_name}")

    def latest(self) -> tuple[float, float, float, float, float] | None:
        with self._lock:
            return self._latest

    def at(self, simulation_time: float) -> tuple[float, float, float, float] | None:
        """Interpolate ground truth at a PX4 simulation timestamp."""
        with self._lock:
            history = list(self._history)
        if not history or simulation_time < history[0][0] or simulation_time > history[-1][0]:
            return None
        for earlier, later in zip(reversed(history[:-1]), reversed(history[1:])):
            if earlier[0] <= simulation_time <= later[0]:
                span = later[0] - earlier[0]
                ratio = 0.0 if span <= 0.0 else (simulation_time - earlier[0]) / span
                return (
                    earlier[1] + ratio * (later[1] - earlier[1]),
                    earlier[2] + ratio * (later[2] - earlier[2]),
                    earlier[3] + ratio * (later[3] - earlier[3]),
                    simulation_time,
                )
        return None

    def close(self) -> None:
        self._process.terminate()
        try:
            self._process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self._process.kill()
        self._thread.join(timeout=2)

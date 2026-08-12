"""Process management for the cooperative simulation.

Child processes receive a parent-death signal so an editor or terminal crash
cannot leave Gazebo, PX4, Nav2, RTAB-Map, or rosbag running unattended.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
from typing import IO


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen[str]
    log_handle: object


def _terminate_when_parent_dies() -> None:
    libc = ctypes.CDLL(None)
    libc.prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG


class ProcessManager:
    def __init__(self, log_dir: Path, env: dict[str, str]) -> None:
        self.log_dir = log_dir
        self.env = env
        self.processes: list[ManagedProcess] = []

    def start(
        self,
        name: str,
        command: list[str],
        *,
        cwd: Path | None = None,
        stdin: int | IO[bytes] | None = subprocess.DEVNULL,
        env: dict[str, str] | None = None,
    ) -> ManagedProcess:
        log_handle = (self.log_dir / f"{name}.log").open("w", encoding="utf-8")
        process = subprocess.Popen(
            command,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            env=self.env if env is None else env,
            cwd=cwd,
            stdin=stdin,
            start_new_session=True,
            preexec_fn=_terminate_when_parent_dies,
        )
        managed = ManagedProcess(name, process, log_handle)
        self.processes.append(managed)
        return managed

    def failures(self, ignored: set[str] | None = None) -> list[tuple[str, int]]:
        ignored = ignored or set()
        return [
            (item.name, code)
            for item in self.processes
            if item.name not in ignored
            and (code := item.process.poll()) is not None
            and code != 0
        ]

    @staticmethod
    def _stop(item: ManagedProcess) -> None:
        if item.process.poll() is None:
            os.killpg(item.process.pid, signal.SIGINT)
        try:
            item.process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(item.process.pid, signal.SIGTERM)
            try:
                item.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(item.process.pid, signal.SIGKILL)
                item.process.wait(timeout=3)
        if not item.log_handle.closed:
            item.log_handle.close()

    def stop(self, name: str) -> None:
        for index in range(len(self.processes) - 1, -1, -1):
            item = self.processes[index]
            if item.name == name:
                self._stop(item)
                self.processes.pop(index)
                return

    def stop_all(self) -> None:
        for item in reversed(self.processes):
            self._stop(item)

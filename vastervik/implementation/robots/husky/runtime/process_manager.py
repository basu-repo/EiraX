"""Manage standalone UGV subprocesses and their logs."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os
import signal
import subprocess
from typing import IO


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen[str]
    log_handle: object


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
    ) -> ManagedProcess:
        log_handle = (self.log_dir / f"{name}.log").open("w", encoding="utf-8")
        process = subprocess.Popen(
            command,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            env=self.env,
            cwd=cwd,
            stdin=stdin,
            start_new_session=True,
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
        for item in reversed(self.processes):
            if item.name == name:
                self._stop(item)
                return

    def stop_all(self) -> None:
        for item in reversed(self.processes):
            self._stop(item)

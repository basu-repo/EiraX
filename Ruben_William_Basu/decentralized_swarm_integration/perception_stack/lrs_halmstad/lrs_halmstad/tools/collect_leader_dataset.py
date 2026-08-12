#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import yaml
from ament_index_python.packages import get_package_share_directory


def _coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _shell_join(parts: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in parts)


def _require_dict(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _require_list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{label} must be a list")
    return value


def _safe_name(value: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return safe.strip("._-") or "unknown"


@dataclass(frozen=True)
class CaptureConfig:
    uav_name: str
    hz: float
    save_overlay: bool
    target_pose_topic: str


@dataclass(frozen=True)
class StartupConfig:
    warmup_s: float


@dataclass(frozen=True)
class Scenario:
    name: str
    waypoint: str
    nav2_goals: str


@dataclass(frozen=True)
class Manifest:
    world: str
    output_dir: str
    capture: CaptureConfig
    startup: StartupConfig
    scenarios: list[Scenario]


class CollectorStopped(RuntimeError):
    pass


class LeaderDatasetCollector:
    def __init__(self, argv: list[str]) -> None:
        self.package_share = Path(get_package_share_directory("lrs_halmstad"))
        self.ws_root = Path(os.environ.get("LRS_HALMSTAD_WS_ROOT", os.getcwd())).resolve()
        self.state_dir = Path("/tmp/halmstad_ws")
        self.tmux_state_dir = self.state_dir / "tmux_sessions"

        self.world_arg: Optional[str] = None
        self.manifest_arg: Optional[str] = None
        self.output_override: Optional[str] = None
        self.scenario_filter_names: list[str] = []
        self.dry_run = False
        self.once = False
        self.poll_s = 2.0

        self._stop_requested = False
        self._active_capture: Optional[subprocess.Popen[str]] = None
        self._summary_path: Optional[Path] = None
        self._summary: dict[str, Any] = {
            "started_at": _utc_now_iso(),
            "mode": "external_collector",
            "dry_run": False,
            "manifest_path": "",
            "output_root": "",
            "scenario_filter": [],
            "runs": [],
        }

        self._parse_args(argv)
        self.manifest_path = self._resolve_manifest_path()
        self.manifest = self._load_manifest(self.manifest_path)
        self.selected_scenarios = self._select_scenarios(self.manifest.scenarios)
        self.world = self._resolve_world()
        self.output_root = self._resolve_output_root()

        self._summary["dry_run"] = self.dry_run
        self._summary["manifest_path"] = str(self.manifest_path)
        self._summary["output_root"] = str(self.output_root)
        self._summary["scenario_filter"] = self.scenario_filter_names

        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

    def _parse_args(self, argv: list[str]) -> None:
        args = list(argv)
        if args and ":=" not in args[0] and "=" not in args[0]:
            self.world_arg = args.pop(0)

        for arg in args:
            if arg.startswith("manifest:="):
                self.manifest_arg = arg.split(":=", 1)[1]
            elif arg.startswith("out:="):
                self.output_override = arg.split(":=", 1)[1]
            elif arg.startswith("route:=") or arg.startswith("scenario:="):
                self.scenario_filter_names.append(arg.split(":=", 1)[1].strip())
            elif arg.startswith("routes:=") or arg.startswith("scenarios:="):
                raw = arg.split(":=", 1)[1]
                self.scenario_filter_names.extend(item.strip() for item in raw.split(","))
            elif arg.startswith("dry_run:="):
                self.dry_run = _coerce_bool(arg.split(":=", 1)[1])
            elif arg.startswith("once:="):
                self.once = _coerce_bool(arg.split(":=", 1)[1])
            elif arg.startswith("poll_s:="):
                self.poll_s = max(0.2, float(arg.split(":=", 1)[1]))
            elif arg.startswith("launcher:="):
                launcher = arg.split(":=", 1)[1].strip()
                if launcher not in ("", "external", "manual", "collector"):
                    raise ValueError(
                        "collect_leader_dataset no longer starts Nav2/localization. "
                        "Start nav2_tuning separately and run this collector without launcher:=..."
                    )
            else:
                raise ValueError(
                    "Unknown argument: "
                    f"{arg}\nUsage: ./run.sh collect_leader_dataset [world] "
                    "[manifest:=path.yaml] [out:=datasets/baylands_leader_routes] "
                    "[route:=name] [routes:=a,b] [once:=true|false] [dry_run:=true|false]"
                )

    def _resolve_manifest_path(self) -> Path:
        source_default_path = (
            self.ws_root / "src" / "lrs_halmstad" / "config" / "baylands_leader_dataset_manifest.yaml"
        )
        default_path = (
            source_default_path
            if source_default_path.is_file()
            else self.package_share / "config" / "baylands_leader_dataset_manifest.yaml"
        )
        raw = self.manifest_arg
        if not raw:
            return default_path

        raw_path = Path(os.path.expanduser(raw))
        candidates = []
        if raw_path.is_absolute():
            candidates.append(raw_path)
        else:
            candidates.append((Path.cwd() / raw_path).resolve())
            candidates.append((self.ws_root / raw_path).resolve())
            candidates.append((self.package_share / "config" / raw_path).resolve())

        for candidate in candidates:
            if candidate.is_file():
                return candidate
        raise FileNotFoundError(f"Manifest not found: {raw}")

    def _load_manifest(self, path: Path) -> Manifest:
        with path.open("r", encoding="utf-8") as handle:
            raw_data = yaml.safe_load(handle) or {}

        data = _require_dict(raw_data, "manifest")
        capture = _require_dict(data.get("capture", {}), "capture")
        startup = _require_dict(data.get("startup", {}), "startup")
        scenarios_raw = _require_list(data.get("scenarios", []), "scenarios")

        scenarios = []
        for index, item in enumerate(scenarios_raw):
            scenario_data = _require_dict(item, "scenario")
            scenarios.append(
                Scenario(
                    name=str(scenario_data.get("name", "")).strip() or f"scenario_{index + 1}",
                    waypoint=str(scenario_data.get("waypoint", "")).strip(),
                    nav2_goals=str(scenario_data.get("nav2_goals", "")).strip(),
                )
            )

        manifest = Manifest(
            world=str(data.get("world", "baylands")).strip() or "baylands",
            output_dir=str(data.get("output_dir", "datasets/baylands_leader_routes")).strip()
            or "datasets/baylands_leader_routes",
            capture=CaptureConfig(
                uav_name=str(capture.get("uav_name", "dji0")).strip() or "dji0",
                hz=float(capture.get("hz", 0.25)),
                save_overlay=_coerce_bool(capture.get("save_overlay", False)),
                target_pose_topic=str(capture.get("target_pose_topic", "/a201_0000/ground_truth/odom")).strip()
                or "/a201_0000/ground_truth/odom",
            ),
            startup=StartupConfig(warmup_s=max(0.0, float(startup.get("warmup_s", 0.0)))),
            scenarios=scenarios,
        )
        self._validate_manifest(manifest)
        return manifest

    def _validate_manifest(self, manifest: Manifest) -> None:
        if manifest.world != "baylands":
            raise ValueError("collect_leader_dataset currently supports world=baylands only")
        if manifest.capture.hz <= 0.0:
            raise ValueError("capture.hz must be > 0")
        if not manifest.scenarios:
            raise ValueError("manifest must contain at least one scenario")
        for scenario in manifest.scenarios:
            if not scenario.name:
                raise ValueError("scenario name must not be empty")

    def _resolve_world(self) -> str:
        if self.world_arg and self.world_arg != self.manifest.world:
            raise ValueError(
                f"World mismatch: command requested '{self.world_arg}' but manifest uses '{self.manifest.world}'"
            )
        return self.world_arg or self.manifest.world

    def _select_scenarios(self, scenarios: list[Scenario]) -> list[Scenario]:
        requested = [item for item in self.scenario_filter_names if item]
        if not requested:
            return scenarios

        selected: list[Scenario] = []
        selected_names: set[str] = set()
        for name in requested:
            match = next(
                (
                    scenario
                    for scenario in scenarios
                    if scenario.name == name or scenario.nav2_goals == name or scenario.waypoint == name
                ),
                None,
            )
            if match is None:
                valid = ", ".join(scenario.name for scenario in scenarios)
                raise ValueError(f"Unknown route/scenario '{name}'. Valid scenarios: {valid}")
            if match.name not in selected_names:
                selected.append(match)
                selected_names.add(match.name)
        return selected

    def _resolve_output_root(self) -> Path:
        raw = self.output_override or self.manifest.output_dir
        path = Path(os.path.expanduser(raw))
        if not path.is_absolute():
            path = (self.ws_root / path).resolve()
        return path

    def _handle_signal(self, signum: int, _frame) -> None:
        signal_name = signal.Signals(signum).name
        print(f"[collect_leader_dataset] Received {signal_name}; stopping.", flush=True)
        self._stop_requested = True

    def _ensure_not_stopped(self) -> None:
        if self._stop_requested:
            raise CollectorStopped("interrupted by signal")

    def _print_command(self, label: str, command: list[str]) -> None:
        print(f"[collect_leader_dataset] {label}: {_shell_join(command)}", flush=True)

    def _run(
        self,
        command: list[str],
        *,
        label: str,
        allow_failure: bool = False,
    ) -> subprocess.CompletedProcess[str]:
        self._print_command(label, command)
        if self.dry_run:
            return subprocess.CompletedProcess(command, 0, stdout="", stderr="")

        completed = subprocess.run(
            command,
            cwd=self.ws_root,
            text=True,
            capture_output=False,
            check=False,
        )
        if completed.returncode != 0 and not allow_failure:
            raise RuntimeError(f"{label} failed with exit code {completed.returncode}")
        return completed

    def _capture_output(self, command: list[str], timeout_s: float = 5.0) -> str:
        if self.dry_run:
            return ""
        try:
            completed = subprocess.run(
                command,
                cwd=self.ws_root,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                timeout=timeout_s,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return ""
        if completed.returncode != 0:
            return ""
        return completed.stdout

    def _spawn(self, command: list[str], *, label: str) -> Optional[subprocess.Popen[str]]:
        self._print_command(label, command)
        if self.dry_run:
            return None
        return subprocess.Popen(
            command,
            cwd=self.ws_root,
            text=True,
            stdout=None,
            stderr=None,
            start_new_session=True,
        )

    def _stop_process(self, process: Optional[subprocess.Popen[str]], label: str) -> None:
        if process is None or process.poll() is not None:
            return

        print(f"[collect_leader_dataset] Stopping {label}", flush=True)
        for sig, wait_s in ((signal.SIGINT, 5.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 0.0)):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                return
            if wait_s <= 0.0:
                break
            deadline = time.monotonic() + wait_s
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    return
                time.sleep(0.2)
        process.wait(timeout=1.0)

    def _node_names(self) -> set[str]:
        return set(self._capture_output(["ros2", "node", "list", "--no-daemon"]).splitlines())

    def _route_driver_is_running(self) -> bool:
        nodes = self._node_names()
        return any(node.endswith("/ugv_nav2_driver") or node == "/ugv_nav2_driver" for node in nodes)

    def _read_latest_nav2_tuning_state(self) -> dict[str, str]:
        if not self.tmux_state_dir.is_dir():
            return {}
        candidates = sorted(
            (path for path in self.tmux_state_dir.glob("*.env") if path.is_file()),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        for path in candidates:
            state: dict[str, str] = {}
            for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
                if "=" not in line or line.startswith("TUNING_LIDAR_ARGS="):
                    continue
                key, raw_value = line.split("=", 1)
                try:
                    values = shlex.split(raw_value)
                except ValueError:
                    values = [raw_value]
                state[key] = values[0] if values else ""
            if state.get("WORLD", self.world) == self.world:
                return state
        return {}

    def _route_from_value(self, value: str) -> str:
        if not value:
            return ""
        value_path = Path(value)
        stem = value_path.stem
        for candidate in (value, stem):
            for scenario in self.manifest.scenarios:
                if candidate in (scenario.name, scenario.nav2_goals, scenario.waypoint):
                    return scenario.name
        if stem.startswith("baylands_waypoints_"):
            return stem.removeprefix("baylands_waypoints_")
        return ""

    def _detect_route_name(self) -> str:
        state = self._read_latest_nav2_tuning_state()
        for key in ("NAV2_GOALS_SOURCE", "NAV2_GOALS", "WAYPOINT"):
            route_name = self._route_from_value(state.get(key, ""))
            if route_name:
                return route_name
        if len(self.selected_scenarios) == 1:
            return self.selected_scenarios[0].name
        return "unknown_route"

    def _route_is_selected(self, route_name: str) -> bool:
        if not self.scenario_filter_names:
            return True
        return any(scenario.name == route_name for scenario in self.selected_scenarios)

    def _next_run_dir(self, route_name: str) -> Path:
        safe_route = _safe_name(route_name)
        route_dir = self.output_root if self.output_root.name == safe_route else self.output_root / safe_route
        index = 1
        while True:
            run_dir = route_dir / f"v{index}"
            if not run_dir.exists():
                return run_dir
            index += 1

    def _write_summary(self) -> None:
        if self.dry_run or self._summary_path is None:
            return
        self._summary["updated_at"] = _utc_now_iso()
        with self._summary_path.open("w", encoding="utf-8") as handle:
            json.dump(self._summary, handle, indent=2, sort_keys=True)

    def _prepare_summary(self) -> None:
        if self.dry_run:
            return
        self.output_root.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._summary_path = self.output_root / f"collection_summary_{timestamp}.json"
        self._write_summary()

    def _wait_for_external_run(self) -> Optional[str]:
        printed_idle = False
        while True:
            self._ensure_not_stopped()

            if self._route_driver_is_running():
                route_name = self._detect_route_name()
                if not self._route_is_selected(route_name):
                    print(
                        f"[collect_leader_dataset] Active route '{route_name}' is not selected; waiting.",
                        flush=True,
                    )
                    time.sleep(self.poll_s)
                    continue
                return route_name

            if not printed_idle:
                selected = ", ".join(scenario.name for scenario in self.selected_scenarios)
                print(
                    "[collect_leader_dataset] Waiting for external Nav2 route driver "
                    f"(selected routes: {selected or 'all'})",
                    flush=True,
                )
                printed_idle = True
            time.sleep(self.poll_s)

    def _start_capture(self, run_dir: Path) -> Optional[subprocess.Popen[str]]:
        capture = self.manifest.capture
        return self._spawn(
            [
                "./run.sh",
                "capture_dataset",
                self.world,
                f"out:={run_dir}",
                f"uav_name:={capture.uav_name}",
                f"hz:={capture.hz}",
                f"save_overlay:={'true' if capture.save_overlay else 'false'}",
                f"target_pose_topic:={capture.target_pose_topic}",
                "save_metadata:=true",
                "save_negative_examples:=true",
            ],
            label="start capture_dataset",
        )

    def _wait_for_manual_stop(self, capture_process: Optional[subprocess.Popen[str]]) -> str:
        print("[collect_leader_dataset] Capture is running. Press Ctrl-C to stop.", flush=True)
        while True:
            self._ensure_not_stopped()
            if capture_process is not None and capture_process.poll() is not None:
                raise RuntimeError(f"capture_dataset exited early with code {capture_process.returncode}")
            time.sleep(self.poll_s)

    def _run_capture_cycle(self, route_name: str) -> str:
        run_dir = self._next_run_dir(route_name)
        run_summary: dict[str, Any] = {
            "route": route_name,
            "run_dir": str(run_dir),
            "started_at": _utc_now_iso(),
            "status": "running",
        }
        self._summary["runs"].append(run_summary)
        self._write_summary()

        if self.manifest.startup.warmup_s > 0.0:
            print(
                f"[collect_leader_dataset] Route '{route_name}' ready; warmup {self.manifest.startup.warmup_s:.1f}s",
                flush=True,
            )
            if not self.dry_run:
                time.sleep(self.manifest.startup.warmup_s)

        self._active_capture = self._start_capture(run_dir)
        if self._active_capture is not None:
            time.sleep(1.0)
            if self._active_capture.poll() is not None:
                raise RuntimeError(f"capture_dataset exited early with code {self._active_capture.returncode}")

        finish_reason = "failed"
        try:
            finish_reason = self._wait_for_manual_stop(self._active_capture)
            run_summary["status"] = "completed"
            return finish_reason
        except CollectorStopped:
            finish_reason = "manual_stop"
            run_summary["status"] = "completed"
            return finish_reason
        finally:
            self._stop_process(self._active_capture, "capture_dataset")
            self._active_capture = None
            run_summary["finished_at"] = _utc_now_iso()
            run_summary["finish_reason"] = finish_reason
            self._write_summary()
            print(
                "[collect_leader_dataset] Capture stopped. Generate Ultralytics OBB labels with:\n"
                f"  ./run.sh dataset_make_obb {run_dir} --overwrite --overlay",
                flush=True,
            )

    def run(self) -> int:
        if self.world != "baylands":
            raise ValueError("collect_leader_dataset currently supports baylands only")

        self._prepare_summary()
        print(
            "[collect_leader_dataset] External collector mode: "
            "start nav2_tuning separately; this process only starts capture. "
            "capture_dataset waits internally for camera and pose topics.",
            flush=True,
        )
        print(
            f"[collect_leader_dataset] output_root={self.output_root} "
            f"hz={self.manifest.capture.hz:g} uav={self.manifest.capture.uav_name}",
            flush=True,
        )

        if self.dry_run:
            for scenario in self.selected_scenarios:
                print(
                    f"[collect_leader_dataset] dry-run route {scenario.name}: "
                    f"{self._next_run_dir(scenario.name)}",
                    flush=True,
                )
            self._summary["status"] = "dry_run"
            self._summary["finished_at"] = _utc_now_iso()
            self._write_summary()
            return 0

        try:
            route_name = self._wait_for_external_run()
            if route_name is None:
                self._summary["status"] = "completed"
                self._summary["finished_at"] = _utc_now_iso()
                self._write_summary()
                return 0

            print(f"[collect_leader_dataset] Capturing route '{route_name}'", flush=True)
            finish_reason = self._run_capture_cycle(route_name)
            print(f"[collect_leader_dataset] Finished capture ({finish_reason}).", flush=True)
            self._summary["status"] = "completed"
            self._summary["finished_at"] = _utc_now_iso()
            self._write_summary()
            return 0
        except CollectorStopped:
            self._summary["status"] = "completed"
            self._summary["finished_at"] = _utc_now_iso()
            self._write_summary()
            return 0
        except Exception as exc:
            self._summary["status"] = "failed"
            self._summary["finished_at"] = _utc_now_iso()
            self._summary["error"] = str(exc)
            self._write_summary()
            raise
        finally:
            self._stop_process(self._active_capture, "capture_dataset")
            self._active_capture = None


def main(argv: Optional[list[str]] = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)
    collector = LeaderDatasetCollector(argv)
    rc = 0
    try:
        rc = collector.run()
    except Exception as exc:
        print(f"[collect_leader_dataset] ERROR: {exc}", file=sys.stderr, flush=True)
        rc = 1
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
